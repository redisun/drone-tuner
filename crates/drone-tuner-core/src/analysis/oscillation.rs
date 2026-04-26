//! Oscillation detection and classification.
//!
//! Consumes [`super::FrequencyAnalysisResult`] from the FFT stage and emits
//! [`DetectedOscillation`]s tagged with type, severity, and confidence.
//! Used by the analysis engine and by the filter optimiser.

use super::{FrequencyAnalysisResult, SpectralPeak};
use crate::domain::{Axis, HardwareConfiguration};
use crate::error::Result;

/// Oscillation detection component
#[derive(Debug)]
pub(super) struct OscillationDetector {
    pub(super) config: OscillationDetectorConfig,
    patterns: Vec<OscillationPattern>,
}

/// Configuration for oscillation detection
#[derive(Debug, Clone)]
pub struct OscillationDetectorConfig {
    /// Minimum amplitude threshold
    pub min_amplitude: f32,
    /// Q-factor threshold for resonances
    pub resonance_q_threshold: f32,
    /// Frequency bands for different oscillation types
    pub frequency_bands: FrequencyBands,
    /// Cross-axis correlation thresholds
    pub correlation_thresholds: CorrelationThresholds,
    /// Severity assessment parameters
    pub severity_params: SeverityParameters,
}

/// Frequency bands for categorizing oscillations
#[derive(Debug, Clone)]
pub struct FrequencyBands {
    /// P-term oscillations typically occur in this range (extended for large builds)
    pub p_term_band: (f32, f32),
    /// D-term oscillations typically occur in this range (extended upper range)
    pub d_term_band: (f32, f32),
    /// Mechanical resonances typically occur in this range (reduced overlap, extended range)
    pub mechanical_band: (f32, f32),
    /// Motor/prop noise typically occurs in this range (extended for larger motors)
    pub motor_noise_band: (f32, f32),
}

/// Cross-axis correlation thresholds for classification
#[derive(Debug, Clone)]
pub struct CorrelationThresholds {
    /// High correlation threshold for P-term oscillations
    pub p_term_correlation: f32,
    /// Low correlation threshold for D-term oscillations
    pub d_term_correlation: f32,
    /// Medium correlation threshold for mechanical resonances
    pub mechanical_correlation: f32,
}

/// Severity assessment parameters
#[derive(Debug, Clone)]
pub struct SeverityParameters {
    /// Critical severity amplitude threshold
    pub critical_amplitude: f32,
    /// High severity amplitude threshold
    pub high_amplitude: f32,
    /// Medium severity amplitude threshold
    pub medium_amplitude: f32,
    /// Flight unsafe Q-factor threshold
    pub critical_q_factor: f32,
    /// Performance degraded Q-factor threshold
    pub high_q_factor: f32,
}

/// Known oscillation patterns for detection
#[derive(Debug, Clone)]
pub struct OscillationPattern {
    /// Pattern name/description
    pub name: String,
    /// Expected frequency range
    pub frequency_range: (f32, f32),
    /// Expected Q-factor range
    pub q_factor_range: (f32, f32),
    /// Which axes are typically affected
    pub typical_axes: Vec<Axis>,
    /// Pattern matching function
    pub matcher: PatternMatcher,
}

/// Pattern matching strategies
#[derive(Debug, Clone)]
pub enum PatternMatcher {
    /// Simple frequency and amplitude thresholds
    Simple {
        /// Minimum amplitude threshold for pattern matching
        min_amplitude: f32,
        /// Maximum amplitude threshold for pattern matching
        max_amplitude: f32,
    },
    /// Cross-correlation based matching
    Correlation {
        /// Template pattern for correlation matching
        template: Vec<f32>,
        /// Correlation threshold for pattern detection
        threshold: f32,
    },
    /// Machine learning based classification
    ML {
        /// Identifier for the ML model to use
        model_id: String,
    },
}

/// Internal: a single detected oscillation, passed between analysis stages.
#[derive(Debug, Clone)]
pub(crate) struct DetectedOscillation {
    pub(crate) frequency: f32,
    pub(crate) amplitude: f32,
    pub(crate) q_factor: f32,
    pub(crate) oscillation_type: OscillationType,
    pub(crate) affected_axes: Vec<Axis>,
    pub(crate) confidence: f32,
    pub(crate) description: String,
    /// Cross-axis correlation coefficient
    pub(crate) cross_axis_correlation: f32,
    /// Severity based on flight impact
    pub(crate) severity: OscillationSeverity,
}

/// Oscillation severity levels based on flight impact
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum OscillationSeverity {
    /// Minor issues, barely noticeable
    Low,
    /// Noticeable but manageable
    Medium,
    /// Performance significantly degraded
    High,
    /// Flight unsafe conditions
    Critical,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum OscillationType {
    PTermOscillation,
    DTermOscillation,
    MechanicalResonance,
    MotorNoise,
}

impl OscillationDetector {
    pub(super) fn new() -> Self {
        let mut detector = Self {
            config: OscillationDetectorConfig::default(),
            patterns: Vec::new(),
        };

        detector.add_default_patterns();
        detector
    }

    /// Add commonly known oscillation patterns
    fn add_default_patterns(&mut self) {
        // P-term oscillations (typically 10-50 Hz)
        self.patterns.push(OscillationPattern {
            name: "Low-frequency P-term oscillations".to_string(),
            frequency_range: (10.0, 50.0),
            q_factor_range: (1.0, 5.0),
            typical_axes: vec![Axis::Roll, Axis::Pitch],
            matcher: PatternMatcher::Simple {
                min_amplitude: 2.0,
                max_amplitude: 20.0,
            },
        });

        // D-term oscillations (typically 80-200 Hz)
        self.patterns.push(OscillationPattern {
            name: "High-frequency D-term noise".to_string(),
            frequency_range: (80.0, 200.0),
            q_factor_range: (0.5, 3.0),
            typical_axes: vec![Axis::Roll, Axis::Pitch, Axis::Yaw],
            matcher: PatternMatcher::Simple {
                min_amplitude: 1.0,
                max_amplitude: 10.0,
            },
        });

        // Mechanical resonance (frame resonance typically 100-300 Hz)
        self.patterns.push(OscillationPattern {
            name: "Frame mechanical resonance".to_string(),
            frequency_range: (100.0, 300.0),
            q_factor_range: (5.0, 20.0),
            typical_axes: vec![Axis::Roll, Axis::Pitch],
            matcher: PatternMatcher::Simple {
                min_amplitude: 3.0,
                max_amplitude: 30.0,
            },
        });
    }

    pub(super) fn detect(
        &self,
        freq_analysis: &FrequencyAnalysisResult,
        hardware: &HardwareConfiguration,
    ) -> Result<Vec<DetectedOscillation>> {
        let mut oscillations = Vec::new();

        for peak in &freq_analysis.peaks {
            let cross_axis_correlation =
                self.get_cross_axis_correlation_for_peak(peak, freq_analysis);

            let oscillation_type = self.classify_oscillation_type_enhanced(
                peak.frequency,
                peak.q_factor,
                cross_axis_correlation,
                hardware,
            );

            let severity = self.assess_oscillation_severity(
                peak.magnitude,
                peak.q_factor,
                &oscillation_type,
                hardware,
            );

            let confidence = self.calculate_detection_confidence_enhanced(
                peak,
                &oscillation_type,
                cross_axis_correlation,
                freq_analysis,
            );

            oscillations.push(DetectedOscillation {
                frequency: peak.frequency,
                amplitude: peak.magnitude,
                q_factor: peak.q_factor,
                oscillation_type: oscillation_type.clone(),
                affected_axes: vec![peak.axis.clone()],
                confidence,
                description: self.generate_description_enhanced(
                    &oscillation_type,
                    peak.frequency,
                    &severity,
                ),
                cross_axis_correlation,
                severity,
            });
        }

        Ok(oscillations)
    }

    /// Classify an oscillation by frequency and Q-factor.
    ///
    /// Used by the test suite to validate frequency-band boundaries; production
    /// code paths use [`classify_oscillation_type_enhanced`] which also factors
    /// in cross-axis correlation.
    #[cfg(test)]
    pub(super) fn classify_oscillation_type(
        &self,
        frequency: f32,
        q_factor: f32,
    ) -> OscillationType {
        if frequency >= self.config.frequency_bands.mechanical_band.0
            && frequency <= self.config.frequency_bands.mechanical_band.1
            && q_factor > self.config.resonance_q_threshold
        {
            return OscillationType::MechanicalResonance;
        }

        match frequency {
            f if f >= self.config.frequency_bands.p_term_band.0
                && f <= self.config.frequency_bands.p_term_band.1 =>
            {
                OscillationType::PTermOscillation
            }
            f if f >= self.config.frequency_bands.d_term_band.0
                && f <= self.config.frequency_bands.d_term_band.1 =>
            {
                OscillationType::DTermOscillation
            }
            f if f >= self.config.frequency_bands.motor_noise_band.0
                && f <= self.config.frequency_bands.motor_noise_band.1 =>
            {
                OscillationType::MotorNoise
            }
            _ => OscillationType::MotorNoise,
        }
    }

    /// Enhanced oscillation classification with cross-axis correlation analysis
    fn classify_oscillation_type_enhanced(
        &self,
        frequency: f32,
        q_factor: f32,
        cross_axis_correlation: f32,
        _hardware: &HardwareConfiguration,
    ) -> OscillationType {
        // Check for mechanical resonance first (high Q-factor is key indicator)
        if frequency >= self.config.frequency_bands.mechanical_band.0
            && frequency <= self.config.frequency_bands.mechanical_band.1
            && q_factor > self.config.resonance_q_threshold
        {
            return OscillationType::MechanicalResonance;
        }

        // Enhanced classification using cross-axis correlation
        match frequency {
            f if f >= self.config.frequency_bands.p_term_band.0
                && f <= self.config.frequency_bands.p_term_band.1 =>
            {
                if cross_axis_correlation > self.config.correlation_thresholds.p_term_correlation
                    && q_factor < 5.0
                {
                    OscillationType::PTermOscillation
                } else if q_factor > self.config.resonance_q_threshold {
                    OscillationType::MechanicalResonance
                } else {
                    OscillationType::PTermOscillation
                }
            }
            f if f >= self.config.frequency_bands.d_term_band.0
                && f <= self.config.frequency_bands.d_term_band.1 =>
            {
                if cross_axis_correlation < self.config.correlation_thresholds.d_term_correlation
                    && q_factor < 3.0
                {
                    OscillationType::DTermOscillation
                } else if q_factor > self.config.resonance_q_threshold {
                    OscillationType::MechanicalResonance
                } else {
                    OscillationType::DTermOscillation
                }
            }
            f if f >= self.config.frequency_bands.motor_noise_band.0
                && f <= self.config.frequency_bands.motor_noise_band.1 =>
            {
                OscillationType::MotorNoise
            }
            _ => OscillationType::MotorNoise,
        }
    }

    /// Get cross-axis correlation for a specific peak
    fn get_cross_axis_correlation_for_peak(
        &self,
        peak: &SpectralPeak,
        freq_analysis: &FrequencyAnalysisResult,
    ) -> f32 {
        match peak.axis {
            Axis::Roll => freq_analysis
                .cross_correlation
                .get(&(Axis::Roll, Axis::Pitch))
                .or_else(|| {
                    freq_analysis
                        .cross_correlation
                        .get(&(Axis::Pitch, Axis::Roll))
                })
                .and_then(|v| v.first())
                .copied()
                .unwrap_or(0.0),
            Axis::Pitch => freq_analysis
                .cross_correlation
                .get(&(Axis::Roll, Axis::Pitch))
                .or_else(|| {
                    freq_analysis
                        .cross_correlation
                        .get(&(Axis::Pitch, Axis::Roll))
                })
                .and_then(|v| v.first())
                .copied()
                .unwrap_or(0.0),
            Axis::Yaw => freq_analysis
                .cross_correlation
                .get(&(Axis::Roll, Axis::Yaw))
                .or_else(|| {
                    freq_analysis
                        .cross_correlation
                        .get(&(Axis::Yaw, Axis::Roll))
                })
                .and_then(|v| v.first())
                .copied()
                .unwrap_or(0.0),
        }
    }

    /// Assess oscillation severity based on flight impact
    fn assess_oscillation_severity(
        &self,
        amplitude: f32,
        q_factor: f32,
        oscillation_type: &OscillationType,
        _hardware: &HardwareConfiguration,
    ) -> OscillationSeverity {
        match oscillation_type {
            OscillationType::MechanicalResonance => {
                if q_factor > self.config.severity_params.critical_q_factor
                    || amplitude > self.config.severity_params.critical_amplitude
                {
                    OscillationSeverity::Critical
                } else if q_factor > self.config.severity_params.high_q_factor
                    || amplitude > self.config.severity_params.high_amplitude
                {
                    OscillationSeverity::High
                } else if amplitude > self.config.severity_params.medium_amplitude {
                    OscillationSeverity::Medium
                } else {
                    OscillationSeverity::Low
                }
            }
            OscillationType::PTermOscillation => {
                if amplitude > self.config.severity_params.critical_amplitude {
                    OscillationSeverity::Critical
                } else if amplitude > self.config.severity_params.high_amplitude {
                    OscillationSeverity::High
                } else if amplitude > self.config.severity_params.medium_amplitude {
                    OscillationSeverity::Medium
                } else {
                    OscillationSeverity::Low
                }
            }
            OscillationType::DTermOscillation => {
                if amplitude > self.config.severity_params.high_amplitude {
                    OscillationSeverity::High
                } else if amplitude > self.config.severity_params.medium_amplitude {
                    OscillationSeverity::Medium
                } else {
                    OscillationSeverity::Low
                }
            }
            OscillationType::MotorNoise => {
                if amplitude > self.config.severity_params.high_amplitude {
                    OscillationSeverity::Medium
                } else {
                    OscillationSeverity::Low
                }
            }
        }
    }

    /// Enhanced confidence calculation with signal quality and cross-axis validation
    fn calculate_detection_confidence_enhanced(
        &self,
        peak: &SpectralPeak,
        osc_type: &OscillationType,
        cross_axis_correlation: f32,
        freq_analysis: &FrequencyAnalysisResult,
    ) -> f32 {
        let mut confidence: f32 = 0.6;

        // Signal quality boost
        if let Some(&snr) = freq_analysis.signal_quality.snr.get(&peak.axis) {
            if snr > 20.0 {
                confidence += 0.2;
            } else if snr > 10.0 {
                confidence += 0.1;
            } else if snr < 5.0 {
                confidence -= 0.1;
            }
        }

        // Peak strength boost
        if peak.magnitude > 5.0 {
            confidence += 0.25;
        } else if peak.magnitude > 2.0 {
            confidence += 0.15;
        } else if peak.magnitude > 1.0 {
            confidence += 0.1;
        }

        // Q-factor consistency boost
        match osc_type {
            OscillationType::MechanicalResonance => {
                if peak.q_factor > 8.0 {
                    confidence += 0.15;
                } else if peak.q_factor > 5.0 {
                    confidence += 0.1;
                }
            }
            OscillationType::PTermOscillation => {
                if peak.q_factor < 5.0 {
                    confidence += 0.1;
                }
            }
            OscillationType::DTermOscillation => {
                if peak.q_factor < 3.0 {
                    confidence += 0.1;
                }
            }
            _ => {}
        }

        // Cross-axis correlation validation
        match osc_type {
            OscillationType::PTermOscillation => {
                if cross_axis_correlation > 0.7 {
                    confidence += 0.2;
                } else if cross_axis_correlation < 0.3 {
                    confidence -= 0.15;
                }
            }
            OscillationType::DTermOscillation => {
                if cross_axis_correlation < 0.5 {
                    confidence += 0.15;
                } else if cross_axis_correlation > 0.8 {
                    confidence -= 0.1;
                }
            }
            _ => {}
        }

        confidence.clamp(0.0, 1.0)
    }

    /// Enhanced description generation with severity context
    fn generate_description_enhanced(
        &self,
        osc_type: &OscillationType,
        frequency: f32,
        severity: &OscillationSeverity,
    ) -> String {
        let severity_prefix = match severity {
            OscillationSeverity::Critical => "CRITICAL: ",
            OscillationSeverity::High => "HIGH: ",
            OscillationSeverity::Medium => "MEDIUM: ",
            OscillationSeverity::Low => "LOW: ",
        };

        let base_description = match osc_type {
            OscillationType::PTermOscillation => match severity {
                OscillationSeverity::Critical => format!(
                    "Severe P-term oscillation at {:.1} Hz causing flight instability. Immediately reduce P gain by 20-30%.",
                    frequency
                ),
                OscillationSeverity::High => format!(
                    "Strong P-term oscillation at {:.1} Hz affecting performance. Reduce P gain by 10-20%.",
                    frequency
                ),
                _ => format!(
                    "P-term oscillation at {:.1} Hz. Consider reducing P gain by 5-10%.",
                    frequency
                ),
            },
            OscillationType::DTermOscillation => match severity {
                OscillationSeverity::High | OscillationSeverity::Critical => format!(
                    "High-frequency D-term oscillation at {:.1} Hz. Reduce D gain or lower D-term filter cutoff significantly.",
                    frequency
                ),
                _ => format!(
                    "D-term oscillation at {:.1} Hz. Consider reducing D gain or lowering D-term filter cutoff.",
                    frequency
                ),
            },
            OscillationType::MechanicalResonance => match severity {
                OscillationSeverity::Critical => format!(
                    "DANGEROUS mechanical resonance at {:.1} Hz. Check frame integrity and add notch filter immediately.",
                    frequency
                ),
                OscillationSeverity::High => format!(
                    "Sharp mechanical resonance at {:.1} Hz affecting flight quality. Add notch filter or check hardware.",
                    frequency
                ),
                _ => format!(
                    "Mechanical resonance at {:.1} Hz. Consider adding notch filter or checking for loose hardware.",
                    frequency
                ),
            },
            OscillationType::MotorNoise => format!(
                "Motor/propeller noise at {:.1} Hz. Check motor/prop balance and mounting.",
                frequency
            ),
        };

        format!("{}{}", severity_prefix, base_description)
    }
}

impl Default for OscillationDetectorConfig {
    fn default() -> Self {
        Self {
            min_amplitude: 0.1,
            resonance_q_threshold: 5.0, // Lowered from 10.0 based on FPV physics feedback
            frequency_bands: FrequencyBands {
                p_term_band: (3.0, 50.0),
                d_term_band: (50.0, 300.0),
                mechanical_band: (200.0, 800.0),
                motor_noise_band: (200.0, 1200.0),
            },
            correlation_thresholds: CorrelationThresholds {
                p_term_correlation: 0.7,
                d_term_correlation: 0.5,
                mechanical_correlation: 0.6,
            },
            severity_params: SeverityParameters {
                critical_amplitude: 20.0,
                high_amplitude: 10.0,
                medium_amplitude: 5.0,
                critical_q_factor: 15.0,
                high_q_factor: 8.0,
            },
        }
    }
}
