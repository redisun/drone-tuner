//! Analysis engine for frequency domain analysis and oscillation detection.

mod filter_optimizer;
mod oscillation;
mod pid;

pub use filter_optimizer::FilterOptimizerConfig;
pub use oscillation::{
    CorrelationThresholds, FrequencyBands, OscillationDetectorConfig, OscillationPattern,
    PatternMatcher, SeverityParameters,
};
pub use pid::{PidAnalyzerConfig, StepResponse};

pub(crate) use oscillation::{DetectedOscillation, OscillationSeverity, OscillationType};

use crate::domain::*;
use crate::error::{DronetunerError, Result};
use filter_optimizer::FilterOptimizer;
use num_complex::Complex;
use oscillation::OscillationDetector;
use pid::PidAnalyzer;
use rustfft::FftPlanner;
use std::collections::HashMap;

/// Main analysis engine that orchestrates all analysis stages
pub struct AnalysisEngine {
    /// FFT planner for frequency analysis
    fft_planner: FftPlanner<f32>,
    /// Oscillation detection component
    oscillation_detector: OscillationDetector,
    /// Filter optimization component
    filter_optimizer: FilterOptimizer,
    /// PID analysis component
    pid_analyzer: PidAnalyzer,
    /// Configuration for analysis parameters
    config: AnalysisConfig,
    /// Memory pool for FFT buffers to reduce allocations
    fft_buffer_pool: Vec<Vec<Complex<f32>>>,
}

/// Configuration for analysis engine
#[derive(Debug, Clone)]
pub struct AnalysisConfig {
    /// FFT window size (should be power of 2)
    pub fft_window_size: usize,
    /// Overlap between FFT windows (0.0-1.0)
    pub window_overlap: f32,
    /// Window function to apply before FFT
    pub window_function: WindowFunction,
    /// Minimum frequency to analyze (Hz)
    pub min_frequency: f32,
    /// Maximum frequency to analyze (Hz)  
    pub max_frequency: f32,
    /// Minimum oscillation amplitude to consider significant
    pub oscillation_threshold: f32,
}

/// Window functions for FFT preprocessing
#[derive(Debug, Clone)]
pub enum WindowFunction {
    /// Rectangular window (no windowing)
    Rectangular,
    /// Hann window (cosine-based, good general purpose)
    Hann,
    /// Hamming window (modified cosine)
    Hamming,
    /// Blackman window (excellent frequency resolution)
    Blackman,
    /// Kaiser window with beta parameter
    Kaiser(f32),
}

/// Results from frequency domain analysis
#[derive(Debug, Clone)]
pub struct FrequencyAnalysisResult {
    /// Frequency bins (Hz)
    pub frequencies: Vec<f32>,
    /// Power spectral density for each axis
    pub psd: HashMap<Axis, Vec<f32>>,
    /// Cross-spectral density between axes
    pub cross_psd: HashMap<(Axis, Axis), Vec<Complex<f32>>>,
    /// Coherence between axes
    pub coherence: HashMap<(Axis, Axis), Vec<f32>>,
    /// Cross-axis correlation coefficients
    pub cross_correlation: HashMap<(Axis, Axis), Vec<f32>>,
    /// Identified spectral peaks
    pub peaks: Vec<SpectralPeak>,
    /// Estimated noise floor for each axis
    pub noise_floor: HashMap<Axis, f32>,
    /// Signal quality metrics
    pub signal_quality: SignalQualityMetrics,
}

/// Signal quality assessment metrics
#[derive(Debug, Clone)]
pub struct SignalQualityMetrics {
    /// Signal-to-noise ratio for each axis
    pub snr: HashMap<Axis, f32>,
    /// Data completeness (0.0-1.0)
    pub completeness: f32,
    /// Sampling consistency score
    pub sampling_consistency: f32,
    /// Overall quality score (0.0-1.0)
    pub overall_quality: f32,
}

/// A peak in the power spectral density
#[derive(Debug, Clone)]
pub struct SpectralPeak {
    /// Peak frequency (Hz)
    pub frequency: f32,
    /// Peak magnitude
    pub magnitude: f32,
    /// Q-factor (sharpness) of the peak
    pub q_factor: f32,
    /// Which axis this peak appears on
    pub axis: Axis,
    /// Width of peak at half maximum
    pub bandwidth: f32,
}

impl AnalysisEngine {
    /// Create a new analysis engine with default configuration
    pub fn new() -> Self {
        Self::with_config(AnalysisConfig::default())
    }

    /// Create a new analysis engine with custom configuration
    pub fn with_config(config: AnalysisConfig) -> Self {
        Self {
            fft_planner: FftPlanner::new(),
            oscillation_detector: OscillationDetector::new(),
            filter_optimizer: FilterOptimizer::new(),
            pid_analyzer: PidAnalyzer::new(),
            config,
            fft_buffer_pool: Vec::new(),
        }
    }

    /// Get a buffer from the FFT pool or allocate a new one
    fn get_fft_buffer(&mut self, size: usize) -> Vec<Complex<f32>> {
        // Try to find a buffer of the right size
        for i in 0..self.fft_buffer_pool.len() {
            if self.fft_buffer_pool[i].len() == size {
                return self.fft_buffer_pool.swap_remove(i);
            }
        }

        // No suitable buffer found, allocate new one
        vec![Complex::new(0.0, 0.0); size]
    }

    /// Return a buffer to the FFT pool
    fn return_fft_buffer(&mut self, mut buffer: Vec<Complex<f32>>) {
        // Clear the buffer and return to pool (keep capacity)
        buffer.fill(Complex::new(0.0, 0.0));

        // Limit pool size to prevent excessive memory usage
        if self.fft_buffer_pool.len() < 10 {
            self.fft_buffer_pool.push(buffer);
        }
    }

    /// Perform complete analysis of a flight session
    pub fn analyze(&mut self, session: &FlightSession) -> Result<AnalysisReport> {
        tracing::info!(
            "Starting analysis of flight session {}",
            session.metadata.session_id
        );

        // Stage 1: Frequency domain analysis
        let freq_analysis = self.perform_frequency_analysis(&session.telemetry)?;
        tracing::debug!(
            "Frequency analysis completed, found {} peaks",
            freq_analysis.peaks.len()
        );

        // Stage 2: Oscillation detection
        let oscillations = self
            .oscillation_detector
            .detect(&freq_analysis, &session.metadata.hardware)?;
        tracing::debug!("Detected {} oscillations", oscillations.len());

        // Stage 3: Filter optimization
        let filter_recommendations = self
            .filter_optimizer
            .optimize(&oscillations, &session.metadata.hardware)?;
        tracing::debug!(
            "Generated {} filter recommendations",
            filter_recommendations.len()
        );

        // Stage 4: PID analysis
        let pid_outcome = self
            .pid_analyzer
            .analyze(&session.telemetry, &session.metadata.hardware.pid_config)?;
        tracing::debug!(
            "Generated {} PID recommendations from {} step responses",
            pid_outcome.recommendations.len(),
            pid_outcome.step_responses.len()
        );

        // Stage 5: Calculate confidence scores
        let confidence_scores = self.calculate_confidence_scores(&freq_analysis, &oscillations)?;

        // Stage 6: Calculate overall tune quality
        let tune_quality_score =
            self.calculate_tune_quality_score(&oscillations, &freq_analysis)?;

        tracing::info!(
            "Analysis completed with tune quality score: {:.1}",
            tune_quality_score
        );

        Ok(AnalysisReport {
            timestamp: chrono::Utc::now(),
            frequency_analysis: self.convert_frequency_analysis(freq_analysis),
            detected_issues: self.convert_oscillations_to_issues(oscillations),
            filter_recommendations,
            pid_recommendations: pid_outcome.recommendations,
            step_responses: pid_outcome.step_responses,
            confidence_scores,
            tune_quality_score,
        })
    }

    /// Perform frequency domain analysis using Welch's method
    fn perform_frequency_analysis(
        &mut self,
        telemetry: &TelemetryData,
    ) -> Result<FrequencyAnalysisResult> {
        tracing::debug!(
            "Starting frequency analysis with {} gyro samples",
            telemetry.gyro.len()
        );

        let sample_rate = telemetry.sample_rate;
        let window_size = self.config.fft_window_size;
        let overlap_samples = (window_size as f32 * self.config.window_overlap) as usize;

        // Analyze each axis independently
        let mut psd = HashMap::new();
        let mut peaks = Vec::new();
        let mut noise_floor = HashMap::new();
        let mut snr_values = HashMap::new();

        // Process gyro data for each axis
        let mut all_frequencies = Vec::new();
        let mut all_axis_data = HashMap::new();

        for axis in [Axis::Roll, Axis::Pitch, Axis::Yaw] {
            let data = match axis {
                Axis::Roll => &telemetry.gyro.x,
                Axis::Pitch => &telemetry.gyro.y,
                Axis::Yaw => &telemetry.gyro.z,
            };

            all_axis_data.insert(axis.clone(), data.as_slice());

            // Compute power spectral density
            let (frequencies, psd_values) =
                self.welch_psd(data, sample_rate, window_size, overlap_samples)?;

            if all_frequencies.is_empty() {
                all_frequencies = frequencies.clone();
            }

            // Find spectral peaks
            let axis_peaks = self.find_spectral_peaks(&frequencies, &psd_values, axis.clone())?;
            peaks.extend(axis_peaks);

            // Estimate noise floor and SNR
            let noise_level = self.estimate_noise_floor(&psd_values);
            let max_signal = psd_values.iter().fold(0.0f32, |acc, &x| acc.max(x));
            let snr = if noise_level > 0.0 {
                max_signal / noise_level
            } else {
                100.0
            };

            noise_floor.insert(axis.clone(), noise_level);
            snr_values.insert(axis.clone(), snr);
            psd.insert(axis, psd_values);
        }

        // Compute cross-axis correlations
        let cross_correlation =
            self.compute_cross_axis_correlations(&all_axis_data, sample_rate)?;

        // Assess signal quality
        let signal_quality = self.assess_signal_quality(&snr_values, telemetry)?;

        Ok(FrequencyAnalysisResult {
            frequencies: all_frequencies,
            psd,
            cross_psd: HashMap::new(), // Could add cross-spectral analysis
            coherence: HashMap::new(), // Could add coherence analysis
            cross_correlation,
            peaks,
            noise_floor,
            signal_quality,
        })
    }

    /// Compute power spectral density using Welch's method
    fn welch_psd(
        &mut self,
        data: &[f32],
        sample_rate: f32,
        window_size: usize,
        overlap: usize,
    ) -> Result<(Vec<f32>, Vec<f32>)> {
        if data.len() < window_size {
            return Err(DronetunerError::analysis_error(
                "Insufficient data for FFT analysis",
            ));
        }

        let step_size = window_size - overlap;
        let num_windows = (data.len() - overlap) / step_size;

        if num_windows == 0 {
            return Err(DronetunerError::analysis_error(
                "No complete windows available for analysis",
            ));
        }

        // Create FFT
        let fft = self.fft_planner.plan_fft_forward(window_size);

        // Generate frequency bins
        let frequencies: Vec<f32> = (0..window_size / 2)
            .map(|i| i as f32 * sample_rate / window_size as f32)
            .collect();

        // Accumulate PSD across windows
        let mut psd_accumulator = vec![0.0; window_size / 2];
        let mut window_count = 0;

        // Get a buffer from the pool for reuse across windows
        let mut window_data = self.get_fft_buffer(window_size);

        for window_start in (0..data.len() - window_size + 1).step_by(step_size) {
            let window_end = window_start + window_size;
            if window_end > data.len() {
                break;
            }

            // Copy data into the reused buffer
            for (i, &value) in data[window_start..window_end].iter().enumerate() {
                window_data[i] = Complex::new(value, 0.0);
            }

            // Apply window function
            self.apply_window_function(&mut window_data, &self.config.window_function);

            // Perform FFT
            fft.process(&mut window_data);

            // Calculate power and accumulate
            for (i, &fft_val) in window_data.iter().enumerate().take(window_size / 2) {
                let power = fft_val.norm_sqr();
                psd_accumulator[i] += power;
            }

            window_count += 1;
        }

        // Return the buffer to the pool for reuse
        self.return_fft_buffer(window_data);

        // Average across windows and normalize
        let psd: Vec<f32> = psd_accumulator
            .iter()
            .map(|&power| power / (window_count as f32 * sample_rate))
            .collect();

        Ok((frequencies, psd))
    }

    /// Apply window function to data
    fn apply_window_function(&self, data: &mut [Complex<f32>], window_func: &WindowFunction) {
        let n = data.len();

        match window_func {
            WindowFunction::Rectangular => {
                // No windowing needed
            }
            WindowFunction::Hann => {
                for (i, sample) in data.iter_mut().enumerate() {
                    let window_val =
                        0.5 - 0.5 * (2.0 * std::f32::consts::PI * i as f32 / (n - 1) as f32).cos();
                    *sample *= window_val;
                }
            }
            WindowFunction::Hamming => {
                for (i, sample) in data.iter_mut().enumerate() {
                    let window_val = 0.54
                        - 0.46 * (2.0 * std::f32::consts::PI * i as f32 / (n - 1) as f32).cos();
                    *sample *= window_val;
                }
            }
            WindowFunction::Blackman => {
                for (i, sample) in data.iter_mut().enumerate() {
                    let angle = 2.0 * std::f32::consts::PI * i as f32 / (n - 1) as f32;
                    let window_val = 0.42 - 0.5 * angle.cos() + 0.08 * (2.0 * angle).cos();
                    *sample *= window_val;
                }
            }
            WindowFunction::Kaiser(beta) => {
                // Kaiser window implementation
                let alpha = n as f32 / 2.0;
                let i0_beta = self.bessel_i0(*beta);

                for (i, sample) in data.iter_mut().enumerate() {
                    let x = (i as f32 - alpha) / alpha;
                    let window_val = self.bessel_i0(*beta * (1.0 - x * x).sqrt()) / i0_beta;
                    *sample *= window_val;
                }
            }
        }
    }

    /// Modified Bessel function of the first kind (order 0)
    fn bessel_i0(&self, x: f32) -> f32 {
        let mut sum = 1.0;
        let mut term = 1.0;
        let x_half = x / 2.0;

        for k in 1..20 {
            // Sufficient precision for window functions
            term *= (x_half * x_half) / (k as f32 * k as f32);
            sum += term;
        }

        sum
    }

    /// Find peaks in power spectral density
    fn find_spectral_peaks(
        &self,
        frequencies: &[f32],
        psd: &[f32],
        axis: Axis,
    ) -> Result<Vec<SpectralPeak>> {
        let mut peaks = Vec::new();

        if frequencies.len() != psd.len() || frequencies.len() < 3 {
            return Ok(peaks);
        }

        // Find local maxima
        for i in 1..psd.len() - 1 {
            let freq = frequencies[i];

            // Skip frequencies outside our analysis range
            if freq < self.config.min_frequency || freq > self.config.max_frequency {
                continue;
            }

            // Check if this is a local maximum
            if psd[i] > psd[i - 1] && psd[i] > psd[i + 1] {
                let magnitude = psd[i];

                // Only consider peaks above threshold
                if magnitude > self.config.oscillation_threshold {
                    // Estimate Q-factor
                    let q_factor = self.estimate_q_factor(frequencies, psd, i);

                    // Estimate bandwidth
                    let bandwidth = freq / q_factor;

                    peaks.push(SpectralPeak {
                        frequency: freq,
                        magnitude,
                        q_factor,
                        axis: axis.clone(),
                        bandwidth,
                    });
                }
            }
        }

        // Sort peaks by magnitude (highest first)
        peaks.sort_by(|a, b| {
            b.magnitude
                .partial_cmp(&a.magnitude)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        Ok(peaks)
    }

    /// Estimate Q-factor of a spectral peak
    fn estimate_q_factor(&self, frequencies: &[f32], psd: &[f32], peak_index: usize) -> f32 {
        let peak_power = psd[peak_index];
        let half_power = peak_power / 2.0;

        // Find frequencies where power drops to half maximum
        let mut lower_freq = frequencies[peak_index];
        let mut upper_freq = frequencies[peak_index];

        // Search backwards for lower -3dB point
        for i in (0..peak_index).rev() {
            if psd[i] <= half_power {
                lower_freq = frequencies[i];
                break;
            }
        }

        // Search forwards for upper -3dB point
        for i in (peak_index + 1)..psd.len() {
            if psd[i] <= half_power {
                upper_freq = frequencies[i];
                break;
            }
        }

        let bandwidth = upper_freq - lower_freq;
        if bandwidth > 0.0 {
            frequencies[peak_index] / bandwidth
        } else {
            1.0 // Default Q-factor
        }
    }

    /// Estimate noise floor of the spectrum
    fn estimate_noise_floor(&self, psd: &[f32]) -> f32 {
        // Use median as a robust estimate of noise floor
        let mut sorted_psd = psd.to_vec();
        sorted_psd.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

        let median_idx = sorted_psd.len() / 2;
        sorted_psd[median_idx]
    }

    /// Compute cross-axis correlations for oscillation classification
    fn compute_cross_axis_correlations(
        &self,
        axis_data: &HashMap<Axis, &[f32]>,
        sample_rate: f32,
    ) -> Result<HashMap<(Axis, Axis), Vec<f32>>> {
        let mut cross_correlations = HashMap::new();

        let axes = vec![Axis::Roll, Axis::Pitch, Axis::Yaw];

        // Compute correlation for each axis pair
        for i in 0..axes.len() {
            for j in (i + 1)..axes.len() {
                let axis1 = &axes[i];
                let axis2 = &axes[j];

                if let (Some(&data1), Some(&data2)) = (axis_data.get(axis1), axis_data.get(axis2)) {
                    let correlation =
                        self.compute_correlation_coefficient(data1, data2, sample_rate)?;
                    cross_correlations.insert((axis1.clone(), axis2.clone()), correlation);
                }
            }
        }

        Ok(cross_correlations)
    }

    /// Compute correlation coefficient between two signals at different frequency bands
    fn compute_correlation_coefficient(
        &self,
        data1: &[f32],
        data2: &[f32],
        _sample_rate: f32,
    ) -> Result<Vec<f32>> {
        if data1.len() != data2.len() || data1.len() < 100 {
            return Ok(vec![0.0]); // Return zero correlation for invalid data
        }

        // For now, compute a simple time-domain correlation
        // In a full implementation, we would compute frequency-domain correlations
        let correlation = self.pearson_correlation(data1, data2);
        Ok(vec![correlation])
    }

    /// Compute Pearson correlation coefficient between two signals
    fn pearson_correlation(&self, x: &[f32], y: &[f32]) -> f32 {
        if x.len() != y.len() || x.is_empty() {
            return 0.0;
        }

        let n = x.len() as f32;
        let mean_x = x.iter().sum::<f32>() / n;
        let mean_y = y.iter().sum::<f32>() / n;

        let mut numerator = 0.0;
        let mut sum_sq_x = 0.0;
        let mut sum_sq_y = 0.0;

        for (&xi, &yi) in x.iter().zip(y.iter()) {
            let dx = xi - mean_x;
            let dy = yi - mean_y;
            numerator += dx * dy;
            sum_sq_x += dx * dx;
            sum_sq_y += dy * dy;
        }

        let denominator = (sum_sq_x * sum_sq_y).sqrt();
        if denominator > 0.0 {
            (numerator / denominator).max(-1.0).min(1.0)
        } else {
            0.0
        }
    }

    /// Assess overall signal quality
    fn assess_signal_quality(
        &self,
        snr_values: &HashMap<Axis, f32>,
        telemetry: &TelemetryData,
    ) -> Result<SignalQualityMetrics> {
        // Calculate average SNR
        let avg_snr = if !snr_values.is_empty() {
            snr_values.values().sum::<f32>() / snr_values.len() as f32
        } else {
            1.0
        };

        // Assess data completeness
        let expected_samples = (telemetry.sample_rate * 1.0).max(1000.0) as usize; // At least 1 second or 1000 samples
        let actual_samples = telemetry.gyro.len();
        let completeness = if expected_samples > 0 {
            (actual_samples as f32 / expected_samples as f32).min(1.0)
        } else {
            0.0
        };

        // Assess sampling consistency (based on loop time variance)
        let sampling_consistency = if telemetry.loop_time_variance < 0.1 {
            1.0
        } else if telemetry.loop_time_variance < 0.5 {
            0.8
        } else {
            0.5
        };

        // Calculate overall quality score
        let snr_score = if avg_snr > 20.0 {
            1.0
        } else if avg_snr > 10.0 {
            0.8
        } else if avg_snr > 5.0 {
            0.6
        } else {
            0.4
        };

        let overall_quality = (snr_score + completeness + sampling_consistency) / 3.0;

        Ok(SignalQualityMetrics {
            snr: snr_values.clone(),
            completeness,
            sampling_consistency,
            overall_quality,
        })
    }

    /// Convert frequency analysis result to domain format
    fn convert_frequency_analysis(&self, result: FrequencyAnalysisResult) -> FrequencyAnalysis {
        let peaks = result
            .peaks
            .into_iter()
            .map(|peak| FrequencyPeak {
                frequency: peak.frequency,
                amplitude: peak.magnitude,
                q_factor: peak.q_factor,
                axes: vec![peak.axis],
            })
            .collect();

        FrequencyAnalysis {
            frequencies: result.frequencies,
            gyro_x_psd: result.psd.get(&Axis::Roll).cloned().unwrap_or_default(),
            gyro_y_psd: result.psd.get(&Axis::Pitch).cloned().unwrap_or_default(),
            gyro_z_psd: result.psd.get(&Axis::Yaw).cloned().unwrap_or_default(),
            peaks,
            noise_floor: result.noise_floor.values().copied().fold(0.0, f32::max),
        }
    }

    /// Convert detected oscillations to issues with enhanced severity mapping
    fn convert_oscillations_to_issues(
        &self,
        oscillations: Vec<DetectedOscillation>,
    ) -> Vec<DetectedIssue> {
        oscillations
            .into_iter()
            .map(|osc| {
                let (issue_type, severity) = match osc.oscillation_type {
                    OscillationType::PTermOscillation => (
                        IssueType::PTermOscillation {
                            frequency: osc.frequency,
                            amplitude: osc.amplitude,
                        },
                        // Map enhanced severity to domain severity
                        match osc.severity {
                            OscillationSeverity::Critical => Severity::Critical,
                            OscillationSeverity::High => Severity::High,
                            OscillationSeverity::Medium => Severity::Medium,
                            OscillationSeverity::Low => Severity::Low,
                        },
                    ),
                    OscillationType::DTermOscillation => (
                        IssueType::DTermOscillation {
                            frequency: osc.frequency,
                            amplitude: osc.amplitude,
                        },
                        match osc.severity {
                            OscillationSeverity::Critical => Severity::Critical,
                            OscillationSeverity::High => Severity::High,
                            OscillationSeverity::Medium => Severity::Medium,
                            OscillationSeverity::Low => Severity::Low,
                        },
                    ),
                    OscillationType::MechanicalResonance => (
                        IssueType::MechanicalResonance {
                            frequency: osc.frequency,
                            q_factor: osc.q_factor,
                        },
                        match osc.severity {
                            OscillationSeverity::Critical => Severity::Critical,
                            OscillationSeverity::High => Severity::High,
                            OscillationSeverity::Medium => Severity::High, // Bump up mechanical issues
                            OscillationSeverity::Low => Severity::Medium,
                        },
                    ),
                    OscillationType::MotorNoise => (
                        IssueType::Imbalance {
                            motors: vec![1, 2, 3, 4],
                        },
                        match osc.severity {
                            OscillationSeverity::Critical | OscillationSeverity::High => {
                                Severity::Medium
                            }
                            OscillationSeverity::Medium => Severity::Medium,
                            OscillationSeverity::Low => Severity::Low,
                        },
                    ),
                };

                DetectedIssue {
                    issue_type,
                    severity,
                    description: osc.description,
                    affected_axes: osc.affected_axes,
                    confidence: osc.confidence,
                }
            })
            .collect()
    }

    /// Calculate confidence scores for the analysis
    fn calculate_confidence_scores(
        &self,
        freq_analysis: &FrequencyAnalysisResult,
        oscillations: &[DetectedOscillation],
    ) -> Result<ConfidenceScores> {
        // Calculate confidence based on data quality and consistency
        let data_quality = freq_analysis.signal_quality.overall_quality;
        let detection_consistency =
            self.assess_detection_consistency_enhanced(oscillations, freq_analysis);
        let cross_axis_validation = self.assess_cross_axis_validation(oscillations, freq_analysis);

        let overall = (data_quality + detection_consistency + cross_axis_validation) / 3.0;

        Ok(ConfidenceScores {
            overall,
            oscillation_detection: detection_consistency,
            filter_recommendations: (data_quality + cross_axis_validation) / 2.0 * 0.95,
            pid_recommendations: data_quality * 0.85,
            mechanical_issues: (detection_consistency + cross_axis_validation) / 2.0 * 0.8,
        })
    }

    /// Enhanced assessment of detection consistency with cross-axis validation
    fn assess_detection_consistency_enhanced(
        &self,
        oscillations: &[DetectedOscillation],
        freq_analysis: &FrequencyAnalysisResult,
    ) -> f32 {
        if oscillations.is_empty() {
            return 0.85; // Slightly higher neutral score for clean flights
        }

        let mut total_confidence = 0.0;
        let mut validated_detections = 0;

        for oscillation in oscillations {
            let mut detection_confidence = oscillation.confidence;

            // Boost confidence for high-quality signals
            if let Some(&snr) = freq_analysis
                .signal_quality
                .snr
                .get(&oscillation.affected_axes[0])
            {
                if snr > 20.0 {
                    detection_confidence *= 1.1;
                } else if snr < 5.0 {
                    detection_confidence *= 0.8;
                }
            }

            // Boost confidence for cross-axis validated detections
            if oscillation.cross_axis_correlation > 0.7 {
                detection_confidence *= 1.15;
            } else if oscillation.cross_axis_correlation < 0.3 {
                detection_confidence *= 0.9;
            }

            total_confidence += detection_confidence.min(1.0);
            validated_detections += 1;
        }

        if validated_detections > 0 {
            total_confidence / validated_detections as f32
        } else {
            0.8
        }
    }

    /// Assess cross-axis validation quality
    fn assess_cross_axis_validation(
        &self,
        oscillations: &[DetectedOscillation],
        _freq_analysis: &FrequencyAnalysisResult,
    ) -> f32 {
        if oscillations.is_empty() {
            return 0.9; // High score for clean flights
        }

        let mut validation_score = 0.0;
        let mut total_weight = 0.0;

        for oscillation in oscillations {
            let weight = oscillation.amplitude; // Weight by amplitude
            let correlation_score = match oscillation.oscillation_type {
                OscillationType::PTermOscillation => {
                    // P-term should have high cross-axis correlation
                    if oscillation.cross_axis_correlation > 0.7 {
                        1.0
                    } else {
                        0.5
                    }
                }
                OscillationType::DTermOscillation => {
                    // D-term should have low cross-axis correlation
                    if oscillation.cross_axis_correlation < 0.5 {
                        1.0
                    } else {
                        0.6
                    }
                }
                OscillationType::MechanicalResonance => {
                    // Mechanical resonance can vary but should be consistent
                    if oscillation.q_factor > 5.0 {
                        0.9
                    } else {
                        0.7
                    }
                }
                OscillationType::MotorNoise => {
                    // Motor noise detection is less reliable
                    0.7
                }
            };

            validation_score += correlation_score * weight;
            total_weight += weight;
        }

        if total_weight > 0.0 {
            (validation_score / total_weight).min(1.0)
        } else {
            0.8
        }
    }

    /// Calculate overall tune quality score
    fn calculate_tune_quality_score(
        &self,
        oscillations: &[DetectedOscillation],
        freq_analysis: &FrequencyAnalysisResult,
    ) -> Result<f32> {
        let mut score = 100.0;

        // Penalize for detected oscillations
        for oscillation in oscillations {
            let penalty = match oscillation.oscillation_type {
                OscillationType::PTermOscillation => oscillation.amplitude * 5.0,
                OscillationType::DTermOscillation => oscillation.amplitude * 3.0,
                OscillationType::MechanicalResonance => oscillation.amplitude * 10.0,
                OscillationType::MotorNoise => oscillation.amplitude * 2.0,
            };
            score -= penalty;
        }

        // Penalize for poor signal quality
        let avg_noise = freq_analysis.noise_floor.values().sum::<f32>()
            / freq_analysis.noise_floor.len() as f32;
        if avg_noise > 1.0 {
            score -= (avg_noise - 1.0) * 20.0;
        }

        Ok(score.min(100.0).max(0.0))
    }
}

impl Default for AnalysisConfig {
    fn default() -> Self {
        Self {
            fft_window_size: 2048,
            window_overlap: 0.5,
            window_function: WindowFunction::Hann,
            min_frequency: 10.0,
            max_frequency: 1000.0,
            oscillation_threshold: 0.1,
        }
    }
}

impl Default for AnalysisEngine {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests;

#[cfg(test)]
mod debug_tests;

#[cfg(test)]
mod realistic_tests;

#[cfg(test)]
mod debug_d_term;

#[cfg(test)]
mod basic_tests {
    use super::*;

    #[test]
    fn test_analysis_engine_creation() {
        let engine = AnalysisEngine::new();
        assert_eq!(engine.config.fft_window_size, 2048);
    }

    #[test]
    fn test_window_function_application() {
        let engine = AnalysisEngine::new();
        let mut data = vec![
            Complex::new(1.0, 0.0),
            Complex::new(1.0, 0.0),
            Complex::new(1.0, 0.0),
            Complex::new(1.0, 0.0),
        ];

        engine.apply_window_function(&mut data, &WindowFunction::Hann);

        // First and last samples should be attenuated by Hann window
        assert!(data[0].re < 1.0);
        assert!(data[3].re < 1.0);
    }

    #[test]
    fn test_q_factor_estimation() {
        let engine = AnalysisEngine::new();
        let frequencies = vec![10.0, 20.0, 30.0, 40.0, 50.0];
        let psd = vec![1.0, 2.0, 4.0, 2.0, 1.0]; // Peak at 30 Hz

        let q_factor = engine.estimate_q_factor(&frequencies, &psd, 2);
        assert!(q_factor > 0.0);
    }

    #[test]
    fn test_pid_analysis_uses_actual_values() {
        // Create a test flight session with specific PID values
        let test_pid_config = PidConfiguration {
            roll: PidValues {
                p: 76.0,  // Actual FC value from the problem description
                i: 122.0, // Actual FC value
                d: 57.0,  // Actual FC value
                f: None,
            },
            pitch: PidValues {
                p: 75.0,
                i: 120.0,
                d: 55.0,
                f: None,
            },
            yaw: PidValues {
                p: 45.0,
                i: 90.0,
                d: 0.0,
                f: None,
            },
            settings: PidSettings {
                tpa: None,
                profile: 1,
                rates: RateSettings {
                    roll_rate: 0.7,
                    pitch_rate: 0.7,
                    yaw_rate: 0.7,
                    expo: ExpoSettings {
                        roll: 0.0,
                        pitch: 0.0,
                        yaw: 0.0,
                    },
                    super_rate: SuperRateSettings {
                        roll: 0.7,
                        pitch: 0.7,
                        yaw: 0.7,
                    },
                },
            },
        };

        // Create test telemetry with high noise to trigger recommendations
        let telemetry = TelemetryData {
            sample_rate: 4000.0,
            gyro: TimeSeriesVector3 {
                x: vec![0.0; 1000]
                    .into_iter()
                    .enumerate()
                    .map(|(i, _)| {
                        // Create noisy data that should trigger D-term reduction
                        (i as f32 * 0.1).sin() * 20.0 + (i as f32 * 2.0).sin() * 5.0
                    })
                    .collect(),
                y: vec![0.0; 1000]
                    .into_iter()
                    .enumerate()
                    .map(|(i, _)| {
                        // Create oscillatory data that should trigger P-term reduction
                        (i as f32 * 0.05).sin() * 30.0
                    })
                    .collect(),
                z: vec![0.0; 1000],
            },
            accel: TimeSeriesVector3 {
                x: vec![0.0; 1000],
                y: vec![0.0; 1000],
                z: vec![0.0; 1000],
            },
            motor: vec![],
            rc_commands: RcCommandTrace {
                roll: vec![],
                pitch: vec![],
                yaw: vec![],
                throttle: vec![],
            },
            pid_error: PidErrorTrace {
                roll: vec![0.0; 1000]
                    .into_iter()
                    .enumerate()
                    .map(|(i, _)| {
                        // Create persistent error bias to trigger I-term increase
                        3.0 + (i as f32 * 0.01).sin() * 0.5
                    })
                    .collect(),
                pitch: vec![],
                yaw: vec![],
            },
            loop_time_variance: 0.05,
            cpu_load: vec![],
        };

        // Test the PID analyzer directly
        let analyzer = PidAnalyzer::new();
        let recommendations = analyzer
            .analyze(&telemetry, &test_pid_config)
            .unwrap()
            .recommendations;

        // Verify that recommendations exist and use actual PID values
        assert!(
            !recommendations.is_empty(),
            "Should generate recommendations for noisy data"
        );

        // Find a recommendation and verify it uses actual PID values
        let roll_recommendation = recommendations.iter().find(|r| r.axis == Axis::Roll);
        assert!(
            roll_recommendation.is_some(),
            "Should have roll axis recommendation"
        );

        let rec = roll_recommendation.unwrap();
        // Should use actual PID values as current_value, not hardcoded 50.0
        match rec.term {
            PidTerm::P => assert_eq!(rec.current_value, test_pid_config.roll.p),
            PidTerm::I => assert_eq!(rec.current_value, test_pid_config.roll.i),
            PidTerm::D => assert_eq!(rec.current_value, test_pid_config.roll.d),
            PidTerm::F => panic!("F-term should not be used in basic PID analysis"),
        }

        // Verify recommendations are based on actual values, not 50.0
        let expected_reduction = match rec.term {
            PidTerm::P => test_pid_config.roll.p * 0.85, // or 0.9 depending on trigger
            PidTerm::I => test_pid_config.roll.i * 1.2,  // or 1.3 for error bias
            PidTerm::D => test_pid_config.roll.d * 0.8,
            PidTerm::F => panic!("F-term should not be used in basic PID analysis"),
        };

        // The recommended value should be close to expected (within 10%)
        let diff_ratio = (rec.recommended_value - expected_reduction).abs() / expected_reduction;
        assert!(
            diff_ratio < 0.1,
            "Recommended value {:.1} should be close to expected {:.1} (diff ratio: {:.3})",
            rec.recommended_value,
            expected_reduction,
            diff_ratio
        );

        // Most importantly: verify we're NOT using hardcoded 50.0
        assert_ne!(
            rec.current_value, 50.0,
            "Should not use hardcoded 50.0 value"
        );

        println!("PID recommendations now use actual values:");
        for rec in &recommendations {
            println!(
                "  {:?} {}: {:.1} → {:.1}",
                rec.axis,
                match rec.term {
                    PidTerm::P => "P",
                    PidTerm::I => "I",
                    PidTerm::D => "D",
                    PidTerm::F => "F",
                },
                rec.current_value,
                rec.recommended_value
            );
        }
    }
}
