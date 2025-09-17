//! Analysis engine for frequency domain analysis and oscillation detection.

use crate::domain::*;
use crate::error::{DronetunerError, Result};
use num_complex::Complex;
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
    /// Identified spectral peaks
    pub peaks: Vec<SpectralPeak>,
    /// Estimated noise floor for each axis
    pub noise_floor: HashMap<Axis, f32>,
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

/// Oscillation detection component
#[derive(Debug)]
pub struct OscillationDetector {
    /// Configuration parameters
    config: OscillationDetectorConfig,
    /// Known oscillation patterns
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
}

/// Frequency bands for categorizing oscillations
#[derive(Debug, Clone)]
pub struct FrequencyBands {
    /// P-term oscillations typically occur in this range
    pub p_term_band: (f32, f32),
    /// D-term oscillations typically occur in this range
    pub d_term_band: (f32, f32),
    /// Mechanical resonances typically occur in this range
    pub mechanical_band: (f32, f32),
    /// Motor/prop noise typically occurs in this range
    pub motor_noise_band: (f32, f32),
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
        min_amplitude: f32,
        max_amplitude: f32,
    },
    /// Cross-correlation based matching
    Correlation { template: Vec<f32>, threshold: f32 },
    /// Machine learning based classification
    ML { model_id: String },
}

/// Filter optimization component  
#[derive(Debug)]
pub struct FilterOptimizer {
    /// Optimization configuration
    config: FilterOptimizerConfig,
}

/// Configuration for filter optimization
#[derive(Debug, Clone)]
pub struct FilterOptimizerConfig {
    /// Maximum number of notch filters to recommend
    pub max_notch_filters: usize,
    /// Minimum attenuation target (dB)
    pub min_attenuation_db: f32,
    /// Maximum acceptable group delay (ms)
    pub max_group_delay_ms: f32,
    /// Preferred filter types in order of preference
    pub preferred_filter_types: Vec<FilterType>,
}

/// PID analysis component
#[derive(Debug)]
pub struct PidAnalyzer {
    /// Analysis configuration
    config: PidAnalyzerConfig,
}

/// Configuration for PID analysis
#[derive(Debug, Clone)]
pub struct PidAnalyzerConfig {
    /// Acceptable error threshold
    pub error_threshold: f32,
    /// Response time analysis window (seconds)
    pub response_window_s: f32,
    /// Overshoot tolerance (percentage)
    pub overshoot_tolerance: f32,
}

/// Represents a detected step response in the control system
#[derive(Debug, Clone)]
pub struct StepResponse {
    /// Which axis this response occurred on
    pub axis: Axis,
    /// Time when the step input occurred (seconds)
    pub start_time: f32,
    /// Magnitude of the command change
    pub command_magnitude: f32,
    /// Rise time (10% to 90% of final value)
    pub rise_time: f32,
    /// Settling time (time to stay within 2% of final value)
    pub settling_time: f32,
    /// Overshoot as percentage of final value
    pub overshoot_percent: f32,
    /// Dominant oscillation frequency in the response
    pub oscillation_frequency: f32,
    /// Estimated damping ratio
    pub damping_ratio: f32,
    /// Steady-state error as percentage
    pub steady_state_error: f32,
}

/// Step response performance metrics
#[derive(Debug, Clone)]
struct StepMetrics {
    /// Rise time in seconds
    rise_time: f32,
    /// Settling time in seconds
    settling_time: f32,
    /// Overshoot percentage
    overshoot_percent: f32,
    /// Oscillation frequency in Hz
    oscillation_frequency: f32,
    /// Damping ratio (0 = undamped, 1 = critically damped)
    damping_ratio: f32,
    /// Steady-state error as ratio
    steady_state_error: f32,
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
        let pid_recommendations = self.pid_analyzer.analyze(&session.telemetry)?;
        tracing::debug!(
            "Generated {} PID recommendations",
            pid_recommendations.len()
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
            pid_recommendations,
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

        // Process gyro data for each axis
        let mut all_frequencies = Vec::new();

        for axis in [Axis::Roll, Axis::Pitch, Axis::Yaw] {
            let data = match axis {
                Axis::Roll => &telemetry.gyro.x,
                Axis::Pitch => &telemetry.gyro.y,
                Axis::Yaw => &telemetry.gyro.z,
            };

            // Compute power spectral density
            let (frequencies, psd_values) =
                self.welch_psd(data, sample_rate, window_size, overlap_samples)?;

            if all_frequencies.is_empty() {
                all_frequencies = frequencies.clone();
            }

            // Find spectral peaks
            let axis_peaks = self.find_spectral_peaks(&frequencies, &psd_values, axis.clone())?;
            peaks.extend(axis_peaks);

            // Estimate noise floor
            let noise_level = self.estimate_noise_floor(&psd_values);
            noise_floor.insert(axis.clone(), noise_level);

            psd.insert(axis, psd_values);
        }

        Ok(FrequencyAnalysisResult {
            frequencies: all_frequencies,
            psd,
            cross_psd: HashMap::new(), // Could add cross-spectral analysis
            coherence: HashMap::new(), // Could add coherence analysis
            peaks,
            noise_floor,
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

        for window_start in (0..data.len() - window_size + 1).step_by(step_size) {
            let window_end = window_start + window_size;
            if window_end > data.len() {
                break;
            }

            // Extract window
            let mut window_data: Vec<Complex<f32>> = data[window_start..window_end]
                .iter()
                .map(|&x| Complex::new(x, 0.0))
                .collect();

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

    /// Convert detected oscillations to issues
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
                        if osc.amplitude > 10.0 {
                            Severity::High
                        } else {
                            Severity::Medium
                        },
                    ),
                    OscillationType::DTermOscillation => (
                        IssueType::DTermOscillation {
                            frequency: osc.frequency,
                            amplitude: osc.amplitude,
                        },
                        if osc.amplitude > 5.0 {
                            Severity::High
                        } else {
                            Severity::Medium
                        },
                    ),
                    OscillationType::MechanicalResonance => (
                        IssueType::MechanicalResonance {
                            frequency: osc.frequency,
                            q_factor: osc.q_factor,
                        },
                        Severity::High,
                    ),
                    OscillationType::MotorNoise => (
                        IssueType::Imbalance {
                            motors: vec![1, 2, 3, 4],
                        },
                        Severity::Medium,
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
        let data_quality = self.assess_data_quality(freq_analysis)?;
        let detection_consistency = self.assess_detection_consistency(oscillations);

        let overall = (data_quality + detection_consistency) / 2.0;

        Ok(ConfidenceScores {
            overall,
            oscillation_detection: detection_consistency,
            filter_recommendations: data_quality * 0.9, // Slightly lower for recommendations
            pid_recommendations: data_quality * 0.8,    // Even lower for PID recommendations
            mechanical_issues: detection_consistency * 0.7, // Lowest for mechanical issues
        })
    }

    /// Assess the quality of the frequency analysis data
    fn assess_data_quality(&self, freq_analysis: &FrequencyAnalysisResult) -> Result<f32> {
        let mut quality_score: f32 = 1.0;

        // Check for sufficient frequency resolution
        if freq_analysis.frequencies.len() < 100 {
            quality_score *= 0.7;
        }

        // Check signal-to-noise ratio
        let max_signal = freq_analysis
            .psd
            .values()
            .flat_map(|psd| psd.iter())
            .fold(0.0f32, |acc, &x| acc.max(x));

        let avg_noise = freq_analysis.noise_floor.values().sum::<f32>()
            / freq_analysis.noise_floor.len() as f32;

        let snr = if avg_noise > 0.0 {
            max_signal / avg_noise
        } else {
            100.0
        };

        if snr < 10.0 {
            quality_score *= 0.5;
        } else if snr < 50.0 {
            quality_score *= 0.8;
        }

        Ok(quality_score.min(1.0).max(0.0))
    }

    /// Assess consistency of oscillation detection
    fn assess_detection_consistency(&self, oscillations: &[DetectedOscillation]) -> f32 {
        if oscillations.is_empty() {
            return 0.8; // Neutral score for no oscillations
        }

        // Calculate average confidence of detections
        let avg_confidence =
            oscillations.iter().map(|osc| osc.confidence).sum::<f32>() / oscillations.len() as f32;

        avg_confidence
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

// Temporary types for internal analysis (will be moved to separate modules)
#[derive(Debug, Clone)]
struct DetectedOscillation {
    frequency: f32,
    amplitude: f32,
    q_factor: f32,
    oscillation_type: OscillationType,
    affected_axes: Vec<Axis>,
    confidence: f32,
    description: String,
}

#[derive(Debug, Clone)]
enum OscillationType {
    PTermOscillation,
    DTermOscillation,
    MechanicalResonance,
    MotorNoise,
}

impl OscillationDetector {
    fn new() -> Self {
        Self {
            config: OscillationDetectorConfig::default(),
            patterns: Vec::new(),
        }
    }

    fn detect(
        &self,
        freq_analysis: &FrequencyAnalysisResult,
        _hardware: &HardwareConfiguration,
    ) -> Result<Vec<DetectedOscillation>> {
        let mut oscillations = Vec::new();

        for peak in &freq_analysis.peaks {
            let oscillation_type = self.classify_oscillation_type(peak.frequency, peak.q_factor);

            oscillations.push(DetectedOscillation {
                frequency: peak.frequency,
                amplitude: peak.magnitude,
                q_factor: peak.q_factor,
                oscillation_type: oscillation_type.clone(),
                affected_axes: vec![peak.axis.clone()],
                confidence: self.calculate_detection_confidence(peak, &oscillation_type),
                description: self.generate_description(&oscillation_type, peak.frequency),
            });
        }

        Ok(oscillations)
    }

    fn classify_oscillation_type(&self, frequency: f32, q_factor: f32) -> OscillationType {
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
            f if f >= self.config.frequency_bands.mechanical_band.0
                && f <= self.config.frequency_bands.mechanical_band.1
                && q_factor > 10.0 =>
            {
                OscillationType::MechanicalResonance
            }
            _ => OscillationType::MotorNoise,
        }
    }

    fn calculate_detection_confidence(
        &self,
        peak: &SpectralPeak,
        _osc_type: &OscillationType,
    ) -> f32 {
        let mut confidence: f32 = 0.7; // Base confidence

        // Higher confidence for stronger peaks
        if peak.magnitude > 1.0 {
            confidence += 0.2;
        }

        // Higher confidence for high Q-factor resonances
        if peak.q_factor > 10.0 {
            confidence += 0.1;
        }

        confidence.min(1.0)
    }

    fn generate_description(&self, osc_type: &OscillationType, frequency: f32) -> String {
        match osc_type {
            OscillationType::PTermOscillation => {
                format!(
                    "P-term oscillation detected at {:.1} Hz. Consider reducing P gain.",
                    frequency
                )
            }
            OscillationType::DTermOscillation => {
                format!("D-term oscillation detected at {:.1} Hz. Consider reducing D gain or lowering D-term filter cutoff.", frequency)
            }
            OscillationType::MechanicalResonance => {
                format!("Mechanical resonance detected at {:.1} Hz. Check for loose hardware or add notch filter.", frequency)
            }
            OscillationType::MotorNoise => {
                format!(
                    "Motor/propeller noise detected at {:.1} Hz. Check motor/prop balance.",
                    frequency
                )
            }
        }
    }
}

impl Default for OscillationDetectorConfig {
    fn default() -> Self {
        Self {
            min_amplitude: 0.1,
            resonance_q_threshold: 5.0,
            frequency_bands: FrequencyBands {
                p_term_band: (5.0, 50.0),
                d_term_band: (50.0, 200.0),
                mechanical_band: (80.0, 500.0),
                motor_noise_band: (200.0, 1000.0),
            },
        }
    }
}

impl FilterOptimizer {
    fn new() -> Self {
        Self {
            config: FilterOptimizerConfig::default(),
        }
    }

    fn optimize(
        &self,
        oscillations: &[DetectedOscillation],
        _hardware: &HardwareConfiguration,
    ) -> Result<Vec<FilterRecommendation>> {
        let mut recommendations = Vec::new();

        for oscillation in oscillations {
            let recommendation = match oscillation.oscillation_type {
                OscillationType::MechanicalResonance if oscillation.q_factor > 10.0 => {
                    FilterRecommendation {
                        recommendation_type: FilterRecommendationType::AddNotchFilter,
                        frequency: oscillation.frequency,
                        q_factor: Some((oscillation.q_factor / 2.0).max(5.0)),
                        expected_improvement: "Significant reduction in mechanical resonance"
                            .to_string(),
                        priority: Priority::High,
                    }
                }
                OscillationType::DTermOscillation => {
                    FilterRecommendation {
                        recommendation_type: FilterRecommendationType::AdjustLowPassCutoff {
                            current: 100.0, // Would get from actual config
                            recommended: oscillation.frequency * 0.7,
                        },
                        frequency: oscillation.frequency,
                        q_factor: None,
                        expected_improvement:
                            "Reduced D-term noise while maintaining responsiveness".to_string(),
                        priority: Priority::Medium,
                    }
                }
                _ => continue, // Skip other types for now
            };

            recommendations.push(recommendation);
        }

        Ok(recommendations)
    }
}

impl Default for FilterOptimizerConfig {
    fn default() -> Self {
        Self {
            max_notch_filters: 2,
            min_attenuation_db: 20.0,
            max_group_delay_ms: 1.0,
            preferred_filter_types: vec![FilterType::LowPass, FilterType::Butterworth],
        }
    }
}

impl PidAnalyzer {
    fn new() -> Self {
        Self {
            config: PidAnalyzerConfig::default(),
        }
    }

    fn analyze(&self, telemetry: &TelemetryData) -> Result<Vec<PidRecommendation>> {
        let mut recommendations = Vec::new();

        // Check if we have RC command data
        if telemetry.rc_commands.roll.is_empty() {
            tracing::warn!("No RC command data available, performing gyro-only analysis");
            // Perform gyro-only analysis
            recommendations.extend(self.analyze_gyro_characteristics(telemetry)?);
        } else {
            tracing::info!("RC command data available, performing step response analysis");

            // Detect step responses for each axis
            let roll_responses = self.detect_step_responses(
                &telemetry.rc_commands.roll,
                &telemetry.gyro.x,
                telemetry.sample_rate,
                Axis::Roll,
            )?;

            let pitch_responses = self.detect_step_responses(
                &telemetry.rc_commands.pitch,
                &telemetry.gyro.y,
                telemetry.sample_rate,
                Axis::Pitch,
            )?;

            let yaw_responses = self.detect_step_responses(
                &telemetry.rc_commands.yaw,
                &telemetry.gyro.z,
                telemetry.sample_rate,
                Axis::Yaw,
            )?;

            // Analyze each axis and generate recommendations
            recommendations.extend(self.analyze_axis_responses(&roll_responses, Axis::Roll)?);
            recommendations.extend(self.analyze_axis_responses(&pitch_responses, Axis::Pitch)?);
            recommendations.extend(self.analyze_axis_responses(&yaw_responses, Axis::Yaw)?);

            tracing::info!(
                "PID analysis found {} step responses: {} roll, {} pitch, {} yaw",
                roll_responses.len() + pitch_responses.len() + yaw_responses.len(),
                roll_responses.len(),
                pitch_responses.len(),
                yaw_responses.len()
            );
        }

        // Check if we have PID error data for additional analysis
        if !telemetry.pid_error.roll.is_empty() {
            tracing::info!("PID error data available, performing error analysis");
            recommendations.extend(self.analyze_pid_errors(telemetry)?);
        }

        Ok(recommendations)
    }

    /// Detect step responses in RC command and gyro data
    fn detect_step_responses(
        &self,
        rc_commands: &[f32],
        gyro_response: &[f32],
        sample_rate: f32,
        axis: Axis,
    ) -> Result<Vec<StepResponse>> {
        let mut responses = Vec::new();

        if rc_commands.len() != gyro_response.len() {
            return Err(DronetunerError::analysis_error(
                "RC command and gyro data length mismatch",
            ));
        }

        let min_step_size = 0.1; // Minimum step size to consider (10% of full range)
        let min_duration_samples = (0.05 * sample_rate) as usize; // 50ms minimum
        let max_duration_samples = (2.0 * sample_rate) as usize; // 2s maximum

        // Find step inputs in RC commands
        for i in 1..rc_commands.len() {
            let step_size = (rc_commands[i] - rc_commands[i - 1]).abs();

            // Check if this is a significant step
            if step_size > min_step_size {
                // Look for the end of the step (when RC command stabilizes)
                let mut step_end = i;
                for j in (i + 1)..rc_commands.len().min(i + max_duration_samples) {
                    if (rc_commands[j] - rc_commands[i]).abs() > step_size * 0.2 {
                        // Command changed significantly again, this step ended
                        break;
                    }
                    step_end = j;
                }

                // Ensure minimum duration
                if step_end - i >= min_duration_samples {
                    let response = self.analyze_step_response(
                        i,
                        step_end,
                        rc_commands,
                        gyro_response,
                        sample_rate,
                        axis.clone(),
                    )?;

                    if let Some(resp) = response {
                        responses.push(resp);
                    }
                }
            }
        }

        Ok(responses)
    }

    /// Analyze a single step response and extract performance metrics
    fn analyze_step_response(
        &self,
        step_start: usize,
        _step_end: usize,
        rc_commands: &[f32],
        gyro_response: &[f32],
        sample_rate: f32,
        axis: Axis,
    ) -> Result<Option<StepResponse>> {
        let step_command = rc_commands[step_start];
        let initial_command = rc_commands[step_start - 1];
        let command_change = step_command - initial_command;

        // Extract response window (extend a bit beyond step to see settling)
        let analysis_window = ((self.config.response_window_s * sample_rate) as usize)
            .min(gyro_response.len() - step_start);

        if analysis_window < 10 {
            return Ok(None); // Too short to analyze
        }

        let response_window = &gyro_response[step_start..step_start + analysis_window];
        let baseline_gyro = gyro_response[step_start - 1];

        // Calculate expected steady-state response
        // For gyro, we expect it to be proportional to the rate command
        let expected_response = command_change * 500.0; // Rough scaling, should be configurable

        // Calculate performance metrics
        let metrics = self.calculate_step_metrics(
            response_window,
            baseline_gyro,
            expected_response,
            sample_rate,
        )?;

        Ok(Some(StepResponse {
            axis,
            start_time: step_start as f32 / sample_rate,
            command_magnitude: command_change.abs(),
            rise_time: metrics.rise_time,
            settling_time: metrics.settling_time,
            overshoot_percent: metrics.overshoot_percent,
            oscillation_frequency: metrics.oscillation_frequency,
            damping_ratio: metrics.damping_ratio,
            steady_state_error: metrics.steady_state_error,
        }))
    }

    /// Calculate step response performance metrics
    fn calculate_step_metrics(
        &self,
        response: &[f32],
        baseline: f32,
        expected_final: f32,
        sample_rate: f32,
    ) -> Result<StepMetrics> {
        let dt = 1.0 / sample_rate;

        // Find rise time (10% to 90% of final value)
        let ten_percent = baseline + 0.1 * expected_final;
        let ninety_percent = baseline + 0.9 * expected_final;

        let mut rise_start_idx = None;
        let mut rise_end_idx = None;

        for (i, &value) in response.iter().enumerate() {
            if rise_start_idx.is_none() && value >= ten_percent {
                rise_start_idx = Some(i);
            }
            if rise_start_idx.is_some() && rise_end_idx.is_none() && value >= ninety_percent {
                rise_end_idx = Some(i);
                break;
            }
        }

        let rise_time = match (rise_start_idx, rise_end_idx) {
            (Some(start), Some(end)) => (end - start) as f32 * dt,
            _ => 0.1, // Default if we can't measure
        };

        // Find peak value for overshoot calculation
        let peak_value = response.iter().fold(baseline, |max, &val| val.max(max));
        let overshoot_percent = if expected_final.abs() > 0.01 {
            ((peak_value - baseline - expected_final) / expected_final.abs()) * 100.0
        } else {
            0.0
        };

        // Calculate settling time (within 2% of final value)
        let settling_tolerance = expected_final.abs() * 0.02;
        let target_value = baseline + expected_final;

        let mut settling_time = response.len() as f32 * dt; // Default to full window
        for (i, &value) in response.iter().enumerate().rev() {
            if (value - target_value).abs() > settling_tolerance {
                settling_time = (i + 1) as f32 * dt;
                break;
            }
        }

        // Estimate oscillation frequency by finding dominant frequency in response
        let oscillation_frequency = self.estimate_oscillation_frequency(response, sample_rate)?;

        // Estimate damping ratio from overshoot
        let damping_ratio = if overshoot_percent > 0.1 {
            // Using overshoot to estimate damping ratio
            let overshoot_ratio = overshoot_percent / 100.0;
            if overshoot_ratio > 0.0 {
                (-((overshoot_ratio * std::f32::consts::PI)
                    / (1.0
                        + overshoot_ratio
                            * overshoot_ratio
                            * std::f32::consts::PI
                            * std::f32::consts::PI)
                        .sqrt()))
                .exp()
            } else {
                1.0
            }
        } else {
            1.0 // Well damped
        };

        // Calculate steady-state error
        let final_samples = response.len().min(10); // Last 10 samples
        let steady_state_value = response[response.len() - final_samples..]
            .iter()
            .sum::<f32>()
            / final_samples as f32;
        let steady_state_error = ((steady_state_value - target_value) / expected_final.abs()).abs();

        Ok(StepMetrics {
            rise_time,
            settling_time,
            overshoot_percent,
            oscillation_frequency,
            damping_ratio,
            steady_state_error,
        })
    }

    /// Estimate the dominant oscillation frequency in the response
    fn estimate_oscillation_frequency(&self, response: &[f32], sample_rate: f32) -> Result<f32> {
        if response.len() < 8 {
            return Ok(0.0);
        }

        // Simple zero-crossing method for frequency estimation
        let mean = response.iter().sum::<f32>() / response.len() as f32;
        let mut zero_crossings = 0;

        for i in 1..response.len() {
            if (response[i - 1] - mean) * (response[i] - mean) < 0.0 {
                zero_crossings += 1;
            }
        }

        if zero_crossings > 2 {
            let frequency = (zero_crossings as f32 / 2.0) / (response.len() as f32 / sample_rate);
            Ok(frequency)
        } else {
            Ok(0.0)
        }
    }

    /// Analyze gyro characteristics when RC command data is not available
    fn analyze_gyro_characteristics(
        &self,
        telemetry: &TelemetryData,
    ) -> Result<Vec<PidRecommendation>> {
        let mut recommendations = Vec::new();

        // Analyze gyro noise and oscillations for each axis
        for (axis, gyro_data) in [
            (Axis::Roll, &telemetry.gyro.x),
            (Axis::Pitch, &telemetry.gyro.y),
            (Axis::Yaw, &telemetry.gyro.z),
        ] {
            let analysis = self.analyze_gyro_noise_and_oscillations(
                gyro_data,
                telemetry.sample_rate,
                axis.clone(),
            )?;
            recommendations.extend(analysis);
        }

        tracing::info!(
            "Generated {} recommendations from gyro analysis",
            recommendations.len()
        );
        Ok(recommendations)
    }

    /// Analyze gyro noise and oscillations for a single axis
    fn analyze_gyro_noise_and_oscillations(
        &self,
        gyro_data: &[f32],
        sample_rate: f32,
        axis: Axis,
    ) -> Result<Vec<PidRecommendation>> {
        let mut recommendations = Vec::new();

        if gyro_data.len() < 100 {
            return Ok(recommendations);
        }

        // Calculate basic statistics
        let mean = gyro_data.iter().sum::<f32>() / gyro_data.len() as f32;
        let variance = gyro_data
            .iter()
            .map(|&x| (x - mean) * (x - mean))
            .sum::<f32>()
            / gyro_data.len() as f32;
        let std_dev = variance.sqrt();

        // Calculate noise level (high frequency component)
        let noise_level = self.estimate_noise_level(gyro_data, sample_rate)?;

        // Check for excessive noise
        if noise_level > 15.0 {
            // Adjustable threshold (lowered to trigger more often)
            recommendations.push(PidRecommendation {
                axis: axis.clone(),
                term: PidTerm::D,
                current_value: 20.0,           // Would get from actual config
                recommended_value: 20.0 * 0.8, // Reduce D-term
                reason: format!(
                    "Reduce D-term to decrease gyro noise (noise level: {:.1})",
                    noise_level
                ),
                priority: Priority::Medium,
            });
        }

        // Check for low frequency oscillations
        let oscillation_amplitude =
            self.detect_low_frequency_oscillations(gyro_data, sample_rate)?;
        if oscillation_amplitude > 5.0 {
            // Adjustable threshold (lowered to trigger more often)
            recommendations.push(PidRecommendation {
                axis: axis.clone(),
                term: PidTerm::P,
                current_value: 50.0,           // Would get from actual config
                recommended_value: 50.0 * 0.9, // Reduce P-term
                reason: format!(
                    "Reduce P-term to decrease low-frequency oscillations (amplitude: {:.1})",
                    oscillation_amplitude
                ),
                priority: Priority::Medium,
            });
        }

        // Check for very high standard deviation (general instability)
        if std_dev > 20.0 {
            // Lowered threshold to trigger more often
            recommendations.push(PidRecommendation {
                axis: axis.clone(),
                term: PidTerm::P,
                current_value: 50.0,
                recommended_value: 50.0 * 0.85, // Reduce P-term more aggressively
                reason: format!(
                    "Reduce P-term to improve general stability (std dev: {:.1})",
                    std_dev
                ),
                priority: Priority::High,
            });
        }

        Ok(recommendations)
    }

    /// Estimate noise level in gyro data
    fn estimate_noise_level(&self, gyro_data: &[f32], sample_rate: f32) -> Result<f32> {
        if gyro_data.len() < 10 {
            return Ok(0.0);
        }

        // Simple high-pass filter to estimate noise
        // Calculate differences between consecutive samples
        let differences: Vec<f32> = gyro_data.windows(2).map(|w| (w[1] - w[0]).abs()).collect();

        // Average absolute difference scaled by sample rate
        let avg_diff = differences.iter().sum::<f32>() / differences.len() as f32;
        let noise_estimate = avg_diff * sample_rate / 100.0; // Scaling factor

        Ok(noise_estimate)
    }

    /// Detect low frequency oscillations in gyro data
    fn detect_low_frequency_oscillations(
        &self,
        gyro_data: &[f32],
        _sample_rate: f32,
    ) -> Result<f32> {
        if gyro_data.len() < 50 {
            return Ok(0.0);
        }

        // Simple approach: look for periodic patterns in a moving window
        let window_size = 20;
        let mut max_oscillation: f32 = 0.0;

        for i in 0..(gyro_data.len() - window_size) {
            let window = &gyro_data[i..i + window_size];
            let window_mean = window.iter().sum::<f32>() / window.len() as f32;
            let max_deviation = window
                .iter()
                .map(|&x| (x - window_mean).abs())
                .fold(0.0, f32::max);

            max_oscillation = max_oscillation.max(max_deviation);
        }

        Ok(max_oscillation)
    }

    /// Analyze PID error signals when available
    fn analyze_pid_errors(&self, telemetry: &TelemetryData) -> Result<Vec<PidRecommendation>> {
        let mut recommendations = Vec::new();

        // Analyze each axis PID error
        for (axis, error_data) in [
            (Axis::Roll, &telemetry.pid_error.roll),
            (Axis::Pitch, &telemetry.pid_error.pitch),
            (Axis::Yaw, &telemetry.pid_error.yaw),
        ] {
            if !error_data.is_empty() {
                let analysis = self.analyze_pid_error_axis(error_data, axis.clone())?;
                recommendations.extend(analysis);
            }
        }

        Ok(recommendations)
    }

    /// Analyze PID error for a single axis
    fn analyze_pid_error_axis(
        &self,
        error_data: &[f32],
        axis: Axis,
    ) -> Result<Vec<PidRecommendation>> {
        let mut recommendations = Vec::new();

        if error_data.len() < 10 {
            return Ok(recommendations);
        }

        // Calculate RMS error
        let rms_error =
            (error_data.iter().map(|&x| x * x).sum::<f32>() / error_data.len() as f32).sqrt();

        // Check for persistent bias in error
        let error_mean = error_data.iter().sum::<f32>() / error_data.len() as f32;
        if error_mean.abs() > 2.0 {
            recommendations.push(PidRecommendation {
                axis: axis.clone(),
                term: PidTerm::I,
                current_value: 30.0,
                recommended_value: 30.0 * 1.3, // Increase I-term more
                reason: format!(
                    "Increase I-term to eliminate error bias (bias: {:.2})",
                    error_mean
                ),
                priority: Priority::High,
            });
        }

        // High RMS error suggests need for more aggressive PID terms
        if rms_error > 5.0 {
            // Adjustable threshold
            recommendations.push(PidRecommendation {
                axis: axis.clone(),
                term: PidTerm::I,
                current_value: 30.0,           // Would get from actual config
                recommended_value: 30.0 * 1.2, // Increase I-term
                reason: format!(
                    "Increase I-term to reduce steady-state error (RMS error: {:.2})",
                    rms_error
                ),
                priority: Priority::Medium,
            });
        }

        Ok(recommendations)
    }

    /// Analyze step responses for an axis and generate PID recommendations
    fn analyze_axis_responses(
        &self,
        responses: &[StepResponse],
        axis: Axis,
    ) -> Result<Vec<PidRecommendation>> {
        if responses.is_empty() {
            return Ok(Vec::new());
        }

        let mut recommendations = Vec::new();

        // Calculate average metrics across all responses
        let avg_rise_time =
            responses.iter().map(|r| r.rise_time).sum::<f32>() / responses.len() as f32;
        let avg_overshoot =
            responses.iter().map(|r| r.overshoot_percent).sum::<f32>() / responses.len() as f32;
        let avg_settling_time =
            responses.iter().map(|r| r.settling_time).sum::<f32>() / responses.len() as f32;
        let avg_oscillation_freq = responses
            .iter()
            .map(|r| r.oscillation_frequency)
            .sum::<f32>()
            / responses.len() as f32;
        let avg_damping =
            responses.iter().map(|r| r.damping_ratio).sum::<f32>() / responses.len() as f32;

        // Analyze P-term based on overshoot and rise time
        if avg_overshoot > self.config.overshoot_tolerance {
            let reduction_percent = (avg_overshoot - self.config.overshoot_tolerance) / 50.0; // Scale factor
            recommendations.push(PidRecommendation {
                axis: axis.clone(),
                term: PidTerm::P,
                current_value: 50.0, // Would get from actual config
                recommended_value: 50.0 * (1.0 - reduction_percent.min(0.3)), // Max 30% reduction
                reason: format!(
                    "Reduce P-term to decrease overshoot from {:.1}% to target {:.1}%",
                    avg_overshoot, self.config.overshoot_tolerance
                ),
                priority: if avg_overshoot > 25.0 {
                    Priority::High
                } else {
                    Priority::Medium
                },
            });
        } else if avg_rise_time > 0.15 && avg_overshoot < 5.0 {
            // Slow rise time with little overshoot suggests P could be increased
            recommendations.push(PidRecommendation {
                axis: axis.clone(),
                term: PidTerm::P,
                current_value: 50.0,
                recommended_value: 50.0 * 1.1, // 10% increase
                reason: format!(
                    "Increase P-term to improve responsiveness (rise time: {:.3}s)",
                    avg_rise_time
                ),
                priority: Priority::Low,
            });
        }

        // Analyze I-term based on steady-state error
        let avg_ss_error =
            responses.iter().map(|r| r.steady_state_error).sum::<f32>() / responses.len() as f32;
        if avg_ss_error > 0.05 {
            recommendations.push(PidRecommendation {
                axis: axis.clone(),
                term: PidTerm::I,
                current_value: 50.0,
                recommended_value: 50.0 * 1.2, // 20% increase
                reason: format!(
                    "Increase I-term to reduce steady-state error ({:.1}%)",
                    avg_ss_error * 100.0
                ),
                priority: Priority::Medium,
            });
        }

        // Analyze D-term based on oscillations and damping
        if avg_oscillation_freq > 10.0 && avg_damping < 0.5 {
            recommendations.push(PidRecommendation {
                axis: axis.clone(),
                term: PidTerm::D,
                current_value: 20.0,
                recommended_value: 20.0 * 1.3, // 30% increase
                reason: format!(
                    "Increase D-term to dampen oscillations ({:.1} Hz, damping: {:.2})",
                    avg_oscillation_freq, avg_damping
                ),
                priority: Priority::Medium,
            });
        } else if avg_settling_time > 0.5 && avg_oscillation_freq < 5.0 {
            // Long settling time might indicate too much D-term
            recommendations.push(PidRecommendation {
                axis: axis.clone(),
                term: PidTerm::D,
                current_value: 20.0,
                recommended_value: 20.0 * 0.8, // 20% decrease
                reason: format!(
                    "Reduce D-term to improve settling time ({:.3}s)",
                    avg_settling_time
                ),
                priority: Priority::Low,
            });
        }

        Ok(recommendations)
    }
}

impl Default for PidAnalyzerConfig {
    fn default() -> Self {
        Self {
            error_threshold: 0.1,
            response_window_s: 1.0,    // Increased to capture full response
            overshoot_tolerance: 15.0, // Reasonable overshoot tolerance
        }
    }
}

#[cfg(test)]
mod tests {
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
}
