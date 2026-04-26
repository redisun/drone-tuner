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
    /// Minimum Q factor to justify a notch filter
    pub min_q_factor_for_notch: f32,
    /// Minimum improvement threshold for recommendations
    pub min_improvement_threshold: f32,
    /// Q factor for recommended notch filters
    pub notch_q_factor: f32,
    /// Default lowpass filter frequency
    pub default_lowpass_frequency: f32,
    /// Frequency multiplier for lowpass recommendations
    pub lowpass_frequency_multiplier: f32,
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
        let pid_recommendations = self
            .pid_analyzer
            .analyze(&session.telemetry, &session.metadata.hardware.pid_config)?;
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
    /// Cross-axis correlation coefficient
    cross_axis_correlation: f32,
    /// Severity based on flight impact
    severity: OscillationSeverity,
}

/// Oscillation severity levels based on flight impact
#[derive(Debug, Clone, PartialEq)]
enum OscillationSeverity {
    /// Minor issues, barely noticeable
    Low,
    /// Noticeable but manageable
    Medium,
    /// Performance significantly degraded
    High,
    /// Flight unsafe conditions
    Critical,
}

#[derive(Debug, Clone, PartialEq)]
enum OscillationType {
    PTermOscillation,
    DTermOscillation,
    MechanicalResonance,
    MotorNoise,
}

impl OscillationDetector {
    fn new() -> Self {
        let mut detector = Self {
            config: OscillationDetectorConfig::default(),
            patterns: Vec::new(),
        };

        // Initialize with common oscillation patterns
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

    fn detect(
        &self,
        freq_analysis: &FrequencyAnalysisResult,
        hardware: &HardwareConfiguration,
    ) -> Result<Vec<DetectedOscillation>> {
        let mut oscillations = Vec::new();

        for peak in &freq_analysis.peaks {
            // Enhanced classification with cross-axis correlation
            let cross_axis_correlation =
                self.get_cross_axis_correlation_for_peak(peak, freq_analysis);

            let oscillation_type = self.classify_oscillation_type_enhanced(
                peak.frequency,
                peak.q_factor,
                cross_axis_correlation,
                hardware,
            );

            // Enhanced severity assessment
            let severity = self.assess_oscillation_severity(
                peak.magnitude,
                peak.q_factor,
                &oscillation_type,
                hardware,
            );

            // Enhanced confidence calculation
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
    fn classify_oscillation_type(&self, frequency: f32, q_factor: f32) -> OscillationType {
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
                // P-term oscillations should have high cross-axis correlation and low Q
                if cross_axis_correlation > self.config.correlation_thresholds.p_term_correlation
                    && q_factor < 5.0
                {
                    OscillationType::PTermOscillation
                } else if q_factor > self.config.resonance_q_threshold {
                    // High Q in P-term range suggests mechanical resonance
                    OscillationType::MechanicalResonance
                } else {
                    OscillationType::PTermOscillation
                }
            }
            f if f >= self.config.frequency_bands.d_term_band.0
                && f <= self.config.frequency_bands.d_term_band.1 =>
            {
                // D-term oscillations should have medium frequency, low Q, single axis correlation
                if cross_axis_correlation < self.config.correlation_thresholds.d_term_correlation
                    && q_factor < 3.0
                {
                    OscillationType::DTermOscillation
                } else if q_factor > self.config.resonance_q_threshold {
                    // High Q in D-term range suggests mechanical resonance
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
            _ => OscillationType::MotorNoise, // Default for high frequency content
        }
    }

    /// Get cross-axis correlation for a specific peak
    fn get_cross_axis_correlation_for_peak(
        &self,
        peak: &SpectralPeak,
        freq_analysis: &FrequencyAnalysisResult,
    ) -> f32 {
        // Simplified: return the first available cross-correlation value
        // In a full implementation, we would compute correlation at the specific frequency
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
        // Flight-impact-based severity assessment
        match oscillation_type {
            OscillationType::MechanicalResonance => {
                if q_factor > self.config.severity_params.critical_q_factor
                    || amplitude > self.config.severity_params.critical_amplitude
                {
                    OscillationSeverity::Critical // Flight unsafe
                } else if q_factor > self.config.severity_params.high_q_factor
                    || amplitude > self.config.severity_params.high_amplitude
                {
                    OscillationSeverity::High // Performance significantly degraded
                } else if amplitude > self.config.severity_params.medium_amplitude {
                    OscillationSeverity::Medium // Noticeable but manageable
                } else {
                    OscillationSeverity::Low // Minor issues
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
                    OscillationSeverity::Medium // Motor noise rarely critical
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
        let mut confidence: f32 = 0.6; // Lower base confidence, build up from evidence

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
            _ => {
                // Neutral for mechanical and motor noise
            }
        }

        // Environmental compensation could be added here
        // based on hardware.environment if available

        confidence.max(0.0).min(1.0)
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
                OscillationSeverity::Critical => {
                    format!("Severe P-term oscillation at {:.1} Hz causing flight instability. Immediately reduce P gain by 20-30%.", frequency)
                }
                OscillationSeverity::High => {
                    format!("Strong P-term oscillation at {:.1} Hz affecting performance. Reduce P gain by 10-20%.", frequency)
                }
                _ => {
                    format!(
                        "P-term oscillation at {:.1} Hz. Consider reducing P gain by 5-10%.",
                        frequency
                    )
                }
            },
            OscillationType::DTermOscillation => match severity {
                OscillationSeverity::High | OscillationSeverity::Critical => {
                    format!("High-frequency D-term oscillation at {:.1} Hz. Reduce D gain or lower D-term filter cutoff significantly.", frequency)
                }
                _ => {
                    format!("D-term oscillation at {:.1} Hz. Consider reducing D gain or lowering D-term filter cutoff.", frequency)
                }
            },
            OscillationType::MechanicalResonance => match severity {
                OscillationSeverity::Critical => {
                    format!("DANGEROUS mechanical resonance at {:.1} Hz. Check frame integrity and add notch filter immediately.", frequency)
                }
                OscillationSeverity::High => {
                    format!("Sharp mechanical resonance at {:.1} Hz affecting flight quality. Add notch filter or check hardware.", frequency)
                }
                _ => {
                    format!("Mechanical resonance at {:.1} Hz. Consider adding notch filter or checking for loose hardware.", frequency)
                }
            },
            OscillationType::MotorNoise => {
                format!(
                    "Motor/propeller noise at {:.1} Hz. Check motor/prop balance and mounting.",
                    frequency
                )
            }
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
                // Updated frequency ranges based on FPV physics analysis
                p_term_band: (3.0, 50.0), // Extended lower for large builds
                d_term_band: (50.0, 300.0), // Extended upper range
                mechanical_band: (200.0, 800.0), // Reduced overlap, extended range
                motor_noise_band: (200.0, 1200.0), // Extended for larger motors
            },
            correlation_thresholds: CorrelationThresholds {
                p_term_correlation: 0.7,     // High cross-axis correlation for P-term
                d_term_correlation: 0.5,     // Low cross-axis correlation for D-term
                mechanical_correlation: 0.6, // Medium correlation for mechanical
            },
            severity_params: SeverityParameters {
                critical_amplitude: 20.0, // Flight unsafe amplitude
                high_amplitude: 10.0,     // Performance significantly degraded
                medium_amplitude: 5.0,    // Noticeable but manageable
                critical_q_factor: 15.0,  // Very sharp resonances are dangerous
                high_q_factor: 8.0,       // Sharp resonances need attention
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
        hardware: &HardwareConfiguration,
    ) -> Result<Vec<FilterRecommendation>> {
        let mut recommendations = Vec::new();
        let mut used_gyro_notches = 0;
        let improvement_threshold = self.config.min_improvement_threshold;

        // Group oscillations by type and frequency range
        let mut mechanical_resonances: Vec<_> = oscillations
            .iter()
            .filter(|osc| matches!(osc.oscillation_type, OscillationType::MechanicalResonance))
            .collect();

        let dterm_oscillations: Vec<_> = oscillations
            .iter()
            .filter(|osc| matches!(osc.oscillation_type, OscillationType::DTermOscillation))
            .collect();

        // Sort by amplitude (strongest first)
        mechanical_resonances.sort_by(|a, b| {
            b.amplitude
                .partial_cmp(&a.amplitude)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        // Handle mechanical resonances with appropriate Betaflight filters
        for oscillation in mechanical_resonances {
            let freq = oscillation.frequency;
            let q_factor = oscillation.q_factor.max(self.config.notch_q_factor);

            // Determine best filter type based on frequency and existing configuration
            if freq > 300.0 && used_gyro_notches < 2 {
                // High frequency - use gyro notch filter
                recommendations.push(FilterRecommendation {
                    recommendation_type: FilterRecommendationType::ConfigureGyroNotch {
                        notch_number: used_gyro_notches + 1,
                        frequency: freq,
                        q_factor,
                        enabled: true,
                    },
                    frequency: freq,
                    q_factor: Some(q_factor),
                    expected_improvement: format!(
                        "Expected {:.1}% reduction in mechanical resonance at {:.0} Hz",
                        improvement_threshold * 100.0,
                        freq
                    ),
                    priority: Priority::High,
                });
                used_gyro_notches += 1;
            } else if freq < 300.0 && hardware.filter_config.dynamic_notch.is_some() {
                // Lower frequency - adjust dynamic notch range
                let current_dynamic = hardware.filter_config.dynamic_notch.as_ref().unwrap();
                let new_min_freq = freq * 0.8;
                let new_max_freq = current_dynamic.max_freq.max(freq * 1.5);

                recommendations.push(FilterRecommendation {
                    recommendation_type: FilterRecommendationType::AdjustDynamicNotch {
                        notch_count: 1,
                        q_factor: q_factor.min(500.0), // Betaflight typical range
                        min_freq: new_min_freq,
                        max_freq: new_max_freq,
                        enabled: true,
                    },
                    frequency: freq,
                    q_factor: Some(q_factor),
                    expected_improvement: format!(
                        "Expected {:.1}% reduction in frame resonance by expanding dynamic notch range",
                        improvement_threshold * 100.0
                    ),
                    priority: Priority::Medium,
                });
            } else if hardware
                .flight_controller
                .firmware
                .to_lowercase()
                .contains("betaflight")
            {
                // Consider RPM filter for motor-related frequencies
                if freq > 50.0 && freq < 200.0 && oscillation.affected_axes.len() > 1 {
                    recommendations.push(FilterRecommendation {
                        recommendation_type: FilterRecommendationType::ConfigureRpmFilter {
                            harmonics: 3,
                            q_factor: 500.0, // Betaflight default
                            min_freq: 100.0,
                            enabled: true,
                        },
                        frequency: freq,
                        q_factor: Some(500.0),
                        expected_improvement: format!(
                            "Expected {:.1}% reduction in motor noise using RPM filtering",
                            improvement_threshold * 100.0
                        ),
                        priority: Priority::High,
                    });
                }
            }
        }

        // Handle D-term oscillations
        for oscillation in dterm_oscillations {
            let freq = oscillation.frequency;
            let recommended_cutoff = freq * self.config.lowpass_frequency_multiplier;

            // Determine which D-term filter stage to adjust
            let current_dterm_filters = &hardware.filter_config.dterm_filters;

            if current_dterm_filters.is_empty()
                || current_dterm_filters[0].cutoff > recommended_cutoff
            {
                // Adjust first stage D-term filter
                recommendations.push(FilterRecommendation {
                    recommendation_type: FilterRecommendationType::AdjustDtermLowpass {
                        stage: 1,
                        current_cutoff: current_dterm_filters.get(0).map(|f| f.cutoff),
                        recommended_cutoff: Some(recommended_cutoff),
                        filter_type: "BIQUAD".to_string(), // Modern Betaflight default
                        dynamic_settings: Some(crate::domain::DynamicFilterSettings {
                            min_cutoff: recommended_cutoff * 0.75,
                            max_cutoff: recommended_cutoff * 1.5,
                            expo: 5.0, // Betaflight default expo
                        }),
                    },
                    frequency: freq,
                    q_factor: None,
                    expected_improvement: format!(
                        "Expected {:.1}% reduction in D-term noise with dynamic filtering",
                        improvement_threshold * 100.0
                    ),
                    priority: Priority::Medium,
                });
            }
        }

        // Always suggest enabling RPM filter if not mentioned and we have motor-related oscillations
        if !recommendations.iter().any(|r| {
            matches!(
                r.recommendation_type,
                FilterRecommendationType::ConfigureRpmFilter { .. }
            )
        }) {
            let motor_oscillations: Vec<_> = oscillations
                .iter()
                .filter(|osc| {
                    osc.frequency > 50.0 && osc.frequency < 200.0 && osc.affected_axes.len() >= 2
                })
                .collect();

            if !motor_oscillations.is_empty() {
                recommendations.push(FilterRecommendation {
                    recommendation_type: FilterRecommendationType::ConfigureRpmFilter {
                        harmonics: 3,
                        q_factor: 500.0,
                        min_freq: 100.0,
                        enabled: true,
                    },
                    frequency: motor_oscillations[0].frequency,
                    q_factor: Some(500.0),
                    expected_improvement:
                        "Enable bidirectional DShot and RPM filtering for motor noise reduction"
                            .to_string(),
                    priority: Priority::High,
                });
            }
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
            min_q_factor_for_notch: 10.0,
            min_improvement_threshold: 0.15, // 15% improvement
            notch_q_factor: 5.0,
            default_lowpass_frequency: 100.0,
            lowpass_frequency_multiplier: 0.7,
        }
    }
}

impl PidAnalyzer {
    fn new() -> Self {
        Self {
            config: PidAnalyzerConfig::default(),
        }
    }

    fn analyze(
        &self,
        telemetry: &TelemetryData,
        pid_config: &PidConfiguration,
    ) -> Result<Vec<PidRecommendation>> {
        let mut recommendations = Vec::new();

        // Check if we have RC command data
        if telemetry.rc_commands.roll.is_empty() {
            tracing::warn!("No RC command data available, performing gyro-only analysis");
            // Perform gyro-only analysis
            recommendations.extend(self.analyze_gyro_characteristics(telemetry, pid_config)?);
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
            recommendations.extend(self.analyze_axis_responses(
                &roll_responses,
                Axis::Roll,
                pid_config,
            )?);
            recommendations.extend(self.analyze_axis_responses(
                &pitch_responses,
                Axis::Pitch,
                pid_config,
            )?);
            recommendations.extend(self.analyze_axis_responses(
                &yaw_responses,
                Axis::Yaw,
                pid_config,
            )?);

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
            recommendations.extend(self.analyze_pid_errors(telemetry, pid_config)?);
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
        pid_config: &PidConfiguration,
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
                pid_config,
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
        pid_config: &PidConfiguration,
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

        // Get actual PID values for this axis
        let current_pid = match axis {
            Axis::Roll => &pid_config.roll,
            Axis::Pitch => &pid_config.pitch,
            Axis::Yaw => &pid_config.yaw,
        };

        // Check for excessive noise
        if noise_level > 15.0 {
            // Adjustable threshold (lowered to trigger more often)
            recommendations.push(PidRecommendation {
                axis: axis.clone(),
                term: PidTerm::D,
                current_value: current_pid.d,
                recommended_value: current_pid.d * 0.8, // Reduce D-term
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
                current_value: current_pid.p,
                recommended_value: current_pid.p * 0.9, // Reduce P-term
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
                current_value: current_pid.p,
                recommended_value: current_pid.p * 0.85, // Reduce P-term more aggressively
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
    fn analyze_pid_errors(
        &self,
        telemetry: &TelemetryData,
        pid_config: &PidConfiguration,
    ) -> Result<Vec<PidRecommendation>> {
        let mut recommendations = Vec::new();

        // Analyze each axis PID error
        for (axis, error_data) in [
            (Axis::Roll, &telemetry.pid_error.roll),
            (Axis::Pitch, &telemetry.pid_error.pitch),
            (Axis::Yaw, &telemetry.pid_error.yaw),
        ] {
            if !error_data.is_empty() {
                let analysis = self.analyze_pid_error_axis(error_data, axis.clone(), pid_config)?;
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
        pid_config: &PidConfiguration,
    ) -> Result<Vec<PidRecommendation>> {
        let mut recommendations = Vec::new();

        if error_data.len() < 10 {
            return Ok(recommendations);
        }

        // Calculate RMS error
        let rms_error =
            (error_data.iter().map(|&x| x * x).sum::<f32>() / error_data.len() as f32).sqrt();

        // Get actual PID values for this axis
        let current_pid = match axis {
            Axis::Roll => &pid_config.roll,
            Axis::Pitch => &pid_config.pitch,
            Axis::Yaw => &pid_config.yaw,
        };

        // Check for persistent bias in error
        let error_mean = error_data.iter().sum::<f32>() / error_data.len() as f32;
        if error_mean.abs() > 2.0 {
            recommendations.push(PidRecommendation {
                axis: axis.clone(),
                term: PidTerm::I,
                current_value: current_pid.i,
                recommended_value: current_pid.i * 1.3, // Increase I-term more
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
                current_value: current_pid.i,
                recommended_value: current_pid.i * 1.2, // Increase I-term
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
        pid_config: &PidConfiguration,
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

        // Get actual PID values for this axis
        let current_pid = match axis {
            Axis::Roll => &pid_config.roll,
            Axis::Pitch => &pid_config.pitch,
            Axis::Yaw => &pid_config.yaw,
        };

        // Analyze P-term based on overshoot and rise time
        if avg_overshoot > self.config.overshoot_tolerance {
            let reduction_percent = (avg_overshoot - self.config.overshoot_tolerance) / 50.0; // Scale factor
            recommendations.push(PidRecommendation {
                axis: axis.clone(),
                term: PidTerm::P,
                current_value: current_pid.p,
                recommended_value: current_pid.p * (1.0 - reduction_percent.min(0.3)), // Max 30% reduction
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
                current_value: current_pid.p,
                recommended_value: current_pid.p * 1.1, // 10% increase
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
                current_value: current_pid.i,
                recommended_value: current_pid.i * 1.2, // 20% increase
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
                current_value: current_pid.d,
                recommended_value: current_pid.d * 1.3, // 30% increase
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
                current_value: current_pid.d,
                recommended_value: current_pid.d * 0.8, // 20% decrease
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
        let recommendations = analyzer.analyze(&telemetry, &test_pid_config).unwrap();

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

        println!("✅ PID recommendations now use actual values:");
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
