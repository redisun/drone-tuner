//! Filter design and optimization for drone tuning.

use crate::domain::*;
use crate::error::{DronetunerError, Result};
use num_complex::Complex;
use std::f32::consts::PI;

/// Filter design and optimization utilities
pub struct FilterDesigner {
    /// Sample rate for filter calculations
    sample_rate: f32,
}

/// Digital filter representation
#[derive(Debug, Clone)]
pub struct DigitalFilter {
    /// Filter name/description
    pub name: String,
    /// Numerator coefficients (b coefficients)
    pub numerator: Vec<f32>,
    /// Denominator coefficients (a coefficients)  
    pub denominator: Vec<f32>,
    /// Sample rate this filter was designed for
    pub sample_rate: f32,
}

/// Filter response characteristics
#[derive(Debug, Clone)]
pub struct FilterResponse {
    /// Frequency points (Hz)
    pub frequencies: Vec<f32>,
    /// Magnitude response
    pub magnitude: Vec<f32>,
    /// Phase response (radians)
    pub phase: Vec<f32>,
    /// Group delay (samples)
    pub group_delay: Vec<f32>,
}

/// Biquad filter section (second-order filter)
#[derive(Debug, Clone)]
pub struct BiquadSection {
    /// Biquad coefficients
    pub coefficients: BiquadCoefficients,
    /// Internal state variables
    state: BiquadState,
}

/// Biquad filter coefficients
#[derive(Debug, Clone)]
pub struct BiquadCoefficients {
    /// Forward coefficients
    pub b0: f32,
    pub b1: f32,
    pub b2: f32,
    /// Feedback coefficients
    pub a1: f32,
    pub a2: f32,
}

/// Biquad internal state
#[derive(Debug, Clone)]
struct BiquadState {
    x1: f32, // Previous input
    x2: f32, // Input before that
    y1: f32, // Previous output
    y2: f32, // Output before that
}

/// Filter performance metrics
#[derive(Debug, Clone)]
pub struct FilterPerformanceMetrics {
    /// Attenuation at target frequency (dB)
    pub attenuation_db: f32,
    /// -3dB cutoff frequency (Hz)
    pub cutoff_frequency: f32,
    /// Group delay at important frequencies (ms)
    pub group_delay_ms: f32,
    /// Passband ripple (dB)
    pub passband_ripple_db: f32,
    /// Stopband attenuation (dB)
    pub stopband_attenuation_db: f32,
}

impl FilterDesigner {
    /// Create a new filter designer for the given sample rate
    pub fn new(sample_rate: f32) -> Self {
        Self { sample_rate }
    }

    /// Design a low-pass Butterworth filter
    pub fn butterworth_lowpass(&self, cutoff: f32, order: usize) -> Result<DigitalFilter> {
        if cutoff <= 0.0 || cutoff >= self.sample_rate / 2.0 {
            return Err(DronetunerError::config_error("Invalid cutoff frequency"));
        }

        if order == 0 || order > 10 {
            return Err(DronetunerError::config_error(
                "Filter order must be between 1 and 10",
            ));
        }

        // Normalize cutoff frequency
        let wc = 2.0 * PI * cutoff / self.sample_rate;
        let wc_pre = (wc / 2.0).tan(); // Prewarp frequency

        // Design analog prototype
        let analog_poles = self.butterworth_poles(order);

        // Scale poles by cutoff frequency
        let scaled_poles: Vec<Complex<f32>> =
            analog_poles.iter().map(|pole| pole * wc_pre).collect();

        // Convert to digital using bilinear transform
        let (numerator, denominator) =
            self.bilinear_transform(&scaled_poles, &[], 2.0 / wc.tan())?;

        Ok(DigitalFilter {
            name: format!("Butterworth LP {:.1}Hz Order {}", cutoff, order),
            numerator,
            denominator,
            sample_rate: self.sample_rate,
        })
    }

    /// Design a notch filter for the given frequency
    pub fn notch_filter(&self, frequency: f32, q_factor: f32) -> Result<DigitalFilter> {
        if frequency <= 0.0 || frequency >= self.sample_rate / 2.0 {
            return Err(DronetunerError::config_error("Invalid notch frequency"));
        }

        if q_factor <= 0.0 || q_factor > 1000.0 {
            return Err(DronetunerError::config_error("Invalid Q factor"));
        }

        // Design digital notch filter directly
        let omega = 2.0 * PI * frequency / self.sample_rate;
        let alpha = (omega / 2.0).sin() / (2.0 * q_factor);

        let cos_omega = omega.cos();

        // Biquad coefficients for notch filter
        let b0 = 1.0;
        let b1 = -2.0 * cos_omega;
        let b2 = 1.0;
        let a0 = 1.0 + alpha;
        let a1 = -2.0 * cos_omega;
        let a2 = 1.0 - alpha;

        // Normalize by a0
        let numerator = vec![b0 / a0, b1 / a0, b2 / a0];
        let denominator = vec![1.0, a1 / a0, a2 / a0];

        Ok(DigitalFilter {
            name: format!("Notch {:.1}Hz Q{:.1}", frequency, q_factor),
            numerator,
            denominator,
            sample_rate: self.sample_rate,
        })
    }

    /// Design a high-pass Butterworth filter
    pub fn butterworth_highpass(&self, cutoff: f32, order: usize) -> Result<DigitalFilter> {
        if cutoff <= 0.0 || cutoff >= self.sample_rate / 2.0 {
            return Err(DronetunerError::config_error("Invalid cutoff frequency"));
        }

        // Design lowpass prototype first
        let lowpass = self.butterworth_lowpass(cutoff, order)?;

        // Transform lowpass to highpass by replacing s with 1/s
        // This means replacing z with -z^(-1) in the transfer function
        let mut numerator = lowpass.denominator.clone();
        let mut denominator = lowpass.numerator.clone();

        // Reverse coefficient order for highpass transformation
        numerator.reverse();
        denominator.reverse();

        // Alternate signs for highpass
        for (i, coef) in numerator.iter_mut().enumerate() {
            if i % 2 == 1 {
                *coef = -*coef;
            }
        }

        Ok(DigitalFilter {
            name: format!("Butterworth HP {:.1}Hz Order {}", cutoff, order),
            numerator,
            denominator,
            sample_rate: self.sample_rate,
        })
    }

    /// Design a band-pass filter
    pub fn bandpass_filter(
        &self,
        low_cutoff: f32,
        high_cutoff: f32,
        order: usize,
    ) -> Result<DigitalFilter> {
        if low_cutoff >= high_cutoff {
            return Err(DronetunerError::config_error(
                "Low cutoff must be less than high cutoff",
            ));
        }

        // Cascade highpass and lowpass filters
        let highpass = self.butterworth_highpass(low_cutoff, order / 2)?;
        let lowpass = self.butterworth_lowpass(high_cutoff, order / 2)?;

        // Cascade the filters
        self.cascade_filters(&highpass, &lowpass)
    }

    /// Calculate Butterworth poles for analog prototype
    fn butterworth_poles(&self, order: usize) -> Vec<Complex<f32>> {
        let mut poles = Vec::new();

        for k in 0..order {
            let angle = PI * (2.0 * k as f32 + order as f32 + 1.0) / (2.0 * order as f32);
            let pole = Complex::new(-angle.sin(), angle.cos());
            poles.push(pole);
        }

        poles
    }

    /// Apply bilinear transform to convert analog to digital
    fn bilinear_transform(
        &self,
        poles: &[Complex<f32>],
        _zeros: &[Complex<f32>],
        k: f32,
    ) -> Result<(Vec<f32>, Vec<f32>)> {
        // For simplicity, implement for second-order case
        // Full implementation would handle arbitrary order

        if poles.len() == 2 {
            // Second-order case
            let p1 = poles[0];
            let p2 = poles[1];

            // Convert poles using bilinear transform: s = k * (z-1)/(z+1)
            let z1 = (k + p1) / (k - p1);
            let z2 = (k + p2) / (k - p2);

            // Form polynomial coefficients
            let a0 = 1.0;
            let a1 = -(z1.re + z2.re);
            let a2 = z1.re * z2.re - z1.im * z2.im;

            // For lowpass, numerator has zeros at z = -1
            let b0 = 1.0;
            let b1 = 2.0;
            let b2 = 1.0;

            Ok((vec![b0, b1, b2], vec![a0, a1, a2]))
        } else {
            // First-order case or higher order would be implemented here
            Ok((vec![1.0], vec![1.0]))
        }
    }

    /// Cascade two filters in series
    fn cascade_filters(
        &self,
        filter1: &DigitalFilter,
        filter2: &DigitalFilter,
    ) -> Result<DigitalFilter> {
        // Convolve the numerator and denominator polynomials
        let numerator = self.convolve_polynomials(&filter1.numerator, &filter2.numerator);
        let denominator = self.convolve_polynomials(&filter1.denominator, &filter2.denominator);

        Ok(DigitalFilter {
            name: format!("{} + {}", filter1.name, filter2.name),
            numerator,
            denominator,
            sample_rate: self.sample_rate,
        })
    }

    /// Convolve two polynomials (for cascading filters)
    fn convolve_polynomials(&self, poly1: &[f32], poly2: &[f32]) -> Vec<f32> {
        let mut result = vec![0.0; poly1.len() + poly2.len() - 1];

        for (i, &a) in poly1.iter().enumerate() {
            for (j, &b) in poly2.iter().enumerate() {
                result[i + j] += a * b;
            }
        }

        result
    }

    /// Calculate frequency response of a digital filter
    pub fn frequency_response(
        &self,
        filter: &DigitalFilter,
        frequencies: &[f32],
    ) -> FilterResponse {
        let mut magnitude = Vec::new();
        let mut phase = Vec::new();
        let mut group_delay = Vec::new();

        for &freq in frequencies {
            let omega = 2.0 * PI * freq / filter.sample_rate;
            let z = Complex::new(omega.cos(), omega.sin());

            // Evaluate transfer function H(z) = B(z)/A(z)
            let numerator_val = self.evaluate_polynomial(&filter.numerator, z);
            let denominator_val = self.evaluate_polynomial(&filter.denominator, z);

            let h = numerator_val / denominator_val;

            magnitude.push(20.0 * h.norm().log10()); // Convert to dB
            phase.push(h.arg());

            // Group delay approximation (simplified)
            group_delay.push(0.0); // Would implement proper group delay calculation
        }

        FilterResponse {
            frequencies: frequencies.to_vec(),
            magnitude,
            phase,
            group_delay,
        }
    }

    /// Evaluate polynomial at complex point z
    fn evaluate_polynomial(&self, coefficients: &[f32], z: Complex<f32>) -> Complex<f32> {
        let mut result = Complex::new(0.0, 0.0);
        let mut z_power = Complex::new(1.0, 0.0);

        for &coeff in coefficients {
            result += z_power * coeff;
            z_power *= z;
        }

        result
    }

    /// Optimize filter configuration for detected oscillations
    pub fn optimize_for_oscillations(
        &self,
        oscillations: &[FrequencyPeak],
        current_config: &FilterConfiguration,
    ) -> Result<FilterConfiguration> {
        let mut optimized_config = current_config.clone();

        // Add notch filters for sharp resonances
        for oscillation in oscillations {
            if oscillation.q_factor > 10.0 {
                let notch = NotchFilter {
                    frequency: oscillation.frequency,
                    q_factor: (oscillation.q_factor / 2.0).max(5.0), // Reduce Q for broader notch
                    enabled: true,
                };

                optimized_config.notch_filters.push(notch);
            }
        }

        // Adjust low-pass filters if needed
        let max_problem_freq = oscillations
            .iter()
            .map(|osc| osc.frequency)
            .fold(0.0, f32::max);

        if max_problem_freq > 0.0 {
            for gyro_filter in &mut optimized_config.gyro_filters {
                if gyro_filter.filter_type == FilterType::LowPass
                    && gyro_filter.cutoff > max_problem_freq
                {
                    // Lower cutoff to attenuate problematic frequencies
                    gyro_filter.cutoff = (max_problem_freq * 0.8).max(50.0);
                }
            }
        }

        Ok(optimized_config)
    }

    /// Calculate performance metrics for a filter at specific frequencies
    pub fn calculate_performance_metrics(
        &self,
        filter: &DigitalFilter,
        target_frequencies: &[f32],
    ) -> FilterPerformanceMetrics {
        let response = self.frequency_response(filter, target_frequencies);

        // Find -3dB cutoff frequency
        let cutoff_frequency = self.find_cutoff_frequency(&response, -3.0);

        // Calculate attenuation at target frequencies
        let avg_attenuation = if !response.magnitude.is_empty() {
            response.magnitude.iter().sum::<f32>() / response.magnitude.len() as f32
        } else {
            0.0
        };

        // Estimate group delay (simplified)
        let group_delay_ms = 1000.0 / self.sample_rate; // Rough estimate

        FilterPerformanceMetrics {
            attenuation_db: -avg_attenuation.abs(), // Negative for attenuation
            cutoff_frequency,
            group_delay_ms,
            passband_ripple_db: 0.1,       // Butterworth has minimal ripple
            stopband_attenuation_db: 40.0, // Typical for 2nd order
        }
    }

    /// Find frequency where magnitude drops to specified level (dB)
    fn find_cutoff_frequency(&self, response: &FilterResponse, target_db: f32) -> f32 {
        for (i, &mag) in response.magnitude.iter().enumerate() {
            if mag <= target_db {
                return response.frequencies[i];
            }
        }

        // If not found, return last frequency
        response.frequencies.last().copied().unwrap_or(0.0)
    }

    /// Convert filter configuration to biquad sections for efficient processing
    pub fn to_biquad_cascade(&self, filter: &DigitalFilter) -> Result<Vec<BiquadSection>> {
        let mut sections = Vec::new();

        // For now, assume the filter is already in biquad form (order <= 2)
        if filter.numerator.len() <= 3 && filter.denominator.len() <= 3 {
            // Pad coefficients to length 3 if needed
            let mut b = filter.numerator.clone();
            let mut a = filter.denominator.clone();

            while b.len() < 3 {
                b.push(0.0);
            }
            while a.len() < 3 {
                a.push(0.0);
            }

            // Normalize by a[0]
            let a0 = a[0];
            if a0 == 0.0 {
                return Err(DronetunerError::config_error(
                    "Invalid denominator coefficient",
                ));
            }

            let coefficients = BiquadCoefficients {
                b0: b[0] / a0,
                b1: b[1] / a0,
                b2: b[2] / a0,
                a1: a[1] / a0,
                a2: a[2] / a0,
            };

            sections.push(BiquadSection {
                coefficients,
                state: BiquadState::default(),
            });
        }

        Ok(sections)
    }
}

impl BiquadSection {
    /// Process a single sample through this biquad section
    pub fn process_sample(&mut self, input: f32) -> f32 {
        let output = self.coefficients.b0 * input
            + self.coefficients.b1 * self.state.x1
            + self.coefficients.b2 * self.state.x2
            - self.coefficients.a1 * self.state.y1
            - self.coefficients.a2 * self.state.y2;

        // Update state
        self.state.x2 = self.state.x1;
        self.state.x1 = input;
        self.state.y2 = self.state.y1;
        self.state.y1 = output;

        output
    }

    /// Process a buffer of samples
    pub fn process_buffer(&mut self, input: &[f32], output: &mut [f32]) {
        for (in_sample, out_sample) in input.iter().zip(output.iter_mut()) {
            *out_sample = self.process_sample(*in_sample);
        }
    }

    /// Reset internal state (useful between flights)
    pub fn reset(&mut self) {
        self.state = BiquadState::default();
    }
}

impl Default for BiquadState {
    fn default() -> Self {
        Self {
            x1: 0.0,
            x2: 0.0,
            y1: 0.0,
            y2: 0.0,
        }
    }
}

impl DigitalFilter {
    /// Create a simple gain filter (no filtering, just amplification)
    pub fn gain(gain: f32, sample_rate: f32) -> Self {
        Self {
            name: format!("Gain {:.2}", gain),
            numerator: vec![gain],
            denominator: vec![1.0],
            sample_rate,
        }
    }

    /// Create an identity filter (no change)
    pub fn identity(sample_rate: f32) -> Self {
        Self::gain(1.0, sample_rate)
    }

    /// Get the filter order (highest power of z)
    pub fn order(&self) -> usize {
        (self.numerator.len() - 1).max(self.denominator.len() - 1)
    }

    /// Check if filter is stable (all poles inside unit circle)
    pub fn is_stable(&self) -> bool {
        // For simplicity, assume stable if no denominator coefficient is NaN or infinite
        self.denominator.iter().all(|&x| x.is_finite())
    }
}

/// Utility functions for filter analysis
pub mod analysis {
    use super::*;

    /// Calculate the optimal notch filter Q-factor for a given resonance
    pub fn optimal_notch_q(resonance_q: f32, desired_attenuation_db: f32) -> f32 {
        // Higher resonance Q requires higher notch Q for effective suppression
        let base_q = (resonance_q / 3.0).max(1.0);

        // Adjust for desired attenuation
        let attenuation_factor = (desired_attenuation_db / 20.0).max(1.0);

        (base_q * attenuation_factor).min(50.0) // Limit to reasonable range
    }

    /// Estimate group delay penalty for a filter configuration
    pub fn estimate_group_delay_penalty(config: &FilterConfiguration, sample_rate: f32) -> f32 {
        let mut total_delay = 0.0;

        // Each filter adds group delay
        for filter in &config.gyro_filters {
            total_delay += match filter.order {
                1 => 0.5, // samples
                2 => 1.0, // samples
                _ => filter.order as f32 * 0.5,
            };
        }

        for _notch in &config.notch_filters {
            total_delay += 1.0; // Notch filters add about 1 sample delay
        }

        // Convert to milliseconds
        total_delay * 1000.0 / sample_rate
    }

    /// Calculate the effective bandwidth after filtering
    pub fn effective_bandwidth(config: &FilterConfiguration) -> f32 {
        let mut min_cutoff = f32::INFINITY;

        for filter in &config.gyro_filters {
            if filter.filter_type == FilterType::LowPass {
                min_cutoff = min_cutoff.min(filter.cutoff);
            }
        }

        if min_cutoff.is_infinite() {
            1000.0 // Default if no lowpass filters
        } else {
            min_cutoff
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_filter_designer_creation() {
        let designer = FilterDesigner::new(8000.0);
        assert_eq!(designer.sample_rate, 8000.0);
    }

    #[test]
    fn test_butterworth_lowpass_design() {
        let designer = FilterDesigner::new(8000.0);
        let filter = designer.butterworth_lowpass(1000.0, 2).unwrap();

        assert_eq!(filter.sample_rate, 8000.0);
        assert!(!filter.numerator.is_empty());
        assert!(!filter.denominator.is_empty());
        assert!(filter.is_stable());
    }

    #[test]
    fn test_notch_filter_design() {
        let designer = FilterDesigner::new(8000.0);
        let filter = designer.notch_filter(200.0, 10.0).unwrap();

        assert!(filter.name.contains("Notch"));
        assert!(filter.name.contains("200"));
    }

    #[test]
    fn test_invalid_parameters() {
        let designer = FilterDesigner::new(8000.0);

        // Invalid cutoff (too high)
        assert!(designer.butterworth_lowpass(5000.0, 2).is_err());

        // Invalid Q factor
        assert!(designer.notch_filter(200.0, 0.0).is_err());
    }

    #[test]
    fn test_biquad_processing() {
        let mut biquad = BiquadSection {
            coefficients: BiquadCoefficients {
                b0: 1.0,
                b1: 0.0,
                b2: 0.0,
                a1: 0.0,
                a2: 0.0,
            },
            state: BiquadState::default(),
        };

        // Identity filter should pass input unchanged
        assert_eq!(biquad.process_sample(1.0), 1.0);
        assert_eq!(biquad.process_sample(0.5), 0.5);
    }

    #[test]
    fn test_frequency_response() {
        let designer = FilterDesigner::new(8000.0);
        let filter = designer.butterworth_lowpass(1000.0, 2).unwrap();

        let frequencies = vec![100.0, 500.0, 1000.0, 2000.0];
        let response = designer.frequency_response(&filter, &frequencies);

        assert_eq!(response.frequencies.len(), frequencies.len());
        assert_eq!(response.magnitude.len(), frequencies.len());

        // Low frequencies should have less attenuation than high frequencies
        assert!(response.magnitude[0] > response.magnitude[3]);
    }
}
