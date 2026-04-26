//! PID step-response and gyro-characteristic analysis.
//!
//! Used by [`super::AnalysisEngine`] to produce per-axis PID recommendations.
//! Falls back to gyro-only heuristics when RC command data is unavailable.

use crate::domain::{Axis, PidConfiguration, PidRecommendation, PidTerm, Priority, TelemetryData};
use crate::error::{DronetunerError, Result};

/// Analyses telemetry to derive PID gain recommendations.
pub(super) struct PidAnalyzer {
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
    rise_time: f32,
    settling_time: f32,
    overshoot_percent: f32,
    oscillation_frequency: f32,
    damping_ratio: f32,
    steady_state_error: f32,
}

impl PidAnalyzer {
    pub(super) fn new() -> Self {
        Self {
            config: PidAnalyzerConfig::default(),
        }
    }

    pub(super) fn analyze(
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
            let reduction_percent = (avg_overshoot - self.config.overshoot_tolerance) / 50.0;
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
