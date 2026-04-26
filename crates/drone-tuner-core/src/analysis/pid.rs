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
    /// Absolute steady-state tracking error in deg/s, when the step was
    /// large enough and the command stayed put long enough to make the
    /// metric meaningful. `None` for transient stick movements where
    /// "steady state" was never reached.
    pub steady_state_error_dps: Option<f32>,
}

/// Step response performance metrics
#[derive(Debug, Clone)]
struct StepMetrics {
    rise_time: f32,
    settling_time: f32,
    overshoot_percent: f32,
    oscillation_frequency: f32,
    damping_ratio: f32,
    steady_state_error_dps: Option<f32>,
}

/// Hard upper bounds for PID gains. The analyzer never recommends values
/// above these and stops recommending increases once the current value is
/// within `headroom_skip_pct` of the cap. Tuned for modern Betaflight 4.x
/// on a 4S-6S 5" freestyle quad — adjust for racing or tinywhoops.
#[derive(Debug, Clone)]
struct PidLimits {
    p_max: f32,
    i_max: f32,
    d_max: f32,
    /// Skip recommending an increase if `current >= max * (1 - headroom_skip_pct)`.
    /// Prevents asymptotic recommendations that nudge gains by less than the
    /// FC's integer resolution.
    headroom_skip_pct: f32,
}

impl Default for PidLimits {
    fn default() -> Self {
        Self {
            p_max: 80.0,
            i_max: 180.0,
            d_max: 60.0,
            headroom_skip_pct: 0.10,
        }
    }
}

/// Minimum stick deflection (fraction of full RC range, [-1.0, 1.0]) for a
/// step to count as steady-state-capable. Below this the gyro never reaches
/// a sustainable rate and "steady-state error" is meaningless.
const SS_CAPABLE_MIN_STEP: f32 = 0.30;
/// Maximum allowed RC command drift across the analysis window for the
/// step to count as steady-state-capable. If the pilot moved the stick
/// again before the response settled, we can't tell command-tracking error
/// from a fresh transient.
const SS_CAPABLE_MAX_DRIFT: f32 = 0.10;
/// Minimum number of steady-state-capable responses we need before we
/// trust an averaged steady-state error figure enough to act on it.
const SS_MIN_VALID_RESPONSES: usize = 3;
/// Threshold in deg/s above which we recommend bumping I-term. 30 deg/s
/// is roughly 4-5% of typical full-stick rate (~600-700 deg/s) — small
/// enough to catch real bias, large enough to ignore measurement noise.
const SS_ERROR_RECOMMEND_DPS: f32 = 30.0;
/// Conservative I-term increase per iteration. The previous value of 20%
/// caused runaway recommendations because the FC's tune quickly lands
/// near (but not at) the SS-error threshold and the algorithm keeps
/// nudging. 10% lets each iteration's effect be visible in the next bbl.
const I_TERM_BUMP: f32 = 1.10;
/// Minimum absolute scale at which we treat an `rc_commands` axis as
/// having meaningful range. Below this we assume the pilot didn't touch
/// the stick during the log and skip step analysis for that axis.
const RC_NORMALIZE_MIN_SCALE: f32 = 5.0;
/// Default Betaflight rcCommand range. Roll/pitch/yaw are signed ints
/// scaled to roughly +/- 500 (post-deadband, pre-rates). We use the
/// observed maximum absolute value in the log as the scale instead of
/// hard-coding 500, so the analyzer also works on logs that already came
/// in normalized — see `normalize_rc_axis`.
const RC_NORMALIZE_FALLBACK: f32 = 500.0;

/// Normalize one axis of rc_commands to roughly [-1.0, 1.0] using the
/// 99th-percentile absolute value as the scale. Returns the scale used so
/// callers can sanity-check it. Robust to the two conventions we see in
/// the wild: raw Betaflight rcCommand ([-500, 500]) and pre-normalized
/// floats ([-1, 1]).
fn normalize_rc_axis(rc: &[f32]) -> (Vec<f32>, f32) {
    if rc.is_empty() {
        return (Vec::new(), 1.0);
    }
    let mut abs_vals: Vec<f32> = rc.iter().map(|v| v.abs()).collect();
    // Partial sort by ordered_cmp so NaNs don't poison the sort.
    abs_vals.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let p99_idx = ((abs_vals.len() as f32) * 0.99) as usize;
    let p99 = abs_vals
        .get(p99_idx.min(abs_vals.len().saturating_sub(1)))
        .copied()
        .unwrap_or(0.0);
    // If the log barely moves the stick we don't have signal — fall back
    // to the Betaflight scale so step detection doesn't divide by ~0.
    let scale = if p99 < RC_NORMALIZE_MIN_SCALE {
        RC_NORMALIZE_FALLBACK
    } else {
        p99
    };
    let normalized = rc.iter().map(|v| v / scale).collect();
    (normalized, scale)
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

            // Normalize rc_commands to ~[-1, 1] before step detection so
            // thresholds (min step, max drift, SS-capable cutoff) are unit-
            // independent of whether the bbl came in raw Betaflight units
            // ([-500, 500]) or already normalized.
            let (roll_rc, roll_scale) = normalize_rc_axis(&telemetry.rc_commands.roll);
            let (pitch_rc, pitch_scale) = normalize_rc_axis(&telemetry.rc_commands.pitch);
            let (yaw_rc, yaw_scale) = normalize_rc_axis(&telemetry.rc_commands.yaw);
            tracing::debug!(
                "RC normalization scales: roll={:.1} pitch={:.1} yaw={:.1}",
                roll_scale,
                pitch_scale,
                yaw_scale
            );

            // Detect step responses for each axis
            let roll_responses = self.detect_step_responses(
                &roll_rc,
                &telemetry.gyro.x,
                telemetry.sample_rate,
                Axis::Roll,
            )?;

            let pitch_responses = self.detect_step_responses(
                &pitch_rc,
                &telemetry.gyro.y,
                telemetry.sample_rate,
                Axis::Pitch,
            )?;

            let yaw_responses = self.detect_step_responses(
                &yaw_rc,
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

        // Real pilot stick movements happen over ~50-200ms. At 3205Hz that's
        // hundreds of samples — a single-sample delta detector would never
        // fire on a gradual stick move and instead picks up only quantization
        // noise. Use a windowed detector: at each position, look at the RC
        // change between `pre_window` ago and `post_window` ahead. If the
        // pre-window was steady, the post-window is steady, and the gap
        // between them is large, it's a real step.
        let pre_window = ((0.05 * sample_rate) as usize).max(5); // 50ms history
        let post_window = ((0.10 * sample_rate) as usize).max(10); // 100ms forward
        let refractory = ((0.30 * sample_rate) as usize).max(30); // 300ms cooldown
        let min_step_size = 0.10; // 10% of full stick deflection
        let max_pre_jitter = 0.05; // pre-step stability tolerance
        let max_post_jitter = 0.10; // post-step settle tolerance

        // Sliding mean+range over a window — cheap stability check.
        let window_range = |start: usize, end: usize| -> (f32, f32) {
            let slice = &rc_commands[start..end];
            let mean = slice.iter().sum::<f32>() / slice.len() as f32;
            let max_dev = slice
                .iter()
                .map(|v| (v - mean).abs())
                .fold(0.0_f32, f32::max);
            (mean, max_dev)
        };

        let mut last_step_end = 0;
        let mut i = pre_window;
        while i + post_window < rc_commands.len() {
            // Skip until we're past the previous step's refractory period.
            if i < last_step_end {
                i += 1;
                continue;
            }

            let (pre_mean, pre_jitter) = window_range(i - pre_window, i);
            let (post_mean, post_jitter) = window_range(i, i + post_window);
            let step_size = (post_mean - pre_mean).abs();

            if step_size > min_step_size
                && pre_jitter < max_pre_jitter
                && post_jitter < max_post_jitter
            {
                let response = self.analyze_step_response(
                    i,
                    i + post_window,
                    rc_commands,
                    gyro_response,
                    sample_rate,
                    axis.clone(),
                )?;
                if let Some(resp) = response {
                    responses.push(resp);
                }
                last_step_end = i + refractory;
                i += refractory;
            } else {
                i += 1;
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
        // Recompute the windowed pre/post means so the step magnitude here
        // matches what the detector saw (single-sample deltas would pick
        // up quantization noise instead of the actual stick movement).
        let pre_w = ((0.05 * sample_rate) as usize).max(5).min(step_start);
        let post_w = ((0.10 * sample_rate) as usize).max(10);
        let pre_slice = &rc_commands[step_start - pre_w..step_start];
        let post_end = (step_start + post_w).min(rc_commands.len());
        let post_slice = &rc_commands[step_start..post_end];
        let initial_command = pre_slice.iter().sum::<f32>() / pre_slice.len() as f32;
        let step_command = post_slice.iter().sum::<f32>() / post_slice.len() as f32;
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

        // For steady-state error to be a real measurement we need (a) a step
        // big enough that the gyro can plausibly reach a sustained rate and
        // (b) the post-step command stayed put. The detector already
        // enforces a stability bound on the analysis window; here we just
        // require a stricter step magnitude and that the command stayed
        // close to `step_command` (the post-step plateau) for the full
        // settling window.
        let cmd_window_end = (step_start + analysis_window).min(rc_commands.len());
        let cmd_window = &rc_commands[step_start..cmd_window_end];
        let max_cmd_drift = cmd_window
            .iter()
            .map(|&c| (c - step_command).abs())
            .fold(0.0f32, f32::max);
        let is_ss_capable =
            command_change.abs() >= SS_CAPABLE_MIN_STEP && max_cmd_drift <= SS_CAPABLE_MAX_DRIFT;

        // Calculate performance metrics
        let metrics = self.calculate_step_metrics(
            response_window,
            baseline_gyro,
            expected_response,
            sample_rate,
            is_ss_capable,
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
            steady_state_error_dps: metrics.steady_state_error_dps,
        }))
    }

    /// Calculate step response performance metrics
    fn calculate_step_metrics(
        &self,
        response: &[f32],
        baseline: f32,
        expected_final: f32,
        sample_rate: f32,
        is_ss_capable: bool,
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

        // Steady-state error: the absolute gap between where the gyro ended
        // up and where it should have ended up, in deg/s. We only emit it
        // for responses where the step was big enough and the command was
        // held steady — see SS_CAPABLE_* constants. Averaging the last
        // ~50ms of the window gives a tighter "where did it actually
        // settle" reading than the previous 10-sample tail.
        let steady_state_error_dps = if is_ss_capable {
            let tail_len = ((sample_rate * 0.05) as usize).clamp(5, response.len());
            let tail_start = response.len() - tail_len;
            let steady_state_value = response[tail_start..].iter().sum::<f32>() / tail_len as f32;
            Some((steady_state_value - target_value).abs())
        } else {
            None
        };

        Ok(StepMetrics {
            rise_time,
            settling_time,
            overshoot_percent,
            oscillation_frequency,
            damping_ratio,
            steady_state_error_dps,
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

        let limits = PidLimits::default();
        let i_headroom_floor = limits.i_max * (1.0 - limits.headroom_skip_pct);

        // Check for persistent bias in error. Use the same conservative
        // bump (+10%) and absolute cap as the step-response path so the
        // two paths can't disagree on what "safe" means.
        let error_mean = error_data.iter().sum::<f32>() / error_data.len() as f32;
        if error_mean.abs() > 2.0 && current_pid.i < i_headroom_floor {
            let proposed = (current_pid.i * I_TERM_BUMP).min(limits.i_max);
            if proposed - current_pid.i >= 1.0 {
                recommendations.push(PidRecommendation {
                    axis: axis.clone(),
                    term: PidTerm::I,
                    current_value: current_pid.i,
                    recommended_value: proposed,
                    reason: format!(
                        "Increase I-term to eliminate error bias (bias: {:.2})",
                        error_mean
                    ),
                    priority: Priority::High,
                });
            }
        }

        // RMS error driven I-bump only fires if the bias check above didn't
        // (otherwise we'd double-recommend the same axis) and only when
        // there's headroom under the cap.
        let already_recommended_i = recommendations.iter().any(|r| matches!(r.term, PidTerm::I));
        if !already_recommended_i && rms_error > 5.0 && current_pid.i < i_headroom_floor {
            let proposed = (current_pid.i * I_TERM_BUMP).min(limits.i_max);
            if proposed - current_pid.i >= 1.0 {
                recommendations.push(PidRecommendation {
                    axis: axis.clone(),
                    term: PidTerm::I,
                    current_value: current_pid.i,
                    recommended_value: proposed,
                    reason: format!("Increase I-term: persistent RMS error {:.2}", rms_error),
                    priority: Priority::Medium,
                });
            }
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

        let limits = PidLimits::default();
        let p_headroom_floor = limits.p_max * (1.0 - limits.headroom_skip_pct);
        let d_headroom_floor = limits.d_max * (1.0 - limits.headroom_skip_pct);

        // Analyze P-term based on overshoot and rise time
        if avg_overshoot > self.config.overshoot_tolerance {
            let reduction_percent = (avg_overshoot - self.config.overshoot_tolerance) / 50.0;
            let proposed = current_pid.p * (1.0 - reduction_percent.min(0.3)); // Max 30% reduction
            if current_pid.p - proposed >= 1.0 {
                recommendations.push(PidRecommendation {
                    axis: axis.clone(),
                    term: PidTerm::P,
                    current_value: current_pid.p,
                    recommended_value: proposed,
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
            }
        } else if avg_rise_time > 0.15 && avg_overshoot < 5.0 && current_pid.p < p_headroom_floor {
            // Slow rise time with little overshoot suggests P could be increased.
            let proposed = (current_pid.p * 1.1).min(limits.p_max); // 10% increase, capped
            if proposed - current_pid.p >= 1.0 {
                recommendations.push(PidRecommendation {
                    axis: axis.clone(),
                    term: PidTerm::P,
                    current_value: current_pid.p,
                    recommended_value: proposed,
                    reason: format!(
                        "Increase P-term to improve responsiveness (rise time: {:.3}s)",
                        avg_rise_time
                    ),
                    priority: Priority::Low,
                });
            }
        }

        // Analyze I-term based on steady-state error.
        //
        // We average only the responses that were large enough and held
        // steady enough to give a meaningful steady-state reading. If we
        // don't have at least SS_MIN_VALID_RESPONSES of those, we say
        // nothing — the previous behaviour of recommending I-bumps from
        // small stick movements gave a false signal that compounded across
        // tuning iterations.
        let ss_capable: Vec<f32> = responses
            .iter()
            .filter_map(|r| r.steady_state_error_dps)
            .collect();
        if ss_capable.len() >= SS_MIN_VALID_RESPONSES {
            let avg_ss_error_dps = ss_capable.iter().sum::<f32>() / ss_capable.len() as f32;
            let i_headroom_floor = limits.i_max * (1.0 - limits.headroom_skip_pct);
            if avg_ss_error_dps > SS_ERROR_RECOMMEND_DPS && current_pid.i < i_headroom_floor {
                let proposed = (current_pid.i * I_TERM_BUMP).min(limits.i_max);
                // Only emit if the change is at least one integer FC unit;
                // otherwise the recommendation is invisible to the FC.
                if proposed - current_pid.i >= 1.0 {
                    recommendations.push(PidRecommendation {
                        axis: axis.clone(),
                        term: PidTerm::I,
                        current_value: current_pid.i,
                        recommended_value: proposed,
                        reason: format!(
                            "Increase I-term: {:.0} deg/s steady-state tracking error across {} valid step responses",
                            avg_ss_error_dps,
                            ss_capable.len()
                        ),
                        priority: Priority::Medium,
                    });
                }
            }
        }

        // Analyze D-term based on oscillations and damping
        if avg_oscillation_freq > 10.0 && avg_damping < 0.5 && current_pid.d < d_headroom_floor {
            let proposed = (current_pid.d * 1.3).min(limits.d_max); // 30% increase, capped
            if proposed - current_pid.d >= 1.0 {
                recommendations.push(PidRecommendation {
                    axis: axis.clone(),
                    term: PidTerm::D,
                    current_value: current_pid.d,
                    recommended_value: proposed,
                    reason: format!(
                        "Increase D-term to dampen oscillations ({:.1} Hz, damping: {:.2})",
                        avg_oscillation_freq, avg_damping
                    ),
                    priority: Priority::Medium,
                });
            }
        } else if avg_settling_time > 0.5 && avg_oscillation_freq < 5.0 {
            // Long settling time might indicate too much D-term
            let proposed = current_pid.d * 0.8; // 20% decrease
            if current_pid.d - proposed >= 1.0 {
                recommendations.push(PidRecommendation {
                    axis: axis.clone(),
                    term: PidTerm::D,
                    current_value: current_pid.d,
                    recommended_value: proposed,
                    reason: format!(
                        "Reduce D-term to improve settling time ({:.3}s)",
                        avg_settling_time
                    ),
                    priority: Priority::Low,
                });
            }
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
