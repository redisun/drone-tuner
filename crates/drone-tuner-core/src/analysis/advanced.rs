//! Heuristics for advanced tuning parameters.
//!
//! Examines the parsed `AdvancedTuningConfig` (blackbox config headers),
//! the current PID table, and actual flight telemetry to recommend changes
//! to parameters beyond basic PID and filter settings: vbat sag
//! compensation, dynamic idle, D Max, TPA, feedforward, and thrust
//! linearization.
//!
//! TPA analysis is data-driven when sufficient telemetry is available,
//! falling back to config-only checks when TPA is explicitly disabled.

use crate::domain::{
    AdvancedParameter, AdvancedRecommendation, AdvancedTuningConfig, Axis, FeedforwardParam,
    HardwareConfiguration, PidConfiguration, PidRecommendation, PidTerm, Priority, TelemetryData,
};

/// Analyses hardware configuration to produce advanced tuning recommendations.
pub(super) struct AdvancedAnalyzer;

impl AdvancedAnalyzer {
    pub(super) fn new() -> Self {
        Self
    }

    pub(super) fn analyze(
        &self,
        hardware: &HardwareConfiguration,
        telemetry: &TelemetryData,
        pid_recs: &[PidRecommendation],
    ) -> Vec<AdvancedRecommendation> {
        let adv = match &hardware.advanced_tuning {
            Some(a) => a,
            None => return Vec::new(),
        };

        let mut recs = Vec::new();

        recs.extend(self.check_vbat_sag(adv));
        recs.extend(self.check_dynamic_idle(adv));
        recs.extend(self.check_thrust_linearization(adv));
        recs.extend(self.check_d_max(adv, &hardware.pid_config));
        // Config-based TPA check fires when TPA rate is explicitly 0.
        recs.extend(self.check_tpa(adv, &hardware.pid_config));
        // Data-driven TPA check fires when TPA is already set but may need
        // adjustment based on actual gyro noise across throttle bands.
        recs.extend(self.check_tpa_from_telemetry(adv, telemetry));
        recs.extend(self.check_feedforward(adv));
        // Simplified tuning slider hints (Betaflight 4.3+).
        recs.extend(self.check_sliders(adv, pid_recs));

        recs
    }

    fn check_vbat_sag(&self, adv: &AdvancedTuningConfig) -> Option<AdvancedRecommendation> {
        if adv.dshot_bidir != Some(true) {
            return None;
        }
        let current = adv.vbat_sag_compensation.unwrap_or(0);
        if current != 0 {
            return None;
        }
        Some(AdvancedRecommendation {
            parameter: AdvancedParameter::VbatSagCompensation {
                current,
                recommended: 100,
            },
            reason: "Bidirectional DShot is active \u{2014} enable vbat sag compensation \
                     for consistent throttle response across battery voltage range"
                .into(),
            priority: Priority::Low,
        })
    }

    fn check_dynamic_idle(&self, adv: &AdvancedTuningConfig) -> Option<AdvancedRecommendation> {
        let protocol = adv.motor_protocol?;
        if !(5..=8).contains(&protocol) {
            return None;
        }
        // None means we can't determine the current state -- skip.
        let current_rpm = adv.dynamic_idle_min_rpm?;
        if current_rpm != 0 {
            return None;
        }
        Some(AdvancedRecommendation {
            parameter: AdvancedParameter::DynamicIdle {
                current_rpm,
                recommended_rpm: 30,
            },
            reason: "DShot motor protocol detected \u{2014} enable dynamic idle \
                     for better prop wash handling and desync prevention"
                .into(),
            priority: Priority::Low,
        })
    }

    fn check_thrust_linearization(
        &self,
        adv: &AdvancedTuningConfig,
    ) -> Option<AdvancedRecommendation> {
        let protocol = adv.motor_protocol?;
        if !(5..=8).contains(&protocol) {
            return None;
        }
        let current = adv.thrust_linearization?;
        if current != 0 {
            return None;
        }
        Some(AdvancedRecommendation {
            parameter: AdvancedParameter::ThrustLinearization {
                current,
                recommended: 25,
            },
            reason: "Enable thrust linearization for more linear throttle response".into(),
            priority: Priority::Low,
        })
    }

    fn check_d_max(
        &self,
        adv: &AdvancedTuningConfig,
        pid: &PidConfiguration,
    ) -> Vec<AdvancedRecommendation> {
        let mut recs = Vec::new();

        // Check d_max_gain first -- if it's explicitly 0, recommend the BF
        // default regardless of the per-axis d_min situation.
        if adv.d_max_gain == Some(0) {
            recs.push(AdvancedRecommendation {
                parameter: AdvancedParameter::Feedforward {
                    // Re-use a lightweight variant -- d_max_gain doesn't have
                    // its own AdvancedParameter variant. We carry the info in
                    // the reason string and the priority.
                    param: FeedforwardParam::Boost, // placeholder
                    current: 0,
                    recommended: 37,
                },
                reason: "D Max gain is 0 (dynamic D range disabled) \u{2014} \
                         recommend restoring Betaflight default of 37"
                    .into(),
                priority: Priority::Low,
            });
        }

        let d_min = match &adv.d_min {
            Some(dm) => dm,
            None => return recs,
        };

        // Roll and pitch only -- yaw D is conventionally 0.
        for (axis, d_min_val, d_val) in [
            (Axis::Roll, d_min.roll, pid.roll.d),
            (Axis::Pitch, d_min.pitch, pid.pitch.d),
        ] {
            let d_max_u8 = d_val as u8;
            if d_min_val == d_max_u8 && d_max_u8 > 0 {
                // d_min == D value: dynamic range is effectively disabled.
                // Recommend setting d_min to ~65% of D.
                #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
                let recommended_d_min = (d_val * 0.65).round() as u8;
                recs.push(AdvancedRecommendation {
                    parameter: AdvancedParameter::DMax {
                        axis,
                        current_d_min: d_min_val,
                        current_d_max: d_max_u8,
                        recommended_d_min,
                        recommended_d_max: d_max_u8,
                    },
                    reason: format!(
                        "D Min equals D Max ({}) \u{2014} the D Max dynamic range is \
                         effectively disabled. Lower D Min to {} to allow the flight \
                         controller to reduce D during calm flight",
                        d_max_u8, recommended_d_min
                    ),
                    priority: Priority::Low,
                });
            }
        }

        recs
    }

    fn check_tpa(
        &self,
        adv: &AdvancedTuningConfig,
        pid: &PidConfiguration,
    ) -> Option<AdvancedRecommendation> {
        let tpa = adv.tpa.as_ref()?;
        if tpa.rate != 0 {
            return None;
        }
        // Only recommend if D gains are high enough to benefit from TPA.
        let max_d = pid.roll.d.max(pid.pitch.d);
        if max_d <= 25.0 {
            return None;
        }
        Some(AdvancedRecommendation {
            parameter: AdvancedParameter::Tpa {
                current_rate: tpa.rate,
                current_breakpoint: tpa.breakpoint,
                recommended_rate: 65,
                recommended_breakpoint: 1350,
            },
            reason: format!(
                "TPA is disabled but D gains are {:.0} \u{2014} enable TPA to \
                 attenuate D at high throttle and prevent hot-motor oscillations",
                max_d
            ),
            priority: Priority::Low,
        })
    }

    /// Minimum number of samples required in each throttle band before we
    /// consider the data sufficient for a TPA recommendation.
    const MIN_BAND_SAMPLES: usize = 500;
    /// Throttle threshold separating low/mid from high throttle band.
    const HIGH_THROTTLE_THRESHOLD: f32 = 0.6;
    /// Ratio above which high-throttle noise is considered significantly
    /// worse than low/mid-throttle noise, warranting more TPA.
    const NOISE_INCREASE_RATIO: f32 = 1.5;
    /// Ratio below which high-throttle noise is considered similar to
    /// low/mid-throttle noise, suggesting TPA may be excessive.
    const NOISE_FLAT_RATIO: f32 = 1.1;

    /// Data-driven TPA analysis: compare gyro RMS between low/mid and high
    /// throttle bands to determine whether TPA needs adjustment.
    ///
    /// This fires only when TPA is already configured (rate > 0). When TPA
    /// rate is 0, the config-based `check_tpa` handles that case.
    fn check_tpa_from_telemetry(
        &self,
        adv: &AdvancedTuningConfig,
        telemetry: &TelemetryData,
    ) -> Option<AdvancedRecommendation> {
        let tpa = adv.tpa.as_ref()?;

        // This method handles the case where TPA is already set.
        // The config-based check_tpa handles rate == 0.
        if tpa.rate == 0 {
            return None;
        }

        let throttle = &telemetry.rc_commands.throttle;
        let gyro_x = &telemetry.gyro.x;
        let gyro_y = &telemetry.gyro.y;

        // We need throttle and gyro arrays to be the same length.
        let n = throttle.len().min(gyro_x.len()).min(gyro_y.len());
        if n == 0 {
            return None;
        }

        // Split into throttle bands and accumulate squared gyro values.
        let mut low_mid_sum_sq: f64 = 0.0;
        let mut low_mid_count: usize = 0;
        let mut high_sum_sq: f64 = 0.0;
        let mut high_count: usize = 0;

        for i in 0..n {
            // Combined roll + pitch noise: sum of squares of both axes.
            let noise = f64::from(gyro_x[i]) * f64::from(gyro_x[i])
                + f64::from(gyro_y[i]) * f64::from(gyro_y[i]);
            if throttle[i] < Self::HIGH_THROTTLE_THRESHOLD {
                low_mid_sum_sq += noise;
                low_mid_count += 1;
            } else {
                high_sum_sq += noise;
                high_count += 1;
            }
        }

        // Need sufficient data in both bands.
        if low_mid_count < Self::MIN_BAND_SAMPLES || high_count < Self::MIN_BAND_SAMPLES {
            return None;
        }

        // RMS for each band.
        let low_mid_rms = (low_mid_sum_sq / low_mid_count as f64).sqrt();
        let high_rms = (high_sum_sq / high_count as f64).sqrt();

        // Avoid division by zero.
        if low_mid_rms < f64::EPSILON {
            return None;
        }

        let ratio = high_rms / low_mid_rms;

        if ratio > f64::from(Self::NOISE_INCREASE_RATIO) {
            // High-throttle noise is significantly worse -- TPA needs to be
            // stronger to attenuate D at high throttle.
            if tpa.rate < 50 {
                return Some(AdvancedRecommendation {
                    parameter: AdvancedParameter::Tpa {
                        current_rate: tpa.rate,
                        current_breakpoint: tpa.breakpoint,
                        recommended_rate: 65,
                        recommended_breakpoint: if tpa.breakpoint > 1400 {
                            1250
                        } else {
                            tpa.breakpoint
                        },
                    },
                    reason: format!(
                        "Gyro noise at high throttle is {ratio:.1}x higher than at \
                         low/mid throttle \u{2014} increase TPA rate from {} to 65 \
                         to reduce D-term oscillations at high throttle",
                        tpa.rate
                    ),
                    priority: Priority::Low,
                });
            }
            if tpa.breakpoint > 1400 {
                return Some(AdvancedRecommendation {
                    parameter: AdvancedParameter::Tpa {
                        current_rate: tpa.rate,
                        current_breakpoint: tpa.breakpoint,
                        recommended_rate: tpa.rate,
                        recommended_breakpoint: 1250,
                    },
                    reason: format!(
                        "Gyro noise at high throttle is {ratio:.1}x higher than at \
                         low/mid throttle \u{2014} lower TPA breakpoint from {} to \
                         1250 so attenuation kicks in earlier",
                        tpa.breakpoint
                    ),
                    priority: Priority::Low,
                });
            }
        } else if ratio < f64::from(Self::NOISE_FLAT_RATIO) && tpa.rate > 80 {
            // Noise is flat across throttle range -- excessive TPA is
            // needlessly reducing flight performance at high throttle.
            return Some(AdvancedRecommendation {
                parameter: AdvancedParameter::Tpa {
                    current_rate: tpa.rate,
                    current_breakpoint: tpa.breakpoint,
                    recommended_rate: 65,
                    recommended_breakpoint: tpa.breakpoint,
                },
                reason: format!(
                    "Gyro noise is similar across throttle range (ratio {ratio:.2}x) \
                     \u{2014} reduce TPA rate from {} to 65 to recover high-throttle \
                     authority",
                    tpa.rate
                ),
                priority: Priority::Low,
            });
        }

        None
    }

    /// Emit slider-aware hints when Betaflight simplified tuning is active.
    ///
    /// In RP mode (`mode == 2`) the pilot adjusts sliders in the
    /// Configurator rather than raw PID values. When sliders are at their
    /// defaults we can suggest small bumps that correspond to the PID
    /// changes detected by the main PID analyzer.
    fn check_sliders(
        &self,
        adv: &AdvancedTuningConfig,
        pid_recs: &[PidRecommendation],
    ) -> Vec<AdvancedRecommendation> {
        let st = match &adv.simplified_tuning {
            Some(s) if s.mode == 2 => s,
            _ => return Vec::new(),
        };

        let mut recs = Vec::new();

        // --- Master Multiplier at default (100) ---
        // If any PID rec suggests increasing P on roll or pitch, the user
        // can achieve that (and scale the whole tune proportionally) by
        // bumping Master Multiplier from 100 to 110.
        if st.master_multiplier == Some(100) {
            let has_p_increase = pid_recs
                .iter()
                .any(|r| r.term == PidTerm::P && r.recommended_value > r.current_value);
            if has_p_increase {
                recs.push(AdvancedRecommendation {
                    parameter: AdvancedParameter::SliderHint {
                        slider_name: "Master Multiplier".into(),
                        current_value: 100,
                        suggested_value: 110,
                    },
                    reason: "PID analysis suggests higher P gains \u{2014} \
                             bumping the Master Multiplier slider from 1.00 to 1.10 \
                             scales all PID terms proportionally and is the \
                             recommended way to increase authority when using \
                             simplified tuning"
                        .into(),
                    priority: Priority::Low,
                });
            }
        }

        // --- D gain slider at default (100) and PID analysis detected
        //     slow settling (D increase recommended) ---
        if st.d_gain == Some(100) {
            let has_d_increase = pid_recs
                .iter()
                .any(|r| r.term == PidTerm::D && r.recommended_value > r.current_value);
            if has_d_increase {
                recs.push(AdvancedRecommendation {
                    parameter: AdvancedParameter::SliderHint {
                        slider_name: "D Gain".into(),
                        current_value: 100,
                        suggested_value: 110,
                    },
                    reason: "PID analysis suggests higher D gains for faster settling \
                             \u{2014} increase the D Gain slider from 1.00 to 1.10 \
                             instead of editing raw PID values when using simplified \
                             tuning"
                        .into(),
                    priority: Priority::Low,
                });
            }
        }

        recs
    }

    fn check_feedforward(&self, adv: &AdvancedTuningConfig) -> Option<AdvancedRecommendation> {
        let ff = adv.feedforward.as_ref()?;
        if ff.jitter_factor != Some(0) {
            return None;
        }
        Some(AdvancedRecommendation {
            parameter: AdvancedParameter::Feedforward {
                param: FeedforwardParam::JitterFactor,
                current: 0,
                recommended: 7,
            },
            reason: "Feedforward jitter factor is 0 \u{2014} set to 7 (Betaflight default) \
                     to reduce RC link jitter in the feedforward path"
                .into(),
            priority: Priority::Low,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::domain::{
        AdvancedTuningConfig, Axis, FeedforwardConfig, HardwareConfiguration, MotorTrace,
        PerAxisU8, PidErrorTrace, PidRecommendation, PidTerm, RcCommandTrace, SimplifiedTuning,
        TimeSeriesVector3, TpaAdvancedConfig,
    };

    fn hw_with_advanced(adv: AdvancedTuningConfig) -> HardwareConfiguration {
        let mut hw = HardwareConfiguration::test_default();
        hw.advanced_tuning = Some(adv);
        hw
    }

    fn default_adv() -> AdvancedTuningConfig {
        AdvancedTuningConfig::default()
    }

    /// Empty telemetry -- not enough data for the telemetry-based TPA
    /// check to fire, so existing config-only tests remain unchanged.
    fn empty_telemetry() -> TelemetryData {
        TelemetryData {
            sample_rate: 4000.0,
            gyro: TimeSeriesVector3 {
                x: Vec::new(),
                y: Vec::new(),
                z: Vec::new(),
            },
            accel: TimeSeriesVector3 {
                x: Vec::new(),
                y: Vec::new(),
                z: Vec::new(),
            },
            motor: vec![MotorTrace {
                motor_id: 1,
                values: Vec::new(),
            }],
            pid_error: PidErrorTrace {
                roll: Vec::new(),
                pitch: Vec::new(),
                yaw: Vec::new(),
            },
            rc_commands: RcCommandTrace {
                roll: Vec::new(),
                pitch: Vec::new(),
                yaw: Vec::new(),
                throttle: Vec::new(),
            },
            loop_time_variance: 0.0,
            cpu_load: Vec::new(),
        }
    }

    /// Build telemetry with distinct noise levels in the two throttle bands.
    ///
    /// `low_mid_noise` is the gyro amplitude for throttle < 0.6,
    /// `high_noise` is the gyro amplitude for throttle >= 0.6.
    /// Each band gets `samples_per_band` samples.
    fn telemetry_with_noise(
        low_mid_noise: f32,
        high_noise: f32,
        samples_per_band: usize,
    ) -> TelemetryData {
        let n = samples_per_band * 2;
        let mut throttle = Vec::with_capacity(n);
        let mut gyro_x = Vec::with_capacity(n);
        let mut gyro_y = Vec::with_capacity(n);

        // First half: low/mid throttle at 0.4
        for _ in 0..samples_per_band {
            throttle.push(0.4);
            gyro_x.push(low_mid_noise);
            gyro_y.push(low_mid_noise);
        }
        // Second half: high throttle at 0.8
        for _ in 0..samples_per_band {
            throttle.push(0.8);
            gyro_x.push(high_noise);
            gyro_y.push(high_noise);
        }

        TelemetryData {
            sample_rate: 4000.0,
            gyro: TimeSeriesVector3 {
                x: gyro_x,
                y: gyro_y,
                z: vec![0.0; n],
            },
            accel: TimeSeriesVector3 {
                x: vec![0.0; n],
                y: vec![0.0; n],
                z: vec![0.0; n],
            },
            motor: vec![MotorTrace {
                motor_id: 1,
                values: vec![0.5; n],
            }],
            pid_error: PidErrorTrace {
                roll: vec![0.0; n],
                pitch: vec![0.0; n],
                yaw: vec![0.0; n],
            },
            rc_commands: RcCommandTrace {
                roll: vec![0.0; n],
                pitch: vec![0.0; n],
                yaw: vec![0.0; n],
                throttle,
            },
            loop_time_variance: 0.0,
            cpu_load: Vec::new(),
        }
    }

    // ---------- vbat sag compensation ----------

    #[test]
    fn vbat_sag_recommended_when_bidir_active_and_comp_zero() {
        let mut adv = default_adv();
        adv.dshot_bidir = Some(true);
        adv.vbat_sag_compensation = Some(0);
        let hw = hw_with_advanced(adv);
        let tel = empty_telemetry();
        let recs = AdvancedAnalyzer::new().analyze(&hw, &tel, &[]);

        let vbat = recs
            .iter()
            .find(|r| matches!(r.parameter, AdvancedParameter::VbatSagCompensation { .. }));
        assert!(vbat.is_some(), "should recommend vbat sag comp");
        if let AdvancedParameter::VbatSagCompensation { recommended, .. } = vbat.unwrap().parameter
        {
            assert_eq!(recommended, 100);
        }
    }

    #[test]
    fn vbat_sag_recommended_when_bidir_active_and_comp_none() {
        let mut adv = default_adv();
        adv.dshot_bidir = Some(true);
        adv.vbat_sag_compensation = None;
        let hw = hw_with_advanced(adv);
        let tel = empty_telemetry();
        let recs = AdvancedAnalyzer::new().analyze(&hw, &tel, &[]);

        assert!(
            recs.iter()
                .any(|r| matches!(r.parameter, AdvancedParameter::VbatSagCompensation { .. })),
            "should recommend vbat sag comp when field is None"
        );
    }

    #[test]
    fn vbat_sag_not_recommended_when_already_set() {
        let mut adv = default_adv();
        adv.dshot_bidir = Some(true);
        adv.vbat_sag_compensation = Some(80);
        let hw = hw_with_advanced(adv);
        let tel = empty_telemetry();
        let recs = AdvancedAnalyzer::new().analyze(&hw, &tel, &[]);

        assert!(
            !recs
                .iter()
                .any(|r| matches!(r.parameter, AdvancedParameter::VbatSagCompensation { .. })),
            "should not recommend when already non-zero"
        );
    }

    #[test]
    fn vbat_sag_not_recommended_without_bidir() {
        let mut adv = default_adv();
        adv.dshot_bidir = Some(false);
        adv.vbat_sag_compensation = Some(0);
        let hw = hw_with_advanced(adv);
        let tel = empty_telemetry();
        let recs = AdvancedAnalyzer::new().analyze(&hw, &tel, &[]);

        assert!(
            !recs
                .iter()
                .any(|r| matches!(r.parameter, AdvancedParameter::VbatSagCompensation { .. })),
            "bidir off => no vbat sag rec"
        );
    }

    // ---------- dynamic idle ----------

    #[test]
    fn dynamic_idle_recommended_when_dshot_and_zero() {
        let mut adv = default_adv();
        adv.motor_protocol = Some(6); // DShot600
        adv.dynamic_idle_min_rpm = Some(0);
        let hw = hw_with_advanced(adv);
        let tel = empty_telemetry();
        let recs = AdvancedAnalyzer::new().analyze(&hw, &tel, &[]);

        let idle = recs
            .iter()
            .find(|r| matches!(r.parameter, AdvancedParameter::DynamicIdle { .. }));
        assert!(idle.is_some(), "should recommend dynamic idle");
        if let AdvancedParameter::DynamicIdle {
            recommended_rpm, ..
        } = idle.unwrap().parameter
        {
            assert_eq!(recommended_rpm, 30);
        }
    }

    #[test]
    fn dynamic_idle_skipped_when_none() {
        let mut adv = default_adv();
        adv.motor_protocol = Some(6);
        adv.dynamic_idle_min_rpm = None; // can't determine current state
        let hw = hw_with_advanced(adv);
        let tel = empty_telemetry();
        let recs = AdvancedAnalyzer::new().analyze(&hw, &tel, &[]);

        assert!(
            !recs
                .iter()
                .any(|r| matches!(r.parameter, AdvancedParameter::DynamicIdle { .. })),
            "should skip when current state is unknown"
        );
    }

    #[test]
    fn dynamic_idle_skipped_when_not_dshot() {
        let mut adv = default_adv();
        adv.motor_protocol = Some(2); // PWM
        adv.dynamic_idle_min_rpm = Some(0);
        let hw = hw_with_advanced(adv);
        let tel = empty_telemetry();
        let recs = AdvancedAnalyzer::new().analyze(&hw, &tel, &[]);

        assert!(
            !recs
                .iter()
                .any(|r| matches!(r.parameter, AdvancedParameter::DynamicIdle { .. })),
            "non-DShot => no dynamic idle rec"
        );
    }

    // ---------- thrust linearization ----------

    #[test]
    fn thrust_lin_recommended_when_dshot_and_zero() {
        let mut adv = default_adv();
        adv.motor_protocol = Some(7); // DShot1200
        adv.thrust_linearization = Some(0);
        let hw = hw_with_advanced(adv);
        let tel = empty_telemetry();
        let recs = AdvancedAnalyzer::new().analyze(&hw, &tel, &[]);

        let tl = recs
            .iter()
            .find(|r| matches!(r.parameter, AdvancedParameter::ThrustLinearization { .. }));
        assert!(tl.is_some(), "should recommend thrust linearization");
        if let AdvancedParameter::ThrustLinearization { recommended, .. } = tl.unwrap().parameter {
            assert_eq!(recommended, 25);
        }
    }

    #[test]
    fn thrust_lin_skipped_when_already_set() {
        let mut adv = default_adv();
        adv.motor_protocol = Some(6);
        adv.thrust_linearization = Some(20);
        let hw = hw_with_advanced(adv);
        let tel = empty_telemetry();
        let recs = AdvancedAnalyzer::new().analyze(&hw, &tel, &[]);

        assert!(
            !recs
                .iter()
                .any(|r| matches!(r.parameter, AdvancedParameter::ThrustLinearization { .. })),
            "already set => no rec"
        );
    }

    // ---------- D Max ----------

    #[test]
    fn d_max_recommended_when_d_min_equals_d() {
        let mut adv = default_adv();
        adv.d_min = Some(PerAxisU8 {
            roll: 38,
            pitch: 42,
            yaw: 0,
        });
        let mut hw = hw_with_advanced(adv);
        // Set PID D values to match d_min so the dynamic range is disabled.
        hw.pid_config.roll.d = 38.0;
        hw.pid_config.pitch.d = 42.0;

        let tel = empty_telemetry();
        let recs = AdvancedAnalyzer::new().analyze(&hw, &tel, &[]);

        let d_max_recs: Vec<_> = recs
            .iter()
            .filter(|r| matches!(r.parameter, AdvancedParameter::DMax { .. }))
            .collect();
        assert_eq!(
            d_max_recs.len(),
            2,
            "should recommend for both roll and pitch"
        );

        // Verify recommended d_min is ~65% of D.
        for rec in &d_max_recs {
            if let AdvancedParameter::DMax {
                current_d_min,
                current_d_max,
                recommended_d_min,
                ..
            } = rec.parameter
            {
                assert_eq!(current_d_min, current_d_max);
                let expected = (current_d_max as f32 * 0.65).round() as u8;
                assert_eq!(recommended_d_min, expected);
            }
        }
    }

    #[test]
    fn d_max_not_recommended_when_range_exists() {
        let mut adv = default_adv();
        adv.d_min = Some(PerAxisU8 {
            roll: 25,
            pitch: 28,
            yaw: 0,
        });
        let mut hw = hw_with_advanced(adv);
        hw.pid_config.roll.d = 38.0;
        hw.pid_config.pitch.d = 42.0;

        let tel = empty_telemetry();
        let recs = AdvancedAnalyzer::new().analyze(&hw, &tel, &[]);

        assert!(
            !recs
                .iter()
                .any(|r| matches!(r.parameter, AdvancedParameter::DMax { .. })),
            "d_min < D => no rec"
        );
    }

    #[test]
    fn d_max_gain_zero_recommends_restore() {
        let mut adv = default_adv();
        adv.d_max_gain = Some(0);
        let hw = hw_with_advanced(adv);
        let tel = empty_telemetry();
        let recs = AdvancedAnalyzer::new().analyze(&hw, &tel, &[]);

        assert!(
            recs.iter().any(|r| r.reason.contains("D Max gain is 0")),
            "should recommend restoring d_max_gain"
        );
    }

    // ---------- TPA (config-based) ----------

    #[test]
    fn tpa_recommended_when_rate_zero_and_d_high() {
        let mut adv = default_adv();
        adv.tpa = Some(TpaAdvancedConfig {
            mode: None,
            rate: 0,
            breakpoint: 1250,
        });
        let mut hw = hw_with_advanced(adv);
        hw.pid_config.roll.d = 38.0;
        hw.pid_config.pitch.d = 42.0;

        let tel = empty_telemetry();
        let recs = AdvancedAnalyzer::new().analyze(&hw, &tel, &[]);

        let tpa = recs
            .iter()
            .find(|r| matches!(r.parameter, AdvancedParameter::Tpa { .. }));
        assert!(tpa.is_some(), "should recommend TPA");
        if let AdvancedParameter::Tpa {
            recommended_rate,
            recommended_breakpoint,
            ..
        } = tpa.unwrap().parameter
        {
            assert_eq!(recommended_rate, 65);
            assert_eq!(recommended_breakpoint, 1350);
        }
    }

    #[test]
    fn tpa_not_recommended_when_d_low() {
        let mut adv = default_adv();
        adv.tpa = Some(TpaAdvancedConfig {
            mode: None,
            rate: 0,
            breakpoint: 1250,
        });
        let mut hw = hw_with_advanced(adv);
        hw.pid_config.roll.d = 20.0;
        hw.pid_config.pitch.d = 22.0;

        let tel = empty_telemetry();
        let recs = AdvancedAnalyzer::new().analyze(&hw, &tel, &[]);

        assert!(
            !recs
                .iter()
                .any(|r| matches!(r.parameter, AdvancedParameter::Tpa { .. })),
            "low D => no TPA rec"
        );
    }

    #[test]
    fn tpa_not_recommended_when_rate_nonzero() {
        let mut adv = default_adv();
        adv.tpa = Some(TpaAdvancedConfig {
            mode: None,
            rate: 65,
            breakpoint: 1350,
        });
        let mut hw = hw_with_advanced(adv);
        hw.pid_config.roll.d = 38.0;

        let tel = empty_telemetry();
        let recs = AdvancedAnalyzer::new().analyze(&hw, &tel, &[]);

        assert!(
            !recs
                .iter()
                .any(|r| matches!(r.parameter, AdvancedParameter::Tpa { .. })),
            "TPA already configured => no rec"
        );
    }

    // ---------- TPA (telemetry-based) ----------

    #[test]
    fn tpa_telemetry_increase_rate_when_high_throttle_noisy() {
        // TPA rate is low (30) and high-throttle noise is 2x low/mid noise
        // => recommend increasing rate to 65.
        let mut adv = default_adv();
        adv.tpa = Some(TpaAdvancedConfig {
            mode: None,
            rate: 30,
            breakpoint: 1350,
        });
        let hw = hw_with_advanced(adv);
        let tel = telemetry_with_noise(10.0, 20.0, 600);
        let recs = AdvancedAnalyzer::new().analyze(&hw, &tel, &[]);

        let tpa = recs
            .iter()
            .find(|r| matches!(r.parameter, AdvancedParameter::Tpa { .. }));
        assert!(tpa.is_some(), "should recommend TPA rate increase");
        if let AdvancedParameter::Tpa {
            recommended_rate,
            recommended_breakpoint,
            ..
        } = tpa.unwrap().parameter
        {
            assert_eq!(recommended_rate, 65);
            assert_eq!(recommended_breakpoint, 1350, "breakpoint <= 1400 stays");
        }
        assert!(
            matches!(tpa.unwrap().priority, Priority::Low),
            "TPA telemetry recs should be Low priority"
        );
    }

    #[test]
    fn tpa_telemetry_lower_breakpoint_when_high_throttle_noisy_and_rate_ok() {
        // TPA rate is 60 (>= 50) but breakpoint is too high (1500).
        // High-throttle noise is 2x => recommend lowering breakpoint to 1250.
        let mut adv = default_adv();
        adv.tpa = Some(TpaAdvancedConfig {
            mode: None,
            rate: 60,
            breakpoint: 1500,
        });
        let hw = hw_with_advanced(adv);
        let tel = telemetry_with_noise(10.0, 20.0, 600);
        let recs = AdvancedAnalyzer::new().analyze(&hw, &tel, &[]);

        let tpa = recs
            .iter()
            .find(|r| matches!(r.parameter, AdvancedParameter::Tpa { .. }));
        assert!(tpa.is_some(), "should recommend breakpoint reduction");
        if let AdvancedParameter::Tpa {
            recommended_rate,
            recommended_breakpoint,
            ..
        } = tpa.unwrap().parameter
        {
            assert_eq!(recommended_rate, 60, "rate stays at current value");
            assert_eq!(recommended_breakpoint, 1250);
        }
    }

    #[test]
    fn tpa_telemetry_reduce_excessive_rate_when_noise_flat() {
        // TPA rate is 90 (> 80) and noise is flat (ratio < 1.1).
        // => recommend reducing rate to 65.
        let mut adv = default_adv();
        adv.tpa = Some(TpaAdvancedConfig {
            mode: None,
            rate: 90,
            breakpoint: 1350,
        });
        let hw = hw_with_advanced(adv);
        // Same noise in both bands => ratio = 1.0
        let tel = telemetry_with_noise(15.0, 15.0, 600);
        let recs = AdvancedAnalyzer::new().analyze(&hw, &tel, &[]);

        let tpa = recs
            .iter()
            .find(|r| matches!(r.parameter, AdvancedParameter::Tpa { .. }));
        assert!(tpa.is_some(), "should recommend TPA rate reduction");
        if let AdvancedParameter::Tpa {
            recommended_rate,
            recommended_breakpoint,
            ..
        } = tpa.unwrap().parameter
        {
            assert_eq!(recommended_rate, 65);
            assert_eq!(recommended_breakpoint, 1350, "breakpoint unchanged");
        }
    }

    #[test]
    fn tpa_telemetry_no_rec_when_noise_moderate() {
        // Noise ratio is between 1.1 and 1.5 => no telemetry-based rec.
        let mut adv = default_adv();
        adv.tpa = Some(TpaAdvancedConfig {
            mode: None,
            rate: 65,
            breakpoint: 1350,
        });
        let hw = hw_with_advanced(adv);
        // ratio = 13/10 = 1.3 -- in the dead zone
        let tel = telemetry_with_noise(10.0, 13.0, 600);
        let recs = AdvancedAnalyzer::new().analyze(&hw, &tel, &[]);

        assert!(
            !recs
                .iter()
                .any(|r| matches!(r.parameter, AdvancedParameter::Tpa { .. })),
            "moderate noise ratio => no TPA rec"
        );
    }

    #[test]
    fn tpa_telemetry_no_rec_when_insufficient_samples() {
        // Only 100 samples per band (< 500 minimum) => no telemetry rec.
        let mut adv = default_adv();
        adv.tpa = Some(TpaAdvancedConfig {
            mode: None,
            rate: 30,
            breakpoint: 1350,
        });
        let hw = hw_with_advanced(adv);
        let tel = telemetry_with_noise(10.0, 20.0, 100);
        let recs = AdvancedAnalyzer::new().analyze(&hw, &tel, &[]);

        // The config-based check won't fire either (rate != 0).
        assert!(
            !recs
                .iter()
                .any(|r| matches!(r.parameter, AdvancedParameter::Tpa { .. })),
            "insufficient samples => no TPA rec"
        );
    }

    #[test]
    fn tpa_telemetry_skipped_when_rate_zero() {
        // Rate == 0 is handled by the config-based check, not telemetry.
        let mut adv = default_adv();
        adv.tpa = Some(TpaAdvancedConfig {
            mode: None,
            rate: 0,
            breakpoint: 1350,
        });
        let mut hw = hw_with_advanced(adv);
        hw.pid_config.roll.d = 38.0;
        hw.pid_config.pitch.d = 42.0;

        let tel = telemetry_with_noise(10.0, 20.0, 600);
        let recs = AdvancedAnalyzer::new().analyze(&hw, &tel, &[]);

        // Should get exactly one TPA rec -- from the config-based check.
        let tpa_recs: Vec<_> = recs
            .iter()
            .filter(|r| matches!(r.parameter, AdvancedParameter::Tpa { .. }))
            .collect();
        assert_eq!(
            tpa_recs.len(),
            1,
            "only config-based TPA rec when rate is 0"
        );
        assert!(
            tpa_recs[0].reason.contains("TPA is disabled"),
            "should be the config-based reason"
        );
    }

    #[test]
    fn tpa_telemetry_increase_rate_and_lower_breakpoint() {
        // TPA rate is low (30) AND breakpoint is high (1500).
        // High-throttle noise is 2x => recommend rate=65 AND breakpoint=1250.
        let mut adv = default_adv();
        adv.tpa = Some(TpaAdvancedConfig {
            mode: None,
            rate: 30,
            breakpoint: 1500,
        });
        let hw = hw_with_advanced(adv);
        let tel = telemetry_with_noise(10.0, 20.0, 600);
        let recs = AdvancedAnalyzer::new().analyze(&hw, &tel, &[]);

        let tpa = recs
            .iter()
            .find(|r| matches!(r.parameter, AdvancedParameter::Tpa { .. }));
        assert!(tpa.is_some(), "should recommend TPA adjustment");
        if let AdvancedParameter::Tpa {
            recommended_rate,
            recommended_breakpoint,
            ..
        } = tpa.unwrap().parameter
        {
            // Rate < 50 triggers the rate-increase path first, which also
            // adjusts breakpoint when > 1400.
            assert_eq!(recommended_rate, 65);
            assert_eq!(recommended_breakpoint, 1250);
        }
    }

    // ---------- feedforward ----------

    #[test]
    fn feedforward_jitter_recommended_when_zero() {
        let mut adv = default_adv();
        adv.feedforward = Some(FeedforwardConfig {
            jitter_factor: Some(0),
            ..FeedforwardConfig::default()
        });
        let hw = hw_with_advanced(adv);
        let tel = empty_telemetry();
        let recs = AdvancedAnalyzer::new().analyze(&hw, &tel, &[]);

        let ff = recs.iter().find(|r| {
            matches!(
                r.parameter,
                AdvancedParameter::Feedforward {
                    param: FeedforwardParam::JitterFactor,
                    ..
                }
            )
        });
        assert!(ff.is_some(), "should recommend jitter factor");
        if let AdvancedParameter::Feedforward { recommended, .. } = ff.unwrap().parameter {
            assert_eq!(recommended, 7);
        }
    }

    #[test]
    fn feedforward_jitter_not_recommended_when_nonzero() {
        let mut adv = default_adv();
        adv.feedforward = Some(FeedforwardConfig {
            jitter_factor: Some(7),
            ..FeedforwardConfig::default()
        });
        let hw = hw_with_advanced(adv);
        let tel = empty_telemetry();
        let recs = AdvancedAnalyzer::new().analyze(&hw, &tel, &[]);

        assert!(
            !recs.iter().any(|r| matches!(
                r.parameter,
                AdvancedParameter::Feedforward {
                    param: FeedforwardParam::JitterFactor,
                    ..
                }
            )),
            "jitter already set => no rec"
        );
    }

    // ---------- no advanced config ----------

    #[test]
    fn no_recs_when_advanced_tuning_is_none() {
        let hw = HardwareConfiguration::test_default();
        assert!(hw.advanced_tuning.is_none());
        let tel = empty_telemetry();
        let recs = AdvancedAnalyzer::new().analyze(&hw, &tel, &[]);
        assert!(recs.is_empty(), "no advanced config => no recs");
    }

    // ---------- all-at-once smoke test ----------

    #[test]
    fn full_sweep_produces_expected_count() {
        let adv = AdvancedTuningConfig {
            d_min: Some(PerAxisU8 {
                roll: 38,
                pitch: 42,
                yaw: 0,
            }),
            d_max_gain: Some(0),
            d_max_advance: None,
            feedforward: Some(FeedforwardConfig {
                jitter_factor: Some(0),
                ..FeedforwardConfig::default()
            }),
            tpa: Some(TpaAdvancedConfig {
                mode: None,
                rate: 0,
                breakpoint: 1250,
            }),
            vbat_sag_compensation: Some(0),
            thrust_linearization: Some(0),
            dynamic_idle_min_rpm: Some(0),
            anti_gravity_gain: None,
            motor_protocol: Some(6),
            dshot_bidir: Some(true),
            motor_kv: None,
            motor_poles: None,
            simplified_tuning: None,
        };
        let mut hw = hw_with_advanced(adv);
        hw.pid_config.roll.d = 38.0;
        hw.pid_config.pitch.d = 42.0;

        // Use empty telemetry so the telemetry-based TPA check doesn't add
        // an extra rec (TPA rate=0 is handled by the config-based check).
        let tel = empty_telemetry();
        let recs = AdvancedAnalyzer::new().analyze(&hw, &tel, &[]);

        // Expected: vbat(1) + idle(1) + thrust_lin(1) + d_max_gain(1) +
        //           d_max roll(1) + d_max pitch(1) + tpa(1) + ff_jitter(1) = 8
        assert_eq!(
            recs.len(),
            8,
            "full sweep should produce 8 recs, got {}: {:?}",
            recs.len(),
            recs.iter().map(|r| &r.reason).collect::<Vec<_>>()
        );

        // Every rec should be Low priority.
        for rec in &recs {
            assert!(
                matches!(rec.priority, Priority::Low),
                "all advanced recs should be Low priority"
            );
        }
    }

    // ---------- simplified tuning sliders ----------

    fn make_pid_rec(
        axis: Axis,
        term: PidTerm,
        current: f32,
        recommended: f32,
    ) -> PidRecommendation {
        PidRecommendation {
            axis,
            term,
            current_value: current,
            recommended_value: recommended,
            reason: "test".into(),
            priority: Priority::Medium,
        }
    }

    #[test]
    fn slider_master_hint_when_p_increase_recommended() {
        let mut adv = default_adv();
        adv.simplified_tuning = Some(SimplifiedTuning {
            mode: 2,
            master_multiplier: Some(100),
            d_gain: Some(120), // not at default, so D hint won't fire
            ..SimplifiedTuning::default()
        });
        let hw = hw_with_advanced(adv);
        let tel = empty_telemetry();
        let pid_recs = vec![make_pid_rec(Axis::Roll, PidTerm::P, 42.0, 48.0)];
        let recs = AdvancedAnalyzer::new().analyze(&hw, &tel, &pid_recs);

        let slider = recs.iter().find(|r| matches!(
            &r.parameter,
            AdvancedParameter::SliderHint { slider_name, .. } if slider_name == "Master Multiplier"
        ));
        assert!(
            slider.is_some(),
            "should hint Master Multiplier when P increase recommended"
        );
        if let AdvancedParameter::SliderHint {
            current_value,
            suggested_value,
            ..
        } = &slider.unwrap().parameter
        {
            assert_eq!(*current_value, 100);
            assert_eq!(*suggested_value, 110);
        }
    }

    #[test]
    fn slider_d_gain_hint_when_d_increase_recommended() {
        let mut adv = default_adv();
        adv.simplified_tuning = Some(SimplifiedTuning {
            mode: 2,
            master_multiplier: Some(120), // not at default, so master hint won't fire
            d_gain: Some(100),
            ..SimplifiedTuning::default()
        });
        let hw = hw_with_advanced(adv);
        let tel = empty_telemetry();
        let pid_recs = vec![make_pid_rec(Axis::Roll, PidTerm::D, 38.0, 44.0)];
        let recs = AdvancedAnalyzer::new().analyze(&hw, &tel, &pid_recs);

        let slider = recs.iter().find(|r| {
            matches!(
                &r.parameter,
                AdvancedParameter::SliderHint { slider_name, .. } if slider_name == "D Gain"
            )
        });
        assert!(
            slider.is_some(),
            "should hint D Gain when D increase recommended"
        );
        if let AdvancedParameter::SliderHint {
            current_value,
            suggested_value,
            ..
        } = &slider.unwrap().parameter
        {
            assert_eq!(*current_value, 100);
            assert_eq!(*suggested_value, 110);
        }
    }

    #[test]
    fn slider_no_hint_when_mode_off() {
        let mut adv = default_adv();
        adv.simplified_tuning = Some(SimplifiedTuning {
            mode: 0,
            master_multiplier: Some(100),
            d_gain: Some(100),
            ..SimplifiedTuning::default()
        });
        let hw = hw_with_advanced(adv);
        let tel = empty_telemetry();
        let pid_recs = vec![
            make_pid_rec(Axis::Roll, PidTerm::P, 42.0, 48.0),
            make_pid_rec(Axis::Roll, PidTerm::D, 38.0, 44.0),
        ];
        let recs = AdvancedAnalyzer::new().analyze(&hw, &tel, &pid_recs);

        assert!(
            !recs
                .iter()
                .any(|r| matches!(r.parameter, AdvancedParameter::SliderHint { .. })),
            "mode 0 => no slider hints"
        );
    }

    #[test]
    fn slider_no_hint_when_no_pid_increase() {
        let mut adv = default_adv();
        adv.simplified_tuning = Some(SimplifiedTuning {
            mode: 2,
            master_multiplier: Some(100),
            d_gain: Some(100),
            ..SimplifiedTuning::default()
        });
        let hw = hw_with_advanced(adv);
        let tel = empty_telemetry();
        // PID rec suggests *decreasing* P -- no slider bump needed.
        let pid_recs = vec![make_pid_rec(Axis::Roll, PidTerm::P, 48.0, 42.0)];
        let recs = AdvancedAnalyzer::new().analyze(&hw, &tel, &pid_recs);

        assert!(
            !recs.iter().any(|r| matches!(
                &r.parameter,
                AdvancedParameter::SliderHint { slider_name, .. } if slider_name == "Master Multiplier"
            )),
            "no P increase => no Master Multiplier hint"
        );
    }

    #[test]
    fn slider_no_hint_when_sliders_already_bumped() {
        let mut adv = default_adv();
        adv.simplified_tuning = Some(SimplifiedTuning {
            mode: 2,
            master_multiplier: Some(120),
            d_gain: Some(110),
            ..SimplifiedTuning::default()
        });
        let hw = hw_with_advanced(adv);
        let tel = empty_telemetry();
        let pid_recs = vec![
            make_pid_rec(Axis::Roll, PidTerm::P, 42.0, 48.0),
            make_pid_rec(Axis::Roll, PidTerm::D, 38.0, 44.0),
        ];
        let recs = AdvancedAnalyzer::new().analyze(&hw, &tel, &pid_recs);

        assert!(
            !recs
                .iter()
                .any(|r| matches!(r.parameter, AdvancedParameter::SliderHint { .. })),
            "sliders already bumped => no hints"
        );
    }

    #[test]
    fn slider_no_hint_when_simplified_tuning_none() {
        let adv = default_adv();
        // simplified_tuning is None by default
        assert!(adv.simplified_tuning.is_none());
        let hw = hw_with_advanced(adv);
        let tel = empty_telemetry();
        let pid_recs = vec![make_pid_rec(Axis::Roll, PidTerm::P, 42.0, 48.0)];
        let recs = AdvancedAnalyzer::new().analyze(&hw, &tel, &pid_recs);

        assert!(
            !recs
                .iter()
                .any(|r| matches!(r.parameter, AdvancedParameter::SliderHint { .. })),
            "no simplified_tuning => no slider hints"
        );
    }

    #[test]
    fn slider_both_hints_fire_together() {
        let mut adv = default_adv();
        adv.simplified_tuning = Some(SimplifiedTuning {
            mode: 2,
            master_multiplier: Some(100),
            d_gain: Some(100),
            ..SimplifiedTuning::default()
        });
        let hw = hw_with_advanced(adv);
        let tel = empty_telemetry();
        let pid_recs = vec![
            make_pid_rec(Axis::Roll, PidTerm::P, 42.0, 48.0),
            make_pid_rec(Axis::Roll, PidTerm::D, 38.0, 44.0),
        ];
        let recs = AdvancedAnalyzer::new().analyze(&hw, &tel, &pid_recs);

        let slider_count = recs
            .iter()
            .filter(|r| matches!(r.parameter, AdvancedParameter::SliderHint { .. }))
            .count();
        assert_eq!(
            slider_count, 2,
            "both Master Multiplier and D Gain hints should fire"
        );
    }
}
