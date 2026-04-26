//! Filter recommendations driven by detected oscillations.
//!
//! Maps oscillation findings to concrete Betaflight filter primitives
//! (gyro notch, dynamic notch, RPM filter, D-term lowpass).

use super::{DetectedOscillation, OscillationType};
use crate::domain::{
    DynamicFilterSettings, FilterRecommendation, FilterRecommendationType, FilterType,
    HardwareConfiguration, Priority,
};
use crate::error::Result;

/// Filter optimization component
#[derive(Debug)]
pub(super) struct FilterOptimizer {
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

impl FilterOptimizer {
    pub(super) fn new() -> Self {
        Self {
            config: FilterOptimizerConfig::default(),
        }
    }

    pub(super) fn optimize(
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
                        dynamic_settings: Some(DynamicFilterSettings {
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
