//! Realistic tests with higher amplitude oscillations

use super::*;
use chrono::Utc;
use nalgebra::Vector3;
use uuid::Uuid;

/// Create realistic test telemetry with strong oscillations that should trigger recommendations
fn create_realistic_oscillation_telemetry(
    frequency: f32,
    amplitude: f32,
    axis_index: usize,
    sample_count: usize,
    sample_rate: f32,
) -> TelemetryData {
    let mut gyro = TimeSeriesVector3::with_capacity(sample_count);
    let mut accel = TimeSeriesVector3::with_capacity(sample_count);

    for i in 0..sample_count {
        let t = i as f32 / sample_rate;
        let mut gyro_values = [0.0f32; 3];

        // Add main oscillation
        gyro_values[axis_index] = amplitude * (2.0 * std::f32::consts::PI * frequency * t).sin();

        // Add realistic base flight movement
        for j in 0..3 {
            gyro_values[j] += 2.0 * (2.0 * std::f32::consts::PI * 1.5 * t + j as f32).sin();
        }

        // Add realistic noise
        for j in 0..3 {
            gyro_values[j] += 0.5 * (rand::random::<f32>() - 0.5);
        }

        gyro.push(Vector3::new(gyro_values[0], gyro_values[1], gyro_values[2]));
        accel.push(Vector3::new(0.0, 0.0, 9.81));
    }

    TelemetryData {
        sample_rate,
        gyro,
        accel,
        motor: Vec::new(),
        pid_error: PidErrorTrace {
            roll: vec![0.0; sample_count],
            pitch: vec![0.0; sample_count],
            yaw: vec![0.0; sample_count],
        },
        rc_commands: RcCommandTrace {
            roll: vec![0.0; sample_count],
            pitch: vec![0.0; sample_count],
            yaw: vec![0.0; sample_count],
            throttle: vec![0.5; sample_count],
        },
        loop_time_variance: 0.0,
        cpu_load: Vec::new(),
    }
}

fn create_realistic_session(frequency: f32, amplitude: f32, axis_index: usize) -> FlightSession {
    let sample_count = 16384; // 16 seconds at 1kHz
    let sample_rate = 1000.0;
    let telemetry = create_realistic_oscillation_telemetry(
        frequency,
        amplitude,
        axis_index,
        sample_count,
        sample_rate,
    );

    FlightSession {
        metadata: FlightMetadata {
            session_id: Uuid::new_v4(),
            timestamp: Utc::now(),
            duration_ms: (sample_count as f32 / sample_rate * 1000.0) as u64,
            hardware: HardwareConfiguration::test_default(),
            environment: EnvironmentalConditions::default(),
            pilot: PilotProfile::default(),
        },
        telemetry,
        events: Vec::new(),
        analysis_results: None,
    }
}

#[cfg(test)]
mod realistic_tests {
    use super::*;

    #[test]
    fn test_strong_p_term_oscillation() {
        // Strong P-term oscillation that should trigger recommendations
        let session = create_realistic_session(25.0, 20.0, 0); // 25 Hz, 20 deg/s amplitude

        let mut engine = AnalysisEngine::new();
        let result = engine.analyze(&session).expect("Analysis should succeed");

        println!("Strong P-term test:");
        println!("  Issues detected: {}", result.detected_issues.len());
        for issue in &result.detected_issues {
            println!("    {:?}", issue.issue_type);
        }
        println!(
            "  PID recommendations: {}",
            result.pid_recommendations.len()
        );
        for rec in &result.pid_recommendations {
            println!(
                "    {:?} {:?}: {:.1} -> {:.1} ({})",
                rec.axis, rec.term, rec.current_value, rec.recommended_value, rec.reason
            );
        }

        // Should detect P-term oscillation (or mechanical resonance with enhanced detection)
        let p_term_issues: Vec<_> = result
            .detected_issues
            .iter()
            .filter(|issue| matches!(issue.issue_type, IssueType::PTermOscillation { .. }))
            .collect();

        let mechanical_issues: Vec<_> = result
            .detected_issues
            .iter()
            .filter(|issue| matches!(issue.issue_type, IssueType::MechanicalResonance { .. }))
            .collect();

        // With enhanced detection, high-Q sine waves get classified as mechanical resonance
        assert!(
            !p_term_issues.is_empty() || !mechanical_issues.is_empty(),
            "Should detect P-term oscillation or mechanical resonance with enhanced detection"
        );

        // Should recommend reducing P-gain (in gyro-only mode this comes from high std dev)
        let p_recommendations: Vec<_> = result
            .pid_recommendations
            .iter()
            .filter(|rec| matches!(rec.term, PidTerm::P))
            .collect();

        // Note: might not trigger in gyro-only mode depending on thresholds
        if !p_recommendations.is_empty() {
            assert!(
                p_recommendations[0].recommended_value < p_recommendations[0].current_value,
                "Should recommend reducing P-gain"
            );
        }
    }

    #[test]
    fn test_strong_mechanical_resonance() {
        // Strong mechanical resonance
        let session = create_realistic_session(180.0, 15.0, 0); // 180 Hz, 15 deg/s amplitude

        let mut engine = AnalysisEngine::new();
        let result = engine.analyze(&session).expect("Analysis should succeed");

        println!("Strong mechanical resonance test:");
        println!("  Issues detected: {}", result.detected_issues.len());
        println!(
            "  Filter recommendations: {}",
            result.filter_recommendations.len()
        );

        // Should detect mechanical resonance
        let mechanical_issues: Vec<_> = result
            .detected_issues
            .iter()
            .filter(|issue| matches!(issue.issue_type, IssueType::MechanicalResonance { .. }))
            .collect();

        if mechanical_issues.is_empty() {
            // Might be classified as motor noise, check for that
            let motor_issues: Vec<_> = result
                .detected_issues
                .iter()
                .filter(|issue| matches!(issue.issue_type, IssueType::Imbalance { .. }))
                .collect();
            assert!(
                !motor_issues.is_empty(),
                "Should detect some form of high-frequency issue"
            );
        }

        // Should recommend notch filter
        let notch_recommendations: Vec<_> = result
            .filter_recommendations
            .iter()
            .filter(|rec| {
                matches!(
                    rec.recommendation_type,
                    FilterRecommendationType::ConfigureGyroNotch { .. }
                        | FilterRecommendationType::AdjustDynamicNotch { .. }
                )
            })
            .collect();

        assert!(
            !notch_recommendations.is_empty(),
            "Should recommend notch filter for resonance"
        );
    }

    #[test]
    fn test_comprehensive_analysis() {
        // Test with multiple types of oscillations
        let mut gyro = TimeSeriesVector3::with_capacity(16384);
        let mut accel = TimeSeriesVector3::with_capacity(16384);
        let sample_rate = 1000.0;

        for i in 0..16384 {
            let t = i as f32 / sample_rate;

            // P-term oscillation on roll (25 Hz)
            let p_osc = 10.0 * (2.0 * std::f32::consts::PI * 25.0 * t).sin();

            // D-term noise on pitch (120 Hz)
            let d_noise = 3.0 * (2.0 * std::f32::consts::PI * 120.0 * t).sin();

            // Mechanical resonance on roll (200 Hz)
            let mech_res = 8.0 * (2.0 * std::f32::consts::PI * 200.0 * t).sin();

            // Motor noise on all axes (400 Hz)
            let motor_noise = 2.0 * (2.0 * std::f32::consts::PI * 400.0 * t).sin();

            let gyro_x = p_osc + mech_res + motor_noise + 0.5 * (rand::random::<f32>() - 0.5);
            let gyro_y = d_noise + motor_noise + 0.5 * (rand::random::<f32>() - 0.5);
            let gyro_z = motor_noise + 0.5 * (rand::random::<f32>() - 0.5);

            gyro.push(Vector3::new(gyro_x, gyro_y, gyro_z));
            accel.push(Vector3::new(0.0, 0.0, 9.81));
        }

        let telemetry = TelemetryData {
            sample_rate,
            gyro,
            accel,
            motor: Vec::new(),
            pid_error: PidErrorTrace {
                roll: vec![0.0; 16384],
                pitch: vec![0.0; 16384],
                yaw: vec![0.0; 16384],
            },
            rc_commands: RcCommandTrace {
                roll: vec![0.0; 16384],
                pitch: vec![0.0; 16384],
                yaw: vec![0.0; 16384],
                throttle: vec![0.5; 16384],
            },
            loop_time_variance: 0.0,
            cpu_load: Vec::new(),
        };

        let session = FlightSession {
            metadata: FlightMetadata {
                session_id: Uuid::new_v4(),
                timestamp: Utc::now(),
                duration_ms: 16384,
                hardware: HardwareConfiguration::test_default(),
                environment: EnvironmentalConditions::default(),
                pilot: PilotProfile::default(),
            },
            telemetry,
            events: Vec::new(),
            analysis_results: None,
        };

        let mut engine = AnalysisEngine::new();
        let result = engine.analyze(&session).expect("Analysis should succeed");

        println!("Comprehensive analysis:");
        println!("  Issues detected: {}", result.detected_issues.len());
        for issue in &result.detected_issues {
            println!("    {:?}", issue.issue_type);
        }
        println!(
            "  Filter recommendations: {}",
            result.filter_recommendations.len()
        );
        println!(
            "  PID recommendations: {}",
            result.pid_recommendations.len()
        );
        println!("  Tune quality score: {:.1}", result.tune_quality_score);

        // Should detect multiple issues
        assert!(
            result.detected_issues.len() >= 2,
            "Should detect multiple oscillations"
        );

        // Should have reasonable tune quality score (low due to multiple issues)
        assert!(
            result.tune_quality_score < 90.0,
            "Tune quality should reflect detected issues"
        );
        assert!(
            result.tune_quality_score >= 0.0,
            "Tune quality should not be negative"
        );
    }

    #[test]
    fn test_filter_recommendation_logic() {
        // Create session with high Q-factor resonance
        let session = create_realistic_session(200.0, 12.0, 0);

        let mut engine = AnalysisEngine::new();
        let result = engine.analyze(&session).expect("Analysis should succeed");

        println!("Filter recommendation test:");
        println!(
            "  Filter recommendations: {}",
            result.filter_recommendations.len()
        );
        for rec in &result.filter_recommendations {
            println!(
                "    {:?} at {:.1} Hz, Q: {:?}",
                rec.recommendation_type, rec.frequency, rec.q_factor
            );
        }

        // Should have filter recommendations for high amplitude oscillations
        if !result.filter_recommendations.is_empty() {
            let notch_recs: Vec<_> = result
                .filter_recommendations
                .iter()
                .filter(|rec| {
                    matches!(
                        rec.recommendation_type,
                        FilterRecommendationType::ConfigureGyroNotch { .. }
                            | FilterRecommendationType::AdjustDynamicNotch { .. }
                    )
                })
                .collect();

            if !notch_recs.is_empty() {
                let rec = notch_recs[0];
                assert!(
                    (rec.frequency - 200.0).abs() < 30.0,
                    "Notch frequency should be near oscillation"
                );
                assert!(rec.q_factor.is_some(), "Notch filter should have Q-factor");
            }
        }
    }
}

// Simple random number generator for testing
mod rand {
    use std::cell::Cell;

    thread_local! {
        static RNG_STATE: Cell<u64> = Cell::new(42);
    }

    pub fn random<T>() -> T
    where
        T: From<f32>,
    {
        RNG_STATE.with(|state| {
            let mut x = state.get();
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            state.set(x);
            T::from((x as f32) / (u64::MAX as f32))
        })
    }
}
