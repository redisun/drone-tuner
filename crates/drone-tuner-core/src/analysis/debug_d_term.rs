//! Debug D-term oscillation detection

use super::*;
use chrono::Utc;
use nalgebra::Vector3;
use uuid::Uuid;

#[cfg(test)]
mod debug_d_term_tests {
    use super::*;

    #[test]
    fn debug_d_term_120hz() {
        // Test data with a D-term oscillation at 120 Hz (same as in failing test)
        let _oscillations = vec![(120.0, 3.0, 1.0)]; // 120 Hz, 3.0 amplitude, pitch axis

        let sample_count = 8192;
        let sample_rate = 1000.0;
        let mut gyro = TimeSeriesVector3::with_capacity(sample_count);
        let mut accel = TimeSeriesVector3::with_capacity(sample_count);

        for i in 0..sample_count {
            let t = i as f32 / sample_rate;
            let mut gyro_values = [0.0f32; 3];

            // Add oscillation to pitch axis (index 1)
            gyro_values[1] = 3.0 * (2.0 * std::f32::consts::PI * 120.0 * t).sin();

            // Add small noise
            for value in &mut gyro_values {
                *value += 0.1 * (simple_random() - 0.5);
            }

            gyro.push(Vector3::new(gyro_values[0], gyro_values[1], gyro_values[2]));
            accel.push(Vector3::new(0.0, 0.0, 9.81));
        }

        let telemetry = TelemetryData {
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
        };

        let session = FlightSession {
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
        };

        let mut engine = AnalysisEngine::new();
        let result = engine.analyze(&session).expect("Analysis should succeed");

        println!("Debug D-term 120 Hz test:");
        println!("  Frequency peaks found: {}", result.frequency_analysis.peaks.len());
        for peak in &result.frequency_analysis.peaks {
            println!("    Peak: {:.1} Hz, amplitude: {:.3}, Q: {:.1} on {:?}",
                    peak.frequency, peak.amplitude, peak.q_factor, peak.axes);
        }
        println!("  Issues detected: {}", result.detected_issues.len());
        for issue in &result.detected_issues {
            println!("    Issue: {:?}", issue.issue_type);
        }

        // Manual frequency analysis check
        if let Some(pitch_psd) = result.frequency_analysis.gyro_y_psd.get(120..125) {
            println!("  PSD around 120 Hz: {:?}", pitch_psd);
        }
    }

    #[test]
    fn debug_amplitude_detection() {
        // Test with different amplitudes to find threshold
        let amplitudes = [0.5, 1.0, 2.0, 5.0, 10.0];

        for &amp in &amplitudes {
            let sample_count = 8192;
            let sample_rate = 1000.0;
            let mut gyro = TimeSeriesVector3::with_capacity(sample_count);

            for i in 0..sample_count {
                let t = i as f32 / sample_rate;
                let signal = amp * (2.0 * std::f32::consts::PI * 120.0 * t).sin();
                gyro.push(Vector3::new(0.0, signal, 0.0));
            }

            let telemetry = TelemetryData {
                sample_rate,
                gyro,
                accel: TimeSeriesVector3::with_capacity(0),
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
            };

            let session = FlightSession {
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
            };

            let mut engine = AnalysisEngine::new();
            let result = engine.analyze(&session).expect("Analysis should succeed");

            println!("Amplitude {:.1}: {} peaks, {} issues",
                    amp, result.frequency_analysis.peaks.len(), result.detected_issues.len());
        }
    }
}

// Simple random number generator
static mut RNG_STATE: u64 = 42;

fn simple_random() -> f32 {
    unsafe {
        RNG_STATE ^= RNG_STATE << 13;
        RNG_STATE ^= RNG_STATE >> 17;
        RNG_STATE ^= RNG_STATE << 5;
        (RNG_STATE as f32) / (u64::MAX as f32)
    }
}