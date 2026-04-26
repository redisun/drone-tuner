//! Debug tests to understand oscillation detection behavior

use super::*;
use chrono::Utc;
use nalgebra::Vector3;
use uuid::Uuid;

/// Create simple test telemetry with a single pure sine wave
fn create_simple_sine_telemetry(
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

        // Add sine wave to specified axis
        gyro_values[axis_index] = amplitude * (2.0 * std::f32::consts::PI * frequency * t).sin();

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

fn create_test_session_from_telemetry(telemetry: TelemetryData) -> FlightSession {
    FlightSession {
        metadata: FlightMetadata {
            session_id: Uuid::new_v4(),
            timestamp: Utc::now(),
            duration_ms: ((telemetry.gyro.len() as f32 / telemetry.sample_rate) * 1000.0) as u64,
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
mod debug_tests {
    use super::*;

    #[test]
    fn debug_frequency_detection() {
        // Test with a pure 25 Hz sine wave (should be P-term)
        let telemetry = create_simple_sine_telemetry(25.0, 5.0, 0, 8192, 1000.0);
        let session = create_test_session_from_telemetry(telemetry);

        let mut engine = AnalysisEngine::new();
        let result = engine.analyze(&session).expect("Analysis should succeed");

        println!("Debug: 25 Hz test");
        println!("  Frequency peaks found: {}", result.frequency_analysis.peaks.len());
        for peak in &result.frequency_analysis.peaks {
            println!("    Peak: {:.1} Hz, amplitude: {:.3}, Q: {:.1}",
                    peak.frequency, peak.amplitude, peak.q_factor);
        }
        println!("  Issues detected: {}", result.detected_issues.len());
        for issue in &result.detected_issues {
            println!("    Issue: {:?}", issue.issue_type);
        }
        println!("  Filter recommendations: {}", result.filter_recommendations.len());
        println!("  PID recommendations: {}", result.pid_recommendations.len());
    }

    #[test]
    fn debug_oscillation_detector() {
        let detector = OscillationDetector::new();

        println!("Debug: Frequency bands configuration");
        println!("  P-term band: {:.1}-{:.1} Hz",
                detector.config.frequency_bands.p_term_band.0,
                detector.config.frequency_bands.p_term_band.1);
        println!("  D-term band: {:.1}-{:.1} Hz",
                detector.config.frequency_bands.d_term_band.0,
                detector.config.frequency_bands.d_term_band.1);
        println!("  Mechanical band: {:.1}-{:.1} Hz",
                detector.config.frequency_bands.mechanical_band.0,
                detector.config.frequency_bands.mechanical_band.1);
        println!("  Motor noise band: {:.1}-{:.1} Hz",
                detector.config.frequency_bands.motor_noise_band.0,
                detector.config.frequency_bands.motor_noise_band.1);

        // Test classification
        let test_cases = [
            (25.0, 2.0, "25 Hz, Q=2.0"),
            (120.0, 2.0, "120 Hz, Q=2.0"),
            (180.0, 15.0, "180 Hz, Q=15.0"),
            (400.0, 2.0, "400 Hz, Q=2.0"),
        ];

        for (freq, q, description) in test_cases {
            let osc_type = detector.classify_oscillation_type(freq, q);
            println!("  {} -> {:?}", description, osc_type);
        }
    }

    #[test]
    fn debug_fft_analysis() {
        // Test FFT with known frequency
        let telemetry = create_simple_sine_telemetry(25.0, 5.0, 0, 8192, 1000.0);
        let session = create_test_session_from_telemetry(telemetry);

        let mut engine = AnalysisEngine::new();
        let freq_analysis = engine.perform_frequency_analysis(&session.telemetry)
            .expect("FFT analysis should succeed");

        println!("Debug: FFT Analysis");
        println!("  Frequency bins: {}", freq_analysis.frequencies.len());
        println!("  Frequency range: {:.1} - {:.1} Hz",
                freq_analysis.frequencies.first().unwrap_or(&0.0),
                freq_analysis.frequencies.last().unwrap_or(&0.0));

        if let Some(psd) = freq_analysis.psd.get(&Axis::Roll) {
            println!("  Roll PSD values: {}", psd.len());
            let max_power = psd.iter().fold(0.0f32, |max, &val| val.max(max));
            let max_idx = psd.iter().enumerate()
                .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
                .map(|(idx, _)| idx)
                .unwrap_or(0);

            println!("  Max power: {:.6} at {:.1} Hz", max_power, freq_analysis.frequencies.get(max_idx).unwrap_or(&0.0));
        }

        println!("  Spectral peaks found: {}", freq_analysis.peaks.len());
        for peak in &freq_analysis.peaks {
            println!("    Peak: {:.1} Hz, magnitude: {:.6}, Q: {:.1}",
                    peak.frequency, peak.magnitude, peak.q_factor);
        }
    }

    #[test]
    fn debug_amplitude_threshold() {
        let mut config = AnalysisConfig::default();
        config.oscillation_threshold = 0.001; // Very low threshold

        let telemetry = create_simple_sine_telemetry(25.0, 1.0, 0, 8192, 1000.0);
        let session = create_test_session_from_telemetry(telemetry);

        let mut engine = AnalysisEngine::with_config(config);
        let result = engine.analyze(&session).expect("Analysis should succeed");

        println!("Debug: Low threshold test");
        println!("  Peaks found: {}", result.frequency_analysis.peaks.len());
        println!("  Issues detected: {}", result.detected_issues.len());

        for issue in &result.detected_issues {
            println!("    Issue: {:?}", issue.issue_type);
        }
    }
}