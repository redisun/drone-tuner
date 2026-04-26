//! Benchmarks for FFT analysis performance.

use chrono::Utc;
use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};
use drone_tuner_core::{
    domain::{
        EnvironmentalConditions, FlightMetadata, FlightSession, FlyingStyle, HardwareConfiguration,
        PilotProfile, SkillLevel, TelemetryData, TimeSeriesVector3,
    },
    AnalysisEngine,
};
use nalgebra::Vector3;
use std::time::Duration;
use uuid::Uuid;

/// Generate synthetic telemetry data for benchmarking
fn generate_synthetic_telemetry(sample_count: usize, sample_rate: f32) -> TelemetryData {
    let mut gyro = TimeSeriesVector3::with_capacity(sample_count);
    let mut accel = TimeSeriesVector3::with_capacity(sample_count);

    // Generate sine waves with some oscillations to make analysis realistic
    for i in 0..sample_count {
        let t = i as f32 / sample_rate;

        // Base flight movement + oscillations
        let base_freq = 2.0; // Hz - basic flight maneuvers
        let osc_freq = 120.0; // Hz - P-term oscillation
        let noise_freq = 300.0; // Hz - motor noise

        let gyro_x = 10.0 * (2.0 * std::f32::consts::PI * base_freq * t).sin()
            + 2.0 * (2.0 * std::f32::consts::PI * osc_freq * t).sin()
            + 0.5 * (2.0 * std::f32::consts::PI * noise_freq * t).sin();

        let gyro_y = 8.0 * (2.0 * std::f32::consts::PI * base_freq * t + 0.5).sin()
            + 1.5 * (2.0 * std::f32::consts::PI * osc_freq * t + 0.3).sin()
            + 0.3 * (2.0 * std::f32::consts::PI * noise_freq * t + 0.7).sin();

        let gyro_z = 5.0 * (2.0 * std::f32::consts::PI * base_freq * t + 1.0).sin()
            + 1.0 * (2.0 * std::f32::consts::PI * osc_freq * t + 1.5).sin()
            + 0.2 * (2.0 * std::f32::consts::PI * noise_freq * t + 2.0).sin();

        gyro.push(Vector3::new(gyro_x, gyro_y, gyro_z));

        // Accelerometer typically has lower frequency content
        let accel_x = 2.0 * (2.0 * std::f32::consts::PI * base_freq * t).sin()
            + 0.1 * (2.0 * std::f32::consts::PI * 50.0 * t).sin();
        let accel_y = 2.0 * (2.0 * std::f32::consts::PI * base_freq * t + 0.5).sin()
            + 0.1 * (2.0 * std::f32::consts::PI * 50.0 * t + 0.3).sin();
        let accel_z = 9.81 + 1.0 * (2.0 * std::f32::consts::PI * base_freq * t).sin(); // 1g + movement

        accel.push(Vector3::new(accel_x, accel_y, accel_z));
    }

    TelemetryData {
        sample_rate,
        gyro,
        accel,
        motor: Vec::new(),
        pid_error: drone_tuner_core::domain::PidErrorTrace {
            roll: vec![0.0; sample_count],
            pitch: vec![0.0; sample_count],
            yaw: vec![0.0; sample_count],
        },
        rc_commands: drone_tuner_core::domain::RcCommandTrace {
            roll: vec![0.0; sample_count],
            pitch: vec![0.0; sample_count],
            yaw: vec![0.0; sample_count],
            throttle: vec![0.5; sample_count],
        },
        loop_time_variance: 0.0,
        cpu_load: Vec::new(),
    }
}

/// Create a minimal flight session for benchmarking
fn create_benchmark_session(sample_count: usize, sample_rate: f32) -> FlightSession {
    let telemetry = generate_synthetic_telemetry(sample_count, sample_rate);

    FlightSession {
        metadata: FlightMetadata {
            session_id: Uuid::new_v4(),
            timestamp: Utc::now(),
            duration_ms: (sample_count as f32 / sample_rate * 1000.0) as u64,
            hardware: BenchmarkHardwareConfiguration::default().into(),
            environment: EnvironmentalConditions {
                temperature_c: Some(25.0),
                wind_speed_ms: None,
                wind_direction_deg: None,
                pressure_hpa: None,
                humidity_percent: None,
            },
            pilot: PilotProfile {
                pilot_id: None,
                skill_level: SkillLevel::Intermediate,
                flying_style: FlyingStyle::Freestyle,
            },
        },
        telemetry,
        events: Vec::new(),
        analysis_results: None,
    }
}

/// Benchmark FFT analysis with different data sizes
fn bench_fft_analysis_scaling(c: &mut Criterion) {
    let mut group = c.benchmark_group("fft_analysis_scaling");

    // Test different flight durations at 1kHz sample rate
    let sample_rate = 1000.0;
    let durations = [1, 5, 10, 30, 60]; // seconds

    for &duration_s in &durations {
        let sample_count = (duration_s as f32 * sample_rate) as usize;
        let session = create_benchmark_session(sample_count, sample_rate);

        group.throughput(Throughput::Elements(sample_count as u64));
        group.bench_with_input(
            BenchmarkId::new("duration_seconds", duration_s),
            &session,
            |b, session| {
                let mut engine = AnalysisEngine::new();
                b.iter(|| {
                    let result = engine.analyze(std::hint::black_box(session));
                    std::hint::black_box(result)
                });
            },
        );
    }

    group.finish();
}

/// Benchmark FFT analysis with different sample rates
fn bench_fft_analysis_sample_rates(c: &mut Criterion) {
    let mut group = c.benchmark_group("fft_analysis_sample_rates");

    // Fixed 10-second flights at different sample rates
    let duration_s = 10;
    let sample_rates = [500.0, 1000.0, 2000.0, 4000.0, 8000.0];

    for &sample_rate in &sample_rates {
        let sample_count = (duration_s as f32 * sample_rate) as usize;
        let session = create_benchmark_session(sample_count, sample_rate);

        group.throughput(Throughput::Elements(sample_count as u64));
        group.bench_with_input(
            BenchmarkId::new("sample_rate_hz", sample_rate as u32),
            &session,
            |b, session| {
                let mut engine = AnalysisEngine::new();
                b.iter(|| {
                    let result = engine.analyze(std::hint::black_box(session));
                    std::hint::black_box(result)
                });
            },
        );
    }

    group.finish();
}

/// Benchmark FFT window sizes
fn bench_fft_window_sizes(c: &mut Criterion) {
    let mut group = c.benchmark_group("fft_window_sizes");
    group.measurement_time(Duration::from_secs(10));

    // Test different FFT window sizes
    let window_sizes = [512, 1024, 2048, 4096, 8192];
    let session = create_benchmark_session(50000, 1000.0); // 50 second flight

    for &window_size in &window_sizes {
        group.bench_with_input(
            BenchmarkId::new("window_size", window_size),
            &window_size,
            |b, &window_size| {
                let mut engine =
                    AnalysisEngine::with_config(drone_tuner_core::analysis::AnalysisConfig {
                        fft_window_size: window_size,
                        ..Default::default()
                    });
                b.iter(|| {
                    let result = engine.analyze(std::hint::black_box(&session));
                    std::hint::black_box(result)
                });
            },
        );
    }

    group.finish();
}

/// Benchmark memory usage patterns for large datasets
fn bench_memory_efficiency(c: &mut Criterion) {
    let mut group = c.benchmark_group("memory_efficiency");
    group.measurement_time(Duration::from_secs(15));

    // Test very large flights that might stress memory allocation
    let large_flights = [
        (100_000, 1000.0), // 100 second flight at 1kHz
        (200_000, 1000.0), // 200 second flight at 1kHz
        (100_000, 2000.0), // 100 second flight at 2kHz
    ];

    for (sample_count, sample_rate) in large_flights {
        let session = create_benchmark_session(sample_count, sample_rate);

        group.throughput(Throughput::Bytes((sample_count * 12) as u64)); // 3 axes * 4 bytes per sample
        group.bench_with_input(
            BenchmarkId::new(
                "samples_rate",
                format!("{}_{}", sample_count, sample_rate as u32),
            ),
            &session,
            |b, session| {
                b.iter(|| {
                    let mut engine = AnalysisEngine::new();
                    let result = engine.analyze(std::hint::black_box(session));
                    std::hint::black_box(result)
                });
            },
        );
    }

    group.finish();
}

/// Benchmark peak detection algorithm
fn bench_peak_detection(c: &mut Criterion) {
    let mut group = c.benchmark_group("peak_detection");

    // Create data with many peaks to stress peak detection
    let sample_count = 100_000;
    let mut engine = AnalysisEngine::new();
    let session = create_benchmark_session(sample_count, 1000.0);

    group.bench_function("peak_detection", |b| {
        b.iter(|| {
            let result = engine.analyze(std::hint::black_box(&session));
            std::hint::black_box(result)
        });
    });

    group.finish();
}

/// Benchmark concurrent analysis (simulating multiple files)
fn bench_concurrent_analysis(c: &mut Criterion) {
    let mut group = c.benchmark_group("concurrent_analysis");
    group.measurement_time(Duration::from_secs(20));

    // Create multiple sessions to analyze concurrently
    let sessions: Vec<_> = (0..4)
        .map(|_| create_benchmark_session(50_000, 1000.0))
        .collect();

    group.bench_function("sequential", |b| {
        b.iter(|| {
            for session in std::hint::black_box(&sessions) {
                let mut engine = AnalysisEngine::new();
                let result = engine.analyze(session);
                let _ = std::hint::black_box(result);
            }
        });
    });

    // Parallel analysis (if we add parallel processing support)
    group.bench_function("parallel_potential", |b| {
        b.iter(|| {
            // This would be implemented if we add parallel processing
            // For now, it's the same as sequential to establish baseline
            for session in std::hint::black_box(&sessions) {
                let mut engine = AnalysisEngine::new();
                let result = engine.analyze(session);
                let _ = std::hint::black_box(result);
            }
        });
    });

    group.finish();
}

/// Benchmark filter optimization algorithm
fn bench_filter_optimization(c: &mut Criterion) {
    let mut group = c.benchmark_group("filter_optimization");

    let session = create_benchmark_session(50_000, 1000.0);

    group.bench_function("full_analysis", |b| {
        b.iter(|| {
            let mut engine = AnalysisEngine::new();
            let result = engine.analyze(std::hint::black_box(&session));
            std::hint::black_box(result)
        });
    });

    group.finish();
}

// Benchmark-specific implementation to avoid orphan rule violation
struct BenchmarkHardwareConfiguration(HardwareConfiguration);

impl Default for BenchmarkHardwareConfiguration {
    fn default() -> Self {
        use drone_tuner_core::domain::*;

        Self(HardwareConfiguration {
            flight_controller: FlightController {
                firmware: "Betaflight".to_string(),
                version: "4.4.0".to_string(),
                target: "STM32F405".to_string(),
                loop_rate: 8000,
            },
            frame: Frame {
                wheelbase_mm: 220,
                weight_g: 650,
                material: "Carbon Fiber".to_string(),
                moment_of_inertia: None,
            },
            propulsion: PropulsionSystem {
                motors: MotorSpec {
                    model: "Benchmark".to_string(),
                    kv: 2300,
                    stator_size: "2207".to_string(),
                },
                props: PropellerSpec {
                    diameter_inches: 5.0,
                    pitch_inches: 4.3,
                    blade_count: 3,
                    material: "Polycarbonate".to_string(),
                },
                esc: EscSpec {
                    model: "Benchmark".to_string(),
                    current_rating: 35,
                    protocol: "DShot600".to_string(),
                },
            },
            pid_config: PidConfiguration {
                roll: PidValues {
                    p: 42.0,
                    i: 85.0,
                    d: 38.0,
                    f: Some(147.0),
                },
                pitch: PidValues {
                    p: 46.0,
                    i: 90.0,
                    d: 42.0,
                    f: Some(157.0),
                },
                yaw: PidValues {
                    p: 45.0,
                    i: 90.0,
                    d: 0.0,
                    f: Some(147.0),
                },
                settings: PidSettings {
                    tpa: Some(TpaSettings {
                        rate: 0.65,
                        breakpoint: 1350.0,
                    }),
                    profile: 1,
                    rates: RateSettings {
                        roll_rate: 670.0,
                        pitch_rate: 670.0,
                        yaw_rate: 670.0,
                        expo: ExpoSettings {
                            roll: 0.0,
                            pitch: 0.0,
                            yaw: 0.0,
                        },
                        super_rate: SuperRateSettings {
                            roll: 0.80,
                            pitch: 0.80,
                            yaw: 0.80,
                        },
                    },
                },
            },
            filter_config: FilterConfiguration {
                gyro_filters: vec![Filter {
                    filter_type: FilterType::LowPass,
                    cutoff: 250.0,
                    order: 2,
                }],
                dterm_filters: vec![Filter {
                    filter_type: FilterType::LowPass,
                    cutoff: 100.0,
                    order: 2,
                }],
                notch_filters: vec![],
                dynamic_notch: Some(DynamicNotchSettings {
                    min_freq: 150.0,
                    max_freq: 600.0,
                    q_factor: 120.0,
                    enabled: true,
                }),
            },
        })
    }
}

impl From<BenchmarkHardwareConfiguration> for HardwareConfiguration {
    fn from(bench_config: BenchmarkHardwareConfiguration) -> Self {
        bench_config.0
    }
}

criterion_group!(
    benches,
    bench_fft_analysis_scaling,
    bench_fft_analysis_sample_rates,
    bench_fft_window_sizes,
    bench_memory_efficiency,
    bench_peak_detection,
    bench_concurrent_analysis,
    bench_filter_optimization
);

criterion_main!(benches);
