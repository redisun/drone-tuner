//! Benchmarks for blackbox parsing performance.

use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};
use drone_tuner_core::BlackboxParser;

/// Generate synthetic blackbox data for benchmarking
fn generate_synthetic_blackbox_data(frame_count: usize, fields_per_frame: usize) -> Vec<u8> {
    let mut data = Vec::new();

    // Add a minimal header
    data.extend_from_slice(b"H Product:Betaflight\n");
    data.extend_from_slice(b"H Data version:2\n");
    data.extend_from_slice(b"H I interval:125\n");
    data.extend_from_slice(b"H P interval:1\n");
    data.extend_from_slice(b"H Field I name:loopIteration\n");
    data.extend_from_slice(b"H Field I name:time\n");
    data.extend_from_slice(b"H Field I name:gyroADC[0]\n");
    data.extend_from_slice(b"H Field I name:gyroADC[1]\n");
    data.extend_from_slice(b"H Field I name:gyroADC[2]\n");
    data.extend_from_slice(b"H Field I name:motor[0]\n");
    data.extend_from_slice(b"H Field I name:motor[1]\n");
    data.extend_from_slice(b"H Field I name:motor[2]\n");
    data.extend_from_slice(b"H Field I name:motor[3]\n");

    // Generate I-frame every 125 frames, P-frames in between
    for i in 0..frame_count {
        if i % 125 == 0 {
            // I-frame with full data
            data.push(b'I');

            // Generate synthetic sensor data
            for field_idx in 0..fields_per_frame.min(9) {
                match field_idx {
                    0 => data.extend_from_slice(&encode_signed_vb(i as i32)), // loop iteration
                    1 => data.extend_from_slice(&encode_signed_vb((i * 125) as i32)), // time
                    2..=4 => {
                        // Gyro data - simulate oscillating values
                        let gyro_val = (1000.0 * (i as f32 * 0.1).sin()) as i32;
                        data.extend_from_slice(&encode_signed_vb(gyro_val));
                    }
                    5..=8 => {
                        // Motor data - simulate throttle values
                        let motor_val = 1000 + (200.0 * (i as f32 * 0.05).sin()) as i32;
                        data.extend_from_slice(&encode_signed_vb(motor_val));
                    }
                    _ => data.extend_from_slice(&encode_signed_vb(0)),
                }
            }
        } else {
            // P-frame with delta data
            data.push(b'P');

            // Generate small deltas
            for field_idx in 0..fields_per_frame.min(9) {
                let delta = match field_idx {
                    0 => 1,   // loop iteration increment
                    1 => 125, // time increment
                    2..=4 => {
                        // Small gyro changes
                        ((10.0 * (i as f32 * 0.2).cos()) as i32).clamp(-50, 50)
                    }
                    5..=8 => {
                        // Small motor changes
                        ((5.0 * (i as f32 * 0.1).sin()) as i32).clamp(-20, 20)
                    }
                    _ => 0,
                };
                data.extend_from_slice(&encode_signed_vb(delta));
            }
        }
    }

    data
}

/// Encode signed integer using variable-byte encoding (simplified)
fn encode_signed_vb(value: i32) -> Vec<u8> {
    if value >= -64 && value <= 63 {
        // Single byte encoding
        if value >= 0 {
            vec![value as u8]
        } else {
            vec![0x40 | ((-value) as u8)]
        }
    } else {
        // Multi-byte encoding (simplified)
        vec![0x80 | (value as u8 & 0x7F), ((value >> 7) as u8) & 0x7F]
    }
}

/// Generate compressed blackbox data
fn generate_compressed_blackbox_data(frame_count: usize) -> Vec<u8> {
    let uncompressed = generate_synthetic_blackbox_data(frame_count, 9);

    // Use flate2 to compress the data
    use flate2::write::GzEncoder;
    use flate2::Compression;
    use std::io::Write;

    let mut encoder = GzEncoder::new(Vec::new(), Compression::default());
    encoder.write_all(&uncompressed).unwrap();
    encoder.finish().unwrap()
}

/// Benchmark parsing performance with different file sizes
fn bench_parsing_file_sizes(c: &mut Criterion) {
    let mut group = c.benchmark_group("parsing_file_sizes");

    // Test different flight durations (frames)
    let frame_counts = [1000, 5000, 10000, 50000, 100000]; // ~1s to ~100s flights

    for &frame_count in &frame_counts {
        let data = generate_synthetic_blackbox_data(frame_count, 9);

        group.throughput(Throughput::Bytes(data.len() as u64));
        group.bench_with_input(BenchmarkId::new("frames", frame_count), &data, |b, data| {
            b.iter(|| {
                let mut parser = BlackboxParser::new();
                let result = parser.parse_file(std::hint::black_box(data));
                std::hint::black_box(result)
            });
        });
    }

    group.finish();
}

/// Benchmark compressed vs uncompressed parsing
fn bench_compression_impact(c: &mut Criterion) {
    let mut group = c.benchmark_group("compression_impact");

    let frame_count = 50000; // ~50 second flight
    let uncompressed_data = generate_synthetic_blackbox_data(frame_count, 9);
    let compressed_data = generate_compressed_blackbox_data(frame_count);

    group.throughput(Throughput::Elements(frame_count as u64));

    group.bench_function("uncompressed", |b| {
        b.iter(|| {
            let mut parser = BlackboxParser::new();
            let result = parser.parse_file(std::hint::black_box(&uncompressed_data));
            std::hint::black_box(result)
        });
    });

    group.bench_function("compressed", |b| {
        b.iter(|| {
            let mut parser = BlackboxParser::new();
            let result = parser.parse_file(std::hint::black_box(&compressed_data));
            std::hint::black_box(result)
        });
    });

    // Show compression ratio
    let compression_ratio = compressed_data.len() as f32 / uncompressed_data.len() as f32;
    println!(
        "Compression ratio: {:.2}% ({} -> {} bytes)",
        compression_ratio * 100.0,
        uncompressed_data.len(),
        compressed_data.len()
    );

    group.finish();
}

/// Benchmark parsing with different numbers of fields
fn bench_field_count_impact(c: &mut Criterion) {
    let mut group = c.benchmark_group("field_count_impact");

    let frame_count = 25000; // Fixed flight length
    let field_counts = [3, 6, 9, 12, 15]; // Different numbers of logged fields

    for &field_count in &field_counts {
        let data = generate_synthetic_blackbox_data(frame_count, field_count);

        group.throughput(Throughput::Elements((frame_count * field_count) as u64));
        group.bench_with_input(BenchmarkId::new("fields", field_count), &data, |b, data| {
            b.iter(|| {
                let mut parser = BlackboxParser::new();
                let result = parser.parse_file(std::hint::black_box(data));
                std::hint::black_box(result)
            });
        });
    }

    group.finish();
}

/// Benchmark parser memory allocation patterns
fn bench_memory_allocation(c: &mut Criterion) {
    let mut group = c.benchmark_group("memory_allocation");
    group.sample_size(20); // Fewer samples for memory-intensive tests

    // Test different frame counts to see allocation scaling
    let frame_counts = [10000, 25000, 50000, 100000];

    for &frame_count in &frame_counts {
        let data = generate_synthetic_blackbox_data(frame_count, 9);

        group.bench_with_input(
            BenchmarkId::new("allocation_scaling", frame_count),
            &data,
            |b, data| {
                b.iter(|| {
                    let mut parser = BlackboxParser::new();
                    let result = parser.parse_file(std::hint::black_box(data));
                    std::hint::black_box(result)
                });
            },
        );
    }

    group.finish();
}

/// Benchmark parsing with corrupted data (error handling)
fn bench_error_handling(c: &mut Criterion) {
    let mut group = c.benchmark_group("error_handling");

    let base_data = generate_synthetic_blackbox_data(10000, 9);

    // Create various corruption scenarios
    let test_cases = [
        ("valid", base_data.clone()),
        ("truncated", base_data[..base_data.len() / 2].to_vec()),
        ("corrupted_header", {
            let mut corrupted = base_data.clone();
            // Corrupt the header
            for i in 0..50.min(corrupted.len()) {
                corrupted[i] = 0xFF;
            }
            corrupted
        }),
        ("corrupted_middle", {
            let mut corrupted = base_data.clone();
            // Corrupt middle section
            let start = corrupted.len() / 2;
            let end = (start + 100).min(corrupted.len());
            for i in start..end {
                corrupted[i] = 0xFF;
            }
            corrupted
        }),
    ];

    for (name, data) in test_cases {
        group.bench_with_input(
            BenchmarkId::new("corruption_type", name),
            &data,
            |b, data| {
                b.iter(|| {
                    let mut parser = BlackboxParser::new();
                    let result = parser.parse_file(std::hint::black_box(data));
                    std::hint::black_box(result)
                });
            },
        );
    }

    group.finish();
}

/// Benchmark multiple file parsing (simulating batch processing)
fn bench_batch_parsing(c: &mut Criterion) {
    let mut group = c.benchmark_group("batch_parsing");
    group.sample_size(10); // Fewer samples for batch tests

    // Create multiple files of different sizes
    let files = vec![
        generate_synthetic_blackbox_data(5000, 9),  // Small flight
        generate_synthetic_blackbox_data(15000, 9), // Medium flight
        generate_synthetic_blackbox_data(30000, 9), // Large flight
        generate_synthetic_blackbox_data(2000, 9),  // Very small flight
        generate_synthetic_blackbox_data(25000, 9), // Another medium flight
    ];

    let total_size: usize = files.iter().map(|f| f.len()).sum();
    group.throughput(Throughput::Bytes(total_size as u64));

    group.bench_function("sequential_batch", |b| {
        b.iter(|| {
            for file_data in std::hint::black_box(&files) {
                let mut parser = BlackboxParser::new();
                let result = parser.parse_file(file_data);
                let _ = std::hint::black_box(result);
            }
        });
    });

    // Test reusing parser (if we implement parser reuse)
    group.bench_function("reused_parser", |b| {
        b.iter(|| {
            let mut parser = BlackboxParser::new();
            for file_data in std::hint::black_box(&files) {
                let result = parser.parse_file(file_data);
                let _ = std::hint::black_box(result);
                // In a real implementation, we'd have parser.reset() here
            }
        });
    });

    group.finish();
}

/// Benchmark variable-byte integer parsing (core parsing primitive)
fn bench_variable_byte_parsing(c: &mut Criterion) {
    let mut group = c.benchmark_group("variable_byte_parsing");

    // Generate arrays of different variable-byte encoded integers
    let test_data = vec![
        (
            "single_byte",
            (0..10000)
                .flat_map(|i| encode_signed_vb(i % 64))
                .collect::<Vec<u8>>(),
        ),
        (
            "two_byte",
            (0..10000)
                .flat_map(|i| encode_signed_vb(i * 100))
                .collect::<Vec<u8>>(),
        ),
        (
            "mixed",
            (0..10000)
                .flat_map(|i| encode_signed_vb(if i % 3 == 0 { i % 64 } else { i * 100 }))
                .collect::<Vec<u8>>(),
        ),
    ];

    for (name, data) in test_data {
        group.throughput(Throughput::Bytes(data.len() as u64));
        group.bench_with_input(BenchmarkId::new("vb_type", name), &data, |b, data| {
            b.iter(|| {
                let mut parser = BlackboxParser::new();
                let result = parser.parse_file(std::hint::black_box(data));
                std::hint::black_box(result)
            });
        });
    }

    group.finish();
}

criterion_group!(
    benches,
    bench_parsing_file_sizes,
    bench_compression_impact,
    bench_field_count_impact,
    bench_memory_allocation,
    bench_error_handling,
    bench_batch_parsing,
    bench_variable_byte_parsing
);

criterion_main!(benches);
