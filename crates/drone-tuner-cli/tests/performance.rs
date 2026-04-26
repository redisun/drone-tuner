//! Performance and stress tests for CLI commands.

mod common;

use common::{create_large_blackbox_data, create_test_command, run_with_timeout, TestFiles};
use predicates::prelude::*;
use std::fs;
use std::time::Instant;
use tempfile::TempDir;

/// Performance benchmarks for CLI commands
mod performance_benchmarks {
    use super::*;

    #[test]
    fn test_analyze_small_file_performance() {
        let test_files = TestFiles::new();

        let start = Instant::now();
        create_test_command()
            .arg("analyze")
            .arg(&test_files.valid_file)
            .assert()
            .success();
        let duration = start.elapsed();

        // Small file should analyze very quickly
        assert!(
            duration.as_secs() < 5,
            "Small file analysis took too long: {:?}",
            duration
        );
        println!("Small file analysis time: {:?}", duration);
    }

    #[test]
    fn test_analyze_medium_file_performance() {
        let temp_dir = TempDir::new().unwrap();
        let medium_file = temp_dir.path().join("medium.bbl");

        // Create medium-sized file (5K frames)
        let medium_data = create_large_blackbox_data(5000);
        fs::write(&medium_file, medium_data).unwrap();

        let start = Instant::now();
        create_test_command()
            .arg("analyze")
            .arg(&medium_file)
            .assert()
            .success();
        let duration = start.elapsed();

        // Medium file should complete within reasonable time
        assert!(
            duration.as_secs() < 15,
            "Medium file analysis took too long: {:?}",
            duration
        );
        println!("Medium file analysis time: {:?}", duration);
    }

    #[test]
    fn test_analyze_large_file_performance() {
        let temp_dir = TempDir::new().unwrap();
        let large_file = temp_dir.path().join("large.bbl");

        // Create large file (20K frames)
        let large_data = create_large_blackbox_data(20000);
        fs::write(&large_file, large_data).unwrap();

        let start = Instant::now();
        {
            let mut cmd = create_test_command();
            cmd.arg("analyze").arg(&large_file);
            run_with_timeout(cmd, 60)
        }
        .success();
        let duration = start.elapsed();

        // Large file should complete within timeout
        assert!(
            duration.as_secs() < 60,
            "Large file analysis took too long: {:?}",
            duration
        );
        println!("Large file analysis time: {:?}", duration);
    }

    #[test]
    fn test_batch_processing_performance() {
        let test_files = TestFiles::new();
        let batch_files = test_files.create_batch_files(10);

        // Copy all files to a batch directory
        let batch_dir = test_files.temp_dir.path().join("batch_perf");
        fs::create_dir(&batch_dir).unwrap();

        for (i, file) in batch_files.iter().enumerate() {
            fs::copy(file, batch_dir.join(format!("batch_{}.bbl", i))).unwrap();
        }

        let start = Instant::now();
        create_test_command()
            .arg("analyze")
            .arg(&batch_dir)
            .assert()
            .success();
        let duration = start.elapsed();

        // Batch processing should scale reasonably
        assert!(
            duration.as_secs() < 30,
            "Batch processing took too long: {:?}",
            duration
        );
        println!("Batch processing time for 10 files: {:?}", duration);
    }

    #[test]
    fn test_export_performance() {
        let test_files = TestFiles::new();
        let output_file = test_files.temp_dir.path().join("perf_export.csv");

        let start = Instant::now();
        create_test_command()
            .arg("export")
            .arg(&test_files.valid_file)
            .arg("--output")
            .arg(&output_file)
            .arg("--format")
            .arg("csv")
            .arg("--include-raw")
            .assert()
            .success();
        let duration = start.elapsed();

        // Export should be fast
        assert!(
            duration.as_secs() < 10,
            "Export took too long: {:?}",
            duration
        );
        println!("Export time: {:?}", duration);
    }

    #[test]
    fn test_validate_performance() {
        let test_files = TestFiles::new();
        let batch_files = test_files.create_batch_files(20);

        let batch_dir = test_files.temp_dir.path().join("validate_perf");
        fs::create_dir(&batch_dir).unwrap();

        for (i, file) in batch_files.iter().enumerate() {
            fs::copy(file, batch_dir.join(format!("validate_{}.bbl", i))).unwrap();
        }

        let start = Instant::now();
        create_test_command()
            .arg("validate")
            .arg(&batch_dir)
            .arg("--check-issues")
            .assert()
            .success();
        let duration = start.elapsed();

        // Validation should be efficient
        assert!(
            duration.as_secs() < 20,
            "Validation took too long: {:?}",
            duration
        );
        println!("Validation time for 20 files: {:?}", duration);
    }

    #[test]
    fn test_compare_performance() {
        let test_files = TestFiles::new();

        let start = Instant::now();
        create_test_command()
            .arg("compare")
            .arg(&test_files.valid_file)
            .arg(&test_files.oscillating_file)
            .arg(&test_files.valid_file) // Compare 3 files
            .assert()
            .success();
        let duration = start.elapsed();

        // Comparison should be reasonably fast
        assert!(
            duration.as_secs() < 20,
            "Comparison took too long: {:?}",
            duration
        );
        println!("Comparison time for 3 files: {:?}", duration);
    }
}

/// Memory usage and resource management tests
mod memory_tests {
    use super::*;

    #[test]
    fn test_memory_efficiency_batch_processing() {
        let test_files = TestFiles::new();

        // Process many files sequentially to test memory cleanup
        let batch_files = test_files.create_batch_files(50);
        let batch_dir = test_files.temp_dir.path().join("memory_test");
        fs::create_dir(&batch_dir).unwrap();

        for (i, file) in batch_files.iter().enumerate() {
            fs::copy(file, batch_dir.join(format!("mem_{}.bbl", i))).unwrap();
        }

        // This should not run out of memory or crash
        {
            let mut cmd = create_test_command();
            cmd.arg("analyze").arg(&batch_dir);
            run_with_timeout(cmd, 120)
        }
        .success()
        .stderr(predicate::str::contains("Found 50 blackbox file(s)"));
    }

    #[test]
    fn test_large_file_memory_handling() {
        let temp_dir = TempDir::new().unwrap();
        let very_large_file = temp_dir.path().join("very_large.bbl");

        // Create very large file (100K frames)
        let very_large_data = create_large_blackbox_data(100000);
        fs::write(&very_large_file, very_large_data).unwrap();

        // Should handle large file without excessive memory usage
        {
            let mut cmd = create_test_command();
            cmd.arg("analyze").arg(&very_large_file);
            run_with_timeout(cmd, 180)
        }
        .success();
    }

    #[test]
    fn test_concurrent_memory_usage() {
        let test_files = TestFiles::new();

        // Run multiple analysis operations concurrently
        let handles: Vec<_> = (0..5)
            .map(|i| {
                let file = if i % 2 == 0 {
                    test_files.valid_file.clone()
                } else {
                    test_files.oscillating_file.clone()
                };

                std::thread::spawn(move || {
                    create_test_command()
                        .arg("analyze")
                        .arg(&file)
                        .assert()
                        .success();
                })
            })
            .collect();

        // All should complete without memory issues
        for handle in handles {
            handle.join().unwrap();
        }
    }

    #[test]
    fn test_export_memory_efficiency() {
        let temp_dir = TempDir::new().unwrap();
        let large_file = temp_dir.path().join("large_export.bbl");

        // Create large file for export test
        let large_data = create_large_blackbox_data(10000);
        fs::write(&large_file, large_data).unwrap();

        let output_file = temp_dir.path().join("large_export.csv");

        // Export with raw data should handle memory efficiently
        {
            let mut cmd = create_test_command();
            cmd.arg("export")
                .arg(&large_file)
                .arg("--output")
                .arg(&output_file)
                .arg("--format")
                .arg("csv")
                .arg("--include-raw");
            run_with_timeout(cmd, 60)
        }
        .success();

        // Verify output was created
        assert!(output_file.exists());
    }
}

/// Stress tests for edge cases
mod stress_tests {
    use super::*;

    #[test]
    fn test_max_files_limit_stress() {
        let test_files = TestFiles::new();

        // Create many files to test max-files limit
        let stress_files = test_files.create_batch_files(1000);
        let stress_dir = test_files.temp_dir.path().join("stress_test");
        fs::create_dir(&stress_dir).unwrap();

        for (i, file) in stress_files.iter().take(100).enumerate() {
            fs::copy(file, stress_dir.join(format!("stress_{:03}.bbl", i))).unwrap();
        }

        // Test with various max-files limits
        for max_files in [10, 50, 100] {
            create_test_command()
                .arg("analyze")
                .arg(&stress_dir)
                .arg("--max-files")
                .arg(max_files.to_string())
                .assert()
                .success()
                .stderr(predicate::str::contains(format!(
                    "Found {} blackbox file(s)",
                    max_files.min(100)
                )));
        }
    }

    #[test]
    fn test_deep_directory_recursion() {
        let test_files = TestFiles::new();

        // Create deeply nested directory structure
        let mut current_path = test_files.temp_dir.path().to_path_buf();
        for level in 0..20 {
            current_path.push(format!("level_{}", level));
            fs::create_dir(&current_path).unwrap();

            // Place a file every few levels
            if level % 5 == 0 {
                let file_path = current_path.join(format!("deep_{}.bbl", level));
                fs::copy(&test_files.valid_file, &file_path).unwrap();
            }
        }

        // Should handle deep recursion without stack overflow
        create_test_command()
            .arg("analyze")
            .arg(test_files.temp_dir.path())
            .assert()
            .success();
    }

    #[test]
    fn test_many_small_files_stress() {
        let test_files = TestFiles::new();

        // Create many small files
        let small_files_dir = test_files.temp_dir.path().join("small_files");
        fs::create_dir(&small_files_dir).unwrap();

        for i in 0..200 {
            let small_file = small_files_dir.join(format!("small_{:03}.bbl", i));
            fs::write(&small_file, common::create_minimal_blackbox_data()).unwrap();
        }

        // Should handle many small files efficiently. Pass --max-files
        // explicitly so we exercise the full 200, not the default 100 cap.
        {
            let mut cmd = create_test_command();
            cmd.arg("analyze")
                .arg(&small_files_dir)
                .arg("--max-files")
                .arg("200");
            run_with_timeout(cmd, 120)
        }
        .success()
        .stderr(predicate::str::contains("Found 200 blackbox file(s)"));
    }

    #[test]
    fn test_mixed_file_sizes_stress() {
        let temp_dir = TempDir::new().unwrap();
        let mixed_dir = temp_dir.path().join("mixed_sizes");
        fs::create_dir(&mixed_dir).unwrap();

        // Create files of various sizes
        let sizes = [100, 1000, 5000, 10000, 20000];
        for (i, &size) in sizes.iter().enumerate() {
            let file_path = mixed_dir.join(format!("size_{}_{}.bbl", size, i));
            let data = create_large_blackbox_data(size);
            fs::write(&file_path, data).unwrap();
        }

        // Should handle mixed file sizes efficiently
        {
            let mut cmd = create_test_command();
            cmd.arg("analyze").arg(&mixed_dir);
            run_with_timeout(cmd, 120)
        }
        .success()
        .stderr(predicate::str::contains("Found 5 blackbox file(s)"));
    }

    #[test]
    fn test_output_format_stress() {
        let test_files = TestFiles::new();

        // Test all output formats with large data
        let formats = ["pretty", "json", "csv"];

        for format in &formats {
            create_test_command()
                .arg("--output-format")
                .arg(format)
                .arg("analyze")
                .arg(&test_files.oscillating_file)
                .assert()
                .success();
        }
    }

    #[test]
    fn test_concurrent_different_commands() {
        let test_files = TestFiles::new();

        // Run different commands concurrently
        let handles = vec![
            {
                let file = test_files.valid_file.clone();
                std::thread::spawn(move || {
                    create_test_command()
                        .arg("analyze")
                        .arg(&file)
                        .assert()
                        .success();
                })
            },
            {
                let file = test_files.valid_file.clone();
                std::thread::spawn(move || {
                    create_test_command()
                        .arg("validate")
                        .arg(&file)
                        .assert()
                        .success();
                })
            },
            {
                let files = (
                    test_files.valid_file.clone(),
                    test_files.oscillating_file.clone(),
                );
                std::thread::spawn(move || {
                    create_test_command()
                        .arg("compare")
                        .arg(&files.0)
                        .arg(&files.1)
                        .assert()
                        .success();
                })
            },
        ];

        for handle in handles {
            handle.join().unwrap();
        }
    }
}

/// Regression tests for performance
mod regression_tests {
    use super::*;

    #[test]
    fn test_startup_time_regression() {
        // Test that basic commands start quickly
        let start = Instant::now();
        create_test_command().arg("--help").assert().success();
        let duration = start.elapsed();

        // Help should display very quickly
        assert!(
            duration.as_millis() < 1000,
            "Help command too slow: {:?}",
            duration
        );
    }

    #[test]
    fn test_version_performance() {
        let start = Instant::now();
        create_test_command().arg("--version").assert().success();
        let duration = start.elapsed();

        // Version should be instant
        assert!(
            duration.as_millis() < 500,
            "Version command too slow: {:?}",
            duration
        );
    }

    #[test]
    fn test_info_command_performance() {
        let start = Instant::now();
        create_test_command().arg("info").assert().success();
        let duration = start.elapsed();

        // Info should be very fast
        assert!(
            duration.as_millis() < 2000,
            "Info command too slow: {:?}",
            duration
        );
    }

    #[test]
    fn test_analysis_time_scaling() {
        let temp_dir = TempDir::new().unwrap();

        // Test files of increasing size to ensure linear scaling
        let sizes = [1000, 2000, 4000];
        let mut times = Vec::new();

        for size in &sizes {
            let file_path = temp_dir.path().join(format!("scale_{}.bbl", size));
            let data = create_large_blackbox_data(*size);
            fs::write(&file_path, data).unwrap();

            let start = Instant::now();
            create_test_command()
                .arg("analyze")
                .arg(&file_path)
                .assert()
                .success();
            let duration = start.elapsed();

            times.push(duration);
            println!("Size {} frames: {:?}", size, duration);
        }

        // Verify roughly linear scaling (allowing for measurement variance)
        // 4x size should not take more than 8x time
        let ratio = times[2].as_millis() as f64 / times[0].as_millis() as f64;
        assert!(
            ratio < 8.0,
            "Analysis time scaling is worse than expected: {}x",
            ratio
        );
    }
}
