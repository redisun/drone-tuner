//! Command-specific integration tests with detailed scenarios.

mod common;

use common::{
    create_test_command, run_with_timeout, verify_output_file, CommandTestExt, TestFiles,
};
use predicates::prelude::*;
use std::fs;
use tempfile::TempDir;

/// Test analyze command with various realistic scenarios
mod analyze_scenarios {
    use super::*;

    #[test]
    fn test_analyze_oscillating_data() {
        let test_files = TestFiles::new();

        create_test_command()
            .arg("analyze")
            .arg(&test_files.oscillating_file)
            .assert()
            .success()
            .stdout(predicate::str::contains("Analysis Summary"))
            .stdout(predicate::str::contains("Tune Quality:"));
    }

    #[test]
    fn test_analyze_detailed_output_content() {
        let test_files = TestFiles::new();

        let output = create_test_command()
            .arg("analyze")
            .arg(&test_files.valid_file)
            .arg("--show-details")
            .assert()
            .success()
            .get_output()
            .stdout
            .clone();

        let output_str = String::from_utf8(output).unwrap();

        // Verify detailed output sections
        assert!(output_str.contains("File Details:"));
        assert!(output_str.contains("Flight Controller Configuration:"));
        assert!(output_str.contains("PID Values:"));
        assert!(output_str.contains("Filter Settings:"));
        assert!(output_str.contains("RC Rates:"));
        assert!(output_str.contains("Verification Notes:"));
    }

    #[test]
    fn test_analyze_json_schema_validation() {
        let test_files = TestFiles::new();

        let (_, json) = {
            let mut cmd = create_test_command();
            cmd.arg("--output-format")
                .arg("json")
                .arg("analyze")
                .arg(&test_files.valid_file);
            cmd.assert_json_output()
        };

        // Validate specific JSON schema
        assert!(json["version"].as_str().is_some());
        assert!(json["timestamp"].as_str().is_some());

        let results = json["results"].as_array().unwrap();
        assert!(!results.is_empty());

        let first_result = &results[0];
        assert!(first_result["file"].as_str().is_some());
        assert_eq!(first_result["status"], "success");
        assert!(first_result["tune_quality"].as_f64().is_some());
        assert!(first_result["samples"].as_u64().is_some());
        assert!(first_result["sample_rate"].as_f64().is_some());
    }

    #[test]
    fn test_analyze_csv_format_validation() {
        let test_files = TestFiles::new();

        let (_, csv_rows) = {
            let mut cmd = create_test_command();
            cmd.arg("--output-format")
                .arg("csv")
                .arg("analyze")
                .arg(&test_files.valid_file);
            cmd.assert_csv_output()
        };

        // Validate CSV structure
        assert!(csv_rows.len() >= 2); // Header + at least one data row

        let header = &csv_rows[0];
        let expected_columns = [
            "file",
            "status",
            "tune_quality",
            "duration_ms",
            "sample_rate",
            "samples",
            "analysis_time_ms",
            "issues",
            "filter_recommendations",
            "pid_recommendations",
        ];

        for expected_col in &expected_columns {
            assert!(
                header.contains(&expected_col.to_string()),
                "CSV header should contain '{}', got: {:?}",
                expected_col,
                header
            );
        }

        // Validate data row
        if csv_rows.len() > 1 {
            let data_row = &csv_rows[1];
            assert_eq!(
                data_row.len(),
                header.len(),
                "Data row should have same column count as header"
            );
            assert_eq!(data_row[1], "success"); // Status should be success
        }
    }

    #[test]
    fn test_analyze_batch_processing_mixed_files() {
        let test_files = TestFiles::new();

        // Create a mix of valid and invalid files
        let batch_dir = test_files.temp_dir.path().join("batch");
        fs::create_dir(&batch_dir).unwrap();

        // Copy valid file
        fs::copy(&test_files.valid_file, batch_dir.join("valid.bbl")).unwrap();
        // Copy corrupted file
        fs::copy(&test_files.corrupted_file, batch_dir.join("corrupted.bbl")).unwrap();
        // Copy empty file
        fs::copy(&test_files.empty_file, batch_dir.join("empty.bbl")).unwrap();

        let assert = create_test_command()
            .arg("analyze")
            .arg(&batch_dir)
            .assert()
            .success();
        let output = assert.get_output();

        let stdout = String::from_utf8(output.stdout.clone()).unwrap();
        let stderr = String::from_utf8(output.stderr.clone()).unwrap();

        // "Found N blackbox file(s)" is a status message → stderr.
        // The "Analysis Summary" block stays on stdout for pretty mode.
        assert!(stderr.contains("Found 3 blackbox file(s)"));
        assert!(stdout.contains("Analysis Summary"));
        assert!(stdout.contains("Successful:"));
        assert!(stdout.contains("Failed:"));
    }

    #[test]
    fn test_analyze_session_strategy_behavior() {
        let test_files = TestFiles::new();

        for strategy in ["first", "last", "longest"] {
            create_test_command()
                .arg("analyze")
                .arg(&test_files.valid_file)
                .arg("--session-strategy")
                .arg(strategy)
                .assert()
                .success()
                .stdout(predicate::str::contains("Analysis Summary"));
        }
    }

    #[test]
    fn test_analyze_confidence_threshold_effects() {
        let test_files = TestFiles::new();

        for confidence in ["0.1", "0.5", "0.9"] {
            create_test_command()
                .arg("analyze")
                .arg(&test_files.valid_file)
                .arg("--min-confidence")
                .arg(confidence)
                .assert()
                .success();
        }
    }

    #[test]
    fn test_analyze_large_file_handling() {
        let temp_dir = TempDir::new().unwrap();
        let large_file = temp_dir.path().join("large.bbl");

        // Create a larger test file (10K frames)
        let large_data = common::create_large_blackbox_data(10000);
        fs::write(&large_file, large_data).unwrap();

        // Test with timeout to ensure it doesn't hang
        {
            let mut cmd = create_test_command();
            cmd.arg("analyze").arg(&large_file);
            run_with_timeout(cmd, 30)
        }
        .success()
        .stdout(predicate::str::contains("Analysis Summary"));
    }
}

/// Test export command with all formats and options
mod export_scenarios {
    use super::*;

    #[test]
    fn test_export_all_formats() {
        let test_files = TestFiles::new();
        let formats = [
            ("csv", "FPV Drone Tuner Analysis Export"),
            ("json", "file_path"),
            ("matlab", "% FPV Drone Tuner Analysis Export"),
            ("python", "# FPV Drone Tuner Analysis Export"),
        ];

        for (format, expected_content) in &formats {
            let output_file = test_files
                .temp_dir
                .path()
                .join(format!("export.{}", format));

            create_test_command()
                .arg("export")
                .arg(&test_files.valid_file)
                .arg("--output")
                .arg(&output_file)
                .arg("--format")
                .arg(format)
                .assert()
                .success()
                .stdout(predicate::str::contains("Export completed"));

            verify_output_file(&output_file, Some(100)); // At least 100 bytes

            let content = fs::read_to_string(&output_file).unwrap();
            assert!(
                content.contains(expected_content),
                "Export format {} should contain '{}', got first 200 chars: {}",
                format,
                expected_content,
                &content[..content.len().min(200)]
            );
        }
    }

    #[test]
    fn test_export_with_raw_data() {
        let test_files = TestFiles::new();
        let output_file = test_files.temp_dir.path().join("with_raw.csv");

        create_test_command()
            .arg("export")
            .arg(&test_files.oscillating_file) // Use oscillating data for more realistic test
            .arg("--output")
            .arg(&output_file)
            .arg("--format")
            .arg("csv")
            .arg("--include-raw")
            .assert()
            .success();

        let content = fs::read_to_string(&output_file).unwrap();
        assert!(content.contains("Raw Gyro Data"));
        assert!(content.contains("time,gyro_x,gyro_y,gyro_z"));
    }

    #[test]
    fn test_export_json_structure() {
        let test_files = TestFiles::new();
        let output_file = test_files.temp_dir.path().join("export.json");

        create_test_command()
            .arg("export")
            .arg(&test_files.valid_file)
            .arg("--output")
            .arg(&output_file)
            .arg("--format")
            .arg("json")
            .arg("--include-raw")
            .assert()
            .success();

        let content = fs::read_to_string(&output_file).unwrap();
        let json: serde_json::Value = serde_json::from_str(&content).unwrap();

        // Validate JSON export structure
        assert!(json["file_path"].as_str().is_some());
        assert!(json["analysis_time_s"].as_f64().is_some());
        assert!(json["tune_quality"].as_f64().is_some());
        assert!(json["pid_recommendations"].as_array().is_some());
        assert!(json["gyro_data"].as_array().is_some()); // Because we included raw data
    }

    #[test]
    fn test_export_matlab_syntax() {
        let test_files = TestFiles::new();
        let output_file = test_files.temp_dir.path().join("export.m");

        create_test_command()
            .arg("export")
            .arg(&test_files.valid_file)
            .arg("--output")
            .arg(&output_file)
            .arg("--format")
            .arg("matlab")
            .arg("--include-raw")
            .assert()
            .success();

        let content = fs::read_to_string(&output_file).unwrap();

        // Validate MATLAB syntax
        assert!(content.contains("tune_quality = "));
        assert!(content.contains("sample_rate = "));
        assert!(content.contains("gyro_data = ["));
        assert!(content.contains("];"));
        assert!(content.contains("t = "));
    }

    #[test]
    fn test_export_python_syntax() {
        let test_files = TestFiles::new();
        let output_file = test_files.temp_dir.path().join("export.py");

        create_test_command()
            .arg("export")
            .arg(&test_files.valid_file)
            .arg("--output")
            .arg(&output_file)
            .arg("--format")
            .arg("python")
            .arg("--include-raw")
            .assert()
            .success();

        let content = fs::read_to_string(&output_file).unwrap();

        // Validate Python syntax
        assert!(content.contains("import numpy as np"));
        assert!(content.contains("import matplotlib.pyplot as plt"));
        assert!(content.contains("gyro_data = np.array(["));
        assert!(content.contains("plt.figure("));
        assert!(content.contains("plt.show()"));
    }

    #[test]
    fn test_export_error_handling() {
        let test_files = TestFiles::new();

        // Test unsupported format
        create_test_command()
            .arg("export")
            .arg(&test_files.valid_file)
            .arg("--output")
            .arg("output.xyz")
            .arg("--format")
            .arg("unsupported")
            .assert()
            .failure()
            .stderr(predicate::str::contains("Unsupported export format"));

        // Test export of non-blackbox file
        let text_file = test_files.temp_dir.path().join("test.txt");
        fs::write(&text_file, b"not a blackbox file").unwrap();

        create_test_command()
            .arg("export")
            .arg(&text_file)
            .arg("--output")
            .arg("output.csv")
            .arg("--format")
            .arg("csv")
            .assert()
            .failure()
            .stderr(predicate::str::contains(
                "CSV export requires blackbox analysis",
            ));
    }
}

/// Test compare command scenarios
mod compare_scenarios {
    use super::*;

    #[test]
    fn test_compare_identical_files() {
        let test_files = TestFiles::new();

        let output = create_test_command()
            .arg("compare")
            .arg(&test_files.valid_file)
            .arg(&test_files.valid_file)
            .assert()
            .success()
            .get_output()
            .stdout
            .clone();

        let output_str = String::from_utf8(output).unwrap();
        assert!(output_str.contains("Flight Comparison"));
        assert!(output_str.contains("Summary:"));
        assert!(output_str.contains("Individual Flights:"));
    }

    #[test]
    fn test_compare_different_files() {
        let test_files = TestFiles::new();

        // "Comparing N flights" is a status message → stderr.
        create_test_command()
            .arg("compare")
            .arg(&test_files.valid_file)
            .arg(&test_files.oscillating_file)
            .assert()
            .success()
            .stderr(predicate::str::contains("Comparing 2 flights"));
    }

    #[test]
    fn test_compare_json_output_structure() {
        let test_files = TestFiles::new();

        let (_, json) = {
            let mut cmd = create_test_command();
            cmd.arg("--output-format")
                .arg("json")
                .arg("compare")
                .arg(&test_files.valid_file)
                .arg(&test_files.oscillating_file);
            cmd.assert_json_output()
        };

        // Validate comparison JSON structure
        assert!(json["flights"].as_array().is_some());
        assert!(json["summary"].as_object().is_some());

        let flights = json["flights"].as_array().unwrap();
        assert_eq!(flights.len(), 2);

        for flight in flights {
            assert!(flight["name"].as_str().is_some());
            assert!(flight["tune_quality"].as_f64().is_some());
            assert!(flight["issues_count"].as_u64().is_some());
            assert!(flight["duration_ms"].as_u64().is_some());
        }

        let summary = json["summary"].as_object().unwrap();
        assert!(summary["best_tune_quality"].as_f64().is_some());
        assert!(summary["worst_tune_quality"].as_f64().is_some());
        assert!(summary["avg_tune_quality"].as_f64().is_some());
    }

    #[test]
    fn test_compare_csv_output_format() {
        let test_files = TestFiles::new();

        let (_, csv_rows) = {
            let mut cmd = create_test_command();
            cmd.arg("--output-format")
                .arg("csv")
                .arg("compare")
                .arg(&test_files.valid_file)
                .arg(&test_files.oscillating_file);
            cmd.assert_csv_output()
        };

        // Validate CSV structure
        assert!(csv_rows.len() >= 3); // Header + 2 data rows

        let header = &csv_rows[0];
        let expected_columns = [
            "name",
            "tune_quality",
            "issues_count",
            "recommendations_count",
            "duration_ms",
        ];

        for expected_col in &expected_columns {
            assert!(header.contains(&expected_col.to_string()));
        }

        // Check data rows
        for row in csv_rows.iter().skip(1) {
            assert_eq!(row.len(), header.len());
            assert!(row[0].ends_with(".bbl")); // Name should be filename
        }
    }

    #[test]
    fn test_compare_many_files() {
        let test_files = TestFiles::new();
        let batch_files = test_files.create_batch_files(5);

        let mut cmd = create_test_command();
        cmd.arg("compare");
        for file in &batch_files {
            cmd.arg(file);
        }

        // "Comparing N flights" is a status message → stderr.
        cmd.assert()
            .success()
            .stderr(predicate::str::contains("Comparing 5 flights"));
    }

    #[test]
    fn test_compare_with_failures() {
        let test_files = TestFiles::new();

        // "Failed to analyze" is a status message → stderr.
        create_test_command()
            .arg("compare")
            .arg(&test_files.valid_file)
            .arg(&test_files.corrupted_file)
            .assert()
            .success()
            .stderr(predicate::str::contains("Failed to analyze"));
    }
}

/// Test validate command scenarios
mod validate_scenarios {
    use super::*;

    #[test]
    fn test_validate_mixed_file_quality() {
        let test_files = TestFiles::new();

        // Create directory with mixed files
        let batch_dir = test_files.temp_dir.path().join("validate_batch");
        fs::create_dir(&batch_dir).unwrap();

        fs::copy(&test_files.valid_file, batch_dir.join("valid.bbl")).unwrap();
        fs::copy(&test_files.corrupted_file, batch_dir.join("corrupted.bbl")).unwrap();
        fs::copy(&test_files.empty_file, batch_dir.join("empty.bbl")).unwrap();

        let output = create_test_command()
            .arg("validate")
            .arg(&batch_dir)
            .arg("--check-issues")
            .assert()
            .success()
            .get_output()
            .stdout
            .clone();

        let output_str = String::from_utf8(output).unwrap();

        assert!(output_str.contains("Validating 3 file(s)"));
        assert!(output_str.contains("Validation Summary:"));
        assert!(output_str.contains("Valid files:"));
        assert!(output_str.contains("Invalid files:"));
    }

    #[test]
    fn test_validate_issue_detection() {
        let test_files = TestFiles::new();

        // Create a very short file that should trigger issues
        let short_file = test_files.temp_dir.path().join("short.bbl");
        let short_data = common::create_minimal_blackbox_data()[..200].to_vec(); // Very short
        fs::write(&short_file, short_data).unwrap();

        let output = create_test_command()
            .arg("validate")
            .arg(&short_file)
            .arg("--check-issues")
            .assert()
            .success()
            .get_output()
            .stdout
            .clone();

        let output_str = String::from_utf8(output).unwrap();
        // Should detect issues with short flight
        assert!(output_str.contains("issues found") || output_str.contains("valid"));
    }

    #[test]
    fn test_validate_performance() {
        let test_files = TestFiles::new();
        let large_batch = test_files.create_batch_files(20);

        let batch_dir = test_files.temp_dir.path().join("large_validate");
        fs::create_dir(&batch_dir).unwrap();

        for (i, file) in large_batch.iter().enumerate() {
            fs::copy(file, batch_dir.join(format!("file_{}.bbl", i))).unwrap();
        }

        // Should complete validation of 20 files within reasonable time
        {
            let mut cmd = create_test_command();
            cmd.arg("validate").arg(&batch_dir);
            run_with_timeout(cmd, 30)
        }
        .success()
        .stdout(predicate::str::contains("Validating 20 file(s)"));
    }
}

/// Test tune command scenarios.
mod tune_scenarios {
    use super::*;

    #[test]
    fn test_tune_recommendations_output() {
        let test_files = TestFiles::new();

        let output = create_test_command()
            .arg("tune")
            .arg(&test_files.oscillating_file) // Use oscillating data for recommendations
            .assert()
            .success()
            .get_output()
            .stdout
            .clone();

        let output_str = String::from_utf8(output).unwrap();

        assert!(output_str.contains("Tuning Recommendations:"));
        // The tune command prints stage banners (e.g. "── Tune ──",
        // "── Analyze ──") so a user can see where they are in the flow.
        assert!(output_str.contains("── Tune ──"));
        assert!(output_str.contains("── Analyze ──"));
    }

    /// Without `--connection`, `--dry-run` short-circuits to "no changes
    /// applied" and never tries to open a port.
    #[test]
    fn test_tune_dry_run_without_connection() {
        let test_files = TestFiles::new();

        create_test_command()
            .arg("tune")
            .arg(&test_files.valid_file)
            .arg("--dry-run")
            .assert()
            .success()
            .stdout(predicate::str::contains("Dry run mode"));
    }

    #[test]
    fn test_tune_without_connection() {
        let test_files = TestFiles::new();

        create_test_command()
            .arg("tune")
            .arg(&test_files.valid_file)
            .assert()
            .success()
            .stdout(predicate::str::contains("Specify --connection"));
    }

    /// `simulator://` spins up an in-process MSP simulator so we can
    /// exercise the full dry-connect-and-diff path without serial hardware.
    #[test]
    fn test_tune_dry_run_with_simulator() {
        let test_files = TestFiles::new();

        create_test_command()
            .arg("tune")
            .arg(&test_files.valid_file)
            .arg("--connection")
            .arg("simulator://")
            .arg("--dry-run")
            .assert()
            .success()
            .stdout(predicate::str::contains("Tuning Recommendations"));
    }
}

/// Test error conditions and edge cases
mod error_scenarios {
    use super::*;

    #[test]
    fn test_graceful_out_of_memory_handling() {
        // Test with extremely large max-files to ensure no crashes
        let test_files = TestFiles::new();

        create_test_command()
            .arg("analyze")
            .arg(&test_files.valid_file)
            .arg("--max-files")
            .arg("999999999") // Very large number
            .assert()
            .success(); // Should not crash
    }

    #[test]
    fn test_unicode_filename_handling() {
        let test_files = TestFiles::new();

        // Create file with unicode characters
        let unicode_file = test_files.temp_dir.path().join("测试文件.bbl");
        fs::copy(&test_files.valid_file, &unicode_file).unwrap();

        create_test_command()
            .arg("analyze")
            .arg(&unicode_file)
            .assert()
            .success();
    }

    #[test]
    fn test_very_long_path_handling() {
        let test_files = TestFiles::new();

        // Create deeply nested directory structure
        let mut deep_path = test_files.temp_dir.path().to_path_buf();
        for i in 0..10 {
            deep_path.push(format!("very_long_directory_name_level_{}", i));
        }
        fs::create_dir_all(&deep_path).unwrap();

        let deep_file = deep_path.join("test.bbl");
        fs::copy(&test_files.valid_file, &deep_file).unwrap();

        create_test_command()
            .arg("analyze")
            .arg(&deep_file)
            .assert()
            .success();
    }

    #[test]
    fn test_concurrent_file_access() {
        let test_files = TestFiles::new();

        // Run multiple commands on same file simultaneously
        let handles: Vec<_> = (0..3)
            .map(|_| {
                let file = test_files.valid_file.clone();
                std::thread::spawn(move || {
                    create_test_command()
                        .arg("validate")
                        .arg(&file)
                        .assert()
                        .success();
                })
            })
            .collect();

        for handle in handles {
            handle.join().unwrap();
        }
    }

    #[test]
    #[ignore = "environment-specific: depends on /root being unwritable and the CLI surfacing the error message in a particular shape"]
    fn test_permission_denied_handling() {
        let test_files = TestFiles::new();

        // Try to write to read-only directory (if we can create one)
        if let Ok(readonly_dir) = TempDir::new() {
            let _readonly_path = readonly_dir.path().join("readonly.json");

            // This might fail on some systems, but should handle gracefully
            let result = create_test_command()
                .arg("--output")
                .arg("json")
                .arg("analyze")
                .arg(&test_files.valid_file)
                .arg("--output-dir")
                .arg("/root") // Likely to be permission denied
                .assert();

            // Should either succeed or fail gracefully with permission error
            let output = result.get_output();
            let stderr = String::from_utf8_lossy(&output.stderr);
            if !stderr.is_empty() {
                assert!(stderr.contains("Permission denied") || stderr.contains("access"));
            }
        }
    }
}
