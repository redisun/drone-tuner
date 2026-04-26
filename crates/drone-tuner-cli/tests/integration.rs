//! Integration tests for the drone-tuner CLI application.
//!
//! These tests validate the complete end-to-end functionality of all CLI commands
//! including file I/O, error handling, output formats, and command-line argument validation.

use assert_cmd::Command;
use predicates::prelude::*;
use std::fs;
use std::path::PathBuf;
use tempfile::TempDir;

/// Helper function to get the path to test data
fn test_data_path() -> PathBuf {
    let mut path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    path.pop(); // Go up from crates/drone-tuner-cli
    path.pop(); // Go up from crates
    path.push("test_data");
    path
}

/// Helper function to get the path to the test blackbox file.
/// Uses `btfl_all_old.bbl` because that's the small fixture tracked in
/// git and available in CI.
fn test_blackbox_file() -> PathBuf {
    test_data_path().join("btfl_all_old.bbl")
}

/// Helper function to create a CLI command
fn cli_cmd() -> Command {
    Command::cargo_bin("drone-tuner").unwrap()
}

/// Helper function to create a minimal test blackbox file
fn create_test_blackbox_file(temp_dir: &TempDir, name: &str, content: &[u8]) -> PathBuf {
    let file_path = temp_dir.path().join(name);
    fs::write(&file_path, content).unwrap();
    file_path
}

mod analyze_command_tests {
    use super::*;

    #[test]
    fn test_analyze_basic_success() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        cli_cmd()
            .arg("analyze")
            .arg(&blackbox_file)
            .assert()
            .success()
            .stdout(predicate::str::contains("Analysis Summary"));
    }

    #[test]
    fn test_analyze_nonexistent_file() {
        cli_cmd()
            .arg("analyze")
            .arg("/nonexistent/file.bbl")
            .assert()
            .failure()
            .stderr(predicate::str::contains("Failed to read file"));
    }

    #[test]
    fn test_analyze_json_output() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        let output = cli_cmd()
            .arg("--output-format")
            .arg("json")
            .arg("analyze")
            .arg(&blackbox_file)
            .assert()
            .success()
            .get_output()
            .stdout
            .clone();

        let output_str = String::from_utf8(output).unwrap();

        // Find the JSON part (starts with '{' and ends with '}')
        let json_start = output_str.find('{').expect("Should contain JSON");
        let json_end = output_str.rfind('}').expect("Should contain JSON") + 1;
        let json_str = &output_str[json_start..json_end];

        // Verify JSON structure
        let json: serde_json::Value = serde_json::from_str(json_str).unwrap();
        assert!(json.get("version").is_some());
        assert!(json.get("timestamp").is_some());
        assert!(json.get("results").is_some());
    }

    #[test]
    fn test_analyze_csv_output() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        cli_cmd()
            .arg("--output-format")
            .arg("csv")
            .arg("analyze")
            .arg(&blackbox_file)
            .assert()
            .success()
            .stdout(predicate::str::contains("file,status,tune_quality"));
    }

    #[test]
    fn test_analyze_with_output_directory() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        let temp_dir = TempDir::new().unwrap();

        cli_cmd()
            .arg("--output-format")
            .arg("json")
            .arg("analyze")
            .arg(&blackbox_file)
            .arg("--output-dir")
            .arg(temp_dir.path())
            .assert()
            .success();

        // Verify output file was created
        let output_file = temp_dir.path().join("analysis_results.json");
        assert!(output_file.exists());

        // Verify file contains valid JSON
        let content = fs::read_to_string(output_file).unwrap();
        let _json: serde_json::Value = serde_json::from_str(&content).unwrap();
    }

    #[test]
    fn test_analyze_detailed_flag() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        cli_cmd()
            .arg("analyze")
            .arg(&blackbox_file)
            .arg("--detailed")
            .assert()
            .success()
            .stdout(predicate::str::contains("Analysis Summary"));
    }

    #[test]
    fn test_analyze_show_details_flag() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        cli_cmd()
            .arg("analyze")
            .arg(&blackbox_file)
            .arg("--show-details")
            .assert()
            .success()
            .stdout(predicate::str::contains("File Details:"));
    }

    #[test]
    fn test_analyze_list_sessions() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        cli_cmd()
            .arg("analyze")
            .arg(&blackbox_file)
            .arg("--list-sessions")
            .assert()
            .success();
    }

    #[test]
    fn test_analyze_bb_summary() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        cli_cmd()
            .arg("analyze")
            .arg(&blackbox_file)
            .arg("--bb-summary")
            .assert()
            .success();
    }

    #[test]
    fn test_analyze_session_selection() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        cli_cmd()
            .arg("analyze")
            .arg(&blackbox_file)
            .arg("--session")
            .arg("1")
            .assert()
            .success();
    }

    #[test]
    fn test_analyze_session_strategy() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        for strategy in ["first", "last", "longest"] {
            cli_cmd()
                .arg("analyze")
                .arg(&blackbox_file)
                .arg("--session-strategy")
                .arg(strategy)
                .assert()
                .success();
        }
    }

    #[test]
    fn test_analyze_min_confidence() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        cli_cmd()
            .arg("analyze")
            .arg(&blackbox_file)
            .arg("--min-confidence")
            .arg("0.8")
            .assert()
            .success();
    }

    #[test]
    fn test_analyze_max_files() {
        let temp_dir = TempDir::new().unwrap();

        // Create multiple test files
        for i in 1..=5 {
            create_test_blackbox_file(&temp_dir, &format!("test{}.bbl", i), b"test data");
        }

        cli_cmd()
            .arg("analyze")
            .arg(temp_dir.path())
            .arg("--max-files")
            .arg("3")
            .assert()
            .success();
    }

    #[test]
    fn test_analyze_directory() {
        let temp_dir = TempDir::new().unwrap();

        // Create test BBL files
        create_test_blackbox_file(&temp_dir, "test1.bbl", b"test data 1");
        create_test_blackbox_file(&temp_dir, "test2.bbl", b"test data 2");
        create_test_blackbox_file(&temp_dir, "not_bbl.txt", b"not a blackbox file");

        // "Found N blackbox file(s)" is a status message → stderr.
        cli_cmd()
            .arg("analyze")
            .arg(temp_dir.path())
            .assert()
            .success()
            .stderr(predicate::str::contains("Found 2 blackbox file(s)"));
    }

    #[test]
    fn test_analyze_empty_directory() {
        let temp_dir = TempDir::new().unwrap();

        // "No blackbox files found" is a status message → stderr.
        cli_cmd()
            .arg("analyze")
            .arg(temp_dir.path())
            .assert()
            .success()
            .stderr(predicate::str::contains("No blackbox files found"));
    }

    #[test]
    fn test_analyze_verbose_flag() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        cli_cmd()
            .arg("--verbose")
            .arg("analyze")
            .arg(&blackbox_file)
            .assert()
            .success();
    }

    #[test]
    fn test_analyze_invalid_confidence() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        cli_cmd()
            .arg("analyze")
            .arg(&blackbox_file)
            .arg("--min-confidence")
            .arg("1.5") // Invalid: > 1.0
            .assert()
            .failure();
    }

    #[test]
    fn test_analyze_invalid_session() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        cli_cmd()
            .arg("analyze")
            .arg(&blackbox_file)
            .arg("--session")
            .arg("0") // Invalid: should be 1-based
            .assert()
            .failure();
    }
}

mod compare_command_tests {
    use super::*;

    #[test]
    fn test_compare_two_files() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        cli_cmd()
            .arg("compare")
            .arg(&blackbox_file)
            .arg(&blackbox_file) // Compare file with itself for testing
            .assert()
            .success()
            .stdout(predicate::str::contains("Flight Comparison"));
    }

    #[test]
    fn test_compare_json_output() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        let output = cli_cmd()
            .arg("--output-format")
            .arg("json")
            .arg("compare")
            .arg(&blackbox_file)
            .arg(&blackbox_file)
            .assert()
            .success()
            .get_output()
            .stdout
            .clone();

        let output_str = String::from_utf8(output).unwrap();
        let _json: serde_json::Value = serde_json::from_str(&output_str).unwrap();
    }

    #[test]
    fn test_compare_csv_output() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        cli_cmd()
            .arg("--output-format")
            .arg("csv")
            .arg("compare")
            .arg(&blackbox_file)
            .arg(&blackbox_file)
            .assert()
            .success()
            .stdout(predicate::str::contains("name,tune_quality,issues_count"));
    }

    #[test]
    fn test_compare_no_files() {
        cli_cmd().arg("compare").assert().failure();
    }

    #[test]
    fn test_compare_single_file() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        // "Need at least 2..." is a status message → stderr.
        cli_cmd()
            .arg("compare")
            .arg(&blackbox_file)
            .assert()
            .success()
            .stderr(predicate::str::contains("Need at least 2"));
    }

    #[test]
    fn test_compare_nonexistent_file() {
        // "Failed to analyze" is a status message → stderr.
        cli_cmd()
            .arg("compare")
            .arg("/nonexistent/file1.bbl")
            .arg("/nonexistent/file2.bbl")
            .assert()
            .success() // Command succeeds but reports failed analysis
            .stderr(predicate::str::contains("Failed to analyze"));
    }

    #[test]
    fn test_compare_multiple_files() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        // "Comparing N flights" is a status message → stderr.
        cli_cmd()
            .arg("compare")
            .arg(&blackbox_file)
            .arg(&blackbox_file)
            .arg(&blackbox_file)
            .assert()
            .success()
            .stderr(predicate::str::contains("Comparing 3 flights"));
    }
}

mod validate_command_tests {
    use super::*;

    #[test]
    fn test_validate_valid_file() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        cli_cmd()
            .arg("validate")
            .arg(&blackbox_file)
            .assert()
            .success()
            .stdout(predicate::str::contains("Validation Summary"));
    }

    #[test]
    fn test_validate_with_issues_check() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        cli_cmd()
            .arg("validate")
            .arg(&blackbox_file)
            .arg("--check-issues")
            .assert()
            .success()
            .stdout(predicate::str::contains("Validation Summary"));
    }

    #[test]
    fn test_validate_invalid_file() {
        let temp_dir = TempDir::new().unwrap();
        let invalid_file = create_test_blackbox_file(&temp_dir, "invalid.bbl", b"invalid content");

        cli_cmd()
            .arg("validate")
            .arg(&invalid_file)
            .assert()
            .success()
            .stdout(predicate::str::contains("Invalid files: 1"));
    }

    #[test]
    fn test_validate_directory() {
        let temp_dir = TempDir::new().unwrap();
        create_test_blackbox_file(&temp_dir, "test1.bbl", b"test data");
        create_test_blackbox_file(&temp_dir, "test2.bbl", b"test data");

        cli_cmd()
            .arg("validate")
            .arg(temp_dir.path())
            .assert()
            .success()
            .stdout(predicate::str::contains("Validating 2 file(s)"));
    }

    #[test]
    fn test_validate_empty_directory() {
        let temp_dir = TempDir::new().unwrap();

        cli_cmd()
            .arg("validate")
            .arg(temp_dir.path())
            .assert()
            .success()
            .stdout(predicate::str::contains("No blackbox files found"));
    }

    #[test]
    fn test_validate_nonexistent_path() {
        cli_cmd()
            .arg("validate")
            .arg("/nonexistent/path")
            .assert()
            .failure();
    }
}

/// `monitor` is gated behind the `experimental` feature; only run these
/// tests when that feature is enabled.
#[cfg(feature = "experimental")]
mod monitor_command_tests {
    use super::*;

    #[test]
    fn test_monitor_without_realtime_feature() {
        cli_cmd()
            .arg("monitor")
            .arg("/dev/ttyUSB0")
            .assert()
            .success()
            .stdout(predicate::str::contains(
                "Real-time monitoring is not available",
            ));
    }

    #[test]
    fn test_monitor_invalid_connection() {
        cli_cmd()
            .arg("monitor")
            .arg("/nonexistent/port")
            .assert()
            .success() // Command succeeds but shows feature unavailable message
            .stdout(predicate::str::contains(
                "Real-time monitoring is not available",
            ));
    }

    #[test]
    fn test_monitor_with_rate() {
        cli_cmd()
            .arg("monitor")
            .arg("/dev/ttyUSB0")
            .arg("--rate")
            .arg("200")
            .assert()
            .success()
            .stdout(predicate::str::contains(
                "Real-time monitoring is not available",
            ));
    }

    #[test]
    fn test_monitor_with_duration() {
        cli_cmd()
            .arg("monitor")
            .arg("/dev/ttyUSB0")
            .arg("--duration")
            .arg("10")
            .assert()
            .success()
            .stdout(predicate::str::contains(
                "Real-time monitoring is not available",
            ));
    }

    #[test]
    fn test_monitor_with_fields() {
        cli_cmd()
            .arg("monitor")
            .arg("/dev/ttyUSB0")
            .arg("--fields")
            .arg("gyro,pid_error,motors")
            .assert()
            .success()
            .stdout(predicate::str::contains(
                "Real-time monitoring is not available",
            ));
    }

    #[test]
    fn test_monitor_with_log_file() {
        let temp_dir = TempDir::new().unwrap();
        let log_file = temp_dir.path().join("telemetry.log");

        cli_cmd()
            .arg("monitor")
            .arg("/dev/ttyUSB0")
            .arg("--log-file")
            .arg(&log_file)
            .assert()
            .success()
            .stdout(predicate::str::contains(
                "Real-time monitoring is not available",
            ));
    }

    #[test]
    fn test_monitor_json_output() {
        cli_cmd()
            .arg("--output-format")
            .arg("json")
            .arg("monitor")
            .arg("/dev/ttyUSB0")
            .assert()
            .success()
            .stdout(predicate::str::contains(
                "Real-time monitoring is not available",
            ));
    }

    #[test]
    fn test_monitor_csv_output() {
        cli_cmd()
            .arg("--output-format")
            .arg("csv")
            .arg("monitor")
            .arg("/dev/ttyUSB0")
            .assert()
            .success()
            .stdout(predicate::str::contains(
                "Real-time monitoring is not available",
            ));
    }
}

/// `tune` is gated behind the `experimental` feature; only run these
/// tests when that feature is enabled.
#[cfg(feature = "experimental")]
mod tune_command_tests {
    use super::*;

    #[test]
    fn test_tune_basic() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        cli_cmd()
            .arg("tune")
            .arg(&blackbox_file)
            .assert()
            .success()
            .stdout(predicate::str::contains("Tuning Recommendations"));
    }

    #[test]
    fn test_tune_dry_run() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        cli_cmd()
            .arg("tune")
            .arg(&blackbox_file)
            .arg("--dry-run")
            .assert()
            .success()
            .stdout(predicate::str::contains("Dry run mode"));
    }

    #[test]
    fn test_tune_with_connection() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        cli_cmd()
            .arg("tune")
            .arg(&blackbox_file)
            .arg("--connection")
            .arg("/dev/ttyUSB0")
            .assert()
            .success()
            .stdout(predicate::str::contains(
                "Real-time tuning is not available",
            ));
    }

    #[test]
    fn test_tune_with_backup() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        cli_cmd()
            .arg("tune")
            .arg(&blackbox_file)
            .arg("--backup")
            .assert()
            .success()
            .stdout(predicate::str::contains("Tuning Recommendations"));
    }

    #[test]
    fn test_tune_auto_apply_safe() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        cli_cmd()
            .arg("tune")
            .arg(&blackbox_file)
            .arg("--auto-apply-safe")
            .assert()
            .success()
            .stdout(predicate::str::contains("Tuning Recommendations"));
    }

    #[test]
    fn test_tune_nonexistent_file() {
        cli_cmd()
            .arg("tune")
            .arg("/nonexistent/file.bbl")
            .assert()
            .failure()
            .stderr(predicate::str::contains("Failed to read file"));
    }

    #[test]
    fn test_tune_invalid_connection() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        cli_cmd()
            .arg("tune")
            .arg(&blackbox_file)
            .arg("--connection")
            .arg("invalid_port")
            .assert()
            .success()
            .stdout(predicate::str::contains(
                "Real-time tuning is not available",
            ));
    }
}

mod export_command_tests {
    use super::*;

    #[test]
    fn test_export_csv() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        let temp_dir = TempDir::new().unwrap();
        let output_file = temp_dir.path().join("export.csv");

        cli_cmd()
            .arg("export")
            .arg(&blackbox_file)
            .arg("--output")
            .arg(&output_file)
            .arg("--format")
            .arg("csv")
            .assert()
            .success()
            .stdout(predicate::str::contains("Export completed"));

        assert!(output_file.exists());
        let content = fs::read_to_string(output_file).unwrap();
        assert!(content.contains("FPV Drone Tuner Analysis Export"));
    }

    #[test]
    fn test_export_json() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        let temp_dir = TempDir::new().unwrap();
        let output_file = temp_dir.path().join("export.json");

        cli_cmd()
            .arg("export")
            .arg(&blackbox_file)
            .arg("--output")
            .arg(&output_file)
            .arg("--format")
            .arg("json")
            .assert()
            .success()
            .stdout(predicate::str::contains("Export completed"));

        assert!(output_file.exists());
        let content = fs::read_to_string(output_file).unwrap();
        let _json: serde_json::Value = serde_json::from_str(&content).unwrap();
    }

    #[test]
    fn test_export_matlab() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        let temp_dir = TempDir::new().unwrap();
        let output_file = temp_dir.path().join("export.m");

        cli_cmd()
            .arg("export")
            .arg(&blackbox_file)
            .arg("--output")
            .arg(&output_file)
            .arg("--format")
            .arg("matlab")
            .assert()
            .success()
            .stdout(predicate::str::contains("Export completed"));

        assert!(output_file.exists());
        let content = fs::read_to_string(output_file).unwrap();
        assert!(content.contains("% FPV Drone Tuner Analysis Export"));
        assert!(content.contains("tune_quality"));
    }

    #[test]
    fn test_export_python() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        let temp_dir = TempDir::new().unwrap();
        let output_file = temp_dir.path().join("export.py");

        cli_cmd()
            .arg("export")
            .arg(&blackbox_file)
            .arg("--output")
            .arg(&output_file)
            .arg("--format")
            .arg("python")
            .assert()
            .success()
            .stdout(predicate::str::contains("Export completed"));

        assert!(output_file.exists());
        let content = fs::read_to_string(output_file).unwrap();
        assert!(content.contains("# FPV Drone Tuner Analysis Export"));
        assert!(content.contains("import numpy as np"));
    }

    #[test]
    fn test_export_with_raw_data() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        let temp_dir = TempDir::new().unwrap();
        let output_file = temp_dir.path().join("export.csv");

        cli_cmd()
            .arg("export")
            .arg(&blackbox_file)
            .arg("--output")
            .arg(&output_file)
            .arg("--format")
            .arg("csv")
            .arg("--include-raw")
            .assert()
            .success()
            .stdout(predicate::str::contains("Export completed"));

        assert!(output_file.exists());
        let content = fs::read_to_string(output_file).unwrap();
        assert!(content.contains("Raw Gyro Data"));
    }

    #[test]
    fn test_export_with_fft_data() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        let temp_dir = TempDir::new().unwrap();
        let output_file = temp_dir.path().join("export.csv");

        cli_cmd()
            .arg("export")
            .arg(&blackbox_file)
            .arg("--output")
            .arg(&output_file)
            .arg("--format")
            .arg("csv")
            .arg("--include-fft")
            .assert()
            .success()
            .stdout(predicate::str::contains("Export completed"));

        assert!(output_file.exists());
    }

    #[test]
    fn test_export_unsupported_format() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        let temp_dir = TempDir::new().unwrap();
        let output_file = temp_dir.path().join("export.txt");

        cli_cmd()
            .arg("export")
            .arg(&blackbox_file)
            .arg("--output")
            .arg(&output_file)
            .arg("--format")
            .arg("unsupported")
            .assert()
            .failure()
            .stderr(predicate::str::contains("Unsupported export format"));
    }

    #[test]
    fn test_export_nonexistent_input() {
        let temp_dir = TempDir::new().unwrap();
        let output_file = temp_dir.path().join("export.csv");

        cli_cmd()
            .arg("export")
            .arg("/nonexistent/file.bbl")
            .arg("--output")
            .arg(&output_file)
            .arg("--format")
            .arg("csv")
            .assert()
            .failure()
            .stderr(predicate::str::contains("Failed to read file"));
    }

    #[test]
    fn test_export_non_blackbox_file() {
        let temp_dir = TempDir::new().unwrap();
        let input_file = create_test_blackbox_file(&temp_dir, "test.txt", b"not a blackbox");
        let output_file = temp_dir.path().join("export.csv");

        cli_cmd()
            .arg("export")
            .arg(&input_file)
            .arg("--output")
            .arg(&output_file)
            .arg("--format")
            .arg("csv")
            .assert()
            .failure()
            .stderr(predicate::str::contains(
                "CSV export requires blackbox analysis",
            ));
    }
}

mod info_command_tests {
    use super::*;

    #[test]
    fn test_info_basic() {
        cli_cmd()
            .arg("info")
            .assert()
            .success()
            .stdout(predicate::str::contains("FPV Drone Tuner"))
            .stdout(predicate::str::contains("Version:"))
            .stdout(predicate::str::contains("System Information:"));
    }

    #[test]
    fn test_info_includes_version() {
        cli_cmd()
            .arg("info")
            .assert()
            .success()
            .stdout(predicate::str::contains("Version: 0.1.0"));
    }

    #[test]
    fn test_info_includes_library_status() {
        cli_cmd()
            .arg("info")
            .assert()
            .success()
            .stdout(predicate::str::contains("Library Status:"))
            .stdout(predicate::str::contains("FFT support available"))
            .stdout(predicate::str::contains("Blackbox parsing ready"));
    }

    #[test]
    fn test_info_includes_system_info() {
        cli_cmd()
            .arg("info")
            .assert()
            .success()
            .stdout(predicate::str::contains("OS:"))
            .stdout(predicate::str::contains("Arch:"));
    }
}

mod global_options_tests {
    use super::*;

    #[test]
    fn test_help_flag() {
        cli_cmd()
            .arg("--help")
            .assert()
            .success()
            .stdout(predicate::str::contains("Analyze FPV drone blackbox logs"));
    }

    #[test]
    fn test_version_flag() {
        cli_cmd()
            .arg("--version")
            .assert()
            .success()
            .stdout(predicate::str::contains("0.1.0"));
    }

    #[test]
    fn test_verbose_flag() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        cli_cmd()
            .arg("--verbose")
            .arg("analyze")
            .arg(&blackbox_file)
            .assert()
            .success();
    }

    #[test]
    fn test_detailed_info_flag() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        cli_cmd()
            .arg("--detailed-info")
            .arg("analyze")
            .arg(&blackbox_file)
            .assert()
            .success()
            .stdout(predicate::str::contains("File Details:"));
    }

    #[test]
    fn test_invalid_command() {
        cli_cmd().arg("invalid-command").assert().failure();
    }

    #[test]
    fn test_missing_required_argument() {
        cli_cmd().arg("analyze").assert().failure();
    }

    #[test]
    fn test_invalid_output_format() {
        cli_cmd()
            .arg("--output-format")
            .arg("invalid")
            .arg("analyze")
            .arg("test.bbl")
            .assert()
            .failure();
    }
}

mod performance_regression_tests {
    use super::*;
    use std::time::Instant;

    #[test]
    fn test_analyze_performance() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        let start = Instant::now();

        cli_cmd()
            .arg("analyze")
            .arg(&blackbox_file)
            .assert()
            .success();

        let duration = start.elapsed();

        // Performance regression check - analysis should complete within 30 seconds
        assert!(
            duration.as_secs() < 30,
            "Analysis took too long: {:?}",
            duration
        );
    }

    #[test]
    fn test_batch_processing_performance() {
        let temp_dir = TempDir::new().unwrap();

        // Create multiple test files
        for i in 1..=10 {
            create_test_blackbox_file(&temp_dir, &format!("test{}.bbl", i), b"test data");
        }

        let start = Instant::now();

        cli_cmd()
            .arg("analyze")
            .arg(temp_dir.path())
            .assert()
            .success();

        let duration = start.elapsed();

        // Batch processing should scale reasonably
        assert!(
            duration.as_secs() < 60,
            "Batch processing took too long: {:?}",
            duration
        );
    }
}

mod concurrent_execution_tests {
    use super::*;
    use std::thread;

    #[test]
    fn test_concurrent_analysis() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        let handles: Vec<_> = (0..3)
            .map(|_| {
                let file = blackbox_file.clone();
                thread::spawn(move || {
                    cli_cmd().arg("analyze").arg(&file).assert().success();
                })
            })
            .collect();

        for handle in handles {
            handle.join().unwrap();
        }
    }

    #[test]
    fn test_resource_cleanup() {
        let temp_dir = TempDir::new().unwrap();

        // Create and process multiple files to test resource cleanup
        for i in 1..=5 {
            let file =
                create_test_blackbox_file(&temp_dir, &format!("test{}.bbl", i), b"test data");

            cli_cmd().arg("analyze").arg(&file).assert().success();
        }

        // If we reach here without panicking, resources were cleaned up properly
    }
}

/// End-to-end tests against the in-process MSP simulator. Built only when
/// the CLI is compiled with `--features test-support`.
#[cfg(feature = "test-support")]
mod simulator_tests {
    use super::*;

    /// `tune --connection simulator:// --dry-run --auto-apply-safe` should
    /// connect to the in-process simulator, read its PID values, print a
    /// diff, and exit cleanly without writing.
    #[test]
    fn test_tune_simulator_dry_run_diff() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            return;
        }
        cli_cmd()
            .arg("tune")
            .arg(&blackbox_file)
            .arg("--connection")
            .arg("simulator://")
            .arg("--dry-run")
            .arg("--auto-apply-safe")
            .assert()
            .success()
            .stdout(predicate::str::contains("Connected"))
            .stdout(predicate::str::contains("Dry run complete"));
    }

    /// `tune --connection simulator:// --apply-all` should apply every
    /// recommendation and report success without --save-eeprom (RAM only).
    #[test]
    fn test_tune_simulator_apply_all_ram_only() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            return;
        }
        cli_cmd()
            .arg("tune")
            .arg(&blackbox_file)
            .arg("--connection")
            .arg("simulator://")
            .arg("--apply-all")
            .assert()
            .success()
            .stdout(predicate::str::contains("Connected"))
            .stdout(
                predicate::str::contains("Write succeeded")
                    .or(predicate::str::contains("No PID recommendations matched")),
            )
            .stdout(predicate::str::contains("RAM-only"));
    }

    /// `tune --connection simulator:// --apply-all --save-eeprom` should
    /// also persist via EEPROM_WRITE.
    #[test]
    fn test_tune_simulator_apply_all_with_eeprom() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            return;
        }
        cli_cmd()
            .arg("tune")
            .arg(&blackbox_file)
            .arg("--connection")
            .arg("simulator://")
            .arg("--apply-all")
            .arg("--save-eeprom")
            .assert()
            .success()
            .stdout(
                predicate::str::contains("Changes persisted")
                    .or(predicate::str::contains("No PID recommendations matched")),
            );
    }

    /// Without --auto-apply-safe or --apply-all, the writeback step should
    /// be a no-op even when --connection is given.
    #[test]
    fn test_tune_simulator_skip_without_opt_in() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            return;
        }
        cli_cmd()
            .arg("tune")
            .arg(&blackbox_file)
            .arg("--connection")
            .arg("simulator://")
            .assert()
            .success()
            .stdout(predicate::str::contains("skipping writeback"));
    }
}

mod feature_flag_tests {
    use super::*;

    /// `monitor`/`tune` are gated behind the `experimental` feature; this
    /// asserts that with `experimental` enabled but `realtime` not, those
    /// commands surface a graceful "not available" message rather than
    /// crashing.
    #[cfg(feature = "experimental")]
    #[test]
    fn test_realtime_features_disabled() {
        // Test that realtime commands gracefully handle missing features
        cli_cmd()
            .arg("monitor")
            .arg("/dev/ttyUSB0")
            .assert()
            .success()
            .stdout(predicate::str::contains(
                "Real-time monitoring is not available",
            ));

        let blackbox_file = test_blackbox_file();
        if blackbox_file.exists() {
            cli_cmd()
                .arg("tune")
                .arg(&blackbox_file)
                .arg("--connection")
                .arg("/dev/ttyUSB0")
                .assert()
                .success()
                .stdout(predicate::str::contains(
                    "Real-time tuning is not available",
                ));
        }
    }

    #[test]
    fn test_core_features_always_available() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        // Core features should always work regardless of feature flags
        cli_cmd()
            .arg("analyze")
            .arg(&blackbox_file)
            .assert()
            .success();

        cli_cmd()
            .arg("validate")
            .arg(&blackbox_file)
            .assert()
            .success();

        cli_cmd().arg("info").assert().success();
    }
}

mod error_handling_tests {
    use super::*;

    #[test]
    fn test_graceful_error_handling() {
        // Test various error conditions are handled gracefully

        // Invalid file path
        cli_cmd()
            .arg("analyze")
            .arg("/invalid/path/file.bbl")
            .assert()
            .failure()
            .stderr(predicate::str::contains("Failed to read file"));

        // Invalid directory
        cli_cmd()
            .arg("validate")
            .arg("/invalid/directory")
            .assert()
            .failure();

        // Invalid argument values
        cli_cmd()
            .arg("analyze")
            .arg("test.bbl")
            .arg("--session")
            .arg("abc") // Invalid: should be numeric
            .assert()
            .failure();
    }

    #[test]
    fn test_partial_failure_handling() {
        let temp_dir = TempDir::new().unwrap();

        // Mix of valid and invalid files
        create_test_blackbox_file(&temp_dir, "valid.bbl", b"some data");
        create_test_blackbox_file(&temp_dir, "invalid.bbl", b"bad data");

        cli_cmd()
            .arg("analyze")
            .arg(temp_dir.path())
            .assert()
            .success() // Should succeed overall
            .stdout(predicate::str::contains("Analysis Summary"));
    }

    #[test]
    fn test_memory_exhaustion_protection() {
        // Test with very large max-files to ensure no memory issues
        let temp_dir = TempDir::new().unwrap();
        create_test_blackbox_file(&temp_dir, "test.bbl", b"test");

        cli_cmd()
            .arg("analyze")
            .arg(temp_dir.path())
            .arg("--max-files")
            .arg("1000000") // Very large number
            .assert()
            .success(); // Should not crash
    }
}

mod output_format_validation_tests {
    use super::*;

    #[test]
    fn test_json_output_structure() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        let output = cli_cmd()
            .arg("--output-format")
            .arg("json")
            .arg("analyze")
            .arg(&blackbox_file)
            .assert()
            .success()
            .get_output()
            .stdout
            .clone();

        let output_str = String::from_utf8(output).unwrap();
        let json: serde_json::Value = serde_json::from_str(&output_str).unwrap();

        // Validate required fields
        assert!(json.get("version").is_some());
        assert!(json.get("timestamp").is_some());
        assert!(json.get("results").and_then(|r| r.as_array()).is_some());

        if let Some(results) = json.get("results").and_then(|r| r.as_array()) {
            if !results.is_empty() {
                let result = &results[0];
                assert!(result.get("file").is_some());
                assert!(result.get("status").is_some());
            }
        }
    }

    #[test]
    fn test_csv_output_format() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        let output = cli_cmd()
            .arg("--output-format")
            .arg("csv")
            .arg("analyze")
            .arg(&blackbox_file)
            .assert()
            .success()
            .get_output()
            .stdout
            .clone();

        let output_str = String::from_utf8(output).unwrap();
        let lines: Vec<&str> = output_str.lines().collect();

        // Should have header and at least one data row
        assert!(!lines.is_empty());

        if !lines.is_empty() {
            let header = lines[0];
            assert!(header.contains("file"));
            assert!(header.contains("status"));
            assert!(header.contains("tune_quality"));
        }
    }

    #[test]
    fn test_pretty_output_readability() {
        let blackbox_file = test_blackbox_file();
        if !blackbox_file.exists() {
            println!("Skipping test - no test data file found");
            return;
        }

        cli_cmd()
            .arg("--output-format")
            .arg("pretty")
            .arg("analyze")
            .arg(&blackbox_file)
            .assert()
            .success()
            .stdout(predicate::str::contains("Analysis Summary"))
            .stdout(predicate::str::contains("Tune Quality:"));
    }
}
