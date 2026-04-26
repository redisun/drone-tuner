//! Common utilities and test helpers for CLI integration tests.
//!
//! Test scaffolding — some helpers and fields are not yet referenced by every
//! test binary. Allow dead_code so we don't fight the linter while the CLI
//! integration suite is still being written.
#![allow(dead_code)]

use std::fs;
use std::path::{Path, PathBuf};
use tempfile::TempDir;

/// Helper to create minimal valid blackbox test data
pub fn create_minimal_blackbox_data() -> Vec<u8> {
    // This creates a minimal blackbox file structure that should parse
    let mut data = Vec::new();

    // Betaflight blackbox header
    data.extend_from_slice(b"Product:Blackbox flight data recorder by Nicholas Sherlock\n");
    data.extend_from_slice(b"Data version:2\n");
    data.extend_from_slice(b"I interval:32\n");
    data.extend_from_slice(b"P interval:0\n");
    data.extend_from_slice(b"Firmware type:Betaflight\n");
    data.extend_from_slice(b"Firmware revision:BTFL_4.2.0\n");
    data.extend_from_slice(b"Firmware date:Jun 04 2021 06:28:16\n");
    data.extend_from_slice(b"Board information:SPRACINGF3EVO\n");
    data.extend_from_slice(b"Log start datetime:2021-06-01T10:30:00.000Z\n");
    data.extend_from_slice(b"H Field I name:loopIteration,time,axisP[0],axisP[1],axisP[2],axisI[0],axisI[1],axisI[2]\n");
    data.extend_from_slice(b"H Field I predictor:0,0,0,0,0,0,0,0\n");
    data.extend_from_slice(b"H Field I encoding:1,1,1,1,1,1,1,1\n");
    data.extend_from_slice(b"\n");

    // Add some minimal frame data
    for i in 0..100 {
        let frame_data = format!("I{},{},{},{},{},{},{},{}\n",
            i, i * 1000, i % 100, (i * 2) % 100, (i * 3) % 100,
            i % 50, (i * 2) % 50, (i * 3) % 50);
        data.extend_from_slice(frame_data.as_bytes());
    }

    data
}

/// Helper to create test data with realistic gyro oscillations
pub fn create_oscillating_blackbox_data() -> Vec<u8> {
    let mut data = Vec::new();

    // Header
    data.extend_from_slice(b"Product:Blackbox flight data recorder by Nicholas Sherlock\n");
    data.extend_from_slice(b"Data version:2\n");
    data.extend_from_slice(b"I interval:32\n");
    data.extend_from_slice(b"P interval:0\n");
    data.extend_from_slice(b"Firmware type:Betaflight\n");
    data.extend_from_slice(b"Firmware revision:BTFL_4.2.0\n");
    data.extend_from_slice(b"Firmware date:Jun 04 2021 06:28:16\n");
    data.extend_from_slice(b"Board information:SPRACINGF3EVO\n");
    data.extend_from_slice(b"Log start datetime:2021-06-01T10:30:00.000Z\n");
    data.extend_from_slice(b"H Field I name:loopIteration,time,gyroADC[0],gyroADC[1],gyroADC[2],motor[0],motor[1],motor[2],motor[3]\n");
    data.extend_from_slice(b"H Field I predictor:0,0,0,0,0,0,0,0,0\n");
    data.extend_from_slice(b"H Field I encoding:1,1,1,1,1,1,1,1,1\n");
    data.extend_from_slice(b"\n");

    // Add oscillating gyro data (simulates P-term oscillation at ~100Hz)
    use std::f32::consts::PI;
    for i in 0..1000 {
        let time = i as f32 * 0.001; // 1ms intervals
        let gyro_x = (50.0 * (2.0 * PI * 100.0 * time).sin()) as i32; // 100Hz oscillation
        let gyro_y = (30.0 * (2.0 * PI * 80.0 * time).sin()) as i32;  // 80Hz oscillation
        let gyro_z = (20.0 * (2.0 * PI * 120.0 * time).sin()) as i32; // 120Hz oscillation

        let motor_base = 1500;
        let motor_1 = motor_base + (gyro_x / 10);
        let motor_2 = motor_base + (gyro_y / 10);
        let motor_3 = motor_base - (gyro_x / 10);
        let motor_4 = motor_base - (gyro_y / 10);

        let frame_data = format!("I{},{},{},{},{},{},{},{},{}\n",
            i, i * 1000, gyro_x, gyro_y, gyro_z, motor_1, motor_2, motor_3, motor_4);
        data.extend_from_slice(frame_data.as_bytes());
    }

    data
}

/// Helper to create corrupted blackbox data for error testing
pub fn create_corrupted_blackbox_data() -> Vec<u8> {
    vec![0xFF, 0xFE, 0xFD, 0xFC, 0x00, 0x01, 0x02, 0x03] // Invalid header
}

/// Helper to create test files in a temporary directory
pub struct TestFiles {
    pub temp_dir: TempDir,
    pub valid_file: PathBuf,
    pub oscillating_file: PathBuf,
    pub corrupted_file: PathBuf,
    pub empty_file: PathBuf,
}

impl TestFiles {
    pub fn new() -> Self {
        let temp_dir = TempDir::new().unwrap();

        let valid_file = temp_dir.path().join("valid.bbl");
        fs::write(&valid_file, create_minimal_blackbox_data()).unwrap();

        let oscillating_file = temp_dir.path().join("oscillating.bbl");
        fs::write(&oscillating_file, create_oscillating_blackbox_data()).unwrap();

        let corrupted_file = temp_dir.path().join("corrupted.bbl");
        fs::write(&corrupted_file, create_corrupted_blackbox_data()).unwrap();

        let empty_file = temp_dir.path().join("empty.bbl");
        fs::write(&empty_file, b"").unwrap();

        Self {
            temp_dir,
            valid_file,
            oscillating_file,
            corrupted_file,
            empty_file,
        }
    }

    /// Create a directory with multiple test files
    pub fn create_batch_files(&self, count: usize) -> Vec<PathBuf> {
        let mut files = Vec::new();
        for i in 0..count {
            let file_path = self.temp_dir.path().join(format!("batch_{}.bbl", i));
            fs::write(&file_path, create_minimal_blackbox_data()).unwrap();
            files.push(file_path);
        }
        files
    }
}

/// Helper to validate JSON output structure
pub fn validate_json_output(output_str: &str) -> serde_json::Value {
    // Find the JSON part (starts with '{' and ends with '}')
    let json_start = output_str.find('{').expect("Should contain JSON");
    let json_end = output_str.rfind('}').expect("Should contain JSON") + 1;
    let json_str = &output_str[json_start..json_end];

    let json: serde_json::Value = serde_json::from_str(json_str)
        .expect("Output should be valid JSON");

    // Validate common structure
    assert!(json.get("version").is_some(), "JSON should have version field");
    assert!(json.get("timestamp").is_some(), "JSON should have timestamp field");

    json
}

/// Helper to validate CSV output format
pub fn validate_csv_output(csv_str: &str) -> Vec<Vec<String>> {
    let lines: Vec<&str> = csv_str.lines().collect();
    assert!(!lines.is_empty(), "CSV should have at least header line");

    let mut rows = Vec::new();
    for line in lines {
        let fields: Vec<String> = line.split(',').map(|s| s.to_string()).collect();
        rows.push(fields);
    }

    rows
}

/// Helper to create performance test data
pub fn create_large_blackbox_data(frame_count: usize) -> Vec<u8> {
    let mut data = Vec::new();

    // Header
    data.extend_from_slice(b"Product:Blackbox flight data recorder by Nicholas Sherlock\n");
    data.extend_from_slice(b"Data version:2\n");
    data.extend_from_slice(b"I interval:32\n");
    data.extend_from_slice(b"P interval:0\n");
    data.extend_from_slice(b"Firmware type:Betaflight\n");
    data.extend_from_slice(b"Firmware revision:BTFL_4.2.0\n");
    data.extend_from_slice(b"Firmware date:Jun 04 2021 06:28:16\n");
    data.extend_from_slice(b"Board information:SPRACINGF3EVO\n");
    data.extend_from_slice(b"Log start datetime:2021-06-01T10:30:00.000Z\n");
    data.extend_from_slice(b"H Field I name:loopIteration,time,gyroADC[0],gyroADC[1],gyroADC[2]\n");
    data.extend_from_slice(b"H Field I predictor:0,0,0,0,0\n");
    data.extend_from_slice(b"H Field I encoding:1,1,1,1,1\n");
    data.extend_from_slice(b"\n");

    // Add many frames for performance testing
    for i in 0..frame_count {
        let frame_data = format!("I{},{},{},{},{}\n",
            i, i * 1000, i % 1000 - 500, (i * 2) % 1000 - 500, (i * 3) % 1000 - 500);
        data.extend_from_slice(frame_data.as_bytes());
    }

    data
}

/// Helper to run command with timeout
pub fn run_with_timeout(mut cmd: assert_cmd::Command, timeout_secs: u64) -> assert_cmd::assert::Assert {
    use std::time::Duration;

    cmd.timeout(Duration::from_secs(timeout_secs));
    cmd.assert()
}

/// Helper to check file existence and basic properties
pub fn verify_output_file(path: &Path, min_size: Option<usize>) {
    assert!(path.exists(), "Output file should exist: {}", path.display());

    if let Some(min_size) = min_size {
        let metadata = fs::metadata(path).unwrap();
        assert!(metadata.len() as usize >= min_size,
            "Output file should be at least {} bytes, got {}", min_size, metadata.len());
    }
}

/// Helper to create command with common test setup
pub fn create_test_command() -> assert_cmd::Command {
    let mut cmd = assert_cmd::Command::cargo_bin("drone-tuner").unwrap();

    // Set environment variables for consistent testing
    cmd.env("RUST_LOG", "warn"); // Reduce log noise in tests
    cmd.env("NO_COLOR", "1"); // Disable colors for predictable output

    cmd
}

/// Trait to extend Command with additional test helpers
pub trait CommandTestExt {
    fn with_test_env(self) -> Self;
    fn assert_json_output(self) -> (assert_cmd::assert::Assert, serde_json::Value);
    fn assert_csv_output(self) -> (assert_cmd::assert::Assert, Vec<Vec<String>>);
}

impl CommandTestExt for assert_cmd::Command {
    fn with_test_env(mut self) -> Self {
        self.env("RUST_LOG", "warn");
        self.env("NO_COLOR", "1");
        self
    }

    fn assert_json_output(mut self) -> (assert_cmd::assert::Assert, serde_json::Value) {
        let assert = self.assert().success();
        let output = String::from_utf8(assert.get_output().stdout.clone()).unwrap();
        let json = validate_json_output(&output);
        (assert, json)
    }

    fn assert_csv_output(mut self) -> (assert_cmd::assert::Assert, Vec<Vec<String>>) {
        let assert = self.assert().success();
        let output = String::from_utf8(assert.get_output().stdout.clone()).unwrap();
        let csv = validate_csv_output(&output);
        (assert, csv)
    }
}