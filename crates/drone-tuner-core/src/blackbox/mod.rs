//! Blackbox log parsing integration using the blackbox-log crate.
//!
//! This module provides a clean API that abstracts the blackbox-log crate details
//! and converts the parsed data into our domain model for tuning analysis.

use crate::domain::*;
use crate::error::{DronetunerError, Result};
use chrono::Utc;
use std::collections::HashMap;
use uuid::Uuid;

pub mod converter;
pub mod simple_parser;

// Use the simple parser as the main implementation for now
pub use converter::DataConverter;
pub use simple_parser::SimpleBlackboxParser as BlackboxParser;

/// Strategy for selecting which session to analyze in multi-session blackbox files
#[derive(Debug, Clone)]
pub enum SessionStrategy {
    /// Use the last session (most recent flight)
    Last,
    /// Use the first session
    First,
    /// Use the longest session (by estimated duration)
    Longest,
}

/// Statistics collected during parsing with the blackbox-log crate
#[derive(Debug, Default, Clone)]
pub struct ParsingStats {
    /// Total frames parsed
    pub total_frames: u64,
    /// Number of main frames
    pub main_frames: u64,
    /// Number of slow frames
    pub slow_frames: u64,
    /// Number of GPS frames
    pub gps_frames: u64,
    /// Number of event frames
    pub event_frames: u64,
    /// Number of corrupted frames skipped
    pub corrupted_frames: u64,
    /// Total bytes processed
    pub bytes_processed: usize,
    /// Parsing duration in milliseconds
    pub parse_duration_ms: u64,
}

/// Field mappings from blackbox log to our domain model
#[derive(Debug, Clone)]
pub struct FieldMappings {
    /// Gyro field indices
    pub gyro_indices: Option<(usize, usize, usize)>,
    /// Accelerometer field indices
    pub accel_indices: Option<(usize, usize, usize)>,
    /// Motor output field indices
    pub motor_indices: Vec<usize>,
    /// RC command field indices
    pub rc_command_indices: Option<(usize, usize, usize, usize)>, // roll, pitch, yaw, throttle
    /// PID error field indices
    pub pid_error_indices: Option<(usize, usize, usize)>,
    /// PID output field indices
    pub pid_output_indices: Option<(usize, usize, usize)>,
    /// Setpoint field indices
    pub setpoint_indices: Option<(usize, usize, usize)>,
    /// Iteration field index (fallback for duration calculation when time_raw() fails)
    pub iteration_idx: Option<usize>,
}

impl Default for FieldMappings {
    fn default() -> Self {
        Self {
            gyro_indices: None,
            accel_indices: None,
            motor_indices: Vec::new(),
            rc_command_indices: None,
            pid_error_indices: None,
            pid_output_indices: None,
            setpoint_indices: None,
            iteration_idx: None,
        }
    }
}

/// Configuration for blackbox parsing
#[derive(Debug, Clone)]
pub struct ParsingConfig {
    /// Maximum number of frames to parse (None for all)
    pub max_frames: Option<u64>,
    /// Skip frames at the beginning
    pub skip_frames: u64,
    /// Enable strict parsing (fail on any error)
    pub strict_parsing: bool,
    /// Enable progress reporting
    pub progress_reporting: bool,
    /// Progress reporting interval in frames
    pub progress_interval: u64,
    /// Selected session to analyze (0-based index, None for first session)
    pub selected_session: Option<usize>,
    /// List all sessions instead of analyzing (for session discovery)
    pub list_sessions_only: bool,
    /// Optional strategy for selecting a session when `selected_session` is not provided
    pub session_strategy: Option<SessionStrategy>,
}

impl Default for ParsingConfig {
    fn default() -> Self {
        Self {
            max_frames: None,
            skip_frames: 0,
            strict_parsing: false,
            progress_reporting: true,
            progress_interval: 1000,
            selected_session: None,
            list_sessions_only: false,
            session_strategy: Some(SessionStrategy::Last),
        }
    }
}

/// High-level error types for blackbox parsing
#[derive(Debug, thiserror::Error)]
pub enum BlackboxError {
    #[error("Invalid blackbox format: {0}")]
    /// Invalid or unrecognized blackbox file format
    InvalidFormat(String),

    #[error("Unsupported blackbox version: {0}")]
    /// Blackbox version is not supported by this parser
    UnsupportedVersion(String),

    #[error("Missing required fields: {missing_fields:?}")]
    /// Required fields are missing from the blackbox data
    MissingFields {
        /// List of missing field names
        missing_fields: Vec<String>,
    },

    #[error("Data corruption detected at frame {frame_index}: {details}")]
    /// Data corruption detected during parsing
    DataCorruption {
        /// Frame index where corruption was detected
        frame_index: u64,
        /// Details about the corruption
        details: String,
    },

    #[error("IO error: {0}")]
    /// I/O error occurred during file operations
    Io(#[from] std::io::Error),

    #[error("Parsing error: {0}")]
    /// Generic parsing error
    Parse(String),
}

/// Result type for blackbox operations
pub type BlackboxResult<T> = std::result::Result<T, BlackboxError>;

/// Convert BlackboxError to our main error type
impl From<BlackboxError> for DronetunerError {
    fn from(err: BlackboxError) -> Self {
        match err {
            BlackboxError::InvalidFormat(msg) => {
                DronetunerError::parse_error(format!("Invalid blackbox format: {}", msg), None)
            }
            BlackboxError::UnsupportedVersion(version) => DronetunerError::parse_error(
                format!("Unsupported blackbox version: {}", version),
                None,
            ),
            BlackboxError::MissingFields { missing_fields } => DronetunerError::parse_error(
                format!("Missing required fields: {}", missing_fields.join(", ")),
                None,
            ),
            BlackboxError::DataCorruption {
                frame_index,
                details,
            } => DronetunerError::parse_error(
                format!("Data corruption at frame {}: {}", frame_index, details),
                Some(frame_index as usize),
            ),
            BlackboxError::Io(io_err) => {
                DronetunerError::parse_error(format!("IO error: {}", io_err), None)
            }
            BlackboxError::Parse(parse_err) => {
                DronetunerError::parse_error(format!("Parsing error: {}", parse_err), None)
            }
        }
    }
}

/// Utility functions for blackbox parsing
pub mod utils {
    use super::*;

    /// Detect if data is a valid blackbox file
    pub fn is_blackbox_file(data: &[u8]) -> bool {
        // Check for blackbox header markers
        if data.len() < 10 {
            return false;
        }

        // Look for common blackbox header patterns
        let header_str = String::from_utf8_lossy(&data[..std::cmp::min(data.len(), 1000)]);

        header_str.contains("H Product:")
            || header_str.contains("H modeActivationConditions")
            || header_str.contains("H features")
            || header_str.contains("H Blackbox version")
    }

    /// Extract basic info from blackbox without full parsing
    pub fn extract_basic_info(data: &[u8]) -> BlackboxResult<HashMap<String, String>> {
        let mut info = HashMap::new();

        if !is_blackbox_file(data) {
            return Err(BlackboxError::InvalidFormat(
                "Not a valid blackbox file".to_string(),
            ));
        }

        let header_str = String::from_utf8_lossy(&data[..std::cmp::min(data.len(), 10000)]);

        // Extract firmware info
        if let Some(line) = header_str.lines().find(|l| l.starts_with("H Firmware")) {
            info.insert(
                "firmware".to_string(),
                line.trim_start_matches("H ").to_string(),
            );
        }

        // Extract product info
        if let Some(line) = header_str.lines().find(|l| l.starts_with("H Product:")) {
            info.insert(
                "product".to_string(),
                line.trim_start_matches("H Product:").trim().to_string(),
            );
        }

        // Extract blackbox version
        if let Some(line) = header_str.lines().find(|l| l.contains("Blackbox version")) {
            info.insert(
                "blackbox_version".to_string(),
                line.trim_start_matches("H ").to_string(),
            );
        }

        Ok(info)
    }

    /// Calculate approximate flight duration from blackbox size
    pub fn estimate_duration_seconds(data: &[u8], sample_rate: Option<f32>) -> f32 {
        let rate = sample_rate.unwrap_or(1000.0); // Default 1kHz
        let estimated_frames = data.len() / 50; // Rough estimate of bytes per frame
        estimated_frames as f32 / rate
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parsing_stats_default() {
        let stats = ParsingStats::default();
        assert_eq!(stats.total_frames, 0);
        assert_eq!(stats.bytes_processed, 0);
    }

    #[test]
    fn test_field_mappings_default() {
        let mappings = FieldMappings::default();
        assert!(mappings.gyro_indices.is_none());
        assert!(mappings.motor_indices.is_empty());
    }

    #[test]
    fn test_parsing_config_default() {
        let config = ParsingConfig::default();
        assert!(config.max_frames.is_none());
        assert_eq!(config.skip_frames, 0);
        assert!(!config.strict_parsing);
        assert!(config.progress_reporting);
    }

    #[test]
    fn test_blackbox_file_detection() {
        // Valid blackbox header
        let valid_data =
            b"H Product:Blackbox flight data recorder by Nicholas Sherlock\nH Data version:2\n";
        assert!(utils::is_blackbox_file(valid_data));

        // Invalid data
        let invalid_data = b"This is not a blackbox file";
        assert!(!utils::is_blackbox_file(invalid_data));

        // Empty data
        assert!(!utils::is_blackbox_file(&[]));
    }

    #[test]
    fn test_basic_info_extraction() {
        let test_data = b"H Product:Betaflight flight data recorder\nH Firmware revision: Betaflight 4.4.0\nH Blackbox version: 1\nEnd of header\nI frame data...";
        let info = utils::extract_basic_info(test_data).unwrap();

        assert!(info.contains_key("product"));
        assert!(info.get("product").unwrap().contains("Betaflight"));
    }

    #[test]
    fn test_duration_estimation() {
        let test_data = vec![0u8; 50000]; // 50KB of data
        let duration = utils::estimate_duration_seconds(&test_data, Some(1000.0));
        assert!(duration > 0.0);
        assert!(duration < 10.0); // Should be reasonable
    }

    #[test]
    fn test_error_conversion() {
        let blackbox_err = BlackboxError::InvalidFormat("test".to_string());
        let drone_err: DronetunerError = blackbox_err.into();

        match drone_err {
            DronetunerError::ParseError { message, .. } => {
                assert!(message.contains("Invalid blackbox format"));
            }
            _ => panic!("Expected ParseError"),
        }
    }
}
