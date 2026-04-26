//! # Drone Tuner Core
//!
//! Core analysis engine for FPV drone tuning, providing blackbox parsing,
//! frequency analysis, oscillation detection, and filter optimization.
//!
//! ## Features
//!
//! - **Blackbox Parsing**: Support for Betaflight and other flight controller logs
//! - **FFT Analysis**: High-performance frequency domain analysis using Welch's method
//! - **Oscillation Detection**: Automated detection of P-term, D-term, and mechanical oscillations
//! - **Filter Optimization**: Intelligent filter configuration recommendations
//! - **Real-time Communication**: Live flight controller connectivity (with `realtime` feature)
//!
//! ## Quick Start
//!
//! ```rust,no_run
//! use drone_tuner_core::{BlackboxParser, AnalysisEngine};
//!
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! // Parse blackbox file
//! let mut parser = BlackboxParser::new();
//! let data = std::fs::read("flight.bbl")?;
//! let session = parser.parse_file(&data)?;
//!
//! // Analyze for oscillations and get recommendations
//! let mut engine = AnalysisEngine::new();
//! let report = engine.analyze(&session)?;
//!
//! println!("Detected {} issues", report.detected_issues.len());
//! println!("Recommended {} filter changes", report.filter_recommendations.len());
//! # Ok(())
//! # }
//! ```

#![warn(clippy::all, clippy::pedantic, clippy::nursery)]
#![warn(missing_docs)]
#![allow(clippy::module_name_repetitions)]

pub mod analysis;
pub mod blackbox;
pub mod domain;
pub mod error;
pub mod filters;

#[cfg(feature = "realtime")]
pub mod realtime;

// Re-export main types for convenience
pub use analysis::AnalysisEngine;
pub use blackbox::{BlackboxParser, DataConverter, ParsingConfig, ParsingStats};
pub use domain::AnalysisReport;
pub use domain::{FlightSession, HardwareConfiguration, TelemetryData};
pub use error::{DronetunerError, Result};

/// Current version of the drone-tuner-core library
pub const VERSION: &str = env!("CARGO_PKG_VERSION");
