//! Error types for the drone tuner core library.

use std::fmt;

/// Result type alias using [`DronetunerError`] as the error type.
pub type Result<T> = std::result::Result<T, DronetunerError>;

/// Main error type for all drone tuner operations.
#[derive(Debug, thiserror::Error)]
pub enum DronetunerError {
    /// Blackbox parsing errors
    #[error("Failed to parse blackbox data: {message}")]
    ParseError {
        /// Description of the parsing error
        message: String,
        /// Optional position in the data where the error occurred
        position: Option<usize>,
    },

    /// Analysis computation errors
    #[error("Analysis failed: {message}")]
    AnalysisError {
        /// Description of the analysis error
        message: String,
    },

    /// I/O related errors
    #[error("I/O error: {0}")]
    IoError(#[from] std::io::Error),

    /// Serialization errors
    #[error("Serialization error: {0}")]
    SerializationError(#[from] serde_json::Error),

    /// Invalid configuration or parameters
    #[error("Invalid configuration: {message}")]
    ConfigError {
        /// Description of the configuration error
        message: String,
    },

    /// Real-time communication errors (only available with realtime feature)
    #[cfg(feature = "realtime")]
    #[error("Communication error: {message}")]
    CommunicationError {
        /// Description of the communication error
        message: String,
    },

    /// Data validation errors
    #[error("Invalid data: {message}")]
    ValidationError {
        /// Description of the validation error
        message: String,
    },
}

impl DronetunerError {
    /// Creates a new parse error with a message and optional position.
    pub fn parse_error(message: impl Into<String>, position: Option<usize>) -> Self {
        Self::ParseError {
            message: message.into(),
            position,
        }
    }

    /// Creates a new analysis error with a message.
    pub fn analysis_error(message: impl Into<String>) -> Self {
        Self::AnalysisError {
            message: message.into(),
        }
    }

    /// Creates a new configuration error with a message.
    pub fn config_error(message: impl Into<String>) -> Self {
        Self::ConfigError {
            message: message.into(),
        }
    }

    /// Creates a new validation error with a message.
    pub fn validation_error(message: impl Into<String>) -> Self {
        Self::ValidationError {
            message: message.into(),
        }
    }

    #[cfg(feature = "realtime")]
    /// Creates a new communication error with a message.
    pub fn communication_error(message: impl Into<String>) -> Self {
        Self::CommunicationError {
            message: message.into(),
        }
    }
}

/// Custom result type for nom parsers to integrate with our error system.
pub type ParseResult<'a, T> = nom::IResult<&'a [u8], T, ParseErrorContext<'a>>;

/// Nom error context that provides better error messages.
#[derive(Debug, Clone)]
pub struct ParseErrorContext<'a> {
    /// Input data where the error occurred
    pub input: &'a [u8],
    /// Description of what was expected
    pub expected: &'static str,
}

impl<'a> fmt::Display for ParseErrorContext<'a> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Expected {} at position in data", self.expected)
    }
}

impl<'a> nom::error::ParseError<&'a [u8]> for ParseErrorContext<'a> {
    fn from_error_kind(input: &'a [u8], _kind: nom::error::ErrorKind) -> Self {
        Self {
            input,
            expected: "valid data",
        }
    }

    fn append(_input: &'a [u8], _kind: nom::error::ErrorKind, other: Self) -> Self {
        other
    }
}

/// Converts nom parsing errors to our error type.
impl<'a> From<nom::Err<ParseErrorContext<'a>>> for DronetunerError {
    fn from(err: nom::Err<ParseErrorContext<'a>>) -> Self {
        match err {
            nom::Err::Incomplete(_) => Self::parse_error("Incomplete data", None),
            nom::Err::Error(ctx) | nom::Err::Failure(ctx) => {
                Self::parse_error(format!("Parse error: {ctx}"), None)
            }
        }
    }
}
