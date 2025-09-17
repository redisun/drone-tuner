//! Blackbox log file parsing for various flight controller formats.

use crate::domain::*;
use crate::error::{DronetunerError, Result};
use chrono::Utc;
use flate2::read::GzDecoder;
use nom::{
    bytes::complete::*,
    number::complete::*,
    IResult,
};
use std::collections::HashMap;
use std::io::Read;
use uuid::Uuid;

/// Blackbox parser for flight controller log files
#[derive(Debug)]
pub struct BlackboxParser {
    /// Current parsing state
    state: ParserState,
    /// Decoded header information
    header: Option<BlackboxHeader>,
    /// Frame field definitions from header  
    field_defs: Vec<FieldDefinition>,
    /// Accumulated parsed frames
    frames: Vec<DataFrame>,
    /// Flight statistics
    stats: ParsingStats,
}

/// Internal parser state
#[derive(Debug, Default)]
struct ParserState {
    /// Current position in data
    position: usize,
    /// Number of frames processed
    frame_count: u64,
    /// Previous frame values for delta encoding
    previous_values: HashMap<String, i32>,
}

/// Parsed blackbox header
#[derive(Debug, Clone)]
struct BlackboxHeader {
    /// Firmware name and version
    pub firmware_info: String,
    /// Hardware target information
    pub hardware_info: String,
    /// PID configuration
    pub pid_config: PidConfiguration,
    /// Filter configuration
    pub filter_config: FilterConfiguration,
    /// Sample rate information
    pub rates: RateInfo,
    /// Field definitions
    pub fields: Vec<FieldInfo>,
}

/// Rate information from header
#[derive(Debug, Clone)]
struct RateInfo {
    /// Main loop rate (PID loop)
    pub pid_rate: u32,
    /// Gyro sample rate
    pub gyro_rate: u32,
    /// Blackbox logging rate
    pub blackbox_rate: u32,
}

/// Field information from header
#[derive(Debug, Clone)]
struct FieldInfo {
    /// Field name
    pub name: String,
    /// Field index
    pub index: usize,
    /// Data type
    pub data_type: DataType,
    /// Encoding method
    pub encoding: EncodingType,
}

/// Data types supported in blackbox
#[derive(Debug, Clone)]
enum DataType {
    /// Signed integer
    SignedInt,
    /// Unsigned integer
    UnsignedInt,
    /// Floating point
    Float,
}

/// Encoding types for efficient storage
#[derive(Debug, Clone)]
enum EncodingType {
    /// Variable-length signed encoding
    SignedVB,
    /// Variable-length unsigned encoding
    UnsignedVB,
    /// Fixed-width encoding
    Fixed(usize),
    /// Delta from previous value
    Delta,
}

/// Field definition for parsing
#[derive(Debug, Clone)]
struct FieldDefinition {
    /// Field name
    name: String,
    /// Field index in frame
    index: usize,
    /// Encoding method
    encoding: EncodingType,
}

/// Parsed frame data
#[derive(Debug, Clone)]
enum DataFrame {
    /// I-frame (full data)
    I(Vec<i32>),
    /// P-frame (delta data)
    P(Vec<i32>),
    /// Event frame
    Event(String, HashMap<String, String>),
}

/// Statistics collected during parsing
#[derive(Debug, Default)]
struct ParsingStats {
    /// Total frames parsed
    pub total_frames: u64,
    /// Number of I-frames
    pub i_frames: u64,
    /// Number of P-frames
    pub p_frames: u64,
    /// Number of event frames
    pub event_frames: u64,
    /// Number of corrupted frames skipped
    pub corrupted_frames: u64,
    /// Total bytes processed
    pub bytes_processed: usize,
}

impl BlackboxParser {
    /// Create a new blackbox parser
    pub fn new() -> Self {
        Self {
            state: ParserState::default(),
            header: None,
            field_defs: Vec::new(),
            frames: Vec::new(),
            stats: ParsingStats::default(),
        }
    }

    /// Parse a blackbox file and return a flight session
    pub fn parse_file(&mut self, data: &[u8]) -> Result<FlightSession> {
        tracing::info!("Starting blackbox parsing, {} bytes", data.len());
        
        // Handle compressed files
        let decoded_data = if self.is_compressed(data)? {
            self.decompress(data)?
        } else {
            data.to_vec()
        };

        self.stats.bytes_processed = decoded_data.len();

        // Parse header
        let (remaining, header) = self.parse_header(&decoded_data)
            .map_err(|e| DronetunerError::parse_error(format!("Header parsing failed: {e}"), None))?;
        
        self.header = Some(header.clone());
        self.field_defs = self.extract_field_definitions(&header);

        tracing::info!("Header parsed successfully, found {} field definitions", self.field_defs.len());

        // Validate we have field definitions before parsing frames
        if self.field_defs.is_empty() {
            return Err(DronetunerError::parse_error(
                "No field definitions found in header - cannot parse frames".to_string(),
                None
            ));
        }

        // Parse frames
        let mut input = remaining;
        self.state.position = decoded_data.len() - input.len();

        while !input.is_empty() {
            self.state.position = decoded_data.len() - input.len();

            match self.parse_frame(input) {
                Ok((rest, frame)) => {
                    self.frames.push(frame);
                    self.stats.total_frames += 1;
                    input = rest;

                    // Update progress periodically
                    if self.stats.total_frames % 1000 == 0 {
                        let progress = 100.0 * (self.stats.bytes_processed - input.len()) as f32 / self.stats.bytes_processed as f32;
                        tracing::debug!("Parsing progress: {:.1}% ({} frames)", progress, self.stats.total_frames);
                    }
                }
                Err(e) if self.is_recoverable_error(&e) => {
                    // Skip corrupted frame and continue
                    tracing::warn!("Skipping corrupted frame at position {} (0x{:x}): {}",
                                   self.state.position, self.state.position, e);

                    let new_input = self.skip_to_next_frame(input);
                    if new_input.len() == input.len() {
                        // No frame found, advance by one byte to avoid infinite loop
                        if input.len() > 1 {
                            input = &input[1..];
                        } else {
                            break;
                        }
                    } else {
                        input = new_input;
                    }
                    self.stats.corrupted_frames += 1;
                }
                Err(e) => {
                    return Err(DronetunerError::parse_error(
                        format!("Unrecoverable parsing error at position {}: {e}", self.state.position),
                        Some(self.state.position)
                    ));
                }
            }
        }

        tracing::info!(
            "Parsing completed: {} total frames ({} I-frames, {} P-frames, {} events, {} corrupted)",
            self.stats.total_frames,
            self.stats.i_frames,
            self.stats.p_frames,
            self.stats.event_frames,
            self.stats.corrupted_frames
        );

        // Convert raw frames to domain model
        self.build_flight_session(&header)
    }

    /// Check if data is compressed (gzip)
    fn is_compressed(&self, data: &[u8]) -> Result<bool> {
        if data.len() < 2 {
            return Ok(false);
        }
        
        // Check for gzip magic number
        Ok(data[0] == 0x1f && data[1] == 0x8b)
    }

    /// Decompress gzipped data
    fn decompress(&self, data: &[u8]) -> Result<Vec<u8>> {
        let mut decoder = GzDecoder::new(data);
        let mut decompressed = Vec::new();
        decoder.read_to_end(&mut decompressed)
            .map_err(|e| DronetunerError::parse_error(format!("Decompression failed: {e}"), None))?;
        
        tracing::info!("Decompressed {} bytes to {} bytes", data.len(), decompressed.len());
        Ok(decompressed)
    }

    /// Parse the blackbox header
    fn parse_header<'a>(&self, input: &'a [u8]) -> IResult<&'a [u8], BlackboxHeader> {
        // Look for header start marker - be more flexible
        let input = if let Ok((input, _)) = tag::<&[u8], &[u8], nom::error::Error<&[u8]>>(b"H Product:")(input) {
            input
        } else if let Ok((input, _)) = tag::<&[u8], &[u8], nom::error::Error<&[u8]>>(b"H ")(input) {
            // Skip first H line if it doesn't start with "Product:"
            let (input, _) = take_until("\n")(input)?;
            let (input, _) = tag(b"\n")(input)?;
            input
        } else {
            return Err(nom::Err::Error(nom::error::Error::new(input, nom::error::ErrorKind::Tag)));
        };

        // Parse header lines until data starts
        let mut lines = Vec::new();
        let mut remaining = input;

        while !remaining.is_empty() && remaining[0] == b'H' {
            match self.parse_header_line(remaining) {
                Ok((rest, line)) => {
                    lines.push(line);
                    remaining = rest;
                }
                Err(_) => {
                    // Skip malformed header line
                    if let Some(newline_pos) = remaining.iter().position(|&b| b == b'\n') {
                        remaining = &remaining[newline_pos + 1..];
                    } else {
                        break;
                    }
                }
            }
        }

        // Build header from parsed lines
        let header = self.build_header_from_lines(lines);
        Ok((remaining, header))
    }

    /// Parse a single header line
    fn parse_header_line<'a>(&self, input: &'a [u8]) -> IResult<&'a [u8], String> {
        let (input, _) = tag(b"H ")(input)?;
        let (input, line_data) = take_until("\n")(input)?;
        let (input, _) = tag(b"\n")(input)?;
        
        let line = String::from_utf8_lossy(line_data).to_string();
        Ok((input, line))
    }

    /// Build header structure from parsed lines
    fn build_header_from_lines(&self, lines: Vec<String>) -> BlackboxHeader {
        let mut firmware_info = String::new();
        let mut hardware_info = String::new();
        let pid_config = PidConfiguration::default();
        let filter_config = FilterConfiguration::default();
        let rates = RateInfo { pid_rate: 8000, gyro_rate: 8000, blackbox_rate: 1000 };
        let mut fields = Vec::new();

        for line in lines {
            if line.starts_with("Firmware") {
                firmware_info = line;
            } else if line.starts_with("Board") {
                hardware_info = line;
            } else if line.starts_with("Field") {
                // Parse field definition
                if let Some(field) = self.parse_field_definition(&line) {
                    fields.push(field);
                }
            }
            // Parse other header fields as needed
        }

        BlackboxHeader {
            firmware_info,
            hardware_info,
            pid_config,
            filter_config,
            rates,
            fields,
        }
    }

    /// Parse field definition from header line
    fn parse_field_definition(&self, line: &str) -> Option<FieldInfo> {
        // Example: "Field I name:gyroADC[0]"
        if let Some(parts) = line.split(':').nth(1) {
            let name = parts.trim().to_string();
            Some(FieldInfo {
                name,
                index: 0, // Will be set later
                data_type: DataType::SignedInt,
                encoding: EncodingType::SignedVB,
            })
        } else {
            None
        }
    }

    /// Extract field definitions for frame parsing
    fn extract_field_definitions(&self, header: &BlackboxHeader) -> Vec<FieldDefinition> {
        header.fields.iter().enumerate().map(|(i, field)| {
            FieldDefinition {
                name: field.name.clone(),
                index: i,
                encoding: field.encoding.clone(),
            }
        }).collect()
    }

    /// Parse a single frame
    fn parse_frame<'a>(&mut self, input: &'a [u8]) -> IResult<&'a [u8], DataFrame> {
        if input.is_empty() {
            return Err(nom::Err::Error(nom::error::Error::new(input, nom::error::ErrorKind::Eof)));
        }

        let frame_type = input[0];
        match frame_type {
            b'I' => {
                self.stats.i_frames += 1;
                self.parse_i_frame(input)
            },
            b'P' => {
                self.stats.p_frames += 1;
                self.parse_p_frame(input)
            },
            b'E' => {
                self.stats.event_frames += 1;
                self.parse_event_frame(input)
            },
            _ => {
                // Provide detailed diagnostic for unexpected frame type
                let context = if input.len() >= 16 {
                    format!("Found 0x{:02x} ('{}'). Next bytes: {:02x?}",
                           frame_type,
                           if frame_type.is_ascii_graphic() { frame_type as char } else { '?' },
                           &input[1..16.min(input.len())])
                } else {
                    format!("Found 0x{:02x} ('{}'). Remaining bytes: {:02x?}",
                           frame_type,
                           if frame_type.is_ascii_graphic() { frame_type as char } else { '?' },
                           &input[1..])
                };

                let custom_error = nom::error::Error {
                    input,
                    code: nom::error::ErrorKind::Tag,
                };
                tracing::debug!("Invalid frame type at position {}: {}", self.state.position, context);
                Err(nom::Err::Error(custom_error))
            },
        }
    }

    /// Parse I-frame (full data)
    fn parse_i_frame<'a>(&mut self, input: &'a [u8]) -> IResult<&'a [u8], DataFrame> {
        let (input, _) = tag(b"I")(input)?;
        
        let mut values = Vec::new();
        let mut remaining = input;
        
        for field_def in &self.field_defs {
            let (rest, value) = match field_def.encoding {
                EncodingType::SignedVB => self.parse_signed_vb(remaining)?,
                EncodingType::UnsignedVB => {
                    let (rest, val) = self.parse_unsigned_vb(remaining)?;
                    (rest, val as i32)
                },
                EncodingType::Fixed(size) => {
                    let (rest, bytes) = take(size)(remaining)?;
                    let value = match size {
                        1 => bytes[0] as i32,
                        2 => i16::from_le_bytes([bytes[0], bytes[1]]) as i32,
                        4 => i32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]),
                        _ => 0, // Unsupported size
                    };
                    (rest, value)
                },
                EncodingType::Delta => {
                    // Delta encoding not used in I-frames
                    let (rest, value) = self.parse_signed_vb(remaining)?;
                    (rest, value)
                },
            };
            
            values.push(value);
            self.state.previous_values.insert(field_def.name.clone(), value);
            remaining = rest;
        }
        
        Ok((remaining, DataFrame::I(values)))
    }

    /// Parse P-frame (delta data)
    fn parse_p_frame<'a>(&mut self, input: &'a [u8]) -> IResult<&'a [u8], DataFrame> {
        let (input, _) = tag(b"P")(input)?;
        
        let mut values = Vec::new();
        let mut remaining = input;
        
        for field_def in &self.field_defs {
            let (rest, delta) = self.parse_signed_vb(remaining)?;
            
            let previous = self.state.previous_values
                .get(&field_def.name)
                .copied()
                .unwrap_or(0);
            
            let value = previous + delta;
            values.push(value);
            self.state.previous_values.insert(field_def.name.clone(), value);
            remaining = rest;
        }
        
        Ok((remaining, DataFrame::P(values)))
    }

    /// Parse event frame
    fn parse_event_frame<'a>(&self, input: &'a [u8]) -> IResult<&'a [u8], DataFrame> {
        let (input, _) = tag(b"E")(input)?;
        let (input, event_data) = take_until("\n")(input)?;
        let (input, _) = tag(b"\n")(input)?;
        
        let event_str = String::from_utf8_lossy(event_data).to_string();
        let data = HashMap::new(); // Parse event data as needed
        
        Ok((input, DataFrame::Event(event_str, data)))
    }

    /// Parse signed variable-byte integer
    fn parse_signed_vb<'a>(&self, input: &'a [u8]) -> IResult<&'a [u8], i32> {
        let (input, first_byte) = le_u8(input)?;
        
        if (first_byte & 0x80) == 0 {
            // Single byte value
            let value = if (first_byte & 0x40) != 0 {
                // Negative value
                -((first_byte & 0x3F) as i32)
            } else {
                (first_byte & 0x3F) as i32
            };
            Ok((input, value))
        } else {
            // Multi-byte value - implement full variable-byte decoding
            // This is a simplified version - full implementation would handle all cases
            let (input, second_byte) = le_u8(input)?;
            let value = ((first_byte & 0x7F) as i32) | (((second_byte & 0x7F) as i32) << 7);
            Ok((input, value))
        }
    }

    /// Parse unsigned variable-byte integer  
    fn parse_unsigned_vb<'a>(&self, input: &'a [u8]) -> IResult<&'a [u8], u32> {
        let (input, first_byte) = le_u8(input)?;
        
        if (first_byte & 0x80) == 0 {
            // Single byte value
            Ok((input, (first_byte & 0x7F) as u32))
        } else {
            // Multi-byte value
            let (input, second_byte) = le_u8(input)?;
            let value = ((first_byte & 0x7F) as u32) | (((second_byte & 0x7F) as u32) << 7);
            Ok((input, value))
        }
    }

    /// Check if a parsing error is recoverable
    fn is_recoverable_error(&self, _error: &nom::Err<nom::error::Error<&[u8]>>) -> bool {
        // For now, treat most errors as recoverable
        // In practice, this would be more sophisticated
        true
    }

    /// Skip to the next frame after encountering an error
    fn skip_to_next_frame<'a>(&self, input: &'a [u8]) -> &'a [u8] {
        // Look for next frame marker (I, P, or E)
        // Start from index 1 to avoid finding the same corrupted frame
        for (i, &byte) in input.iter().enumerate().skip(1) {
            if byte == b'I' || byte == b'P' || byte == b'E' {
                tracing::debug!("Found next frame marker '{}' at offset {}", byte as char, i);
                return &input[i..];
            }
        }
        tracing::debug!("No more frame markers found in remaining {} bytes", input.len());
        &[] // No more frames found
    }

    /// Convert parsed frames to flight session
    fn build_flight_session(&self, header: &BlackboxHeader) -> Result<FlightSession> {
        let session_id = Uuid::new_v4();
        let timestamp = Utc::now(); // In practice, extract from header
        
        // Build telemetry data from frames
        let telemetry = self.build_telemetry_data()?;
        
        // Extract hardware configuration from header
        let hardware = self.build_hardware_config(header);
        
        // Build metadata
        let metadata = FlightMetadata {
            session_id,
            timestamp,
            duration_ms: self.calculate_duration(&telemetry),
            hardware,
            environment: EnvironmentalConditions {
                temperature_c: None,
                wind_speed_ms: None,
                wind_direction_deg: None,
                pressure_hpa: None,
                humidity_percent: None,
            },
            pilot: PilotProfile {
                pilot_id: None,
                skill_level: SkillLevel::Intermediate,
                flying_style: FlyingStyle::Mixed,
            },
        };

        let events = self.build_flight_events();

        Ok(FlightSession {
            metadata,
            telemetry,
            events,
            analysis_results: None,
        })
    }

    /// Build telemetry data from parsed frames
    fn build_telemetry_data(&self) -> Result<TelemetryData> {
        let sample_rate = 1000.0; // Extract from header
        let capacity = self.frames.len();
        
        let mut gyro = TimeSeriesVector3::with_capacity(capacity);
        let mut accel = TimeSeriesVector3::with_capacity(capacity);
        let mut motor = vec![
            MotorTrace { motor_id: 1, values: Vec::with_capacity(capacity) },
            MotorTrace { motor_id: 2, values: Vec::with_capacity(capacity) },
            MotorTrace { motor_id: 3, values: Vec::with_capacity(capacity) },
            MotorTrace { motor_id: 4, values: Vec::with_capacity(capacity) },
        ];
        let mut pid_error = PidErrorTrace {
            roll: Vec::with_capacity(capacity),
            pitch: Vec::with_capacity(capacity),
            yaw: Vec::with_capacity(capacity),
        };
        let mut rc_commands = RcCommandTrace {
            roll: Vec::with_capacity(capacity),
            pitch: Vec::with_capacity(capacity),
            yaw: Vec::with_capacity(capacity),
            throttle: Vec::with_capacity(capacity),
        };

        // Convert frames to time series data
        for frame in &self.frames {
            match frame {
                DataFrame::I(values) | DataFrame::P(values) => {
                    // Map frame values to telemetry fields based on field definitions
                    // This is simplified - real implementation would use field mappings
                    if values.len() >= 12 {
                        // Gyro data (assume first 3 values)
                        gyro.push(nalgebra::Vector3::new(
                            values[0] as f32 / 16.0, // Scale factor from gyro range
                            values[1] as f32 / 16.0,
                            values[2] as f32 / 16.0,
                        ));
                        
                        // Accelerometer data (assume next 3 values)
                        accel.push(nalgebra::Vector3::new(
                            values[3] as f32 / 512.0, // Scale factor from accel range
                            values[4] as f32 / 512.0,
                            values[5] as f32 / 512.0,
                        ));
                        
                        // Motor outputs (assume next 4 values)
                        for i in 0..4 {
                            if let Some(motor_trace) = motor.get_mut(i) {
                                motor_trace.values.push(values[6 + i] as f32 / 2000.0);
                            }
                        }
                        
                        // PID errors and RC commands would be extracted similarly
                        pid_error.roll.push(0.0);
                        pid_error.pitch.push(0.0);
                        pid_error.yaw.push(0.0);
                        
                        rc_commands.roll.push(0.0);
                        rc_commands.pitch.push(0.0);
                        rc_commands.yaw.push(0.0);
                        rc_commands.throttle.push(0.0);
                    }
                }
                DataFrame::Event(_, _) => {
                    // Events don't contribute to telemetry time series
                }
            }
        }

        Ok(TelemetryData {
            sample_rate,
            gyro,
            accel,
            motor,
            pid_error,
            rc_commands,
            loop_time_variance: 0.0, // Calculate from actual data
            cpu_load: Vec::new(),    // Extract if available
        })
    }

    /// Build hardware configuration from header
    fn build_hardware_config(&self, header: &BlackboxHeader) -> HardwareConfiguration {
        // Extract hardware info from header - this is simplified
        HardwareConfiguration {
            flight_controller: FlightController {
                firmware: "Betaflight".to_string(),
                version: "4.4.0".to_string(),
                target: "STM32F405".to_string(),
                loop_rate: header.rates.pid_rate,
            },
            frame: Frame {
                wheelbase_mm: 220,
                weight_g: 650,
                material: "Carbon Fiber".to_string(),
                moment_of_inertia: None,
            },
            propulsion: PropulsionSystem {
                motors: MotorSpec {
                    model: "Unknown".to_string(),
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
                    model: "Unknown".to_string(),
                    current_rating: 35,
                    protocol: "DShot600".to_string(),
                },
            },
            pid_config: header.pid_config.clone(),
            filter_config: header.filter_config.clone(),
        }
    }

    /// Calculate flight duration from telemetry
    fn calculate_duration(&self, telemetry: &TelemetryData) -> u64 {
        if telemetry.sample_rate > 0.0 && telemetry.gyro.len() > 0 {
            ((telemetry.gyro.len() as f32 / telemetry.sample_rate) * 1000.0) as u64
        } else {
            0
        }
    }

    /// Build flight events from parsed event frames
    fn build_flight_events(&self) -> Vec<FlightEvent> {
        let mut events = Vec::new();
        
        for frame in &self.frames {
            if let DataFrame::Event(event_str, data) = frame {
                let event_type = if event_str.contains("ARMED") {
                    FlightEventType::Armed
                } else if event_str.contains("DISARMED") {
                    FlightEventType::Disarmed
                } else {
                    FlightEventType::Custom(event_str.clone())
                };
                
                events.push(FlightEvent {
                    timestamp_ms: 0, // Calculate from frame position
                    event_type,
                    data: Some(data.clone()),
                });
            }
        }
        
        events
    }

    /// Get parsing statistics
    pub fn stats(&self) -> &ParsingStats {
        &self.stats
    }
}

impl Default for BlackboxParser {
    fn default() -> Self {
        Self::new()
    }
}

// Default implementations for domain types used in parsing
impl Default for PidConfiguration {
    fn default() -> Self {
        Self {
            roll: PidValues { p: 42.0, i: 85.0, d: 38.0, f: Some(147.0) },
            pitch: PidValues { p: 46.0, i: 90.0, d: 42.0, f: Some(157.0) },
            yaw: PidValues { p: 45.0, i: 90.0, d: 0.0, f: Some(147.0) },
            settings: PidSettings {
                tpa: Some(TpaSettings { rate: 0.65, breakpoint: 1350.0 }),
                profile: 1,
                rates: RateSettings {
                    roll_rate: 670.0,
                    pitch_rate: 670.0,
                    yaw_rate: 670.0,
                    expo: ExpoSettings { roll: 0.0, pitch: 0.0, yaw: 0.0 },
                    super_rate: SuperRateSettings { roll: 0.80, pitch: 0.80, yaw: 0.80 },
                },
            },
        }
    }
}

impl Default for FilterConfiguration {
    fn default() -> Self {
        Self {
            gyro_filters: vec![
                Filter {
                    filter_type: FilterType::LowPass,
                    cutoff: 250.0,
                    order: 2,
                }
            ],
            dterm_filters: vec![
                Filter {
                    filter_type: FilterType::LowPass,
                    cutoff: 100.0,
                    order: 2,
                }
            ],
            notch_filters: vec![],
            dynamic_notch: Some(DynamicNotchSettings {
                min_freq: 150.0,
                max_freq: 600.0,
                q_factor: 120.0,
                enabled: true,
            }),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parser_creation() {
        let parser = BlackboxParser::new();
        assert_eq!(parser.stats.total_frames, 0);
        assert!(parser.header.is_none());
    }

    #[test]
    fn test_compression_detection() {
        let parser = BlackboxParser::new();
        
        // Test gzip magic number
        let gzip_data = [0x1f, 0x8b, 0x08, 0x00];
        assert!(parser.is_compressed(&gzip_data).unwrap());
        
        // Test non-compressed data
        let plain_data = [0x48, 0x20, 0x50, 0x72];
        assert!(!parser.is_compressed(&plain_data).unwrap());
    }

    #[test]
    fn test_variable_byte_parsing() {
        let parser = BlackboxParser::new();
        
        // Test single byte positive value
        let data = [0x2A]; // 42
        let (_, value) = parser.parse_signed_vb(&data).unwrap();
        assert_eq!(value, 42);
        
        // Test single byte negative value
        let data = [0x6A]; // -42 (with sign bit)
        let (_, value) = parser.parse_signed_vb(&data).unwrap();
        assert_eq!(value, -42);
    }

    #[test]
    fn test_empty_data_handling() {
        let mut parser = BlackboxParser::new();
        let result = parser.parse_file(&[]);
        assert!(result.is_err());
    }
}