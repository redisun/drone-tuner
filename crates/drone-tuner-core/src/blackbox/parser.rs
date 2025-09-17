//! Blackbox parser implementation using the blackbox-log crate.

use super::*;
use blackbox_log::{File, Headers, Event, frame::Frame};
// Removed unused imports
use std::time::Instant;

/// Blackbox parser that integrates with the blackbox-log crate
#[derive(Debug)]
pub struct BlackboxParser {
    /// Parsing configuration
    config: ParsingConfig,
    /// Field mappings extracted from headers
    field_mappings: FieldMappings,
    /// Parsing statistics
    stats: ParsingStats,
    /// Sample rate from headers
    sample_rate: f32,
}

impl BlackboxParser {
    /// Create a new blackbox parser with default configuration
    pub fn new() -> Self {
        Self::with_config(ParsingConfig::default())
    }

    /// Create a new blackbox parser with custom configuration
    pub fn with_config(config: ParsingConfig) -> Self {
        Self {
            config,
            field_mappings: FieldMappings::default(),
            stats: ParsingStats::default(),
            sample_rate: 1000.0, // Default sample rate
        }
    }

    /// Parse a blackbox file and return a flight session
    pub fn parse_file(&mut self, data: &[u8]) -> Result<FlightSession> {
        let start_time = Instant::now();
        info!("Starting blackbox parsing with blackbox-log crate, {} bytes", data.len());

        // Validate the input
        if !utils::is_blackbox_file(data) {
            return Err(BlackboxError::InvalidFormat("Invalid blackbox file format".to_string()).into());
        }

        self.stats.bytes_processed = data.len();

        // Create file from data
        let file = File::new(data)
            .map_err(|e| BlackboxError::Parse(format!("Failed to create file: {:?}", e)))?;

        info!("Blackbox file created successfully");

        // Parse headers
        let headers = file.headers();
        info!("Headers extracted successfully");

        // Extract field mappings and sample rate
        self.extract_field_mappings(&headers)?;
        self.extract_sample_rate(&headers);

        // Parse data frames
        let telemetry = self.parse_data_frames(&file)?;

        // Build flight session
        let session = self.build_flight_session(headers, telemetry)?;

        self.stats.parse_duration_ms = start_time.elapsed().as_millis() as u64;
        info!(
            "Parsing completed in {}ms: {} total frames processed",
            self.stats.parse_duration_ms,
            self.stats.total_frames
        );

        Ok(session)
    }

    /// Extract field mappings from headers
    fn extract_field_mappings(&mut self, headers: &Headers) -> BlackboxResult<()> {
        let mut field_names = Vec::new();

        // Collect field definitions from headers
        for (key, value) in headers.iter() {
            if key.starts_with("Field I") && value.contains(':') {
                if let Some(field_name) = value.split(':').nth(1) {
                    field_names.push(field_name.trim().to_string());
                }
            }
        }

        debug!("Found {} field definitions: {:?}", field_names.len(), field_names);

        // Map fields to our telemetry structure
        self.field_mappings = self.create_field_mappings(&field_names)?;

        Ok(())
    }

    /// Create field mappings from field names
    fn create_field_mappings(&self, field_names: &[String]) -> BlackboxResult<FieldMappings> {
        let mut mappings = FieldMappings::default();

        for (index, field_name) in field_names.iter().enumerate() {
            match field_name.as_str() {
                // Gyro fields
                "gyroADC[0]" | "gyroUnfilt[0]" => {
                    mappings.gyro_indices = Some((index, mappings.gyro_indices.map_or(0, |(_, y, z)| y), mappings.gyro_indices.map_or(0, |(_, _, z)| z)));
                }
                "gyroADC[1]" | "gyroUnfilt[1]" => {
                    mappings.gyro_indices = Some((mappings.gyro_indices.map_or(0, |(x, _, _)| x), index, mappings.gyro_indices.map_or(0, |(_, _, z)| z)));
                }
                "gyroADC[2]" | "gyroUnfilt[2]" => {
                    mappings.gyro_indices = Some((mappings.gyro_indices.map_or(0, |(x, _, _)| x), mappings.gyro_indices.map_or(0, |(_, y, _)| y), index));
                }

                // Accelerometer fields
                "accSmooth[0]" | "acc[0]" => {
                    mappings.accel_indices = Some((index, mappings.accel_indices.map_or(0, |(_, y, z)| y), mappings.accel_indices.map_or(0, |(_, _, z)| z)));
                }
                "accSmooth[1]" | "acc[1]" => {
                    mappings.accel_indices = Some((mappings.accel_indices.map_or(0, |(x, _, _)| x), index, mappings.accel_indices.map_or(0, |(_, _, z)| z)));
                }
                "accSmooth[2]" | "acc[2]" => {
                    mappings.accel_indices = Some((mappings.accel_indices.map_or(0, |(x, _, _)| x), mappings.accel_indices.map_or(0, |(_, y, _)| y), index));
                }

                // Motor outputs
                field if field.starts_with("motor[") => {
                    mappings.motor_indices.push(index);
                }

                // RC commands
                "rcCommand[0]" => {
                    mappings.rc_command_indices = Some((index, mappings.rc_command_indices.map_or(0, |(_, p, y, t)| p), mappings.rc_command_indices.map_or(0, |(_, _, y, t)| y), mappings.rc_command_indices.map_or(0, |(_, _, _, t)| t)));
                }
                "rcCommand[1]" => {
                    mappings.rc_command_indices = Some((mappings.rc_command_indices.map_or(0, |(r, _, _, _)| r), index, mappings.rc_command_indices.map_or(0, |(_, _, y, t)| y), mappings.rc_command_indices.map_or(0, |(_, _, _, t)| t)));
                }
                "rcCommand[2]" => {
                    mappings.rc_command_indices = Some((mappings.rc_command_indices.map_or(0, |(r, _, _, _)| r), mappings.rc_command_indices.map_or(0, |(_, p, _, _)| p), index, mappings.rc_command_indices.map_or(0, |(_, _, _, t)| t)));
                }
                "rcCommand[3]" => {
                    mappings.rc_command_indices = Some((mappings.rc_command_indices.map_or(0, |(r, _, _, _)| r), mappings.rc_command_indices.map_or(0, |(_, p, _, _)| p), mappings.rc_command_indices.map_or(0, |(_, _, y, _)| y), index));
                }

                _ => {
                    debug!("Unknown field: {}", field_name);
                }
            }
        }

        debug!("Field mappings created: {:?}", mappings);
        Ok(mappings)
    }

    /// Extract sample rate from headers
    fn extract_sample_rate(&mut self, headers: &Headers) {
        // Look for loop time or rate information
        for (key, value) in headers.iter() {
            if key.contains("looptime") || key.contains("pid_rate") {
                if let Ok(rate_us) = value.parse::<f32>() {
                    self.sample_rate = 1_000_000.0 / rate_us; // Convert microseconds to Hz
                    break;
                }
            } else if key.contains("I interval") {
                if let Ok(interval) = value.parse::<f32>() {
                    self.sample_rate = 32000.0 / interval; // Betaflight specific calculation
                    break;
                }
            }
        }

        // Default fallback
        if self.sample_rate <= 0.0 {
            self.sample_rate = 1000.0; // 1kHz default
            warn!("Could not determine sample rate from headers, using default 1kHz");
        }

        info!("Sample rate: {:.1} Hz", self.sample_rate);
    }

    /// Parse data frames and convert to telemetry
    fn parse_data_frames(&mut self, file: &File) -> BlackboxResult<TelemetryData> {
        let capacity = (self.stats.bytes_processed / 50).max(1000); // Estimate capacity
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

        let mut frame_count = 0u64;

        // Get the first data parser
        if let Some(data_parser) = file.data_parsers().get(0) {
            // Skip initial frames if configured
            for _ in 0..self.config.skip_frames {
                if data_parser.next().is_none() {
                    break;
                }
            }

            // Parse frames
            while let Some(event_result) = data_parser.next() {
                if let Some(max_frames) = self.config.max_frames {
                    if frame_count >= max_frames {
                        break;
                    }
                }

                match event_result {
                    Ok(event) => {
                        match event {
                            Event::Frame(frame) => {
                                self.process_frame(&frame, &mut gyro, &mut accel, &mut motor, &mut pid_error, &mut rc_commands)?;
                                frame_count += 1;
                                self.stats.total_frames += 1;

                                // Progress reporting
                                if self.config.progress_reporting && frame_count % self.config.progress_interval == 0 {
                                    debug!("Processed {} frames", frame_count);
                                }
                            }
                            Event::Event(_event_frame) => {
                                self.stats.event_frames += 1;
                                // Could process events here if needed
                            }
                            Event::Gps(_gps_frame) => {
                                self.stats.gps_frames += 1;
                                // GPS data not needed for basic tuning analysis
                            }
                        }
                    }
                    Err(e) => {
                        if self.config.strict_parsing {
                            return Err(BlackboxError::Parse(format!("Frame parsing error: {:?}", e)));
                        } else {
                            warn!("Skipping corrupted frame: {:?}", e);
                            self.stats.corrupted_frames += 1;
                        }
                    }
                }
            }
        } else {
            return Err(BlackboxError::MissingFields {
                missing_fields: vec!["No data parsers found".to_string()]
            });
        }

        info!("Processed {} frames successfully", frame_count);

        Ok(TelemetryData {
            sample_rate: self.sample_rate,
            gyro,
            accel,
            motor,
            pid_error,
            rc_commands,
            loop_time_variance: 0.0, // Would need to calculate from actual timing data
            cpu_load: Vec::new(),    // Not available in most logs
        })
    }

    /// Process a single frame and extract telemetry data
    fn process_frame(
        &self,
        frame: &Frame,
        gyro: &mut TimeSeriesVector3,
        accel: &mut TimeSeriesVector3,
        motor: &mut Vec<MotorTrace>,
        pid_error: &mut PidErrorTrace,
        rc_commands: &mut RcCommandTrace,
    ) -> BlackboxResult<()> {
        match frame {
            Frame::Main(main_frame) => {
                self.stats.main_frames.clone_from(&self.stats.main_frames) + 1;
                self.process_main_frame(main_frame, gyro, accel, motor, pid_error, rc_commands)?;
            }
            Frame::Slow(_slow_frame) => {
                // Slow frames contain less frequent data like GPS, battery, etc.
                // For tuning analysis, we primarily need the main frame data
                self.stats.slow_frames.clone_from(&self.stats.slow_frames) + 1;
            }
        }

        Ok(())
    }

    /// Process a main frame containing primary telemetry data
    fn process_main_frame(
        &self,
        main_frame: &blackbox_log::frame::MainFrame,
        gyro: &mut TimeSeriesVector3,
        accel: &mut TimeSeriesVector3,
        motor: &mut Vec<MotorTrace>,
        pid_error: &mut PidErrorTrace,
        rc_commands: &mut RcCommandTrace,
    ) -> BlackboxResult<()> {
        let values = main_frame.values();

        // Extract gyro data
        if let Some((x_idx, y_idx, z_idx)) = self.field_mappings.gyro_indices {
            if let (Some(&x), Some(&y), Some(&z)) = (values.get(x_idx), values.get(y_idx), values.get(z_idx)) {
                gyro.push(nalgebra::Vector3::new(
                    x as f32 / 16.0, // Typical gyro scale factor
                    y as f32 / 16.0,
                    z as f32 / 16.0,
                ));
            }
        }

        // Extract accelerometer data
        if let Some((x_idx, y_idx, z_idx)) = self.field_mappings.accel_indices {
            if let (Some(&x), Some(&y), Some(&z)) = (values.get(x_idx), values.get(y_idx), values.get(z_idx)) {
                accel.push(nalgebra::Vector3::new(
                    x as f32 / 512.0, // Typical accelerometer scale factor
                    y as f32 / 512.0,
                    z as f32 / 512.0,
                ));
            }
        }

        // Extract motor data
        for (motor_idx, &field_idx) in self.field_mappings.motor_indices.iter().enumerate() {
            if let Some(&motor_value) = values.get(field_idx) {
                if let Some(motor_trace) = motor.get_mut(motor_idx) {
                    motor_trace.values.push(motor_value as f32 / 2000.0); // Normalize to 0-1 range
                }
            }
        }

        // Extract RC commands
        if let Some((r_idx, p_idx, y_idx, t_idx)) = self.field_mappings.rc_command_indices {
            if let (Some(&roll), Some(&pitch), Some(&yaw), Some(&throttle)) = (
                values.get(r_idx),
                values.get(p_idx),
                values.get(y_idx),
                values.get(t_idx),
            ) {
                rc_commands.roll.push(roll as f32);
                rc_commands.pitch.push(pitch as f32);
                rc_commands.yaw.push(yaw as f32);
                rc_commands.throttle.push(throttle as f32);
            }
        }

        // Extract PID errors (if available)
        if let Some((r_idx, p_idx, y_idx)) = self.field_mappings.pid_error_indices {
            if let (Some(&roll), Some(&pitch), Some(&yaw)) = (
                values.get(r_idx),
                values.get(p_idx),
                values.get(y_idx),
            ) {
                pid_error.roll.push(roll as f32);
                pid_error.pitch.push(pitch as f32);
                pid_error.yaw.push(yaw as f32);
            }
        } else {
            // If no PID error data, fill with zeros
            pid_error.roll.push(0.0);
            pid_error.pitch.push(0.0);
            pid_error.yaw.push(0.0);
        }

        Ok(())
    }

    /// Build a complete flight session from parsed data
    fn build_flight_session(&self, headers: Headers, telemetry: TelemetryData) -> BlackboxResult<FlightSession> {
        let session_id = Uuid::new_v4();
        let timestamp = Utc::now(); // In practice, extract from headers if available

        // Build hardware configuration from headers
        let hardware = self.build_hardware_config(&headers);

        // Calculate flight duration
        let duration_ms = if telemetry.sample_rate > 0.0 && !telemetry.gyro.is_empty() {
            ((telemetry.gyro.len() as f32 / telemetry.sample_rate) * 1000.0) as u64
        } else {
            0
        };

        let metadata = FlightMetadata {
            session_id,
            timestamp,
            duration_ms,
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

        let events = Vec::new(); // Could extract from event frames

        Ok(FlightSession {
            metadata,
            telemetry,
            events,
            analysis_results: None,
        })
    }

    /// Build hardware configuration from headers
    fn build_hardware_config(&self, headers: &Headers) -> HardwareConfiguration {
        let mut firmware = "Unknown".to_string();
        let mut version = "Unknown".to_string();
        let mut target = "Unknown".to_string();

        // Extract firmware information from headers
        for (key, value) in headers.iter() {
            if key.contains("Firmware") {
                if value.contains("Betaflight") {
                    firmware = "Betaflight".to_string();
                } else if value.contains("INAV") {
                    firmware = "INAV".to_string();
                } else if value.contains("ArduPilot") {
                    firmware = "ArduPilot".to_string();
                }

                // Extract version
                if let Some(version_part) = value.split_whitespace().find(|s| s.contains('.')) {
                    version = version_part.to_string();
                }
            }

            if key.contains("Board") || key.contains("Target") {
                target = value.clone();
            }
        }

        HardwareConfiguration {
            flight_controller: FlightController {
                firmware,
                version,
                target,
                loop_rate: self.sample_rate as u32,
            },
            frame: Frame {
                wheelbase_mm: 220, // Default values - would need to be extracted or configured
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
            pid_config: PidConfiguration::default(),
            filter_config: FilterConfiguration::default(),
        }
    }

    /// Get parsing statistics
    pub fn stats(&self) -> &ParsingStats {
        &self.stats
    }

    /// Get field mappings used during parsing
    pub fn field_mappings(&self) -> &FieldMappings {
        &self.field_mappings
    }
}

impl Default for BlackboxParser {
    fn default() -> Self {
        Self::new()
    }
}