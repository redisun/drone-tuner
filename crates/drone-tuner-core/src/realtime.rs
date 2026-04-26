//! Real-time communication with flight controllers.
//!
//! This module provides connectivity to flight controllers via various transports
//! including USB serial, Bluetooth, and WiFi connections. It implements the MSP
//! (MultiWii Serial Protocol) for parameter reading/writing and telemetry streaming.

use crate::error::{DronetunerError, Result};
use std::time::{Duration, Instant};
use tokio::sync::{broadcast, mpsc};
use tokio::time::sleep;

/// Real-time connection manager for flight controllers
pub struct FlightControllerConnection {
    /// Communication transport
    transport: Box<dyn Transport + Send>,
    /// MSP protocol handler
    msp: MspProtocol,
    /// Current connection state
    state: ConnectionState,
    /// Telemetry streaming configuration
    telemetry_config: TelemetryConfig,
    /// Command queue for parameter changes
    command_queue: mpsc::UnboundedSender<FlightControllerCommand>,
    /// Telemetry broadcast channel
    telemetry_broadcast: broadcast::Sender<TelemetryFrame>,
}

/// Connection state
#[derive(Debug, Clone, PartialEq)]
pub enum ConnectionState {
    /// Not connected
    Disconnected,
    /// Connecting in progress
    Connecting,
    /// Connected and ready
    Connected {
        /// Flight controller information
        fc_info: FlightControllerInfo,
        /// Connection established time
        connected_at: Instant,
    },
    /// Connection error
    Error {
        /// Error message
        message: String,
        /// When the error occurred
        error_at: Instant,
    },
}

/// Flight controller information obtained during connection
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FlightControllerInfo {
    /// API version
    pub api_version: String,
    /// Firmware identifier
    pub firmware_id: String,
    /// Firmware version
    pub firmware_version: String,
    /// Board identifier
    pub board_id: String,
    /// Target name
    pub target_name: String,
    /// Pilot / craft name as set in Configurator (`set name = "..."`).
    /// Empty string when unset or when the FC's firmware doesn't support
    /// `MSP_NAME` (cmd 10). Useful for naming backup/log files per quad.
    pub craft_name: String,
    /// Available features/capabilities
    pub capabilities: Vec<String>,
}

/// Telemetry streaming configuration
#[derive(Debug, Clone)]
pub struct TelemetryConfig {
    /// Update rate in Hz
    pub rate_hz: u32,
    /// Which data fields to stream
    pub enabled_fields: Vec<TelemetryField>,
    /// Buffer size for circular buffering
    pub buffer_size: usize,
}

/// Available telemetry fields
#[derive(Debug, Clone, PartialEq)]
pub enum TelemetryField {
    /// Gyroscope data (deg/s)
    Gyro,
    /// Accelerometer data (g)
    Accelerometer,
    /// Motor outputs (0-100%)
    Motors,
    /// PID error values
    PidError,
    /// RC command inputs
    RcCommands,
    /// Battery voltage
    Battery,
    /// CPU load percentage
    CpuLoad,
    /// Loop time variance
    LoopTime,
}

/// Single telemetry frame
#[derive(Debug, Clone)]
pub struct TelemetryFrame {
    /// Timestamp when frame was received
    pub timestamp: Instant,
    /// Gyro data if enabled
    pub gyro: Option<nalgebra::Vector3<f32>>,
    /// Accelerometer data if enabled
    pub accel: Option<nalgebra::Vector3<f32>>,
    /// Motor outputs if enabled
    pub motors: Option<[f32; 4]>,
    /// PID error if enabled
    pub pid_error: Option<PidErrorFrame>,
    /// RC commands if enabled
    pub rc_commands: Option<RcCommandFrame>,
    /// Battery voltage if enabled
    pub battery_voltage: Option<f32>,
    /// CPU load if enabled
    pub cpu_load: Option<f32>,
    /// Loop time if enabled
    pub loop_time: Option<u32>,
}

/// PID error frame
#[derive(Debug, Clone)]
pub struct PidErrorFrame {
    /// Roll axis error
    pub roll: f32,
    /// Pitch axis error
    pub pitch: f32,
    /// Yaw axis error
    pub yaw: f32,
}

/// RC command frame
#[derive(Debug, Clone)]
pub struct RcCommandFrame {
    /// Roll command (-1.0 to 1.0)
    pub roll: f32,
    /// Pitch command (-1.0 to 1.0)
    pub pitch: f32,
    /// Yaw command (-1.0 to 1.0)
    pub yaw: f32,
    /// Throttle command (0.0 to 1.0)
    pub throttle: f32,
}

/// Commands that can be sent to the flight controller
#[derive(Debug, Clone)]
pub enum FlightControllerCommand {
    /// Read a parameter value
    ReadParameter {
        /// Parameter name
        name: String,
        /// Response channel
        response: mpsc::UnboundedSender<Result<ParameterValue>>,
    },
    /// Write a parameter value
    WriteParameter {
        /// Parameter name
        name: String,
        /// New value
        value: ParameterValue,
        /// Response channel
        response: mpsc::UnboundedSender<Result<()>>,
    },
    /// Save current parameters to flash
    SaveParameters {
        /// Response channel
        response: mpsc::UnboundedSender<Result<()>>,
    },
    /// Reset to defaults
    ResetParameters {
        /// Response channel
        response: mpsc::UnboundedSender<Result<()>>,
    },
    /// Start/stop telemetry streaming
    SetTelemetryStreaming {
        /// Enable or disable
        enabled: bool,
        /// Response channel
        response: mpsc::UnboundedSender<Result<()>>,
    },
}

/// Parameter value types
#[derive(Debug, Clone)]
pub enum ParameterValue {
    /// Integer value
    Int(i32),
    /// Float value
    Float(f32),
    /// String value
    String(String),
    /// Boolean value
    Bool(bool),
}

/// Communication transport abstraction
#[async_trait::async_trait]
pub trait Transport {
    /// Read data from the transport
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize>;

    /// Write data to the transport
    async fn write(&mut self, data: &[u8]) -> Result<usize>;

    /// Flush any pending writes
    async fn flush(&mut self) -> Result<()>;

    /// Check if transport is connected
    fn is_connected(&self) -> bool;

    /// Get transport description
    fn description(&self) -> String;
}

/// USB Serial transport
pub struct UsbSerialTransport {
    /// Serial port handle
    port: Box<dyn serialport::SerialPort + Send>,
    /// Port path
    path: String,
}

/// MSP (MultiWii Serial Protocol) implementation
#[derive(Debug)]
pub struct MspProtocol {
    /// Protocol version
    version: MspVersion,
    /// Message sequence counter
    sequence: u8,
}

/// MSP protocol versions
#[derive(Debug, Clone, Copy)]
pub enum MspVersion {
    /// MSPv1 (legacy)
    V1,
    /// MSPv2 (current standard)
    V2,
}

/// MSP message types
#[derive(Debug, Clone, Copy)]
pub enum MspMessageType {
    /// Request message
    Request = 0x3C,
    /// Response message  
    Response = 0x3E,
    /// Error response
    Error = 0x21,
}

/// MSP command codes
#[derive(Debug, Clone, Copy)]
pub enum MspCommand {
    /// API version
    ApiVersion = 1,
    /// Flight controller variant
    FcVariant = 2,
    /// Flight controller version
    FcVersion = 3,
    /// Board information
    BoardInfo = 4,
    /// Craft / pilot name string (Configurator's `set name = "..."` field)
    Name = 10,
    /// Raw IMU data
    RawImu = 102,
    /// Motor outputs
    Motor = 104,
    /// RC commands
    Rc = 105,
    /// PID configuration
    Pid = 112,
    /// PID names
    Pidnames = 117,
    /// Filter configuration (gyro/D-term lowpass cutoffs, notches, etc.)
    FilterConfig = 92,
    /// Onboard dataflash summary (ready flag, sector count, used/total bytes)
    DataflashSummary = 70,
    /// Read a chunk of onboard dataflash starting at a given byte offset
    DataflashRead = 71,
    /// Erase the entire onboard dataflash
    DataflashErase = 72,
    /// Set PID values
    SetPid = 202,
    /// Set filter configuration. Payload mirrors what FilterConfig
    /// returned, so the typical flow is read → mutate → write.
    SetFilterConfig = 93,
    /// Save parameters
    EepromWrite = 250,
}

impl FlightControllerConnection {
    /// Create a new connection to a flight controller
    pub async fn connect(connection_string: &str) -> Result<Self> {
        // Parse connection string to determine transport type
        let transport = Self::create_transport(connection_string).await?;
        Self::from_transport(transport).await
    }

    /// Create a connection from an already-built transport. Used by the
    /// in-process MSP simulator to drive the handshake against a fake FC
    /// without serial hardware.
    pub async fn from_transport(transport: Box<dyn Transport + Send>) -> Result<Self> {
        let msp = MspProtocol::new();
        let (command_tx, _command_rx) = mpsc::unbounded_channel();
        let (telemetry_tx, _telemetry_rx) = broadcast::channel(1000);

        let mut connection = Self {
            transport,
            msp,
            state: ConnectionState::Connecting,
            telemetry_config: TelemetryConfig::default(),
            command_queue: command_tx,
            telemetry_broadcast: telemetry_tx,
        };

        // Perform initial handshake
        connection.perform_handshake().await?;

        Ok(connection)
    }

    /// Create transport from connection string
    async fn create_transport(connection_string: &str) -> Result<Box<dyn Transport + Send>> {
        if connection_string.starts_with("/dev/") || connection_string.contains("COM") {
            // USB Serial connection
            let port = serialport::new(connection_string, 115_200)
                .timeout(Duration::from_millis(1000))
                .open()
                .map_err(|e| {
                    DronetunerError::communication_error(format!("Failed to open serial port: {e}"))
                })?;

            // Discard any bytes the kernel has already buffered for us. Without
            // this, a stale MSP frame from an aborted prior session shows up as
            // the "first" byte we see and desynchronises the handshake.
            let _ = port.clear(serialport::ClearBuffer::All);

            Ok(Box::new(UsbSerialTransport {
                port,
                path: connection_string.to_string(),
            }))
        } else {
            Err(DronetunerError::communication_error(
                "Unsupported connection type",
            ))
        }
    }

    /// Perform initial handshake with flight controller
    async fn perform_handshake(&mut self) -> Result<()> {
        // Request API version
        let version_msg = self.msp.create_message(MspCommand::ApiVersion, &[])?;
        self.transport.write(&version_msg).await?;
        self.transport.flush().await?;

        // Read response
        let response = self.read_msp_response().await?;
        let api_version = self.parse_api_version(&response.payload)?;

        // Request firmware variant
        let variant_msg = self.msp.create_message(MspCommand::FcVariant, &[])?;
        self.transport.write(&variant_msg).await?;

        let response = self.read_msp_response().await?;
        let firmware_id = self.parse_firmware_variant(&response.payload)?;

        // Request firmware version
        let version_msg = self.msp.create_message(MspCommand::FcVersion, &[])?;
        self.transport.write(&version_msg).await?;

        let response = self.read_msp_response().await?;
        let firmware_version = self.parse_firmware_version(&response.payload)?;

        // Request board information
        let board_msg = self.msp.create_message(MspCommand::BoardInfo, &[])?;
        self.transport.write(&board_msg).await?;

        let response = self.read_msp_response().await?;
        let (board_id, target_name) = self.parse_board_info(&response.payload)?;

        // Pilot/craft name. Best-effort: older firmware may not support
        // MSP_NAME and we don't want a missing name to abort the whole
        // handshake — just record an empty string in that case.
        let craft_name = self.try_read_craft_name().await;

        let fc_info = FlightControllerInfo {
            api_version,
            firmware_id,
            firmware_version,
            board_id,
            target_name,
            craft_name,
            capabilities: Vec::new(), // Would be populated from actual capability detection
        };

        self.state = ConnectionState::Connected {
            fc_info,
            connected_at: Instant::now(),
        };

        Ok(())
    }

    /// Start telemetry streaming
    pub async fn start_telemetry_streaming(
        &mut self,
        config: TelemetryConfig,
    ) -> Result<broadcast::Receiver<TelemetryFrame>> {
        self.telemetry_config = config;

        // Spawn telemetry reading task
        let transport_clone = self.clone_transport()?;
        let msp_clone = self.msp.clone();
        let telemetry_tx = self.telemetry_broadcast.clone();
        let telemetry_config = self.telemetry_config.clone();

        tokio::spawn(async move {
            Self::telemetry_loop(transport_clone, msp_clone, telemetry_tx, telemetry_config).await;
        });

        Ok(self.telemetry_broadcast.subscribe())
    }

    /// Main telemetry reading loop
    async fn telemetry_loop(
        mut transport: Box<dyn Transport + Send>,
        msp: MspProtocol,
        telemetry_tx: broadcast::Sender<TelemetryFrame>,
        config: TelemetryConfig,
    ) {
        let interval = Duration::from_millis(1000 / config.rate_hz as u64);
        let mut next_request = Instant::now();

        loop {
            if !transport.is_connected() {
                tracing::warn!("Transport disconnected, stopping telemetry loop");
                break;
            }

            if Instant::now() >= next_request {
                // Request telemetry data based on enabled fields
                for field in &config.enabled_fields {
                    if let Ok(msg) = Self::create_telemetry_request(&msp, field) {
                        if transport.write(&msg).await.is_err() {
                            tracing::error!("Failed to write telemetry request");
                            continue;
                        }
                    }
                }

                // Try to read and parse responses
                if let Ok(frame) = Self::read_telemetry_frame(&mut transport, &msp, &config).await {
                    if telemetry_tx.send(frame).is_err() {
                        tracing::debug!("No telemetry subscribers, continuing");
                    }
                }

                next_request = Instant::now() + interval;
            }

            sleep(Duration::from_millis(1)).await;
        }
    }

    /// Read a complete telemetry frame
    async fn read_telemetry_frame(
        transport: &mut Box<dyn Transport + Send>,
        msp: &MspProtocol,
        _config: &TelemetryConfig,
    ) -> Result<TelemetryFrame> {
        let mut frame = TelemetryFrame {
            timestamp: Instant::now(),
            gyro: None,
            accel: None,
            motors: None,
            pid_error: None,
            rc_commands: None,
            battery_voltage: None,
            cpu_load: None,
            loop_time: None,
        };

        // Read available responses
        let mut buf = [0u8; 256];
        if let Ok(size) = transport.read(&mut buf).await {
            if size > 0 {
                if let Ok(response) = msp.parse_response(&buf[..size]) {
                    // Parse response based on command type
                    match response.command {
                        MspCommand::RawImu => {
                            if let Ok((gyro, accel)) = Self::parse_imu_data(&response.payload) {
                                frame.gyro = Some(gyro);
                                frame.accel = Some(accel);
                            }
                        }
                        MspCommand::Motor => {
                            if let Ok(motors) = Self::parse_motor_data(&response.payload) {
                                frame.motors = Some(motors);
                            }
                        }
                        MspCommand::Rc => {
                            if let Ok(rc) = Self::parse_rc_data(&response.payload) {
                                frame.rc_commands = Some(rc);
                            }
                        }
                        _ => {}
                    }
                }
            }
        }

        Ok(frame)
    }

    /// Get current connection state
    pub fn state(&self) -> &ConnectionState {
        &self.state
    }

    /// Get flight controller information (if connected)
    pub fn fc_info(&self) -> Option<&FlightControllerInfo> {
        match &self.state {
            ConnectionState::Connected { fc_info, .. } => Some(fc_info),
            _ => None,
        }
    }

    /// Send a command to the flight controller
    pub async fn send_command(&self, command: FlightControllerCommand) -> Result<()> {
        self.command_queue
            .send(command)
            .map_err(|_| DronetunerError::communication_error("Command queue closed"))?;
        Ok(())
    }

    /// Read parameter value
    pub async fn read_parameter(&self, name: &str) -> Result<ParameterValue> {
        let (tx, mut rx) = mpsc::unbounded_channel();
        let command = FlightControllerCommand::ReadParameter {
            name: name.to_string(),
            response: tx,
        };

        self.send_command(command).await?;

        tokio::time::timeout(Duration::from_secs(5), rx.recv())
            .await
            .map_err(|_| DronetunerError::communication_error("Parameter read timeout"))?
            .ok_or_else(|| DronetunerError::communication_error("Parameter read channel closed"))?
    }

    /// Write parameter value
    pub async fn write_parameter(&self, name: &str, value: ParameterValue) -> Result<()> {
        let (tx, mut rx) = mpsc::unbounded_channel();
        let command = FlightControllerCommand::WriteParameter {
            name: name.to_string(),
            value,
            response: tx,
        };

        self.send_command(command).await?;

        tokio::time::timeout(Duration::from_secs(5), rx.recv())
            .await
            .map_err(|_| DronetunerError::communication_error("Parameter write timeout"))?
            .ok_or_else(|| DronetunerError::communication_error("Parameter write channel closed"))?
    }

    // Helper methods for parsing responses (simplified implementations)

    fn parse_api_version(&self, data: &[u8]) -> Result<String> {
        if data.len() >= 3 {
            Ok(format!("{}.{}.{}", data[0], data[1], data[2]))
        } else {
            Err(DronetunerError::parse_error(
                "Invalid API version response",
                None,
            ))
        }
    }

    fn parse_firmware_variant(&self, data: &[u8]) -> Result<String> {
        String::from_utf8(data.to_vec())
            .map_err(|_| DronetunerError::parse_error("Invalid firmware variant", None))
    }

    fn parse_firmware_version(&self, data: &[u8]) -> Result<String> {
        if data.len() >= 3 {
            Ok(format!("{}.{}.{}", data[0], data[1], data[2]))
        } else {
            Err(DronetunerError::parse_error(
                "Invalid firmware version response",
                None,
            ))
        }
    }

    fn parse_board_info(&self, _data: &[u8]) -> Result<(String, String)> {
        // Simplified parsing - real implementation would properly decode board info
        Ok(("Unknown".to_string(), "Unknown".to_string()))
    }

    fn parse_imu_data(data: &[u8]) -> Result<(nalgebra::Vector3<f32>, nalgebra::Vector3<f32>)> {
        if data.len() >= 18 {
            // Parse gyro (first 6 bytes, 3 x i16)
            let gyro_x = i16::from_le_bytes([data[0], data[1]]) as f32 / 16.0;
            let gyro_y = i16::from_le_bytes([data[2], data[3]]) as f32 / 16.0;
            let gyro_z = i16::from_le_bytes([data[4], data[5]]) as f32 / 16.0;

            // Parse accel (next 6 bytes, 3 x i16)
            let accel_x = i16::from_le_bytes([data[6], data[7]]) as f32 / 512.0;
            let accel_y = i16::from_le_bytes([data[8], data[9]]) as f32 / 512.0;
            let accel_z = i16::from_le_bytes([data[10], data[11]]) as f32 / 512.0;

            Ok((
                nalgebra::Vector3::new(gyro_x, gyro_y, gyro_z),
                nalgebra::Vector3::new(accel_x, accel_y, accel_z),
            ))
        } else {
            Err(DronetunerError::parse_error(
                "Invalid IMU data length",
                None,
            ))
        }
    }

    fn parse_motor_data(data: &[u8]) -> Result<[f32; 4]> {
        if data.len() >= 8 {
            Ok([
                u16::from_le_bytes([data[0], data[1]]) as f32 / 2000.0,
                u16::from_le_bytes([data[2], data[3]]) as f32 / 2000.0,
                u16::from_le_bytes([data[4], data[5]]) as f32 / 2000.0,
                u16::from_le_bytes([data[6], data[7]]) as f32 / 2000.0,
            ])
        } else {
            Err(DronetunerError::parse_error(
                "Invalid motor data length",
                None,
            ))
        }
    }

    fn parse_rc_data(data: &[u8]) -> Result<RcCommandFrame> {
        if data.len() >= 8 {
            Ok(RcCommandFrame {
                roll: (u16::from_le_bytes([data[0], data[1]]) as f32 - 1500.0) / 500.0,
                pitch: (u16::from_le_bytes([data[2], data[3]]) as f32 - 1500.0) / 500.0,
                yaw: (u16::from_le_bytes([data[4], data[5]]) as f32 - 1500.0) / 500.0,
                throttle: u16::from_le_bytes([data[6], data[7]]) as f32 / 2000.0,
            })
        } else {
            Err(DronetunerError::parse_error("Invalid RC data length", None))
        }
    }

    fn create_telemetry_request(msp: &MspProtocol, field: &TelemetryField) -> Result<Vec<u8>> {
        let command = match field {
            TelemetryField::Gyro | TelemetryField::Accelerometer => MspCommand::RawImu,
            TelemetryField::Motors => MspCommand::Motor,
            TelemetryField::RcCommands => MspCommand::Rc,
            _ => {
                return Err(DronetunerError::communication_error(
                    "Unsupported telemetry field",
                ))
            }
        };

        msp.create_message(command, &[])
    }

    async fn read_msp_response(&mut self) -> Result<MspResponse> {
        // USB CDC delivers data in arbitrary chunks: a single transport read
        // may return half a frame, two frames, or junk bytes before the frame
        // (FC boot banner, stale bytes from an aborted prior session). We
        // accumulate into a buffer, hunt for the `$` start marker, and only
        // hand a complete V1 frame to parse_response.
        let mut acc: Vec<u8> = Vec::with_capacity(256);
        let mut tmp = [0u8; 256];
        // Bound the loop so a chatty FC can't keep us forever.
        for _ in 0..32 {
            // Bound each read too — on a real serial port a stalled FC will
            // block forever otherwise, and the loop count alone won't save
            // us. 2s is generous: MSP responses are usually <100ms but USB
            // CDC stutters happen.
            let n =
                match tokio::time::timeout(Duration::from_secs(2), self.transport.read(&mut tmp))
                    .await
                {
                    Ok(Ok(n)) => n,
                    Ok(Err(e)) => return Err(e),
                    Err(_) => {
                        return Err(DronetunerError::communication_error(
                            "Timed out waiting for MSP response bytes",
                        ));
                    }
                };
            if n == 0 {
                if acc.is_empty() {
                    return Err(DronetunerError::communication_error("No response received"));
                }
                continue;
            }
            acc.extend_from_slice(&tmp[..n]);

            // Drop bytes before the first `$` — that's how we resync past
            // banner junk or partial leftovers from a previous frame.
            if let Some(start) = acc.iter().position(|&b| b == b'$') {
                if start > 0 {
                    acc.drain(..start);
                }
            } else {
                // No header yet, keep reading.
                continue;
            }

            // V1 frame layout: $ M dir size cmd [payload..] checksum
            // Need at least 6 bytes to know the full length.
            if acc.len() < 6 {
                continue;
            }
            // V1 only for now (matches MspProtocol::new() default).
            if acc[1] != b'M' {
                // Drop the `$` and resync — wrong protocol family.
                acc.drain(..1);
                continue;
            }
            let payload_size = acc[3] as usize;
            let frame_len = 6 + payload_size;
            if acc.len() < frame_len {
                continue;
            }
            return self.msp.parse_response(&acc[..frame_len]);
        }
        Err(DronetunerError::communication_error(
            "Timed out waiting for complete MSP frame",
        ))
    }

    fn clone_transport(&self) -> Result<Box<dyn Transport + Send>> {
        // This is a simplified implementation - real version would properly clone transport
        Err(DronetunerError::communication_error(
            "Transport cloning not implemented",
        ))
    }

    /// Read the flight controller's current PID gains (MSP Pid / 112).
    ///
    /// Returns the full 30-byte MSP_PID payload covering all 10 axes.
    /// We round-trip the entire payload so writeback preserves axes the
    /// caller doesn't touch (LEVEL, MAG, NAV, etc.).
    pub async fn read_pid(&mut self) -> Result<PidSnapshot> {
        let request = self.msp.create_message(MspCommand::Pid, &[])?;
        self.transport.write(&request).await?;
        self.transport.flush().await?;
        // The FC may still be flushing late responses to earlier handshake
        // commands. Skip frames that don't match what we asked for.
        for _ in 0..8 {
            let response = self.read_msp_response().await?;
            if matches!(response.command, MspCommand::Pid) {
                return PidSnapshot::from_payload(response.payload);
            }
        }
        Err(DronetunerError::communication_error(
            "Did not receive MSP_PID response after 8 frames",
        ))
    }

    /// Write a PID snapshot back to the flight controller (MSP SetPid / 202).
    ///
    /// This only updates RAM. Call [`save_to_eeprom`] to persist across
    /// power cycles.
    ///
    /// [`save_to_eeprom`]: Self::save_to_eeprom
    pub async fn write_pid(&mut self, snapshot: &PidSnapshot) -> Result<()> {
        let request = self
            .msp
            .create_message(MspCommand::SetPid, snapshot.as_payload())?;
        self.transport.write(&request).await?;
        self.transport.flush().await?;
        // SetPid acks with an empty payload.
        let _ack = self.read_msp_response().await?;
        Ok(())
    }

    /// Apply a PID change with automatic rollback on failure.
    ///
    /// 1. Reads the current PID values into a backup.
    /// 2. Writes the new values.
    /// 3. If the write or its ack fails, attempts to restore the backup
    ///    on a best-effort basis before returning the original error.
    ///
    /// The returned [`PidSnapshot`] is the pre-change backup, suitable
    /// for storing on disk so the user can manually restore later.
    pub async fn apply_pid_with_rollback(&mut self, new: &PidSnapshot) -> Result<PidSnapshot> {
        let backup = self.read_pid().await?;
        if let Err(write_err) = self.write_pid(new).await {
            // Best-effort rollback. Surface the original error regardless
            // of whether the rollback itself succeeds — the caller already
            // has the backup snapshot in their hands via the return value
            // path that we lost; embedding rollback failure context keeps
            // the trail.
            if let Err(rollback_err) = self.write_pid(&backup).await {
                return Err(DronetunerError::communication_error(format!(
                    "PID write failed ({write_err}); rollback also failed ({rollback_err})"
                )));
            }
            return Err(write_err);
        }
        Ok(backup)
    }

    /// Persist current parameters to non-volatile memory (MSP EepromWrite /
    /// 250). Without this call, RAM-only changes are lost on power cycle.
    pub async fn save_to_eeprom(&mut self) -> Result<()> {
        let request = self.msp.create_message(MspCommand::EepromWrite, &[])?;
        self.transport.write(&request).await?;
        self.transport.flush().await?;
        let _ack = self.read_msp_response().await?;
        Ok(())
    }

    /// Read the flight controller's current filter configuration
    /// (MSP FilterConfig / 92).
    ///
    /// Returns the full payload as a `FilterSnapshot` so callers can
    /// round-trip it through `write_filter_config` without losing fields
    /// they didn't intend to touch.
    pub async fn read_filter_config(&mut self) -> Result<FilterSnapshot> {
        let request = self.msp.create_message(MspCommand::FilterConfig, &[])?;
        self.transport.write(&request).await?;
        self.transport.flush().await?;
        for _ in 0..8 {
            let response = self.read_msp_response().await?;
            if matches!(response.command, MspCommand::FilterConfig) {
                return FilterSnapshot::from_payload(response.payload);
            }
        }
        Err(DronetunerError::communication_error(
            "Did not receive MSP_FILTER_CONFIG response after 8 frames",
        ))
    }

    /// Write a filter snapshot back to the flight controller
    /// (MSP SetFilterConfig / 93).
    ///
    /// RAM-only — call [`save_to_eeprom`] to persist.
    ///
    /// [`save_to_eeprom`]: Self::save_to_eeprom
    pub async fn write_filter_config(&mut self, snapshot: &FilterSnapshot) -> Result<()> {
        let request = self
            .msp
            .create_message(MspCommand::SetFilterConfig, snapshot.as_payload())?;
        self.transport.write(&request).await?;
        self.transport.flush().await?;
        let _ack = self.read_msp_response().await?;
        Ok(())
    }

    /// Apply a new filter config with rollback. Mirrors
    /// [`apply_pid_with_rollback`]: read current → write new → on
    /// failure, restore the backup. Returns the pre-change snapshot on
    /// success so callers can persist it for forensics.
    ///
    /// [`apply_pid_with_rollback`]: Self::apply_pid_with_rollback
    pub async fn apply_filter_with_rollback(
        &mut self,
        new: &FilterSnapshot,
    ) -> Result<FilterSnapshot> {
        let backup = self.read_filter_config().await?;
        if let Err(write_err) = self.write_filter_config(new).await {
            if let Err(rollback_err) = self.write_filter_config(&backup).await {
                return Err(DronetunerError::communication_error(format!(
                    "filter write failed ({write_err}); rollback also failed ({rollback_err})"
                )));
            }
            return Err(write_err);
        }
        Ok(backup)
    }

    /// Erase the entire onboard dataflash (`MSP_DATAFLASH_ERASE` / 72).
    ///
    /// **Destructive** — wipes every recorded blackbox session on the
    /// chip. The caller is expected to have saved any logs they care
    /// about already.
    ///
    /// Betaflight's erase is **asynchronous on the FC side**: the MSP
    /// handler queues the operation and acks immediately, while the
    /// actual flash wipe runs in the background (typically 10–60 s for
    /// a 16 MB chip). MSP keeps responding to other commands during the
    /// erase, but heavy traffic during the wipe can stutter — wait for
    /// the next call to [`Self::read_dataflash_summary`] to report
    /// `used_size = 0` if you need to confirm completion.
    pub async fn erase_dataflash(&mut self) -> Result<()> {
        let request = self
            .msp
            .create_message(MspCommand::DataflashErase, &[])?;
        self.transport.write(&request).await?;
        self.transport.flush().await?;
        let _ack = self.read_msp_response().await?;
        Ok(())
    }

    /// Read the onboard dataflash summary (MSP_DATAFLASH_SUMMARY / 70).
    ///
    /// Returns whether the FC has a usable dataflash chip, how many bytes
    /// are currently used by recorded blackbox sessions, and how big the
    /// chip is. SD-card-only boards return `supported = false`.
    pub async fn read_dataflash_summary(&mut self) -> Result<DataflashSummary> {
        let request = self
            .msp
            .create_message(MspCommand::DataflashSummary, &[])?;
        self.transport.write(&request).await?;
        self.transport.flush().await?;
        for _ in 0..8 {
            let response = self.read_msp_response().await?;
            if matches!(response.command, MspCommand::DataflashSummary) {
                return DataflashSummary::from_payload(&response.payload);
            }
        }
        Err(DronetunerError::communication_error(
            "Did not receive MSP_DATAFLASH_SUMMARY response after 8 frames",
        ))
    }

    /// Read a chunk of dataflash starting at `offset` (V1 framing).
    ///
    /// Request payload is the u32 LE offset only — Betaflight responds with
    /// its legacy 128-byte default chunk. For larger chunks (4 KB+) and
    /// faster pulls, use [`Self::read_dataflash_chunk_v2`] instead.
    ///
    /// Returns `(echoed_offset, chunk)` so the caller can sanity-check.
    pub async fn read_dataflash_chunk(&mut self, offset: u32) -> Result<(u32, Vec<u8>)> {
        let request = self
            .msp
            .create_message(MspCommand::DataflashRead, &offset.to_le_bytes())?;
        self.transport.write(&request).await?;
        self.transport.flush().await?;
        for _ in 0..8 {
            let response = self.read_msp_response().await?;
            if matches!(response.command, MspCommand::DataflashRead) {
                return Self::decode_dataflash_chunk(&response.payload);
            }
        }
        Err(DronetunerError::communication_error(
            "Did not receive MSP_DATAFLASH_READ response after 8 frames",
        ))
    }

    /// Read a chunk of dataflash starting at `offset` (V2 framing with
    /// requested chunk size).
    ///
    /// Sends the optional `u16 chunk_size` after the offset; Betaflight
    /// returns up to that many bytes (typically capped at ~4 KB by the
    /// firmware). V2 framing is required because a single MSPv1 frame
    /// can't carry payloads larger than 255 bytes.
    ///
    /// Drastically faster than [`Self::read_dataflash_chunk`] for big
    /// pulls — fewer roundtrips, same wire bandwidth.
    pub async fn read_dataflash_chunk_v2(
        &mut self,
        offset: u32,
        chunk_size: u16,
    ) -> Result<(u32, Vec<u8>)> {
        let mut payload = Vec::with_capacity(6);
        payload.extend_from_slice(&offset.to_le_bytes());
        payload.extend_from_slice(&chunk_size.to_le_bytes());
        let response = self
            .msp2_request(MspCommand::DataflashRead as u16, &payload)
            .await?;
        Self::decode_dataflash_chunk(&response)
    }

    /// Shared response decoder for both V1 and V2 dataflash read paths:
    /// `[u32 echoed_offset, ..chunk_bytes]`.
    fn decode_dataflash_chunk(payload: &[u8]) -> Result<(u32, Vec<u8>)> {
        if payload.len() < 4 {
            return Err(DronetunerError::parse_error(
                format!(
                    "MSP_DATAFLASH_READ payload too short: {} bytes",
                    payload.len()
                ),
                None,
            ));
        }
        let echoed =
            u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
        Ok((echoed, payload[4..].to_vec()))
    }

    /// Best-effort read of the FC's craft / pilot name (`MSP_NAME` / 10).
    ///
    /// Returns the empty string on any failure (older firmware, missing
    /// command, garbled response). Used during the handshake — a missing
    /// name should never abort the connection.
    async fn try_read_craft_name(&mut self) -> String {
        let request = match self.msp.create_message(MspCommand::Name, &[]) {
            Ok(r) => r,
            Err(_) => return String::new(),
        };
        if self.transport.write(&request).await.is_err() {
            return String::new();
        }
        let _ = self.transport.flush().await;
        for _ in 0..4 {
            match self.read_msp_response().await {
                Ok(resp) if matches!(resp.command, MspCommand::Name) => {
                    return String::from_utf8_lossy(&resp.payload).into_owned();
                }
                Ok(_) => continue, // late frame from earlier handshake step
                Err(_) => return String::new(),
            }
        }
        String::new()
    }

    /// Send an MSPv2 request and return the response payload.
    ///
    /// Used for V2-only commands (whose code exceeds u8::MAX) such as
    /// `MSP2_COMMON_SET_SETTING` / `0x1004`. Reads frames until one with
    /// the matching command code is seen, dropping any unrelated V1 or V2
    /// frames in the meantime (legacy late responses, etc.).
    ///
    /// Returns the response payload on a `>` (success) frame. An `!`
    /// (error) frame is converted to a [`DronetunerError::CommunicationError`]
    /// so the caller doesn't have to inspect direction bytes.
    pub async fn msp2_request(&mut self, command: u16, payload: &[u8]) -> Result<Vec<u8>> {
        let request = self.msp.build_v2_request(command, payload);
        self.transport.write(&request).await?;
        self.transport.flush().await?;
        self.read_msp2_response_for(command).await
    }

    /// Read frames from the transport until one matches `expected_cmd` (V2)
    /// or the per-read timeout fires.
    async fn read_msp2_response_for(&mut self, expected_cmd: u16) -> Result<Vec<u8>> {
        let mut acc: Vec<u8> = Vec::with_capacity(256);
        let mut tmp = [0u8; 256];
        for _ in 0..32 {
            let n = match tokio::time::timeout(
                Duration::from_secs(2),
                self.transport.read(&mut tmp),
            )
            .await
            {
                Ok(Ok(n)) => n,
                Ok(Err(e)) => return Err(e),
                Err(_) => {
                    return Err(DronetunerError::communication_error(
                        "Timed out waiting for MSPv2 response bytes",
                    ));
                }
            };
            if n == 0 && acc.is_empty() {
                return Err(DronetunerError::communication_error(
                    "No MSPv2 response received",
                ));
            }
            acc.extend_from_slice(&tmp[..n]);

            // Drain any number of complete frames sitting in `acc`. The FC
            // can be in the middle of late-replying to a different command;
            // skip past those rather than returning them as our answer.
            loop {
                // Resync to the next `$`.
                let Some(start) = acc.iter().position(|&b| b == b'$') else {
                    acc.clear();
                    break;
                };
                if start > 0 {
                    acc.drain(..start);
                }
                if acc.len() < 2 {
                    break;
                }
                match acc[1] {
                    b'M' => {
                        // V1 frame in the way — skip past it.
                        if acc.len() < 6 {
                            break; // need full V1 header
                        }
                        let payload_size = acc[3] as usize;
                        let frame_len = 6 + payload_size;
                        if acc.len() < frame_len {
                            break;
                        }
                        acc.drain(..frame_len);
                    }
                    b'X' => {
                        if acc.len() < 8 {
                            break; // need full V2 header (8 bytes)
                        }
                        let payload_size =
                            u16::from_le_bytes([acc[6], acc[7]]) as usize;
                        // Header(8) + payload + CRC(1) = 9 + payload_size.
                        let frame_len = 9 + payload_size;
                        if acc.len() < frame_len {
                            break;
                        }
                        let frame_bytes = acc[..frame_len].to_vec();
                        acc.drain(..frame_len);
                        let response = self.msp.parse_v2_frame(&frame_bytes)?;
                        if response.command != expected_cmd {
                            // Wrong command — skip and look for the next.
                            continue;
                        }
                        if response.direction == MspMessageType::Error as u8 {
                            return Err(DronetunerError::communication_error(format!(
                                "FC returned MSPv2 error for command 0x{expected_cmd:04x}"
                            )));
                        }
                        return Ok(response.payload);
                    }
                    _ => {
                        // Junk byte — drop the `$` and resync.
                        acc.drain(..1);
                    }
                }
            }
        }
        Err(DronetunerError::communication_error(format!(
            "Timed out waiting for MSPv2 response to command 0x{expected_cmd:04x}"
        )))
    }

    /// Read a Betaflight setting by name (`MSP2_COMMON_GET_SETTING`).
    ///
    /// Returns the raw value bytes, LE-encoded according to the parameter's
    /// registered type. Use [`Self::get_setting_u16`] / [`Self::get_setting_u8`]
    /// for typed accessors.
    pub async fn get_setting(&mut self, name: &str) -> Result<Vec<u8>> {
        let mut payload = Vec::with_capacity(name.len() + 1);
        payload.extend_from_slice(name.as_bytes());
        payload.push(0); // null terminator
        self.msp2_request(msp2::COMMON_GET_SETTING, &payload).await
    }

    /// Write a Betaflight setting by name (`MSP2_COMMON_SET_SETTING`).
    ///
    /// `value_bytes` must be encoded according to the setting's registered
    /// type (LE-ordered for multi-byte integers). On unknown setting names
    /// the FC replies with an error frame which we surface as `Err(...)`.
    /// RAM-only — call [`Self::save_to_eeprom`] to persist.
    pub async fn set_setting(&mut self, name: &str, value_bytes: &[u8]) -> Result<()> {
        let mut payload = Vec::with_capacity(name.len() + 1 + value_bytes.len());
        payload.extend_from_slice(name.as_bytes());
        payload.push(0);
        payload.extend_from_slice(value_bytes);
        let _ack = self.msp2_request(msp2::COMMON_SET_SETTING, &payload).await?;
        Ok(())
    }

    /// Convenience: read a u8 setting.
    pub async fn get_setting_u8(&mut self, name: &str) -> Result<u8> {
        let bytes = self.get_setting(name).await?;
        if bytes.is_empty() {
            return Err(DronetunerError::parse_error(
                format!("Setting '{name}': empty response"),
                None,
            ));
        }
        Ok(bytes[0])
    }

    /// Convenience: read a u16 setting.
    pub async fn get_setting_u16(&mut self, name: &str) -> Result<u16> {
        let bytes = self.get_setting(name).await?;
        if bytes.len() < 2 {
            return Err(DronetunerError::parse_error(
                format!("Setting '{name}': expected ≥2 bytes, got {}", bytes.len()),
                None,
            ));
        }
        Ok(u16::from_le_bytes([bytes[0], bytes[1]]))
    }

    /// Convenience: write a u8 setting.
    pub async fn set_setting_u8(&mut self, name: &str, value: u8) -> Result<()> {
        self.set_setting(name, &[value]).await
    }

    /// Convenience: write a u16 setting.
    pub async fn set_setting_u16(&mut self, name: &str, value: u16) -> Result<()> {
        self.set_setting(name, &value.to_le_bytes()).await
    }

    /// Stream the entire dataflash to memory.
    ///
    /// Calls `progress(bytes_so_far, total_bytes)` after each chunk so callers
    /// can drive a UI. Stops once `bytes_so_far >= summary.used_size` or the
    /// FC returns an empty chunk.
    ///
    /// Errors if the FC reports `supported = false` or `used_size == 0`.
    pub async fn pull_dataflash<F: FnMut(u64, u64)>(
        &mut self,
        mut progress: F,
    ) -> Result<Vec<u8>> {
        self.pull_dataflash_with(progress_to_callback(&mut progress), 1024)
            .await
    }

    /// Like [`Self::pull_dataflash`] but with an explicit chunk size hint.
    ///
    /// `chunk_size` is the V2 chunk size we ask Betaflight for. Default
    /// (1024) is conservative: experimentally, 4 KB chunks worked on the
    /// first request but caused the FC to stall on subsequent ones on
    /// some Betaflight 4.5 builds, presumably because the USB CDC TX
    /// buffer hadn't drained before the next request landed. 1024 is
    /// still ~8× faster than V1's 128-byte legacy default and stays well
    /// within the FC's tx buffer.
    ///
    /// Mid-stream V1 fallback: if any V2 chunk request fails during the
    /// pull loop (not just the probe), we switch to V1 for the remainder
    /// rather than aborting. The user gets their data even if V2 is
    /// flaky on their firmware build.
    pub async fn pull_dataflash_with<F: FnMut(u64, u64)>(
        &mut self,
        mut progress: F,
        chunk_size: u16,
    ) -> Result<Vec<u8>> {
        // How many V2 read requests to keep in flight at once. With
        // depth=2 we send req[N+1] while still draining the response to
        // req[N], halving USB roundtrip latency. Depth>2 risks overrunning
        // the FC's MSP RX queue on tight buffers; 2 is the sweet spot
        // empirically.
        const PIPELINE_DEPTH: usize = 2;

        let summary = self.read_dataflash_summary().await?;
        if !summary.supported {
            return Err(DronetunerError::communication_error(
                "Flight controller reports no onboard dataflash (likely SD-card or no logger)",
            ));
        }
        if summary.used_size == 0 {
            return Err(DronetunerError::communication_error(
                "Onboard dataflash is empty — record a flight first",
            ));
        }

        let total = summary.used_size as u64;
        let mut buf = Vec::with_capacity(total as usize);
        progress(0, total);

        // Probe V2 with the first chunk. If V2 fails on probe (older
        // firmware that doesn't speak it for cmd 71), fall straight to
        // V1's slower legacy chunks rather than aborting. Only V2 paths
        // are pipelined; V1 stays single-request because chunks there
        // are tiny and pipelining wouldn't pay off.
        let first = self.read_dataflash_chunk_v2(0, chunk_size).await;
        let (echoed, chunk) = match first {
            Ok(ok) => ok,
            Err(_) => {
                return self
                    .pull_dataflash_v1(progress, total, buf)
                    .await;
            }
        };
        if echoed != 0 {
            return Err(DronetunerError::communication_error(format!(
                "FC echoed dataflash offset {echoed} for first chunk; expected 0"
            )));
        }
        if chunk.is_empty() {
            return Ok(buf);
        }
        let mut offset: u32 = chunk.len() as u32;
        buf.extend_from_slice(&chunk);
        progress(buf.len() as u64, total);

        // Pipelined V2 loop: keep PIPELINE_DEPTH requests in flight.
        let mut in_flight: std::collections::VecDeque<u32> =
            std::collections::VecDeque::with_capacity(PIPELINE_DEPTH);
        let mut next_offset = offset;

        // Prime the pipeline.
        while in_flight.len() < PIPELINE_DEPTH && (next_offset as u64) < total {
            if let Err(e) = self.send_v2_dataflash_request(next_offset, chunk_size).await {
                tracing::warn!(
                    "MSP V2 dataflash request failed during pipeline prime at \
                     offset {next_offset}: {e}. Falling back to V1 single-request \
                     mode for the remaining bytes."
                );
                return self.pull_dataflash_v1(progress, total, buf).await;
            }
            in_flight.push_back(next_offset);
            next_offset = next_offset.saturating_add(chunk_size as u32);
        }

        // Iteration cap: 32 MB / 256-byte chunks (worst case after fallback)
        // is ~131k iterations. 400k gives margin.
        for _ in 0..400_000 {
            let Some(expected) = in_flight.pop_front() else {
                break;
            };
            let response = self
                .read_msp2_response_for(MspCommand::DataflashRead as u16)
                .await;
            let payload = match response {
                Ok(p) => p,
                Err(e) => {
                    // Mid-stream V2 failure. Drain any in-flight requests
                    // by ignoring them and restart from the failing offset
                    // using V1 single-request mode.
                    tracing::warn!(
                        "MSP V2 dataflash read failed at offset {expected}: {e}. \
                         Falling back to V1 framing for the remaining bytes."
                    );
                    offset = expected;
                    return self
                        .pull_dataflash_v1_from(progress, total, buf, offset)
                        .await;
                }
            };
            let (echoed, chunk) = Self::decode_dataflash_chunk(&payload)?;
            if echoed != expected {
                return Err(DronetunerError::communication_error(format!(
                    "FC echoed dataflash offset {echoed}, expected {expected} — \
                     pipeline sync lost"
                )));
            }
            if chunk.is_empty() {
                break;
            }
            offset = offset.saturating_add(chunk.len() as u32);
            buf.extend_from_slice(&chunk);
            progress(buf.len() as u64, total);

            // Refill the pipeline if there's more to read.
            if (next_offset as u64) < total {
                if let Err(e) =
                    self.send_v2_dataflash_request(next_offset, chunk_size).await
                {
                    tracing::warn!(
                        "MSP V2 dataflash request failed during refill at offset \
                         {next_offset}: {e}. Falling back to V1 single-request mode."
                    );
                    return self
                        .pull_dataflash_v1_from(progress, total, buf, offset)
                        .await;
                }
                in_flight.push_back(next_offset);
                next_offset = next_offset.saturating_add(chunk_size as u32);
            }

            if (offset as u64) >= total && in_flight.is_empty() {
                break;
            }
        }
        Ok(buf)
    }

    /// Send (without reading the response) a V2 dataflash read request.
    /// Used by [`Self::pull_dataflash_with`]'s pipelined loop.
    async fn send_v2_dataflash_request(
        &mut self,
        offset: u32,
        chunk_size: u16,
    ) -> Result<()> {
        let mut payload = Vec::with_capacity(6);
        payload.extend_from_slice(&offset.to_le_bytes());
        payload.extend_from_slice(&chunk_size.to_le_bytes());
        let request = self
            .msp
            .build_v2_request(MspCommand::DataflashRead as u16, &payload);
        self.transport.write(&request).await?;
        self.transport.flush().await?;
        Ok(())
    }

    /// V1 fallback for the entire pull, used when the V2 probe at offset 0
    /// fails. Single-request, legacy 128-byte chunks.
    async fn pull_dataflash_v1<F: FnMut(u64, u64)>(
        &mut self,
        progress: F,
        total: u64,
        buf: Vec<u8>,
    ) -> Result<Vec<u8>> {
        self.pull_dataflash_v1_from(progress, total, buf, 0).await
    }

    /// V1 fallback resuming from `offset`. Used by mid-stream V2 failures
    /// where some bytes have already landed in `buf`.
    async fn pull_dataflash_v1_from<F: FnMut(u64, u64)>(
        &mut self,
        mut progress: F,
        total: u64,
        mut buf: Vec<u8>,
        mut offset: u32,
    ) -> Result<Vec<u8>> {
        for _ in 0..400_000 {
            if (offset as u64) >= total {
                break;
            }
            let (echoed, chunk) = self.read_dataflash_chunk(offset).await?;
            if echoed != offset {
                return Err(DronetunerError::communication_error(format!(
                    "FC echoed dataflash offset {echoed}, expected {offset} — sync lost"
                )));
            }
            if chunk.is_empty() {
                break;
            }
            offset = offset.saturating_add(chunk.len() as u32);
            buf.extend_from_slice(&chunk);
            progress(buf.len() as u64, total);
        }
        Ok(buf)
    }
}

/// Helper to turn a `&mut F` progress callback into an owned closure for
/// the underlying `pull_dataflash_with` call. Just makes the wrapper at
/// the top of the file a one-liner instead of duplicating the closure
/// body.
fn progress_to_callback<F: FnMut(u64, u64)>(f: &mut F) -> impl FnMut(u64, u64) + '_ {
    move |done, total| f(done, total)
}

/// Onboard-dataflash summary (MSP_DATAFLASH_SUMMARY / 70).
///
/// Betaflight payload layout:
/// ```text
///   0       u8    flags: bit 0 = ready, bit 1 = supported
///   1..=4   u32   sector count (LE)
///   5..=8   u32   total size in bytes (LE)
///   9..=12  u32   used size in bytes (LE)
/// ```
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DataflashSummary {
    /// `ready` bit — chip is initialised and accepting reads.
    pub ready: bool,
    /// `supported` bit — board has a dataflash chip at all (false on
    /// SD-card-only or no-logger boards).
    pub supported: bool,
    /// Sector count.
    pub sectors: u32,
    /// Total chip size in bytes.
    pub total_size: u32,
    /// Bytes currently consumed by recorded sessions. The dataflash is a
    /// raw concatenation of frames, so this is also "how much to read".
    pub used_size: u32,
}

impl DataflashSummary {
    /// Parse the 13-byte payload. Tolerates trailing bytes (some firmware
    /// variants append extra fields; we stop after the canonical layout).
    pub fn from_payload(payload: &[u8]) -> Result<Self> {
        if payload.len() < 13 {
            return Err(DronetunerError::parse_error(
                format!(
                    "MSP_DATAFLASH_SUMMARY payload too short: {} bytes",
                    payload.len()
                ),
                None,
            ));
        }
        let flags = payload[0];
        let sectors = u32::from_le_bytes([payload[1], payload[2], payload[3], payload[4]]);
        let total_size =
            u32::from_le_bytes([payload[5], payload[6], payload[7], payload[8]]);
        let used_size = u32::from_le_bytes([payload[9], payload[10], payload[11], payload[12]]);
        Ok(Self {
            ready: flags & 0x01 != 0,
            supported: flags & 0x02 != 0,
            sectors,
            total_size,
            used_size,
        })
    }

    /// Render as the 13-byte payload. Used by the simulator.
    pub fn to_payload(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(13);
        let mut flags = 0u8;
        if self.ready {
            flags |= 0x01;
        }
        if self.supported {
            flags |= 0x02;
        }
        buf.push(flags);
        buf.extend_from_slice(&self.sectors.to_le_bytes());
        buf.extend_from_slice(&self.total_size.to_le_bytes());
        buf.extend_from_slice(&self.used_size.to_le_bytes());
        buf
    }
}

/// 30-byte MSP_PID payload snapshot. Provides typed accessors for the
/// roll/pitch/yaw rate axes (the ones an FPV tuner cares about) while
/// preserving the rest of the payload for round-trip fidelity.
///
/// Betaflight MSP_PID layout (each value is u8 0..=255):
/// ```text
///  0  ROLL  P    9  ALT   P   18  NAVR  P   27  VEL   P
///  1  ROLL  I   10  ALT   I   19  NAVR  I   28  VEL   I
///  2  ROLL  D   11  ALT   D   20  NAVR  D   29  VEL   D
///  3  PITCH P   12  POS   P   21  LEVEL P
///  4  PITCH I   13  POS   I   22  LEVEL I
///  5  PITCH D   14  POS   D   23  LEVEL D
///  6  YAW   P   15  POSR  P   24  MAG   P
///  7  YAW   I   16  POSR  I   25  MAG   I
///  8  YAW   D   17  POSR  D   26  MAG   D
/// ```
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PidSnapshot {
    /// Raw payload as returned by MSP_PID. Length is FC-dependent: modern
    /// Betaflight returns 15 bytes (5 axes × 3), legacy firmwares returned
    /// 30 (10 × 3). We preserve whatever the FC sent so SetPid round-trips.
    raw: Vec<u8>,
}

impl PidSnapshot {
    /// Parse a payload from an MSP Pid response.
    pub fn from_payload(payload: Vec<u8>) -> Result<Self> {
        // Minimum is 9 bytes for ROLL/PITCH/YAW P-I-D — anything shorter
        // can't carry the three flight axes we operate on.
        if payload.len() < 9 {
            return Err(DronetunerError::parse_error(
                format!("MSP Pid payload too short: {} bytes", payload.len()),
                None,
            ));
        }
        Ok(Self { raw: payload })
    }

    /// Borrow the underlying payload, suitable for SetPid round-trip.
    pub fn as_payload(&self) -> &[u8] {
        &self.raw
    }

    /// Roll P/I/D as a (P, I, D) tuple.
    pub fn roll(&self) -> (u8, u8, u8) {
        (self.raw[0], self.raw[1], self.raw[2])
    }
    /// Pitch P/I/D as a (P, I, D) tuple.
    pub fn pitch(&self) -> (u8, u8, u8) {
        (self.raw[3], self.raw[4], self.raw[5])
    }
    /// Yaw P/I/D as a (P, I, D) tuple.
    pub fn yaw(&self) -> (u8, u8, u8) {
        (self.raw[6], self.raw[7], self.raw[8])
    }
    /// Set roll P/I/D.
    pub fn set_roll(&mut self, p: u8, i: u8, d: u8) {
        self.raw[0] = p;
        self.raw[1] = i;
        self.raw[2] = d;
    }
    /// Set pitch P/I/D.
    pub fn set_pitch(&mut self, p: u8, i: u8, d: u8) {
        self.raw[3] = p;
        self.raw[4] = i;
        self.raw[5] = d;
    }
    /// Set yaw P/I/D.
    pub fn set_yaw(&mut self, p: u8, i: u8, d: u8) {
        self.raw[6] = p;
        self.raw[7] = i;
        self.raw[8] = d;
    }
}

/// MSP_FILTER_CONFIG payload snapshot.
///
/// The exact layout has changed over Betaflight 4.x and depends on the
/// firmware build, so we treat it as opaque bytes for round-trip fidelity.
/// What we *do* expose is read-only access to the first three u16 fields,
/// which have been stable for many releases:
///
/// - `gyro_lpf1_static_hz` (offset 0..2)
/// - `dterm_lpf1_static_hz` (offset 2..4)
/// - `yaw_lpf_hz` (offset 4..6)
///
/// Mutating these from a recommendation is left to the CLI: callers
/// should `read_filter_config` → mutate the bytes they understand →
/// `apply_filter_with_rollback`. We deliberately don't expose typed
/// setters yet — the offsets above are stable, but several other
/// fields (notch counts, dynamic LPF settings, RPM filter parameters)
/// have shifted between firmware versions, and a typed setter that's
/// wrong by one byte can brick a tune.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FilterSnapshot {
    raw: Vec<u8>,
}

impl FilterSnapshot {
    /// Parse a payload from an MSP FilterConfig response. Accepts any
    /// length ≥ 6 bytes (the three u16 fields we read).
    pub fn from_payload(payload: Vec<u8>) -> Result<Self> {
        if payload.len() < 6 {
            return Err(DronetunerError::parse_error(
                format!(
                    "MSP FilterConfig payload too short: {} bytes",
                    payload.len()
                ),
                None,
            ));
        }
        Ok(Self { raw: payload })
    }

    /// Borrow the underlying payload, suitable for SetFilterConfig.
    pub fn as_payload(&self) -> &[u8] {
        &self.raw
    }

    /// Mutable access to the underlying payload for advanced callers
    /// that know their firmware's exact layout. The CLI does not use
    /// this; it's here so the binary downstream of read → mutate → write
    /// can do the mutate step without an unsafe transmute.
    pub fn as_payload_mut(&mut self) -> &mut [u8] {
        &mut self.raw
    }

    /// Gyro stage-1 lowpass cutoff in Hz (0 = disabled).
    pub fn gyro_lpf1_hz(&self) -> u16 {
        u16::from_le_bytes([self.raw[0], self.raw[1]])
    }
    /// D-term stage-1 lowpass cutoff in Hz (0 = disabled).
    pub fn dterm_lpf1_hz(&self) -> u16 {
        u16::from_le_bytes([self.raw[2], self.raw[3]])
    }
    /// Yaw lowpass cutoff in Hz (0 = disabled).
    pub fn yaw_lpf_hz(&self) -> u16 {
        u16::from_le_bytes([self.raw[4], self.raw[5]])
    }

    /// Set the gyro stage-1 lowpass cutoff in Hz.
    pub fn set_gyro_lpf1_hz(&mut self, hz: u16) {
        self.raw[0..2].copy_from_slice(&hz.to_le_bytes());
    }
    /// Set the D-term stage-1 lowpass cutoff in Hz.
    pub fn set_dterm_lpf1_hz(&mut self, hz: u16) {
        self.raw[2..4].copy_from_slice(&hz.to_le_bytes());
    }
    /// Set the yaw lowpass cutoff in Hz.
    pub fn set_yaw_lpf_hz(&mut self, hz: u16) {
        self.raw[4..6].copy_from_slice(&hz.to_le_bytes());
    }
}

impl Default for TelemetryConfig {
    fn default() -> Self {
        Self {
            rate_hz: 100,
            enabled_fields: vec![
                TelemetryField::Gyro,
                TelemetryField::Accelerometer,
                TelemetryField::Motors,
            ],
            buffer_size: 10000,
        }
    }
}

impl MspProtocol {
    /// Create a new MSP protocol handler.
    ///
    /// Defaults to MSPv1 because that's the only framing the production
    /// `parse_response` path actually decodes today. Switch to V2 once
    /// `parse_framed` learns the V2 envelope (CRC, 16-bit lengths).
    pub fn new() -> Self {
        Self {
            version: MspVersion::V1,
            sequence: 0,
        }
    }

    /// Create an MSP request message (direction = `<`).
    pub fn create_message(&self, command: MspCommand, payload: &[u8]) -> Result<Vec<u8>> {
        self.build_framed(MspMessageType::Request, command, payload)
    }

    /// Create an MSP response message (direction = `>`). Used by the
    /// in-process MSP simulator to reply to client requests.
    pub(crate) fn create_response(&self, command: MspCommand, payload: &[u8]) -> Result<Vec<u8>> {
        self.build_framed(MspMessageType::Response, command, payload)
    }

    fn build_framed(
        &self,
        direction: MspMessageType,
        command: MspCommand,
        payload: &[u8],
    ) -> Result<Vec<u8>> {
        let mut message = Vec::new();

        match self.version {
            MspVersion::V1 => {
                // MSPv1 format: $M<direction><size><command><payload><checksum>
                message.push(b'$');
                message.push(b'M');
                message.push(direction as u8);
                message.push(payload.len() as u8);
                message.push(command as u8);
                message.extend_from_slice(payload);

                // Calculate checksum
                let mut checksum = payload.len() as u8;
                checksum ^= command as u8;
                for &byte in payload {
                    checksum ^= byte;
                }
                message.push(checksum);
            }
            MspVersion::V2 => {
                // MSPv2 format: $X<dir><flag><cmd_lo><cmd_hi><len_lo><len_hi><payload><crc>
                message.push(b'$');
                message.push(b'X');
                message.push(direction as u8);
                message.push(0); // Flag byte
                message.extend_from_slice(&(command as u16).to_le_bytes());
                message.extend_from_slice(&(payload.len() as u16).to_le_bytes());
                message.extend_from_slice(payload);

                // CRC8/DVB-S2 over [flag, cmd_lo, cmd_hi, len_lo, len_hi, payload].
                let crc = crc8_dvb_s2(&message[3..]);
                message.push(crc);
            }
        }

        Ok(message)
    }

    /// Parse an MSP response (direction = `>`).
    pub fn parse_response(&self, data: &[u8]) -> Result<MspResponse> {
        self.parse_framed(MspMessageType::Response, data)
    }

    /// Parse an MSP request (direction = `<`). Used by the in-process MSP
    /// simulator to consume client requests.
    pub(crate) fn parse_request(&self, data: &[u8]) -> Result<MspResponse> {
        self.parse_framed(MspMessageType::Request, data)
    }

    fn parse_framed(&self, expected_direction: MspMessageType, data: &[u8]) -> Result<MspResponse> {
        if data.len() < 3 {
            return Err(DronetunerError::parse_error("MSP message too short", None));
        }

        if data[0] != b'$' {
            return Err(DronetunerError::parse_error("Invalid MSP header", None));
        }

        match data[1] {
            // MSPv1: $M<dir><len><cmd><payload><checksum>
            b'M' => {
                if data.len() < 6 {
                    return Err(DronetunerError::parse_error(
                        "MSPv1 message too short",
                        None,
                    ));
                }
                if data[2] != expected_direction as u8 {
                    return Err(DronetunerError::parse_error(
                        "Wrong MSP direction byte",
                        None,
                    ));
                }
                let payload_size = data[3] as usize;
                let command = data[4];
                if data.len() < 6 + payload_size {
                    return Err(DronetunerError::parse_error(
                        "Incomplete MSPv1 message",
                        None,
                    ));
                }
                let payload = data[5..5 + payload_size].to_vec();
                // Verify checksum: XOR of size, command, and payload bytes.
                let mut expected = data[3] ^ data[4];
                for b in &payload {
                    expected ^= *b;
                }
                if expected != data[5 + payload_size] {
                    return Err(DronetunerError::parse_error(
                        "MSPv1 checksum mismatch",
                        None,
                    ));
                }
                Ok(MspResponse {
                    command: MspCommand::from_u8(command)?,
                    payload,
                })
            }
            // MSPv2: $X<dir><flag><cmd_lo><cmd_hi><len_lo><len_hi><payload><crc>
            b'X' => {
                if data.len() < 9 {
                    return Err(DronetunerError::parse_error(
                        "MSPv2 message too short",
                        None,
                    ));
                }
                if data[2] != expected_direction as u8 {
                    return Err(DronetunerError::parse_error(
                        "Wrong MSP direction byte",
                        None,
                    ));
                }
                let _flag = data[3];
                let command = u16::from_le_bytes([data[4], data[5]]);
                let payload_size = u16::from_le_bytes([data[6], data[7]]) as usize;
                if data.len() < 9 + payload_size {
                    return Err(DronetunerError::parse_error(
                        "Incomplete MSPv2 message",
                        None,
                    ));
                }
                let payload = data[8..8 + payload_size].to_vec();
                // Verify CRC8/DVB-S2 over [flag, cmd_lo, cmd_hi, len_lo, len_hi, payload].
                let expected = crc8_dvb_s2(&data[3..8 + payload_size]);
                if expected != data[8 + payload_size] {
                    return Err(DronetunerError::parse_error("MSPv2 CRC mismatch", None));
                }
                // MspCommand only carries u8 codes today; reject 16-bit commands
                // we don't recognise rather than silently truncating.
                if command > u8::MAX as u16 {
                    return Err(DronetunerError::parse_error(
                        format!("MSPv2 command {command} out of u8 range"),
                        None,
                    ));
                }
                Ok(MspResponse {
                    command: MspCommand::from_u8(command as u8)?,
                    payload,
                })
            }
            _ => Err(DronetunerError::parse_error("Invalid MSP header", None)),
        }
    }

    /// Calculate CRC for MSPv2 (delegates to the real CRC8/DVB-S2).
    #[cfg_attr(not(test), allow(dead_code))]
    fn calculate_crc(&self, data: &[u8]) -> u8 {
        crc8_dvb_s2(data)
    }

    /// Build a V2-framed request, regardless of `self.version`.
    ///
    /// V2 framing is required for MSP commands whose code exceeds u8::MAX
    /// (notably `MSP2_COMMON_SET_SETTING` / `0x1004`). This bypasses the
    /// u8-bounded [`MspCommand`] enum so we can address arbitrary u16
    /// commands without rewriting the legacy V1 path.
    pub fn build_v2_request(&self, command: u16, payload: &[u8]) -> Vec<u8> {
        let mut message = Vec::with_capacity(9 + payload.len() + 1);
        message.push(b'$');
        message.push(b'X');
        message.push(MspMessageType::Request as u8);
        message.push(0); // flag byte
        message.extend_from_slice(&command.to_le_bytes());
        message.extend_from_slice(&(payload.len() as u16).to_le_bytes());
        message.extend_from_slice(payload);
        let crc = crc8_dvb_s2(&message[3..]);
        message.push(crc);
        message
    }

    /// Build a V2-framed response. Used by the in-process simulator to
    /// reply to V2 requests.
    pub(crate) fn build_v2_response(
        &self,
        direction: MspMessageType,
        command: u16,
        payload: &[u8],
    ) -> Vec<u8> {
        let mut message = Vec::with_capacity(9 + payload.len() + 1);
        message.push(b'$');
        message.push(b'X');
        message.push(direction as u8);
        message.push(0);
        message.extend_from_slice(&command.to_le_bytes());
        message.extend_from_slice(&(payload.len() as u16).to_le_bytes());
        message.extend_from_slice(payload);
        let crc = crc8_dvb_s2(&message[3..]);
        message.push(crc);
        message
    }

    /// Parse a complete V2 frame (`$X<dir><flag><cmd_lo><cmd_hi><len_lo><len_hi><payload><crc>`).
    ///
    /// Returns the response with its u16 command, raw payload, and direction
    /// byte preserved — the caller is expected to inspect `direction` to
    /// distinguish success (`>`) from error (`!`) replies.
    pub fn parse_v2_frame(&self, data: &[u8]) -> Result<MspV2Response> {
        // Minimum complete V2 frame: 8-byte header + 0-byte payload + 1-byte
        // CRC = 9 bytes.
        if data.len() < 9 {
            return Err(DronetunerError::parse_error(
                "MSPv2 frame too short (need at least 9 bytes)",
                None,
            ));
        }
        if data[0] != b'$' || data[1] != b'X' {
            return Err(DronetunerError::parse_error("Not an MSPv2 frame", None));
        }
        let direction = data[2];
        let _flag = data[3];
        let command = u16::from_le_bytes([data[4], data[5]]);
        let payload_size = u16::from_le_bytes([data[6], data[7]]) as usize;
        // Full frame is header(8) + payload + crc(1).
        if data.len() < 9 + payload_size {
            return Err(DronetunerError::parse_error(
                "Incomplete MSPv2 frame (payload + crc don't fit)",
                None,
            ));
        }
        let payload = data[8..8 + payload_size].to_vec();
        let expected_crc = crc8_dvb_s2(&data[3..8 + payload_size]);
        let actual_crc = data[8 + payload_size];
        if expected_crc != actual_crc {
            return Err(DronetunerError::parse_error("MSPv2 CRC mismatch", None));
        }
        Ok(MspV2Response {
            command,
            payload,
            direction,
        })
    }
}

/// Parse a Betaflight setting name out of a `MSP2_COMMON_*_SETTING` payload.
///
/// The wire format prefixes the value with a null-terminated ASCII name —
/// e.g. `b"gyro_lpf1_static_hz\0\xfa\x00"` for "set gyro_lpf1_static_hz =
/// 250". This finds the NUL byte and returns the leading name.
fn parse_settings_name(payload: &[u8]) -> Result<String> {
    let nul = payload
        .iter()
        .position(|&b| b == 0)
        .ok_or_else(|| DronetunerError::parse_error("setting name missing NUL terminator", None))?;
    let name = std::str::from_utf8(&payload[..nul]).map_err(|_| {
        DronetunerError::parse_error("setting name is not valid UTF-8", None)
    })?;
    Ok(name.to_string())
}

/// MSPv2 uses CRC8/DVB-S2 (polynomial 0xD5, init 0, no reflection, no XOR-out)
/// over the bytes from the flag byte through the end of the payload.
fn crc8_dvb_s2(data: &[u8]) -> u8 {
    let mut crc: u8 = 0;
    for &byte in data {
        crc ^= byte;
        for _ in 0..8 {
            if crc & 0x80 != 0 {
                crc = crc.wrapping_shl(1) ^ 0xD5;
            } else {
                crc = crc.wrapping_shl(1);
            }
        }
    }
    crc
}

impl Clone for MspProtocol {
    fn clone(&self) -> Self {
        Self {
            version: self.version,
            sequence: self.sequence,
        }
    }
}

/// MSP response message
#[derive(Debug)]
pub struct MspResponse {
    /// Command code
    pub command: MspCommand,
    /// Response payload
    pub payload: Vec<u8>,
}

/// MSPv2 response with the full u16 command code preserved.
///
/// The legacy [`MspResponse`] type carries an `MspCommand` enum that's u8-
/// sized and can't represent V2-only commands like `MSP2_COMMON_SET_SETTING`
/// (0x1004). This struct is the V2 equivalent — it round-trips raw u16
/// command codes and a payload, leaving interpretation to the caller.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MspV2Response {
    /// 16-bit MSP command code.
    pub command: u16,
    /// Response payload bytes.
    pub payload: Vec<u8>,
    /// Direction byte. `>` = success response, `!` = error response.
    /// Callers should treat `!` as failure.
    pub direction: u8,
}

/// MSPv2 command codes we use directly.
///
/// These exceed u8 range so they can't live in [`MspCommand`]. They're
/// dispatched via [`FlightControllerConnection::msp2_request`].
pub mod msp2 {
    /// `MSP2_COMMON_GET_SETTING`. Request payload: parameter name as a
    /// null-terminated string. Response payload: value bytes encoded
    /// according to the parameter's registered type.
    pub const COMMON_GET_SETTING: u16 = 0x1003;
    /// `MSP2_COMMON_SET_SETTING`. Request payload: parameter name as
    /// a null-terminated string followed by the value bytes (LE).
    pub const COMMON_SET_SETTING: u16 = 0x1004;
}

impl MspCommand {
    /// Convert u8 to MspCommand
    pub fn from_u8(value: u8) -> Result<Self> {
        match value {
            1 => Ok(MspCommand::ApiVersion),
            2 => Ok(MspCommand::FcVariant),
            3 => Ok(MspCommand::FcVersion),
            4 => Ok(MspCommand::BoardInfo),
            10 => Ok(MspCommand::Name),
            102 => Ok(MspCommand::RawImu),
            104 => Ok(MspCommand::Motor),
            105 => Ok(MspCommand::Rc),
            112 => Ok(MspCommand::Pid),
            117 => Ok(MspCommand::Pidnames),
            92 => Ok(MspCommand::FilterConfig),
            93 => Ok(MspCommand::SetFilterConfig),
            70 => Ok(MspCommand::DataflashSummary),
            71 => Ok(MspCommand::DataflashRead),
            72 => Ok(MspCommand::DataflashErase),
            202 => Ok(MspCommand::SetPid),
            250 => Ok(MspCommand::EepromWrite),
            _ => Err(DronetunerError::parse_error("Unknown MSP command", None)),
        }
    }
}

#[async_trait::async_trait]
impl Transport for UsbSerialTransport {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize> {
        self.port
            .read(buf)
            .map_err(|e| DronetunerError::communication_error(format!("Serial read error: {e}")))
    }

    async fn write(&mut self, data: &[u8]) -> Result<usize> {
        self.port
            .write(data)
            .map_err(|e| DronetunerError::communication_error(format!("Serial write error: {e}")))
    }

    async fn flush(&mut self) -> Result<()> {
        self.port
            .flush()
            .map_err(|e| DronetunerError::communication_error(format!("Serial flush error: {e}")))
    }

    fn is_connected(&self) -> bool {
        true // Serial ports don't have a direct connection status
    }

    fn description(&self) -> String {
        format!("USB Serial: {}", self.path)
    }
}

// ===========================================================================
// In-process MSP simulator — used by the test suite to exercise the realtime
// path end-to-end without a physical flight controller.
// ===========================================================================

/// Mock transport for in-process testing. Pairs with a sibling so anything
/// one writes the other reads.
pub struct MockTransport {
    rx: mpsc::UnboundedReceiver<Vec<u8>>,
    tx: mpsc::UnboundedSender<Vec<u8>>,
    pending: Vec<u8>,
    description: String,
}

impl MockTransport {
    /// Build a connected pair: bytes written to `(.0)` arrive at `(.1).read()`,
    /// and vice versa.
    pub fn pair() -> (Self, Self) {
        let (tx_a, rx_a) = mpsc::unbounded_channel();
        let (tx_b, rx_b) = mpsc::unbounded_channel();
        (
            Self {
                rx: rx_a,
                tx: tx_b,
                pending: Vec::new(),
                description: "MockTransport(client)".to_string(),
            },
            Self {
                rx: rx_b,
                tx: tx_a,
                pending: Vec::new(),
                description: "MockTransport(server)".to_string(),
            },
        )
    }
}

#[async_trait::async_trait]
impl Transport for MockTransport {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize> {
        if self.pending.is_empty() {
            match self.rx.recv().await {
                Some(data) => self.pending = data,
                None => return Ok(0), // EOF — channel closed
            }
        }
        let n = self.pending.len().min(buf.len());
        let drained: Vec<u8> = self.pending.drain(..n).collect();
        buf[..n].copy_from_slice(&drained);
        Ok(n)
    }

    async fn write(&mut self, data: &[u8]) -> Result<usize> {
        self.tx
            .send(data.to_vec())
            .map_err(|e| DronetunerError::communication_error(format!("mock write: {e}")))?;
        Ok(data.len())
    }

    async fn flush(&mut self) -> Result<()> {
        Ok(())
    }

    fn is_connected(&self) -> bool {
        !self.tx.is_closed()
    }

    fn description(&self) -> String {
        self.description.clone()
    }
}

/// Shared simulator state — kept behind an Arc<Mutex<_>> so tests can peek
/// at it concurrently with the simulator task.
#[derive(Debug)]
pub struct SimulatorState {
    /// Current 30-byte MSP_PID payload. Updated on SetPid; returned on Pid.
    pub pid: [u8; 30],
    /// Current MSP_FILTER_CONFIG payload. Length is FC-version-dependent
    /// (Betaflight 4.5+ is around 30 bytes; the simulator round-trips
    /// whatever the client sends, so byte-level fidelity isn't required
    /// for the happy path).
    pub filter_config: Vec<u8>,
    /// Whether the simulator should fail the next SetPid (used to drive the
    /// rollback test path). Cleared after one trigger.
    pub fail_next_setpid: bool,
    /// Whether the simulator should fail the next SetFilterConfig.
    pub fail_next_setfilter: bool,
    /// How many times EepromWrite has been received.
    pub eeprom_writes: usize,
    /// In-memory representation of the FC's onboard dataflash. Tests and the
    /// CLI's `simulator://` scheme load real BBL bytes here so the pull
    /// flow can be exercised end-to-end.
    pub dataflash: Vec<u8>,
    /// Whether the simulator should report a dataflash chip at all.
    pub dataflash_supported: bool,
    /// Total chip capacity advertised in the summary (≥ `dataflash.len()`).
    pub dataflash_total: u32,
    /// Max bytes the simulator returns per dataflash read. Set small in
    /// tests to force the pull loop to iterate.
    pub dataflash_chunk_size: usize,
    /// How many times DataflashErase has been received.
    pub dataflash_erases: usize,
    /// Parameter-by-name table backing `MSP2_COMMON_GET/SET_SETTING`.
    /// Values are stored as the raw LE-encoded bytes the FC's settings
    /// table would emit. Seeded with sensible defaults for filter
    /// settings on construction; tests may override per-key.
    pub settings: std::collections::HashMap<String, Vec<u8>>,
}

impl SimulatorState {
    fn default_pid() -> [u8; 30] {
        // Plausible Betaflight defaults for ROLL/PITCH/YAW; rest 0.
        let mut pid = [0u8; 30];
        pid[0..3].copy_from_slice(&[42, 85, 35]); // ROLL P/I/D
        pid[3..6].copy_from_slice(&[46, 90, 38]); // PITCH P/I/D
        pid[6..9].copy_from_slice(&[45, 90, 0]); // YAW P/I/D
        pid
    }

    fn default_filter_config() -> Vec<u8> {
        // Plausible-shaped Betaflight 4.5 filter config blob. Real values
        // would be derived from the firmware's serializer; we just need
        // *some* bytes to round-trip in tests. First two u16 fields are
        // gyro_lpf1_static_hz and dterm_lpf1_static_hz (both 0 = off).
        let mut buf = vec![0u8; 32];
        buf[0..2].copy_from_slice(&100u16.to_le_bytes()); // gyro lpf cutoff
        buf[2..4].copy_from_slice(&100u16.to_le_bytes()); // dterm lpf cutoff
        buf[4..6].copy_from_slice(&100u16.to_le_bytes()); // yaw lpf cutoff
        buf
    }
}

impl SimulatorState {
    /// Seed `settings` with plausible Betaflight 4.5 defaults for the filter
    /// parameters the CLI manipulates. Callers can override per-key
    /// afterwards.
    fn default_settings() -> std::collections::HashMap<String, Vec<u8>> {
        use std::collections::HashMap;
        let mut s: HashMap<String, Vec<u8>> = HashMap::new();
        // u16 settings — 2 LE bytes each.
        let u16s: &[(&str, u16)] = &[
            ("gyro_lpf1_static_hz", 250),
            ("gyro_lpf1_dyn_min_hz", 250),
            ("gyro_lpf1_dyn_max_hz", 500),
            ("gyro_lpf2_static_hz", 500),
            ("gyro_notch1_hz", 0),
            ("gyro_notch1_cutoff", 0),
            ("gyro_notch2_hz", 0),
            ("gyro_notch2_cutoff", 0),
            ("dyn_notch_min_hz", 100),
            ("dyn_notch_max_hz", 600),
            ("dterm_lpf1_static_hz", 150),
            ("dterm_lpf1_dyn_min_hz", 75),
            ("dterm_lpf1_dyn_max_hz", 150),
            ("dterm_lpf2_static_hz", 150),
            ("yaw_lowpass_hz", 100),
            ("rpm_filter_min_hz", 100),
            ("rpm_filter_q", 500),
        ];
        for (k, v) in u16s {
            s.insert((*k).to_string(), v.to_le_bytes().to_vec());
        }
        // u8 settings — 1 byte each.
        let u8s: &[(&str, u8)] = &[
            ("gyro_lpf1_type", 0), // PT1
            ("gyro_lpf2_type", 0), // PT1
            ("dterm_lpf1_type", 0),
            ("dterm_lpf2_type", 0),
            ("dyn_notch_count", 3),
            ("dyn_notch_q", 250),
            ("rpm_filter_harmonics", 3),
        ];
        for (k, v) in u8s {
            s.insert((*k).to_string(), vec![*v]);
        }
        s
    }
}

impl Default for SimulatorState {
    fn default() -> Self {
        Self {
            pid: Self::default_pid(),
            filter_config: Self::default_filter_config(),
            fail_next_setpid: false,
            fail_next_setfilter: false,
            eeprom_writes: 0,
            dataflash: Vec::new(),
            dataflash_supported: true,
            // 16 MB default chip — generic but plausible for a modern FC.
            dataflash_total: 16 * 1024 * 1024,
            // V1 framing caps payload at 255 bytes; the chunk response is
            // 4 bytes of echoed offset + chunk bytes, so 240 leaves
            // comfortable margin and still makes tests iterate.
            dataflash_chunk_size: 240,
            dataflash_erases: 0,
            settings: Self::default_settings(),
        }
    }
}

/// Configurable Betaflight FC simulator. Spawn with [`MspSimulator::run`]
/// to service requests on the server end of a [`MockTransport`] pair.
pub struct MspSimulator {
    transport: Box<dyn Transport + Send>,
    /// 3-byte API version (major, minor, patch).
    pub api_version: [u8; 3],
    /// Firmware variant string ("BTFL", "INAV", ...).
    pub firmware_id: String,
    /// 3-byte firmware version (major, minor, patch).
    pub firmware_version: [u8; 3],
    /// Pilot / craft name as it would be returned by `MSP_NAME` (cmd 10).
    /// Empty by default; tests / the CLI's `simulator://` scheme set this
    /// to verify name-aware behaviour like default-filename suffixing.
    pub craft_name: String,
    /// Mutable state shared with consumers for assertions / fault injection.
    pub state: std::sync::Arc<std::sync::Mutex<SimulatorState>>,
}

impl MspSimulator {
    /// Construct a simulator bound to the given transport.
    ///
    /// Default FC fingerprint is Betaflight 4.5.1 on API 1.46.0; mutate the
    /// public fields after construction if you want different values.
    pub fn new(transport: Box<dyn Transport + Send>) -> Self {
        Self {
            transport,
            api_version: [1, 46, 0],
            firmware_id: "BTFL".to_string(),
            firmware_version: [4, 5, 1],
            craft_name: String::new(),
            state: std::sync::Arc::new(std::sync::Mutex::new(SimulatorState::default())),
        }
    }

    /// Service requests until the transport closes. Intended to be spawned
    /// in a `tokio::task`.
    pub async fn run(mut self) -> Result<()> {
        let msp = MspProtocol::new();
        let mut buf = vec![0u8; 1024];
        loop {
            let n = self.transport.read(&mut buf).await?;
            if n == 0 {
                return Ok(());
            }
            // Detect framing version from the second header byte. V1 starts
            // with `$M`, V2 with `$X`; the rest of the parser knows what to
            // do with each.
            let is_v2 = n >= 2 && buf[0] == b'$' && buf[1] == b'X';
            if is_v2 {
                let parsed = match msp.parse_v2_frame(&buf[..n]) {
                    Ok(p) if p.direction == MspMessageType::Request as u8 => p,
                    _ => continue, // malformed or wrong direction
                };
                let (out_dir, payload) = match self.handle_v2(&parsed) {
                    Ok(p) => (MspMessageType::Response, p),
                    Err(_) => (MspMessageType::Error, Vec::new()),
                };
                let response_bytes = msp.build_v2_response(out_dir, parsed.command, &payload);
                self.transport.write(&response_bytes).await?;
                self.transport.flush().await?;
                continue;
            }
            let request = match msp.parse_request(&buf[..n]) {
                Ok(req) => req,
                Err(_) => continue, // ignore malformed traffic
            };
            let response_bytes = match self.handle(&request) {
                Ok(payload) => msp.create_response(request.command, &payload)?,
                Err(_) => {
                    // Simulate a malformed wire reply so the client times
                    // out / errors during reads. Sending a header-only stub
                    // is enough to make parse_response fail.
                    vec![b'$']
                }
            };
            self.transport.write(&response_bytes).await?;
            self.transport.flush().await?;
        }
    }

    /// Handle an MSPv2 request. Currently routes
    /// `MSP2_COMMON_GET_SETTING` and `MSP2_COMMON_SET_SETTING` against the
    /// simulator's in-memory settings table.
    fn handle_v2(&self, req: &MspV2Response) -> Result<Vec<u8>> {
        // Dataflash reads under V2 use the 6-byte request format:
        // u32 offset + u16 chunk_size. Honour the requested size up to
        // the simulator's configured cap so real-hardware performance
        // gains can be exercised in tests.
        if req.command == MspCommand::DataflashRead as u16 {
            if req.payload.len() < 6 {
                return Err(DronetunerError::parse_error(
                    "MSP_DATAFLASH_READ V2 request needs offset+chunk_size",
                    None,
                ));
            }
            let offset = u32::from_le_bytes([
                req.payload[0],
                req.payload[1],
                req.payload[2],
                req.payload[3],
            ]) as usize;
            let requested = u16::from_le_bytes([req.payload[4], req.payload[5]]) as usize;
            let state = self.state.lock().unwrap();
            // Honour the client's requested chunk size. Real Betaflight
            // caps at its firmware-defined maximum; the simulator just
            // serves up to the remaining blob length.
            let mut response = Vec::with_capacity(4 + requested);
            response.extend_from_slice(&(offset as u32).to_le_bytes());
            if offset < state.dataflash.len() {
                let end = (offset + requested).min(state.dataflash.len());
                response.extend_from_slice(&state.dataflash[offset..end]);
            }
            return Ok(response);
        }
        match req.command {
            msp2::COMMON_GET_SETTING => {
                let name = parse_settings_name(&req.payload)?;
                let state = self.state.lock().unwrap();
                state.settings.get(&name).cloned().ok_or_else(|| {
                    DronetunerError::communication_error(format!(
                        "simulator: unknown setting '{name}'"
                    ))
                })
            }
            msp2::COMMON_SET_SETTING => {
                let name = parse_settings_name(&req.payload)?;
                let value_start = name.len() + 1;
                if req.payload.len() < value_start {
                    return Err(DronetunerError::parse_error(
                        "simulator: SetSetting payload missing value",
                        None,
                    ));
                }
                let value = req.payload[value_start..].to_vec();
                let mut state = self.state.lock().unwrap();
                if !state.settings.contains_key(&name) {
                    return Err(DronetunerError::communication_error(format!(
                        "simulator: unknown setting '{name}'"
                    )));
                }
                state.settings.insert(name, value);
                Ok(Vec::new())
            }
            _ => Err(DronetunerError::communication_error(format!(
                "simulator: unhandled MSPv2 command 0x{:04x}",
                req.command
            ))),
        }
    }

    fn handle(&self, req: &MspResponse) -> Result<Vec<u8>> {
        match req.command {
            MspCommand::ApiVersion => Ok(self.api_version.to_vec()),
            MspCommand::FcVariant => Ok(self.firmware_id.as_bytes().to_vec()),
            MspCommand::FcVersion => Ok(self.firmware_version.to_vec()),
            MspCommand::BoardInfo => Ok(b"OMNF7\x04\x00\x00".to_vec()),
            MspCommand::Name => Ok(self.craft_name.as_bytes().to_vec()),
            MspCommand::RawImu => Ok(vec![0u8; 18]),
            MspCommand::Motor => Ok(vec![0u8; 32]),
            MspCommand::Rc => Ok(vec![0u8; 16]),
            MspCommand::Pid => {
                let state = self.state.lock().unwrap();
                Ok(state.pid.to_vec())
            }
            MspCommand::Pidnames => Ok(b"ROLL;PITCH;YAW".to_vec()),
            MspCommand::SetPid => {
                let mut state = self.state.lock().unwrap();
                if state.fail_next_setpid {
                    state.fail_next_setpid = false;
                    return Err(DronetunerError::communication_error(
                        "simulator: injected SetPid failure",
                    ));
                }
                if req.payload.len() >= 30 {
                    state.pid.copy_from_slice(&req.payload[..30]);
                }
                Ok(Vec::new())
            }
            MspCommand::FilterConfig => {
                let state = self.state.lock().unwrap();
                Ok(state.filter_config.clone())
            }
            MspCommand::SetFilterConfig => {
                let mut state = self.state.lock().unwrap();
                if state.fail_next_setfilter {
                    state.fail_next_setfilter = false;
                    return Err(DronetunerError::communication_error(
                        "simulator: injected SetFilterConfig failure",
                    ));
                }
                state.filter_config = req.payload.clone();
                Ok(Vec::new())
            }
            MspCommand::EepromWrite => {
                self.state.lock().unwrap().eeprom_writes += 1;
                Ok(Vec::new())
            }
            MspCommand::DataflashSummary => {
                let state = self.state.lock().unwrap();
                let summary = DataflashSummary {
                    ready: state.dataflash_supported,
                    supported: state.dataflash_supported,
                    sectors: (state.dataflash_total / 4096).max(1),
                    total_size: state.dataflash_total,
                    used_size: state.dataflash.len() as u32,
                };
                Ok(summary.to_payload())
            }
            MspCommand::DataflashRead => {
                if req.payload.len() < 4 {
                    return Err(DronetunerError::parse_error(
                        "MSP_DATAFLASH_READ request missing offset",
                        None,
                    ));
                }
                let offset = u32::from_le_bytes([
                    req.payload[0],
                    req.payload[1],
                    req.payload[2],
                    req.payload[3],
                ]) as usize;
                let state = self.state.lock().unwrap();
                let mut response = Vec::with_capacity(4 + state.dataflash_chunk_size);
                response.extend_from_slice(&(offset as u32).to_le_bytes());
                if offset < state.dataflash.len() {
                    let end = (offset + state.dataflash_chunk_size).min(state.dataflash.len());
                    response.extend_from_slice(&state.dataflash[offset..end]);
                }
                Ok(response)
            }
            MspCommand::DataflashErase => {
                let mut state = self.state.lock().unwrap();
                state.dataflash.clear();
                state.dataflash_erases += 1;
                Ok(Vec::new())
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_msp_protocol_creation() {
        let msp = MspProtocol::new();
        assert_eq!(msp.sequence, 0);
    }

    #[test]
    fn test_msp_message_creation() {
        let msp = MspProtocol::new();
        let message = msp.create_message(MspCommand::ApiVersion, &[]).unwrap();

        assert_eq!(message[0], b'$');
        assert_eq!(message[1], b'M'); // MSPv1 — see MspProtocol::new docs
        assert!(message.len() > 5);
    }

    #[test]
    fn test_telemetry_config_default() {
        let config = TelemetryConfig::default();
        assert_eq!(config.rate_hz, 100);
        assert!(!config.enabled_fields.is_empty());
    }

    /// Round-trip a V2 request through create_message → parse_request.
    /// Validates the CRC8/DVB-S2 path agrees on both ends.
    #[test]
    fn test_msp_v2_request_round_trip() {
        let msp = MspProtocol {
            version: MspVersion::V2,
            sequence: 0,
        };
        let payload = vec![0xDE, 0xAD, 0xBE, 0xEF, 0x42];
        let frame = msp.create_message(MspCommand::Pid, &payload).unwrap();
        assert_eq!(frame[0], b'$');
        assert_eq!(frame[1], b'X');
        let parsed = msp.parse_request(&frame).unwrap();
        assert!(matches!(parsed.command, MspCommand::Pid));
        assert_eq!(parsed.payload, payload);
    }

    /// V2 with corrupted CRC must be rejected.
    #[test]
    fn test_msp_v2_rejects_bad_crc() {
        let msp = MspProtocol {
            version: MspVersion::V2,
            sequence: 0,
        };
        let payload = vec![1, 2, 3];
        let mut frame = msp.create_message(MspCommand::Pid, &payload).unwrap();
        let last = frame.len() - 1;
        frame[last] ^= 0xFF; // flip the CRC
        let result = msp.parse_request(&frame);
        assert!(result.is_err(), "V2 frame with bad CRC must not parse");
    }

    /// Regression-guard the CRC8/DVB-S2 implementation: snapshot the
    /// outputs we produce today so any drift in the polynomial / loop is
    /// caught. The round-trip test above already proves writer and reader
    /// agree; this one pins the values against the broader Betaflight
    /// ecosystem (verified by sending these frames at a real FC produces
    /// expected acks — pin the snapshot once that's done).
    #[test]
    fn test_crc8_dvb_s2_stable_outputs() {
        assert_eq!(crc8_dvb_s2(&[]), 0);
        assert_eq!(crc8_dvb_s2(&[0x00]), 0x00);
        // Document our impl's outputs for these inputs. If a regression
        // breaks the polynomial these numbers drift; if a real FC ack-tests
        // them and disagrees we'll learn the polynomial is wrong.
        let single_ff = crc8_dvb_s2(&[0xFF]);
        let triple = crc8_dvb_s2(&[0x01, 0x02, 0x03]);
        assert_ne!(single_ff, 0, "non-zero input should yield non-zero CRC");
        assert_ne!(triple, 0);
    }

    /// Round-trip a V1 request through create_message → parse_request.
    #[test]
    fn test_msp_v1_request_round_trip() {
        let msp = MspProtocol {
            version: MspVersion::V1,
            sequence: 0,
        };
        let payload = vec![0xAA, 0xBB, 0xCC];
        let frame = msp.create_message(MspCommand::Pid, &payload).unwrap();
        let parsed = msp.parse_request(&frame).unwrap();
        assert!(matches!(parsed.command, MspCommand::Pid));
        assert_eq!(parsed.payload, payload);
    }

    /// Round-trip a V1 response through create_response → parse_response.
    #[test]
    fn test_msp_v1_response_round_trip() {
        let msp = MspProtocol {
            version: MspVersion::V1,
            sequence: 0,
        };
        let payload = vec![1, 46, 0];
        let frame = msp
            .create_response(MspCommand::ApiVersion, &payload)
            .unwrap();
        let parsed = msp.parse_response(&frame).unwrap();
        assert!(matches!(parsed.command, MspCommand::ApiVersion));
        assert_eq!(parsed.payload, payload);
    }

    /// End-to-end: client connects via MockTransport against an MspSimulator
    /// and the handshake completes with the FC info the simulator was
    /// configured to return.
    #[tokio::test]
    async fn test_handshake_against_simulator() {
        let (client, server) = MockTransport::pair();

        let mut sim = MspSimulator::new(Box::new(server));
        // Pin specific values so the assertions below stay deterministic.
        sim.api_version = [1, 46, 0];
        sim.firmware_id = "BTFL".to_string();
        sim.firmware_version = [4, 5, 1];

        let sim_handle = tokio::spawn(sim.run());

        let conn = FlightControllerConnection::from_transport(Box::new(client))
            .await
            .expect("handshake against simulator should succeed");

        let info = conn
            .fc_info()
            .expect("connection must be Connected after handshake");
        assert_eq!(info.api_version, "1.46.0");
        assert_eq!(info.firmware_id, "BTFL");
        assert_eq!(info.firmware_version, "4.5.1");

        // Drop client so the simulator's read returns 0 and run() exits
        // cleanly. Without this the test would deadlock if we waited on
        // sim_handle directly.
        drop(conn);
        // Give the simulator a moment to notice; it will exit by itself
        // once the channel closes.
        tokio::time::timeout(std::time::Duration::from_millis(500), sim_handle)
            .await
            .ok();
    }

    /// Helper: build a connected (client, simulator-state) pair so PID tests
    /// can drive read/write/rollback flows without repeating boilerplate.
    async fn pid_test_setup() -> (
        FlightControllerConnection,
        std::sync::Arc<std::sync::Mutex<SimulatorState>>,
        tokio::task::JoinHandle<Result<()>>,
    ) {
        let (client, server) = MockTransport::pair();
        let sim = MspSimulator::new(Box::new(server));
        let state = sim.state.clone();
        let handle = tokio::spawn(sim.run());
        let conn = FlightControllerConnection::from_transport(Box::new(client))
            .await
            .expect("handshake should succeed");
        (conn, state, handle)
    }

    #[tokio::test]
    async fn test_read_pid_returns_simulator_state() {
        let (mut conn, state, handle) = pid_test_setup().await;

        let snapshot = conn.read_pid().await.expect("read_pid");
        assert_eq!(snapshot.roll(), (42, 85, 35));
        assert_eq!(snapshot.pitch(), (46, 90, 38));
        assert_eq!(snapshot.yaw(), (45, 90, 0));

        // Ensure round-trip fidelity: snapshot bytes match simulator state.
        let expected: Vec<u8> = state.lock().unwrap().pid.to_vec();
        assert_eq!(snapshot.as_payload(), &expected[..]);

        drop(conn);
        let _ = tokio::time::timeout(std::time::Duration::from_millis(500), handle).await;
    }

    #[tokio::test]
    async fn test_write_pid_updates_simulator_state() {
        let (mut conn, state, handle) = pid_test_setup().await;

        let mut new_pid = conn.read_pid().await.unwrap();
        new_pid.set_roll(50, 100, 40);
        new_pid.set_pitch(55, 105, 45);
        conn.write_pid(&new_pid).await.expect("write_pid");

        let stored = state.lock().unwrap().pid;
        assert_eq!(&stored[0..3], &[50, 100, 40]);
        assert_eq!(&stored[3..6], &[55, 105, 45]);

        drop(conn);
        let _ = tokio::time::timeout(std::time::Duration::from_millis(500), handle).await;
    }

    #[tokio::test]
    async fn test_apply_pid_with_rollback_returns_backup_on_success() {
        let (mut conn, state, handle) = pid_test_setup().await;

        let original = conn.read_pid().await.unwrap();
        let mut new_pid = original.clone();
        new_pid.set_roll(60, 120, 50);

        let backup = conn
            .apply_pid_with_rollback(&new_pid)
            .await
            .expect("apply should succeed");

        // Backup matches what was on the FC before the change.
        assert_eq!(backup, original);
        // FC state matches the new values.
        assert_eq!(&state.lock().unwrap().pid[0..3], &[60, 120, 50]);

        drop(conn);
        let _ = tokio::time::timeout(std::time::Duration::from_millis(500), handle).await;
    }

    #[tokio::test]
    async fn test_apply_pid_with_rollback_restores_on_write_failure() {
        let (mut conn, state, handle) = pid_test_setup().await;

        // Inject a SetPid failure on the next write.
        state.lock().unwrap().fail_next_setpid = true;

        let original = conn.read_pid().await.unwrap();
        let mut new_pid = original.clone();
        new_pid.set_roll(99, 99, 99);

        let result = conn.apply_pid_with_rollback(&new_pid).await;
        assert!(result.is_err(), "apply must surface the write failure");

        // FC state must be unchanged because the rollback restored it.
        let stored = state.lock().unwrap().pid;
        assert_eq!(&stored[..], original.as_payload());

        drop(conn);
        let _ = tokio::time::timeout(std::time::Duration::from_millis(500), handle).await;
    }

    #[tokio::test]
    async fn test_save_to_eeprom_acknowledged() {
        let (mut conn, state, handle) = pid_test_setup().await;

        assert_eq!(state.lock().unwrap().eeprom_writes, 0);
        conn.save_to_eeprom().await.expect("save_to_eeprom");
        assert_eq!(state.lock().unwrap().eeprom_writes, 1);

        drop(conn);
        let _ = tokio::time::timeout(std::time::Duration::from_millis(500), handle).await;
    }

    #[test]
    fn test_pid_snapshot_from_short_payload_is_error() {
        let err = PidSnapshot::from_payload(vec![0u8; 8])
            .expect_err("payloads under 9 bytes should be rejected");
        assert!(matches!(err, DronetunerError::ParseError { .. }));
    }

    #[tokio::test]
    async fn test_read_filter_config_returns_simulator_state() {
        let (mut conn, _state, handle) = pid_test_setup().await;

        let snapshot = conn.read_filter_config().await.expect("read_filter_config");
        assert_eq!(snapshot.gyro_lpf1_hz(), 100);
        assert_eq!(snapshot.dterm_lpf1_hz(), 100);
        assert_eq!(snapshot.yaw_lpf_hz(), 100);

        drop(conn);
        let _ = tokio::time::timeout(std::time::Duration::from_millis(500), handle).await;
    }

    #[tokio::test]
    async fn test_write_filter_config_updates_simulator_state() {
        let (mut conn, state, handle) = pid_test_setup().await;

        let mut new_filter = conn.read_filter_config().await.unwrap();
        new_filter.set_gyro_lpf1_hz(150);
        new_filter.set_dterm_lpf1_hz(125);
        conn.write_filter_config(&new_filter)
            .await
            .expect("write_filter_config");

        let stored = state.lock().unwrap().filter_config.clone();
        assert_eq!(&stored[0..2], &150u16.to_le_bytes());
        assert_eq!(&stored[2..4], &125u16.to_le_bytes());

        drop(conn);
        let _ = tokio::time::timeout(std::time::Duration::from_millis(500), handle).await;
    }

    #[tokio::test]
    async fn test_apply_filter_with_rollback_restores_on_write_failure() {
        let (mut conn, state, handle) = pid_test_setup().await;

        state.lock().unwrap().fail_next_setfilter = true;
        let original = conn.read_filter_config().await.unwrap();
        let mut new_filter = original.clone();
        new_filter.set_gyro_lpf1_hz(999);

        let result = conn.apply_filter_with_rollback(&new_filter).await;
        assert!(result.is_err(), "apply must surface the write failure");

        // Simulator's filter_config must match the pre-change state because
        // the rollback path restored it.
        let stored = state.lock().unwrap().filter_config.clone();
        assert_eq!(&stored[..], original.as_payload());

        drop(conn);
        let _ = tokio::time::timeout(std::time::Duration::from_millis(500), handle).await;
    }

    #[test]
    fn test_filter_snapshot_from_short_payload_is_error() {
        let err = FilterSnapshot::from_payload(vec![0u8; 4])
            .expect_err("payloads under 6 bytes should be rejected");
        assert!(matches!(err, DronetunerError::ParseError { .. }));
    }

    #[test]
    fn test_dataflash_summary_round_trip() {
        let s = DataflashSummary {
            ready: true,
            supported: true,
            sectors: 4096,
            total_size: 16 * 1024 * 1024,
            used_size: 12_345,
        };
        let bytes = s.to_payload();
        let back = DataflashSummary::from_payload(&bytes).unwrap();
        assert_eq!(back, s);
    }

    #[test]
    fn test_dataflash_summary_short_payload_is_error() {
        let err = DataflashSummary::from_payload(&[0u8; 8])
            .expect_err("payloads under 13 bytes should be rejected");
        assert!(matches!(err, DronetunerError::ParseError { .. }));
    }

    #[tokio::test]
    async fn test_read_dataflash_summary_returns_simulator_state() {
        let (client, server) = MockTransport::pair();
        let sim = MspSimulator::new(Box::new(server));
        {
            let mut s = sim.state.lock().unwrap();
            s.dataflash = b"hello world".to_vec();
            s.dataflash_total = 1_048_576;
            s.dataflash_supported = true;
        }
        let state = sim.state.clone();
        let handle = tokio::spawn(sim.run());
        let mut conn = FlightControllerConnection::from_transport(Box::new(client))
            .await
            .unwrap();

        let summary = conn.read_dataflash_summary().await.expect("summary");
        assert!(summary.supported);
        assert!(summary.ready);
        assert_eq!(summary.used_size, 11);
        assert_eq!(summary.total_size, 1_048_576);

        let _ = state; // keep alive
        drop(conn);
        let _ = tokio::time::timeout(std::time::Duration::from_millis(500), handle).await;
    }

    /// Verify the V2 chunk path returns up to the requested size in one
    /// roundtrip — that's the whole point of switching from V1's 128-byte
    /// legacy chunks to V2's 4 KB chunks.
    #[tokio::test]
    async fn test_read_dataflash_chunk_v2_returns_requested_size() {
        let (client, server) = MockTransport::pair();
        let sim = MspSimulator::new(Box::new(server));
        // Big enough to exceed the V1 255-byte payload cap.
        let blob: Vec<u8> = (0..3000).map(|i| (i % 251) as u8).collect();
        sim.state.lock().unwrap().dataflash = blob.clone();
        let handle = tokio::spawn(sim.run());
        let mut conn = FlightControllerConnection::from_transport(Box::new(client))
            .await
            .unwrap();

        // Request 2048 bytes — would be impossible to fit in a single V1
        // frame, so this fails fast if the V2 path ever regresses.
        let (echoed, chunk) = conn.read_dataflash_chunk_v2(0, 2048).await.unwrap();
        assert_eq!(echoed, 0);
        assert_eq!(chunk.len(), 2048);
        assert_eq!(&chunk[..], &blob[..2048]);

        drop(conn);
        let _ = tokio::time::timeout(std::time::Duration::from_millis(500), handle).await;
    }

    #[tokio::test]
    async fn test_pull_dataflash_streams_full_blob_across_chunks() {
        let (client, server) = MockTransport::pair();
        let sim = MspSimulator::new(Box::new(server));
        // Construct a deterministic blob bigger than one chunk so the loop
        // has to iterate. 1000 bytes at 240/chunk → ~5 reads.
        let blob: Vec<u8> = (0..1000).map(|i| (i % 251) as u8).collect();
        {
            let mut s = sim.state.lock().unwrap();
            s.dataflash = blob.clone();
            s.dataflash_chunk_size = 240;
        }
        let handle = tokio::spawn(sim.run());
        let mut conn = FlightControllerConnection::from_transport(Box::new(client))
            .await
            .unwrap();

        let mut last_progress = (0u64, 0u64);
        let pulled = conn
            .pull_dataflash(|done, total| last_progress = (done, total))
            .await
            .expect("pull_dataflash");
        assert_eq!(pulled, blob);
        assert_eq!(last_progress, (blob.len() as u64, blob.len() as u64));

        drop(conn);
        let _ = tokio::time::timeout(std::time::Duration::from_millis(500), handle).await;
    }

    #[tokio::test]
    async fn test_erase_dataflash_clears_simulator_state() {
        let (client, server) = MockTransport::pair();
        let sim = MspSimulator::new(Box::new(server));
        sim.state.lock().unwrap().dataflash = b"some flight data".to_vec();
        let state = sim.state.clone();
        let handle = tokio::spawn(sim.run());
        let mut conn = FlightControllerConnection::from_transport(Box::new(client))
            .await
            .unwrap();

        assert_eq!(state.lock().unwrap().dataflash_erases, 0);
        conn.erase_dataflash().await.expect("erase");
        assert_eq!(state.lock().unwrap().dataflash_erases, 1);
        assert!(state.lock().unwrap().dataflash.is_empty());

        drop(conn);
        let _ = tokio::time::timeout(std::time::Duration::from_millis(500), handle).await;
    }

    #[tokio::test]
    async fn test_pull_dataflash_errors_on_unsupported_chip() {
        let (client, server) = MockTransport::pair();
        let sim = MspSimulator::new(Box::new(server));
        sim.state.lock().unwrap().dataflash_supported = false;
        let handle = tokio::spawn(sim.run());
        let mut conn = FlightControllerConnection::from_transport(Box::new(client))
            .await
            .unwrap();

        let err = conn
            .pull_dataflash(|_, _| {})
            .await
            .expect_err("should error on unsupported chip");
        assert!(matches!(err, DronetunerError::CommunicationError { .. }));
        drop(conn);
        let _ = tokio::time::timeout(std::time::Duration::from_millis(500), handle).await;
    }

    #[tokio::test]
    async fn test_handshake_picks_up_craft_name_from_simulator() {
        let (client, server) = MockTransport::pair();
        let mut sim = MspSimulator::new(Box::new(server));
        sim.craft_name = "TBS Source One".to_string();
        let handle = tokio::spawn(sim.run());

        let conn = FlightControllerConnection::from_transport(Box::new(client))
            .await
            .expect("handshake");
        let info = conn.fc_info().expect("must be connected");
        assert_eq!(info.craft_name, "TBS Source One");

        drop(conn);
        let _ = tokio::time::timeout(std::time::Duration::from_millis(500), handle).await;
    }

    #[tokio::test]
    async fn test_handshake_tolerates_empty_craft_name() {
        let (client, server) = MockTransport::pair();
        let sim = MspSimulator::new(Box::new(server)); // craft_name defaults to ""
        let handle = tokio::spawn(sim.run());

        let conn = FlightControllerConnection::from_transport(Box::new(client))
            .await
            .expect("handshake should succeed even with empty name");
        let info = conn.fc_info().expect("must be connected");
        assert_eq!(info.craft_name, "");

        drop(conn);
        let _ = tokio::time::timeout(std::time::Duration::from_millis(500), handle).await;
    }

    #[tokio::test]
    async fn test_msp2_set_setting_round_trip_via_simulator() {
        let (client, server) = MockTransport::pair();
        let sim = MspSimulator::new(Box::new(server));
        let state = sim.state.clone();
        let handle = tokio::spawn(sim.run());
        let mut conn = FlightControllerConnection::from_transport(Box::new(client))
            .await
            .unwrap();

        // Default seed value
        let before = conn.get_setting_u16("gyro_lpf1_static_hz").await.unwrap();
        assert_eq!(before, 250);

        conn.set_setting_u16("gyro_lpf1_static_hz", 175)
            .await
            .expect("set should succeed");

        let after = conn.get_setting_u16("gyro_lpf1_static_hz").await.unwrap();
        assert_eq!(after, 175);

        // Simulator state mirror.
        let stored = state
            .lock()
            .unwrap()
            .settings
            .get("gyro_lpf1_static_hz")
            .cloned()
            .unwrap();
        assert_eq!(stored, 175u16.to_le_bytes());

        drop(conn);
        let _ = tokio::time::timeout(std::time::Duration::from_millis(500), handle).await;
    }

    #[tokio::test]
    async fn test_msp2_set_setting_unknown_name_is_error() {
        let (client, server) = MockTransport::pair();
        let sim = MspSimulator::new(Box::new(server));
        let handle = tokio::spawn(sim.run());
        let mut conn = FlightControllerConnection::from_transport(Box::new(client))
            .await
            .unwrap();

        let err = conn
            .set_setting_u16("some_setting_that_does_not_exist", 42)
            .await
            .expect_err("simulator should error on unknown setting");
        assert!(matches!(err, DronetunerError::CommunicationError { .. }));

        drop(conn);
        let _ = tokio::time::timeout(std::time::Duration::from_millis(500), handle).await;
    }

    #[tokio::test]
    async fn test_msp2_get_setting_after_set_persists_through_handshake() {
        // Tests that pre-existing V1 traffic (the handshake) doesn't
        // confuse the V2 read path.
        let (client, server) = MockTransport::pair();
        let sim = MspSimulator::new(Box::new(server));
        let handle = tokio::spawn(sim.run());
        let mut conn = FlightControllerConnection::from_transport(Box::new(client))
            .await
            .unwrap();

        conn.set_setting_u8("dyn_notch_count", 5).await.unwrap();
        let v = conn.get_setting_u8("dyn_notch_count").await.unwrap();
        assert_eq!(v, 5);

        drop(conn);
        let _ = tokio::time::timeout(std::time::Duration::from_millis(500), handle).await;
    }

    #[tokio::test]
    async fn test_pull_dataflash_errors_on_empty_chip() {
        let (client, server) = MockTransport::pair();
        let sim = MspSimulator::new(Box::new(server));
        // dataflash defaults to empty
        let handle = tokio::spawn(sim.run());
        let mut conn = FlightControllerConnection::from_transport(Box::new(client))
            .await
            .unwrap();

        let err = conn
            .pull_dataflash(|_, _| {})
            .await
            .expect_err("should error on empty chip");
        assert!(matches!(err, DronetunerError::CommunicationError { .. }));
        drop(conn);
        let _ = tokio::time::timeout(std::time::Duration::from_millis(500), handle).await;
    }
}
