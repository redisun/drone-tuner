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
    /// Set PID values
    SetPid = 202,
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

        let fc_info = FlightControllerInfo {
            api_version,
            firmware_id,
            firmware_version,
            board_id,
            target_name,
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
        let mut buf = [0u8; 256];
        let size = self.transport.read(&mut buf).await?;

        if size == 0 {
            return Err(DronetunerError::communication_error("No response received"));
        }

        self.msp.parse_response(&buf[..size])
    }

    fn clone_transport(&self) -> Result<Box<dyn Transport + Send>> {
        // This is a simplified implementation - real version would properly clone transport
        Err(DronetunerError::communication_error(
            "Transport cloning not implemented",
        ))
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
    #[cfg(test)]
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
                // MSPv2 format is more complex but provides better reliability
                message.push(b'$');
                message.push(b'X');
                message.push(direction as u8);
                message.push(0); // Flag byte
                message.extend_from_slice(&(command as u16).to_le_bytes());
                message.extend_from_slice(&(payload.len() as u16).to_le_bytes());
                message.extend_from_slice(payload);

                // CRC checksum for MSPv2
                let crc = self.calculate_crc(&message[3..]);
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
    #[cfg(test)]
    pub(crate) fn parse_request(&self, data: &[u8]) -> Result<MspResponse> {
        self.parse_framed(MspMessageType::Request, data)
    }

    fn parse_framed(&self, expected_direction: MspMessageType, data: &[u8]) -> Result<MspResponse> {
        if data.len() < 5 {
            return Err(DronetunerError::parse_error("MSP message too short", None));
        }

        // Check for MSP header
        if data[0] != b'$' || data[1] != b'M' {
            return Err(DronetunerError::parse_error("Invalid MSP header", None));
        }

        let direction = data[2];
        if direction != expected_direction as u8 {
            return Err(DronetunerError::parse_error(
                "Wrong MSP direction byte",
                None,
            ));
        }

        let payload_size = data[3] as usize;
        let command = data[4];

        if data.len() < 6 + payload_size {
            return Err(DronetunerError::parse_error("Incomplete MSP message", None));
        }

        let payload = data[5..5 + payload_size].to_vec();

        Ok(MspResponse {
            command: MspCommand::from_u8(command)?,
            payload,
        })
    }

    /// Calculate CRC for MSPv2
    fn calculate_crc(&self, data: &[u8]) -> u8 {
        let mut crc = 0u8;
        for &byte in data {
            crc ^= byte;
        }
        crc
    }
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

impl MspCommand {
    /// Convert u8 to MspCommand
    pub fn from_u8(value: u8) -> Result<Self> {
        match value {
            1 => Ok(MspCommand::ApiVersion),
            2 => Ok(MspCommand::FcVariant),
            3 => Ok(MspCommand::FcVersion),
            4 => Ok(MspCommand::BoardInfo),
            102 => Ok(MspCommand::RawImu),
            104 => Ok(MspCommand::Motor),
            105 => Ok(MspCommand::Rc),
            112 => Ok(MspCommand::Pid),
            117 => Ok(MspCommand::Pidnames),
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
#[cfg(test)]
pub(crate) struct MockTransport {
    rx: mpsc::UnboundedReceiver<Vec<u8>>,
    tx: mpsc::UnboundedSender<Vec<u8>>,
    pending: Vec<u8>,
    description: String,
}

#[cfg(test)]
impl MockTransport {
    /// Build a connected pair: bytes written to `(.0)` arrive at `(.1).read()`,
    /// and vice versa.
    pub(crate) fn pair() -> (Self, Self) {
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

#[cfg(test)]
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

/// Configurable Betaflight FC simulator. Spawn with [`MspSimulator::run`]
/// to service requests on the server end of a [`MockTransport`] pair.
#[cfg(test)]
pub(crate) struct MspSimulator {
    transport: Box<dyn Transport + Send>,
    /// 3-byte API version (major, minor, patch).
    pub(crate) api_version: [u8; 3],
    /// Firmware variant string ("BTFL", "INAV", ...).
    pub(crate) firmware_id: String,
    /// 3-byte firmware version (major, minor, patch).
    pub(crate) firmware_version: [u8; 3],
}

#[cfg(test)]
impl MspSimulator {
    pub(crate) fn new(transport: Box<dyn Transport + Send>) -> Self {
        Self {
            transport,
            api_version: [1, 46, 0],
            firmware_id: "BTFL".to_string(),
            firmware_version: [4, 5, 1],
        }
    }

    /// Service requests until the transport closes. Intended to be spawned
    /// in a `tokio::task`.
    pub(crate) async fn run(mut self) -> Result<()> {
        let msp = MspProtocol::new();
        let mut buf = vec![0u8; 1024];
        loop {
            let n = self.transport.read(&mut buf).await?;
            if n == 0 {
                return Ok(());
            }
            let request = match msp.parse_request(&buf[..n]) {
                Ok(req) => req,
                Err(_) => continue, // ignore malformed traffic
            };
            let payload = self.handle(&request);
            let response = msp.create_response(request.command, &payload)?;
            self.transport.write(&response).await?;
            self.transport.flush().await?;
        }
    }

    fn handle(&self, req: &MspResponse) -> Vec<u8> {
        match req.command {
            MspCommand::ApiVersion => self.api_version.to_vec(),
            MspCommand::FcVariant => self.firmware_id.as_bytes().to_vec(),
            MspCommand::FcVersion => self.firmware_version.to_vec(),
            MspCommand::BoardInfo => b"OMNF7\x04\x00\x00".to_vec(),
            // RawImu: 18 bytes, 3xi16 gyro + 3xi16 accel + 3xi16 mag
            MspCommand::RawImu => vec![0u8; 18],
            // 16 motor outputs, u16 each (only first 4 used in quad)
            MspCommand::Motor => vec![0u8; 32],
            MspCommand::Rc => vec![0u8; 16],
            MspCommand::Pid => vec![0u8; 30],
            // Pidnames is a comma-separated string; keep it simple.
            MspCommand::Pidnames => b"ROLL;PITCH;YAW".to_vec(),
            // Writes return empty payload to acknowledge receipt.
            MspCommand::SetPid | MspCommand::EepromWrite => Vec::new(),
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

    /// Round-trip a V1 request through create_message → parse_request.
    /// The V2 path uses a different framing and parse_framed only handles
    /// V1 today; this guards V1 against regression.
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
}
