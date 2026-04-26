//! Data conversion utilities for blackbox parsing.

use super::*;
use crate::domain::{FilterType, TelemetryData, TimeSeriesVector3};
use tracing::debug;

/// Data converter for transforming blackbox data to domain model
#[derive(Debug)]
pub struct DataConverter {
    /// Scale factors for different data types
    scale_factors: ScaleFactors,
}

/// Scale factors for converting raw values to physical units
#[derive(Debug, Clone)]
pub struct ScaleFactors {
    /// Gyro scale factor (raw to degrees/second)
    pub gyro: f32,
    /// Accelerometer scale factor (raw to g)
    pub accel: f32,
    /// Motor scale factor (raw to 0-1 range)
    pub motor: f32,
    /// RC command scale factor
    pub rc_command: f32,
    /// PID error scale factor
    pub pid_error: f32,
}

impl Default for ScaleFactors {
    fn default() -> Self {
        Self {
            gyro: 1.0 / 16.0,    // Typical 2000 dps range
            accel: 1.0 / 512.0,  // Typical 8g range
            motor: 1.0 / 2000.0, // PWM to 0-1 range
            rc_command: 1.0,     // Already in PWM units
            pid_error: 1.0,      // Already in correct units
        }
    }
}

impl DataConverter {
    /// Create a new data converter with default scale factors
    pub fn new() -> Self {
        Self {
            scale_factors: ScaleFactors::default(),
        }
    }

    /// Create a data converter with custom scale factors
    pub fn with_scale_factors(scale_factors: ScaleFactors) -> Self {
        Self { scale_factors }
    }

    /// Convert raw gyro values to degrees per second
    pub fn convert_gyro(&self, raw_values: &[i32; 3]) -> nalgebra::Vector3<f32> {
        nalgebra::Vector3::new(
            raw_values[0] as f32 * self.scale_factors.gyro,
            raw_values[1] as f32 * self.scale_factors.gyro,
            raw_values[2] as f32 * self.scale_factors.gyro,
        )
    }

    /// Convert raw accelerometer values to g
    pub fn convert_accel(&self, raw_values: &[i32; 3]) -> nalgebra::Vector3<f32> {
        nalgebra::Vector3::new(
            raw_values[0] as f32 * self.scale_factors.accel,
            raw_values[1] as f32 * self.scale_factors.accel,
            raw_values[2] as f32 * self.scale_factors.accel,
        )
    }

    /// Convert raw motor values to normalized range
    pub fn convert_motor(&self, raw_value: i32) -> f32 {
        (raw_value as f32 * self.scale_factors.motor).clamp(0.0, 1.0)
    }

    /// Convert raw RC command values
    pub fn convert_rc_command(&self, raw_value: i32) -> f32 {
        raw_value as f32 * self.scale_factors.rc_command
    }

    /// Convert raw PID error values
    pub fn convert_pid_error(&self, raw_value: i32) -> f32 {
        raw_value as f32 * self.scale_factors.pid_error
    }

    /// Detect scale factors from field definitions and ranges
    pub fn detect_scale_factors(
        &mut self,
        field_mappings: &FieldMappings,
        sample_data: &[Vec<i32>],
    ) {
        // Analyze sample data to determine appropriate scale factors
        self.detect_gyro_scale(field_mappings, sample_data);
        self.detect_accel_scale(field_mappings, sample_data);
        self.detect_motor_scale(field_mappings, sample_data);
    }

    /// Detect gyro scale factor from data range
    fn detect_gyro_scale(&mut self, field_mappings: &FieldMappings, sample_data: &[Vec<i32>]) {
        if let Some((x_idx, y_idx, z_idx)) = field_mappings.gyro_indices {
            let mut max_value = 0i32;

            for frame in sample_data.iter().take(1000) {
                // Sample first 1000 frames
                if let (Some(&x), Some(&y), Some(&z)) =
                    (frame.get(x_idx), frame.get(y_idx), frame.get(z_idx))
                {
                    max_value = max_value.max(x.abs()).max(y.abs()).max(z.abs());
                }
            }

            // Estimate scale factor based on maximum observed value
            if max_value > 0 {
                // Assume maximum reasonable gyro rate is 2000 deg/s
                let estimated_scale = 2000.0 / max_value as f32;
                if estimated_scale > 0.001 && estimated_scale < 10.0 {
                    self.scale_factors.gyro = estimated_scale;
                    debug!("Detected gyro scale factor: {}", estimated_scale);
                }
            }
        }
    }

    /// Detect accelerometer scale factor from data range
    fn detect_accel_scale(&mut self, field_mappings: &FieldMappings, sample_data: &[Vec<i32>]) {
        if let Some((x_idx, y_idx, z_idx)) = field_mappings.accel_indices {
            let mut values = Vec::new();

            for frame in sample_data.iter().take(1000) {
                if let (Some(&x), Some(&y), Some(&z)) =
                    (frame.get(x_idx), frame.get(y_idx), frame.get(z_idx))
                {
                    values.push(x);
                    values.push(y);
                    values.push(z);
                }
            }

            if !values.is_empty() {
                // Calculate magnitude to find 1g reference
                let avg_magnitude = values.iter().map(|&v| v as f32).map(|v| v * v).sum::<f32>()
                    / values.len() as f32;
                let avg_magnitude = avg_magnitude.sqrt();

                if avg_magnitude > 0.0 {
                    // Assume average magnitude represents 1g
                    let estimated_scale = 1.0 / avg_magnitude;
                    if estimated_scale > 0.0001 && estimated_scale < 1.0 {
                        self.scale_factors.accel = estimated_scale;
                        debug!("Detected accel scale factor: {}", estimated_scale);
                    }
                }
            }
        }
    }

    /// Detect motor scale factor from data range
    fn detect_motor_scale(&mut self, field_mappings: &FieldMappings, sample_data: &[Vec<i32>]) {
        if !field_mappings.motor_indices.is_empty() {
            let mut max_value = 0i32;

            for frame in sample_data.iter().take(1000) {
                for &motor_idx in &field_mappings.motor_indices {
                    if let Some(&motor_value) = frame.get(motor_idx) {
                        max_value = max_value.max(motor_value);
                    }
                }
            }

            if max_value > 0 {
                // Assume maximum motor value should map to 1.0
                let estimated_scale = 1.0 / max_value as f32;
                if estimated_scale > 0.0001 && estimated_scale < 1.0 {
                    self.scale_factors.motor = estimated_scale;
                    debug!("Detected motor scale factor: {}", estimated_scale);
                }
            }
        }
    }

    /// Apply filtering to reduce noise in converted data
    /// Note: This is a placeholder - real filtering would be implemented in the analysis module
    pub fn apply_filtering(&self, _data: &mut TimeSeriesVector3, _filter_type: FilterType) {
        // Filtering implementation would go in the analysis module
        // This is just a placeholder for the API
        debug!("Filtering requested but not implemented in converter");
    }

    // Filtering methods removed - will be implemented in analysis module

    /// Validate converted data for reasonableness
    pub fn validate_data(&self, telemetry: &TelemetryData) -> Vec<String> {
        let mut warnings = Vec::new();

        // Check gyro data using magnitude calculation from TimeSeriesVector3
        if !telemetry.gyro.is_empty() {
            let max_gyro = telemetry
                .gyro
                .magnitude()
                .iter()
                .fold(0.0f32, |max, &val| max.max(val));

            if max_gyro > 5000.0 {
                warnings.push(format!(
                    "Unusually high gyro values detected: {:.1} deg/s",
                    max_gyro
                ));
            }
        }

        // Check accelerometer data
        if !telemetry.accel.is_empty() {
            let magnitudes = telemetry.accel.magnitude();
            let avg_magnitude = magnitudes.iter().sum::<f32>() / magnitudes.len() as f32;

            if avg_magnitude < 0.5 || avg_magnitude > 2.0 {
                warnings.push(format!(
                    "Unusual accelerometer magnitude: {:.2}g (expected ~1g)",
                    avg_magnitude
                ));
            }
        }

        // Check motor data
        for motor in &telemetry.motor {
            if let Some(&max_motor) = motor.values.iter().max_by(|a, b| a.partial_cmp(b).unwrap()) {
                if max_motor > 1.5 {
                    warnings.push(format!(
                        "Motor {} values exceed expected range: {:.2}",
                        motor.motor_id, max_motor
                    ));
                }
            }
        }

        warnings
    }
}

impl Default for DataConverter {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_scale_factors_default() {
        let scale_factors = ScaleFactors::default();
        assert!(scale_factors.gyro > 0.0);
        assert!(scale_factors.accel > 0.0);
        assert!(scale_factors.motor > 0.0);
    }

    #[test]
    fn test_converter_creation() {
        let converter = DataConverter::new();
        assert!(converter.scale_factors.gyro > 0.0);
    }

    #[test]
    fn test_gyro_conversion() {
        let converter = DataConverter::new();
        let raw = [160, 320, 480]; // Example raw values
        let converted = converter.convert_gyro(&raw);

        // Check that conversion produces reasonable values
        assert_eq!(converted.x, 160.0 * converter.scale_factors.gyro);
        assert_eq!(converted.y, 320.0 * converter.scale_factors.gyro);
        assert_eq!(converted.z, 480.0 * converter.scale_factors.gyro);
    }

    #[test]
    fn test_motor_conversion() {
        let converter = DataConverter::new();
        let raw = 1500; // Example PWM value
        let converted = converter.convert_motor(raw);

        assert!(converted >= 0.0);
        assert!(converted <= 1.0);
        assert_eq!(converted, 1500.0 * converter.scale_factors.motor);
    }

    #[test]
    fn test_data_validation() {
        let converter = DataConverter::new();
        let mut gyro = TimeSeriesVector3::with_capacity(1);
        gyro.push(nalgebra::Vector3::new(100.0, 200.0, 300.0));

        let mut accel = TimeSeriesVector3::with_capacity(1);
        accel.push(nalgebra::Vector3::new(0.0, 0.0, 1.0)); // 1g downward

        let telemetry = TelemetryData {
            sample_rate: 1000.0,
            gyro,
            accel,
            motor: vec![MotorTrace {
                motor_id: 1,
                values: vec![0.5],
            }],
            pid_error: PidErrorTrace {
                roll: vec![0.0],
                pitch: vec![0.0],
                yaw: vec![0.0],
            },
            rc_commands: RcCommandTrace {
                roll: vec![1500.0],
                pitch: vec![1500.0],
                yaw: vec![1500.0],
                throttle: vec![1000.0],
            },
            loop_time_variance: 0.0,
            cpu_load: Vec::new(),
        };

        let warnings = converter.validate_data(&telemetry);
        assert!(warnings.is_empty()); // Should be no warnings for reasonable data

        // Test with high gyro values
        let mut high_gyro = TimeSeriesVector3::with_capacity(1);
        high_gyro.push(nalgebra::Vector3::new(10000.0, 0.0, 0.0));

        let high_gyro_telemetry = TelemetryData {
            sample_rate: 1000.0,
            gyro: high_gyro,
            accel: TimeSeriesVector3::with_capacity(0), // Empty
            motor: vec![],
            pid_error: PidErrorTrace {
                roll: vec![],
                pitch: vec![],
                yaw: vec![],
            },
            rc_commands: RcCommandTrace {
                roll: vec![],
                pitch: vec![],
                yaw: vec![],
                throttle: vec![],
            },
            loop_time_variance: 0.0,
            cpu_load: Vec::new(),
        };

        let warnings = converter.validate_data(&high_gyro_telemetry);
        assert!(!warnings.is_empty()); // Should have warnings now
    }

    #[test]
    fn test_filtering() {
        let converter = DataConverter::new();
        let mut data = TimeSeriesVector3::with_capacity(4);
        data.push(nalgebra::Vector3::new(1.0, 0.0, 0.0));
        data.push(nalgebra::Vector3::new(-1.0, 0.0, 0.0));
        data.push(nalgebra::Vector3::new(1.0, 0.0, 0.0));
        data.push(nalgebra::Vector3::new(-1.0, 0.0, 0.0));

        // Test that filtering function doesn't crash (it's a placeholder)
        converter.apply_filtering(&mut data, FilterType::LowPass);

        // Data should be unchanged since filtering is not implemented
        assert_eq!(data.len(), 4);
    }
}
