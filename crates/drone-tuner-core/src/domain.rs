//! Domain models representing flight data and analysis results.

use chrono::{DateTime, Utc};
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Unique identifier for flight sessions
pub type SessionId = uuid::Uuid;

/// Represents a complete flight session with all telemetry and analysis
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FlightSession {
    /// Session metadata including timing and hardware
    pub metadata: FlightMetadata,
    /// Raw telemetry data from the flight
    pub telemetry: TelemetryData,
    /// Flight events (mode changes, arming, etc.)
    pub events: Vec<FlightEvent>,
    /// Analysis results if computed
    pub analysis_results: Option<AnalysisReport>,
}

/// Metadata about a flight session
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FlightMetadata {
    /// Unique session identifier
    pub session_id: SessionId,
    /// When the flight occurred
    pub timestamp: DateTime<Utc>,
    /// Flight duration in milliseconds
    pub duration_ms: u64,
    /// Hardware configuration
    pub hardware: HardwareConfiguration,
    /// Environmental conditions
    pub environment: EnvironmentalConditions,
    /// Pilot profile information
    pub pilot: PilotProfile,
}

/// Hardware configuration of the drone
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HardwareConfiguration {
    /// Flight controller type and firmware version
    pub flight_controller: FlightController,
    /// Frame specifications
    pub frame: Frame,
    /// Motor and propeller setup
    pub propulsion: PropulsionSystem,
    /// Current PID configuration
    pub pid_config: PidConfiguration,
    /// Current filter settings
    pub filter_config: FilterConfiguration,
}

/// Flight controller information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FlightController {
    /// Firmware name (e.g., "Betaflight", "INAV", "ArduPilot")
    pub firmware: String,
    /// Firmware version
    pub version: String,
    /// Target board (e.g., "STM32F405", "STM32F7X2")
    pub target: String,
    /// Control loop rate in Hz
    pub loop_rate: u32,
}

/// Frame specifications
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Frame {
    /// Wheelbase in millimeters
    pub wheelbase_mm: u16,
    /// Total weight in grams
    pub weight_g: u16,
    /// Frame material
    pub material: String,
    /// Approximate moment of inertia (computed or estimated)
    #[serde(skip)]
    pub moment_of_inertia: Option<Vector3<f32>>,
}

/// Propulsion system details
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PropulsionSystem {
    /// Motor specifications
    pub motors: MotorSpec,
    /// Propeller specifications
    pub props: PropellerSpec,
    /// ESC specifications
    pub esc: EscSpec,
}

/// Motor specifications
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotorSpec {
    /// Motor model/name
    pub model: String,
    /// KV rating (RPM per volt)
    pub kv: u16,
    /// Stator size (e.g., "2207")
    pub stator_size: String,
}

/// Propeller specifications
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PropellerSpec {
    /// Diameter in inches
    pub diameter_inches: f32,
    /// Pitch in inches
    pub pitch_inches: f32,
    /// Number of blades
    pub blade_count: u8,
    /// Propeller material
    pub material: String,
}

/// ESC specifications
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EscSpec {
    /// ESC model/name
    pub model: String,
    /// Current rating in amps
    pub current_rating: u16,
    /// Protocol used (PWM, OneShot, DShot, etc.)
    pub protocol: String,
}

/// Current PID configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PidConfiguration {
    /// Roll axis PID values
    pub roll: PidValues,
    /// Pitch axis PID values  
    pub pitch: PidValues,
    /// Yaw axis PID values
    pub yaw: PidValues,
    /// Additional PID-related settings
    pub settings: PidSettings,
}

/// PID values for a single axis
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PidValues {
    /// Proportional gain
    pub p: f32,
    /// Integral gain
    pub i: f32,
    /// Derivative gain
    pub d: f32,
    /// Feedforward gain
    pub f: Option<f32>,
}

/// Additional PID settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PidSettings {
    /// TPA (Throttle PID Attenuation) settings
    pub tpa: Option<TpaSettings>,
    /// PID profile name/number
    pub profile: u8,
    /// Rate profile settings
    pub rates: RateSettings,
}

/// TPA (Throttle PID Attenuation) settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TpaSettings {
    /// TPA rate (0.0-1.0)
    pub rate: f32,
    /// Throttle breakpoint (0.0-1.0)
    pub breakpoint: f32,
}

/// Rate settings for stick response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RateSettings {
    /// Roll rate
    pub roll_rate: f32,
    /// Pitch rate
    pub pitch_rate: f32,
    /// Yaw rate
    pub yaw_rate: f32,
    /// RC expo values
    pub expo: ExpoSettings,
    /// Super rate values
    pub super_rate: SuperRateSettings,
}

/// Expo settings for stick feel
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExpoSettings {
    /// Roll expo
    pub roll: f32,
    /// Pitch expo
    pub pitch: f32,
    /// Yaw expo
    pub yaw: f32,
}

/// Super rate settings for maximum rates
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SuperRateSettings {
    /// Roll super rate
    pub roll: f32,
    /// Pitch super rate
    pub pitch: f32,
    /// Yaw super rate
    pub yaw: f32,
}

/// Current filter configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FilterConfiguration {
    /// Gyro low-pass filters
    pub gyro_filters: Vec<Filter>,
    /// D-term low-pass filters
    pub dterm_filters: Vec<Filter>,
    /// Notch filters
    pub notch_filters: Vec<NotchFilter>,
    /// Dynamic notch filter settings
    pub dynamic_notch: Option<DynamicNotchSettings>,
}

/// Low-pass or high-pass filter
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Filter {
    /// Filter type
    pub filter_type: FilterType,
    /// Cutoff frequency in Hz
    pub cutoff: f32,
    /// Filter order (1st order, 2nd order, etc.)
    pub order: u8,
}

/// Notch filter for targeting specific frequencies
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NotchFilter {
    /// Center frequency in Hz
    pub frequency: f32,
    /// Q factor (higher = narrower notch)
    pub q_factor: f32,
    /// Whether the filter is enabled
    pub enabled: bool,
}

/// Dynamic notch filter settings
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DynamicNotchSettings {
    /// Minimum frequency to track
    pub min_freq: f32,
    /// Maximum frequency to track
    pub max_freq: f32,
    /// Q factor
    pub q_factor: f32,
    /// Whether dynamic notch is enabled
    pub enabled: bool,
}

/// Types of filters available
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum FilterType {
    /// Low-pass filter (attenuates high frequencies)
    LowPass,
    /// High-pass filter (attenuates low frequencies)
    HighPass,
    /// Band-pass filter (allows specific frequency range)
    BandPass,
    /// Butterworth filter
    Butterworth,
    /// Chebyshev filter
    Chebyshev,
    /// Bessel filter
    Bessel,
}

/// Environmental conditions during flight
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnvironmentalConditions {
    /// Temperature in Celsius
    pub temperature_c: Option<f32>,
    /// Wind speed in m/s
    pub wind_speed_ms: Option<f32>,
    /// Wind direction in degrees
    pub wind_direction_deg: Option<f32>,
    /// Atmospheric pressure in hPa
    pub pressure_hpa: Option<f32>,
    /// Humidity as percentage
    pub humidity_percent: Option<f32>,
}

/// Pilot profile information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PilotProfile {
    /// Pilot identifier (anonymous)
    pub pilot_id: Option<String>,
    /// Skill level estimation
    pub skill_level: SkillLevel,
    /// Preferred flying style
    pub flying_style: FlyingStyle,
}

/// Pilot skill levels
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum SkillLevel {
    /// New to FPV flying
    Beginner,
    /// Comfortable with basic maneuvers
    Intermediate,
    /// Experienced with advanced techniques
    Advanced,
    /// Professional/competitive level
    Expert,
}

/// Flying styles affect tuning requirements
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum FlyingStyle {
    /// Smooth cinematic flight
    Cinematic,
    /// Freestyle with tricks and flips
    Freestyle,
    /// Racing with high-speed turns
    Racing,
    /// Long-range exploration
    LongRange,
    /// Mixed flying styles
    Mixed,
}

/// Raw telemetry data from the flight
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TelemetryData {
    /// Sampling rate in Hz
    pub sample_rate: f32,
    /// Gyroscope data (rotation rates)
    pub gyro: TimeSeriesVector3,
    /// Accelerometer data (linear acceleration)
    pub accel: TimeSeriesVector3,
    /// Motor output traces
    pub motor: Vec<MotorTrace>,
    /// PID error signals
    pub pid_error: PidErrorTrace,
    /// RC command inputs
    pub rc_commands: RcCommandTrace,
    /// Control loop timing variance
    pub loop_time_variance: f32,
    /// CPU load over time
    pub cpu_load: Vec<f32>,
}

/// Time-series data for 3D vectors (gyro, accel, etc.)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimeSeriesVector3 {
    /// X-axis data points
    pub x: Vec<f32>,
    /// Y-axis data points
    pub y: Vec<f32>,
    /// Z-axis data points
    pub z: Vec<f32>,
}

impl TimeSeriesVector3 {
    /// Create new time series with the given capacity
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            x: Vec::with_capacity(capacity),
            y: Vec::with_capacity(capacity),
            z: Vec::with_capacity(capacity),
        }
    }

    /// Add a new data point
    pub fn push(&mut self, point: Vector3<f32>) {
        self.x.push(point.x);
        self.y.push(point.y);
        self.z.push(point.z);
    }

    /// Get the number of data points
    pub fn len(&self) -> usize {
        self.x.len()
    }

    /// Check if empty
    pub fn is_empty(&self) -> bool {
        self.x.is_empty()
    }

    /// Compute magnitude at each time point
    pub fn magnitude(&self) -> Vec<f32> {
        (0..self.len())
            .map(|i| {
                let v = Vector3::new(self.x[i], self.y[i], self.z[i]);
                v.magnitude()
            })
            .collect()
    }

    /// Get a specific time point as a Vector3
    pub fn get(&self, index: usize) -> Option<Vector3<f32>> {
        if index < self.len() {
            Some(Vector3::new(self.x[index], self.y[index], self.z[index]))
        } else {
            None
        }
    }
}

/// Motor output trace for a single motor
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotorTrace {
    /// Motor number (1-based)
    pub motor_id: u8,
    /// Motor output values over time (0.0-1.0)
    pub values: Vec<f32>,
}

/// PID error traces for all axes
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PidErrorTrace {
    /// Roll axis error
    pub roll: Vec<f32>,
    /// Pitch axis error
    pub pitch: Vec<f32>,
    /// Yaw axis error
    pub yaw: Vec<f32>,
}

/// RC command traces
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RcCommandTrace {
    /// Roll commands (-1.0 to 1.0)
    pub roll: Vec<f32>,
    /// Pitch commands (-1.0 to 1.0)
    pub pitch: Vec<f32>,
    /// Yaw commands (-1.0 to 1.0)
    pub yaw: Vec<f32>,
    /// Throttle commands (0.0 to 1.0)
    pub throttle: Vec<f32>,
}

/// Flight events (mode changes, arming, etc.)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FlightEvent {
    /// Timestamp relative to start of flight (milliseconds)
    pub timestamp_ms: u64,
    /// Type of event
    pub event_type: FlightEventType,
    /// Optional additional data
    pub data: Option<HashMap<String, String>>,
}

/// Types of flight events
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum FlightEventType {
    /// Drone was armed
    Armed,
    /// Drone was disarmed
    Disarmed,
    /// Flight mode changed
    ModeChange {
        /// Previous mode
        from: String,
        /// New mode
        to: String,
    },
    /// Failsafe triggered
    Failsafe,
    /// GPS fix acquired
    GpsLock,
    /// Battery voltage warning
    LowBattery,
    /// Custom event with description
    Custom(String),
}

/// Results from analyzing a flight session
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnalysisReport {
    /// When this analysis was performed
    pub timestamp: DateTime<Utc>,
    /// Frequency domain analysis results
    pub frequency_analysis: FrequencyAnalysis,
    /// Detected issues and problems
    pub detected_issues: Vec<DetectedIssue>,
    /// Recommended filter changes
    pub filter_recommendations: Vec<FilterRecommendation>,
    /// Recommended PID changes
    pub pid_recommendations: Vec<PidRecommendation>,
    /// Confidence scores for various analyses
    pub confidence_scores: ConfidenceScores,
    /// Overall tune quality score (0-100)
    pub tune_quality_score: f32,
}

/// Frequency domain analysis results
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FrequencyAnalysis {
    /// Frequency bins (Hz)
    pub frequencies: Vec<f32>,
    /// Power spectral density for gyro X
    pub gyro_x_psd: Vec<f32>,
    /// Power spectral density for gyro Y
    pub gyro_y_psd: Vec<f32>,
    /// Power spectral density for gyro Z
    pub gyro_z_psd: Vec<f32>,
    /// Dominant frequency peaks
    pub peaks: Vec<FrequencyPeak>,
    /// Estimated noise floor
    pub noise_floor: f32,
}

/// A peak in the frequency spectrum
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FrequencyPeak {
    /// Peak frequency in Hz
    pub frequency: f32,
    /// Peak amplitude
    pub amplitude: f32,
    /// Q-factor (sharpness) of the peak
    pub q_factor: f32,
    /// Which axes this peak appears on
    pub axes: Vec<Axis>,
}

/// Axis enumeration
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq, Hash)]
pub enum Axis {
    /// Roll axis
    Roll,
    /// Pitch axis
    Pitch,
    /// Yaw axis
    Yaw,
}

/// Detected issues in the flight
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DetectedIssue {
    /// Type of issue detected
    pub issue_type: IssueType,
    /// Severity of the issue
    pub severity: Severity,
    /// Detailed description
    pub description: String,
    /// Affected axes
    pub affected_axes: Vec<Axis>,
    /// Confidence in this detection (0.0-1.0)
    pub confidence: f32,
}

/// Types of issues that can be detected
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum IssueType {
    /// P-term induced oscillation
    PTermOscillation {
        /// Frequency of oscillation
        frequency: f32,
        /// Amplitude of oscillation
        amplitude: f32,
    },
    /// D-term induced oscillation
    DTermOscillation {
        /// Frequency of oscillation
        frequency: f32,
        /// Amplitude of oscillation  
        amplitude: f32,
    },
    /// Mechanical resonance
    MechanicalResonance {
        /// Resonant frequency
        frequency: f32,
        /// Q-factor of resonance
        q_factor: f32,
    },
    /// Motor/prop imbalance
    Imbalance {
        /// Motor numbers affected
        motors: Vec<u8>,
    },
    /// Vibration from loose hardware
    LooseHardware,
    /// Insufficient filtering
    InsufficientFiltering {
        /// Frequency range needing attention
        frequency_range: (f32, f32),
    },
    /// Excessive filtering (loss of performance)
    ExcessiveFiltering {
        /// Estimated performance loss
        performance_loss: f32,
    },
}

/// Issue severity levels
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Severity {
    /// Low impact, cosmetic issue
    Low,
    /// Moderate impact on performance
    Medium,
    /// High impact, needs immediate attention
    High,
    /// Critical issue, unsafe to fly
    Critical,
}

/// Filter configuration recommendations
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FilterRecommendation {
    /// Type of filter change
    pub recommendation_type: FilterRecommendationType,
    /// Target frequency
    pub frequency: f32,
    /// Recommended Q-factor (for notch filters)
    pub q_factor: Option<f32>,
    /// Expected improvement
    pub expected_improvement: String,
    /// Priority of this recommendation
    pub priority: Priority,
}

/// Types of filter recommendations that match Betaflight's filter configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum FilterRecommendationType {
    /// Adjust gyro lowpass filter settings
    AdjustGyroLowpass {
        /// Which stage (1 or 2)
        stage: u8,
        /// Current cutoff frequency
        current_cutoff: f32,
        /// Recommended cutoff frequency
        recommended_cutoff: f32,
        /// Filter type (PT1, BIQUAD, etc.)
        filter_type: String,
    },
    /// Configure gyro notch filter
    ConfigureGyroNotch {
        /// Which notch filter (1 or 2)
        notch_number: u8,
        /// Center frequency
        frequency: f32,
        /// Q factor
        q_factor: f32,
        /// Enable or disable
        enabled: bool,
    },
    /// Adjust dynamic notch filter settings
    AdjustDynamicNotch {
        /// Number of notches
        notch_count: u8,
        /// Q factor
        q_factor: f32,
        /// Minimum frequency
        min_freq: f32,
        /// Maximum frequency
        max_freq: f32,
        /// Enable or disable
        enabled: bool,
    },
    /// Configure RPM filter
    ConfigureRpmFilter {
        /// Number of harmonics (0-3)
        harmonics: u8,
        /// Q factor
        q_factor: f32,
        /// Minimum frequency
        min_freq: f32,
        /// Enable or disable
        enabled: bool,
    },
    /// Adjust D-term lowpass filter
    AdjustDtermLowpass {
        /// Which stage (1 or 2)
        stage: u8,
        /// Current cutoff (None if dynamic)
        current_cutoff: Option<f32>,
        /// Recommended cutoff (None if should be dynamic)
        recommended_cutoff: Option<f32>,
        /// Filter type (PT1, BIQUAD)
        filter_type: String,
        /// Dynamic mode settings
        dynamic_settings: Option<DynamicFilterSettings>,
    },
    /// Adjust yaw lowpass filter
    AdjustYawLowpass {
        /// Current cutoff
        current_cutoff: f32,
        /// Recommended cutoff
        recommended_cutoff: f32,
    },
}

/// Dynamic filter settings for D-term filters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DynamicFilterSettings {
    /// Minimum cutoff frequency
    pub min_cutoff: f32,
    /// Maximum cutoff frequency
    pub max_cutoff: f32,
    /// Dynamic curve expo value
    pub expo: f32,
}

/// PID configuration recommendations
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PidRecommendation {
    /// Which axis this applies to
    pub axis: Axis,
    /// Which PID term to adjust
    pub term: PidTerm,
    /// Current value
    pub current_value: f32,
    /// Recommended value
    pub recommended_value: f32,
    /// Reason for the change
    pub reason: String,
    /// Priority of this recommendation
    pub priority: Priority,
}

/// PID terms
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum PidTerm {
    /// Proportional term
    P,
    /// Integral term
    I,
    /// Derivative term
    D,
    /// Feedforward term
    F,
}

/// Recommendation priorities
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Priority {
    /// Low priority, optional improvement
    Low,
    /// Medium priority, recommended
    Medium,
    /// High priority, strongly recommended
    High,
    /// Critical priority, required for safety
    Critical,
}

/// Confidence scores for different analyses
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConfidenceScores {
    /// Overall analysis confidence
    pub overall: f32,
    /// Oscillation detection confidence
    pub oscillation_detection: f32,
    /// Filter recommendation confidence
    pub filter_recommendations: f32,
    /// PID recommendation confidence
    pub pid_recommendations: f32,
    /// Mechanical issue detection confidence
    pub mechanical_issues: f32,
}

impl Default for EnvironmentalConditions {
    fn default() -> Self {
        Self {
            temperature_c: Some(25.0),
            wind_speed_ms: None,
            wind_direction_deg: None,
            pressure_hpa: None,
            humidity_percent: None,
        }
    }
}

impl Default for PilotProfile {
    fn default() -> Self {
        Self {
            pilot_id: None,
            skill_level: SkillLevel::Intermediate,
            flying_style: FlyingStyle::Freestyle,
        }
    }
}

impl HardwareConfiguration {
    /// Plausible default hardware configuration for tests and demos.
    ///
    /// Models a typical 5-inch freestyle quad: 2207-2300kv motors,
    /// triblade props, Betaflight 4.4, 1 kHz loop. Used by synthetic
    /// calibration tests and any other code that needs a non-empty
    /// hardware spec without a real flight log.
    #[must_use]
    pub fn test_default() -> Self {
        Self {
            flight_controller: FlightController {
                firmware: "Betaflight".to_string(),
                version: "4.4.0".to_string(),
                target: "STM32F405".to_string(),
                loop_rate: 1000,
            },
            frame: Frame {
                wheelbase_mm: 220,
                weight_g: 650,
                material: "Carbon Fiber".to_string(),
                moment_of_inertia: None,
            },
            propulsion: PropulsionSystem {
                motors: MotorSpec {
                    model: "Test Motor".to_string(),
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
                    model: "Test ESC".to_string(),
                    current_rating: 35,
                    protocol: "DShot600".to_string(),
                },
            },
            pid_config: PidConfiguration::default(),
            filter_config: FilterConfiguration::default(),
        }
    }
}

// Default implementations for configuration structures

impl Default for PidConfiguration {
    fn default() -> Self {
        Self {
            roll: PidValues {
                p: 42.0,
                i: 85.0,
                d: 38.0,
                f: Some(147.0),
            },
            pitch: PidValues {
                p: 46.0,
                i: 90.0,
                d: 42.0,
                f: Some(157.0),
            },
            yaw: PidValues {
                p: 45.0,
                i: 90.0,
                d: 0.0,
                f: Some(147.0),
            },
            settings: PidSettings {
                tpa: Some(TpaSettings {
                    rate: 0.65,
                    breakpoint: 1350.0,
                }),
                profile: 1,
                rates: RateSettings {
                    roll_rate: 670.0,
                    pitch_rate: 670.0,
                    yaw_rate: 670.0,
                    expo: ExpoSettings {
                        roll: 0.0,
                        pitch: 0.0,
                        yaw: 0.0,
                    },
                    super_rate: SuperRateSettings {
                        roll: 0.80,
                        pitch: 0.80,
                        yaw: 0.80,
                    },
                },
            },
        }
    }
}

impl Default for FilterConfiguration {
    fn default() -> Self {
        Self {
            gyro_filters: vec![Filter {
                filter_type: FilterType::LowPass,
                cutoff: 250.0,
                order: 2,
            }],
            dterm_filters: vec![Filter {
                filter_type: FilterType::LowPass,
                cutoff: 100.0,
                order: 2,
            }],
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
