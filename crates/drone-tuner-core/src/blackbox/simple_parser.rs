//! Simple blackbox parser using the blackbox-log crate with correct API.

use super::*;
use crate::domain::{
    EnvironmentalConditions, FlyingStyle, MotorTrace, PidErrorTrace, PilotProfile, RcCommandTrace,
    SkillLevel, TimeSeriesVector3,
};
use blackbox_log::frame::{MainFrame, MainValue};
use blackbox_log::units::si::{
    acceleration::standard_gravity, angular_velocity::radian_per_second,
};
use blackbox_log::{prelude::*, File, ParserEvent};
use std::time::Instant;
use tracing::{debug, info, warn};

/// Frame interval configuration extracted from headers - matches Betaflight defaults
#[derive(Debug, Clone)]
struct FrameIntervals {
    /// I-interval: how often main frames are logged (in FC loop cycles)
    frame_interval_i: u32,
    /// P-interval numerator: controls partial frame density
    frame_interval_p_num: u32,
    /// P-interval denominator: controls partial frame density
    frame_interval_p_denom: u32,
    /// Base flight controller loop rate in Hz
    base_rate: Option<f32>,
}

impl Default for FrameIntervals {
    fn default() -> Self {
        // Betaflight defaults from defaultSysConfig
        Self {
            frame_interval_i: 32,
            frame_interval_p_num: 1,
            frame_interval_p_denom: 1,
            base_rate: None,
        }
    }
}

impl FrameIntervals {
    /// Exact Betaflight shouldHaveFrame logic - determines if a frame with given index should exist
    /// Reference: flightlog_parser.js line 1168-1176
    fn should_have_frame(&self, frame_index: u32) -> bool {
        ((frame_index % self.frame_interval_i) + self.frame_interval_p_num - 1)
            % self.frame_interval_p_denom
            < self.frame_interval_p_num
    }

    /// Calculate effective sample rate using Betaflight's shouldHaveFrame logic
    /// This counts exactly how many frames exist in a representative sample
    fn calculate_effective_rate(&self, base_rate: f32) -> f32 {
        // Calculate how many frames should exist in a full I-interval cycle
        // This gives us the exact frame density according to Betaflight logic
        let frames_in_cycle = self.count_expected_frames(0, self.frame_interval_i);

        // The cycle duration is I-interval base loop cycles
        let cycle_duration_s = self.frame_interval_i as f32 / base_rate;

        // Effective rate is frames per second
        let effective_rate = frames_in_cycle as f32 / cycle_duration_s;

        debug!(
            "Betaflight rate calculation: base={:.1}Hz, I-interval={}, P-interval={}/{}",
            base_rate,
            self.frame_interval_i,
            self.frame_interval_p_num,
            self.frame_interval_p_denom
        );
        debug!(
            "Frame pattern: {} frames every {} cycles = {:.1}Hz effective rate",
            frames_in_cycle, self.frame_interval_i, effective_rate
        );

        effective_rate
    }

    /// Count frames that should exist in a given range (for validation)
    fn count_expected_frames(&self, start_frame: u32, end_frame: u32) -> u32 {
        let mut count = 0;
        for frame_idx in start_frame..end_frame {
            if self.should_have_frame(frame_idx) {
                count += 1;
            }
        }
        count
    }
}

/// Centralized Betaflight-style time acceptance and period management
#[derive(Debug, Clone, Default)]
struct TimeSemantics {
    last_time_us: Option<u64>,
    current_period_start: Option<u64>,
    current_period_end: Option<u64>,
    best_period_start: Option<u64>,
    best_period_end: Option<u64>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FrameAction {
    /// Drop frame (e.g., time moved backwards)
    Skip,
    /// Accept frame in the current period
    AcceptSamePeriod,
    /// Accept frame and start a new period (after large forward jump or initial frame)
    AcceptNewPeriod,
}

impl TimeSemantics {
    /// Match Betaflight Explorer: split periods on forward jumps > 10s
    const MAX_FORWARD_JUMP_US: u64 = 10_000_000;

    /// Process a frame timestamp and decide acceptance semantics.
    fn on_time(&mut self, time_us: u64) -> FrameAction {
        match self.last_time_us {
            None => {
                // First valid timestamp starts a new period
                self.current_period_start = Some(time_us);
                self.current_period_end = Some(time_us);
                self.last_time_us = Some(time_us);
                FrameAction::AcceptNewPeriod
            }
            Some(last) => {
                if time_us < last {
                    // Strict: drop any backward time movement
                    return FrameAction::Skip;
                }

                if time_us > last.saturating_add(Self::MAX_FORWARD_JUMP_US) {
                    // Large forward jump: finalize current period, then start a new one
                    self.finalize_current_period_into_best();
                    self.current_period_start = Some(time_us);
                    self.current_period_end = Some(time_us);
                    self.last_time_us = Some(time_us);
                    return FrameAction::AcceptNewPeriod;
                }

                // Normal in-order frame
                if self.current_period_start.is_none() {
                    self.current_period_start = Some(time_us);
                }
                self.current_period_end = Some(time_us);
                self.last_time_us = Some(time_us);
                FrameAction::AcceptSamePeriod
            }
        }
    }

    /// Finalize current period at end of stream
    fn finalize_at_end(&mut self) {
        self.finalize_current_period_into_best();
    }

    /// Get the best (longest) continuous period detected
    fn best_period(&self) -> (Option<u64>, Option<u64>) {
        (self.best_period_start, self.best_period_end)
    }

    fn finalize_current_period_into_best(&mut self) {
        if let (Some(start), Some(end)) = (self.current_period_start, self.current_period_end) {
            let duration = end.saturating_sub(start);
            let best_duration = if let (Some(bstart), Some(bend)) = (self.best_period_start, self.best_period_end) {
                bend.saturating_sub(bstart)
            } else {
                0
            };
            if duration > best_duration {
                self.best_period_start = self.current_period_start;
                self.best_period_end = self.current_period_end;
            }
        }
    }
}

/// Helper to backfill event timestamps using the next main-frame time (Betaflight semantics)
#[derive(Debug, Default)]
struct EventBackfiller {
    /// Events collected in order; None means time not assigned yet
    event_times_us: Vec<Option<u64>>,
    /// Indices of events waiting for the next main-frame time
    pending_indices: Vec<usize>,
}

impl EventBackfiller {
    fn new() -> Self {
        Self::default()
    }

    /// Called when an event is encountered without a known timestamp
    fn on_event_without_time(&mut self) {
        let idx = self.event_times_us.len();
        self.event_times_us.push(None);
        self.pending_indices.push(idx);
    }

    /// Called when a main frame time is known; assigns this time to all pending events
    fn on_main_frame_time(&mut self, time_us: u64) {
        if self.pending_indices.is_empty() {
            return;
        }
        for idx in self.pending_indices.drain(..) {
            if let Some(slot) = self.event_times_us.get_mut(idx) {
                *slot = Some(time_us);
            }
        }
    }

    /// Finalize at end-of-stream: assign any trailing events the last known time
    fn finalize_end(&mut self, last_time_us: Option<u64>) {
        if let Some(t) = last_time_us {
            for idx in self.pending_indices.drain(..) {
                if let Some(slot) = self.event_times_us.get_mut(idx) {
                    *slot = Some(t);
                }
            }
        } else {
            // No known time; leave as None
            self.pending_indices.clear();
        }
    }

    fn take_times(self) -> Vec<Option<u64>> {
        self.event_times_us
    }
}

/// Information about a session in a blackbox file
#[derive(Debug, Clone)]
struct SessionInfo {
    /// Session index (0-based)
    index: usize,
    /// Number of main frames in this session
    main_frames: u64,
    /// Estimated duration in seconds
    duration_estimate_s: f32,
}

/// Simple blackbox parser that works with the actual blackbox-log API
#[derive(Debug)]
pub struct SimpleBlackboxParser {
    /// Parsing configuration
    config: ParsingConfig,
    /// Parsing statistics
    stats: ParsingStats,
    /// Optional concise summary of extracted BB parameters
    bb_summary: Option<String>,
}

impl SimpleBlackboxParser {
    /// Create a new simple blackbox parser
    pub fn new() -> Self {
        Self {
            config: ParsingConfig::default(),
            stats: ParsingStats::default(),
            bb_summary: None,
        }
    }

    /// Create parser with custom configuration
    pub fn with_config(config: ParsingConfig) -> Self {
        Self {
            config,
            stats: ParsingStats::default(),
            bb_summary: None,
        }
    }

    /// Parse a blackbox file and return basic telemetry data
    pub fn parse_file(&mut self, data: &[u8]) -> Result<FlightSession> {
        let start_time = Instant::now();
        info!("Starting simple blackbox parsing, {} bytes", data.len());

        // Validate the input
        if !utils::is_blackbox_file(data) {
            return Err(
                BlackboxError::InvalidFormat("Invalid blackbox file format".to_string()).into(),
            );
        }

        self.stats.bytes_processed = data.len();

        // Extract sample rate from raw headers before processing
        let sample_rate = self.extract_sample_rate_from_raw_data(data);

        // Create file from data
        let file = File::new(data);
        info!("Blackbox file created successfully");

        // Parse the blackbox file
        let (mut telemetry, duration_ms_override) = self.parse_blackbox_data(&file, data)?;

        // Update telemetry with extracted sample rate - if actual duration was calculated, use that for accuracy
        if telemetry.sample_rate <= 0.0 {
            telemetry.sample_rate = sample_rate;
        }

        // Extract hardware configuration from headers
        let hardware_config = if let Some(headers_result) = file.iter().next() {
            match headers_result {
                Ok(headers) => self.extract_hardware_configuration(&headers, sample_rate),
                Err(_) => {
                    warn!("Failed to extract headers for configuration, using defaults");
                    self.create_default_hardware_configuration(sample_rate)
                }
            }
        } else {
            warn!("No headers found for configuration extraction, using defaults");
            self.create_default_hardware_configuration(sample_rate)
        };

        // Build flight session with extracted configuration
        let session = self.build_flight_session_with_config_and_duration(
            telemetry,
            hardware_config,
            duration_ms_override,
        )?;

        self.stats.parse_duration_ms = start_time.elapsed().as_millis() as u64;
        info!(
            "Simple parsing completed in {}ms, {} total frames",
            self.stats.parse_duration_ms, self.stats.total_frames
        );

        Ok(session)
    }

    /// Parse the actual blackbox data and extract telemetry
    fn parse_blackbox_data(
        &mut self,
        file: &File,
        data: &[u8],
    ) -> BlackboxResult<(TelemetryData, Option<u64>)> {
        let mut telemetry_data = TelemetryData {
            sample_rate: 1000.0, // Will be updated when we find the actual rate
            gyro: TimeSeriesVector3::with_capacity(10000),
            accel: TimeSeriesVector3::with_capacity(10000),
            motor: Vec::new(),
            pid_error: PidErrorTrace {
                roll: Vec::new(),
                pitch: Vec::new(),
                yaw: Vec::new(),
            },
            rc_commands: RcCommandTrace {
                roll: Vec::new(),
                pitch: Vec::new(),
                yaw: Vec::new(),
                throttle: Vec::new(),
            },
            loop_time_variance: 0.0,
            cpu_load: Vec::new(),
        };

        let mut field_mappings = FieldMappings::default();
        let converter = DataConverter::new(); // No longer need scale factor detection
        let mut total_main_frames = 0u64;

        // First pass: Count all sessions to determine total count
        let session_info = self.detect_all_sessions(file, data)?;
        let total_sessions = session_info.len();

        info!("Detected {} session(s) in blackbox file", total_sessions);

        // If session listing is requested, return session information
        if self.config.list_sessions_only {
            self.log_session_breakdown(&session_info);
            // Return minimal telemetry data for listing mode
            return Ok((telemetry_data, None));
        }

        // Determine which session to process based on configuration
        let selected_session_index = if let Some(explicit_index) = self.config.selected_session {
            explicit_index
        } else {
            match self
                .config
                .session_strategy
                .clone()
                .unwrap_or(super::SessionStrategy::Last)
            {
                super::SessionStrategy::Last => total_sessions.saturating_sub(1),
                super::SessionStrategy::First => 0,
                super::SessionStrategy::Longest => {
                    // Choose the session with the longest estimated duration
                    let mut best_idx = 0usize;
                    let mut best_duration = -1.0f32;
                    for s in &session_info {
                        if s.duration_estimate_s > best_duration {
                            best_duration = s.duration_estimate_s;
                            best_idx = s.index;
                        }
                    }
                    best_idx
                }
            }
        };

        if selected_session_index >= total_sessions {
            return Err(BlackboxError::Parse(format!(
                "Invalid session index {}. File contains {} session(s) (valid range: 0-{})",
                selected_session_index,
                total_sessions,
                total_sessions.saturating_sub(1)
            )));
        }

        info!(
            "Processing session {} of {} (index {})",
            selected_session_index + 1,
            total_sessions,
            selected_session_index
        );

        // We'll compute exact duration for the selected session from the time field if present
        let mut selected_min_time_us: Option<u64> = None;
        let mut selected_max_time_us: Option<u64> = None;
        
        // Betaflight-style time semantics (period management, acceptance rules)
        let mut bf_time = TimeSemantics::default();

        // Second pass: Process only the selected session
        for (idx, headers_result) in file.iter().enumerate() {
            if idx != selected_session_index {
                continue; // Skip all sessions except the selected one
            }
            let headers = headers_result
                .map_err(|e| BlackboxError::Parse(format!("Failed to parse headers: {:?}", e)))?;

                debug!(
                "Processing session {} with {} main fields",
                selected_session_index + 1,
                headers.main_frame_def().len()
            );

            // Extract field mappings from headers
            self.extract_field_mappings(&headers, &mut field_mappings)?;

            // Sample rate is already set from raw header parsing

            // Create data parser
            let mut parser = headers.data_parser();
            // Event backfilling helper for this session
            let mut event_backfill = EventBackfiller::new();

            // Parse frames - Track iterations for proper duration calculation
            let mut main_frame_count = 0u64;

            while let Some(event) = parser.next() {
                match event {
                    ParserEvent::Main(main_frame) => {
                        main_frame_count += 1;
                        self.stats.main_frames += 1;

                        // Debug: Log every 1000th frame to understand the pattern
                        if main_frame_count % 1000 == 0 {
                            debug!(
                                "Main frame #{}: gyro samples so far = {}",
                                main_frame_count,
                                telemetry_data.gyro.len()
                            );
                        }

                        // Track min/max time for exact Betaflight-style duration using crate API
                        // Prefer direct time_raw() from MainFrame to avoid reliance on field names/filters
                        let t = main_frame.time_raw(); // raw is microseconds in u64
                        
                        match bf_time.on_time(t) {
                            FrameAction::Skip => {
                                // Drop this frame entirely (backward time)
                                continue;
                            }
                            FrameAction::AcceptSamePeriod | FrameAction::AcceptNewPeriod => {
                                // Inform backfiller about the main-frame time
                                event_backfill.on_main_frame_time(t);
                            }
                        }
                        
                        // Keep overall min/max for backward compatibility
                        selected_min_time_us = Some(selected_min_time_us.map_or(t, |m| m.min(t)));
                        selected_max_time_us = Some(selected_max_time_us.map_or(t, |m| m.max(t)));

                        // Extract telemetry data from main frame
                        self.extract_main_frame_data(
                            &main_frame,
                            &field_mappings,
                            &converter,
                            &mut telemetry_data,
                        );

                        // Apply frame limits if configured (only for main frames)
                        if let Some(max_frames) = self.config.max_frames {
                            if main_frame_count >= max_frames {
                                break;
                            }
                        }

                        // Progress reporting
                        if self.config.progress_reporting
                            && main_frame_count % self.config.progress_interval == 0
                        {
                            debug!("Processed {} main frames", main_frame_count);
                        }
                    }
                    ParserEvent::Slow(_slow_frame) => {
                        self.stats.slow_frames += 1;
                        // These are auxiliary frames, don't count towards duration
                    }
                    ParserEvent::Gps(_gps_frame) => {
                        self.stats.gps_frames += 1;
                        // GPS frames are separate from main data flow
                    }
                    ParserEvent::Event(_event_frame) => {
                        self.stats.event_frames += 1;
                        // Event frames are markers, not continuous data
                        // Queue for timestamp backfilling (Betaflight semantics)
                        event_backfill.on_event_without_time();
                    }
                }
            }

            // Finalize event backfilling with the last known time (best period end if present)
            {
                let (_best_start, best_end) = bf_time.best_period();
                event_backfill.finalize_end(best_end);
                let _event_times = event_backfill.take_times();
            }

            // Set total frames to the frames from the selected session only
            total_main_frames = main_frame_count;
            info!(
                "Session {} completed: {} main frames processed",
                selected_session_index + 1,
                main_frame_count
            );

            // Only process the selected session, then break
            break;
        }

        self.stats.total_frames = total_main_frames;

        // Update sample rate with proper algorithmic calculation
        // Use the frame interval configuration to get the actual effective rate
        let (intervals, found_intervals) = self.extract_frame_intervals_from_raw_data(data);
        if found_intervals {
            if let Some(base_rate) = intervals.base_rate {
                let pid_denom = self.extract_pid_process_denom(data).unwrap_or(1) as f32;
                let pid_rate = base_rate / pid_denom;
                let effective_rate = intervals.calculate_effective_rate(pid_rate);
                telemetry_data.sample_rate = effective_rate;
                debug!("Updated sample rate with pid_process_denom: base={:.1}Hz, denom={:.0} ⇒ pid={:.1}Hz, effective={:.1}Hz",
                      base_rate, pid_denom, pid_rate, effective_rate);
                // Build concise summary
                let p = format!(
                    "base={:.0}Hz, pid_denom={:.0} → pid={:.0}Hz, I={}, P={}/{}, effective≈{:.0}Hz",
                    base_rate,
                    pid_denom,
                    pid_rate,
                    intervals.frame_interval_i,
                    intervals.frame_interval_p_num,
                    intervals.frame_interval_p_denom,
                    effective_rate
                );
                self.bb_summary = Some(p);
            }
        } else {
            warn!(
                "No frame intervals found, keeping extracted sample rate: {:.1}Hz",
                telemetry_data.sample_rate
            );
        }

        debug!(
            "Extracted {} gyro samples, {} accel samples from {} main frames in session {}",
            telemetry_data.gyro.len(),
            telemetry_data.accel.len(),
            total_main_frames,
            selected_session_index + 1
        );

        // Debug: Check the samples-to-frames ratio
        let samples_per_frame = if total_main_frames > 0 {
            telemetry_data.gyro.len() as f64 / total_main_frames as f64
        } else {
            0.0
        };
        debug!(
            "Samples per frame ratio: {:.2} (expected: 1.0)",
            samples_per_frame
        );
        debug!("Final sample rate: {:.1}Hz", telemetry_data.sample_rate);

        // Initialize motor traces if we found motor data
        if !field_mappings.motor_indices.is_empty() {
            for (i, _) in field_mappings.motor_indices.iter().enumerate() {
                telemetry_data.motor.push(MotorTrace {
                    motor_id: (i + 1) as u8,
                    values: Vec::new(),
                });
            }
        }

        // Validate the extracted data
        let warnings = converter.validate_data(&telemetry_data);
        for warning in warnings {
            warn!("Data validation warning: {}", warning);
        }

        // Finalize the best period for main parsing (handle last period)
        bf_time.finalize_at_end();

        // Compute exact duration (ms) using time_raw() method (primary approach)
        let duration_ms_override =
            if let (Some(best_start), Some(best_end)) = bf_time.best_period() {
                // Use the longest continuous period (filters out setup/historical data)
                let delta_us = best_end.saturating_sub(best_start) as u64;
                let ms = (delta_us + 500) / 1000; // round to nearest ms
                debug!(
                    "Session {}: time_raw() duration = {:.1}s (best period: {} - {} = {} μs)",
                    selected_session_index + 1,
                    ms as f32 / 1000.0,
                    best_start,
                    best_end,
                    delta_us
                );
                Some(ms)
            } else if let (Some(min_t), Some(max_t)) = (selected_min_time_us, selected_max_time_us) {
                // Fallback to overall range if no periods detected
                let delta_us = max_t.saturating_sub(min_t) as u64;
                let ms = (delta_us + 500) / 1000; // round to nearest ms
                debug!(
                    "Session {}: time_raw() duration = {:.1}s (overall range: {} - {} = {} μs)",
                    selected_session_index + 1,
                    ms as f32 / 1000.0,
                    min_t,
                    max_t,
                    delta_us
                );
                Some(ms)
            } else if let Some(iter_idx) = field_mappings.iteration_idx {
                // Fallback: compute duration using loopIteration (when time_raw() fails)
                let (intervals, _) = self.extract_frame_intervals_from_raw_data(data);
                let base_rate = intervals.base_rate.unwrap_or(8000.0);
                let pid_denom = self.extract_pid_process_denom(data).unwrap_or(1) as f32;
                let mut min_iter: Option<u64> = None;
                let mut max_iter: Option<u64> = None;
                // Re-iterate quickly to find min/max iteration for the selected session
                if let Some(headers_result) = file.iter().nth(selected_session_index) {
                    if let Ok(headers) = headers_result {
                        let mut parser = headers.data_parser();
                        while let Some(event) = parser.next() {
                            if let ParserEvent::Main(main_frame) = event {
                                if let Some(v) = main_frame.get(iter_idx) {
                                    let it = self.main_value_to_u64(&v);
                                    min_iter = Some(min_iter.map_or(it, |m| m.min(it)));
                                    max_iter = Some(max_iter.map_or(it, |m| m.max(it)));
                                }
                            }
                        }
                    }
                }
                if let (Some(min_it), Some(max_it)) = (min_iter, max_iter) {
                    let iter_range = max_it.saturating_sub(min_it) as f32;
                    let pid_rate = base_rate / pid_denom;
                    let seconds = iter_range / pid_rate;
                    let ms = (seconds * 1000.0).round() as u64;
                    debug!("Session {}: iteration fallback duration = {:.1}s (iter {}..{}, base={:.1}Hz, denom {:.0}, pid={:.1}Hz)",
                          selected_session_index + 1, ms as f32 / 1000.0, min_it, max_it, base_rate, pid_denom, pid_rate);
                    Some(ms)
                } else {
                    None
                }
            } else {
                None
            };

        // Derive a sample rate from time_raw() if available as a robust fallback.
        // This reflects the actual logged main-frame rate (frames per second).
        if let (Some(min_t), Some(max_t)) = (selected_min_time_us, selected_max_time_us) {
            let seconds = (max_t.saturating_sub(min_t) as f32) / 1_000_000.0;
            if seconds > 0.0 && total_main_frames > 1 {
                let time_based_rate = total_main_frames as f32 / seconds;
                // If interval headers were missing or we previously used the heuristic default,
                // prefer the time-derived rate.
                let guessed_default = (telemetry_data.sample_rate - self.get_intelligent_default()).abs() < 1.0;
                if !found_intervals || guessed_default {
                    debug!(
                        "Updated sample rate from time_raw(): {:.1}Hz ({} frames / {:.3}s)",
                        time_based_rate,
                        total_main_frames,
                        seconds
                    );
                    telemetry_data.sample_rate = time_based_rate;
                    // Summary when intervals missing
                    self.bb_summary = Some(format!(
                        "frames={}, duration={:.3}s → effective≈{:.0}Hz (time_raw)",
                        total_main_frames, seconds, time_based_rate
                    ));
                }
            }
        }

        Ok((telemetry_data, duration_ms_override))
    }

    /// Extract sample rate and frame intervals from raw blackbox file data
    /// Returns the effective sample rate and frame interval configuration
    fn extract_sample_rate_from_raw_data(&self, data: &[u8]) -> f32 {
        let (intervals, found_intervals) = self.extract_frame_intervals_from_raw_data(data);

        if let Some(base_rate) = intervals.base_rate {
            if found_intervals {
                intervals.calculate_effective_rate(base_rate)
            } else {
                // Only base rate found, no interval configuration
                debug!(
                    "Using base loop rate {:.1}Hz (no interval configuration)",
                    base_rate
                );
                base_rate
            }
        } else {
            // Look for direct pid_rate header as fallback
            if let Some(direct_rate) = self.extract_direct_pid_rate(data) {
                debug!("Using direct pid_rate: {:.1}Hz", direct_rate);
                return direct_rate;
            }

            // Last resort: intelligent default
            let default_rate = self.get_intelligent_default();
            warn!(
                "Could not find sample rate in blackbox headers. \
                 Using intelligent default of {:.1}Hz. This may affect frequency analysis accuracy.",
                default_rate
            );
            default_rate
        }
    }

    /// Extract frame interval configuration from raw blackbox data
    /// This matches Betaflight's header parsing exactly
    fn extract_frame_intervals_from_raw_data(&self, data: &[u8]) -> (FrameIntervals, bool) {
        let content = String::from_utf8_lossy(data);
        let mut intervals = FrameIntervals::default();
        let mut found_i_interval = false;
        let mut found_p_interval = false;

        // Parse header lines (they start with 'H ')
        for line in content.lines() {
            if !line.starts_with("H ") {
                // Headers are at the beginning, stop when we hit non-header content
                break;
            }

            let header_content = &line[2..]; // Remove "H " prefix

            // Look for looptime headers (base flight controller loop rate)
            if header_content.contains("looptime:") {
                if let Some(looptime_us) = self.parse_looptime_value(header_content) {
                    intervals.base_rate = Some(1_000_000.0 / looptime_us);
                    info!(
                        "Found looptime: {}μs ({:.1}Hz base rate)",
                        looptime_us,
                        intervals.base_rate.unwrap()
                    );
                }
            }

            // Look for I interval headers (Betaflight main frame logging interval)
            // Reference: flightlog_parser.js line 588-592
            if header_content.contains("I interval:") {
                if let Some(interval) = self.parse_i_interval_value(header_content) {
                    intervals.frame_interval_i = interval.max(1); // Ensure at least 1
                    found_i_interval = true;
                    info!("Found I interval: {} cycles", intervals.frame_interval_i);
                }
            }

            // Look for P interval headers (Betaflight partial frame configuration)
            // Reference: flightlog_parser.js line 593-603
            if header_content.contains("P interval:") {
                if let Some((p_num, p_denom)) = self.parse_p_interval_value(header_content) {
                    intervals.frame_interval_p_num = p_num;
                    intervals.frame_interval_p_denom = p_denom;
                    found_p_interval = true;
                    info!("Found P interval: {}/{}", p_num, p_denom);
                }
            }
        }

        let found_intervals = found_i_interval || found_p_interval;
        if !found_intervals {
            warn!("No frame interval configuration found, using defaults (I=32, P=1/1)");
        }

        (intervals, found_intervals)
    }

    /// Extract direct pid_rate header as fallback
    fn extract_direct_pid_rate(&self, data: &[u8]) -> Option<f32> {
        let content = String::from_utf8_lossy(data);

        for line in content.lines() {
            if !line.starts_with("H ") {
                break;
            }

            let header_content = &line[2..];
            if header_content.contains("pid_rate:") {
                return self.parse_pid_rate_header(header_content);
            }
        }

        None
    }

    /// Parse looptime value from header (in microseconds)
    /// Format: "looptime:125" -> 125μs
    fn parse_looptime_value(&self, header: &str) -> Option<f32> {
        if let Some(colon_pos) = header.find(':') {
            let value_str = &header[colon_pos + 1..].trim();
            if let Ok(looptime_us) = value_str.parse::<f32>() {
                if looptime_us > 0.0 {
                    return Some(looptime_us);
                }
            }
        }
        None
    }

    /// Parse I interval value from header (in loop cycles)
    /// Format: "I interval:128" -> 128 cycles
    /// Reference: flightlog_parser.js line 588-592
    fn parse_i_interval_value(&self, header: &str) -> Option<u32> {
        if let Some(colon_pos) = header.find(':') {
            let value_str = header[colon_pos + 1..].trim();
            if let Ok(interval) = value_str.parse::<u32>() {
                if interval > 0 {
                    return Some(interval);
                }
            }
        }
        None
    }

    /// Parse P interval value from header (partial frame configuration)
    /// Format: "P interval:1/4" -> (1, 4) or "P interval:4" -> (1, 4)
    /// Reference: flightlog_parser.js line 593-603
    fn parse_p_interval_value(&self, header: &str) -> Option<(u32, u32)> {
        if let Some(colon_pos) = header.find(':') {
            let value_str = header[colon_pos + 1..].trim();

            // Handle "1/4" format - exact Betaflight logic
            if let Some(slash_pos) = value_str.find('/') {
                let num_str = &value_str[..slash_pos];
                let denom_str = &value_str[slash_pos + 1..];

                if let (Ok(num), Ok(denom)) = (num_str.parse::<u32>(), denom_str.parse::<u32>()) {
                    if num > 0 && denom > 0 {
                        return Some((num, denom));
                    }
                }
            }
            // Handle "4" format (implies 1/4) - Betaflight compatibility
            else if let Ok(denom) = value_str.parse::<u32>() {
                if denom > 0 {
                    return Some((1, denom));
                }
            }
        }
        None
    }

    

    /// Parse pid_rate header to extract sample rate
    /// Format: "pid_rate:4000" (already in Hz)
    fn parse_pid_rate_header(&self, header: &str) -> Option<f32> {
        if let Some(colon_pos) = header.find(':') {
            let value_str = &header[colon_pos + 1..].trim();
            if let Ok(rate) = value_str.parse::<f32>() {
                if rate > 0.0 {
                    return Some(rate);
                }
            }
        }
        None
    }

    /// Extract pid_process_denom from headers (affects loopIteration frequency)
    /// Format: "pid_process_denom:2" -> 2
    fn extract_pid_process_denom(&self, data: &[u8]) -> Option<u32> {
        let content: std::borrow::Cow<'_, str> = String::from_utf8_lossy(data);

        for line in content.lines() {
            if !line.starts_with("H ") {
                break;
            }

            let header_content = &line[2..];
            if header_content.starts_with("pid_process_denom:") {
                if let Some(colon_pos) = header_content.find(':') {
                    let value_str = header_content[colon_pos + 1..].trim();
                    if let Ok(denom) = value_str.parse::<u32>() {
                        if denom > 0 {
                            return Some(denom);
                        }
                    }
                }
            }
        }

        None
    }

    /// Get intelligent default sample rate based on modern firmware patterns
    fn get_intelligent_default(&self) -> f32 {
        // Modern racing quads typically run at:
        // - 4000Hz (250μs) - Most common for 4" and 5" racing quads
        // - 8000Hz (125μs) - High-performance setups with powerful FCs
        // - 2000Hz (500μs) - Some freestyle/cinematic setups
        // - 1000Hz (1000μs) - Old firmware (pre-2020)

        // 4kHz is the most common rate for modern racing quads
        4000.0
    }

    /// Extract field mappings from blackbox headers
    fn extract_field_mappings(
        &self,
        headers: &blackbox_log::Headers,
        mappings: &mut FieldMappings,
    ) -> BlackboxResult<()> {
        let main_fields = headers.main_frame_def();
        // CRITICAL DEBUG: Log all available fields to understand structure
        debug!("Available main frame fields ({} total):", main_fields.len());
        for (idx, field) in main_fields.iter().enumerate() {
            debug!("  [{}]: {} (checking for time field)", idx, field.name);
            // Add explicit time field detection debugging
            if field.name == "time" {
                debug!("  >>> FOUND EXACT TIME FIELD AT INDEX {}", idx);
            }
            if field.name.contains("time") {
                debug!(
                    "  >>> FOUND TIME-CONTAINING FIELD '{}' AT INDEX {}",
                    field.name, idx
                );
            }
        }

        // Find gyro fields (gyroADC[0], gyroADC[1], gyroADC[2])
        let mut gyro_x_idx = None;
        let mut gyro_y_idx = None;
        let mut gyro_z_idx = None;

        // Find accelerometer fields (accSmooth[0], accSmooth[1], accSmooth[2])
        let mut accel_x_idx = None;
        let mut accel_y_idx = None;
        let mut accel_z_idx = None;

        // Find motor fields
        let mut motor_indices = Vec::new();

        // Find RC command fields - CRITICAL FIX: Add RC command field detection
        let mut rc_roll_idx = None;
        let mut rc_pitch_idx = None;
        let mut rc_yaw_idx = None;
        let mut rc_throttle_idx = None;

        // Find iteration field (critical for fallback duration calculation)
        let mut iteration_idx = None;

        for (idx, field) in main_fields.iter().enumerate() {
            let field_name = field.name;
            let lname = field_name.to_ascii_lowercase();
            let normalized: String = lname
                .chars()
                .filter(|c| c.is_ascii_alphanumeric())
                .collect();

            // Iteration field (fallback for duration calculation when time_raw() fails)
            if normalized == "loopiteration" || normalized == "iteration" {
                iteration_idx = Some(idx);
                debug!("Found iteration field '{}' at index {}", field_name, idx);
            }
            // Gyro fields
            else if field_name == "gyroADC[0]" {
                gyro_x_idx = Some(idx);
            } else if field_name == "gyroADC[1]" {
                gyro_y_idx = Some(idx);
            } else if field_name == "gyroADC[2]" {
                gyro_z_idx = Some(idx);
            }
            // Accelerometer fields
            else if field_name == "accSmooth[0]" {
                accel_x_idx = Some(idx);
            } else if field_name == "accSmooth[1]" {
                accel_y_idx = Some(idx);
            } else if field_name == "accSmooth[2]" {
                accel_z_idx = Some(idx);
            }
            // Motor fields
            else if field_name.starts_with("motor[") {
                motor_indices.push(idx);
            }
            // RC command fields - CRITICAL FIX: Implement RC command field extraction
            else if field_name == "rcCommand[0]" {
                rc_roll_idx = Some(idx);
                debug!("Found RC roll command at index {}", idx);
            } else if field_name == "rcCommand[1]" {
                rc_pitch_idx = Some(idx);
                debug!("Found RC pitch command at index {}", idx);
            } else if field_name == "rcCommand[2]" {
                rc_yaw_idx = Some(idx);
                debug!("Found RC yaw command at index {}", idx);
            } else if field_name == "rcCommand[3]" {
                rc_throttle_idx = Some(idx);
                debug!("Found RC throttle command at index {}", idx);
            }
            // PID error fields
            // TODO: Add more field mappings as needed
        }

        // Set gyro indices if all three axes found
        if let (Some(x), Some(y), Some(z)) = (gyro_x_idx, gyro_y_idx, gyro_z_idx) {
            mappings.gyro_indices = Some((x, y, z));
            debug!("Found gyro fields at indices: {}, {}, {}", x, y, z);
        } else {
            warn!("Could not find all gyro fields in blackbox data");
        }

        // Set accelerometer indices if all three axes found
        if let (Some(x), Some(y), Some(z)) = (accel_x_idx, accel_y_idx, accel_z_idx) {
            mappings.accel_indices = Some((x, y, z));
            debug!("Found accel fields at indices: {}, {}, {}", x, y, z);
        } else {
            warn!("Could not find all accelerometer fields in blackbox data");
        }

        // Set motor indices
        mappings.motor_indices = motor_indices;
        if !mappings.motor_indices.is_empty() {
            debug!("Found {} motor fields", mappings.motor_indices.len());
        }

        // Set RC command indices - CRITICAL FIX: Update field mappings with RC indices
        if let (Some(roll), Some(pitch), Some(yaw), Some(throttle)) =
            (rc_roll_idx, rc_pitch_idx, rc_yaw_idx, rc_throttle_idx)
        {
            mappings.rc_command_indices = Some((roll, pitch, yaw, throttle));
            debug!(
                "Found all RC command fields at indices: {}, {}, {}, {}",
                roll, pitch, yaw, throttle
            );
        } else {
            warn!("RC command fields not found - step response analysis will be limited");
        }

        // Set iteration index for proper duration calculation
        mappings.iteration_idx = iteration_idx;
        if iteration_idx.is_some() {
            debug!("Found iteration field - duration calculation will use FC loop iterations");
        } else {
            warn!("No iteration field found - duration calculation will use frame count (may be inaccurate)");
        }

        // Time tracking uses time_raw() method from blackbox-log crate (primary approach)
        // Iteration field is used as fallback only if time_raw() fails
        debug!("Duration calculation: time_raw() (primary), loopIteration (fallback)");

        Ok(())
    }

    /// Extract hardware configuration from blackbox headers
    fn extract_hardware_configuration(
        &self,
        headers: &blackbox_log::Headers,
        sample_rate: f32,
    ) -> HardwareConfiguration {
        // Get unknown headers that contain configuration data
        let unknown_headers = headers.unknown();

        debug!(
            "Extracting configuration from {} unknown headers",
            unknown_headers.len()
        );

        // Convert headers to a more convenient format for processing
        let header_map: std::collections::HashMap<String, String> = unknown_headers
            .iter()
            .map(|(k, v)| (k.to_string(), v.to_string()))
            .collect();

        // Extract flight controller info
        let flight_controller = self.extract_flight_controller_info(&header_map, sample_rate);

        // Extract PID configuration
        let pid_config = self.extract_pid_configuration(&header_map);

        // Extract filter configuration
        let filter_config = self.extract_filter_configuration(&header_map);

        // Create hardware configuration with extracted values
        HardwareConfiguration {
            flight_controller,
            frame: crate::domain::Frame {
                wheelbase_mm: 220,                    // Default - not typically in blackbox
                weight_g: 650,                        // Default - not typically in blackbox
                material: "Carbon Fiber".to_string(), // Default
                moment_of_inertia: None,
            },
            propulsion: PropulsionSystem {
                motors: MotorSpec {
                    model: "Unknown".to_string(),    // Not typically in blackbox
                    kv: 2300,                        // Default estimate
                    stator_size: "2207".to_string(), // Default estimate
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
                    protocol: "DShot600".to_string(), // Common default
                },
            },
            pid_config,
            filter_config,
        }
    }

    /// Extract flight controller information from headers
    fn extract_flight_controller_info(
        &self,
        headers: &std::collections::HashMap<String, String>,
        sample_rate: f32,
    ) -> FlightController {
        let firmware = headers
            .get("Firmware revision")
            .map(|s| {
                if s.contains("Betaflight") {
                    "Betaflight".to_string()
                } else if s.contains("INAV") {
                    "INAV".to_string()
                } else if s.contains("ArduPilot") {
                    "ArduPilot".to_string()
                } else {
                    "Unknown".to_string()
                }
            })
            .unwrap_or_else(|| "Unknown".to_string());

        let version = headers
            .get("Firmware revision")
            .and_then(|s| {
                // Extract version number from strings like "Betaflight 4.4.0"
                s.split_whitespace().last().map(|v| v.to_string())
            })
            .unwrap_or_else(|| "Unknown".to_string());

        let target = headers
            .get("Board information")
            .or_else(|| headers.get("target"))
            .unwrap_or(&"Unknown".to_string())
            .clone();

        FlightController {
            firmware,
            version,
            target,
            loop_rate: sample_rate as u32,
        }
    }

    /// Extract PID configuration from headers
    fn extract_pid_configuration(
        &self,
        headers: &std::collections::HashMap<String, String>,
    ) -> PidConfiguration {
        // Extract PID values
        let roll_pid = self.parse_pid_triple(headers.get("rollPID"));
        let pitch_pid = self.parse_pid_triple(headers.get("pitchPID"));
        let yaw_pid = self.parse_pid_triple(headers.get("yawPID"));
        info!(
            "Extracted PID values - Roll: {:?}, Pitch: {:?}, Yaw: {:?}",
            roll_pid, pitch_pid, yaw_pid
        );

        // Extract D-min values
        let d_min = self.parse_pid_triple(headers.get("d_min"));
        info!("Extracted D-min values: {:?}", d_min);

        // Extract RC rates
        let rc_rates = self.parse_triple_values(headers.get("rc_rates"));
        let rc_expo = self.parse_triple_values(headers.get("rc_expo"));
        let rates = self.parse_triple_values(headers.get("rates"));
        info!(
            "Extracted RC settings - Rates: {:?}, Expo: {:?}, Super rates: {:?}",
            rc_rates, rc_expo, rates
        );

        PidConfiguration {
            roll: PidValues {
                p: roll_pid.0,
                i: roll_pid.1,
                d: d_min.0.max(roll_pid.2), // Use d_min if available, otherwise D value
                f: None,                    // F-term not commonly logged
            },
            pitch: PidValues {
                p: pitch_pid.0,
                i: pitch_pid.1,
                d: d_min.1.max(pitch_pid.2),
                f: None,
            },
            yaw: PidValues {
                p: yaw_pid.0,
                i: yaw_pid.1,
                d: d_min.2.max(yaw_pid.2),
                f: None,
            },
            settings: PidSettings {
                tpa: None,  // TPA not commonly in basic headers
                profile: 1, // Default profile
                rates: RateSettings {
                    roll_rate: rc_rates.0 / 100.0, // Convert from percentage
                    pitch_rate: rc_rates.1 / 100.0,
                    yaw_rate: rc_rates.2 / 100.0,
                    expo: ExpoSettings {
                        roll: rc_expo.0 / 100.0,
                        pitch: rc_expo.1 / 100.0,
                        yaw: rc_expo.2 / 100.0,
                    },
                    super_rate: SuperRateSettings {
                        roll: rates.0 / 100.0,
                        pitch: rates.1 / 100.0,
                        yaw: rates.2 / 100.0,
                    },
                },
            },
        }
    }

    /// Extract filter configuration from headers
    fn extract_filter_configuration(
        &self,
        headers: &std::collections::HashMap<String, String>,
    ) -> FilterConfiguration {
        let mut gyro_filters = Vec::new();
        let mut dterm_filters = Vec::new();
        let mut dynamic_notch = None;

        // Extract gyro low-pass filter
        if let Some(gyro_lpf) = headers
            .get("gyro_lpf2_static_hz")
            .or_else(|| headers.get("gyro_lowpass_hz"))
        {
            if let Ok(cutoff) = gyro_lpf.parse::<f32>() {
                if cutoff > 0.0 {
                    gyro_filters.push(Filter {
                        filter_type: FilterType::LowPass,
                        cutoff,
                        order: 2, // Typical for gyro filters
                    });
                    debug!("Extracted gyro LPF: {}Hz", cutoff);
                }
            }
        }

        // Extract D-term low-pass filter
        if let Some(dterm_lpf) = headers
            .get("dterm_lpf1_static_hz")
            .or_else(|| headers.get("dterm_lowpass_hz"))
        {
            if let Ok(cutoff) = dterm_lpf.parse::<f32>() {
                if cutoff > 0.0 {
                    dterm_filters.push(Filter {
                        filter_type: FilterType::LowPass,
                        cutoff,
                        order: 1, // Typical for D-term filters
                    });
                    debug!("Extracted D-term LPF: {}Hz", cutoff);
                }
            }
        }

        // Extract dynamic notch settings
        if let Some(dyn_notch_max) = headers.get("dyn_notch_max_hz") {
            if let Ok(max_freq) = dyn_notch_max.parse::<f32>() {
                if max_freq > 0.0 {
                    // Look for min frequency, default to common value if not found
                    let min_freq = headers
                        .get("dyn_notch_min_hz")
                        .and_then(|s| s.parse::<f32>().ok())
                        .unwrap_or(150.0);

                    dynamic_notch = Some(DynamicNotchSettings {
                        min_freq,
                        max_freq,
                        q_factor: 120.0, // Common Q factor
                        enabled: true,
                    });
                    debug!("Extracted dynamic notch: {}-{}Hz", min_freq, max_freq);
                }
            }
        }

        // If no filters were extracted, use sensible defaults
        if gyro_filters.is_empty() {
            gyro_filters.push(Filter {
                filter_type: FilterType::LowPass,
                cutoff: 500.0, // Conservative default
                order: 2,
            });
            warn!("No gyro filter configuration found, using default 500Hz LPF");
        }

        if dterm_filters.is_empty() {
            dterm_filters.push(Filter {
                filter_type: FilterType::LowPass,
                cutoff: 100.0, // Conservative default
                order: 1,
            });
            warn!("No D-term filter configuration found, using default 100Hz LPF");
        }

        FilterConfiguration {
            gyro_filters,
            dterm_filters,
            notch_filters: Vec::new(), // Static notch filters not commonly in basic headers
            dynamic_notch,
        }
    }

    /// Parse PID triple values from header string
    /// Expected format: "61,110,41" -> (P, I, D)
    fn parse_pid_triple(&self, header_value: Option<&String>) -> (f32, f32, f32) {
        match header_value {
            Some(value) => {
                let parts: Vec<&str> = value.split(',').collect();
                if parts.len() >= 3 {
                    let p = parts[0].trim().parse::<f32>().unwrap_or(45.0);
                    let i = parts[1].trim().parse::<f32>().unwrap_or(90.0);
                    let d = parts[2].trim().parse::<f32>().unwrap_or(35.0);
                    (p, i, d)
                } else {
                    warn!("Invalid PID triple format: {}", value);
                    (45.0, 90.0, 35.0) // Sensible defaults
                }
            }
            None => (45.0, 90.0, 35.0), // Sensible defaults
        }
    }

    /// Parse general triple values from header string
    /// Expected format: "70,70,70" -> (val1, val2, val3)
    fn parse_triple_values(&self, header_value: Option<&String>) -> (f32, f32, f32) {
        match header_value {
            Some(value) => {
                let parts: Vec<&str> = value.split(',').collect();
                if parts.len() >= 3 {
                    let val1 = parts[0].trim().parse::<f32>().unwrap_or(70.0);
                    let val2 = parts[1].trim().parse::<f32>().unwrap_or(70.0);
                    let val3 = parts[2].trim().parse::<f32>().unwrap_or(70.0);
                    (val1, val2, val3)
                } else {
                    warn!("Invalid triple values format: {}", value);
                    (70.0, 70.0, 70.0) // Sensible defaults
                }
            }
            None => (70.0, 70.0, 70.0), // Sensible defaults
        }
    }

    /// Extract telemetry data from a main frame
    fn extract_main_frame_data(
        &self,
        frame: &MainFrame<'_, '_, '_>,
        mappings: &FieldMappings,
        _converter: &DataConverter,
        telemetry: &mut TelemetryData,
    ) {
        // Extract gyro data - blackbox-log crate already converts to rad/s
        if let Some((x_idx, y_idx, z_idx)) = mappings.gyro_indices {
            if let (Some(x_val), Some(y_val), Some(z_val)) =
                (frame.get(x_idx), frame.get(y_idx), frame.get(z_idx))
            {
                // The blackbox-log crate already converts gyro values to rad/s
                let gyro_vector = nalgebra::Vector3::new(
                    self.main_value_to_f32(&x_val),
                    self.main_value_to_f32(&y_val),
                    self.main_value_to_f32(&z_val),
                );
                telemetry.gyro.push(gyro_vector);
            }
        }

        // Extract accelerometer data - blackbox-log crate already converts to g-force
        if let Some((x_idx, y_idx, z_idx)) = mappings.accel_indices {
            if let (Some(x_val), Some(y_val), Some(z_val)) =
                (frame.get(x_idx), frame.get(y_idx), frame.get(z_idx))
            {
                // The blackbox-log crate already converts accel values to g-force
                let accel_vector = nalgebra::Vector3::new(
                    self.main_value_to_f32(&x_val),
                    self.main_value_to_f32(&y_val),
                    self.main_value_to_f32(&z_val),
                );
                telemetry.accel.push(accel_vector);
            }
        }

        // Extract motor data
        for (motor_idx, &field_idx) in mappings.motor_indices.iter().enumerate() {
            if let Some(motor_val) = frame.get(field_idx) {
                // Motor values are typically PWM values (unitless)
                let motor_value = self.main_value_to_f32(&motor_val);
                let normalized_motor = (motor_value / 2000.0).clamp(0.0, 1.0); // Normalize PWM to 0-1

                // Ensure we have enough motor traces
                while telemetry.motor.len() <= motor_idx {
                    telemetry.motor.push(MotorTrace {
                        motor_id: (telemetry.motor.len() + 1) as u8,
                        values: Vec::new(),
                    });
                }

                telemetry.motor[motor_idx].values.push(normalized_motor);
            }
        }

        // Extract RC command data - CRITICAL FIX: Add RC data extraction in frame processing
        if let Some((roll_idx, pitch_idx, yaw_idx, throttle_idx)) = mappings.rc_command_indices {
            if let Some(roll_val) = frame.get(roll_idx) {
                telemetry
                    .rc_commands
                    .roll
                    .push(self.main_value_to_f32(&roll_val));
            }
            if let Some(pitch_val) = frame.get(pitch_idx) {
                telemetry
                    .rc_commands
                    .pitch
                    .push(self.main_value_to_f32(&pitch_val));
            }
            if let Some(yaw_val) = frame.get(yaw_idx) {
                telemetry
                    .rc_commands
                    .yaw
                    .push(self.main_value_to_f32(&yaw_val));
            }
            if let Some(throttle_val) = frame.get(throttle_idx) {
                telemetry
                    .rc_commands
                    .throttle
                    .push(self.main_value_to_f32(&throttle_val));
            }
        }

        // TODO: Extract PID errors, etc.
    }

    /// Convert blackbox MainValue to f32 for telemetry data
    fn main_value_to_f32(&self, value: &MainValue) -> f32 {
        match value {
            MainValue::Signed(v) => *v as f32,
            MainValue::Unsigned(v) => *v as f32,
            MainValue::Amperage(current) => {
                current.get::<blackbox_log::units::si::electric_current::ampere>() as f32
            }
            MainValue::Voltage(voltage) => {
                voltage.get::<blackbox_log::units::si::electric_potential::volt>() as f32
            }
            MainValue::Acceleration(accel) => accel.get::<standard_gravity>() as f32, // g-force
            MainValue::Rotation(angular_vel) => angular_vel.get::<radian_per_second>() as f32, // rad/s
        }
    }

    

    /// Convert blackbox MainValue to u64 for iteration tracking
    fn main_value_to_u64(&self, value: &MainValue) -> u64 {
        self.main_value_to_f32(value) as u64
    }

    

    /// Create default hardware configuration when extraction fails
    fn create_default_hardware_configuration(&self, sample_rate: f32) -> HardwareConfiguration {
        HardwareConfiguration {
            flight_controller: FlightController {
                firmware: "Betaflight".to_string(),
                version: "Unknown".to_string(),
                target: "Unknown".to_string(),
                loop_rate: sample_rate as u32,
            },
            frame: crate::domain::Frame {
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
            pid_config: PidConfiguration::default(),
            filter_config: FilterConfiguration::default(),
        }
    }

    /// Build flight session with extracted hardware configuration
    fn build_flight_session_with_config_and_duration(
        &self,
        telemetry: TelemetryData,
        hardware: HardwareConfiguration,
        duration_ms_override: Option<u64>,
    ) -> BlackboxResult<FlightSession> {
        let session_id = Uuid::new_v4();
        let timestamp = Utc::now();

        let duration_ms = if let Some(ms) = duration_ms_override {
            ms
        } else if telemetry.sample_rate > 0.0 && !telemetry.gyro.is_empty() {
            ((telemetry.gyro.len() as f32) / telemetry.sample_rate * 1000.0) as u64
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

        Ok(FlightSession {
            metadata,
            telemetry,
            events: Vec::new(),
            analysis_results: None,
        })
    }

    /// Detect all sessions in the blackbox file and return their metadata
    /// Uses time-based duration calculation like Betaflight for accuracy
    fn detect_all_sessions(
        &self,
        file: &File<'_>,
        data: &[u8],
    ) -> BlackboxResult<Vec<SessionInfo>> {
        let mut sessions = Vec::new();

        for (idx, headers_result) in file.iter().enumerate() {
            match headers_result {
                Ok(headers) => {
                    // Extract field mappings to find time and iteration fields
                    let mut temp_mappings = FieldMappings::default();
                    if let Err(e) = self.extract_field_mappings(&headers, &mut temp_mappings) {
                        warn!(
                            "Failed to extract field mappings for session {}: {:?}",
                            idx + 1,
                            e
                        );
                        continue;
                    }

                    // Create a parser to analyze this session
                    let mut parser = headers.data_parser();
                    let mut main_frame_count = 0u64;
                    let mut min_time: Option<u64> = None;
                    let mut max_time: Option<u64> = None;
                    let mut min_iteration: Option<u64> = None;
                    let mut max_iteration: Option<u64> = None;
                    
                    // Betaflight-style time semantics for this session
                    let mut bf_time = TimeSemantics::default();

                    // Parse frames to track time and iteration ranges
                    while let Some(event) = parser.next() {
                        if let blackbox_log::ParserEvent::Main(main_frame) = event {
                            main_frame_count += 1;

                            // Track time using crate API; independent from field names
                            let time = main_frame.time_raw();
                            match bf_time.on_time(time) {
                                FrameAction::Skip => {
                                    // Drop backward-time frame
                                    continue;
                                }
                                FrameAction::AcceptSamePeriod | FrameAction::AcceptNewPeriod => {}
                            }
                            
                            // Keep overall min/max for backward compatibility
                            min_time = Some(min_time.map_or(time, |min| min.min(time)));
                            max_time = Some(max_time.map_or(time, |max| max.max(time)));

                            // Also track iteration range as fallback
                            if let Some(iteration_idx) = temp_mappings.iteration_idx {
                                if let Some(iteration_value) = main_frame.get(iteration_idx) {
                                    let iteration = self.main_value_to_u64(&iteration_value);
                                    min_iteration = Some(
                                        min_iteration.map_or(iteration, |min| min.min(iteration)),
                                    );
                                    max_iteration = Some(
                                        max_iteration.map_or(iteration, |max| max.max(iteration)),
                                    );
                                }
                            }
                        }
                    }

                    // Finalize the best period (handle last period)
                    bf_time.finalize_at_end();
                    
                    // Calculate duration using the best continuous time period (Betaflight method)
                    let duration_s = if let (Some(best_start), Some(best_end)) = bf_time.best_period() {
                        // Use the longest continuous period (filters out setup/historical data)
                        let duration = (best_end - best_start) as f32 / 1_000_000.0;
                        debug!(
                            "Session {}: time-based duration = {:.1}s (best period: {} - {} = {} μs)",
                            idx + 1,
                            duration,
                            best_start,
                            best_end,
                            best_end - best_start
                        );
                        duration
                    } else if let (Some(min_t), Some(max_t)) = (min_time, max_time) {
                        // Fallback to overall range if no periods detected
                        let duration = (max_t - min_t) as f32 / 1_000_000.0;
                        debug!(
                            "Session {}: time-based duration = {:.1}s (overall range: {} - {} = {} μs)",
                            idx + 1,
                            duration,
                            min_t,
                            max_t,
                            max_t - min_t
                        );
                        duration
                    } else if let (Some(min_iter), Some(max_iter)) = (min_iteration, max_iteration) {
                        // Use PID rate: loopIteration increments at base_rate / pid_process_denom
                        let (intervals, _) = self.extract_frame_intervals_from_raw_data(data);
                        let base_rate = intervals.base_rate.unwrap_or(8000.0);
                        let pid_denom = self.extract_pid_process_denom(data).unwrap_or(1) as f32;
                        let iteration_range = (max_iter - min_iter) as f32;
                        let iteration_rate_hz = base_rate / pid_denom;
                        let duration = iteration_range / iteration_rate_hz;
                        debug!(
                            "Session {}: iteration-based duration (PID denom) = {:.1}s (range: {} - {} = {} cycles at {:.1}Hz / denom {:.0} = {:.1}Hz)",
                            idx + 1,
                            duration,
                            min_iter,
                            max_iter,
                            (max_iter - min_iter),
                            base_rate,
                            pid_denom,
                            iteration_rate_hz
                        );
                        duration
                    } else {
                        // Last resort: frame-based estimate
                        let fallback_rate = 1000.0;
                        let duration = main_frame_count as f32 / fallback_rate;
                        warn!(
                            "Session {}: no time/iteration data, using fallback duration = {:.1}s",
                            idx + 1,
                            duration
                        );
                        duration
                    };

                    sessions.push(SessionInfo {
                        index: idx,
                        main_frames: main_frame_count,
                        duration_estimate_s: duration_s,
                    });

                    info!(
                        "Session {}: {} main frames, {:.1}s duration",
                        idx + 1,
                        main_frame_count,
                        duration_s
                    );
                }
                Err(e) => {
                    warn!("Failed to parse session {}: {:?}", idx + 1, e);
                    // Continue processing other sessions
                }
            }
        }

        Ok(sessions)
    }

    /// Log detailed session breakdown for --list-sessions
    fn log_session_breakdown(&self, sessions: &[SessionInfo]) {
        info!("📁 Sessions in blackbox file:");

        let mut _total_frames = 0u64;
        let mut total_duration = 0.0f32;

        for session in sessions {
            _total_frames += session.main_frames;
            total_duration += session.duration_estimate_s;

            info!(
                "  Session {}: {:.1}s, {} samples",
                session.index + 1,
                session.duration_estimate_s,
                session.main_frames
            );
        }

        info!("");
        info!(
            "  Total: {:.1}s across {} sessions",
            total_duration,
            sessions.len()
        );
        info!("  Use --session N to analyze specific session");
    }

    /// Get parsing statistics
    pub fn stats(&self) -> &ParsingStats {
        &self.stats
    }

    /// Get concise BB sampling summary string, if available
    pub fn bb_summary(&self) -> Option<&str> {
        self.bb_summary.as_deref()
    }
}

impl Default for SimpleBlackboxParser {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_simple_parser_creation() {
        let parser = SimpleBlackboxParser::new();
        assert_eq!(parser.stats.total_frames, 0);
    }

    #[test]
    fn test_simple_parser_with_config() {
        let config = ParsingConfig {
            max_frames: Some(100),
            strict_parsing: true,
            ..Default::default()
        };
        let parser = SimpleBlackboxParser::with_config(config);
        assert_eq!(parser.config.max_frames, Some(100));
        assert!(parser.config.strict_parsing);
    }

    #[test]
    fn test_invalid_data_handling() {
        let mut parser = SimpleBlackboxParser::new();
        let invalid_data = b"This is not a blackbox file";
        let result = parser.parse_file(invalid_data);
        assert!(result.is_err());
    }

    #[test]
    fn test_header_parsing_edge_cases() {
        let parser = SimpleBlackboxParser::new();

        // Test malformed looptime header
        assert!(parser.parse_looptime_value("looptime").is_none());
        assert!(parser.parse_looptime_value("looptime:").is_none());
        assert!(parser.parse_looptime_value("looptime:abc").is_none());
        assert!(parser.parse_looptime_value("looptime:0").is_none());

        // Test valid looptime header (returns microseconds)
        assert_eq!(parser.parse_looptime_value("looptime:250").unwrap(), 250.0);

        // Test I interval parsing (returns cycles)
        assert_eq!(parser.parse_i_interval_value("I interval:32").unwrap(), 32);

        // Test pid_rate parsing
        assert_eq!(
            parser.parse_pid_rate_header("pid_rate:8000").unwrap(),
            8000.0
        );
    }

    #[test]
    fn test_empty_data_handling() {
        let mut parser = SimpleBlackboxParser::new();
        let result = parser.parse_file(&[]);
        assert!(result.is_err());
    }

    #[test]
    fn test_sample_rate_extraction_looptime() {
        let parser = SimpleBlackboxParser::new();

        // Test looptime header parsing (125μs = 8000Hz base, but no I interval)
        let test_data = b"H Product:Blackbox flight data recorder\nH looptime:125\nH Firmware revision:Betaflight 4.4.0\n";
        let sample_rate = parser.extract_sample_rate_from_raw_data(test_data);
        assert!(
            (sample_rate - 8000.0).abs() < 0.1,
            "Expected ~8000Hz base rate, got {}",
            sample_rate
        );
    }

    #[test]
    fn test_sample_rate_extraction_interval() {
        let parser = SimpleBlackboxParser::new();

        // Test combined looptime + I interval (125μs = 8000Hz base, with I=32 and P=1/1 all frames exist)
        let test_data = b"H Product:Blackbox flight data recorder\nH looptime:125\nH I interval:32\nH Firmware revision:Betaflight 4.4.0\n";
        let sample_rate = parser.extract_sample_rate_from_raw_data(test_data);
        // With P=1/1 (default), all frames exist, so effective rate = base rate = 8000Hz
        assert!(
            (sample_rate - 8000.0).abs() < 0.1,
            "Expected ~8000Hz effective rate, got {}",
            sample_rate
        );
    }

    #[test]
    fn test_sample_rate_extraction_pid_rate() {
        let parser = SimpleBlackboxParser::new();

        // Test pid_rate header parsing (direct frequency override)
        let test_data = b"H Product:Blackbox flight data recorder\nH pid_rate:4000\nH Firmware revision:Betaflight 4.4.0\n";
        let sample_rate = parser.extract_sample_rate_from_raw_data(test_data);
        assert!(
            (sample_rate - 4000.0).abs() < 0.1,
            "Expected ~4000Hz, got {}",
            sample_rate
        );
    }

    #[test]
    fn test_betaflight_style_sample_rate() {
        let parser = SimpleBlackboxParser::new();

        // Test Betaflight-style calculation: 8000Hz base with I=128 interval and P=1/1 (all frames)
        let test_data = b"H Product:Blackbox flight data recorder\nH looptime:125\nH I interval:128\nH Firmware revision:Betaflight 4.5.2\n";
        let sample_rate = parser.extract_sample_rate_from_raw_data(test_data);
        // With I=128 and P=1/1, all 128 frames exist: 128 frames / (128/8000)s = 8000Hz
        assert!(
            (sample_rate - 8000.0).abs() < 0.1,
            "Expected ~8000Hz effective rate, got {}",
            sample_rate
        );
    }

    #[test]
    fn test_sample_rate_fallback() {
        let parser = SimpleBlackboxParser::new();

        // Test fallback when no rate headers are present
        let test_data =
            b"H Product:Blackbox flight data recorder\nH Firmware revision:Betaflight 4.4.0\n";
        let sample_rate = parser.extract_sample_rate_from_raw_data(test_data);
        assert!(
            (sample_rate - 4000.0).abs() < 0.1,
            "Expected default 4000Hz, got {}",
            sample_rate
        );
    }

    #[test]
    fn test_pid_triple_parsing() {
        let parser = SimpleBlackboxParser::new();

        // Test valid PID triple
        let test_value = "61,110,41".to_string();
        let (p, i, d) = parser.parse_pid_triple(Some(&test_value));
        assert!((p - 61.0).abs() < 0.1, "Expected P=61, got {}", p);
        assert!((i - 110.0).abs() < 0.1, "Expected I=110, got {}", i);
        assert!((d - 41.0).abs() < 0.1, "Expected D=41, got {}", d);

        // Test missing value
        let (p, i, d) = parser.parse_pid_triple(None);
        assert!((p - 45.0).abs() < 0.1, "Expected default P=45, got {}", p);
        assert!((i - 90.0).abs() < 0.1, "Expected default I=90, got {}", i);
        assert!((d - 35.0).abs() < 0.1, "Expected default D=35, got {}", d);

        // Test malformed value
        let test_value = "invalid".to_string();
        let (p, _i, _d) = parser.parse_pid_triple(Some(&test_value));
        assert!(
            (p - 45.0).abs() < 0.1,
            "Expected default P=45 for invalid input, got {}",
            p
        );
    }

    #[test]
    fn test_frame_intervals_defaults() {
        let intervals = FrameIntervals::default();
        assert_eq!(intervals.frame_interval_i, 32);
        assert_eq!(intervals.frame_interval_p_num, 1);
        assert_eq!(intervals.frame_interval_p_denom, 1);
        assert!(intervals.base_rate.is_none());
    }

    #[test]
    fn test_should_have_frame_logic() {
        // Test Betaflight shouldHaveFrame logic exactly
        // Reference: ((frameIndex % frameIntervalI) + frameIntervalPNum - 1) % frameIntervalPDenom < frameIntervalPNum

        // Test default intervals (I=32, P=1/1)
        let intervals = FrameIntervals::default();

        // With P=1/1, ALL frames should exist because:
        // ((frameIndex % 32) + 1 - 1) % 1 = (frameIndex % 32) % 1 = 0 < 1 = true for any frameIndex
        assert!(intervals.should_have_frame(0));
        assert!(intervals.should_have_frame(1));
        assert!(intervals.should_have_frame(31));
        assert!(intervals.should_have_frame(32));
        assert!(intervals.should_have_frame(33));
        assert!(intervals.should_have_frame(64));

        // Test P-interval 1/4 case (more restrictive)
        let intervals_p4 = FrameIntervals {
            frame_interval_i: 32,
            frame_interval_p_num: 1,
            frame_interval_p_denom: 4,
            base_rate: None,
        };

        // Frame 0: ((0 % 32) + 1 - 1) % 4 = 0 % 4 = 0 < 1 = true
        assert!(intervals_p4.should_have_frame(0));
        // Frame 1: ((1 % 32) + 1 - 1) % 4 = 1 % 4 = 1 < 1 = false
        assert!(!intervals_p4.should_have_frame(1));
        // Frame 2: ((2 % 32) + 1 - 1) % 4 = 2 % 4 = 2 < 1 = false
        assert!(!intervals_p4.should_have_frame(2));
        // Frame 3: ((3 % 32) + 1 - 1) % 4 = 3 % 4 = 3 < 1 = false
        assert!(!intervals_p4.should_have_frame(3));
        // Frame 4: ((4 % 32) + 1 - 1) % 4 = 4 % 4 = 0 < 1 = true
        assert!(intervals_p4.should_have_frame(4));
        // Frame 32: ((32 % 32) + 1 - 1) % 4 = 0 % 4 = 0 < 1 = true
        assert!(intervals_p4.should_have_frame(32));
        // Frame 36: ((36 % 32) + 1 - 1) % 4 = (4 + 1 - 1) % 4 = 4 % 4 = 0 < 1 = true
        assert!(intervals_p4.should_have_frame(36));

        // Count frames in a cycle to verify pattern
        let count_32 = intervals_p4.count_expected_frames(0, 32);
        assert_eq!(
            count_32, 8,
            "Expected 8 frames in 32-frame cycle with P=1/4, got {}",
            count_32
        );
    }

    #[test]
    fn test_effective_rate_calculation() {
        // Test basic I-interval rate calculation (P=1/1 means every frame)
        let intervals = FrameIntervals {
            frame_interval_i: 32,
            frame_interval_p_num: 1,
            frame_interval_p_denom: 1,
            base_rate: None,
        };

        let effective_rate = intervals.calculate_effective_rate(8000.0);
        // With P=1/1, all 32 frames in the cycle exist: 32 frames / (32/8000)s = 8000Hz
        assert!(
            (effective_rate - 8000.0).abs() < 0.1,
            "Expected 8000Hz (all frames), got {}",
            effective_rate
        );

        // Test P-interval 1/4 effect (only every 4th frame exists)
        let intervals_p4 = FrameIntervals {
            frame_interval_i: 32,
            frame_interval_p_num: 1,
            frame_interval_p_denom: 4,
            base_rate: None,
        };

        let effective_rate_p4 = intervals_p4.calculate_effective_rate(8000.0);
        // With P=1/4, only 8 frames in the 32-frame cycle exist: 8 frames / (32/8000)s = 2000Hz
        assert!(
            (effective_rate_p4 - 2000.0).abs() < 0.1,
            "Expected 2000Hz (1/4 frames), got {}",
            effective_rate_p4
        );
    }

    #[test]
    fn test_p_interval_parsing() {
        let parser = SimpleBlackboxParser::new();

        // Test "1/4" format
        assert_eq!(
            parser.parse_p_interval_value("P interval:1/4").unwrap(),
            (1, 4)
        );

        // Test "4" format (implies 1/4)
        assert_eq!(
            parser.parse_p_interval_value("P interval:4").unwrap(),
            (1, 4)
        );

        // Test "2/8" format
        assert_eq!(
            parser.parse_p_interval_value("P interval:2/8").unwrap(),
            (2, 8)
        );

        // Test invalid formats
        assert!(parser.parse_p_interval_value("P interval:0/4").is_none());
        assert!(parser.parse_p_interval_value("P interval:1/0").is_none());
        assert!(parser.parse_p_interval_value("P interval:abc").is_none());
    }

    #[test]
    fn test_frame_interval_extraction() {
        let parser = SimpleBlackboxParser::new();

        // Test complete Betaflight-style headers
        let test_data = b"H Product:Blackbox flight data recorder\nH looptime:125\nH I interval:32\nH P interval:1/4\nH Firmware revision:Betaflight 4.4.0\n";
        let (intervals, found_intervals) = parser.extract_frame_intervals_from_raw_data(test_data);

        assert!(found_intervals, "Should have found interval configuration");

        assert_eq!(intervals.frame_interval_i, 32);
        assert_eq!(intervals.frame_interval_p_num, 1);
        assert_eq!(intervals.frame_interval_p_denom, 4);
        assert!((intervals.base_rate.unwrap() - 8000.0).abs() < 0.1);

        // Calculate effective rate using proper frame counting logic
        // With P=1/4, we get 8 out of every 32 frames: 8 frames / (32/8000)s = 2000Hz
        let effective = intervals.calculate_effective_rate(intervals.base_rate.unwrap());
        assert!(
            (effective - 2000.0).abs() < 0.1,
            "Expected 2000Hz, got {}",
            effective
        );

        // Verify this gives reasonable flight durations
        // For example: 30,000 frames at 2000Hz = 15 seconds (realistic short flight)
        let frames = 30_000u64;
        let duration_s = frames as f32 / effective;
        assert!(
            (duration_s - 15.0).abs() < 0.1,
            "Expected 15s duration for 30k frames, got {:.1}s",
            duration_s
        );
    }

    #[test]
    fn test_frame_counting_with_p_interval() {
        // Test that frame counting works correctly for P-interval 1/4
        let intervals_p4 = FrameIntervals {
            frame_interval_i: 32,
            frame_interval_p_num: 1,
            frame_interval_p_denom: 4,
            base_rate: None,
        };

        // Count frames in one full cycle (32 frames)
        let frames_in_cycle = intervals_p4.count_expected_frames(0, 32);

        // With P=1/4, we expect every 4th frame: 0, 4, 8, 12, 16, 20, 24, 28 = 8 frames
        assert_eq!(
            frames_in_cycle, 8,
            "Expected 8 frames out of 32 with P=1/4, got {}",
            frames_in_cycle
        );

        // Verify the effective rate calculation
        let effective_rate = intervals_p4.calculate_effective_rate(8000.0);
        // 8 frames / (32/8000)s = 8 / 0.004s = 2000Hz
        assert!(
            (effective_rate - 2000.0).abs() < 0.1,
            "Expected 2000Hz, got {}",
            effective_rate
        );
    }

    #[test]
    fn test_realistic_duration_calculation() {
        let parser = SimpleBlackboxParser::new();

        // Test realistic blackbox scenario:
        // - 8kHz base rate (125μs looptime)
        // - I interval = 128 (common for data logging)
        // - P interval = 1/4 (reduces data volume)
        // Expected effective rate: count frames in 128-frame cycle
        let test_data = b"H Product:Blackbox flight data recorder\nH looptime:125\nH I interval:128\nH P interval:1/4\nH Firmware revision:Betaflight 4.4.0\n";
        let sample_rate = parser.extract_sample_rate_from_raw_data(test_data);

        // With the new logic: count frames in 128-frame cycle
        let (intervals, _) = parser.extract_frame_intervals_from_raw_data(test_data);
        let frames_in_cycle = intervals.count_expected_frames(0, 128);

        // P=1/4 means every 4th frame: 0, 4, 8, 12, ... 124 = 32 frames out of 128
        assert_eq!(
            frames_in_cycle, 32,
            "Expected 32 frames out of 128 with P=1/4, got {}",
            frames_in_cycle
        );

        // Duration calculation: 32 frames / (128/8000)s = 32 / 0.016s = 2000Hz
        let expected_rate = 32.0 / (128.0 / 8000.0);
        assert!(
            (sample_rate - expected_rate).abs() < 0.1,
            "Expected {:.1}Hz, got {:.1}Hz",
            expected_rate,
            sample_rate
        );

        // For a typical 3-minute flight:
        // - 180 seconds × 2000Hz = 360,000 samples
        // - This is much more reasonable than the previous 30+ minute calculations!
        let flight_duration_s = 180.0; // 3 minutes
        let expected_samples = flight_duration_s * sample_rate;
        assert!(
            (expected_samples - 360_000.0).abs() < 1.0,
            "Expected ~360k samples for 3min flight at {:.1}Hz, got {:.0}",
            sample_rate,
            expected_samples
        );
    }

    #[test]
    fn test_configuration_extraction() {
        let parser = SimpleBlackboxParser::new();

        // Create a test header map with some configuration data
        let mut headers = std::collections::HashMap::new();
        headers.insert("rollPID".to_string(), "61,110,41".to_string());
        headers.insert("pitchPID".to_string(), "67,121,51".to_string());
        headers.insert("yawPID".to_string(), "61,110,0".to_string());
        headers.insert("d_min".to_string(), "41,51,0".to_string());
        headers.insert("rc_rates".to_string(), "4,4,4".to_string());
        headers.insert("dterm_lpf1_static_hz".to_string(), "75".to_string());
        headers.insert("gyro_lpf2_static_hz".to_string(), "500".to_string());
        headers.insert("dyn_notch_max_hz".to_string(), "800".to_string());

        // Test PID configuration extraction
        let pid_config = parser.extract_pid_configuration(&headers);
        assert!(
            (pid_config.roll.p - 61.0).abs() < 0.1,
            "Expected roll P=61, got {}",
            pid_config.roll.p
        );
        assert!(
            (pid_config.pitch.p - 67.0).abs() < 0.1,
            "Expected pitch P=67, got {}",
            pid_config.pitch.p
        );
        assert!(
            (pid_config.yaw.p - 61.0).abs() < 0.1,
            "Expected yaw P=61, got {}",
            pid_config.yaw.p
        );

        // Test that D-min values are used when larger than D values
        assert!(
            (pid_config.roll.d - 41.0).abs() < 0.1,
            "Expected roll D=41 (d_min), got {}",
            pid_config.roll.d
        );
        assert!(
            (pid_config.pitch.d - 51.0).abs() < 0.1,
            "Expected pitch D=51 (d_min), got {}",
            pid_config.pitch.d
        );

        // Test filter configuration extraction
        let filter_config = parser.extract_filter_configuration(&headers);

        // Check that we extracted the gyro filter
        assert!(
            !filter_config.gyro_filters.is_empty(),
            "Expected at least one gyro filter"
        );
        let gyro_filter = &filter_config.gyro_filters[0];
        assert!(
            (gyro_filter.cutoff - 500.0).abs() < 0.1,
            "Expected gyro filter cutoff=500Hz, got {}",
            gyro_filter.cutoff
        );

        // Check that we extracted the D-term filter
        assert!(
            !filter_config.dterm_filters.is_empty(),
            "Expected at least one D-term filter"
        );
        let dterm_filter = &filter_config.dterm_filters[0];
        assert!(
            (dterm_filter.cutoff - 75.0).abs() < 0.1,
            "Expected D-term filter cutoff=75Hz, got {}",
            dterm_filter.cutoff
        );

        // Check dynamic notch configuration
        assert!(
            filter_config.dynamic_notch.is_some(),
            "Expected dynamic notch configuration"
        );
        let dyn_notch = filter_config.dynamic_notch.unwrap();
        assert!(
            (dyn_notch.max_freq - 800.0).abs() < 0.1,
            "Expected dynamic notch max=800Hz, got {}",
            dyn_notch.max_freq
        );
        assert!(dyn_notch.enabled, "Expected dynamic notch to be enabled");
    }
}
