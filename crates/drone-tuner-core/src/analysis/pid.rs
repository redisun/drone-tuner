//! PID step-response and gyro-characteristic analysis.
//!
//! Used by [`super::AnalysisEngine`] to produce per-axis PID recommendations.
//! Falls back to gyro-only heuristics when RC command data is unavailable.

use crate::domain::{Axis, PidConfiguration, PidRecommendation, PidTerm, Priority, TelemetryData};
use crate::error::{DronetunerError, Result};
use serde::{Deserialize, Serialize};

/// Analyses telemetry to derive PID gain recommendations.
pub(super) struct PidAnalyzer {
    config: PidAnalyzerConfig,
}

/// What [`PidAnalyzer::analyze`] hands back: both the actionable
/// recommendations and the underlying step-response observations they
/// were derived from. Surfacing the steps lets callers (CLI, future GUI)
/// show the analyser's reasoning instead of just its conclusions.
#[derive(Debug, Clone)]
pub struct PidAnalysisOutcome {
    /// Actionable PID gain changes.
    pub recommendations: Vec<PidRecommendation>,
    /// Step responses detected on each axis. Empty when the log has no
    /// RC command data (gyro-only path) or the pilot didn't move the
    /// stick enough to trigger detection.
    pub step_responses: Vec<StepResponse>,
}

/// Configuration for PID analysis
#[derive(Debug, Clone)]
pub struct PidAnalyzerConfig {
    /// Acceptable error threshold
    pub error_threshold: f32,
    /// Response time analysis window (seconds)
    pub response_window_s: f32,
    /// Overshoot tolerance (percentage)
    pub overshoot_tolerance: f32,
}

/// Represents a detected step response in the control system
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StepResponse {
    /// Which axis this response occurred on
    pub axis: Axis,
    /// Time when the step input occurred (seconds)
    pub start_time: f32,
    /// Magnitude of the command change
    pub command_magnitude: f32,
    /// Rise time (10% to 90% of final value)
    pub rise_time: f32,
    /// Settling time (time to stay within 2% of final value)
    pub settling_time: f32,
    /// Overshoot as percentage of final value
    pub overshoot_percent: f32,
    /// Dominant oscillation frequency in the response
    pub oscillation_frequency: f32,
    /// Estimated damping ratio
    pub damping_ratio: f32,
    /// Absolute steady-state tracking error in deg/s, when the step was
    /// large enough and the command stayed put long enough to make the
    /// metric meaningful. `None` for transient stick movements where
    /// "steady state" was never reached.
    pub steady_state_error_dps: Option<f32>,
    /// Sample rate of the underlying log (Hz). Carried alongside the
    /// trace so renderers can reconstruct the time axis.
    pub sample_rate: f32,
    /// Gyro response over the analysis window, starting at the step.
    /// Units: deg/s. Length matches `command_trace`.
    pub gyro_trace: Vec<f32>,
    /// RC command over the analysis window, normalised to ~[-1.0, 1.0].
    /// The pre-step plateau is implicit in the value at index 0.
    pub command_trace: Vec<f32>,
}

/// Step response performance metrics
#[derive(Debug, Clone)]
struct StepMetrics {
    rise_time: f32,
    settling_time: f32,
    overshoot_percent: f32,
    oscillation_frequency: f32,
    damping_ratio: f32,
    steady_state_error_dps: Option<f32>,
}

/// Hard upper bounds for PID gains. The analyzer never recommends values
/// above these and stops recommending increases once the current value is
/// within `headroom_skip_pct` of the cap. Tuned for modern Betaflight 4.x
/// on a 4S-6S 5" freestyle quad — adjust for racing or tinywhoops.
#[derive(Debug, Clone)]
struct PidLimits {
    p_max: f32,
    i_max: f32,
    d_max: f32,
    /// Skip recommending an increase if `current >= max * (1 - headroom_skip_pct)`.
    /// Prevents asymptotic recommendations that nudge gains by less than the
    /// FC's integer resolution.
    headroom_skip_pct: f32,
}

impl Default for PidLimits {
    fn default() -> Self {
        Self {
            p_max: 80.0,
            i_max: 180.0,
            d_max: 60.0,
            headroom_skip_pct: 0.10,
        }
    }
}

/// Minimum stick deflection (fraction of full RC range, [-1.0, 1.0]) for a
/// step to count as steady-state-capable. Below this the gyro never reaches
/// a sustainable rate and "steady-state error" is meaningless.
const SS_CAPABLE_MIN_STEP: f32 = 0.30;
/// Maximum allowed RC command drift across the analysis window for the
/// step to count as steady-state-capable. If the pilot moved the stick
/// again before the response settled, we can't tell command-tracking error
/// from a fresh transient.
const SS_CAPABLE_MAX_DRIFT: f32 = 0.10;
/// Minimum number of steady-state-capable responses we need before we
/// trust an averaged steady-state error figure enough to act on it.
const SS_MIN_VALID_RESPONSES: usize = 3;
/// Threshold in deg/s above which we recommend bumping I-term. 30 deg/s
/// is roughly 4-5% of typical full-stick rate (~600-700 deg/s) — small
/// enough to catch real bias, large enough to ignore measurement noise.
const SS_ERROR_RECOMMEND_DPS: f32 = 30.0;
// ---------- step-size guardrails (Item 2 of algorithm-improvements.md) ----------
//
// Each tune iteration changes a single gain by at most these fractions.
// The previous values (P-cut up to 30%, D-bump 30%, D-cut 20%, I-bump 10%)
// compounded into runaway across multiple `tune --apply-all` iterations:
// MEDIUM FUCKER drifted Roll D 28→58 (+107%) and Pitch P 51→29 (−43%) in
// four iterations until unflyable. See docs/algorithm-improvements.md for
// the full incident analysis and the trajectory of pre-runaway backups.
//
// Conservative step sizes mean each iteration's effect is visible in the
// next bbl without overshooting. They also widen the safety margin against
// the *categorical* failure modes (Item 1, Item 3, Item 5) — even if the
// router picks the wrong gain, an 8% nudge is recoverable; a 30% nudge is
// not. Tune cautiously, iterate often.

/// Maximum P-term reduction per recommendation. Cap on the overshoot path,
/// the std-dev path, and the band-routed P-cut path. Set to 15% — large
/// enough to make a felt difference in one iteration, small enough that
/// two consecutive misroutes don't destroy the tune.
const MAX_P_CUT_PCT: f32 = 0.15;

/// P-term increase per recommendation (slow-rise / under-responsive case).
const P_BUMP_PCT: f32 = 0.08;

/// D-term step (both bump and cut). The pre-Item-2 30% bump was the load-
/// bearing arithmetic in the MEDIUM FUCKER runaway: D 28 → 28×1.3 = 36 →
/// 36×1.3 = 47 → 47×1.3 = 61 (clamped to 60). Held to 8%, the same chain
/// is 28 → 30 → 33 → 35: still corrective, but recoverable.
const D_STEP_PCT: f32 = 0.08;

/// I-term increase per recommendation. Previously 10% (was 20% before that).
const I_BUMP_PCT: f32 = 0.08;

/// Convenience multipliers derived from the percentages above. Kept as
/// constants so call sites read naturally and tests have a single source
/// of truth to assert against.
const P_CUT_MAX_FACTOR: f32 = 1.0 - MAX_P_CUT_PCT; // 0.85
const P_BUMP_FACTOR: f32 = 1.0 + P_BUMP_PCT; // 1.08
const D_BUMP_FACTOR: f32 = 1.0 + D_STEP_PCT; // 1.08
const D_CUT_FACTOR: f32 = 1.0 - D_STEP_PCT; // 0.92

/// Backwards-compat alias for the I-term multiplier. Kept (rather than
/// replacing every call site) so the diff stays focused on the magnitude
/// change, not the rename.
const I_TERM_BUMP: f32 = 1.0 + I_BUMP_PCT;

/// Item 4: maximum drift any single recommendation is allowed to push a
/// gain *away* from its baseline value. The previous regime bounded
/// recommendations vs. the *current* PID, which let gains drift across
/// many iterations because the bound moved with each accepted change.
/// MEDIUM FUCKER's roll P 47 → 33 was a cumulative −30% from baseline,
/// but only 30% on the iteration that made it: invisible to the per-step
/// cap. Anchored at ±15% of *baseline*, the chain stops at 47 → 39.95.
pub const BASELINE_BOUND_PCT: f32 = 0.15;

/// Oscillation-frequency bands that drive the PID recommendation routing.
/// The rule of thumb (PIDtoolbox / plasmatree / BetaFlight conventions) is
/// that the *frequency* of an oscillation tells you which gain to touch:
///
/// - 5–15 Hz: P-too-high overshoot ringing — phase margin collapses near
///   the closed-loop bandwidth and the system rings. Fix: **reduce P**.
/// - 15–30 Hz: borderline; commonly an I-term or rate-config interaction.
///   We don't auto-recommend in this band — too many false positives.
/// - 30–80 Hz: classic D-too-low band. Phase lag from gyro+filter chain
///   eats the damping margin. Fix: **bump D** (or look at D-LPF cutoff).
/// - >80 Hz: filter / motor-noise / mechanical territory. Touching PID
///   here is the bug that caused MEDIUM FUCKER's runaway — the analyzer
///   used to bump D for *any* sub-Nyquist oscillation with low damping.
///   The proper fix is a filter recommendation; we suppress here and let
///   the FilterOptimizer pass handle it.
///
/// These boundaries are deliberately conservative: real oscillations
/// rarely live exactly at a band edge, and false routing is worse than
/// no recommendation. Tune in tests with the MEDIUM FUCKER 11.9 Hz hover
/// case as the canonical regression example.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum OscillationBand {
    /// f < 5 Hz — subharmonic / drift / breathing. Ignore.
    Subharmonic,
    /// 5 ≤ f < 15 Hz — P-too-high overshoot ringing.
    PTooHigh,
    /// 15 ≤ f < 30 Hz — ambiguous (I-term / phase-margin). Suppress.
    Ambiguous,
    /// 30 ≤ f < 80 Hz — D-too-low or D-LPF cutoff too low.
    DTooLow,
    /// f ≥ 80 Hz — filter / mechanical noise. Not a PID problem.
    Noise,
}

const BAND_PTOOHIGH_LOWER_HZ: f32 = 5.0;
const BAND_PTOOHIGH_UPPER_HZ: f32 = 15.0;
const BAND_DTOOLOW_LOWER_HZ: f32 = 30.0;
const BAND_DTOOLOW_UPPER_HZ: f32 = 80.0;

fn classify_oscillation_band(freq_hz: f32) -> OscillationBand {
    if !freq_hz.is_finite() || freq_hz < BAND_PTOOHIGH_LOWER_HZ {
        OscillationBand::Subharmonic
    } else if freq_hz < BAND_PTOOHIGH_UPPER_HZ {
        OscillationBand::PTooHigh
    } else if freq_hz < BAND_DTOOLOW_LOWER_HZ {
        OscillationBand::Ambiguous
    } else if freq_hz < BAND_DTOOLOW_UPPER_HZ {
        OscillationBand::DTooLow
    } else {
        OscillationBand::Noise
    }
}

/// Map [`Priority`] to a numeric rank for sort comparisons. Higher = more
/// important. The enum's `Ord` derive can't be used directly because the
/// declaration order (Low → Critical) matches semantic order, but we want
/// to be explicit about the mapping rather than depend on declaration
/// order being preserved.
fn priority_rank(p: &Priority) -> u8 {
    match p {
        Priority::Critical => 4,
        Priority::High => 3,
        Priority::Medium => 2,
        Priority::Low => 1,
    }
}

/// Map [`PidTerm`] to a tiebreaker rank when two recs on the same axis
/// share priority *and* direction. P > D > I because P sets the closed-
/// loop bandwidth — if it's wrong, every other gain is being tuned
/// against an incorrect response shape. F is highest only because it
/// shouldn't be appearing here at all (and we want it to dominate so a
/// stray F-term recommendation is obvious in tests).
fn term_rank(t: &PidTerm) -> u8 {
    match t {
        PidTerm::F => 4,
        PidTerm::P => 3,
        PidTerm::D => 2,
        PidTerm::I => 1,
    }
}

/// Item 3: collapse a per-axis multi-term recommendation list to at most
/// one rec per axis. Selection priority, in order:
///
/// 1. Higher [`Priority`] wins (Critical > High > Medium > Low).
/// 2. Among ties, prefer **cuts** (`recommended < current`) over **bumps**:
///    a reduction is the safer direction when in doubt — the system can
///    only get less aggressive, never more.
/// 3. Among further ties, prefer the term that sets the response shape:
///    P > D > I.
///
/// Reasoning for the cap: applying a P-cut, an I-bump, and a D-bump on
/// the same axis in a single pass shifts dynamics in three directions
/// simultaneously, which makes the *next* analysis's diagnosis unreliable
/// — its model assumes a step-response shape that may not exist anymore.
/// Better to take the most important fix and re-analyse.
fn collapse_recs_per_axis(recs: Vec<PidRecommendation>) -> Vec<PidRecommendation> {
    use std::collections::HashMap;
    let mut best: HashMap<Axis, PidRecommendation> = HashMap::new();
    for rec in recs {
        let key = rec.axis.clone();
        match best.get(&key) {
            None => {
                best.insert(key, rec);
            }
            Some(current) if rec_outranks(&rec, current) => {
                best.insert(key, rec);
            }
            Some(_) => {} // existing pick wins
        }
    }
    // Stable axis ordering for downstream consumers (matches
    // Roll → Pitch → Yaw display convention).
    let mut out: Vec<PidRecommendation> = best.into_values().collect();
    out.sort_by_key(|r| match r.axis {
        Axis::Roll => 0,
        Axis::Pitch => 1,
        Axis::Yaw => 2,
    });
    out
}

/// Item 4: clamp recommendations against a baseline PID configuration.
///
/// Each rec's `recommended_value` is clamped so it stays within
/// `baseline ± max(BASELINE_BOUND_PCT * baseline, |current - baseline|)`.
/// The `max` admits an "anti-drift envelope": once a craft has drifted
/// outside the baseline bound (because some prior iteration moved it
/// there, or the user manually set unusual gains), the analyzer can
/// still recommend further moves, but only *toward* baseline — never
/// farther away.
///
/// When clamping would reverse the rec's intended direction (i.e. the
/// rec wanted to go down, but the only legal move is up), the rec is
/// dropped — that signals the model has already overshot the safe
/// envelope and the right move is to stop, not negate.
///
/// Returns the clamped list, with dropped recs removed. Recs whose term
/// is `F` are passed through unchanged: the analyzer never emits them,
/// and the schema doesn't carry baseline F values.
pub fn clamp_recs_to_baseline(
    recs: Vec<PidRecommendation>,
    baseline: &PidConfiguration,
) -> Vec<PidRecommendation> {
    recs.into_iter()
        .filter_map(|mut rec| {
            let baseline_value = match (&rec.axis, &rec.term) {
                (Axis::Roll, PidTerm::P) => baseline.roll.p,
                (Axis::Roll, PidTerm::I) => baseline.roll.i,
                (Axis::Roll, PidTerm::D) => baseline.roll.d,
                (Axis::Pitch, PidTerm::P) => baseline.pitch.p,
                (Axis::Pitch, PidTerm::I) => baseline.pitch.i,
                (Axis::Pitch, PidTerm::D) => baseline.pitch.d,
                (Axis::Yaw, PidTerm::P) => baseline.yaw.p,
                (Axis::Yaw, PidTerm::I) => baseline.yaw.i,
                (Axis::Yaw, PidTerm::D) => baseline.yaw.d,
                (_, PidTerm::F) => return Some(rec), // F has no baseline anchor
            };

            // A baseline of 0 (e.g. yaw D, which is conventionally 0)
            // can't be sensibly clamped — anchoring 0 ± 15% × 0 = 0
            // would forbid any change. Pass through.
            if baseline_value <= f32::EPSILON {
                return Some(rec);
            }

            match clamp_value_to_baseline(rec.recommended_value, rec.current_value, baseline_value)
            {
                ClampOutcome::Allow(v) => {
                    if (v - rec.recommended_value).abs() > f32::EPSILON {
                        rec.reason = format!(
                            "{} (clamped to ±{:.0}% of baseline {:.1})",
                            rec.reason,
                            BASELINE_BOUND_PCT * 100.0,
                            baseline_value
                        );
                    }
                    rec.recommended_value = v;
                    Some(rec)
                }
                ClampOutcome::Drop => {
                    tracing::info!(
                        "Dropping {:?} {:?} rec ({}→{}): would push gain outside ±{:.0}% envelope of baseline {}",
                        rec.axis,
                        rec.term,
                        rec.current_value,
                        rec.recommended_value,
                        BASELINE_BOUND_PCT * 100.0,
                        baseline_value
                    );
                    None
                }
            }
        })
        .collect()
}

enum ClampOutcome {
    Allow(f32),
    Drop,
}

fn clamp_value_to_baseline(recommended: f32, current: f32, baseline: f32) -> ClampOutcome {
    let bound_radius = baseline * BASELINE_BOUND_PCT;
    let d_cur = (current - baseline).abs();
    // Allowed envelope half-width: at least the bound radius, but expand
    // to enclose `current` if the craft has already drifted outside.
    let cap = bound_radius.max(d_cur);
    let d_new = (recommended - baseline).abs();
    if d_new <= cap {
        return ClampOutcome::Allow(recommended);
    }
    // Recommendation is outside the envelope. Clamp toward baseline.
    let clamped = if recommended > baseline {
        baseline + cap
    } else {
        baseline - cap
    };
    let intended_dir = (recommended - current).signum();
    let clamped_dir = (clamped - current).signum();
    // Drop the rec if clamping reverses intent (rec wanted to go up, the
    // envelope only allows going down) OR if clamping produces a write
    // that's a no-op once rounded to FC integer units (the rec's intent
    // was real but the envelope says "you've already gone too far").
    if intended_dir != 0.0 && clamped_dir != 0.0 && intended_dir != clamped_dir {
        return ClampOutcome::Drop;
    }
    if (clamped - current).abs() < 1.0 {
        return ClampOutcome::Drop;
    }
    ClampOutcome::Allow(clamped)
}

/// Returns true when `candidate` should replace `incumbent` in the
/// per-axis collapse. See [`collapse_recs_per_axis`] for the full rule.
fn rec_outranks(candidate: &PidRecommendation, incumbent: &PidRecommendation) -> bool {
    let cand_pri = priority_rank(&candidate.priority);
    let inc_pri = priority_rank(&incumbent.priority);
    if cand_pri != inc_pri {
        return cand_pri > inc_pri;
    }
    let cand_is_cut = candidate.recommended_value < candidate.current_value;
    let inc_is_cut = incumbent.recommended_value < incumbent.current_value;
    if cand_is_cut != inc_is_cut {
        return cand_is_cut; // cut beats bump
    }
    term_rank(&candidate.term) > term_rank(&incumbent.term)
}

/// Minimum absolute scale at which we treat an `rc_commands` axis as
/// having meaningful range. Below this we assume the pilot didn't touch
/// the stick during the log and skip step analysis for that axis.
const RC_NORMALIZE_MIN_SCALE: f32 = 5.0;
/// Default Betaflight rcCommand range. Roll/pitch/yaw are signed ints
/// scaled to roughly +/- 500 (post-deadband, pre-rates). We use the
/// observed maximum absolute value in the log as the scale instead of
/// hard-coding 500, so the analyzer also works on logs that already came
/// in normalized — see `normalize_rc_axis`.
const RC_NORMALIZE_FALLBACK: f32 = 500.0;

/// Normalize one axis of rc_commands to roughly [-1.0, 1.0] using the
/// 99th-percentile absolute value as the scale. Returns the scale used so
/// callers can sanity-check it. Robust to the two conventions we see in
/// the wild: raw Betaflight rcCommand ([-500, 500]) and pre-normalized
/// floats ([-1, 1]).
fn normalize_rc_axis(rc: &[f32]) -> (Vec<f32>, f32) {
    if rc.is_empty() {
        return (Vec::new(), 1.0);
    }
    let mut abs_vals: Vec<f32> = rc.iter().map(|v| v.abs()).collect();
    // Partial sort by ordered_cmp so NaNs don't poison the sort.
    abs_vals.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let p99_idx = ((abs_vals.len() as f32) * 0.99) as usize;
    let p99 = abs_vals
        .get(p99_idx.min(abs_vals.len().saturating_sub(1)))
        .copied()
        .unwrap_or(0.0);
    // If the log barely moves the stick we don't have signal — fall back
    // to the Betaflight scale so step detection doesn't divide by ~0.
    let scale = if p99 < RC_NORMALIZE_MIN_SCALE {
        RC_NORMALIZE_FALLBACK
    } else {
        p99
    };
    let normalized = rc.iter().map(|v| v / scale).collect();
    (normalized, scale)
}

impl PidAnalyzer {
    pub(super) fn new() -> Self {
        Self {
            config: PidAnalyzerConfig::default(),
        }
    }

    pub(super) fn analyze(
        &self,
        telemetry: &TelemetryData,
        pid_config: &PidConfiguration,
    ) -> Result<PidAnalysisOutcome> {
        let mut recommendations = Vec::new();
        let mut step_responses: Vec<StepResponse> = Vec::new();

        // Check if we have RC command data
        if telemetry.rc_commands.roll.is_empty() {
            tracing::warn!("No RC command data available, performing gyro-only analysis");
            // Perform gyro-only analysis
            recommendations.extend(self.analyze_gyro_characteristics(telemetry, pid_config)?);
        } else {
            tracing::info!("RC command data available, performing step response analysis");

            // Normalize rc_commands to ~[-1, 1] before step detection so
            // thresholds (min step, max drift, SS-capable cutoff) are unit-
            // independent of whether the bbl came in raw Betaflight units
            // ([-500, 500]) or already normalized.
            let (roll_rc, roll_scale) = normalize_rc_axis(&telemetry.rc_commands.roll);
            let (pitch_rc, pitch_scale) = normalize_rc_axis(&telemetry.rc_commands.pitch);
            let (yaw_rc, yaw_scale) = normalize_rc_axis(&telemetry.rc_commands.yaw);
            tracing::debug!(
                "RC normalization scales: roll={:.1} pitch={:.1} yaw={:.1}",
                roll_scale,
                pitch_scale,
                yaw_scale
            );

            // Detect step responses for each axis
            let roll_responses = self.detect_step_responses(
                &roll_rc,
                &telemetry.gyro.x,
                telemetry.sample_rate,
                Axis::Roll,
            )?;

            let pitch_responses = self.detect_step_responses(
                &pitch_rc,
                &telemetry.gyro.y,
                telemetry.sample_rate,
                Axis::Pitch,
            )?;

            let yaw_responses = self.detect_step_responses(
                &yaw_rc,
                &telemetry.gyro.z,
                telemetry.sample_rate,
                Axis::Yaw,
            )?;

            // Analyze each axis and generate recommendations
            recommendations.extend(self.analyze_axis_responses(
                &roll_responses,
                Axis::Roll,
                pid_config,
            )?);
            recommendations.extend(self.analyze_axis_responses(
                &pitch_responses,
                Axis::Pitch,
                pid_config,
            )?);
            recommendations.extend(self.analyze_axis_responses(
                &yaw_responses,
                Axis::Yaw,
                pid_config,
            )?);

            tracing::info!(
                "PID analysis found {} step responses: {} roll, {} pitch, {} yaw",
                roll_responses.len() + pitch_responses.len() + yaw_responses.len(),
                roll_responses.len(),
                pitch_responses.len(),
                yaw_responses.len()
            );

            step_responses.extend(roll_responses);
            step_responses.extend(pitch_responses);
            step_responses.extend(yaw_responses);
        }

        // Check if we have PID error data for additional analysis
        if !telemetry.pid_error.roll.is_empty() {
            tracing::info!("PID error data available, performing error analysis");
            recommendations.extend(self.analyze_pid_errors(telemetry, pid_config)?);
        }

        // Item 3: collapse to at most one recommendation per axis. The
        // pre-collapse list can carry P, I, *and* D recs for the same axis
        // (overshoot path → P-cut, SS-error path → I-bump, band-routed
        // path → D-bump), and applying all three in one pass invalidates
        // the next analysis's model — the FC's dynamics shifted in three
        // independent directions at once. Take the highest-priority single
        // change per axis and let the next iteration handle the rest.
        let recommendations = collapse_recs_per_axis(recommendations);

        Ok(PidAnalysisOutcome {
            recommendations,
            step_responses,
        })
    }

    /// Detect step responses in RC command and gyro data
    fn detect_step_responses(
        &self,
        rc_commands: &[f32],
        gyro_response: &[f32],
        sample_rate: f32,
        axis: Axis,
    ) -> Result<Vec<StepResponse>> {
        let mut responses = Vec::new();

        if rc_commands.len() != gyro_response.len() {
            return Err(DronetunerError::analysis_error(
                "RC command and gyro data length mismatch",
            ));
        }

        // Real pilot stick movements happen over ~50-200ms. At 3205Hz that's
        // hundreds of samples — a single-sample delta detector would never
        // fire on a gradual stick move and instead picks up only quantization
        // noise. Use a windowed detector: at each position, look at the RC
        // change between `pre_window` ago and `post_window` ahead. If the
        // pre-window was steady, the post-window is steady, and the gap
        // between them is large, it's a real step.
        let pre_window = ((0.05 * sample_rate) as usize).max(5); // 50ms history
        let post_window = ((0.10 * sample_rate) as usize).max(10); // 100ms forward
        let refractory = ((0.30 * sample_rate) as usize).max(30); // 300ms cooldown
        let min_step_size = 0.10; // 10% of full stick deflection
        let max_pre_jitter = 0.05; // pre-step stability tolerance
        let max_post_jitter = 0.10; // post-step settle tolerance

        // Sliding mean+range over a window — cheap stability check.
        let window_range = |start: usize, end: usize| -> (f32, f32) {
            let slice = &rc_commands[start..end];
            let mean = slice.iter().sum::<f32>() / slice.len() as f32;
            let max_dev = slice
                .iter()
                .map(|v| (v - mean).abs())
                .fold(0.0_f32, f32::max);
            (mean, max_dev)
        };

        let mut last_step_end = 0;
        let mut i = pre_window;
        while i + post_window < rc_commands.len() {
            // Skip until we're past the previous step's refractory period.
            if i < last_step_end {
                i += 1;
                continue;
            }

            let (pre_mean, pre_jitter) = window_range(i - pre_window, i);
            let (post_mean, post_jitter) = window_range(i, i + post_window);
            let step_size = (post_mean - pre_mean).abs();

            if step_size > min_step_size
                && pre_jitter < max_pre_jitter
                && post_jitter < max_post_jitter
            {
                let response = self.analyze_step_response(
                    i,
                    i + post_window,
                    rc_commands,
                    gyro_response,
                    sample_rate,
                    axis.clone(),
                )?;
                if let Some(resp) = response {
                    responses.push(resp);
                }
                last_step_end = i + refractory;
                i += refractory;
            } else {
                i += 1;
            }
        }

        Ok(responses)
    }

    /// Analyze a single step response and extract performance metrics
    fn analyze_step_response(
        &self,
        step_start: usize,
        _step_end: usize,
        rc_commands: &[f32],
        gyro_response: &[f32],
        sample_rate: f32,
        axis: Axis,
    ) -> Result<Option<StepResponse>> {
        // Recompute the windowed pre/post means so the step magnitude here
        // matches what the detector saw (single-sample deltas would pick
        // up quantization noise instead of the actual stick movement).
        let pre_w = ((0.05 * sample_rate) as usize).max(5).min(step_start);
        let post_w = ((0.10 * sample_rate) as usize).max(10);
        let pre_slice = &rc_commands[step_start - pre_w..step_start];
        let post_end = (step_start + post_w).min(rc_commands.len());
        let post_slice = &rc_commands[step_start..post_end];
        let initial_command = pre_slice.iter().sum::<f32>() / pre_slice.len() as f32;
        let step_command = post_slice.iter().sum::<f32>() / post_slice.len() as f32;
        let command_change = step_command - initial_command;

        // Extract response window (extend a bit beyond step to see settling)
        let analysis_window = ((self.config.response_window_s * sample_rate) as usize)
            .min(gyro_response.len() - step_start);

        if analysis_window < 10 {
            return Ok(None); // Too short to analyze
        }

        let response_window = &gyro_response[step_start..step_start + analysis_window];
        let baseline_gyro = gyro_response[step_start - 1];

        // Calculate expected steady-state response
        // For gyro, we expect it to be proportional to the rate command
        let expected_response = command_change * 500.0; // Rough scaling, should be configurable

        // For steady-state error to be a real measurement we need (a) a step
        // big enough that the gyro can plausibly reach a sustained rate and
        // (b) the post-step command stayed put. The detector already
        // enforces a stability bound on the analysis window; here we just
        // require a stricter step magnitude and that the command stayed
        // close to `step_command` (the post-step plateau) for the full
        // settling window.
        let cmd_window_end = (step_start + analysis_window).min(rc_commands.len());
        let cmd_window = &rc_commands[step_start..cmd_window_end];
        let max_cmd_drift = cmd_window
            .iter()
            .map(|&c| (c - step_command).abs())
            .fold(0.0f32, f32::max);
        let is_ss_capable =
            command_change.abs() >= SS_CAPABLE_MIN_STEP && max_cmd_drift <= SS_CAPABLE_MAX_DRIFT;

        // Calculate performance metrics
        let metrics = self.calculate_step_metrics(
            response_window,
            baseline_gyro,
            expected_response,
            sample_rate,
            is_ss_capable,
        )?;

        Ok(Some(StepResponse {
            axis,
            start_time: step_start as f32 / sample_rate,
            command_magnitude: command_change.abs(),
            rise_time: metrics.rise_time,
            settling_time: metrics.settling_time,
            overshoot_percent: metrics.overshoot_percent,
            oscillation_frequency: metrics.oscillation_frequency,
            damping_ratio: metrics.damping_ratio,
            steady_state_error_dps: metrics.steady_state_error_dps,
            sample_rate,
            gyro_trace: response_window.to_vec(),
            command_trace: cmd_window.to_vec(),
        }))
    }

    /// Calculate step response performance metrics
    fn calculate_step_metrics(
        &self,
        response: &[f32],
        baseline: f32,
        expected_final: f32,
        sample_rate: f32,
        is_ss_capable: bool,
    ) -> Result<StepMetrics> {
        let dt = 1.0 / sample_rate;

        // Find rise time (10% to 90% of final value)
        let ten_percent = baseline + 0.1 * expected_final;
        let ninety_percent = baseline + 0.9 * expected_final;

        let mut rise_start_idx = None;
        let mut rise_end_idx = None;

        for (i, &value) in response.iter().enumerate() {
            if rise_start_idx.is_none() && value >= ten_percent {
                rise_start_idx = Some(i);
            }
            if rise_start_idx.is_some() && rise_end_idx.is_none() && value >= ninety_percent {
                rise_end_idx = Some(i);
                break;
            }
        }

        let rise_time = match (rise_start_idx, rise_end_idx) {
            (Some(start), Some(end)) => (end - start) as f32 * dt,
            _ => 0.1, // Default if we can't measure
        };

        // Find peak value for overshoot calculation
        let peak_value = response.iter().fold(baseline, |max, &val| val.max(max));
        let overshoot_percent = if expected_final.abs() > 0.01 {
            ((peak_value - baseline - expected_final) / expected_final.abs()) * 100.0
        } else {
            0.0
        };

        // Calculate settling time (within 2% of final value)
        let settling_tolerance = expected_final.abs() * 0.02;
        let target_value = baseline + expected_final;

        let mut settling_time = response.len() as f32 * dt; // Default to full window
        for (i, &value) in response.iter().enumerate().rev() {
            if (value - target_value).abs() > settling_tolerance {
                settling_time = (i + 1) as f32 * dt;
                break;
            }
        }

        // Estimate oscillation frequency by finding dominant frequency in response
        let oscillation_frequency = self.estimate_oscillation_frequency(response, sample_rate)?;

        // Estimate damping ratio from overshoot
        let damping_ratio = if overshoot_percent > 0.1 {
            // Using overshoot to estimate damping ratio
            let overshoot_ratio = overshoot_percent / 100.0;
            if overshoot_ratio > 0.0 {
                (-((overshoot_ratio * std::f32::consts::PI)
                    / (1.0
                        + overshoot_ratio
                            * overshoot_ratio
                            * std::f32::consts::PI
                            * std::f32::consts::PI)
                        .sqrt()))
                .exp()
            } else {
                1.0
            }
        } else {
            1.0 // Well damped
        };

        // Steady-state error: the absolute gap between where the gyro ended
        // up and where it should have ended up, in deg/s. We only emit it
        // for responses where the step was big enough and the command was
        // held steady — see SS_CAPABLE_* constants. Averaging the last
        // ~50ms of the window gives a tighter "where did it actually
        // settle" reading than the previous 10-sample tail.
        let steady_state_error_dps = if is_ss_capable {
            let tail_len = ((sample_rate * 0.05) as usize).clamp(5, response.len());
            let tail_start = response.len() - tail_len;
            let steady_state_value = response[tail_start..].iter().sum::<f32>() / tail_len as f32;
            Some((steady_state_value - target_value).abs())
        } else {
            None
        };

        Ok(StepMetrics {
            rise_time,
            settling_time,
            overshoot_percent,
            oscillation_frequency,
            damping_ratio,
            steady_state_error_dps,
        })
    }

    /// Estimate the dominant oscillation frequency in the response
    fn estimate_oscillation_frequency(&self, response: &[f32], sample_rate: f32) -> Result<f32> {
        if response.len() < 8 {
            return Ok(0.0);
        }

        // Simple zero-crossing method for frequency estimation
        let mean = response.iter().sum::<f32>() / response.len() as f32;
        let mut zero_crossings = 0;

        for i in 1..response.len() {
            if (response[i - 1] - mean) * (response[i] - mean) < 0.0 {
                zero_crossings += 1;
            }
        }

        if zero_crossings > 2 {
            let frequency = (zero_crossings as f32 / 2.0) / (response.len() as f32 / sample_rate);
            Ok(frequency)
        } else {
            Ok(0.0)
        }
    }

    /// Analyze gyro characteristics when RC command data is not available
    fn analyze_gyro_characteristics(
        &self,
        telemetry: &TelemetryData,
        pid_config: &PidConfiguration,
    ) -> Result<Vec<PidRecommendation>> {
        let mut recommendations = Vec::new();

        // Analyze gyro noise and oscillations for each axis
        for (axis, gyro_data) in [
            (Axis::Roll, &telemetry.gyro.x),
            (Axis::Pitch, &telemetry.gyro.y),
            (Axis::Yaw, &telemetry.gyro.z),
        ] {
            let analysis = self.analyze_gyro_noise_and_oscillations(
                gyro_data,
                telemetry.sample_rate,
                axis.clone(),
                pid_config,
            )?;
            recommendations.extend(analysis);
        }

        tracing::info!(
            "Generated {} recommendations from gyro analysis",
            recommendations.len()
        );
        Ok(recommendations)
    }

    /// Analyze gyro noise and oscillations for a single axis
    fn analyze_gyro_noise_and_oscillations(
        &self,
        gyro_data: &[f32],
        sample_rate: f32,
        axis: Axis,
        pid_config: &PidConfiguration,
    ) -> Result<Vec<PidRecommendation>> {
        let mut recommendations = Vec::new();

        if gyro_data.len() < 100 {
            return Ok(recommendations);
        }

        // Calculate basic statistics
        let mean = gyro_data.iter().sum::<f32>() / gyro_data.len() as f32;
        let variance = gyro_data
            .iter()
            .map(|&x| (x - mean) * (x - mean))
            .sum::<f32>()
            / gyro_data.len() as f32;
        let std_dev = variance.sqrt();

        // Calculate noise level (high frequency component)
        let noise_level = self.estimate_noise_level(gyro_data, sample_rate)?;

        // Get actual PID values for this axis
        let current_pid = match axis {
            Axis::Roll => &pid_config.roll,
            Axis::Pitch => &pid_config.pitch,
            Axis::Yaw => &pid_config.yaw,
        };

        // Check for excessive noise
        if noise_level > 15.0 {
            // Adjustable threshold (lowered to trigger more often)
            recommendations.push(PidRecommendation {
                axis: axis.clone(),
                term: PidTerm::D,
                current_value: current_pid.d,
                recommended_value: current_pid.d * D_CUT_FACTOR,
                reason: format!(
                    "Reduce D-term to decrease gyro noise (noise level: {:.1})",
                    noise_level
                ),
                priority: Priority::Medium,
            });
        }

        // Check for low frequency oscillations
        let oscillation_amplitude =
            self.detect_low_frequency_oscillations(gyro_data, sample_rate)?;
        if oscillation_amplitude > 5.0 {
            // Adjustable threshold (lowered to trigger more often).
            // Mild low-freq oscillation: a softer cut than the std-dev-driven
            // path below — half the conservative cap so we don't fight the
            // overshoot path on the same iteration.
            recommendations.push(PidRecommendation {
                axis: axis.clone(),
                term: PidTerm::P,
                current_value: current_pid.p,
                recommended_value: current_pid.p * (1.0 - MAX_P_CUT_PCT * 0.5),
                reason: format!(
                    "Reduce P-term to decrease low-frequency oscillations (amplitude: {:.1})",
                    oscillation_amplitude
                ),
                priority: Priority::Medium,
            });
        }

        // Check for very high standard deviation (general instability)
        if std_dev > 20.0 {
            // Severe instability — apply the full conservative cap.
            recommendations.push(PidRecommendation {
                axis: axis.clone(),
                term: PidTerm::P,
                current_value: current_pid.p,
                recommended_value: current_pid.p * P_CUT_MAX_FACTOR,
                reason: format!(
                    "Reduce P-term to improve general stability (std dev: {:.1})",
                    std_dev
                ),
                priority: Priority::High,
            });
        }

        Ok(recommendations)
    }

    /// Estimate noise level in gyro data
    fn estimate_noise_level(&self, gyro_data: &[f32], sample_rate: f32) -> Result<f32> {
        if gyro_data.len() < 10 {
            return Ok(0.0);
        }

        // Simple high-pass filter to estimate noise
        // Calculate differences between consecutive samples
        let differences: Vec<f32> = gyro_data.windows(2).map(|w| (w[1] - w[0]).abs()).collect();

        // Average absolute difference scaled by sample rate
        let avg_diff = differences.iter().sum::<f32>() / differences.len() as f32;
        let noise_estimate = avg_diff * sample_rate / 100.0; // Scaling factor

        Ok(noise_estimate)
    }

    /// Detect low frequency oscillations in gyro data
    fn detect_low_frequency_oscillations(
        &self,
        gyro_data: &[f32],
        _sample_rate: f32,
    ) -> Result<f32> {
        if gyro_data.len() < 50 {
            return Ok(0.0);
        }

        // Simple approach: look for periodic patterns in a moving window
        let window_size = 20;
        let mut max_oscillation: f32 = 0.0;
        for i in 0..(gyro_data.len() - window_size) {
            let window = &gyro_data[i..i + window_size];
            let window_mean = window.iter().sum::<f32>() / window.len() as f32;
            let max_deviation = window
                .iter()
                .map(|&x| (x - window_mean).abs())
                .fold(0.0, f32::max);

            max_oscillation = max_oscillation.max(max_deviation);
        }

        Ok(max_oscillation)
    }

    /// Analyze PID error signals when available
    fn analyze_pid_errors(
        &self,
        telemetry: &TelemetryData,
        pid_config: &PidConfiguration,
    ) -> Result<Vec<PidRecommendation>> {
        let mut recommendations = Vec::new();

        // Analyze each axis PID error
        for (axis, error_data) in [
            (Axis::Roll, &telemetry.pid_error.roll),
            (Axis::Pitch, &telemetry.pid_error.pitch),
            (Axis::Yaw, &telemetry.pid_error.yaw),
        ] {
            if !error_data.is_empty() {
                let analysis = self.analyze_pid_error_axis(error_data, axis.clone(), pid_config)?;
                recommendations.extend(analysis);
            }
        }

        Ok(recommendations)
    }

    /// Analyze PID error for a single axis
    fn analyze_pid_error_axis(
        &self,
        error_data: &[f32],
        axis: Axis,
        pid_config: &PidConfiguration,
    ) -> Result<Vec<PidRecommendation>> {
        let mut recommendations = Vec::new();

        if error_data.len() < 10 {
            return Ok(recommendations);
        }

        // Calculate RMS error
        let rms_error =
            (error_data.iter().map(|&x| x * x).sum::<f32>() / error_data.len() as f32).sqrt();

        // Get actual PID values for this axis
        let current_pid = match axis {
            Axis::Roll => &pid_config.roll,
            Axis::Pitch => &pid_config.pitch,
            Axis::Yaw => &pid_config.yaw,
        };

        let limits = PidLimits::default();
        let i_headroom_floor = limits.i_max * (1.0 - limits.headroom_skip_pct);

        // Check for persistent bias in error. Use the same conservative
        // bump (+10%) and absolute cap as the step-response path so the
        // two paths can't disagree on what "safe" means.
        let error_mean = error_data.iter().sum::<f32>() / error_data.len() as f32;
        if error_mean.abs() > 2.0 && current_pid.i < i_headroom_floor {
            let proposed = (current_pid.i * I_TERM_BUMP).min(limits.i_max);
            if proposed - current_pid.i >= 1.0 {
                recommendations.push(PidRecommendation {
                    axis: axis.clone(),
                    term: PidTerm::I,
                    current_value: current_pid.i,
                    recommended_value: proposed,
                    reason: format!(
                        "Increase I-term to eliminate error bias (bias: {:.2})",
                        error_mean
                    ),
                    priority: Priority::High,
                });
            }
        }

        // RMS error driven I-bump only fires if the bias check above didn't
        // (otherwise we'd double-recommend the same axis) and only when
        // there's headroom under the cap.
        let already_recommended_i = recommendations.iter().any(|r| matches!(r.term, PidTerm::I));
        if !already_recommended_i && rms_error > 5.0 && current_pid.i < i_headroom_floor {
            let proposed = (current_pid.i * I_TERM_BUMP).min(limits.i_max);
            if proposed - current_pid.i >= 1.0 {
                recommendations.push(PidRecommendation {
                    axis: axis.clone(),
                    term: PidTerm::I,
                    current_value: current_pid.i,
                    recommended_value: proposed,
                    reason: format!("Increase I-term: persistent RMS error {:.2}", rms_error),
                    priority: Priority::Medium,
                });
            }
        }

        Ok(recommendations)
    }

    /// Analyze step responses for an axis and generate PID recommendations
    fn analyze_axis_responses(
        &self,
        responses: &[StepResponse],
        axis: Axis,
        pid_config: &PidConfiguration,
    ) -> Result<Vec<PidRecommendation>> {
        if responses.is_empty() {
            return Ok(Vec::new());
        }

        let mut recommendations = Vec::new();

        // Calculate average metrics across all responses
        let avg_rise_time =
            responses.iter().map(|r| r.rise_time).sum::<f32>() / responses.len() as f32;
        let avg_overshoot =
            responses.iter().map(|r| r.overshoot_percent).sum::<f32>() / responses.len() as f32;
        let avg_settling_time =
            responses.iter().map(|r| r.settling_time).sum::<f32>() / responses.len() as f32;
        let avg_oscillation_freq = responses
            .iter()
            .map(|r| r.oscillation_frequency)
            .sum::<f32>()
            / responses.len() as f32;
        let avg_damping =
            responses.iter().map(|r| r.damping_ratio).sum::<f32>() / responses.len() as f32;

        // Get actual PID values for this axis
        let current_pid = match axis {
            Axis::Roll => &pid_config.roll,
            Axis::Pitch => &pid_config.pitch,
            Axis::Yaw => &pid_config.yaw,
        };

        let limits = PidLimits::default();
        let p_headroom_floor = limits.p_max * (1.0 - limits.headroom_skip_pct);
        let d_headroom_floor = limits.d_max * (1.0 - limits.headroom_skip_pct);

        // Analyze P-term based on overshoot and rise time
        if avg_overshoot > self.config.overshoot_tolerance {
            // Scale the cut by how much overshoot exceeds tolerance, capped
            // at MAX_P_CUT_PCT. The 50.0 divisor keeps the slope gentle:
            // 30% overshoot above tolerance ⇒ 30/50 = 0.6 raw, clamped to
            // MAX_P_CUT_PCT (0.15). The pre-Item-2 cap was 0.30, which is
            // what drove pitch P from 51 to 41 to 29 in three iterations.
            let reduction_percent = (avg_overshoot - self.config.overshoot_tolerance) / 50.0;
            let proposed = current_pid.p * (1.0 - reduction_percent.min(MAX_P_CUT_PCT));
            if current_pid.p - proposed >= 1.0 {
                recommendations.push(PidRecommendation {
                    axis: axis.clone(),
                    term: PidTerm::P,
                    current_value: current_pid.p,
                    recommended_value: proposed,
                    reason: format!(
                        "Reduce P-term to decrease overshoot from {:.1}% to target {:.1}%",
                        avg_overshoot, self.config.overshoot_tolerance
                    ),
                    priority: if avg_overshoot > 25.0 {
                        Priority::High
                    } else {
                        Priority::Medium
                    },
                });
            }
        } else if avg_rise_time > 0.15 && avg_overshoot < 5.0 && current_pid.p < p_headroom_floor {
            // Slow rise time with little overshoot suggests P could be increased.
            let proposed = (current_pid.p * P_BUMP_FACTOR).min(limits.p_max);
            if proposed - current_pid.p >= 1.0 {
                recommendations.push(PidRecommendation {
                    axis: axis.clone(),
                    term: PidTerm::P,
                    current_value: current_pid.p,
                    recommended_value: proposed,
                    reason: format!(
                        "Increase P-term to improve responsiveness (rise time: {:.3}s)",
                        avg_rise_time
                    ),
                    priority: Priority::Low,
                });
            }
        }

        // Analyze I-term based on steady-state error.
        //
        // We average only the responses that were large enough and held
        // steady enough to give a meaningful steady-state reading. If we
        // don't have at least SS_MIN_VALID_RESPONSES of those, we say
        // nothing — the previous behaviour of recommending I-bumps from
        // small stick movements gave a false signal that compounded across
        // tuning iterations.
        let ss_capable: Vec<f32> = responses
            .iter()
            .filter_map(|r| r.steady_state_error_dps)
            .collect();
        if ss_capable.len() >= SS_MIN_VALID_RESPONSES {
            let avg_ss_error_dps = ss_capable.iter().sum::<f32>() / ss_capable.len() as f32;
            let i_headroom_floor = limits.i_max * (1.0 - limits.headroom_skip_pct);
            if avg_ss_error_dps > SS_ERROR_RECOMMEND_DPS && current_pid.i < i_headroom_floor {
                let proposed = (current_pid.i * I_TERM_BUMP).min(limits.i_max);
                // Only emit if the change is at least one integer FC unit;
                // otherwise the recommendation is invisible to the FC.
                if proposed - current_pid.i >= 1.0 {
                    recommendations.push(PidRecommendation {
                        axis: axis.clone(),
                        term: PidTerm::I,
                        current_value: current_pid.i,
                        recommended_value: proposed,
                        reason: format!(
                            "Increase I-term: {:.0} deg/s steady-state tracking error across {} valid step responses",
                            avg_ss_error_dps,
                            ss_capable.len()
                        ),
                        priority: Priority::Medium,
                    });
                }
            }
        }

        // Route the detected oscillation to the right gain by *frequency band*,
        // not just by "any oscillation with low damping → bump D" (which was
        // the load-bearing bug that drove MEDIUM FUCKER into runaway).
        // See `OscillationBand` for the band semantics.
        if avg_damping < 0.5 {
            match classify_oscillation_band(avg_oscillation_freq) {
                OscillationBand::PTooHigh => {
                    // 5–15 Hz overshoot ringing: cut P by half the
                    // conservative cap. The std-dev path applies the full
                    // cap for *severe* instability — leave headroom for
                    // that to dominate when both fire.
                    let proposed = current_pid.p * (1.0 - MAX_P_CUT_PCT * 0.5);
                    if current_pid.p - proposed >= 1.0 {
                        recommendations.push(PidRecommendation {
                            axis: axis.clone(),
                            term: PidTerm::P,
                            current_value: current_pid.p,
                            recommended_value: proposed,
                            reason: format!(
                                "Reduce P-term to suppress {:.1} Hz overshoot ringing (damping: {:.2})",
                                avg_oscillation_freq, avg_damping
                            ),
                            priority: Priority::Medium,
                        });
                    }
                }
                OscillationBand::DTooLow => {
                    // 30–80 Hz: classic D-too-low. Bump D by the global
                    // step. Suppress when D is already at the headroom
                    // ceiling — the previous 30% bump pegged D at d_max
                    // in 2-3 iterations and never recovered.
                    if current_pid.d < d_headroom_floor {
                        let proposed = (current_pid.d * D_BUMP_FACTOR).min(limits.d_max);
                        if proposed - current_pid.d >= 1.0 {
                            recommendations.push(PidRecommendation {
                                axis: axis.clone(),
                                term: PidTerm::D,
                                current_value: current_pid.d,
                                recommended_value: proposed,
                                reason: format!(
                                    "Increase D-term to dampen {:.1} Hz oscillation (damping: {:.2})",
                                    avg_oscillation_freq, avg_damping
                                ),
                                priority: Priority::Medium,
                            });
                        }
                    }
                }
                OscillationBand::Noise => {
                    // >80 Hz: motor / filter / mechanical noise, NOT a PID
                    // problem. Bumping D here was the bug. Suppress and let
                    // the FilterOptimizer pass handle it via its own recs.
                    tracing::info!(
                        "{:?}: {:.1} Hz oscillation suppressed — frequency suggests filter/noise issue, not a PID problem",
                        axis,
                        avg_oscillation_freq
                    );
                }
                OscillationBand::Ambiguous => {
                    // 15–30 Hz: too many false positives across I-term, rate
                    // config, and phase-margin issues. Stay quiet.
                    tracing::debug!(
                        "{:?}: {:.1} Hz oscillation in ambiguous band — no PID rec",
                        axis,
                        avg_oscillation_freq
                    );
                }
                OscillationBand::Subharmonic => {
                    // <5 Hz drift; the slow-settling branch below catches
                    // genuine over-damped settling cases.
                }
            }
        }
        if avg_settling_time > 0.5 && avg_oscillation_freq < 5.0 {
            // Long settling time with no oscillation: classic over-damped.
            // Cut D by the global step.
            let proposed = current_pid.d * D_CUT_FACTOR;
            if current_pid.d - proposed >= 1.0 {
                recommendations.push(PidRecommendation {
                    axis: axis.clone(),
                    term: PidTerm::D,
                    current_value: current_pid.d,
                    recommended_value: proposed,
                    reason: format!(
                        "Reduce D-term to improve settling time ({:.3}s)",
                        avg_settling_time
                    ),
                    priority: Priority::Low,
                });
            }
        }

        Ok(recommendations)
    }
}

impl Default for PidAnalyzerConfig {
    fn default() -> Self {
        Self {
            error_threshold: 0.1,
            response_window_s: 1.0,    // Increased to capture full response
            overshoot_tolerance: 15.0, // Reasonable overshoot tolerance
        }
    }
}

#[cfg(test)]
mod band_routing_tests {
    use super::*;
    use crate::domain::{PidConfiguration, PidValues};

    #[test]
    fn classifier_partitions_known_frequencies() {
        // Sanity-check the band edges so the regression below has a stable foundation.
        assert_eq!(classify_oscillation_band(2.0), OscillationBand::Subharmonic);
        assert_eq!(classify_oscillation_band(4.99), OscillationBand::Subharmonic);
        assert_eq!(classify_oscillation_band(5.0), OscillationBand::PTooHigh);
        assert_eq!(classify_oscillation_band(11.9), OscillationBand::PTooHigh);
        assert_eq!(classify_oscillation_band(15.0), OscillationBand::Ambiguous);
        assert_eq!(classify_oscillation_band(25.0), OscillationBand::Ambiguous);
        assert_eq!(classify_oscillation_band(30.0), OscillationBand::DTooLow);
        assert_eq!(classify_oscillation_band(60.0), OscillationBand::DTooLow);
        assert_eq!(classify_oscillation_band(80.0), OscillationBand::Noise);
        assert_eq!(classify_oscillation_band(150.0), OscillationBand::Noise);
        // NaN is treated as subharmonic — we never want a NaN-driven recommendation.
        assert_eq!(classify_oscillation_band(f32::NAN), OscillationBand::Subharmonic);
    }

    fn make_response(axis: Axis, freq_hz: f32, damping: f32) -> StepResponse {
        StepResponse {
            axis,
            start_time: 0.0,
            command_magnitude: 0.5,
            rise_time: 0.05,
            settling_time: 0.20,
            overshoot_percent: 8.0, // below overshoot_tolerance so the P-cut from
                                    // overshoot path doesn't fire and steal credit
            oscillation_frequency: freq_hz,
            damping_ratio: damping,
            steady_state_error_dps: None,
            sample_rate: 4000.0,
            gyro_trace: Vec::new(),
            command_trace: Vec::new(),
        }
    }

    fn pid_cfg(p: f32, i: f32, d: f32) -> PidConfiguration {
        let mut cfg = PidConfiguration::default();
        cfg.roll = PidValues { p, i, d, f: None };
        cfg.pitch = PidValues { p, i, d, f: None };
        cfg.yaw = PidValues { p, i, d, f: None };
        cfg
    }

    /// Regression test for the MEDIUM FUCKER runaway: a 11.9 Hz oscillation
    /// with poor damping must produce a P-CUT, never a D-BUMP. Before the
    /// band-routing fix, the analyzer mapped any sub-Nyquist underdamped
    /// oscillation to D, which compounded into unflyable PIDs across a few
    /// `tune --apply-all` iterations.
    #[test]
    fn low_frequency_oscillation_recommends_p_cut_not_d_bump() {
        let analyzer = PidAnalyzer::new();
        // Three roll-axis responses at 11.9 Hz with 0.3 damping (clearly
        // underdamped) — the same shape as the real failed flight.
        let responses = vec![
            make_response(Axis::Roll, 11.9, 0.3),
            make_response(Axis::Roll, 12.1, 0.28),
            make_response(Axis::Roll, 11.7, 0.32),
        ];
        let cfg = pid_cfg(47.0, 75.0, 28.0); // baseline pre-runaway gains
        let recs = analyzer
            .analyze_axis_responses(&responses, Axis::Roll, &cfg)
            .expect("analysis must succeed on synthetic data");

        let p_cuts: Vec<_> = recs
            .iter()
            .filter(|r| matches!(r.term, PidTerm::P) && r.recommended_value < r.current_value)
            .collect();
        let d_bumps: Vec<_> = recs
            .iter()
            .filter(|r| matches!(r.term, PidTerm::D) && r.recommended_value > r.current_value)
            .collect();

        assert!(
            !p_cuts.is_empty(),
            "11.9 Hz underdamped oscillation must yield a P-cut. Got: {:?}",
            recs
        );
        assert!(
            d_bumps.is_empty(),
            "11.9 Hz oscillation must NOT trigger a D-bump (was the runaway bug). Got: {:?}",
            recs
        );
    }

    /// Mid-band oscillation (50 Hz, classic D-too-low) should still bump D.
    /// Validates the routing didn't lose the legitimate D-recommendation path.
    #[test]
    fn mid_band_oscillation_still_recommends_d_bump() {
        let analyzer = PidAnalyzer::new();
        let responses = vec![
            make_response(Axis::Pitch, 50.0, 0.3),
            make_response(Axis::Pitch, 48.0, 0.28),
            make_response(Axis::Pitch, 52.0, 0.32),
        ];
        let cfg = pid_cfg(50.0, 80.0, 30.0);
        let recs = analyzer
            .analyze_axis_responses(&responses, Axis::Pitch, &cfg)
            .expect("analysis must succeed");

        let d_bumps: Vec<_> = recs
            .iter()
            .filter(|r| matches!(r.term, PidTerm::D) && r.recommended_value > r.current_value)
            .collect();
        assert!(
            !d_bumps.is_empty(),
            "50 Hz underdamped oscillation in classic D-too-low band must yield a D-bump. Got: {:?}",
            recs
        );
    }

    /// High-frequency oscillation (>80 Hz) is filter / motor-noise territory.
    /// The analyzer must NOT emit any PID recommendation in this band — that
    /// was the load-bearing failure mode.
    #[test]
    fn high_frequency_noise_emits_no_pid_recommendation() {
        let analyzer = PidAnalyzer::new();
        let responses = vec![
            make_response(Axis::Roll, 120.0, 0.3),
            make_response(Axis::Roll, 130.0, 0.28),
            make_response(Axis::Roll, 110.0, 0.32),
        ];
        let cfg = pid_cfg(50.0, 80.0, 30.0);
        let recs = analyzer
            .analyze_axis_responses(&responses, Axis::Roll, &cfg)
            .expect("analysis must succeed");

        let oscillation_driven: Vec<_> = recs
            .iter()
            .filter(|r| !matches!(r.term, PidTerm::I)) // I-term recs are driven by SS error, not freq
            .collect();
        assert!(
            oscillation_driven.is_empty(),
            ">80 Hz noise must not drive any P/D recommendation. Got: {:?}",
            recs
        );
    }

    // ---------- step-size guardrail tests (Item 2) ----------

    /// Sanity-check the step-size constants. Locks in the post-Item-2
    /// values; any future loosening should land alongside an explicit
    /// review of the runaway scenario in docs/algorithm-improvements.md.
    #[test]
    fn step_size_constants_are_conservative() {
        assert!(
            MAX_P_CUT_PCT <= 0.15,
            "MAX_P_CUT_PCT must stay ≤15% — looser caps are what drove pitch P 51→29 in MEDIUM FUCKER"
        );
        assert!(
            D_STEP_PCT <= 0.10,
            "D_STEP_PCT must stay ≤10% — the pre-Item-2 30% bump was the runaway arithmetic"
        );
        assert!((P_BUMP_PCT - 0.08).abs() < 1e-6, "P_BUMP_PCT == 8%");
        assert!((I_BUMP_PCT - 0.08).abs() < 1e-6, "I_BUMP_PCT == 8%");
        // Derived multipliers stay in sync with their percentages.
        assert!((P_CUT_MAX_FACTOR - (1.0 - MAX_P_CUT_PCT)).abs() < 1e-6);
        assert!((P_BUMP_FACTOR - (1.0 + P_BUMP_PCT)).abs() < 1e-6);
        assert!((D_BUMP_FACTOR - (1.0 + D_STEP_PCT)).abs() < 1e-6);
        assert!((D_CUT_FACTOR - (1.0 - D_STEP_PCT)).abs() < 1e-6);
        assert!((I_TERM_BUMP - (1.0 + I_BUMP_PCT)).abs() < 1e-6);
    }

    /// Mid-band D-bump must respect the 8% step. Pre-Item-2 it was 30%,
    /// which compounded into the MEDIUM FUCKER D 28→58 jump.
    #[test]
    fn d_bump_respects_step_size_cap() {
        let analyzer = PidAnalyzer::new();
        let responses = vec![
            make_response(Axis::Roll, 50.0, 0.3),
            make_response(Axis::Roll, 50.0, 0.3),
            make_response(Axis::Roll, 50.0, 0.3),
        ];
        let cfg = pid_cfg(40.0, 80.0, 28.0);
        let recs = analyzer
            .analyze_axis_responses(&responses, Axis::Roll, &cfg)
            .expect("analysis must succeed");

        let d_bump = recs
            .iter()
            .find(|r| matches!(r.term, PidTerm::D) && r.recommended_value > r.current_value)
            .expect("a D-bump should fire for 50 Hz / damping 0.3");
        let pct_increase = (d_bump.recommended_value - d_bump.current_value) / d_bump.current_value;
        assert!(
            pct_increase <= D_STEP_PCT + 1e-6,
            "D-bump exceeded the {:.0}% cap: {} → {} ({:.1}% jump)",
            D_STEP_PCT * 100.0,
            d_bump.current_value,
            d_bump.recommended_value,
            pct_increase * 100.0
        );
    }

    /// Severe overshoot must cap the P-cut at MAX_P_CUT_PCT, never the
    /// pre-Item-2 30% maximum. Pre-fix, a 65% overshoot would yield a 30%
    /// P-cut — that's how pitch P collapsed 51 → ~36 in one iteration.
    #[test]
    fn overshoot_p_cut_respects_step_size_cap() {
        let analyzer = PidAnalyzer::new();
        // 65% overshoot, way above the default 15% tolerance — would have
        // hit the old 30% cap. Must now clip at MAX_P_CUT_PCT (15%).
        let make_overshoot_response = |overshoot: f32| StepResponse {
            axis: Axis::Pitch,
            start_time: 0.0,
            command_magnitude: 0.5,
            rise_time: 0.05,
            settling_time: 0.20,
            overshoot_percent: overshoot,
            oscillation_frequency: 25.0, // ambiguous band — no extra rec from band routing
            damping_ratio: 0.7,          // damped, no D recs
            steady_state_error_dps: None,
            sample_rate: 4000.0,
            gyro_trace: Vec::new(),
            command_trace: Vec::new(),
        };
        let responses = vec![
            make_overshoot_response(65.0),
            make_overshoot_response(65.0),
            make_overshoot_response(65.0),
        ];
        let cfg = pid_cfg(51.0, 82.0, 32.0);
        let recs = analyzer
            .analyze_axis_responses(&responses, Axis::Pitch, &cfg)
            .expect("analysis must succeed");

        let p_cut = recs
            .iter()
            .find(|r| matches!(r.term, PidTerm::P) && r.recommended_value < r.current_value)
            .expect("severe overshoot must yield a P-cut");
        let pct_cut = (p_cut.current_value - p_cut.recommended_value) / p_cut.current_value;
        assert!(
            pct_cut <= MAX_P_CUT_PCT + 1e-6,
            "P-cut exceeded the {:.0}% cap: {} → {} ({:.1}% drop)",
            MAX_P_CUT_PCT * 100.0,
            p_cut.current_value,
            p_cut.recommended_value,
            pct_cut * 100.0
        );
    }

    /// End-to-end MEDIUM FUCKER scenario: starting from baseline (Roll D=28),
    /// simulate 3 consecutive tunes recommending the same D-bump direction.
    /// Pre-Item-2 chain was 28 → 36 → 47 → 61 (capped 60). With the new cap
    /// it must stay below 36 after three iterations.
    #[test]
    fn three_iterations_of_d_bumps_dont_runaway() {
        let analyzer = PidAnalyzer::new();
        let mut current_d: f32 = 28.0;
        for _ in 0..3 {
            let responses = vec![
                make_response(Axis::Roll, 50.0, 0.3),
                make_response(Axis::Roll, 50.0, 0.3),
                make_response(Axis::Roll, 50.0, 0.3),
            ];
            let cfg = pid_cfg(40.0, 80.0, current_d);
            let recs = analyzer
                .analyze_axis_responses(&responses, Axis::Roll, &cfg)
                .expect("analysis must succeed");
            if let Some(d_bump) = recs
                .iter()
                .find(|r| matches!(r.term, PidTerm::D) && r.recommended_value > r.current_value)
            {
                current_d = d_bump.recommended_value;
            }
        }
        assert!(
            current_d < 36.0,
            "After 3 D-bumps, current_d={current_d} should stay <36 (pre-Item-2 chain reached 47)"
        );
    }

    // ---------- per-axis stacking limit tests (Item 3) ----------

    fn make_rec(
        axis: Axis,
        term: PidTerm,
        current: f32,
        recommended: f32,
        priority: Priority,
    ) -> PidRecommendation {
        PidRecommendation {
            axis,
            term,
            current_value: current,
            recommended_value: recommended,
            reason: String::new(),
            priority,
        }
    }

    #[test]
    fn collapse_keeps_at_most_one_rec_per_axis() {
        let recs = vec![
            make_rec(Axis::Roll, PidTerm::P, 50.0, 45.0, Priority::Medium),
            make_rec(Axis::Roll, PidTerm::I, 80.0, 86.0, Priority::Medium),
            make_rec(Axis::Roll, PidTerm::D, 30.0, 33.0, Priority::Medium),
        ];
        let out = collapse_recs_per_axis(recs);
        assert_eq!(out.len(), 1, "Three same-axis recs must collapse to one");
        assert_eq!(out[0].axis, Axis::Roll);
    }

    #[test]
    fn collapse_per_axis_independent() {
        let recs = vec![
            make_rec(Axis::Roll, PidTerm::P, 50.0, 45.0, Priority::Medium),
            make_rec(Axis::Roll, PidTerm::I, 80.0, 86.0, Priority::Medium),
            make_rec(Axis::Pitch, PidTerm::D, 30.0, 33.0, Priority::Medium),
            make_rec(Axis::Yaw, PidTerm::P, 40.0, 38.0, Priority::Medium),
        ];
        let out = collapse_recs_per_axis(recs);
        assert_eq!(out.len(), 3, "Each axis collapses independently");
        let axes: Vec<_> = out.iter().map(|r| r.axis.clone()).collect();
        assert_eq!(axes, vec![Axis::Roll, Axis::Pitch, Axis::Yaw],
                   "Output stably ordered Roll → Pitch → Yaw");
    }

    #[test]
    fn collapse_higher_priority_wins() {
        // A High-priority I-bump beats a Medium-priority P-cut even though
        // the P-cut would otherwise win on direction + term tiebreakers.
        let recs = vec![
            make_rec(Axis::Roll, PidTerm::P, 50.0, 45.0, Priority::Medium),
            make_rec(Axis::Roll, PidTerm::I, 80.0, 88.0, Priority::High),
        ];
        let out = collapse_recs_per_axis(recs);
        assert_eq!(out.len(), 1);
        assert!(matches!(out[0].term, PidTerm::I), "High-priority I-bump must win, got {:?}", out[0]);
    }

    #[test]
    fn collapse_cut_beats_bump_on_same_priority() {
        // Both Medium priority: cut wins (safer direction).
        let recs = vec![
            make_rec(Axis::Pitch, PidTerm::D, 30.0, 33.0, Priority::Medium), // bump
            make_rec(Axis::Pitch, PidTerm::I, 80.0, 73.6, Priority::Medium), // cut
        ];
        let out = collapse_recs_per_axis(recs);
        assert_eq!(out.len(), 1);
        assert!(out[0].recommended_value < out[0].current_value,
                "Cut must beat bump at same priority");
    }

    #[test]
    fn collapse_p_beats_d_beats_i_on_full_tie() {
        // Three Medium-priority cuts, same direction. P should win.
        let recs = vec![
            make_rec(Axis::Roll, PidTerm::I, 80.0, 73.0, Priority::Medium),
            make_rec(Axis::Roll, PidTerm::D, 30.0, 27.0, Priority::Medium),
            make_rec(Axis::Roll, PidTerm::P, 50.0, 45.0, Priority::Medium),
        ];
        let out = collapse_recs_per_axis(recs);
        assert_eq!(out.len(), 1);
        assert!(matches!(out[0].term, PidTerm::P), "P must beat D and I on full tie, got {:?}", out[0]);

        // Without the P entry, D should win over I.
        let recs = vec![
            make_rec(Axis::Roll, PidTerm::I, 80.0, 73.0, Priority::Medium),
            make_rec(Axis::Roll, PidTerm::D, 30.0, 27.0, Priority::Medium),
        ];
        let out = collapse_recs_per_axis(recs);
        assert!(matches!(out[0].term, PidTerm::D), "D must beat I on tie, got {:?}", out[0]);
    }

    #[test]
    fn collapse_preserves_critical_over_high() {
        let recs = vec![
            make_rec(Axis::Yaw, PidTerm::P, 40.0, 36.0, Priority::High),
            make_rec(Axis::Yaw, PidTerm::D, 20.0, 18.0, Priority::Critical),
        ];
        let out = collapse_recs_per_axis(recs);
        assert_eq!(out.len(), 1);
        assert!(matches!(out[0].priority, Priority::Critical), "Critical must beat High");
    }

    #[test]
    fn collapse_empty_input_is_empty() {
        let out = collapse_recs_per_axis(Vec::new());
        assert!(out.is_empty());
    }

    /// Realistic end-to-end scenario: an underdamped Roll axis triggers
    /// both an overshoot-driven P-cut AND an SS-error-driven I-bump in
    /// the same pass. Pre-Item-3, both would be applied in one tune
    /// iteration. Post-Item-3, only the P-cut survives the collapse.
    #[test]
    fn collapse_drops_i_bump_when_p_cut_also_present() {
        // Realistic shape of the per-axis output before collapse:
        // overshoot path emits P-cut, SS-error path emits I-bump.
        let recs = vec![
            // Overshoot P-cut (Medium, cut)
            make_rec(Axis::Roll, PidTerm::P, 50.0, 42.5, Priority::Medium),
            // SS-error I-bump (Medium, bump)
            make_rec(Axis::Roll, PidTerm::I, 80.0, 86.4, Priority::Medium),
        ];
        let out = collapse_recs_per_axis(recs);
        assert_eq!(out.len(), 1);
        assert!(matches!(out[0].term, PidTerm::P),
                "P-cut should win over I-bump at same priority (cut > bump)");
    }

    // ---------- baseline-anchored bound tests (Item 4) ----------

    #[test]
    fn clamp_passes_through_when_within_bounds() {
        // Baseline P=50, current=50, recommended=46 (8% cut). Inside ±15%.
        let recs = vec![make_rec(Axis::Roll, PidTerm::P, 50.0, 46.0, Priority::Medium)];
        let baseline = pid_cfg(50.0, 80.0, 30.0);
        let out = clamp_recs_to_baseline(recs, &baseline);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].recommended_value, 46.0);
    }

    #[test]
    fn clamp_caps_aggressive_p_cut_at_baseline_envelope() {
        // Baseline P=47, current=47, recommended=33 (−30%). Must clamp
        // to baseline*0.85 = 39.95.
        let recs = vec![make_rec(Axis::Roll, PidTerm::P, 47.0, 33.0, Priority::High)];
        let baseline = pid_cfg(47.0, 75.0, 28.0);
        let out = clamp_recs_to_baseline(recs, &baseline);
        assert_eq!(out.len(), 1);
        let expected = 47.0 * (1.0 - BASELINE_BOUND_PCT);
        assert!((out[0].recommended_value - expected).abs() < 1e-3,
                "expected {expected:.2}, got {}", out[0].recommended_value);
        assert!(out[0].reason.contains("clamped"), "clamp note must annotate the rec");
    }

    #[test]
    fn clamp_allows_movement_toward_baseline_when_already_drifted() {
        // Real-world MEDIUM FUCKER recovery: baseline P=47, current=33
        // (already drifted), recommended=37 (moving toward baseline).
        // The envelope expands to enclose current, so 37 must be allowed.
        let recs = vec![make_rec(Axis::Roll, PidTerm::P, 33.0, 37.0, Priority::Medium)];
        let baseline = pid_cfg(47.0, 75.0, 28.0);
        let out = clamp_recs_to_baseline(recs, &baseline);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].recommended_value, 37.0,
                   "recovery moves toward baseline must be allowed unchanged");
    }

    #[test]
    fn clamp_drops_rec_that_would_drift_further_from_baseline() {
        // Baseline P=47, current=33 (already low), recommended=28
        // (further reduction). Clamping toward baseline would reverse
        // the rec's intent, so the rec must be dropped entirely.
        let recs = vec![make_rec(Axis::Roll, PidTerm::P, 33.0, 28.0, Priority::Medium)];
        let baseline = pid_cfg(47.0, 75.0, 28.0);
        let out = clamp_recs_to_baseline(recs, &baseline);
        assert!(out.is_empty(),
                "rec that drifts further from baseline must be dropped, got {:?}", out);
    }

    #[test]
    fn clamp_passes_through_yaw_d_with_zero_baseline() {
        // Yaw D conventionally 0; can't sensibly clamp 0 ± 15% × 0 = 0.
        // Pass-through avoids forbidding any future yaw-D recommendation.
        let recs = vec![make_rec(Axis::Yaw, PidTerm::D, 0.0, 5.0, Priority::Low)];
        let baseline = pid_cfg(47.0, 75.0, 0.0); // baseline yaw D = 0
        let out = clamp_recs_to_baseline(recs, &baseline);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].recommended_value, 5.0);
    }

    /// MEDIUM FUCKER regression: at the crisis point (current Roll D=58,
    /// baseline=28), a fresh tune that recommends bumping D further
    /// (to ~63) must be either clamped to no more than baseline*1.15=32.2,
    /// or dropped because the clamp would reverse the rec's "go up" intent.
    #[test]
    fn clamp_prevents_d_runaway_at_drifted_state() {
        let recs = vec![make_rec(Axis::Roll, PidTerm::D, 58.0, 63.0, Priority::Medium)];
        let baseline = pid_cfg(47.0, 75.0, 28.0);
        let out = clamp_recs_to_baseline(recs, &baseline);
        assert!(
            out.is_empty(),
            "An out-of-envelope D-bump that would push gain further from baseline \
             must be dropped (got {:?})",
            out
        );
    }
}
