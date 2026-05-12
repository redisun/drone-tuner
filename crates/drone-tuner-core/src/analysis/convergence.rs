//! Cross-tune convergence detection (Item 5 of `docs/algorithm-improvements.md`).
//!
//! A single tune iteration is stateless: the analyzer looks at one .bbl, emits
//! recommendations, and forgets. That works fine on the first pass, but it
//! creates a failure mode the per-iteration guardrails (Items 1–4) can't
//! catch: the analyzer can be **right about the symptom but wrong about the
//! cause**, iteration after iteration.
//!
//! Concretely: a 65 Hz oscillation driven by a damaged prop or a missing RPM
//! filter looks like the textbook "D-too-low" case to a single .bbl analysis
//! — bump D, and the next pass sees a slightly cleaner step response, so it
//! bumps D again, and again. Each individual rec passes Item 2's step-size
//! cap and Item 4's baseline envelope; nothing in the per-iteration model
//! says "stop, you're tuning the wrong thing."
//!
//! This module reads the persisted tune history and detects exactly that
//! pattern: the same `(axis, term)` was pushed in the same direction across
//! recent iterations. When the new analysis wants to push it *again* in the
//! same direction, we suppress the rec and emit a high-priority advisory
//! pointing the user at filter / mechanical investigation instead.

use crate::domain::{Axis, PidRecommendation, PidTerm};

/// One historical PID-gain change recorded by a prior tune iteration. Sequence
/// is **chronological, oldest first** — the suppression algorithm walks it
/// from the back to find the most recent activity for each `(axis, term)`.
#[derive(Debug, Clone)]
pub struct PidChangeRecord {
    /// Which axis the historical change was on.
    pub axis: Axis,
    /// Which PID term the historical change touched.
    pub term: PidTerm,
    /// Signed change in raw FC integer units (post − pre). Positive = bumped
    /// up, negative = cut, zero = the iteration left this gain alone.
    pub delta: i32,
}

/// Default number of consecutive same-direction changes before we declare
/// convergence. Two is the smallest value that captures "this isn't a one-
/// off correction" without requiring the user to sit through three failed
/// tunes before the system speaks up.
pub const DEFAULT_MIN_REPEATED: usize = 2;

/// Outcome of [`apply_convergence_suppression`]: the recommendations we kept,
/// and the ones we suppressed (with advisory text the caller can surface).
#[derive(Debug)]
pub struct ConvergenceOutcome {
    /// Recommendations that survived the convergence check. Order is
    /// preserved relative to the input.
    pub kept: Vec<PidRecommendation>,
    /// Recommendations that were dropped because the same `(axis, term)`
    /// has been pushed in the same direction across `iterations` past tunes
    /// without the underlying problem resolving.
    pub suppressed: Vec<SuppressedRecommendation>,
}

/// One PID rec we dropped because cross-tune history says we're stuck.
#[derive(Debug, Clone)]
pub struct SuppressedRecommendation {
    /// The original recommendation that would have been applied. Carried so
    /// the CLI can show what would have happened.
    pub original: PidRecommendation,
    /// How many prior iterations (including the current one) pushed this
    /// `(axis, term)` in the same direction. Always ≥ `min_repeated + 1`.
    pub iterations_in_direction: usize,
    /// Pre-rendered human-readable advisory string, suitable for printing
    /// directly. Always names the gain, the direction, and points at the
    /// filter/mechanical follow-up.
    pub advisory: String,
}

/// Filter `recs` against historical convergence: drop any rec whose
/// `(axis, term, direction)` matches the trailing `min_repeated` history
/// entries for that gain. Empty `history` is a no-op (first iteration).
///
/// `min_repeated == 0` short-circuits to no-op; pass [`DEFAULT_MIN_REPEATED`]
/// for production use.
pub fn apply_convergence_suppression(
    recs: Vec<PidRecommendation>,
    history: &[PidChangeRecord],
    min_repeated: usize,
) -> ConvergenceOutcome {
    if min_repeated == 0 || history.is_empty() {
        return ConvergenceOutcome {
            kept: recs,
            suppressed: Vec::new(),
        };
    }

    let mut kept = Vec::with_capacity(recs.len());
    let mut suppressed = Vec::new();

    for rec in recs {
        let rec_delta = rec.recommended_value - rec.current_value;
        let rec_sign: i32 = if rec_delta > 0.5 {
            1
        } else if rec_delta < -0.5 {
            -1
        } else {
            // Round-to-zero rec: there's nothing to suppress because the
            // applied change would be invisible to the FC anyway. Keep it
            // (downstream filters drop sub-unit recs by their own logic).
            kept.push(rec);
            continue;
        };

        // Walk history newest-first, collecting the last `min_repeated`
        // non-zero changes for this (axis, term). If they all share the
        // rec's direction, this is a runaway in the making.
        let recent_signs: Vec<i32> = history
            .iter()
            .rev()
            .filter(|h| h.axis == rec.axis && h.term == rec.term && h.delta != 0)
            .take(min_repeated)
            .map(|h| h.delta.signum())
            .collect();

        let converging =
            recent_signs.len() >= min_repeated && recent_signs.iter().all(|s| *s == rec_sign);

        if converging {
            let advisory = build_advisory(&rec, rec_sign, recent_signs.len() + 1);
            suppressed.push(SuppressedRecommendation {
                original: rec,
                iterations_in_direction: recent_signs.len() + 1,
                advisory,
            });
        } else {
            kept.push(rec);
        }
    }

    ConvergenceOutcome { kept, suppressed }
}

/// Render the human-readable advisory for a suppressed rec. Public-ish
/// (`pub(crate)`) so future CLI callers can format it consistently if they
/// reconstruct the message from raw fields.
fn build_advisory(rec: &PidRecommendation, rec_sign: i32, iterations: usize) -> String {
    let direction = if rec_sign > 0 { "up" } else { "down" };
    // Direction-specific follow-up. Both branches point at filter / RPM /
    // mechanical investigation — the underlying message is the same:
    // a PID gain that needs to keep moving in one direction is a sign the
    // PID loop isn't the failing component.
    let followup = match (rec_sign, &rec.term) {
        (_, PidTerm::D) => {
            "Bumping D iteratively masks gyro/D-term LPF noise that should be \
             filtered, not damped. Investigate: D-term LPF cutoff (try 100–150 Hz), \
             RPM filter enabled, prop balance, motor health."
        }
        (1, PidTerm::I) => {
            "Repeated I-bumps suggest persistent error the PID can't track — often \
             rate config, RC link delay, or motor desync. Investigate: rcCommand \
             filtering, rates curve, motor protocol/timing."
        }
        (-1, PidTerm::P) => {
            "Repeated P-cuts suggest the system is hunting at a frequency the PID \
             can't suppress. Investigate: gyro LPF cutoff (try 90–120 Hz), \
             frame stiffness, motor mounting."
        }
        _ => {
            "Investigate filter chain (gyro/D-term LPF), RPM filter, and mechanical \
             health (prop balance, frame stiffness) before further PID adjustments."
        }
    };
    format!(
        "{:?} {:?} has been pushed {} for {} iterations without resolving — \
         this is likely a filter / mechanical issue, not a PID problem. {}",
        rec.axis, rec.term, direction, iterations, followup
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::domain::Priority;

    fn rec(axis: Axis, term: PidTerm, current: f32, recommended: f32) -> PidRecommendation {
        PidRecommendation {
            axis,
            term,
            current_value: current,
            recommended_value: recommended,
            reason: "test".to_string(),
            priority: Priority::Medium,
        }
    }

    fn change(axis: Axis, term: PidTerm, delta: i32) -> PidChangeRecord {
        PidChangeRecord { axis, term, delta }
    }

    #[test]
    fn empty_history_is_no_op() {
        let recs = vec![rec(Axis::Roll, PidTerm::D, 30.0, 33.0)];
        let out = apply_convergence_suppression(recs, &[], DEFAULT_MIN_REPEATED);
        assert_eq!(out.kept.len(), 1);
        assert!(out.suppressed.is_empty());
    }

    #[test]
    fn min_repeated_zero_is_no_op() {
        let recs = vec![rec(Axis::Roll, PidTerm::D, 30.0, 33.0)];
        let history = vec![
            change(Axis::Roll, PidTerm::D, 3),
            change(Axis::Roll, PidTerm::D, 4),
        ];
        let out = apply_convergence_suppression(recs, &history, 0);
        assert_eq!(out.kept.len(), 1);
        assert!(out.suppressed.is_empty());
    }

    #[test]
    fn single_prior_change_does_not_trigger() {
        // Only one prior iteration moved D up; one is not enough to call
        // it convergence — that's a normal corrective tune.
        let recs = vec![rec(Axis::Roll, PidTerm::D, 30.0, 33.0)];
        let history = vec![change(Axis::Roll, PidTerm::D, 3)];
        let out = apply_convergence_suppression(recs, &history, DEFAULT_MIN_REPEATED);
        assert_eq!(out.kept.len(), 1);
        assert!(out.suppressed.is_empty());
    }

    /// The MEDIUM FUCKER motivating scenario: Roll D has been bumped up in
    /// the last 2 tunes (28 → 31 → 34), and now the analyzer wants to bump
    /// it again to 37. Item 5 must drop the rec and emit an advisory.
    #[test]
    fn three_consecutive_same_direction_pushes_get_suppressed() {
        let recs = vec![rec(Axis::Roll, PidTerm::D, 34.0, 37.0)];
        let history = vec![
            change(Axis::Roll, PidTerm::D, 3), // 28 → 31
            change(Axis::Roll, PidTerm::D, 3), // 31 → 34
        ];
        let out = apply_convergence_suppression(recs, &history, DEFAULT_MIN_REPEATED);
        assert!(out.kept.is_empty(), "rec must be suppressed");
        assert_eq!(out.suppressed.len(), 1);
        let s = &out.suppressed[0];
        assert_eq!(s.iterations_in_direction, 3);
        assert!(
            s.advisory.contains("filter") || s.advisory.contains("mechanical"),
            "advisory must redirect to filter/mechanical follow-up: {}",
            s.advisory
        );
        assert!(
            s.advisory.contains("Roll") && s.advisory.contains("D"),
            "advisory must name the gain: {}",
            s.advisory
        );
    }

    #[test]
    fn opposite_direction_after_runaway_is_kept() {
        // History: 2 consecutive D-bumps. New rec wants to *cut* D — that's
        // exactly the corrective move the user needs and must NOT be
        // suppressed by the convergence detector.
        let recs = vec![rec(Axis::Roll, PidTerm::D, 34.0, 31.0)];
        let history = vec![
            change(Axis::Roll, PidTerm::D, 3),
            change(Axis::Roll, PidTerm::D, 3),
        ];
        let out = apply_convergence_suppression(recs, &history, DEFAULT_MIN_REPEATED);
        assert_eq!(
            out.kept.len(),
            1,
            "opposite-direction recovery rec must survive"
        );
        assert!(out.suppressed.is_empty());
    }

    #[test]
    fn mixed_history_does_not_trigger() {
        // Last two D changes were +3, then -2. The most recent move was a
        // cut, so the system isn't "stuck pushing up". A new bump rec
        // is fine — possibly it overshot and now we want to nudge back.
        let recs = vec![rec(Axis::Roll, PidTerm::D, 32.0, 35.0)];
        let history = vec![
            change(Axis::Roll, PidTerm::D, 3),  // up
            change(Axis::Roll, PidTerm::D, -2), // down
        ];
        let out = apply_convergence_suppression(recs, &history, DEFAULT_MIN_REPEATED);
        assert_eq!(out.kept.len(), 1);
        assert!(out.suppressed.is_empty());
    }

    #[test]
    fn other_axis_history_is_ignored() {
        // Roll D has been pushed up twice; new rec is for *Pitch* D. The
        // convergence on Roll says nothing about Pitch — must keep.
        let recs = vec![rec(Axis::Pitch, PidTerm::D, 30.0, 33.0)];
        let history = vec![
            change(Axis::Roll, PidTerm::D, 3),
            change(Axis::Roll, PidTerm::D, 3),
        ];
        let out = apply_convergence_suppression(recs, &history, DEFAULT_MIN_REPEATED);
        assert_eq!(out.kept.len(), 1);
        assert!(out.suppressed.is_empty());
    }

    #[test]
    fn other_term_history_is_ignored() {
        // P has been cut twice; new rec is for D. Different term — keep.
        let recs = vec![rec(Axis::Roll, PidTerm::D, 30.0, 33.0)];
        let history = vec![
            change(Axis::Roll, PidTerm::P, -3),
            change(Axis::Roll, PidTerm::P, -2),
        ];
        let out = apply_convergence_suppression(recs, &history, DEFAULT_MIN_REPEATED);
        assert_eq!(out.kept.len(), 1);
        assert!(out.suppressed.is_empty());
    }

    #[test]
    fn zero_delta_history_entries_are_skipped() {
        // History has many "this iteration didn't touch Roll D" rows mixed
        // with two real D-bumps. The detector must look past the zeros to
        // the actual changes and still trigger.
        let recs = vec![rec(Axis::Roll, PidTerm::D, 34.0, 37.0)];
        let history = vec![
            change(Axis::Roll, PidTerm::D, 3),
            change(Axis::Roll, PidTerm::D, 0), // unchanged iteration
            change(Axis::Roll, PidTerm::D, 0),
            change(Axis::Roll, PidTerm::D, 3),
        ];
        let out = apply_convergence_suppression(recs, &history, DEFAULT_MIN_REPEATED);
        assert!(
            out.kept.is_empty(),
            "zero deltas must not break the detector"
        );
        assert_eq!(out.suppressed.len(), 1);
    }

    #[test]
    fn no_op_rec_passes_through() {
        // Rec where recommended ≈ current (the FC integer rounding will make
        // it a no-op anyway). Don't suppress, just let it through and let
        // downstream code drop it on its own.
        let recs = vec![rec(Axis::Roll, PidTerm::D, 30.0, 30.2)];
        let history = vec![
            change(Axis::Roll, PidTerm::D, 3),
            change(Axis::Roll, PidTerm::D, 3),
        ];
        let out = apply_convergence_suppression(recs, &history, DEFAULT_MIN_REPEATED);
        assert_eq!(
            out.kept.len(),
            1,
            "round-to-zero recs aren't convergence concerns"
        );
        assert!(out.suppressed.is_empty());
    }

    #[test]
    fn each_axis_term_evaluated_independently() {
        // Two recs in one batch: Roll D (converging) and Pitch I (single
        // prior bump only). Roll D suppressed, Pitch I kept.
        let recs = vec![
            rec(Axis::Roll, PidTerm::D, 34.0, 37.0),
            rec(Axis::Pitch, PidTerm::I, 80.0, 86.0),
        ];
        let history = vec![
            change(Axis::Roll, PidTerm::D, 3),
            change(Axis::Roll, PidTerm::D, 3),
            change(Axis::Pitch, PidTerm::I, 4), // only one prior I bump
        ];
        let out = apply_convergence_suppression(recs, &history, DEFAULT_MIN_REPEATED);
        assert_eq!(out.kept.len(), 1);
        assert!(matches!(out.kept[0].term, PidTerm::I));
        assert_eq!(out.suppressed.len(), 1);
        assert!(matches!(out.suppressed[0].original.term, PidTerm::D));
    }

    #[test]
    fn advisory_for_d_bump_mentions_filter() {
        let recs = vec![rec(Axis::Roll, PidTerm::D, 34.0, 37.0)];
        let history = vec![
            change(Axis::Roll, PidTerm::D, 3),
            change(Axis::Roll, PidTerm::D, 3),
        ];
        let out = apply_convergence_suppression(recs, &history, DEFAULT_MIN_REPEATED);
        let advisory = &out.suppressed[0].advisory;
        assert!(
            advisory.to_lowercase().contains("lpf") || advisory.to_lowercase().contains("filter"),
            "D-bump advisory must point at LPF / filter chain: {}",
            advisory
        );
    }

    #[test]
    fn advisory_for_p_cut_mentions_gyro_lpf() {
        let recs = vec![rec(Axis::Roll, PidTerm::P, 40.0, 36.0)];
        let history = vec![
            change(Axis::Roll, PidTerm::P, -3),
            change(Axis::Roll, PidTerm::P, -3),
        ];
        let out = apply_convergence_suppression(recs, &history, DEFAULT_MIN_REPEATED);
        let advisory = &out.suppressed[0].advisory;
        assert!(
            advisory.to_lowercase().contains("gyro lpf")
                || advisory.to_lowercase().contains("hunting"),
            "P-cut advisory must point at gyro LPF / hunting frequency: {}",
            advisory
        );
    }
}
