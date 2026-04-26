//! Calibration / golden tests for the analysis pipeline.
//!
//! Each test runs the parse → FFT → oscillation detection → recommendation
//! pipeline on a real Betaflight blackbox file from `test_data/`, then
//! snapshots a *summary* of the analysis report (counts and bucket totals,
//! not raw numerical values). Snapshots are stored in
//! `crates/drone-tuner-core/tests/snapshots/` and reviewed with
//! `cargo insta review`.
//!
//! Why summaries instead of full reports? Recommendation amplitudes drift
//! with FFT window choices and floating-point accumulation. Counts and
//! categories drift only when behaviour actually changes, which is what we
//! want a regression test to flag.
//!
//! ## Adding a new fixture
//!
//! 1. Drop a `.bbl` file in `test_data/`.
//! 2. Add a `#[test]` here that calls [`analyze_fixture`] with the filename.
//! 3. Run `cargo test -p drone-tuner-core --test calibration`. The first
//!    run creates a `.snap.new` file — review it with `cargo insta review`
//!    and accept if the summary looks right.
//! 4. Future runs fail-fast if the summary changes. Inspect the diff with
//!    `cargo insta review`; accept on intentional changes, debug otherwise.

use std::collections::BTreeMap;
use std::path::PathBuf;

use drone_tuner_core::domain::{
    AnalysisReport, FilterRecommendationType, IssueType, PidRecommendation, Severity,
};
use drone_tuner_core::{AnalysisEngine, BlackboxParser};
use serde::Serialize;

/// Compact, drift-resistant view of an [`AnalysisReport`].
///
/// Aggregates by category so floating-point jitter in frequencies/amplitudes
/// doesn't trigger false-positive snapshot diffs. A real algorithmic change
/// (e.g. a new oscillation gets detected, or a recommendation type stops
/// being emitted) flips a count and gets caught.
#[derive(Serialize)]
struct ReportSummary {
    issues_by_severity: BTreeMap<&'static str, usize>,
    issues_by_type: BTreeMap<&'static str, usize>,
    filter_recommendations_by_type: BTreeMap<&'static str, usize>,
    pid_recommendations_by_axis_term: BTreeMap<String, usize>,
    tune_quality_bucket: &'static str,
}

impl ReportSummary {
    fn from_report(report: &AnalysisReport) -> Self {
        let mut issues_by_severity = BTreeMap::new();
        let mut issues_by_type = BTreeMap::new();

        for issue in &report.detected_issues {
            *issues_by_severity
                .entry(severity_label(&issue.severity))
                .or_insert(0) += 1;
            *issues_by_type
                .entry(issue_type_label(&issue.issue_type))
                .or_insert(0) += 1;
        }

        let mut filter_recommendations_by_type = BTreeMap::new();
        for rec in &report.filter_recommendations {
            *filter_recommendations_by_type
                .entry(filter_type_label(&rec.recommendation_type))
                .or_insert(0) += 1;
        }

        let pid_recommendations_by_axis_term = pid_summary(&report.pid_recommendations);

        Self {
            issues_by_severity,
            issues_by_type,
            filter_recommendations_by_type,
            pid_recommendations_by_axis_term,
            tune_quality_bucket: bucket_quality(report.tune_quality_score),
        }
    }
}

fn severity_label(s: &Severity) -> &'static str {
    match s {
        Severity::Critical => "critical",
        Severity::High => "high",
        Severity::Medium => "medium",
        Severity::Low => "low",
    }
}

fn issue_type_label(t: &IssueType) -> &'static str {
    match t {
        IssueType::PTermOscillation { .. } => "p_term_oscillation",
        IssueType::DTermOscillation { .. } => "d_term_oscillation",
        IssueType::MechanicalResonance { .. } => "mechanical_resonance",
        IssueType::Imbalance { .. } => "imbalance",
        IssueType::LooseHardware => "loose_hardware",
        IssueType::InsufficientFiltering { .. } => "insufficient_filtering",
        IssueType::ExcessiveFiltering { .. } => "excessive_filtering",
    }
}

fn filter_type_label(t: &FilterRecommendationType) -> &'static str {
    match t {
        FilterRecommendationType::AdjustGyroLowpass { .. } => "adjust_gyro_lowpass",
        FilterRecommendationType::ConfigureGyroNotch { .. } => "configure_gyro_notch",
        FilterRecommendationType::AdjustDynamicNotch { .. } => "adjust_dynamic_notch",
        FilterRecommendationType::ConfigureRpmFilter { .. } => "configure_rpm_filter",
        FilterRecommendationType::AdjustDtermLowpass { .. } => "adjust_dterm_lowpass",
        FilterRecommendationType::AdjustYawLowpass { .. } => "adjust_yaw_lowpass",
    }
}

fn pid_summary(recs: &[PidRecommendation]) -> BTreeMap<String, usize> {
    let mut out = BTreeMap::new();
    for rec in recs {
        let key = format!("{:?}_{:?}", rec.axis, rec.term).to_lowercase();
        *out.entry(key).or_insert(0) += 1;
    }
    out
}

/// Coarse quality buckets so a few-tenths-of-a-percent score drift does
/// not invalidate snapshots.
fn bucket_quality(score: f32) -> &'static str {
    match score {
        s if s < 0.0 => "invalid",
        s if s < 25.0 => "very_poor",
        s if s < 50.0 => "poor",
        s if s < 75.0 => "fair",
        s if s < 90.0 => "good",
        _ => "excellent",
    }
}

fn workspace_root() -> PathBuf {
    // CARGO_MANIFEST_DIR resolves to .../crates/drone-tuner-core, walk up two.
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .expect("workspace root should be reachable from CARGO_MANIFEST_DIR")
        .to_path_buf()
}

fn analyze_fixture(filename: &str) -> AnalysisReport {
    let bbl_path = workspace_root().join("test_data").join(filename);
    let data = std::fs::read(&bbl_path)
        .unwrap_or_else(|e| panic!("failed to read fixture {}: {}", bbl_path.display(), e));

    let mut parser = BlackboxParser::new();
    let session = parser
        .parse_file(&data)
        .unwrap_or_else(|e| panic!("failed to parse {}: {}", filename, e));

    let mut engine = AnalysisEngine::new();
    engine
        .analyze(&session)
        .unwrap_or_else(|e| panic!("failed to analyze {}: {}", filename, e))
}

/// Small (~4 MB) fixture committed to git so this test runs in CI.
#[test]
fn calibration_btfl_all_old() {
    let report = analyze_fixture("btfl_all_old.bbl");
    insta::assert_json_snapshot!(ReportSummary::from_report(&report));
}

/// Large (~16 MB) fixture not tracked in git. Run locally with
/// `cargo test --test calibration -- --ignored` after dropping the
/// matching `.bbl` into `test_data/`.
#[test]
#[ignore = "requires test_data/btfl_all.bbl (not tracked, ~16 MB)"]
fn calibration_btfl_all() {
    let report = analyze_fixture("btfl_all.bbl");
    insta::assert_json_snapshot!(ReportSummary::from_report(&report));
}

/// Large (~16 MB) fixture not tracked in git.
#[test]
#[ignore = "requires test_data/btfl_all_pre.bbl (not tracked, ~16 MB)"]
fn calibration_btfl_all_pre() {
    let report = analyze_fixture("btfl_all_pre.bbl");
    insta::assert_json_snapshot!(ReportSummary::from_report(&report));
}

// ===========================================================================
// Labelled synthetic cases
//
// The .bbl-based snapshot tests above guard against drift but can't tell us
// whether the analyser is fundamentally right — they just freeze its current
// behaviour. The cases below construct synthetic FlightSessions with a
// single, *known* property (clean signal, P-term oscillation, mechanical
// resonance, etc.) and assert a specific shape of recommendation comes out.
//
// These are the regression tests for the algorithm's actual correctness.
// ===========================================================================

mod labelled {
    use super::*;
    use chrono::Utc;
    use drone_tuner_core::domain::{
        EnvironmentalConditions, FlightMetadata, FlightSession, HardwareConfiguration,
        PidErrorTrace, PilotProfile, RcCommandTrace, TelemetryData, TimeSeriesVector3,
    };
    use nalgebra::Vector3;
    use uuid::Uuid;

    const SAMPLE_RATE: f32 = 1000.0;
    const SAMPLE_COUNT: usize = 16384;

    /// Build a `FlightSession` with a single sine-wave oscillation injected
    /// onto one gyro axis, on top of mild base motion and noise.
    ///
    /// `axis_index`: 0 = roll, 1 = pitch, 2 = yaw.
    fn synthetic_session(frequency: f32, amplitude: f32, axis_index: usize) -> FlightSession {
        assert!(axis_index < 3, "axis_index must be 0, 1, or 2");

        let mut gyro = TimeSeriesVector3::with_capacity(SAMPLE_COUNT);
        let accel = {
            let mut a = TimeSeriesVector3::with_capacity(SAMPLE_COUNT);
            for _ in 0..SAMPLE_COUNT {
                a.push(Vector3::new(0.0, 0.0, 9.81));
            }
            a
        };

        // Deterministic LCG so tests are reproducible across runs without
        // pulling in a rand crate dependency.
        let mut rng_state: u64 = 0xDEADBEEF;
        let mut next_noise = || -> f32 {
            rng_state = rng_state
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            ((rng_state >> 33) as f32) / (u32::MAX as f32) - 0.5
        };

        for i in 0..SAMPLE_COUNT {
            let t = i as f32 / SAMPLE_RATE;
            let mut g = [0.0f32; 3];
            // Slow base motion on every axis so the analyser has signal.
            for (j, slot) in g.iter_mut().enumerate() {
                *slot += 2.0 * (2.0 * std::f32::consts::PI * 1.5 * t + j as f32).sin();
                *slot += 0.5 * next_noise();
            }
            // Inject the oscillation on the requested axis.
            g[axis_index] += amplitude * (2.0 * std::f32::consts::PI * frequency * t).sin();
            gyro.push(Vector3::new(g[0], g[1], g[2]));
        }

        FlightSession {
            metadata: FlightMetadata {
                session_id: Uuid::new_v4(),
                timestamp: Utc::now(),
                duration_ms: (SAMPLE_COUNT as f32 / SAMPLE_RATE * 1000.0) as u64,
                hardware: HardwareConfiguration::test_default(),
                environment: EnvironmentalConditions::default(),
                pilot: PilotProfile::default(),
            },
            telemetry: TelemetryData {
                sample_rate: SAMPLE_RATE,
                gyro,
                accel,
                motor: Vec::new(),
                pid_error: PidErrorTrace {
                    roll: vec![0.0; SAMPLE_COUNT],
                    pitch: vec![0.0; SAMPLE_COUNT],
                    yaw: vec![0.0; SAMPLE_COUNT],
                },
                rc_commands: RcCommandTrace {
                    roll: vec![0.0; SAMPLE_COUNT],
                    pitch: vec![0.0; SAMPLE_COUNT],
                    yaw: vec![0.0; SAMPLE_COUNT],
                    throttle: vec![0.5; SAMPLE_COUNT],
                },
                loop_time_variance: 0.0,
                cpu_load: Vec::new(),
            },
            events: Vec::new(),
            analysis_results: None,
        }
    }

    fn analyze(session: &FlightSession) -> AnalysisReport {
        AnalysisEngine::new()
            .analyze(session)
            .expect("analysis on synthetic session must succeed")
    }

    /// 400 Hz mechanical resonance on roll → analyser must emit a notch
    /// recommendation (gyro notch or dynamic notch).
    #[test]
    fn labelled_mechanical_resonance_400hz_emits_notch_filter() {
        let session = synthetic_session(400.0, 15.0, 0);
        let report = analyze(&session);

        let notch_count = report
            .filter_recommendations
            .iter()
            .filter(|rec| {
                matches!(
                    rec.recommendation_type,
                    FilterRecommendationType::ConfigureGyroNotch { .. }
                        | FilterRecommendationType::AdjustDynamicNotch { .. }
                )
            })
            .count();

        assert!(
            notch_count >= 1,
            "expected at least one notch filter recommendation for a 400 Hz \
             resonance, got {} filter recommendations: {:#?}",
            report.filter_recommendations.len(),
            report
                .filter_recommendations
                .iter()
                .map(|r| &r.recommendation_type)
                .collect::<Vec<_>>()
        );
    }

    /// 25 Hz oscillation on roll → analyser must classify it as a P-term
    /// oscillation (or a low-frequency mechanical resonance). Either way
    /// it should NOT be classified as motor noise — that would mean the
    /// frequency-band classifier is broken.
    #[test]
    fn labelled_p_term_25hz_not_classified_as_motor_noise() {
        let session = synthetic_session(25.0, 12.0, 0);
        let report = analyze(&session);

        let motor_noise_issues = report
            .detected_issues
            .iter()
            .filter(|i| matches!(i.issue_type, IssueType::Imbalance { .. }))
            .count();
        let acceptable_issues = report
            .detected_issues
            .iter()
            .filter(|i| {
                matches!(
                    i.issue_type,
                    IssueType::PTermOscillation { .. } | IssueType::MechanicalResonance { .. }
                )
            })
            .count();

        assert_eq!(
            motor_noise_issues,
            0,
            "25 Hz oscillation must not be classified as motor noise; \
             issues: {:#?}",
            report
                .detected_issues
                .iter()
                .map(|i| &i.issue_type)
                .collect::<Vec<_>>()
        );
        assert!(
            acceptable_issues >= 1,
            "expected the analyser to detect a P-term oscillation or a \
             low-frequency resonance for a strong 25 Hz signal; \
             got: {:#?}",
            report
                .detected_issues
                .iter()
                .map(|i| &i.issue_type)
                .collect::<Vec<_>>()
        );
    }

    /// 120 Hz oscillation on pitch → must be classified as D-term territory
    /// (or mechanical resonance, which the analyser sometimes prefers when
    /// Q is high). Crucially: never as P-term.
    #[test]
    fn labelled_d_term_120hz_not_classified_as_p_term() {
        let session = synthetic_session(120.0, 8.0, 1);
        let report = analyze(&session);

        let p_term_issues = report
            .detected_issues
            .iter()
            .filter(|i| matches!(i.issue_type, IssueType::PTermOscillation { .. }))
            .count();

        assert_eq!(
            p_term_issues,
            0,
            "120 Hz oscillation should not be classified as a P-term \
             oscillation (P-term band is 5..50 Hz); issues: {:#?}",
            report
                .detected_issues
                .iter()
                .map(|i| &i.issue_type)
                .collect::<Vec<_>>()
        );
    }

    /// A synthetic session with no oscillation and only mild base motion
    /// should not generate critical-severity issues. Nothing's broken,
    /// nothing should panic the pilot.
    #[test]
    fn labelled_clean_flight_has_no_critical_issues() {
        // Amplitude 0 → zero injected oscillation, just baseline motion + noise.
        let session = synthetic_session(50.0, 0.0, 0);
        let report = analyze(&session);

        let critical_issues = report
            .detected_issues
            .iter()
            .filter(|i| matches!(i.severity, Severity::Critical))
            .count();

        assert_eq!(
            critical_issues,
            0,
            "clean flight should not produce critical issues; \
             issues: {:#?}",
            report
                .detected_issues
                .iter()
                .map(|i| (&i.issue_type, &i.severity))
                .collect::<Vec<_>>()
        );
    }
}
