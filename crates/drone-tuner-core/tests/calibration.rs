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
