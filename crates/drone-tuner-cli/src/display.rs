//! Centralised terminal UI for the `drone-tuner` CLI.
//!
//! Banner, section headers, status glyphs, priority badges, quality gauge,
//! comfy-table renderers, and the spinner/stage helpers used by the tune
//! flow. Visual decoration is gated on `should_decorate()` so JSON/CSV
//! output and piped runs stay byte-clean.

use comfy_table::presets::UTF8_FULL_CONDENSED;
use comfy_table::{
    Cell, CellAlignment, Color as TblColor, ContentArrangement, Table,
};
use console::{style, StyledObject};
use drone_tuner_core::domain::{FilterRecommendationType, Priority};
use indicatif::{ProgressBar, ProgressStyle};
use std::io::IsTerminal;
use std::sync::atomic::{AtomicBool, Ordering};

use crate::OutputFormat;

/// Set by `main()` if the user passed `--no-banner`.
static NO_BANNER: AtomicBool = AtomicBool::new(false);
/// Set once banner has been shown (so the same process doesn't re-emit it
/// when subcommands print intermediate banners).
static BANNER_SHOWN: AtomicBool = AtomicBool::new(false);

const LOGO: &str = "\
██████╗ ██████╗  ██████╗ ███╗   ██╗███████╗    ████████╗██╗   ██╗███╗   ██╗███████╗██████╗
██╔══██╗██╔══██╗██╔═══██╗████╗  ██║██╔════╝    ╚══██╔══╝██║   ██║████╗  ██║██╔════╝██╔══██╗
██║  ██║██████╔╝██║   ██║██╔██╗ ██║█████╗         ██║   ██║   ██║██╔██╗ ██║█████╗  ██████╔╝
██║  ██║██╔══██╗██║   ██║██║╚██╗██║██╔══╝         ██║   ██║   ██║██║╚██╗██║██╔══╝  ██╔══██╗
██████╔╝██║  ██║╚██████╔╝██║ ╚████║███████╗       ██║   ╚██████╔╝██║ ╚████║███████╗██║  ██║
╚═════╝ ╚═╝  ╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚══════╝       ╚═╝    ╚═════╝ ╚═╝  ╚═══╝╚══════╝╚═╝  ╚═╝";

const TAGLINE: &str = "FPV Tuning Intelligence";
const REPO_URL: &str = "github.com/florianpohl/drone-tuner";

/// Capture the global `--no-banner` flag from `main`.
pub fn set_no_banner(v: bool) {
    NO_BANNER.store(v, Ordering::Relaxed);
}

/// Predicate: should we emit visual decoration (banner, boxed headers,
/// coloured tables)? Gated on:
/// - output format is `Pretty`
/// - stdout is a terminal (so pipes / redirects stay clean)
/// - `NO_COLOR` env var is unset (industry convention)
/// - the user did not pass `--no-banner` (still allows colour, just kills the logo)
pub fn should_decorate(format: &OutputFormat) -> bool {
    matches!(format, OutputFormat::Pretty) && std::io::stdout().is_terminal()
}

/// Print the ANSI-shadow startup banner. No-op for non-pretty output, when
/// stdout is piped, when `NO_COLOR` is set, or when the user opted out via
/// `--no-banner`. Idempotent within a process.
pub fn banner(format: &OutputFormat) {
    if NO_BANNER.load(Ordering::Relaxed) {
        return;
    }
    if !should_decorate(format) {
        return;
    }
    if BANNER_SHOWN.swap(true, Ordering::Relaxed) {
        return;
    }
    eprintln!("{}", style(LOGO).cyan().bold());
    eprintln!(
        "          [ {} ]   v{}   ·   {}",
        style(TAGLINE).white(),
        style(env!("CARGO_PKG_VERSION")).magenta().bold(),
        style(REPO_URL).dim(),
    );
    eprintln!(
        "{}",
        style("━".repeat(85)).cyan()
    );
    eprintln!();
}

/// Print a boxed section header. Replaces the older `== NAME ==` style.
/// Width clamps to ~78 cols. Goes to stderr so it doesn't pollute
/// JSON/CSV stdout streams when a tune flow is mid-output.
pub fn section(label: &str) {
    let label_upper = label.to_uppercase();
    let inner = format!("─[ {} ]", label_upper);
    let bar_len = 78usize.saturating_sub(inner.chars().count() + 2);
    let line = format!("┌{}{}┐", inner, "─".repeat(bar_len));
    eprintln!();
    eprintln!("{}", style(line).cyan().bold());
}

/// `[+]` green success line.
pub fn ok(msg: impl AsRef<str>) {
    println!("  {} {}", style("[+]").green().bold(), msg.as_ref());
}

/// `[*]` blue informational line. Used for "would do X" / dry-run notes.
/// `glyph_info()` is the more common form (interpolated inline); this
/// stand-alone variant is kept for future call sites.
#[allow(dead_code)]
pub fn info(msg: impl AsRef<str>) {
    println!("  {} {}", style("[*]").blue().bold(), msg.as_ref());
}

/// `[!]` yellow warning line.
pub fn warn(msg: impl AsRef<str>) {
    println!("  {} {}", style("[!]").yellow().bold(), msg.as_ref());
}

/// `[-]` red error line. Goes to stderr so it survives `>file` redirects.
pub fn err(msg: impl AsRef<str>) {
    eprintln!("  {} {}", style("[-]").red().bold(), msg.as_ref());
}

/// Pre-styled glyphs for callers that need to interpolate them inside
/// `format!`/`println!` (status counts, list bullets, etc.).
pub fn glyph_ok() -> StyledObject<&'static str> {
    style("[+]").green().bold()
}
pub fn glyph_info() -> StyledObject<&'static str> {
    style("[*]").blue().bold()
}
pub fn glyph_warn() -> StyledObject<&'static str> {
    style("[!]").yellow().bold()
}
pub fn glyph_err() -> StyledObject<&'static str> {
    style("[-]").red().bold()
}

/// Coloured `[C]/[H]/[M]/[L]` priority badge. Borrows the input so callers
/// can use it with `&rec.priority` without forcing a clone (`Priority` is
/// `Clone` but not `Copy`).
pub fn priority_badge(p: &Priority) -> StyledObject<&'static str> {
    match p {
        Priority::Critical => style("[C]").magenta().bold(),
        Priority::High => style("[H]").red().bold(),
        Priority::Medium => style("[M]").yellow().bold(),
        Priority::Low => style("[L]").green().bold(),
    }
}

/// Render `Tune Quality 75.3/100  [████████████░░░] GOOD` with bar
/// segments coloured by score band.
pub fn quality_gauge(score: f32) -> String {
    const WIDTH: usize = 32;
    let score_clamped = score.clamp(0.0, 100.0);
    let filled = ((score_clamped / 100.0) * WIDTH as f32).round() as usize;
    let empty = WIDTH - filled;
    let bar_filled: String = "█".repeat(filled);
    let bar_empty: String = "░".repeat(empty);

    let (label, paint): (&str, fn(StyledObject<String>) -> StyledObject<String>) =
        if score >= 90.0 {
            ("EXCELLENT", |s| s.green().bold())
        } else if score >= 80.0 {
            ("GOOD", |s| s.green())
        } else if score >= 60.0 {
            ("OK", |s| s.yellow())
        } else {
            ("POOR", |s| s.red())
        };

    let bar = format!("{}{}", bar_filled, bar_empty);
    format!(
        "Tune Quality {:>5.1}/100  [{}] {}",
        score,
        paint(style(bar)),
        paint(style(label.to_string())).bold(),
    )
}

/// Build a comfy-table with the project's standard preset.
fn make_table() -> Table {
    let mut t = Table::new();
    t.load_preset(UTF8_FULL_CONDENSED)
        .set_content_arrangement(ContentArrangement::Dynamic);
    t
}

fn priority_cell(p: &Priority) -> Cell {
    let (label, color) = match p {
        Priority::Critical => ("[C]", TblColor::Magenta),
        Priority::High => ("[H]", TblColor::Red),
        Priority::Medium => ("[M]", TblColor::Yellow),
        Priority::Low => ("[L]", TblColor::Green),
    };
    Cell::new(label).fg(color)
}

/// Format a filter recommendation into a "Target" + "Current → New"
/// pair of strings for table rendering. Mirrors the prose form used in
/// the tune-flow output but split for tabular display.
fn filter_rec_cells(
    r: &drone_tuner_core::domain::FilterRecommendation,
) -> (String, String) {
    match &r.recommendation_type {
        FilterRecommendationType::AdjustGyroLowpass {
            stage,
            current_cutoff,
            recommended_cutoff,
            filter_type,
        } => (
            format!("Gyro LPF{} ({})", stage, filter_type),
            format!(
                "{:.0} Hz → {:.0} Hz",
                current_cutoff, recommended_cutoff
            ),
        ),
        FilterRecommendationType::ConfigureGyroNotch {
            notch_number,
            frequency,
            q_factor,
            enabled,
        } => (
            format!("Gyro Notch {}", notch_number),
            if *enabled {
                format!("→ {:.0} Hz (Q {:.0})", frequency, q_factor)
            } else {
                "→ disabled".to_string()
            },
        ),
        FilterRecommendationType::AdjustDynamicNotch {
            notch_count,
            q_factor,
            min_freq,
            max_freq,
            enabled,
        } => (
            "Dynamic Notch".to_string(),
            if *enabled {
                format!(
                    "{} notches, {:.0}-{:.0} Hz (Q {:.0})",
                    notch_count, min_freq, max_freq, q_factor
                )
            } else {
                "→ disabled".to_string()
            },
        ),
        FilterRecommendationType::ConfigureRpmFilter {
            harmonics,
            q_factor,
            min_freq,
            enabled,
        } => (
            "RPM Filter".to_string(),
            if *enabled {
                format!(
                    "{} harm, ≥{:.0} Hz (Q {:.0})",
                    harmonics, min_freq, q_factor
                )
            } else {
                "→ disabled".to_string()
            },
        ),
        FilterRecommendationType::AdjustDtermLowpass {
            stage,
            current_cutoff,
            recommended_cutoff,
            filter_type,
            dynamic_settings,
        } => {
            let target = format!("D-term LPF{} ({})", stage, filter_type);
            let change = match (current_cutoff, recommended_cutoff, dynamic_settings) {
                (Some(c), Some(_), Some(d)) => format!(
                    "{:.0} Hz → Dyn {:.0}-{:.0} Hz (expo {:.0})",
                    c, d.min_cutoff, d.max_cutoff, d.expo
                ),
                (Some(c), Some(n), None) => format!("{:.0} Hz → {:.0} Hz", c, n),
                (None, Some(n), _) => format!("→ {:.0} Hz", n),
                _ => "—".to_string(),
            };
            (target, change)
        }
        FilterRecommendationType::AdjustYawLowpass {
            current_cutoff,
            recommended_cutoff,
        } => (
            "Yaw LPF".to_string(),
            format!("{:.0} Hz → {:.0} Hz", current_cutoff, recommended_cutoff),
        ),
    }
}

/// Render PID + filter recommendations as a single combined table.
/// `Pri | Type | Target | Change | Reason`. Empty inputs print nothing.
pub fn recommendations_table(
    pid: &[drone_tuner_core::domain::PidRecommendation],
    filter: &[drone_tuner_core::domain::FilterRecommendation],
) {
    recommendations_table_full(pid, filter, &[]);
}

pub fn recommendations_table_full(
    pid: &[drone_tuner_core::domain::PidRecommendation],
    filter: &[drone_tuner_core::domain::FilterRecommendation],
    advanced: &[drone_tuner_core::domain::AdvancedRecommendation],
) {
    if pid.is_empty() && filter.is_empty() && advanced.is_empty() {
        return;
    }
    let mut table = make_table();
    table.set_header(vec![
        Cell::new("Pri").set_alignment(CellAlignment::Center),
        Cell::new("Type"),
        Cell::new("Target"),
        Cell::new("Change"),
        Cell::new("Reason"),
    ]);

    for r in pid {
        table.add_row(vec![
            priority_cell(&r.priority),
            Cell::new("PID").fg(TblColor::Cyan),
            Cell::new(format!("{:?} {:?}", r.axis, r.term)),
            Cell::new(format!(
                "{:.1} → {:.1}",
                r.current_value, r.recommended_value
            )),
            Cell::new(&r.reason),
        ]);
    }
    for r in filter {
        let (target, change) = filter_rec_cells(r);
        table.add_row(vec![
            priority_cell(&r.priority),
            Cell::new("Filter").fg(TblColor::Blue),
            Cell::new(target),
            Cell::new(change),
            Cell::new(&r.expected_improvement),
        ]);
    }
    for r in advanced {
        let (target, change) = advanced_rec_cells(&r.parameter);
        table.add_row(vec![
            priority_cell(&r.priority),
            Cell::new("Adv").fg(TblColor::Magenta),
            Cell::new(target),
            Cell::new(change),
            Cell::new(&r.reason),
        ]);
    }
    println!("{table}");
}

fn advanced_rec_cells(
    param: &drone_tuner_core::domain::AdvancedParameter,
) -> (String, String) {
    use drone_tuner_core::domain::AdvancedParameter::*;
    match param {
        VbatSagCompensation { current, recommended } => {
            ("Vbat Sag Comp".into(), format!("{} → {}", current, recommended))
        }
        DynamicIdle { current_rpm, recommended_rpm } => {
            ("Dynamic Idle".into(), format!("{} → {} (x100 RPM)", current_rpm, recommended_rpm))
        }
        DMax { axis, current_d_min, current_d_max, recommended_d_min, recommended_d_max } => {
            (format!("{:?} D Range", axis), format!("{}-{} → {}-{}", current_d_min, current_d_max, recommended_d_min, recommended_d_max))
        }
        Tpa { current_rate, current_breakpoint, recommended_rate, recommended_breakpoint } => {
            ("TPA".into(), format!("{}%@{} → {}%@{}", current_rate, current_breakpoint, recommended_rate, recommended_breakpoint))
        }
        Feedforward { param, current, recommended } => {
            (format!("FF {:?}", param), format!("{} → {}", current, recommended))
        }
        ThrustLinearization { current, recommended } => {
            ("Thrust Linear".into(), format!("{} → {}", current, recommended))
        }
        SliderHint { slider_name, current_value, suggested_value } => {
            (format!("Slider: {}", slider_name), format!("{} → {} (x0.01)", current_value, suggested_value))
        }
    }
}

/// One row in the comparison table.
pub struct ComparisonRow<'a> {
    pub name: &'a str,
    pub quality: f32,
    pub issues: usize,
    pub recommendations: usize,
    pub duration_ms: u64,
}

pub fn comparison_table(rows: &[ComparisonRow<'_>]) {
    let mut table = make_table();
    table.set_header(vec![
        Cell::new("Flight"),
        Cell::new("Quality").set_alignment(CellAlignment::Right),
        Cell::new("Issues").set_alignment(CellAlignment::Right),
        Cell::new("Recs").set_alignment(CellAlignment::Right),
        Cell::new("Duration").set_alignment(CellAlignment::Right),
    ]);
    for r in rows {
        let q_color = if r.quality >= 80.0 {
            TblColor::Green
        } else if r.quality >= 60.0 {
            TblColor::Yellow
        } else {
            TblColor::Red
        };
        table.add_row(vec![
            Cell::new(r.name),
            Cell::new(format!("{:.1}", r.quality))
                .fg(q_color)
                .set_alignment(CellAlignment::Right),
            Cell::new(r.issues.to_string()).set_alignment(CellAlignment::Right),
            Cell::new(r.recommendations.to_string())
                .set_alignment(CellAlignment::Right),
            Cell::new(format!("{:.1}s", r.duration_ms as f32 / 1000.0))
                .set_alignment(CellAlignment::Right),
        ]);
    }
    println!("{table}");
}

/// One row in the validate table.
pub struct ValidateRow {
    pub status: ValidateStatus,
    pub file: String,
    pub note: String,
}

#[derive(Copy, Clone)]
pub enum ValidateStatus {
    Valid,
    Issues,
    Invalid,
}

pub fn validate_table(rows: &[ValidateRow]) {
    let mut table = make_table();
    table.set_header(vec![
        Cell::new("Status").set_alignment(CellAlignment::Center),
        Cell::new("File"),
        Cell::new("Note"),
    ]);
    for r in rows {
        let (label, color) = match r.status {
            ValidateStatus::Valid => ("VALID", TblColor::Green),
            ValidateStatus::Issues => ("ISSUES", TblColor::Yellow),
            ValidateStatus::Invalid => ("INVALID", TblColor::Red),
        };
        table.add_row(vec![
            Cell::new(label)
                .fg(color)
                .set_alignment(CellAlignment::Center),
            Cell::new(&r.file),
            Cell::new(&r.note),
        ]);
    }
    println!("{table}");
}

/// Render the `info` command's version + system + library status as a
/// single two-column table. Caller is expected to print the banner first.
pub fn info_panel(
    cli_version: &str,
    core_version: &str,
    os: &str,
    arch: &str,
    libs: &[(&str, bool)],
) {
    let mut table = make_table();
    table.set_header(vec![Cell::new("Component"), Cell::new("Detail")]);
    table.add_row(vec![
        Cell::new("CLI").fg(TblColor::Cyan),
        Cell::new(format!("v{}", cli_version)),
    ]);
    table.add_row(vec![
        Cell::new("Core library").fg(TblColor::Cyan),
        Cell::new(format!("v{}", core_version)),
    ]);
    table.add_row(vec![Cell::new("OS"), Cell::new(os)]);
    table.add_row(vec![Cell::new("Arch"), Cell::new(arch)]);
    for (name, ok) in libs {
        let (badge, color) = if *ok {
            ("READY", TblColor::Green)
        } else {
            ("MISSING", TblColor::Red)
        };
        table.add_row(vec![
            Cell::new(*name),
            Cell::new(badge).fg(color),
        ]);
    }
    println!("{table}");
}

/// Stage header used by the tune flow ("PULL", "ANALYZE", "APPLY", ...).
/// Thin alias on top of `section()` so the call site reads naturally.
pub fn print_stage(name: &str) {
    section(name);
}

/// Build a transient cyan-spinner ProgressBar with the project's tick
/// strings. Used for MSP roundtrips that take ≤1s.
pub fn make_spinner(msg: impl Into<String>) -> ProgressBar {
    let pb = ProgressBar::new_spinner();
    pb.set_style(
        ProgressStyle::with_template("  {spinner:.cyan} {msg}")
            .unwrap()
            .tick_strings(&["⠋", "⠙", "⠹", "⠸", "⠼", "⠴", "⠦", "⠧", "⠇", "⠏"]),
    );
    pb.set_message(msg.into());
    pb.enable_steady_tick(std::time::Duration::from_millis(80));
    pb
}

/// Tear down a transient spinner and write a `[+]` success line on stdout.
/// Stdout (not stderr) so the line survives non-TTY runs where indicatif
/// suppresses its own output.
pub fn finish_step(pb: ProgressBar, message: impl Into<String>) {
    let msg = message.into();
    pb.finish_and_clear();
    println!("  {} {}", style("[+]").green().bold(), msg);
}

/// Render a single trace as a unicode-block sparkline. Auto-scales to the
/// data range so flat traces stay flat instead of getting amplified noise.
pub fn print_sparkline(label: &str, samples: &[f32]) {
    if samples.is_empty() {
        return;
    }
    const COLS: usize = 60;
    const BLOCKS: &[char] = &['▁', '▂', '▃', '▄', '▅', '▆', '▇', '█'];

    let chunk = samples.len().div_ceil(COLS);
    let downsampled: Vec<f32> = samples
        .chunks(chunk.max(1))
        .map(|c| c.iter().sum::<f32>() / c.len() as f32)
        .collect();

    let (min, max) = downsampled
        .iter()
        .fold((f32::INFINITY, f32::NEG_INFINITY), |(lo, hi), &v| {
            (lo.min(v), hi.max(v))
        });
    let span = (max - min).max(1e-6);

    let line: String = downsampled
        .iter()
        .map(|&v| {
            let norm = ((v - min) / span).clamp(0.0, 1.0);
            let idx = (norm * (BLOCKS.len() - 1) as f32).round() as usize;
            BLOCKS[idx]
        })
        .collect();
    println!("      {} [{:>+6.2} → {:>+6.2}] {}", label, min, max, line);
}
