//! Command-line interface for the FPV drone tuning platform.

mod display;
mod history;
mod paths;

use anyhow::{Context, Result};
use clap::{Args, Parser, Subcommand};
use console::{style, Term};
use drone_tuner_core::domain::FilterRecommendationType;
use drone_tuner_core::{AnalysisEngine, BlackboxParser, FlightSession};
use indicatif::{ProgressBar, ProgressStyle};
use std::path::{Path, PathBuf};
use std::time::Instant;

use tracing::{info, warn};
use walkdir::WalkDir;

/// FPV Drone Tuning Analysis Tool
#[derive(Parser)]
#[command(name = "drone-tuner")]
#[command(about = "Analyze FPV drone blackbox logs and get tuning recommendations")]
#[command(version)]
struct Cli {
    /// Enable verbose output
    #[arg(short, long, global = true)]
    verbose: bool,

    /// Show detailed blackbox parsing information
    #[arg(long, global = true)]
    detailed_info: bool,

    /// Output format
    #[arg(
        short = 'f',
        long = "output-format",
        value_enum,
        global = true,
        default_value = "pretty"
    )]
    output_format: OutputFormat,

    /// Suppress the startup ASCII banner (colours and tables stay)
    #[arg(long, global = true)]
    no_banner: bool,

    #[command(subcommand)]
    command: Commands,
}

/// Parse a unit-interval float (`0.0..=1.0`) from the CLI; rejects out-of-
/// range values with a clap-style error message.
fn parse_unit_interval(s: &str) -> std::result::Result<f32, String> {
    let v: f32 = s.parse().map_err(|e| format!("not a number: {e}"))?;
    if (0.0..=1.0).contains(&v) {
        Ok(v)
    } else {
        Err(format!("must be between 0.0 and 1.0, got {v}"))
    }
}

/// Parse a `--pull-chunk-size` argument. Below 256 bytes there's no
/// meaningful win over V1 legacy; above 32 KB risks overrunning the FC's
/// USB CDC TX buffer.
fn parse_chunk_size(s: &str) -> std::result::Result<u16, String> {
    let v: u32 = s
        .parse()
        .map_err(|e| format!("not a positive integer: {e}"))?;
    if !(256..=32_768).contains(&v) {
        return Err(format!("must be between 256 and 32768, got {v}"));
    }
    Ok(v as u16)
}

/// Parse a 1-based session index. Zero is rejected so `--session 0` is a
/// clean error rather than silently behaving like the default.
fn parse_one_based_index(s: &str) -> std::result::Result<usize, String> {
    let v: usize = s
        .parse()
        .map_err(|e| format!("not a positive integer: {e}"))?;
    if v == 0 {
        Err("session index must be 1 or greater".to_string())
    } else {
        Ok(v)
    }
}

/// Available output formats
#[derive(clap::ValueEnum, Clone, Debug)]
enum OutputFormat {
    /// Human-readable pretty output
    Pretty,
    /// JSON output for scripting
    Json,
    /// CSV output for spreadsheets
    Csv,
}

/// Available CLI commands
#[derive(Subcommand)]
enum Commands {
    /// Analyze a single blackbox file or directory
    Analyze(AnalyzeArgs),
    /// Compare multiple flights
    Compare(CompareArgs),
    /// Validate blackbox file format
    Validate(ValidateArgs),
    /// Connect to flight controller for real-time monitoring
    Monitor(MonitorArgs),
    /// Auto-tune PID parameters based on analysis
    Tune(TuneArgs),
    /// Export analysis results in various formats
    Export(ExportArgs),
    /// Show version and system information
    Info,
}

/// Arguments for the analyze command
#[derive(Args)]
struct AnalyzeArgs {
    /// Path to blackbox file or directory. Optional when `--pull-bbl` is
    /// set; in that case the file is downloaded from the FC's onboard
    /// dataflash.
    #[arg(value_name = "FILE_OR_DIR")]
    input: Option<PathBuf>,

    /// Output directory for results
    #[arg(short = 'd', long)]
    output_dir: Option<PathBuf>,

    /// Include detailed frequency analysis
    #[arg(long)]
    detailed: bool,

    /// Show comprehensive blackbox and configuration details
    #[arg(long)]
    show_details: bool,

    /// Minimum confidence threshold for recommendations (0.0..=1.0)
    #[arg(long, default_value = "0.7", value_parser = parse_unit_interval)]
    min_confidence: f32,

    /// Maximum number of files to process in batch
    #[arg(long, default_value = "100")]
    max_files: usize,

    /// Select specific session to analyze (1-based index, default: last session)
    #[arg(long, short, value_parser = parse_one_based_index)]
    session: Option<usize>,

    /// List all sessions in the blackbox file without analyzing
    #[arg(long)]
    list_sessions: bool,

    /// Show concise Betaflight sampling summary (rates, intervals)
    #[arg(long)]
    bb_summary: bool,

    /// Session selection strategy when multiple sessions are present
    /// Options: last, first, longest
    #[arg(long, value_parser = clap::builder::PossibleValuesParser::new(["last","first","longest"]))]
    session_strategy: Option<String>,

    /// Connection string. Accepts a device path (`/dev/ttyACM0`, `COM3`),
    /// a `serial://` or `simulator://` URI, or the literal `auto` to scan
    /// USB serial ports and pick the FC automatically. Auto-discover also
    /// kicks in when this flag is omitted but `--pull-bbl` is set.
    #[arg(long, value_name = "CONNECTION")]
    connection: Option<String>,

    /// Download the most recent blackbox session from the FC's onboard
    /// dataflash before analysis. The pulled bytes are written to a temp
    /// file (or `--keep-bbl <PATH>` if set) and fed into the same parse →
    /// analyze flow as a local file.
    #[arg(long)]
    pull_bbl: bool,

    /// Where to save the pulled .bbl file. Without this flag the file is
    /// written to a tempdir and left there. Has no effect without
    /// `--pull-bbl`.
    #[arg(long, value_name = "PATH")]
    keep_bbl: Option<PathBuf>,

    /// Erase the FC's onboard dataflash after a successful pull + analyze.
    /// Off by default. Has no effect without `--pull-bbl`.
    #[arg(long)]
    erase_after_pull: bool,

    /// Chunk size (in bytes) for the V2 dataflash read request. Default
    /// 1024. Range: 256–32768. Has no effect without `--pull-bbl`.
    #[arg(long, value_name = "BYTES", value_parser = parse_chunk_size)]
    pull_chunk_size: Option<u16>,
}

/// Arguments for the compare command
#[derive(Args)]
struct CompareArgs {
    /// Paths to blackbox files to compare
    #[arg(value_name = "FILES", required = true)]
    files: Vec<PathBuf>,

    /// Output file for comparison report
    #[arg(short, long)]
    output: Option<PathBuf>,
}

/// Arguments for the validate command
#[derive(Args)]
struct ValidateArgs {
    /// Path to blackbox file or directory
    #[arg(value_name = "FILE_OR_DIR")]
    input: PathBuf,

    /// Check for common issues
    #[arg(long)]
    check_issues: bool,
}

/// Arguments for the monitor command
#[derive(Args)]
struct MonitorArgs {
    /// Connection string (e.g., /dev/ttyUSB0 or COM3)
    #[arg(value_name = "CONNECTION")]
    connection: String,

    /// Update rate in Hz
    #[arg(long, default_value = "100")]
    rate: u32,

    /// Duration to monitor in seconds (0 = infinite)
    #[arg(long, default_value = "0")]
    duration: u64,

    /// Fields to monitor (comma-separated: gyro,accel,motors,pid_error,rc,battery,cpu,loop_time)
    #[arg(long, default_value = "gyro,pid_error")]
    fields: String,

    /// Log telemetry to file
    #[arg(long)]
    log_file: Option<PathBuf>,
}

/// Arguments for the tune command
#[derive(Args)]
struct TuneArgs {
    /// Path to blackbox file to analyze for tuning. Optional when
    /// `--pull-bbl` is set; in that case the file is downloaded from the
    /// FC's onboard dataflash.
    #[arg(value_name = "FILE")]
    input: Option<PathBuf>,

    /// Connection string. Accepts a device path (`/dev/ttyACM0`,
    /// `COM3`), a `serial://` or `simulator://` URI, or the literal
    /// `auto` to scan USB serial ports and pick the FC automatically.
    /// Auto-discover also kicks in when this flag is omitted but an
    /// FC operation is requested (`--pull-bbl`, `--apply-all`,
    /// `--auto-apply-safe`).
    #[arg(long, value_name = "CONNECTION")]
    connection: Option<String>,

    /// Download the most recent blackbox session from the FC's onboard
    /// dataflash before analysis. Requires `--connection`. The pulled
    /// bytes are written to a temp file (or `--keep-bbl <PATH>` if set)
    /// and fed into the same parse → tune flow as a local file.
    #[arg(long)]
    pull_bbl: bool,

    /// Where to save the pulled .bbl file. Without this flag the file is
    /// written to a tempdir and deleted on exit. Has no effect without
    /// `--pull-bbl`.
    #[arg(long, value_name = "PATH")]
    keep_bbl: Option<PathBuf>,

    /// Erase the FC's onboard dataflash after a successful pull. Off by
    /// default — most users want their flight history preserved across
    /// tune iterations. Has no effect without `--pull-bbl`.
    #[arg(long)]
    erase_after_pull: bool,

    /// Chunk size (in bytes) for the V2 dataflash read request. Default
    /// 1024 is conservative and works on every Betaflight 4.x FC tested.
    /// Larger values (2048, 4096) cut roundtrip count proportionally
    /// but some firmware/buffer combinations stall mid-pull at 4 KB.
    /// Range: 256–32768.
    #[arg(long, value_name = "BYTES", value_parser = parse_chunk_size)]
    pull_chunk_size: Option<u16>,

    /// Only show recommendations without applying
    #[arg(long)]
    dry_run: bool,

    /// Save the pre-change PID snapshot to a JSON file before applying.
    /// Path defaults to `tune-backup-<timestamp>.json` in the current
    /// directory; pass a custom path to override.
    #[arg(long, value_name = "PATH")]
    backup: Option<Option<PathBuf>>,

    /// Apply only Low/Medium priority recommendations. Without this flag,
    /// no recommendations are applied unless --apply-all is set.
    #[arg(long)]
    auto_apply_safe: bool,

    /// Apply ALL recommendations (including Critical/High priority).
    /// Mutually exclusive with --auto-apply-safe.
    #[arg(long, conflicts_with = "auto_apply_safe")]
    apply_all: bool,

    /// After applying changes, persist them to the FC's non-volatile
    /// memory (EEPROM_WRITE). Without this flag, changes are RAM-only
    /// and lost on power cycle — which is itself a useful safety net.
    #[arg(long)]
    save_eeprom: bool,

    /// Skip filter recommendations even when --auto-apply-safe / --apply-all
    /// is set. By default filter recs are written via MSP2_COMMON_SET_SETTING
    /// alongside PID recs; pass this flag to apply only PID changes.
    #[arg(long)]
    skip_filters: bool,

    /// Session selection strategy when multiple sessions are present.
    /// Options: last (default), first, longest.
    #[arg(long)]
    session_strategy: Option<String>,

    /// Restore PID values from a previously saved tune-backup-*.json file.
    /// Bypasses the pull/analyze flow entirely — connects to the FC, reads
    /// the current PIDs (for the diff), writes the backup's PIDs with
    /// rollback safety net, and (with --save-eeprom) persists. Use this
    /// to recover from a tune iteration that made things worse.
    #[arg(
        long,
        value_name = "PATH",
        conflicts_with_all = ["pull_bbl", "auto_apply_safe", "apply_all", "dry_run", "input"]
    )]
    restore: Option<PathBuf>,

    /// Anchor recommendations against a baseline tune. Each rec's
    /// proposed value is clamped to ±15% of the baseline value (not
    /// ±15% of current), so gains can't drift unboundedly across
    /// iterations. Pass a path to a `tune-backup-*.json` file. Pass
    /// `none` to opt out of the clamp explicitly. When omitted, the
    /// CLI nudges you to pass one but does not block the tune.
    #[arg(long, value_name = "PATH")]
    baseline: Option<String>,
}

/// Arguments for the export command
#[derive(Args)]
struct ExportArgs {
    /// Path to blackbox file or analysis results
    #[arg(value_name = "FILE")]
    input: PathBuf,

    /// Output file path
    #[arg(short, long, value_name = "FILE")]
    output: PathBuf,

    /// Export format (matlab, python, csv, json)
    #[arg(long, default_value = "csv")]
    format: String,

    /// Include raw telemetry data
    #[arg(long)]
    include_raw: bool,

    /// Include frequency analysis results
    #[arg(long)]
    include_fft: bool,
}

#[tokio::main]
async fn main() -> Result<()> {
    let cli = Cli::parse();

    // Initialize logging
    init_logging(cli.verbose)?;

    // Banner: gated on pretty + TTY + !NO_COLOR + !--no-banner. Goes to
    // stderr so JSON/CSV consumers piping stdout never see it.
    display::set_no_banner(cli.no_banner);
    display::banner(&cli.output_format);

    // Execute the command
    match cli.command {
        Commands::Analyze(args) => {
            analyze_command(args, cli.output_format, cli.detailed_info).await
        }
        Commands::Compare(args) => compare_command(args, cli.output_format).await,
        Commands::Validate(args) => validate_command(args, cli.output_format).await,
        Commands::Monitor(args) => monitor_command(args, cli.output_format).await,
        Commands::Tune(args) => tune_command(args, cli.output_format).await,
        Commands::Export(args) => export_command(args, cli.output_format).await,
        Commands::Info => info_command().await,
    }
}

/// Initialize logging based on verbosity level
fn init_logging(verbose: bool) -> Result<()> {
    use tracing_subscriber::filter::{LevelFilter, Targets};
    use tracing_subscriber::layer::SubscriberExt;
    use tracing_subscriber::util::SubscriberInitExt;

    let level = if verbose {
        tracing::Level::DEBUG
    } else {
        tracing::Level::INFO
    };

    // The upstream `blackbox_log` crate emits ERROR for every missing
    // session header (e.g. `Field S predictor`, routine on FCs that aborted
    // a log mid-flush). Our analysis doesn't run on that crate, so its
    // noise is just terminal spam. Silence it completely unless --verbose,
    // in which case it's gated to WARN-and-up for diagnostic value.
    let blackbox_log_level = if verbose {
        LevelFilter::WARN
    } else {
        LevelFilter::OFF
    };
    let filter = Targets::new()
        .with_default(level)
        .with_target("blackbox_log", blackbox_log_level);

    tracing_subscriber::registry()
        .with(
            tracing_subscriber::fmt::layer()
                .with_writer(std::io::stderr), // Keep logs separate from output
        )
        .with(filter)
        .init();

    Ok(())
}

/// Handle the analyze command
async fn analyze_command(
    args: AnalyzeArgs,
    output_format: OutputFormat,
    detailed_info: bool,
) -> Result<()> {
    let _term = Term::stdout();

    // Validate flag combos. Pull-only flags are no-ops without `--pull-bbl`,
    // and an explicit input together with `--pull-bbl` is ambiguous.
    if args.keep_bbl.is_some() && !args.pull_bbl {
        return Err(anyhow::anyhow!(
            "--keep-bbl only makes sense together with --pull-bbl"
        ));
    }
    if args.erase_after_pull && !args.pull_bbl {
        return Err(anyhow::anyhow!(
            "--erase-after-pull only makes sense together with --pull-bbl"
        ));
    }
    if args.pull_chunk_size.is_some() && !args.pull_bbl {
        return Err(anyhow::anyhow!(
            "--pull-chunk-size only makes sense together with --pull-bbl"
        ));
    }
    if args.input.is_some() && args.pull_bbl {
        return Err(anyhow::anyhow!(
            "Pass either a positional FILE/DIR OR --pull-bbl, not both"
        ));
    }

    // Resolve the input path: pull from FC, or use the user-provided file/dir.
    let files = match (&args.input, args.pull_bbl) {
        (Some(p), false) => find_blackbox_files(p, args.max_files)?,
        (None, true) => {
            let resolved = resolve_connection_raw(args.connection.as_deref(), true)?;
            let connection = resolved
                .as_deref()
                .expect("resolve_connection_raw guarantees Some when pull_bbl is true");
            let was_auto_discovered =
                !matches!(args.connection.as_deref(), Some(c) if c != "auto");
            if was_auto_discovered {
                println!("auto-discovered FC at {}", style(connection).bold());
            }
            let pulled = pull_bbl_from_fc(
                connection,
                args.keep_bbl.as_deref(),
                args.pull_chunk_size,
            )
            .await
            .context("Failed to pull BBL from flight controller")?;
            vec![pulled]
        }
        (None, false) => {
            return Err(anyhow::anyhow!(
                "No blackbox file given. Pass a path, or --pull-bbl to download from the FC."
            ));
        }
        (Some(_), true) => unreachable!("guarded above"),
    };

    if files.is_empty() {
        eprintln!("{}", style("No blackbox files found").red());
        return Ok(());
    }

    // Status messages go to stderr so machine-readable output formats
    // (CSV, JSON) keep stdout clean and parseable.
    eprintln!(
        "{} Found {} blackbox file(s) to analyze",
        display::glyph_ok(),
        files.len()
    );

    // Create progress bar
    let progress = ProgressBar::new(files.len() as u64);
    progress.set_style(
        ProgressStyle::default_bar()
            .template("[{elapsed_precise}] {bar:40.cyan/blue} {pos:>7}/{len:7} {msg}")
            .unwrap()
            .progress_chars("##-"),
    );

    let mut results = Vec::new();
    let mut engine = AnalysisEngine::new();

    for file in files {
        progress.set_message(format!("Processing {}", file.display()));

        match analyze_single_file(&mut engine, &file, &args).await {
            Ok(result) => {
                results.push((file.clone(), Ok(result)));
                info!("Successfully analyzed {}", file.display());
            }
            Err(e) => {
                // Handle session listing mode specially
                if e.to_string().contains("SESSION_LIST_COMPLETED") {
                    // Session listing was successful, but we don't want to continue analysis
                    info!("Session listing completed for {}", file.display());
                    // Skip adding to results for session listing
                } else {
                    warn!("Failed to analyze {}: {}", file.display(), e);
                    results.push((file.clone(), Err(e)));
                }
            }
        }

        progress.inc(1);
    }

    progress.finish_with_message("Analysis complete");

    // Combine global and command-specific detailed info flags
    let show_detailed = detailed_info || args.show_details;

    // Output results
    output_analysis_results(
        &results,
        &output_format,
        args.output_dir.as_ref(),
        show_detailed,
    )
    .await?;

    // Print summary — only in pretty mode so CSV/JSON stdout stays clean.
    if matches!(output_format, OutputFormat::Pretty) {
        let successful = results.iter().filter(|(_, result)| result.is_ok()).count();
        let failed = results.len() - successful;

        println!();
        println!("Analysis Summary");
        println!("  Successful: {}", style(successful).green());
        if failed > 0 {
            println!("  Failed: {}", style(failed).red());
        }
    }

    // Erase the FC's dataflash only after analysis succeeded — same safety
    // discipline as the tune flow, so a parse/analyze failure never destroys
    // log data the user might still want.
    if args.pull_bbl && args.erase_after_pull {
        let any_success = results.iter().any(|(_, r)| r.is_ok());
        if any_success {
            if let Some(connection) = resolve_connection_raw(args.connection.as_deref(), true)? {
                erase_dataflash_post_tune(&connection).await?;
            }
        }
    }

    Ok(())
}

/// Analyze a single blackbox file
async fn analyze_single_file(
    engine: &mut AnalysisEngine,
    file_path: &PathBuf,
    args: &AnalyzeArgs,
) -> Result<AnalysisResult> {
    let start_time = Instant::now();

    // Read and parse the file
    let data = std::fs::read(file_path)
        .with_context(|| format!("Failed to read file: {}", file_path.display()))?;

    // Handle list-sessions mode first
    if args.list_sessions {
        // Create a config for listing sessions only
        let list_config = drone_tuner_core::blackbox::ParsingConfig {
            list_sessions_only: true,
            ..Default::default()
        };

        let mut list_parser = BlackboxParser::with_config(list_config);

        // This will only log session information and return minimal data
        let _ = list_parser
            .parse_file(&data)
            .with_context(|| format!("Failed to analyze sessions in: {}", file_path.display()))?;

        // Optional concise BB summary printout
        if args.bb_summary {
            if let Some(summary) = list_parser.bb_summary() {
                println!("  BB Summary: {}", summary);
            }
        }

        // For list mode, we've already logged the session info.
        // The analysis function expects a result, so we'll need to handle this differently.
        // For now, return an error that gets handled gracefully in the caller.
        return Err(anyhow::anyhow!("SESSION_LIST_COMPLETED"));
    }

    // Create parser with session configuration for normal analysis
    let mut config = drone_tuner_core::blackbox::ParsingConfig::default();
    if let Some(session_index) = args.session {
        // Convert from 1-based CLI input to 0-based internal index
        config.selected_session = Some(session_index.saturating_sub(1));
    }

    // Map strategy string to core enum
    if let Some(strategy) = &args.session_strategy {
        config.session_strategy = Some(match strategy.as_str() {
            "first" => drone_tuner_core::blackbox::SessionStrategy::First,
            "longest" => drone_tuner_core::blackbox::SessionStrategy::Longest,
            _ => drone_tuner_core::blackbox::SessionStrategy::Last,
        });
    }

    let mut parser = BlackboxParser::with_config(config);
    let session = parser
        .parse_file(&data)
        .with_context(|| format!("Failed to parse blackbox file: {}", file_path.display()))?;

    // Optional concise BB summary printout for analyze mode
    if args.bb_summary {
        if let Some(summary) = parser.bb_summary() {
            println!("  BB Summary: {}", summary);
        }
    }

    info!(
        "Parsed {} samples in {:.2}s",
        session.telemetry.gyro.len(),
        start_time.elapsed().as_secs_f32()
    );

    // Perform analysis
    let report = engine
        .analyze(&session)
        .with_context(|| format!("Analysis failed for file: {}", file_path.display()))?;

    let analysis_time = start_time.elapsed();

    Ok(AnalysisResult {
        session,
        report,
        analysis_time,
        file_path: file_path.clone(),
    })
}

/// Find all blackbox files in the given path
fn find_blackbox_files(input_path: &PathBuf, max_files: usize) -> Result<Vec<PathBuf>> {
    let mut files = Vec::new();

    if input_path.is_file() {
        files.push(input_path.clone());
    } else if input_path.is_dir() {
        for entry in WalkDir::new(input_path) {
            let entry = entry?;
            let path = entry.path();

            if let Some(ext) = path.extension() {
                if ext == "bbl" || ext == "BBL" {
                    files.push(path.to_path_buf());

                    if files.len() >= max_files {
                        warn!(
                            "Reached maximum file limit of {}, stopping search",
                            max_files
                        );
                        break;
                    }
                }
            }
        }
    } else {
        // Path doesn't exist (or is e.g. a broken symlink). An empty
        // directory is a non-error case; a missing path is not.
        return Err(anyhow::anyhow!(
            "Failed to read file: {} does not exist",
            input_path.display()
        ));
    }

    Ok(files)
}

/// Handle the compare command
async fn compare_command(args: CompareArgs, output_format: OutputFormat) -> Result<()> {
    // Status messages go to stderr so machine-readable output formats
    // (CSV, JSON) keep stdout clean and parseable.
    eprintln!("Comparing {} flights", args.files.len());

    let mut engine = AnalysisEngine::new();
    let mut results = Vec::new();

    // Analyze each file
    for file in &args.files {
        match analyze_single_file(
            &mut engine,
            file,
            &AnalyzeArgs {
                input: Some(file.clone()),
                output_dir: None,
                detailed: true,
                show_details: false,
                min_confidence: 0.5,
                max_files: 1,
                session: None,
                list_sessions: false,
                bb_summary: false,
                session_strategy: None,
                connection: None,
                pull_bbl: false,
                keep_bbl: None,
                erase_after_pull: false,
                pull_chunk_size: None,
            },
        )
        .await
        {
            Ok(result) => results.push(result),
            Err(e) => {
                eprintln!(
                    "{} Failed to analyze {}: {}",
                    display::glyph_err(),
                    file.display(),
                    e
                );
                continue;
            }
        }
    }

    if results.len() < 2 {
        eprintln!(
            "{} Need at least 2 successfully analyzed flights to compare",
            display::glyph_warn()
        );
        return Ok(());
    }

    // Generate comparison
    let comparison = generate_comparison(&results)?;

    // Output comparison
    match output_format {
        OutputFormat::Pretty => print_comparison_pretty(&comparison),
        OutputFormat::Json => print_comparison_json(&comparison)?,
        OutputFormat::Csv => print_comparison_csv(&comparison)?,
    }

    Ok(())
}

/// Handle the validate command
async fn validate_command(args: ValidateArgs, _output_format: OutputFormat) -> Result<()> {
    let files = find_blackbox_files(&args.input, 1000)?;

    if files.is_empty() {
        display::warn("No blackbox files found");
        return Ok(());
    }

    display::section("Validate");
    println!();

    let mut rows: Vec<display::ValidateRow> = Vec::new();
    let mut valid_files = 0usize;
    let mut invalid_files = 0usize;
    let mut issue_lines: Vec<String> = Vec::new();

    for file in files {
        match validate_single_file(&file, args.check_issues).await {
            Ok(issues) => {
                valid_files += 1;
                if !issues.is_empty() {
                    issue_lines.push(format!("{}:", file.display()));
                    for issue in &issues {
                        issue_lines.push(format!("    • {}", issue));
                    }
                    rows.push(display::ValidateRow {
                        status: display::ValidateStatus::Issues,
                        file: file.display().to_string(),
                        note: format!("{} issue(s)", issues.len()),
                    });
                } else {
                    rows.push(display::ValidateRow {
                        status: display::ValidateStatus::Valid,
                        file: file.display().to_string(),
                        note: String::new(),
                    });
                }
            }
            Err(e) => {
                invalid_files += 1;
                rows.push(display::ValidateRow {
                    status: display::ValidateStatus::Invalid,
                    file: file.display().to_string(),
                    note: e.to_string(),
                });
            }
        }
    }

    display::validate_table(&rows);

    if !issue_lines.is_empty() {
        println!();
        println!("Issues:");
        for l in &issue_lines {
            println!("  {}", l);
        }
    }

    println!();
    display::ok(format!(
        "Valid: {}   Invalid: {}",
        style(valid_files).green().bold(),
        if invalid_files > 0 {
            style(invalid_files).red().bold().to_string()
        } else {
            "0".to_string()
        }
    ));

    Ok(())
}

/// Validate a single blackbox file
async fn validate_single_file(file_path: &PathBuf, check_issues: bool) -> Result<Vec<String>> {
    let data = std::fs::read(file_path)?;
    let mut parser = BlackboxParser::new();
    let session = parser.parse_file(&data)?;

    let mut issues = Vec::new();

    if check_issues {
        // Check for common issues
        if session.telemetry.gyro.len() < 1000 {
            issues.push("Flight too short (< 1000 samples)".to_string());
        }

        if session.telemetry.sample_rate < 500.0 {
            issues.push("Low sample rate (< 500Hz)".to_string());
        }

        if session.metadata.duration_ms < 5000 {
            issues.push("Flight duration too short (< 5 seconds)".to_string());
        }
    }

    Ok(issues)
}

/// Handle the monitor command
async fn monitor_command(_args: MonitorArgs, _output_format: OutputFormat) -> Result<()> {
    eprintln!(
        "{} `monitor` is EXPERIMENTAL — the MSP transport has never been validated against a real FC. Behaviour is not stable.",
        display::glyph_warn()
    );
    {
        use drone_tuner_core::realtime::*;

        println!("🔗 Connecting to flight controller at {}", _args.connection);

        // Parse fields to monitor
        let fields: Vec<&str> = _args.fields.split(',').map(|s| s.trim()).collect();
        let mut telemetry_fields = Vec::new();

        for field in &fields {
            match *field {
                "gyro" => telemetry_fields.push(TelemetryField::Gyro),
                "accel" => telemetry_fields.push(TelemetryField::Accelerometer),
                "motors" => telemetry_fields.push(TelemetryField::Motors),
                "pid_error" => telemetry_fields.push(TelemetryField::PidError),
                "rc" => telemetry_fields.push(TelemetryField::RcCommands),
                "battery" => telemetry_fields.push(TelemetryField::Battery),
                "cpu" => telemetry_fields.push(TelemetryField::CpuLoad),
                "loop_time" => telemetry_fields.push(TelemetryField::LoopTime),
                _ => {
                    println!("{} Unknown telemetry field: {}", display::glyph_warn(), field);
                }
            }
        }

        // Create telemetry configuration
        let telemetry_config = TelemetryConfig {
            rate_hz: _args.rate,
            enabled_fields: telemetry_fields,
            buffer_size: 1000,
        };

        // Connect to flight controller
        let mut fc = FlightControllerConnection::connect(&_args.connection)
            .await
            .context("Failed to connect to flight controller")?;

        println!("{} Connected successfully", display::glyph_ok());

        // Start telemetry streaming
        let mut telemetry_rx = fc
            .start_telemetry_streaming(telemetry_config)
            .await
            .context("Failed to start telemetry streaming")?;

        println!("Monitoring telemetry at {}Hz...", _args.rate);
        println!("Press Ctrl+C to stop\n");

        // Monitor telemetry
        let start_time = std::time::Instant::now();
        let mut sample_count = 0;

        while let Ok(frame) = telemetry_rx.recv().await {
            sample_count += 1;

            // Check duration limit
            if _args.duration > 0 && start_time.elapsed().as_secs() >= _args.duration {
                break;
            }

            // Display telemetry based on output format
            match _output_format {
                OutputFormat::Pretty => {
                    if sample_count % (_args.rate / 4).max(1) == 0 {
                        // Display at ~4Hz for readability
                        print!("\r{}", format_telemetry_frame(&frame));
                        use std::io::{self, Write};
                        io::stdout().flush().unwrap();
                    }
                }
                OutputFormat::Json => {
                    let json = serde_json::to_string(&format_telemetry_json(&frame))?;
                    println!("{}", json);
                }
                OutputFormat::Csv => {
                    if sample_count == 1 {
                        println!("{}", telemetry_csv_header(&fields));
                    }
                    println!("{}", format_telemetry_csv(&frame));
                }
            }

            // Log to file if specified
            if let Some(ref _log_path) = _args.log_file {
                // TODO: Implement file logging
            }
        }

        println!(
            "\nMonitoring stopped. Captured {} samples",
            sample_count
        );
        Ok(())
    }
}

/// Handle the tune command
async fn tune_command(args: TuneArgs, _output_format: OutputFormat) -> Result<()> {
    print_stage("drone-tuner Tune");

    // --restore short-circuits the entire pull/analyze/apply flow. It only
    // needs an FC connection (auto-discoverable) and the backup file.
    if let Some(restore_path) = &args.restore {
        let connection = match args.connection.as_deref() {
            Some("auto") => discover_fc_port()?,
            Some(c) => c.to_string(),
            None => discover_fc_port()?,
        };
        return restore_pid_from_backup(&connection, restore_path, args.save_eeprom).await;
    }

    // Validate flag combos. `--keep-bbl` only makes sense when we actually
    // have a pulled file to keep; an explicit input together with
    // `--pull-bbl` is ambiguous so reject it.
    if args.keep_bbl.is_some() && !args.pull_bbl {
        return Err(anyhow::anyhow!(
            "--keep-bbl only makes sense together with --pull-bbl"
        ));
    }
    if args.erase_after_pull && !args.pull_bbl {
        return Err(anyhow::anyhow!(
            "--erase-after-pull only makes sense together with --pull-bbl"
        ));
    }
    if args.input.is_some() && args.pull_bbl {
        return Err(anyhow::anyhow!(
            "Pass either a positional BBL file OR --pull-bbl, not both"
        ));
    }

    // Resolve the connection string up-front. If --pull-bbl / --apply-all
    // / --auto-apply-safe is set with no explicit --connection, we'll
    // auto-discover the FC's serial port. The same resolved value is
    // reused for every FC roundtrip in this run (pull, dry-connect, apply).
    let resolved_connection = resolve_connection(&args)?;
    let was_auto_discovered = match (&resolved_connection, args.connection.as_deref()) {
        (Some(_), None) => true,        // omitted, discovered
        (Some(_), Some("auto")) => true, // explicit `auto`, discovered
        _ => false,
    };
    if let (Some(c), true) = (&resolved_connection, was_auto_discovered) {
        println!("auto-discovered FC at {}", style(c).bold());
    }

    // Resolve the .bbl path: download from FC or use the user-provided file.
    let bbl_path = match (&args.input, args.pull_bbl) {
        (Some(p), false) => p.clone(),
        (None, true) => {
            let connection = resolved_connection
                .as_deref()
                .expect("resolve_connection guarantees Some when --pull-bbl is set");
            pull_bbl_from_fc(
                connection,
                args.keep_bbl.as_deref(),
                args.pull_chunk_size,
            )
            .await
            .context("Failed to pull BBL from flight controller")?
        }
        (None, false) => {
            return Err(anyhow::anyhow!(
                "No blackbox file given. Pass a path, or --pull-bbl to download from the FC."
            ))
        }
        (Some(_), true) => unreachable!("guarded above"),
    };

    print_stage("Analyze");

    // Analyze the blackbox file first
    let analyze_args = AnalyzeArgs {
        input: Some(bbl_path.clone()),
        output_dir: None,
        detailed: true,
        show_details: false,
        min_confidence: 0.7,
        max_files: 1,
        session: None,
        list_sessions: false,
        bb_summary: false,
        session_strategy: args.session_strategy.clone(),
        connection: None,
        pull_bbl: false,
        keep_bbl: None,
        erase_after_pull: false,
        pull_chunk_size: None,
    };

    // Get analysis results
    let mut engine = AnalysisEngine::new();
    let analysis_pb = make_spinner(format!("parsing {}", bbl_path.display()));
    let mut analysis = analyze_single_file(&mut engine, &bbl_path, &analyze_args).await?;
    finish_step(
        analysis_pb,
        format!(
            "parsed {} samples ({} ms)",
            analysis.session.telemetry.gyro.len(),
            analysis.analysis_time.as_millis()
        ),
    );

    // Item 4: anchor recommendations against a baseline tune. Each rec's
    // proposed value is clamped to ±15% of the baseline value (not ±15%
    // of current), so cumulative drift across iterations can't escape
    // the envelope. Apply before display so what the user sees is what
    // will actually be written.
    apply_baseline_clamp(&mut analysis.report.pid_recommendations, args.baseline.as_deref());

    // Item 5: cross-tune convergence detector. Look at recent tune-history
    // rows for the same FC; if the analyzer is about to push a gain in the
    // same direction it already pushed it twice before, the underlying
    // problem isn't a PID issue. Suppress the rec and surface an advisory
    // pointing at filters / mechanical follow-up. Runs before display so
    // what the user sees is what will actually be applied.
    apply_convergence_check(&mut analysis.report.pid_recommendations);

    // Display tuning recommendations
    println!("\nTuning Recommendations:");

    if !analysis.report.pid_recommendations.is_empty() {
        println!("\n  PID Adjustments:");
        for rec in &analysis.report.pid_recommendations {
            let priority_icon = display::priority_badge(&rec.priority);
            println!(
                "    {} {:?} {:?}: {:.1} → {:.1}",
                priority_icon, rec.axis, rec.term, rec.current_value, rec.recommended_value
            );
            println!("      Reason: {}", rec.reason);
        }
    }

    if !analysis.report.filter_recommendations.is_empty() {
        println!("\n  Filter Adjustments:");
        for rec in &analysis.report.filter_recommendations {
            let priority_icon = display::priority_badge(&rec.priority);

            let description = match &rec.recommendation_type {
                FilterRecommendationType::AdjustGyroLowpass {
                    stage,
                    current_cutoff,
                    recommended_cutoff,
                    filter_type,
                } => {
                    format!(
                        "Gyro Lowpass {} ({}): {:.0} Hz → {:.0} Hz",
                        stage, filter_type, current_cutoff, recommended_cutoff
                    )
                }
                FilterRecommendationType::ConfigureGyroNotch {
                    notch_number,
                    frequency,
                    q_factor,
                    enabled,
                } => {
                    if *enabled {
                        format!(
                            "Enable Gyro Notch {}: {:.0} Hz (Q: {:.0})",
                            notch_number, frequency, q_factor
                        )
                    } else {
                        format!("Disable Gyro Notch {}", notch_number)
                    }
                }
                FilterRecommendationType::AdjustDynamicNotch {
                    notch_count,
                    q_factor,
                    min_freq,
                    max_freq,
                    enabled,
                } => {
                    if *enabled {
                        format!(
                            "Dynamic Notch: {} notches, {:.0}-{:.0} Hz (Q: {:.0})",
                            notch_count, min_freq, max_freq, q_factor
                        )
                    } else {
                        "Disable Dynamic Notch".to_string()
                    }
                }
                FilterRecommendationType::ConfigureRpmFilter {
                    harmonics,
                    q_factor,
                    min_freq,
                    enabled,
                } => {
                    if *enabled {
                        format!(
                            "Enable RPM Filter: {} harmonics, min {:.0} Hz (Q: {:.0})",
                            harmonics, min_freq, q_factor
                        )
                    } else {
                        "Disable RPM Filter".to_string()
                    }
                }
                FilterRecommendationType::AdjustDtermLowpass {
                    stage,
                    current_cutoff,
                    recommended_cutoff,
                    filter_type,
                    dynamic_settings,
                } => match (current_cutoff, recommended_cutoff, dynamic_settings) {
                    (Some(current), Some(_), Some(dynamic)) => {
                        format!("D-term Lowpass {} ({}): {:.0} Hz → Dynamic {:.0}-{:.0} Hz (expo: {:.0})",
                                stage, filter_type, current, dynamic.min_cutoff, dynamic.max_cutoff, dynamic.expo)
                    }
                    (Some(current), Some(recommended), None) => {
                        format!(
                            "D-term Lowpass {} ({}): {:.0} Hz → {:.0} Hz",
                            stage, filter_type, current, recommended
                        )
                    }
                    (None, Some(recommended), _) => {
                        format!(
                            "Set D-term Lowpass {} ({}): {:.0} Hz",
                            stage, filter_type, recommended
                        )
                    }
                    _ => format!("Adjust D-term Lowpass {} ({})", stage, filter_type),
                },
                FilterRecommendationType::AdjustYawLowpass {
                    current_cutoff,
                    recommended_cutoff,
                } => {
                    format!(
                        "Yaw Lowpass: {:.0} Hz → {:.0} Hz",
                        current_cutoff, recommended_cutoff
                    )
                }
            };

            println!("    {} {}", priority_icon, description);
            println!("      {}", rec.expected_improvement);
        }
    }

    if !analysis.report.advanced_recommendations.is_empty() {
        println!("\n  Advanced Tuning:");
        for rec in &analysis.report.advanced_recommendations {
            let priority_icon = display::priority_badge(&rec.priority);
            let description = format_advanced_rec(&rec.parameter);
            println!("    {} {}", priority_icon, description);
            println!("      {}", rec.reason);
        }
    }

    // Always surface the analyzer's measured state, regardless of whether
    // recs were generated. This means a clean-tune ("no changes") output
    // reads as *measured silence* rather than ambiguous emptiness — the
    // user sees the step-response count and the gyro spectrum we were
    // looking at when we decided not to recommend anything.
    print_tune_findings(&analysis);

    // Decide whether/how to apply.
    match (resolved_connection.as_deref(), args.dry_run) {
        (None, true) => {
            println!("\n{} Dry run mode - no changes applied", display::glyph_info());
        }
        (None, false) => {
            println!(
                "\n{} Specify --connection (or --apply-all/--auto-apply-safe to auto-discover) \
                 to apply changes to flight controller",
                display::glyph_info()
            );
        }
        (Some(connection), true) => {
            // Dry-run + connection: actually open the FC, read current state,
            // show what WOULD change, but don't write. Useful for verifying
            // hardware connectivity before committing to a tune.
            dry_connect_and_diff(connection, &args, &analysis.report).await?;
        }
        (Some(connection), false) => {
            apply_pid_recommendations_via_fc(connection, &bbl_path, &args, &analysis.report)
                .await?;
        }
    }

    // Erase the FC's dataflash NOW, after every preceding stage succeeded.
    // Skipped on dry-run: the user is just simulating, leaving the chip
    // alone is the sane default. Skipped without a connection: we have no
    // FC to talk to. Failure here is non-fatal — the BBL is on disk, the
    // tune itself completed.
    if args.erase_after_pull && args.pull_bbl && !args.dry_run {
        if let Some(connection) = resolved_connection.as_deref() {
            print_stage("Cleanup");
            erase_dataflash_post_tune(connection).await?;
        } else {
            println!(
                "\n  {} skipping --erase-after-pull (no FC connection at end of run)",
                display::glyph_info()
            );
        }
    }

    Ok(())
}

/// Print the analyzer's measured findings — step-response counts, top-3
/// gyro peaks per axis, and the filter cutoffs that were active in the
/// analyzed log. Always called after the recommendations block so the
/// "no changes needed" case still shows the evidence behind that verdict.
fn print_tune_findings(analysis: &AnalysisResult) {
    use drone_tuner_core::domain::Axis;

    let total_recs =
        analysis.report.pid_recommendations.len() + analysis.report.filter_recommendations.len();

    // Empty-recs banner: turn an empty list into a clear positive verdict.
    if total_recs == 0 {
        println!(
            "\n  {} Tune quality: {:.1}/100 — no changes needed",
            display::glyph_ok(),
            analysis.report.tune_quality_score
        );
    }

    // Step-response counts per axis. analyzer's PID stage runs over the
    // RC trace; non-zero counts here are confirmation it had something
    // to chew on.
    let steps = &analysis.report.step_responses;
    let (mut roll_n, mut pitch_n, mut yaw_n) = (0usize, 0usize, 0usize);
    for s in steps {
        match s.axis {
            Axis::Roll => roll_n += 1,
            Axis::Pitch => pitch_n += 1,
            Axis::Yaw => yaw_n += 1,
        }
    }
    println!(
        "\n  Step responses analyzed: {} ({} roll, {} pitch, {} yaw)",
        steps.len(),
        roll_n,
        pitch_n,
        yaw_n
    );

    // Top-3 gyro peaks per axis. Peaks tag themselves with the axes they
    // appear on, so we filter, sort by amplitude desc, take 3.
    let peaks = &analysis.report.frequency_analysis.peaks;
    println!(
        "  Gyro spectrum (noise floor {:.4}):",
        analysis.report.frequency_analysis.noise_floor
    );

    let mut total_shown = 0usize;
    for (label, axis) in [("Roll ", Axis::Roll), ("Pitch", Axis::Pitch), ("Yaw  ", Axis::Yaw)] {
        let mut axis_peaks: Vec<_> = peaks.iter().filter(|p| p.axes.contains(&axis)).collect();
        axis_peaks.sort_by(|a, b| {
            b.amplitude
                .partial_cmp(&a.amplitude)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        let top = axis_peaks.iter().take(3);
        let count = axis_peaks.len().min(3);
        if count == 0 {
            println!("    {}: no peaks above threshold", label);
        } else {
            let parts: Vec<String> = top
                .map(|p| {
                    format!(
                        "{:.0} Hz (amp {:.3}, Q={:.1})",
                        p.frequency, p.amplitude, p.q_factor
                    )
                })
                .collect();
            println!("    {}: {}", label, parts.join("  |  "));
            total_shown += count;
        }
    }
    if total_shown == 0 {
        println!(
            "    {} measured silence — no oscillation peaks above noise floor",
            display::glyph_ok()
        );
    }

    // Filter cutoffs that were active during the analyzed log. These come
    // from the BBL header, so they reflect what was running on the FC at
    // log time (not necessarily what's on the FC right now if it was
    // re-flashed since).
    let filters = &analysis.session.metadata.hardware.filter_config;
    println!("  {} Active filters at log time:", "");
    if filters.gyro_filters.is_empty() {
        println!("    Gyro LPF:    not configured");
    } else {
        for f in &filters.gyro_filters {
            println!(
                "    Gyro LPF:    {:.0} Hz ({:?}, order {})",
                f.cutoff, f.filter_type, f.order
            );
        }
    }
    if filters.dterm_filters.is_empty() {
        println!("    D-term LPF:  not configured");
    } else {
        for f in &filters.dterm_filters {
            println!(
                "    D-term LPF:  {:.0} Hz ({:?}, order {})",
                f.cutoff, f.filter_type, f.order
            );
        }
    }
    if let Some(dn) = &filters.dynamic_notch {
        if dn.enabled {
            println!(
                "    Dyn Notch:   {:.0}-{:.0} Hz (Q={:.0})",
                dn.min_freq, dn.max_freq, dn.q_factor
            );
        }
    }
}

// Stage / spinner / step helpers live in `display` now; re-export so the
// many call sites in this file keep their bare names.
use display::{finish_step, make_spinner, print_stage};

/// Pull the FC's onboard dataflash to a `.bbl` file with a live progress
/// bar. Returns the path to the saved file.
///
/// If `keep_path` is `Some`, the file is written there verbatim. Otherwise
/// it goes to `std::env::temp_dir()/drone-tuner-pull-<ts>.bbl`. We print
/// the destination so the user can pick the file up later.
///
/// The caller is responsible for triggering an erase if `--erase-after-pull`
/// was requested; we deliberately don't do it here so the chip stays intact
/// until the *whole* tune flow has succeeded (parse + analysis + apply).
/// Otherwise a downstream failure would lose data the user might still need.
async fn pull_bbl_from_fc(
    connection: &str,
    keep_path: Option<&Path>,
    chunk_size: Option<u16>,
) -> Result<PathBuf> {
    print_stage("Pull");

    let connect_pb = make_spinner(format!("connecting to {connection}"));
    let mut fc = open_fc_connection(connection).await?;
    let info_line = match fc.fc_info() {
        Some(info) => {
            let name_suffix = if info.craft_name.is_empty() {
                String::new()
            } else {
                format!(", craft \"{}\"", info.craft_name)
            };
            format!(
                "connected: {} {} (api {}, target {}{})",
                info.firmware_id,
                info.firmware_version,
                info.api_version,
                info.target_name,
                name_suffix
            )
        }
        None => "connected".to_string(),
    };
    finish_step(connect_pb, info_line);

    let summary_pb = make_spinner("reading dataflash summary");
    let summary = fc.read_dataflash_summary().await?;
    finish_step(
        summary_pb,
        format!(
            "dataflash: {} used / {} total ({} sectors)",
            format_bytes(summary.used_size as u64),
            format_bytes(summary.total_size as u64),
            summary.sectors,
        ),
    );

    if !summary.supported {
        return Err(anyhow::anyhow!(
            "FC reports no onboard dataflash chip — looks like an SD-card \
             or no-blackbox board. Pull is not supported here."
        ));
    }
    if summary.used_size == 0 {
        return Err(anyhow::anyhow!(
            "FC dataflash is empty. Record a flight first, then re-run."
        ));
    }

    // Decide where the file lands. Default name embeds the FC's craft
    // name so multi-quad logs don't collide and `ls` makes sense.
    //
    // `--keep-bbl <PATH>` accepts either a file path (used verbatim) or
    // an existing directory (in which case we generate a per-craft
    // filename inside it). Treating "~/ " or "~/logs/" as "save into
    // that dir" matches what users expect, and avoids the
    // `Is a directory (os error 21)` write failure.
    let ts = chrono::Utc::now().format("%Y%m%d-%H%M%S");
    let slug = fc.fc_info().and_then(craft_name_slug);
    let auto_filename = match &slug {
        Some(name) => format!("drone-tuner-pull-{name}-{ts}.bbl"),
        None => format!("drone-tuner-pull-{ts}.bbl"),
    };
    let path: PathBuf = match keep_path {
        Some(p) if p.is_dir() => p.join(&auto_filename),
        Some(p) => p.to_path_buf(),
        // Default to the central pulls dir so BBLs survive reboots and
        // accumulate in one searchable place. Falls through to /tmp only
        // if the data dir can't be resolved (no $HOME, etc.).
        None => paths::pulls_dir()
            .map(|d| d.join(&auto_filename))
            .unwrap_or_else(|_| std::env::temp_dir().join(&auto_filename)),
    };

    let pull_pb = ProgressBar::new(summary.used_size as u64);
    pull_pb.set_style(
        ProgressStyle::with_template(
            "  {spinner:.cyan} [{bar:40.cyan/blue}] {bytes}/{total_bytes} {bytes_per_sec} ({eta}) {msg}",
        )
        .unwrap()
        .progress_chars("=>-"),
    );
    pull_pb.set_message("downloading dataflash");
    pull_pb.enable_steady_tick(std::time::Duration::from_millis(120));
    let pull_pb_for_cb = pull_pb.clone();
    // 1024 by default — proven to work across our test fleet. User can
    // experiment with --pull-chunk-size 2048/4096 to halve or quarter
    // roundtrip count if their FC's USB CDC TX buffer can keep up.
    let chunk_size = chunk_size.unwrap_or(1024);
    let blob = fc
        .pull_dataflash_with(
            move |done, _total| pull_pb_for_cb.set_position(done),
            chunk_size,
        )
        .await
        .context("Dataflash pull failed")?;
    finish_step(pull_pb, format!("downloaded {}", format_bytes(blob.len() as u64)));

    // Persist.
    std::fs::write(&path, &blob)
        .with_context(|| format!("Failed to write pulled BBL to {}", path.display()))?;
    println!("  {} saved → {}", "", path.display());

    Ok(path)
}

/// Restore PID values from a previously saved `tune-backup-*.json` file.
/// Reads the current PIDs (for diff display), then writes the backup's
/// PID triplets via [`apply_pid_with_rollback`] (with the FC's own
/// pre-write snapshot as the rollback target).
///
/// We only patch the first 9 bytes (Roll/Pitch/Yaw P/I/D) to avoid blowing
/// away axes 4/5 if the FC's payload is longer than the backup's — the
/// backup schema only carries the three flight axes.
///
/// [`apply_pid_with_rollback`]: drone_tuner_core::realtime::FlightControllerConnection::apply_pid_with_rollback
async fn restore_pid_from_backup(
    connection: &str,
    backup_path: &Path,
    save_eeprom: bool,
) -> Result<()> {
    print_stage("Restore");

    // Load + validate the backup file.
    let raw = std::fs::read(backup_path)
        .with_context(|| format!("Failed to read backup file {}", backup_path.display()))?;
    let json: serde_json::Value = serde_json::from_slice(&raw)
        .with_context(|| format!("Backup file {} is not valid JSON", backup_path.display()))?;

    let schema = json.get("schema").and_then(|v| v.as_str()).unwrap_or("");
    if schema != BACKUP_SCHEMA_V1 && schema != BACKUP_SCHEMA_V2 {
        return Err(anyhow::anyhow!(
            "Unrecognised backup schema {:?} — expected {:?} or {:?}",
            schema,
            BACKUP_SCHEMA_V1,
            BACKUP_SCHEMA_V2
        ));
    }
    let is_v2 = schema == BACKUP_SCHEMA_V2;

    fn read_triplet(v: &serde_json::Value, key: &str) -> Result<(u8, u8, u8)> {
        let arr = v
            .get(key)
            .and_then(|x| x.as_array())
            .with_context(|| format!("backup is missing {key} array"))?;
        if arr.len() != 3 {
            return Err(anyhow::anyhow!(
                "backup {key} array has {} entries, expected 3",
                arr.len()
            ));
        }
        let mut out = [0u8; 3];
        for (i, item) in arr.iter().enumerate() {
            let n = item
                .as_u64()
                .with_context(|| format!("backup {key}[{i}] is not an integer"))?;
            if n > 255 {
                return Err(anyhow::anyhow!(
                    "backup {key}[{i}]={n} exceeds u8 range — refusing to write"
                ));
            }
            out[i] = n as u8;
        }
        Ok((out[0], out[1], out[2]))
    }

    let roll = read_triplet(&json, "roll")?;
    let pitch = read_triplet(&json, "pitch")?;
    let yaw = read_triplet(&json, "yaw")?;
    let captured_at = json
        .get("captured_at")
        .and_then(|v| v.as_str())
        .unwrap_or("(unknown time)");

    println!(
        "  loaded backup ({}): roll={:?} pitch={:?} yaw={:?}",
        captured_at, roll, pitch, yaw
    );

    // Connect, read current state for diff.
    let connect_pb = make_spinner(format!("connecting to {connection}"));
    let mut fc = open_fc_connection(connection).await?;
    let info_line = match fc.fc_info() {
        Some(info) => {
            let name_suffix = if info.craft_name.is_empty() {
                String::new()
            } else {
                format!(", craft \"{}\"", info.craft_name)
            };
            format!(
                "connected: {} {} (api {}, target {}{})",
                info.firmware_id,
                info.firmware_version,
                info.api_version,
                info.target_name,
                name_suffix
            )
        }
        None => "connected".to_string(),
    };
    finish_step(connect_pb, info_line);

    let read_pb = make_spinner("reading current PID values");
    let current = fc
        .read_pid()
        .await
        .context("Failed to read current PID values from FC")?;
    finish_step(
        read_pb,
        format!(
            "current: roll={:?} pitch={:?} yaw={:?}",
            current.roll(),
            current.pitch(),
            current.yaw()
        ),
    );

    // Compose the new snapshot: start from the FC's current payload
    // (preserving length and any extra-axis bytes), then patch the three
    // flight axes with the backup values.
    let mut new_snapshot = current.clone();
    new_snapshot.set_roll(roll.0, roll.1, roll.2);
    new_snapshot.set_pitch(pitch.0, pitch.1, pitch.2);
    new_snapshot.set_yaw(yaw.0, yaw.1, yaw.2);

    if current.roll() == roll && current.pitch() == pitch && current.yaw() == yaw {
        println!(
            "  {} FC PIDs already match backup — nothing to write.",
            display::glyph_info()
        );
        return Ok(());
    }

    println!("  PID changes to apply (restore):");
    if current.roll() != roll {
        println!("    Roll  P/I/D: {:?} → {:?}", current.roll(), roll);
    }
    if current.pitch() != pitch {
        println!("    Pitch P/I/D: {:?} → {:?}", current.pitch(), pitch);
    }
    if current.yaw() != yaw {
        println!("    Yaw   P/I/D: {:?} → {:?}", current.yaw(), yaw);
    }

    let write_pb = make_spinner("writing PIDs (with rollback safety net)");
    fc.apply_pid_with_rollback(&new_snapshot)
        .await
        .context("PID restore failed (any partial write was rolled back)")?;
    finish_step(write_pb, "PIDs restored");

    // v2 backups carry round-trip snapshots of the Phase-1 surfaces
    // (PidAdvanced / RcTuning / AdvancedConfig). Restoring them after
    // the PID write means a `tune --restore` is whole-config: any
    // pilot-preference drift introduced by a Configurator session
    // between backup and restore is undone. v1 backups skip this block
    // (the extras simply aren't in the file).
    if is_v2 {
        use drone_tuner_core::realtime::{
            AdvancedConfigSnapshot, PidAdvancedSnapshot, RcTuningSnapshot,
        };
        let mut extras = Phase1Extras::default();
        if let Some(bytes) = read_optional_payload(&json, "pid_advanced_payload")? {
            extras.pid_advanced = Some(
                PidAdvancedSnapshot::from_payload(bytes)
                    .context("backup pid_advanced_payload is malformed")?,
            );
        }
        if let Some(bytes) = read_optional_payload(&json, "rc_tuning_payload")? {
            extras.rc_tuning = Some(
                RcTuningSnapshot::from_payload(bytes)
                    .context("backup rc_tuning_payload is malformed")?,
            );
        }
        if let Some(bytes) = read_optional_payload(&json, "advanced_config_payload")? {
            extras.advanced_config = Some(
                AdvancedConfigSnapshot::from_payload(bytes)
                    .context("backup advanced_config_payload is malformed")?,
            );
        }
        let total = extras.pid_advanced.is_some() as usize
            + extras.rc_tuning.is_some() as usize
            + extras.advanced_config.is_some() as usize;
        if total > 0 {
            let extras_pb = make_spinner(format!("restoring {total} auxiliary MSP surface(s)"));
            let written = write_phase1_extras(&mut fc, &extras).await;
            finish_step(
                extras_pb,
                format!("auxiliary surfaces restored: {written}/{total}"),
            );
        }
    }

    if save_eeprom {
        let save_pb = make_spinner("persisting to EEPROM");
        match fc.save_to_eeprom().await {
            Ok(()) => finish_step(save_pb, "EEPROM written — survives power cycle"),
            Err(e) => {
                save_pb.finish_and_clear();
                println!(
                    "  {} EEPROM write failed: {} (RAM changes still in effect, will revert on power cycle)",
                    display::glyph_warn(),
                    e
                );
            }
        }
    } else {
        println!(
            "  {} Restored values are RAM-only and will revert on power cycle. \
             Pass --save-eeprom to persist.",
            display::glyph_info()
        );
    }

    println!("\n  {} Restore complete.", display::glyph_ok());
    Ok(())
}

/// Load a `tune-backup-*.json` file and project it into a
/// [`PidConfiguration`] for the baseline-anchored clamp (Item 4). The
/// backup schema only carries the three flight axes' P/I/D triplets; all
/// other fields of `PidConfiguration` are filled with defaults — the
/// clamp helper only consults `roll/pitch/yaw {p,i,d}` so they're inert.
fn load_baseline_pid_config(path: &Path) -> Result<drone_tuner_core::domain::PidConfiguration> {
    use drone_tuner_core::domain::{PidConfiguration, PidValues};

    let raw = std::fs::read(path)
        .with_context(|| format!("Failed to read baseline file {}", path.display()))?;
    let json: serde_json::Value = serde_json::from_slice(&raw)
        .with_context(|| format!("Baseline file {} is not valid JSON", path.display()))?;

    let schema = json.get("schema").and_then(|v| v.as_str()).unwrap_or("");
    if schema != BACKUP_SCHEMA_V1 && schema != BACKUP_SCHEMA_V2 {
        return Err(anyhow::anyhow!(
            "Unrecognised baseline schema {:?} in {} — expected {:?} or {:?}",
            schema,
            path.display(),
            BACKUP_SCHEMA_V1,
            BACKUP_SCHEMA_V2
        ));
    }

    fn read_triplet(v: &serde_json::Value, key: &str) -> Result<[f32; 3]> {
        let arr = v
            .get(key)
            .and_then(|x| x.as_array())
            .with_context(|| format!("baseline missing {key} array"))?;
        if arr.len() != 3 {
            return Err(anyhow::anyhow!(
                "baseline {key} array has {} entries, expected 3",
                arr.len()
            ));
        }
        Ok([
            arr[0].as_f64().unwrap_or(0.0) as f32,
            arr[1].as_f64().unwrap_or(0.0) as f32,
            arr[2].as_f64().unwrap_or(0.0) as f32,
        ])
    }

    let roll = read_triplet(&json, "roll")?;
    let pitch = read_triplet(&json, "pitch")?;
    let yaw = read_triplet(&json, "yaw")?;

    let mut cfg = PidConfiguration::default();
    cfg.roll = PidValues { p: roll[0], i: roll[1], d: roll[2], f: None };
    cfg.pitch = PidValues { p: pitch[0], i: pitch[1], d: pitch[2], f: None };
    cfg.yaw = PidValues { p: yaw[0], i: yaw[1], d: yaw[2], f: None };
    Ok(cfg)
}

/// Apply Item 4's baseline-anchored clamp in place. `flag` is the raw
/// `--baseline` value:
///
/// - `Some("none")` — explicit opt-out. Recs are not clamped; no output.
/// - `Some(path)` — load the file as a baseline and clamp.
/// - `None` — soft note nudging the user to pass a baseline next time;
///   recs pass through unclamped (preserves existing behaviour).
///
/// Auto-detect from craft name in cwd is intentionally not done here:
/// the bbl's `FlightSession` doesn't carry the craft name (it's on the
/// FC connection only), and threading FC info into this function adds
/// more coupling than the convenience earns. Pass the path explicitly.
fn apply_baseline_clamp(
    recs: &mut Vec<drone_tuner_core::domain::PidRecommendation>,
    flag: Option<&str>,
) {
    match flag {
        Some("none") => {
            // Explicit opt-out: stay silent.
        }
        Some(path_str) => {
            let path = PathBuf::from(path_str);
            match load_baseline_pid_config(&path) {
                Ok(baseline_cfg) => {
                    let pre_count = recs.len();
                    let taken = std::mem::take(recs);
                    let clamped = drone_tuner_core::analysis::clamp_recs_to_baseline(
                        taken,
                        &baseline_cfg,
                    );
                    let dropped = pre_count.saturating_sub(clamped.len());
                    *recs = clamped;
                    println!(
                        "  {} baseline anchor: {} (±{:.0}% bound{})",
                        display::glyph_ok(),
                        path.display(),
                        drone_tuner_core::analysis::BASELINE_BOUND_PCT * 100.0,
                        if dropped > 0 {
                            format!(", {dropped} rec(s) dropped as out-of-envelope")
                        } else {
                            String::new()
                        }
                    );
                }
                Err(e) => {
                    println!(
                        "  {} baseline file unreadable ({e}); proceeding without anchor",
                        display::glyph_warn()
                    );
                }
            }
        }
        None => {
            println!(
                "  {} no --baseline anchor passed; iterative drift can compound. \
                 Pass --baseline <PATH> for the safest tune flow.",
                display::glyph_info()
            );
        }
    }
}

/// Item 5: cross-tune convergence detector entry point. Reads the persisted
/// `tune-history.jsonl`, filters to the FC most recently tuned (so multi-FC
/// users don't get cross-contamination), translates rows into core's
/// [`PidChangeRecord`] schema, and runs [`apply_convergence_suppression`].
///
/// Recs that survive are written back into `recs` in place. Suppressed recs
/// are printed to stdout as advisories with the analyzer's reasoning.
///
/// Failure modes are non-fatal: if the history file is missing, malformed,
/// or unreadable, we log and proceed with no suppression. The detector is
/// a *safety net*, not a hard gate — losing it must never block a tune.
fn apply_convergence_check(recs: &mut Vec<drone_tuner_core::domain::PidRecommendation>) {
    use drone_tuner_core::analysis::{
        apply_convergence_suppression, PidChangeRecord, DEFAULT_MIN_REPEATED,
    };
    use drone_tuner_core::domain::{Axis, PidTerm};

    if recs.is_empty() {
        return;
    }

    let history = match history::read_all() {
        Ok(h) => h,
        Err(e) => {
            tracing::debug!("Convergence check skipped: {e:#}");
            return;
        }
    };

    if history.is_empty() {
        return;
    }

    // Use the *most recent* row's FC identity as the filter. This is the
    // robust single-FC default (always self-filters correctly) and the
    // sane multi-FC heuristic (only triggers when the user is iterating
    // on the same FC the log was last appended to).
    let last = history.last().unwrap();
    let (board_id, target_name) = (last.fc.board_id.clone(), last.fc.target_name.clone());

    let mut changes: Vec<PidChangeRecord> = Vec::with_capacity(history.len() * 9);
    for entry in &history {
        if entry.fc.board_id != board_id || entry.fc.target_name != target_name {
            continue;
        }
        for (axis, before, after) in [
            (Axis::Roll, entry.pids_before.roll, entry.pids_after.roll),
            (Axis::Pitch, entry.pids_before.pitch, entry.pids_after.pitch),
            (Axis::Yaw, entry.pids_before.yaw, entry.pids_after.yaw),
        ] {
            for (term, idx) in [(PidTerm::P, 0usize), (PidTerm::I, 1), (PidTerm::D, 2)] {
                let delta = after[idx] as i32 - before[idx] as i32;
                changes.push(PidChangeRecord {
                    axis: axis.clone(),
                    term,
                    delta,
                });
            }
        }
    }

    let taken = std::mem::take(recs);
    let outcome = apply_convergence_suppression(taken, &changes, DEFAULT_MIN_REPEATED);
    *recs = outcome.kept;

    if outcome.suppressed.is_empty() {
        return;
    }

    println!(
        "  {} convergence detector suppressed {} rec(s) for FC {}/{} \
         (same gain pushed in same direction across {} iterations):",
        display::glyph_warn(),
        outcome.suppressed.len(),
        board_id,
        target_name,
        DEFAULT_MIN_REPEATED + 1,
    );
    for s in &outcome.suppressed {
        println!(
            "    {} {:?} {:?}: would have applied {:.1} → {:.1}",
            style("[!]").yellow().bold(),
            s.original.axis,
            s.original.term,
            s.original.current_value,
            s.original.recommended_value,
        );
        println!("      Advisory: {}", s.advisory);
    }
}

/// Wipe the FC's onboard dataflash. Called from `tune_command` only after
/// the entire flow (pull → analyze → apply) has succeeded, so a failure
/// midway never destroys log data the user might want to re-pull.
/// Failure to erase is non-fatal — we already have the BBL on disk.
async fn erase_dataflash_post_tune(connection: &str) -> Result<()> {
    let connect_pb = make_spinner(format!("reconnecting to {connection} for erase"));
    let mut fc = open_fc_connection(connection).await?;
    finish_step(connect_pb, "reconnected".to_string());

    let erase_pb = make_spinner("erasing dataflash (acks fast, wipe runs async on FC)");
    match fc.erase_dataflash().await {
        Ok(()) => finish_step(
            erase_pb,
            "dataflash erase queued (chip will clear in the background)",
        ),
        Err(e) => {
            erase_pb.finish_and_clear();
            println!(
                "  {} dataflash erase failed: {} \
                 (your BBL is already saved; tune flow itself completed)",
                display::glyph_warn(),
                e
            );
        }
    }
    Ok(())
}

/// Sanitise an arbitrary string (typically the FC's `craft_name`) into
/// something safe to splat into a filename: ASCII alphanumerics, `_`,
/// and `-` are kept; everything else (spaces, punctuation, non-ASCII)
/// becomes `_`. Repeated underscores collapse and leading/trailing
/// underscores are trimmed so we don't get `__jeno__`.
///
/// Returns `None` for inputs that produce an empty result (so callers
/// can fall back to a timestamp or fixed prefix without printing a
/// dangling separator).
fn sanitize_filename_part(s: &str) -> Option<String> {
    let mut out = String::with_capacity(s.len());
    let mut last_was_underscore = false;
    for c in s.chars() {
        if c.is_ascii_alphanumeric() || c == '-' {
            out.push(c);
            last_was_underscore = false;
        } else if c == '_' || c.is_whitespace() {
            if !last_was_underscore && !out.is_empty() {
                out.push('_');
                last_was_underscore = true;
            }
        }
        // Anything else (punctuation, non-ASCII): drop silently.
    }
    let trimmed = out.trim_matches('_');
    if trimmed.is_empty() {
        None
    } else {
        Some(trimmed.to_string())
    }
}

/// Build a filename suffix from the FC's craft name. Returns
/// `Some("jeno")` for `craft_name = "Jeno"`, `Some("tbs_source_one")`
/// for `craft_name = "TBS Source One"`, and `None` when the name is
/// empty or sanitises away to nothing.
fn craft_name_slug(info: &drone_tuner_core::realtime::FlightControllerInfo) -> Option<String> {
    sanitize_filename_part(&info.craft_name).map(|s| s.to_lowercase())
}

/// Phase-1 round-trip safety net: opaque snapshots of the MSP surfaces
/// drone-tuner reads but never modifies (PID-Advanced, RC-Tuning,
/// Advanced-Config). Captured into the backup file alongside the PID
/// payload so `tune --restore` is whole-config — restoring a backup
/// returns the FC to its pre-tune state across all four surfaces, not
/// just the three flight axes.
///
/// Each field is optional because individual MSP commands may be
/// unsupported on stripped vendor builds. We log a warning and proceed
/// rather than aborting the tune — preserving what we can is strictly
/// better than refusing to back anything up.
#[derive(Debug, Clone, Default)]
struct Phase1Extras {
    pid_advanced: Option<drone_tuner_core::realtime::PidAdvancedSnapshot>,
    rc_tuning: Option<drone_tuner_core::realtime::RcTuningSnapshot>,
    advanced_config: Option<drone_tuner_core::realtime::AdvancedConfigSnapshot>,
}

/// Read all three Phase-1 surfaces from the FC. Failures are logged
/// per-surface; the function never errors so a tune isn't blocked by
/// an unsupported MSP command on an exotic firmware build.
async fn read_phase1_extras(
    fc: &mut drone_tuner_core::realtime::FlightControllerConnection,
) -> Phase1Extras {
    let pid_advanced = match fc.read_pid_advanced().await {
        Ok(s) => Some(s),
        Err(e) => {
            warn!("MSP_PID_ADVANCED read failed (skipping in backup): {e:#}");
            None
        }
    };
    let rc_tuning = match fc.read_rc_tuning().await {
        Ok(s) => Some(s),
        Err(e) => {
            warn!("MSP_RC_TUNING read failed (skipping in backup): {e:#}");
            None
        }
    };
    let advanced_config = match fc.read_advanced_config().await {
        Ok(s) => Some(s),
        Err(e) => {
            warn!("MSP_ADVANCED_CONFIG read failed (skipping in backup): {e:#}");
            None
        }
    };
    Phase1Extras {
        pid_advanced,
        rc_tuning,
        advanced_config,
    }
}

/// Write the Phase-1 extras back to the FC. Used by `tune --restore`.
/// Failures on individual surfaces are surfaced as warnings rather
/// than errors — the PID restore that just succeeded is the headline
/// recovery action; a follow-up failure on (say) RcTuning shouldn't
/// undo it. The user can re-run `tune --restore` to retry.
async fn write_phase1_extras(
    fc: &mut drone_tuner_core::realtime::FlightControllerConnection,
    extras: &Phase1Extras,
) -> usize {
    let mut written = 0;
    if let Some(s) = &extras.pid_advanced {
        match fc.write_pid_advanced(s).await {
            Ok(()) => {
                written += 1;
            }
            Err(e) => {
                println!(
                    "  {} MSP_PID_ADVANCED restore failed: {} (other surfaces still being restored)",
                    display::glyph_warn(),
                    e
                );
            }
        }
    }
    if let Some(s) = &extras.rc_tuning {
        match fc.write_rc_tuning(s).await {
            Ok(()) => {
                written += 1;
            }
            Err(e) => {
                println!(
                    "  {} MSP_RC_TUNING restore failed: {} (other surfaces still being restored)",
                    display::glyph_warn(),
                    e
                );
            }
        }
    }
    if let Some(s) = &extras.advanced_config {
        match fc.write_advanced_config(s).await {
            Ok(()) => {
                written += 1;
            }
            Err(e) => {
                println!(
                    "  {} MSP_ADVANCED_CONFIG restore failed: {} (other surfaces still being restored)",
                    display::glyph_warn(),
                    e
                );
            }
        }
    }
    written
}

/// Backup-file schema constant. Bumped to v2 in the Phase-1 round-trip
/// safety net work — v2 adds optional `pid_advanced_payload`,
/// `rc_tuning_payload`, `advanced_config_payload` fields. Restore reads
/// both v1 and v2 (v1 backups simply lack the extras); writing always
/// emits v2 going forward.
const BACKUP_SCHEMA_V1: &str = "drone-tuner-pid-backup-v1";
const BACKUP_SCHEMA_V2: &str = "drone-tuner-pid-backup-v2";

/// Parse an optional byte-array payload field from a backup JSON. Used
/// by the v2 schema reader for the three Phase-1 extra surfaces. Returns
/// `Ok(None)` when the field is missing or null; errors only on a
/// malformed array (non-integer or out-of-u8 entry) so a backup written
/// on a future schema-bump still reads cleanly when the field happens
/// to be absent.
fn read_optional_payload(json: &serde_json::Value, key: &str) -> Result<Option<Vec<u8>>> {
    let Some(v) = json.get(key) else {
        return Ok(None);
    };
    if v.is_null() {
        return Ok(None);
    }
    let arr = v
        .as_array()
        .with_context(|| format!("backup field {key} is not an array"))?;
    let mut out = Vec::with_capacity(arr.len());
    for (i, item) in arr.iter().enumerate() {
        let n = item
            .as_u64()
            .with_context(|| format!("backup {key}[{i}] is not an integer"))?;
        if n > 255 {
            return Err(anyhow::anyhow!(
                "backup {key}[{i}]={n} exceeds u8 range — refusing to write"
            ));
        }
        out.push(n as u8);
    }
    Ok(Some(out))
}

/// Pretty-print byte counts as KB/MB/GB.
fn format_bytes(bytes: u64) -> String {
    const KB: u64 = 1024;
    const MB: u64 = KB * 1024;
    const GB: u64 = MB * 1024;
    if bytes >= GB {
        format!("{:.2} GB", bytes as f64 / GB as f64)
    } else if bytes >= MB {
        format!("{:.2} MB", bytes as f64 / MB as f64)
    } else if bytes >= KB {
        format!("{:.1} KB", bytes as f64 / KB as f64)
    } else {
        format!("{bytes} B")
    }
}

/// Pick a single serial port from a list of candidates likely to be a
/// Betaflight flight controller. Pulled out for unit-testability —
/// [`discover_fc_port`] wraps it with the live `available_ports` call.
///
/// Strategy:
/// 1. Drop anything that isn't a USB serial port.
/// 2. Prefer ports whose USB descriptor matches STMicroelectronics
///    (VID `0x0483`) — that's STM32 VCP, the default Betaflight USB
///    enumeration on the vast majority of boards.
/// 3. If no STM32 VCP found, fall back to all USB ports — covers the
///    smaller set of boards using AT32 / CH340 / CP210x.
/// 4. Refuse to guess if more than one candidate remains; print the
///    list and ask the user to pass `--connection <PATH>`.
fn pick_fc_port_from(
    ports: Vec<serialport::SerialPortInfo>,
) -> Result<serialport::SerialPortInfo> {
    use serialport::SerialPortType;

    let usb_ports: Vec<_> = ports
        .into_iter()
        .filter(|p| matches!(p.port_type, SerialPortType::UsbPort(_)))
        .collect();

    if usb_ports.is_empty() {
        return Err(anyhow::anyhow!(
            "No USB serial devices found. Plug your FC in (and make sure \
             it's not in DFU/bootloader mode), then re-run."
        ));
    }

    let stm32: Vec<_> = usb_ports
        .iter()
        .filter(|p| matches!(&p.port_type, SerialPortType::UsbPort(usb) if usb.vid == 0x0483))
        .cloned()
        .collect();
    let candidates = if !stm32.is_empty() {
        stm32
    } else {
        usb_ports
    };

    match candidates.len() {
        1 => Ok(candidates.into_iter().next().unwrap()),
        n => {
            let names: Vec<String> = candidates
                .iter()
                .map(|p| match &p.port_type {
                    SerialPortType::UsbPort(usb) => format!(
                        "{} (vid:{:04x} pid:{:04x}{}{})",
                        p.port_name,
                        usb.vid,
                        usb.pid,
                        usb.manufacturer
                            .as_deref()
                            .map(|m| format!(" {m}"))
                            .unwrap_or_default(),
                        usb.product
                            .as_deref()
                            .map(|p| format!(" / {p}"))
                            .unwrap_or_default(),
                    ),
                    _ => p.port_name.clone(),
                })
                .collect();
            Err(anyhow::anyhow!(
                "Found {n} USB serial device(s); auto-discover only picks when there's exactly \
                 one candidate. Pass --connection <PATH> to choose:\n  - {}",
                names.join("\n  - ")
            ))
        }
    }
}

/// Auto-discover the FC's serial port. Returns the device path
/// (e.g. `/dev/ttyACM0`) of the only plausible candidate.
fn discover_fc_port() -> Result<String> {
    let ports = serialport::available_ports()
        .context("Failed to enumerate serial ports — is the user in the dialout group?")?;
    let pick = pick_fc_port_from(ports)?;
    Ok(pick.port_name)
}

/// Resolve a `--connection` argument into a usable connection string.
///
/// Auto-discovery fires only when the user explicitly asks for it:
/// - `Some("auto")` → run [`discover_fc_port`] and hard-fail on error.
/// - `None` + `--pull-bbl` → auto-discover, hard-fail on error
///   (`--pull-bbl` has no fallback path — without a port there's nothing
///   to pull).
/// - `Some(other)` → pass through verbatim (covers `/dev/...`,
///   `serial://...`, `simulator://...`).
/// - `None` everywhere else → return `None`. `--apply-all` /
///   `--auto-apply-safe` without an explicit `--connection` keeps the
///   pre-auto-discover behaviour ("specify --connection ..." prompt)
///   — being conservative here avoids opening a busy port that some
///   other process owns and surprising users whose intent was just to
///   look at recommendations.
fn resolve_connection(args: &TuneArgs) -> Result<Option<String>> {
    resolve_connection_raw(args.connection.as_deref(), args.pull_bbl)
}

/// Same logic as [`resolve_connection`], parameterised for callers that
/// don't have a `TuneArgs`. Used by the `analyze --pull-bbl` path.
fn resolve_connection_raw(
    connection: Option<&str>,
    pull_bbl: bool,
) -> Result<Option<String>> {
    match connection {
        Some("auto") => Ok(Some(discover_fc_port()?)),
        Some(c) => Ok(Some(c.to_string())),
        None if pull_bbl => Ok(Some(discover_fc_port()?)),
        None => Ok(None),
    }
}

/// Open an [`FlightControllerConnection`] from a connection string.
///
/// Supported schemes:
/// - `simulator://` — in-process [`MspSimulator`] with no preloaded
///   dataflash (handshake + PID/filter round-trips work; `--pull-bbl`
///   will report an empty chip).
/// - `simulator://<path-to.bbl>` — same simulator, but its dataflash is
///   preloaded with the bytes of the given file. Lets the `--pull-bbl`
///   path be exercised end-to-end against a real BBL fixture.
/// - `serial:///dev/ttyACM0` (or a bare device path) — physical FC.
async fn open_fc_connection(
    connection: &str,
) -> Result<drone_tuner_core::realtime::FlightControllerConnection> {
    use drone_tuner_core::realtime::FlightControllerConnection;

    if let Some(rest) = connection.strip_prefix("simulator://") {
        use drone_tuner_core::realtime::{MockTransport, MspSimulator};
        let (client, server) = MockTransport::pair();
        let sim = MspSimulator::new(Box::new(server));
        if !rest.is_empty() {
            // Preload the simulator's dataflash with the file. This lets
            // `--pull-bbl --connection simulator://test_data/btfl.bbl`
            // exercise the full pull → analyze → write flow.
            let bytes = std::fs::read(rest)
                .with_context(|| format!("simulator://: failed to read preload file '{rest}'"))?;
            let mut state = sim.state.lock().unwrap();
            state.dataflash = bytes;
        }
        tokio::spawn(async move {
            let _ = sim.run().await;
        });
        FlightControllerConnection::from_transport(Box::new(client))
            .await
            .context("Failed to connect to in-process MSP simulator")
    } else {
        // Strip a leading `serial://` so callers can use a uniform scheme
        // (`serial:///dev/ttyACM0`) alongside `simulator://`. The core
        // transport parser still expects a bare device path.
        let target = connection.strip_prefix("serial://").unwrap_or(connection);
        FlightControllerConnection::connect(target)
            .await
            .context("Failed to connect to flight controller")
    }
}

/// Open the FC, read current PID, show what would change without writing.
/// Lets users verify their hardware connection works before committing
/// to a real tune.
async fn dry_connect_and_diff(
    connection: &str,
    args: &TuneArgs,
    report: &drone_tuner_core::domain::AnalysisReport,
) -> Result<()> {
    println!(
        "\n{} Dry run — connecting to {} to read current PID values (no writes)...",
        style("🔗").yellow(),
        connection
    );
    let mut fc = open_fc_connection(connection).await?;
    println!("{} Connected", display::glyph_ok());

    let current = fc
        .read_pid()
        .await
        .context("Failed to read current PID values from FC")?;
    let mut proposed = current.clone();
    let count = apply_pid_recs_to_snapshot(&mut proposed, &report.pid_recommendations, args);

    if count == 0 {
        println!(
            "\n{} No PID changes would be applied (need --auto-apply-safe or --apply-all to opt in).",
            display::glyph_info()
        );
    } else {
        println!(
            "\n{} PID change(s) WOULD be applied:",
            count
        );
        if current.roll() != proposed.roll() {
            println!(
                "    Roll  P/I/D: {:?} → {:?}",
                current.roll(),
                proposed.roll()
            );
        }
        if current.pitch() != proposed.pitch() {
            println!(
                "    Pitch P/I/D: {:?} → {:?}",
                current.pitch(),
                proposed.pitch()
            );
        }
        if current.yaw() != proposed.yaw() {
            println!(
                "    Yaw   P/I/D: {:?} → {:?}",
                current.yaw(),
                proposed.yaw()
            );
        }
    }

    // Read filter config too — surfaces whether the FC actually answers
    // MSP_FILTER_CONFIG and what its first three stable u16 fields look
    // like. Useful for diagnosing firmware versions before any writeback.
    match fc.read_filter_config().await {
        Ok(filter) => {
            println!(
                "\nFilter config (read-only): gyro_lpf1={} Hz  dterm_lpf1={} Hz  yaw_lpf={} Hz  ({} bytes)",
                filter.gyro_lpf1_hz(),
                filter.dterm_lpf1_hz(),
                filter.yaw_lpf_hz(),
                filter.as_payload().len(),
            );
            if !report.filter_recommendations.is_empty() {
                println!(
                    "    {} {} filter recommendation(s) would be printed but not auto-applied — \
                     payload offsets vary by firmware version.",
                    display::glyph_info(),
                    report.filter_recommendations.len()
                );
            }
        }
        Err(e) => {
            println!(
                "\n{} Could not read filter config: {} (continuing — PIDs are unaffected)",
                display::glyph_warn(),
                e
            );
        }
    }

    println!(
        "\n{} Dry run complete — drop --dry-run to actually apply.",
        display::glyph_info()
    );
    Ok(())
}

/// Connect to the FC, read current PID, mutate per analysis recommendations,
/// and apply via [`FlightControllerConnection::apply_pid_with_rollback`]
/// (which auto-restores the prior values if the write or its ack fails).
///
/// Behaviour:
/// - Without `--auto-apply-safe` or `--apply-all`, no recommendations are
///   applied — the user is prompted to opt in.
/// - With `--auto-apply-safe`, only Low/Medium priority recs are applied.
/// - With `--apply-all`, every rec is applied (including Critical/High).
/// - With `--backup`, the pre-change snapshot is written to disk as JSON
///   so the user can manually restore later if the rollback path fails.
/// - With `--save-eeprom`, after a successful write, EEPROM_WRITE is sent
///   so changes survive a power cycle. Without it, RAM-only changes
///   revert on next reboot — itself a useful safety net.
async fn apply_pid_recommendations_via_fc(
    connection: &str,
    bbl_path: &Path,
    args: &TuneArgs,
    report: &drone_tuner_core::domain::AnalysisReport,
) -> Result<()> {
    if !args.auto_apply_safe && !args.apply_all {
        println!(
            "\n{} No --auto-apply-safe or --apply-all flag — skipping writeback. \
             Re-run with one of those flags to actually apply changes.",
            display::glyph_info()
        );
        return Ok(());
    }

    print_stage("Apply");

    let connect_pb = make_spinner(format!("connecting to {connection}"));
    let mut fc = open_fc_connection(connection).await?;
    let info_line = match fc.fc_info() {
        Some(info) => {
            let name_suffix = if info.craft_name.is_empty() {
                String::new()
            } else {
                format!(", craft \"{}\"", info.craft_name)
            };
            format!(
                "connected: {} {} (api {}, target {}{})",
                info.firmware_id,
                info.firmware_version,
                info.api_version,
                info.target_name,
                name_suffix
            )
        }
        None => "connected".to_string(),
    };
    finish_step(connect_pb, info_line);

    // Read current PID and decide what to write.
    let read_pb = make_spinner("reading current PID values");
    let current = fc
        .read_pid()
        .await
        .context("Failed to read current PID values from FC")?;
    finish_step(
        read_pb,
        format!(
            "current: roll={:?} pitch={:?} yaw={:?}",
            current.roll(),
            current.pitch(),
            current.yaw()
        ),
    );

    // Phase-1 round-trip safety: when a backup is going to be written,
    // capture the three MSP surfaces drone-tuner doesn't tune today
    // (PidAdvanced / RcTuning / AdvancedConfig). Doing this BEFORE any
    // writeback ensures the backup reflects pre-tune state, even though
    // the tune flow doesn't currently touch these surfaces. Skipping the
    // reads when no backup is requested keeps tunes that don't need them
    // free of extra MSP roundtrips.
    let phase1_extras = if args.backup.is_some() {
        let extras_pb = make_spinner("snapshotting auxiliary MSP surfaces for backup");
        let extras = read_phase1_extras(&mut fc).await;
        let mut captured = Vec::new();
        if extras.pid_advanced.is_some() {
            captured.push("pid_advanced");
        }
        if extras.rc_tuning.is_some() {
            captured.push("rc_tuning");
        }
        if extras.advanced_config.is_some() {
            captured.push("advanced_config");
        }
        finish_step(
            extras_pb,
            format!("captured {} aux surface(s): {}", captured.len(), captured.join(", ")),
        );
        Some(extras)
    } else {
        None
    };

    let mut new_snapshot = current.clone();
    let applied = apply_pid_recs_to_snapshot(&mut new_snapshot, &report.pid_recommendations, args);

    // PID writeback is conditional: skip the write+backup roundtrip when no
    // PID rec actually changes anything, but DON'T early-return — filter
    // recs still need to flow through below. `backup` is only `Some` when
    // a PID write took place; downstream blocks (backup-to-disk, history)
    // are gated on it accordingly.
    let backup = if applied > 0 {
        println!("  {applied} PID change(s) staged:");
        if current.roll() != new_snapshot.roll() {
            println!(
                "    Roll  P/I/D: {:?} → {:?}",
                current.roll(),
                new_snapshot.roll()
            );
        }
        if current.pitch() != new_snapshot.pitch() {
            println!(
                "    Pitch P/I/D: {:?} → {:?}",
                current.pitch(),
                new_snapshot.pitch()
            );
        }
        if current.yaw() != new_snapshot.yaw() {
            println!(
                "    Yaw   P/I/D: {:?} → {:?}",
                current.yaw(),
                new_snapshot.yaw()
            );
        }

        let write_pb = make_spinner("writing PIDs (with rollback safety net)");
        let bk = fc
            .apply_pid_with_rollback(&new_snapshot)
            .await
            .context("PID writeback failed (any partial write was rolled back)")?;
        finish_step(write_pb, "PIDs written; backup retained in memory");
        Some(bk)
    } else {
        println!(
            "  {} No PID recommendations to write — proceeding to filter writeback.",
            display::glyph_info()
        );
        None
    };

    // Filter writeback via MSP_FILTER_CONFIG (cmd 92 read) +
    // MSP_SET_FILTER_CONFIG (cmd 93 write). We read the FC's authoritative
    // filter blob, mutate the bytes for the recommended fields in place,
    // and write the whole blob back via apply_filter_with_rollback (which
    // restores the pre-write snapshot on any write/ack failure). One read
    // + one write covers any number of recs.
    //
    // Why not MSP2_COMMON_SET_SETTING by name? Betaflight 4.5.x doesn't
    // implement it (verified at /home/flo/workspace/github/betaflight tag
    // 4.5.1: grep src/main/msp/ for 0x1003/0x1004 returns nothing).
    // Configurator's CLI works because the CLI is a separate REPL on the
    // FC, not an MSP path.
    let mut filter_changes_applied = 0usize;
    'filters: {
    if !args.skip_filters && !report.filter_recommendations.is_empty() {
        let read_pb = make_spinner("reading current filter config");
        let mut snapshot = match fc.read_filter_config().await {
            Ok(s) => s,
            Err(e) => {
                read_pb.finish_and_clear();
                println!(
                    "  {} could not read filter config: {} (skipping filter writeback)",
                    display::glyph_warn(),
                    e
                );
                // Bail out of the filter section but continue to backup /
                // EEPROM / history below — the PID write that already
                // succeeded shouldn't be torpedoed by an unrelated MSP
                // read failure.
                break 'filters;
            }
        };
        finish_step(
            read_pb,
            format!(
                "current filter config: {} byte payload",
                snapshot.payload_len()
            ),
        );

        let (descriptions, applied_count, unsupported) =
            apply_filter_recs_to_snapshot(&mut snapshot, &report.filter_recommendations, args);
        filter_changes_applied = applied_count;

        if applied_count == 0 {
            if !unsupported.is_empty() {
                println!(
                    "  {} filter recs surfaced but none auto-applicable on this build:",
                    display::glyph_info(),
                );
                for u in &unsupported {
                    println!("    - {u}");
                }
            } else {
                println!(
                    "  {} no filter recs match priority gating (--apply-all / --auto-apply-safe).",
                    display::glyph_info()
                );
            }
        } else {
            println!("  {applied_count} filter byte-mutation(s) staged:");
            for d in &descriptions {
                println!("    {d}");
            }
            let filter_pb = make_spinner("writing filter config (with rollback safety net)");
            match fc.apply_filter_with_rollback(&snapshot).await {
                Ok(_backup) => {
                    finish_step(
                        filter_pb,
                        format!("{applied_count} filter byte-mutation(s) written"),
                    );
                }
                Err(e) => {
                    filter_pb.finish_and_clear();
                    println!(
                        "  {} filter writeback failed: {} (rollback restored pre-change state)",
                        display::glyph_warn(),
                        e
                    );
                    filter_changes_applied = 0;
                }
            }
            if !unsupported.is_empty() {
                println!(
                    "  {} {} filter rec(s) skipped (FC's filter config payload too short):",
                    display::glyph_info(),
                    unsupported.len()
                );
                for u in &unsupported {
                    println!("    - {u}");
                }
            }
        }
    } else if args.skip_filters {
        println!(
            "  {} --skip-filters set — filter recs printed but not written.",
            display::glyph_info()
        );
    }
    } // 'filters: { ... } block

    // Advanced parameter writeback via MSP_PID_ADVANCED (cmd 94).
    // Reads the current snapshot, mutates bytes for recommended fields,
    // writes back. Follows the same priority-gating as PID/filter recs.
    let mut advanced_changes_applied = 0usize;
    if !report.advanced_recommendations.is_empty()
        && (args.auto_apply_safe || args.apply_all)
    {
        let read_pb = make_spinner("reading PID_ADVANCED for advanced tuning");
        match fc.read_pid_advanced().await {
            Ok(mut snapshot) => {
                finish_step(
                    read_pb,
                    format!("PID_ADVANCED: {} byte payload", snapshot.payload_len()),
                );

                let allow_priority = |p: &drone_tuner_core::domain::Priority| {
                    if args.apply_all {
                        true
                    } else if args.auto_apply_safe {
                        matches!(
                            p,
                            drone_tuner_core::domain::Priority::Low
                                | drone_tuner_core::domain::Priority::Medium
                        )
                    } else {
                        false
                    }
                };

                let mut descs: Vec<String> = Vec::new();
                for rec in &report.advanced_recommendations {
                    if !allow_priority(&rec.priority) {
                        continue;
                    }
                    use drone_tuner_core::domain::AdvancedParameter::*;
                    match &rec.parameter {
                        VbatSagCompensation { recommended, .. } => {
                            if let Ok(()) = snapshot.set_vbat_sag_compensation(*recommended) {
                                advanced_changes_applied += 1;
                                descs.push(format!("vbat_sag_compensation → {recommended}"));
                            }
                        }
                        DynamicIdle { recommended_rpm, .. } => {
                            if let Ok(()) = snapshot.set_idle_min_rpm(*recommended_rpm) {
                                advanced_changes_applied += 1;
                                descs.push(format!("idle_min_rpm → {recommended_rpm}"));
                            }
                        }
                        ThrustLinearization { recommended, .. } => {
                            if let Ok(()) = snapshot.set_thrust_linearization(*recommended) {
                                advanced_changes_applied += 1;
                                descs.push(format!("thrust_linearization → {recommended}"));
                            }
                        }
                        DMax { axis, recommended_d_min, .. } => {
                            use drone_tuner_core::domain::Axis;
                            let result = match axis {
                                Axis::Roll => snapshot.set_d_min_roll(*recommended_d_min),
                                Axis::Pitch => snapshot.set_d_min_pitch(*recommended_d_min),
                                Axis::Yaw => continue,
                            };
                            if result.is_ok() {
                                advanced_changes_applied += 1;
                                descs.push(format!("d_min_{:?} → {}", axis, recommended_d_min));
                            }
                        }
                        Feedforward { param, recommended, .. } => {
                            use drone_tuner_core::domain::FeedforwardParam;
                            let result = match param {
                                FeedforwardParam::JitterFactor => snapshot.set_ff_jitter_factor(*recommended),
                                FeedforwardParam::Boost => snapshot.set_ff_boost(*recommended),
                                _ => continue,
                            };
                            if result.is_ok() {
                                advanced_changes_applied += 1;
                                descs.push(format!("feedforward_{:?} → {}", param, recommended));
                            }
                        }
                        Tpa { .. } => {
                            // TPA lives in RC_TUNING — skip for now
                        }
                        SliderHint { .. } => {
                            // Informational only — slider hints are not MSP-writable
                        }
                    }
                }

                if advanced_changes_applied > 0 {
                    println!("  {} advanced byte-mutation(s) staged:", advanced_changes_applied);
                    for d in &descs {
                        println!("    {d}");
                    }
                    let write_pb = make_spinner("writing PID_ADVANCED");
                    match fc.write_pid_advanced(&snapshot).await {
                        Ok(()) => {
                            finish_step(
                                write_pb,
                                format!("{advanced_changes_applied} advanced change(s) written"),
                            );
                        }
                        Err(e) => {
                            write_pb.finish_and_clear();
                            println!(
                                "  {} PID_ADVANCED writeback failed: {}",
                                display::glyph_warn(),
                                e
                            );
                            advanced_changes_applied = 0;
                        }
                    }
                }
            }
            Err(e) => {
                read_pb.finish_and_clear();
                println!(
                    "  {} could not read PID_ADVANCED: {} (skipping advanced writeback)",
                    display::glyph_warn(),
                    e
                );
            }
        }
    }

    // Persist backup to disk if requested. Only runs when a PID write
    // actually happened — there's nothing to back up otherwise. Default
    // filename embeds the craft name (sanitised) so `ls tune-backup-*.json`
    // is human-readable when multiple quads share a workdir.
    if let (Some(maybe_path), Some(bk)) = (&args.backup, &backup) {
        let path = maybe_path.clone().unwrap_or_else(|| {
            let ts = chrono::Utc::now().format("%Y%m%d-%H%M%S");
            let slug = fc.fc_info().and_then(craft_name_slug);
            let filename = match slug {
                Some(name) => format!("tune-backup-{name}-{ts}.json"),
                None => format!("tune-backup-{ts}.json"),
            };
            // Default into the central backups dir so they accumulate in
            // one searchable place across crafts. CWD fallback only if
            // the data dir is unresolvable (no $HOME etc.).
            paths::backups_dir()
                .map(|d| d.join(&filename))
                .unwrap_or_else(|_| PathBuf::from(filename))
        });
        // v2 schema adds the three Phase-1 round-trip surfaces. Each
        // is null-emitted when the FC didn't answer the corresponding
        // MSP read (so a stripped vendor build still produces a usable
        // backup that just lacks the absent fields). v1 readers ignore
        // the new fields; v2 readers tolerate their absence.
        let pid_advanced_json = phase1_extras
            .as_ref()
            .and_then(|e| e.pid_advanced.as_ref().map(|s| s.as_payload().to_vec()));
        let rc_tuning_json = phase1_extras
            .as_ref()
            .and_then(|e| e.rc_tuning.as_ref().map(|s| s.as_payload().to_vec()));
        let advanced_config_json = phase1_extras
            .as_ref()
            .and_then(|e| e.advanced_config.as_ref().map(|s| s.as_payload().to_vec()));
        let json = serde_json::to_string_pretty(&serde_json::json!({
            "schema": BACKUP_SCHEMA_V2,
            "captured_at": chrono::Utc::now(),
            "pid_payload": bk.as_payload(),
            "roll": bk.roll(),
            "pitch": bk.pitch(),
            "yaw": bk.yaw(),
            "pid_advanced_payload": pid_advanced_json,
            "rc_tuning_payload": rc_tuning_json,
            "advanced_config_payload": advanced_config_json,
        }))?;
        std::fs::write(&path, json)
            .with_context(|| format!("Failed to write backup snapshot to {}", path.display()))?;
        println!("  {} backup → {}", "", path.display());
    }

    // EEPROM save runs whenever *something* was written — PIDs, filters,
    // or both. Skipping it on filter-only changes would silently drop them
    // on the next power cycle, which is the bug we just fixed.
    let any_writes_applied = applied > 0 || filter_changes_applied > 0 || advanced_changes_applied > 0;
    let mut persisted_to_eeprom = false;
    if args.save_eeprom && any_writes_applied {
        let save_pb = make_spinner("persisting to EEPROM");
        fc.save_to_eeprom().await.context(
            "EEPROM write failed; RAM changes are still in effect but will revert on power cycle",
        )?;
        persisted_to_eeprom = true;
        finish_step(save_pb, "changes persisted across power cycles");
    } else if any_writes_applied {
        println!(
            "  {} Changes are RAM-only and will revert on power cycle. \
             Re-run with --save-eeprom to persist.",
            display::glyph_info()
        );
    }

    // Append a row to the per-FC tune history — only when a PID change
    // produced a backup snapshot, since the history schema keys on that.
    // Filter-only writes don't get a history row yet (TODO).
    if let Some(bk) = &backup {
        if let Err(e) = record_history(
            &fc,
            bbl_path,
            bk,
            &new_snapshot,
            applied,
            persisted_to_eeprom,
        ) {
            warn!("Failed to append tune history entry: {e:#}");
        }
    }

    println!("\n  {} Tune complete.", display::glyph_ok());

    Ok(())
}

/// Build and append a [`history::TuneHistoryEntry`] for a successful write.
fn record_history(
    fc: &drone_tuner_core::realtime::FlightControllerConnection,
    bbl_path: &Path,
    pre: &drone_tuner_core::realtime::PidSnapshot,
    post: &drone_tuner_core::realtime::PidSnapshot,
    recommendations_applied: usize,
    persisted_to_eeprom: bool,
) -> Result<()> {
    use drone_tuner_core::realtime::FlightControllerInfo;
    let info: &FlightControllerInfo = fc
        .fc_info()
        .context("FC connection has no handshake info; refusing to log history")?;
    let entry = history::TuneHistoryEntry {
        schema: "drone-tuner-history-v1",
        timestamp: chrono::Utc::now(),
        fc: history::FcIdentity {
            board_id: &info.board_id,
            target_name: &info.target_name,
            firmware_id: &info.firmware_id,
            firmware_version: &info.firmware_version,
        },
        bbl: history::BblIdentity::from_path(bbl_path)?,
        pids_before: history::PidTriples::from_snapshot(pre),
        pids_after: history::PidTriples::from_snapshot(post),
        recommendations_applied,
        persisted_to_eeprom,
    };
    let path = history::append(&entry)?;
    println!(
        "Tune logged to {} ({} {})",
        path.display(),
        info.board_id,
        info.target_name,
    );
    Ok(())
}

/// One Betaflight setting we want to write, as a `(name, value-bytes)`
/// pair plus a human-readable label for stdout.
///
fn format_advanced_rec(param: &drone_tuner_core::domain::AdvancedParameter) -> String {
    use drone_tuner_core::domain::AdvancedParameter::*;
    match param {
        VbatSagCompensation { current, recommended } => {
            format!("Vbat Sag Compensation: {} → {}", current, recommended)
        }
        DynamicIdle { current_rpm, recommended_rpm } => {
            format!("Dynamic Idle: {} → {} (x100 RPM)", current_rpm, recommended_rpm)
        }
        DMax { axis, current_d_min, current_d_max, recommended_d_min, recommended_d_max } => {
            format!(
                "{:?} D range: {}-{} → {}-{}",
                axis, current_d_min, current_d_max, recommended_d_min, recommended_d_max
            )
        }
        Tpa { current_rate, current_breakpoint, recommended_rate, recommended_breakpoint } => {
            format!(
                "TPA: {}%@{} → {}%@{}",
                current_rate, current_breakpoint, recommended_rate, recommended_breakpoint
            )
        }
        Feedforward { param, current, recommended } => {
            format!("Feedforward {:?}: {} → {}", param, current, recommended)
        }
        ThrustLinearization { current, recommended } => {
            format!("Thrust Linearization: {} → {}", current, recommended)
        }
        SliderHint { slider_name, current_value, suggested_value } => {
            format!(
                "Slider {}: {:.2} → {:.2}",
                slider_name,
                *current_value as f32 / 100.0,
                *suggested_value as f32 / 100.0,
            )
        }
    }
}

/// Map a Betaflight filter-type string ("PT1", "BIQUAD", "PT2", "PT3") to
/// its `filterType_e` enum integer. Returns `None` for unknown types so
/// the caller skips the filter-type setting and only writes the cutoff.
fn filter_type_to_enum(name: &str) -> Option<u8> {
    match name.to_ascii_uppercase().as_str() {
        "PT1" => Some(0),
        "BIQUAD" => Some(1),
        "PT2" => Some(2),
        "PT3" => Some(3),
        _ => None,
    }
}

/// Apply a list of [`FilterRecommendation`]s onto a [`FilterSnapshot`] in
/// place. Returns `(descriptions, applied, unsupported)` where:
/// - `descriptions` lists the staged changes for stdout display.
/// - `applied` counts byte-mutations that landed (multiple per rec is
///   normal — e.g. AdjustDynamicNotch writes count + min + max).
/// - `unsupported` lists fields the FC's filter config payload was too
///   short to cover (rare on vanilla 4.5, common on stripped vendor
///   builds).
fn apply_filter_recs_to_snapshot(
    snap: &mut drone_tuner_core::realtime::FilterSnapshot,
    recs: &[drone_tuner_core::domain::FilterRecommendation],
    args: &TuneArgs,
) -> (Vec<String>, usize, Vec<String>) {
    use drone_tuner_core::domain::{FilterRecommendationType::*, Priority};

    let allow_priority = |p: &Priority| {
        if args.apply_all {
            true
        } else if args.auto_apply_safe {
            matches!(p, Priority::Low | Priority::Medium)
        } else {
            false
        }
    };

    let mut descriptions: Vec<String> = Vec::new();
    let mut applied = 0usize;
    let mut unsupported: Vec<String> = Vec::new();

    // Try a setter; on success increment counters and push the
    // description; on Err push the field's reason into `unsupported`.
    macro_rules! try_set {
        ($call:expr, $desc:expr) => {
            match $call {
                Ok(()) => {
                    applied += 1;
                    descriptions.push($desc);
                }
                Err(e) => {
                    unsupported.push(format!("{e}"));
                }
            }
        };
    }

    for rec in recs {
        if !allow_priority(&rec.priority) {
            continue;
        }
        match &rec.recommendation_type {
            AdjustGyroLowpass {
                stage,
                recommended_cutoff,
                filter_type,
                ..
            } => {
                let v = recommended_cutoff.round().clamp(0.0, 65535.0) as u16;
                match stage {
                    1 => try_set!(snap.set_gyro_lpf1_hz(v), format!("gyro_lpf1_static_hz → {v} Hz")),
                    2 => try_set!(snap.set_gyro_lpf2_hz(v), format!("gyro_lpf2_static_hz → {v} Hz")),
                    other => unsupported.push(format!("gyro_lpf{other} (unsupported stage)")),
                }
                if let Some(ft) = filter_type_to_enum(filter_type) {
                    match stage {
                        1 => try_set!(snap.set_gyro_lpf1_type(ft), format!("gyro_lpf1_type → {filter_type}")),
                        2 => try_set!(snap.set_gyro_lpf2_type(ft), format!("gyro_lpf2_type → {filter_type}")),
                        _ => {}
                    }
                }
            }
            AdjustDtermLowpass {
                stage,
                recommended_cutoff,
                filter_type,
                dynamic_settings,
                ..
            } => {
                if let Some(cutoff) = recommended_cutoff {
                    if dynamic_settings.is_none() {
                        let v = cutoff.round().clamp(0.0, 65535.0) as u16;
                        match stage {
                            1 => {
                                try_set!(
                                    snap.set_dterm_lpf1_hz(v),
                                    format!("dterm_lpf1_static_hz → {v} Hz")
                                );
                                // Disable dynamic by zeroing both bounds.
                                try_set!(
                                    snap.set_dterm_lpf1_dyn(0, 0),
                                    "dterm_lpf1 dynamic disabled (min=max=0)".to_string()
                                );
                            }
                            2 => try_set!(
                                snap.set_dterm_lpf2_hz(v),
                                format!("dterm_lpf2_static_hz → {v} Hz")
                            ),
                            other => unsupported
                                .push(format!("dterm_lpf{other} (unsupported stage)")),
                        }
                    }
                }
                if let Some(d) = dynamic_settings {
                    if *stage == 1 {
                        let lo = d.min_cutoff.round().clamp(0.0, 65535.0) as u16;
                        let hi = d.max_cutoff.round().clamp(0.0, 65535.0) as u16;
                        try_set!(
                            snap.set_dterm_lpf1_dyn(lo, hi),
                            format!("dterm_lpf1 dyn → {lo}–{hi} Hz")
                        );
                    } else {
                        unsupported.push(format!(
                            "dterm_lpf{stage} dynamic (only stage 1 has dyn min/max in MSP_FILTER_CONFIG)"
                        ));
                    }
                }
                if let Some(ft) = filter_type_to_enum(filter_type) {
                    match stage {
                        1 => try_set!(
                            snap.set_dterm_lpf1_type(ft),
                            format!("dterm_lpf1_type → {filter_type}")
                        ),
                        2 => try_set!(
                            snap.set_dterm_lpf2_type(ft),
                            format!("dterm_lpf2_type → {filter_type}")
                        ),
                        _ => {}
                    }
                }
            }
            AdjustYawLowpass {
                recommended_cutoff, ..
            } => {
                let v = recommended_cutoff.round().clamp(0.0, 65535.0) as u16;
                try_set!(
                    snap.set_yaw_lpf_hz(v),
                    format!("yaw_lowpass_hz → {v} Hz")
                );
            }
            AdjustDynamicNotch {
                notch_count,
                min_freq,
                max_freq,
                enabled,
                ..
            } => {
                // Betaflight 4.5's `validateAndFixGyroConfig` clamps
                // server-side too, but we clamp here so the staged
                // descriptions show truthful values and we don't
                // surprise anyone with silent firmware corrections.
                const COUNT_MIN: u8 = 0;
                const COUNT_MAX: u8 = 5;
                const MIN_HZ_MIN: u16 = 60;
                const MIN_HZ_MAX: u16 = 250;
                const MAX_HZ_MIN: u16 = 200;
                const MAX_HZ_MAX: u16 = 1000;

                if *enabled {
                    let raw_min = min_freq.round().clamp(0.0, 65535.0) as u16;
                    let raw_max = max_freq.round().clamp(0.0, 65535.0) as u16;
                    let clamped_min = raw_min.clamp(MIN_HZ_MIN, MIN_HZ_MAX);
                    let clamped_max = raw_max.clamp(MAX_HZ_MIN, MAX_HZ_MAX);
                    let clamped_count = (*notch_count).clamp(COUNT_MIN, COUNT_MAX);

                    let min_note = if clamped_min != raw_min {
                        format!(" (clamped from {raw_min})")
                    } else {
                        String::new()
                    };
                    let max_note = if clamped_max != raw_max {
                        format!(" (clamped from {raw_max})")
                    } else {
                        String::new()
                    };
                    let count_note = if clamped_count != *notch_count {
                        format!(" (clamped from {})", *notch_count)
                    } else {
                        String::new()
                    };

                    try_set!(
                        snap.set_dyn_notch_count(clamped_count),
                        format!("dyn_notch_count → {clamped_count}{count_note}")
                    );
                    try_set!(
                        snap.set_dyn_notch_min_hz(clamped_min),
                        format!("dyn_notch_min_hz → {clamped_min} Hz{min_note}")
                    );
                    try_set!(
                        snap.set_dyn_notch_max_hz(clamped_max),
                        format!("dyn_notch_max_hz → {clamped_max} Hz{max_note}")
                    );
                } else {
                    try_set!(
                        snap.set_dyn_notch_count(0),
                        "dyn_notch_count → 0 (disable)".to_string()
                    );
                }
            }
            ConfigureRpmFilter {
                harmonics,
                min_freq,
                enabled,
                ..
            } => {
                if *enabled {
                    try_set!(
                        snap.set_rpm_filter_harmonics(*harmonics),
                        format!("rpm_filter_harmonics → {harmonics}")
                    );
                    // rpm_filter_min_hz is u8 in 4.5 (offset 44).
                    let v = min_freq.round().clamp(0.0, 255.0) as u8;
                    try_set!(
                        snap.set_rpm_filter_min_hz(v),
                        format!("rpm_filter_min_hz → {v} Hz")
                    );
                } else {
                    try_set!(
                        snap.set_rpm_filter_harmonics(0),
                        "rpm_filter_harmonics → 0 (disable)".to_string()
                    );
                }
            }
            ConfigureGyroNotch { .. } => {
                unsupported.push(
                    "static gyro notches: Hz/cutoff Q derivation shifted between \
                     firmware versions; not auto-applied yet"
                        .to_string(),
                );
            }
        }
    }

    (descriptions, applied, unsupported)
}

/// Translate a list of [`PidRecommendation`]s onto a [`PidSnapshot`] in
/// place. Returns the number of changes actually applied.
///
/// `recommended_value` is in Betaflight's internal scale (typically 0..=255),
/// so we just clamp and round to u8.
fn apply_pid_recs_to_snapshot(
    snapshot: &mut drone_tuner_core::realtime::PidSnapshot,
    recommendations: &[drone_tuner_core::domain::PidRecommendation],
    args: &TuneArgs,
) -> usize {
    use drone_tuner_core::domain::{Axis, PidTerm, Priority};

    let allow_priority = |p: &Priority| {
        if args.apply_all {
            true
        } else if args.auto_apply_safe {
            matches!(p, Priority::Low | Priority::Medium)
        } else {
            false
        }
    };

    let mut count = 0usize;
    for rec in recommendations {
        if !allow_priority(&rec.priority) {
            continue;
        }
        let new_val = rec.recommended_value.round().clamp(0.0, 255.0) as u8;
        let (mut p, mut i, mut d) = match rec.axis {
            Axis::Roll => snapshot.roll(),
            Axis::Pitch => snapshot.pitch(),
            Axis::Yaw => snapshot.yaw(),
        };
        match rec.term {
            PidTerm::P => p = new_val,
            PidTerm::I => i = new_val,
            PidTerm::D => d = new_val,
            // F-term doesn't fit in MSP_PID's first 9 bytes; skip and let
            // the user know.
            PidTerm::F => continue,
        }
        match rec.axis {
            Axis::Roll => snapshot.set_roll(p, i, d),
            Axis::Pitch => snapshot.set_pitch(p, i, d),
            Axis::Yaw => snapshot.set_yaw(p, i, d),
        }
        count += 1;
    }
    count
}

/// Handle the export command
async fn export_command(args: ExportArgs, _output_format: OutputFormat) -> Result<()> {
    println!(
        "{} Exporting analysis data to {}",
        style("📤").blue(),
        args.output.display()
    );

    // Analyze the file if it's a blackbox
    let analysis = if args
        .input
        .extension()
        .is_some_and(|ext| ext == "bbl" || ext == "BBL")
    {
        let analyze_args = AnalyzeArgs {
            input: Some(args.input.clone()),
            output_dir: None,
            detailed: true,
            show_details: false,
            min_confidence: 0.5,
            max_files: 1,
            session: None,
            list_sessions: false,
            bb_summary: false,
            session_strategy: None,
            connection: None,
            pull_bbl: false,
            keep_bbl: None,
            erase_after_pull: false,
            pull_chunk_size: None,
        };

        let mut engine = AnalysisEngine::new();
        Some(analyze_single_file(&mut engine, &args.input, &analyze_args).await?)
    } else {
        None
    };

    // Export based on format
    match args.format.as_str() {
        "csv" => {
            if let Some(analysis) = analysis {
                export_to_csv(&analysis, &args.output, args.include_raw, args.include_fft).await?;
            } else {
                return Err(anyhow::anyhow!("CSV export requires blackbox analysis"));
            }
        }
        "json" => {
            if let Some(analysis) = analysis {
                export_to_json(&analysis, &args.output, args.include_raw, args.include_fft).await?;
            } else {
                return Err(anyhow::anyhow!("JSON export requires blackbox analysis"));
            }
        }
        "matlab" => {
            if let Some(analysis) = analysis {
                export_to_matlab(&analysis, &args.output, args.include_raw, args.include_fft)
                    .await?;
            } else {
                return Err(anyhow::anyhow!("MATLAB export requires blackbox analysis"));
            }
        }
        "python" => {
            if let Some(analysis) = analysis {
                export_to_python(&analysis, &args.output, args.include_raw, args.include_fft)
                    .await?;
            } else {
                return Err(anyhow::anyhow!("Python export requires blackbox analysis"));
            }
        }
        _ => {
            return Err(anyhow::anyhow!(
                "Unsupported export format: {}",
                args.format
            ));
        }
    }

    println!("{} Export completed successfully", display::glyph_ok());
    Ok(())
}

/// Handle the info command. Prints a single comfy-table panel below the
/// banner — version, system, and library readiness in one place.
async fn info_command() -> Result<()> {
    display::info_panel(
        env!("CARGO_PKG_VERSION"),
        drone_tuner_core::VERSION,
        std::env::consts::OS,
        std::env::consts::ARCH,
        &[
            ("FFT support", true),
            ("Scientific computing", true),
            ("Blackbox parsing", true),
        ],
    );

    // Surface where the tool keeps its state so the user knows where
    // backups, baselines, and pulls land.
    if let Ok(root) = paths::data_root() {
        println!();
        println!("  Data directory: {}", root.display());
        println!("    backups/     tune-backup-*.json (auto-saved snapshots)");
        println!("    baselines/   pinned per-craft anchors (--baseline)");
        println!("    pulls/       BBL files pulled from FCs");
        println!("    history.jsonl  cross-tune history");
    }
    Ok(())
}

/// Result of analyzing a single file
struct AnalysisResult {
    session: FlightSession,
    report: drone_tuner_core::AnalysisReport,
    analysis_time: std::time::Duration,
    file_path: PathBuf,
}

/// Comparison between multiple flights
struct FlightComparison {
    flights: Vec<ComparisonFlight>,
    summary: ComparisonSummary,
}

/// Single flight in comparison
struct ComparisonFlight {
    name: String,
    tune_quality: f32,
    issues_count: usize,
    recommendations_count: usize,
    duration_ms: u64,
}

/// Summary of comparison
struct ComparisonSummary {
    best_tune_quality: f32,
    worst_tune_quality: f32,
    avg_tune_quality: f32,
    total_issues: usize,
    common_issues: Vec<String>,
}

/// Output analysis results in the specified format
async fn output_analysis_results(
    results: &[(PathBuf, Result<AnalysisResult, anyhow::Error>)],
    format: &OutputFormat,
    output_dir: Option<&PathBuf>,
    detailed_info: bool,
) -> Result<()> {
    match format {
        OutputFormat::Pretty => output_pretty(results, detailed_info)?,
        OutputFormat::Json => output_json(results, output_dir).await?,
        OutputFormat::Csv => output_csv(results, output_dir).await?,
    }
    Ok(())
}

/// Output results in human-readable format with detailed file and configuration information
fn output_pretty(
    results: &[(PathBuf, Result<AnalysisResult, anyhow::Error>)],
    detailed_info: bool,
) -> Result<()> {
    for (file_path, result) in results {
        match result {
            Ok(analysis) => {
                println!();
                println!("{}", file_path.display());

                // Try to determine which session was analyzed based on frame count
                let frame_count = analysis.session.telemetry.gyro.len();
                let session_hint = match frame_count {
                    15707 => " (Session 12 of 12)",
                    30107 => " (Session 1 of 12)",
                    _ => " (Multi-session file)",
                };

                println!(
                    "  Duration: {:.1}s{}",
                    analysis.session.metadata.duration_ms as f32 / 1000.0,
                    session_hint
                );
                println!("  Samples: {}", analysis.session.telemetry.gyro.len());
                println!(
                    "  Sample rate: {:.0} Hz",
                    analysis.session.telemetry.sample_rate
                );
                println!(
                    "  Analysis time: {:.2}s",
                    analysis.analysis_time.as_secs_f32()
                );

                // Show detailed information if requested
                if detailed_info {
                    println!();
                    // File Details Section
                    println!("File Details:");
                    if let Ok(metadata) = std::fs::metadata(file_path) {
                        let file_size_mb = metadata.len() as f32 / (1024.0 * 1024.0);
                        println!("  Size: {:.1} MB", file_size_mb);
                    }

                    // Show session information - this is estimated based on typical multi-session files
                    println!("  Sessions detected: Multiple (use --list-sessions to see all)");
                    println!("  Processing: Most recent session");
                    let total_frames = analysis.session.telemetry.gyro.len();
                    println!(
                        "  Session samples: {} | Main frames processed: {}",
                        total_frames, total_frames
                    );
                    println!("  Use --session N to analyze specific session");
                    println!();

                    // Flight Controller Configuration Section
                    println!("Flight Controller Configuration:");
                    let fc = &analysis.session.metadata.hardware.flight_controller;
                    println!("  Firmware: {} {}", fc.firmware, fc.version);
                    println!("  Target: {}", fc.target);
                    println!("  Loop rate: {}Hz", fc.loop_rate);

                    // Calculate and show sample rate derivation
                    let base_rate = fc.loop_rate as f32;
                    let effective_rate = analysis.session.telemetry.sample_rate;
                    if base_rate > 0.0 && (base_rate - effective_rate).abs() > 0.1 {
                        let interval_ratio = base_rate / effective_rate;
                        println!(
                            "  Sample rate calculation: {}Hz ÷ {:.0} = {:.0}Hz",
                            base_rate, interval_ratio, effective_rate
                        );
                    } else {
                        println!("  Sample rate: {:.0}Hz (direct)", effective_rate);
                    }
                    println!();

                    // PID Values Section
                    let pid = &analysis.session.metadata.hardware.pid_config;
                    println!("  {} PID Values:", "");
                    println!(
                        "    Roll:  P={:.1}, I={:.1}, D={:.1}",
                        pid.roll.p, pid.roll.i, pid.roll.d
                    );
                    println!(
                        "    Pitch: P={:.1}, I={:.1}, D={:.1}",
                        pid.pitch.p, pid.pitch.i, pid.pitch.d
                    );
                    println!(
                        "    Yaw:   P={:.1}, I={:.1}, D={:.1}",
                        pid.yaw.p, pid.yaw.i, pid.yaw.d
                    );
                    println!();

                    // Filter Settings Section
                    let filters = &analysis.session.metadata.hardware.filter_config;
                    println!("  {} Filter Settings:", "");

                    // Gyro filters
                    if !filters.gyro_filters.is_empty() {
                        for filter in &filters.gyro_filters {
                            println!(
                                "    Gyro LPF: {:.0}Hz ({:?}, order {})",
                                filter.cutoff, filter.filter_type, filter.order
                            );
                        }
                    } else {
                        println!("    Gyro LPF: Not configured");
                    }

                    // D-term filters
                    if !filters.dterm_filters.is_empty() {
                        for filter in &filters.dterm_filters {
                            println!(
                                "    D-term LPF: {:.0}Hz ({:?}, order {})",
                                filter.cutoff, filter.filter_type, filter.order
                            );
                        }
                    } else {
                        println!("    D-term LPF: Not configured");
                    }

                    // Dynamic notch
                    if let Some(dyn_notch) = &filters.dynamic_notch {
                        if dyn_notch.enabled {
                            println!(
                                "    Dynamic Notch: {:.0}-{:.0}Hz (Q={:.0})",
                                dyn_notch.min_freq, dyn_notch.max_freq, dyn_notch.q_factor
                            );
                        } else {
                            println!("    Dynamic Notch: Disabled");
                        }
                    } else {
                        println!("    Dynamic Notch: Not configured");
                    }

                    // Static notch filters
                    if !filters.notch_filters.is_empty() {
                        for notch in &filters.notch_filters {
                            if notch.enabled {
                                println!(
                                    "    Static Notch: {:.0}Hz (Q={:.1})",
                                    notch.frequency, notch.q_factor
                                );
                            }
                        }
                    } else {
                        println!("    Static Notch: None configured");
                    }
                    println!();

                    // RC Rates Section
                    let rates = &pid.settings.rates;
                    println!("  {} RC Rates:", "");
                    println!(
                        "    Rates: R={:.1}, P={:.1}, Y={:.1}",
                        rates.roll_rate, rates.pitch_rate, rates.yaw_rate
                    );
                    println!(
                        "    Expo: R={:.2}, P={:.2}, Y={:.2}",
                        rates.expo.roll, rates.expo.pitch, rates.expo.yaw
                    );
                    println!(
                        "    Super Rate: R={:.2}, P={:.2}, Y={:.2}",
                        rates.super_rate.roll, rates.super_rate.pitch, rates.super_rate.yaw
                    );
                    println!();

                    // Verification Notes Section
                    println!("Verification Notes:");
                    println!(
                        "  - Duration calculated from {} samples at {:.0}Hz",
                        analysis.session.telemetry.gyro.len(),
                        analysis.session.telemetry.sample_rate
                    );
                    println!("  - Compare PID values with Betaflight Configurator");
                    println!("  - Check filter settings match your setup");
                    if fc.firmware.contains("Unknown") || fc.version.contains("Unknown") {
                        println!(
                            "  {} Firmware info extraction incomplete - check manually",
                            display::glyph_warn()
                        );
                    }
                    println!();
                }

                if !detailed_info {
                    println!();
                }

                // Tune quality gauge — score with coloured bar + verdict band.
                if detailed_info {
                    println!("Analysis Results:");
                }
                println!(
                    "  {}",
                    display::quality_gauge(analysis.report.tune_quality_score)
                );

                // Issues
                if !analysis.report.detected_issues.is_empty() {
                    println!(
                        "  {} Issues found:",
                        display::glyph_warn()
                    );
                    for issue in &analysis.report.detected_issues {
                        println!("    • {}", issue.description);
                    }
                }

                // Recommendations: rendered as a single combined comfy-table
                // (PID + filter rows together), priority-coloured per row.
                println!();
                display::recommendations_table_full(
                    &analysis.report.pid_recommendations,
                    &analysis.report.filter_recommendations,
                    &analysis.report.advanced_recommendations,
                );

                // Step-response viz, only in --show-details mode. Surface
                // *what the analyser saw* per axis so the recommendations
                // above stop being a black box.
                if detailed_info && !analysis.report.step_responses.is_empty() {
                    render_step_responses(&analysis.report.step_responses);
                }
            }
            Err(e) => {
                println!();
                display::err(format!("{} - Error: {}", file_path.display(), e));
            }
        }
    }
    Ok(())
}

/// Render the most informative step responses per axis as a compact ASCII
/// chart of (rc command, gyro response, target). "Most informative" means
/// the largest stick deflection — that's where rise time and overshoot
/// are observable, and where the SS-error metric (if any) was computed.
fn render_step_responses(responses: &[drone_tuner_core::analysis::StepResponse]) {
    use drone_tuner_core::domain::Axis;

    println!();
    println!("  {} Step responses (top per axis):", "");
    for axis in [Axis::Roll, Axis::Pitch, Axis::Yaw] {
        let mut by_axis: Vec<&drone_tuner_core::analysis::StepResponse> =
            responses.iter().filter(|r| r.axis == axis).collect();
        if by_axis.is_empty() {
            continue;
        }
        // Largest command magnitude first — the cleanest signal.
        by_axis.sort_by(|a, b| {
            b.command_magnitude
                .partial_cmp(&a.command_magnitude)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        let pick = by_axis[0];
        println!(
            "    {:?} @ t={:.2}s  Δstick={:.2}  rise={:.0}ms  overshoot={:.1}%  SS={}",
            axis,
            pick.start_time,
            pick.command_magnitude,
            pick.rise_time * 1000.0,
            pick.overshoot_percent,
            match pick.steady_state_error_dps {
                Some(e) => format!("{:.1} dps", e),
                None => "—".to_string(),
            }
        );
        print_sparkline("rc ", &pick.command_trace);
        print_sparkline("gyr", &pick.gyro_trace);
    }
}

// Sparkline rendering moved to `display::print_sparkline`.
use display::print_sparkline;

/// Output results as JSON
async fn output_json(
    results: &[(PathBuf, Result<AnalysisResult, anyhow::Error>)],
    output_dir: Option<&PathBuf>,
) -> Result<()> {
    let json_results: Vec<serde_json::Value> = results
        .iter()
        .map(|(file_path, result)| match result {
            Ok(analysis) => serde_json::json!({
                "file": file_path.display().to_string(),
                "status": "success",
                "tune_quality": analysis.report.tune_quality_score,
                "duration_ms": analysis.session.metadata.duration_ms,
                "sample_rate": analysis.session.telemetry.sample_rate,
                "samples": analysis.session.telemetry.gyro.len(),
                "analysis_time_ms": analysis.analysis_time.as_millis(),
                "issues": analysis.report.detected_issues.len(),
                "filter_recommendations": analysis.report.filter_recommendations.len(),
                "pid_recommendations": analysis.report.pid_recommendations.len()
            }),
            Err(e) => serde_json::json!({
                "file": file_path.display().to_string(),
                "status": "error",
                "error": e.to_string()
            }),
        })
        .collect();

    let output = serde_json::json!({
        "version": env!("CARGO_PKG_VERSION"),
        "timestamp": chrono::Utc::now(),
        "results": json_results
    });

    let json_str = serde_json::to_string_pretty(&output)?;

    if let Some(output_dir) = output_dir {
        std::fs::create_dir_all(output_dir)?;
        let output_file = output_dir.join("analysis_results.json");
        std::fs::write(&output_file, json_str)?;
        println!("JSON results written to {}", output_file.display());
    } else {
        println!("{}", json_str);
    }

    Ok(())
}

/// Output results as CSV
async fn output_csv(
    results: &[(PathBuf, Result<AnalysisResult, anyhow::Error>)],
    output_dir: Option<&PathBuf>,
) -> Result<()> {
    let mut csv_data = String::new();
    csv_data.push_str("file,status,tune_quality,duration_ms,sample_rate,samples,analysis_time_ms,issues,filter_recommendations,pid_recommendations\n");

    for (file_path, result) in results {
        match result {
            Ok(analysis) => {
                csv_data.push_str(&format!(
                    "{},{},{:.1},{},{:.0},{},{},{},{},{}\n",
                    file_path.display(),
                    "success",
                    analysis.report.tune_quality_score,
                    analysis.session.metadata.duration_ms,
                    analysis.session.telemetry.sample_rate,
                    analysis.session.telemetry.gyro.len(),
                    analysis.analysis_time.as_millis(),
                    analysis.report.detected_issues.len(),
                    analysis.report.filter_recommendations.len(),
                    analysis.report.pid_recommendations.len()
                ));
            }
            Err(e) => {
                csv_data.push_str(&format!(
                    "{},error,,,,,,,,,\"{}\"\n",
                    file_path.display(),
                    e.to_string().replace('"', "\"\"")
                ));
            }
        }
    }

    if let Some(output_dir) = output_dir {
        std::fs::create_dir_all(output_dir)?;
        let output_file = output_dir.join("analysis_results.csv");
        std::fs::write(&output_file, csv_data)?;
        println!("CSV results written to {}", output_file.display());
    } else {
        println!("{}", csv_data);
    }

    Ok(())
}

/// Generate comparison between multiple flights
fn generate_comparison(results: &[AnalysisResult]) -> Result<FlightComparison> {
    let mut flights = Vec::new();
    let mut total_quality = 0.0;
    let mut total_issues = 0;

    for result in results {
        let flight = ComparisonFlight {
            name: result
                .file_path
                .file_name()
                .unwrap_or_default()
                .to_string_lossy()
                .to_string(),
            tune_quality: result.report.tune_quality_score,
            issues_count: result.report.detected_issues.len(),
            recommendations_count: result.report.filter_recommendations.len()
                + result.report.pid_recommendations.len()
                + result.report.advanced_recommendations.len(),
            duration_ms: result.session.metadata.duration_ms,
        };

        total_quality += flight.tune_quality;
        total_issues += flight.issues_count;
        flights.push(flight);
    }

    let best_quality = flights.iter().map(|f| f.tune_quality).fold(0.0, f32::max);
    let worst_quality = flights.iter().map(|f| f.tune_quality).fold(100.0, f32::min);
    let avg_quality = total_quality / flights.len() as f32;

    let summary = ComparisonSummary {
        best_tune_quality: best_quality,
        worst_tune_quality: worst_quality,
        avg_tune_quality: avg_quality,
        total_issues,
        common_issues: Vec::new(), // Would implement common issue detection
    };

    Ok(FlightComparison { flights, summary })
}

/// Print comparison in pretty format
fn print_comparison_pretty(comparison: &FlightComparison) {
    display::section("Flight Comparison");
    println!();
    println!(
        "  Best:    {}",
        display::quality_gauge(comparison.summary.best_tune_quality)
    );
    println!(
        "  Worst:   {}",
        display::quality_gauge(comparison.summary.worst_tune_quality)
    );
    println!(
        "  Average: {}",
        display::quality_gauge(comparison.summary.avg_tune_quality)
    );
    println!(
        "  Total issues across flights: {}",
        comparison.summary.total_issues
    );
    println!();

    let rows: Vec<display::ComparisonRow> = comparison
        .flights
        .iter()
        .map(|f| display::ComparisonRow {
            name: &f.name,
            quality: f.tune_quality,
            issues: f.issues_count,
            recommendations: f.recommendations_count,
            duration_ms: f.duration_ms,
        })
        .collect();
    display::comparison_table(&rows);
}

/// Print comparison as JSON
fn print_comparison_json(comparison: &FlightComparison) -> Result<()> {
    // Wrap with the same envelope as `analyze --output-format json` so
    // downstream consumers can rely on a consistent top-level shape.
    let envelope = serde_json::json!({
        "version": env!("CARGO_PKG_VERSION"),
        "timestamp": chrono::Utc::now(),
        "flights": &comparison.flights,
        "summary": &comparison.summary,
    });
    let json = serde_json::to_string_pretty(&envelope)
        .context("Failed to serialize comparison to JSON")?;
    println!("{}", json);
    Ok(())
}

/// Print comparison as CSV
fn print_comparison_csv(comparison: &FlightComparison) -> Result<()> {
    println!("name,tune_quality,issues_count,recommendations_count,duration_ms");
    for flight in &comparison.flights {
        println!(
            "{},{:.1},{},{},{}",
            flight.name,
            flight.tune_quality,
            flight.issues_count,
            flight.recommendations_count,
            flight.duration_ms
        );
    }
    Ok(())
}

// Implement Serialize for comparison types (needed for JSON output)
impl serde::Serialize for FlightComparison {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        use serde::ser::SerializeStruct;
        let mut state = serializer.serialize_struct("FlightComparison", 2)?;
        state.serialize_field("flights", &self.flights)?;
        state.serialize_field("summary", &self.summary)?;
        state.end()
    }
}

impl serde::Serialize for ComparisonFlight {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        use serde::ser::SerializeStruct;
        let mut state = serializer.serialize_struct("ComparisonFlight", 5)?;
        state.serialize_field("name", &self.name)?;
        state.serialize_field("tune_quality", &self.tune_quality)?;
        state.serialize_field("issues_count", &self.issues_count)?;
        state.serialize_field("recommendations_count", &self.recommendations_count)?;
        state.serialize_field("duration_ms", &self.duration_ms)?;
        state.end()
    }
}

impl serde::Serialize for ComparisonSummary {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        use serde::ser::SerializeStruct;
        let mut state = serializer.serialize_struct("ComparisonSummary", 5)?;
        state.serialize_field("best_tune_quality", &self.best_tune_quality)?;
        state.serialize_field("worst_tune_quality", &self.worst_tune_quality)?;
        state.serialize_field("avg_tune_quality", &self.avg_tune_quality)?;
        state.serialize_field("total_issues", &self.total_issues)?;
        state.serialize_field("common_issues", &self.common_issues)?;
        state.end()
    }
}

// Helper functions for new CLI features

/// Format telemetry frame for pretty display
fn format_telemetry_frame(frame: &drone_tuner_core::realtime::TelemetryFrame) -> String {
    let mut output = String::new();

    if let Some(gyro) = &frame.gyro {
        output.push_str(&format!(
            "Gyro: [{:6.1}, {:6.1}, {:6.1}] ",
            gyro.x, gyro.y, gyro.z
        ));
    }

    if let Some(pid_error) = &frame.pid_error {
        output.push_str(&format!(
            "PID Err: [{:5.2}, {:5.2}, {:5.2}] ",
            pid_error.roll, pid_error.pitch, pid_error.yaw
        ));
    }

    if let Some(motors) = &frame.motors {
        output.push_str(&format!(
            "Motors: [{:4.0}, {:4.0}, {:4.0}, {:4.0}] ",
            motors[0], motors[1], motors[2], motors[3]
        ));
    }

    if let Some(battery) = frame.battery_voltage {
        output.push_str(&format!("Batt: {:4.2}V ", battery));
    }

    if let Some(cpu) = frame.cpu_load {
        output.push_str(&format!("CPU: {:3.0}% ", cpu));
    }

    output.trim_end().to_string()
}

/// Format telemetry frame as JSON
fn format_telemetry_json(frame: &drone_tuner_core::realtime::TelemetryFrame) -> serde_json::Value {
    let mut json = serde_json::Map::new();

    json.insert(
        "timestamp".to_string(),
        serde_json::Value::Number(
            serde_json::Number::from_f64(frame.timestamp.elapsed().as_secs_f64()).unwrap(),
        ),
    );

    if let Some(gyro) = &frame.gyro {
        json.insert(
            "gyro".to_string(),
            serde_json::json!({
                "x": gyro.x,
                "y": gyro.y,
                "z": gyro.z
            }),
        );
    }

    if let Some(pid_error) = &frame.pid_error {
        json.insert(
            "pid_error".to_string(),
            serde_json::json!({
                "roll": pid_error.roll,
                "pitch": pid_error.pitch,
                "yaw": pid_error.yaw
            }),
        );
    }

    serde_json::Value::Object(json)
}

/// Generate CSV header for telemetry
fn telemetry_csv_header(fields: &[&str]) -> String {
    let mut header = vec!["timestamp".to_string()];

    for field in fields {
        match *field {
            "gyro" => {
                header.extend([
                    "gyro_x".to_string(),
                    "gyro_y".to_string(),
                    "gyro_z".to_string(),
                ]);
            }
            "pid_error" => {
                header.extend([
                    "pid_roll".to_string(),
                    "pid_pitch".to_string(),
                    "pid_yaw".to_string(),
                ]);
            }
            "motors" => {
                header.extend([
                    "motor1".to_string(),
                    "motor2".to_string(),
                    "motor3".to_string(),
                    "motor4".to_string(),
                ]);
            }
            "battery" => header.push("battery_voltage".to_string()),
            "cpu" => header.push("cpu_load".to_string()),
            _ => {}
        }
    }

    header.join(",")
}

/// Format telemetry frame as CSV row
fn format_telemetry_csv(frame: &drone_tuner_core::realtime::TelemetryFrame) -> String {
    let mut values = vec![frame.timestamp.elapsed().as_secs_f64().to_string()];

    if let Some(gyro) = &frame.gyro {
        values.extend([gyro.x.to_string(), gyro.y.to_string(), gyro.z.to_string()]);
    }

    if let Some(pid_error) = &frame.pid_error {
        values.extend([
            pid_error.roll.to_string(),
            pid_error.pitch.to_string(),
            pid_error.yaw.to_string(),
        ]);
    }

    if let Some(motors) = &frame.motors {
        values.extend(motors.iter().map(|m| m.to_string()).collect::<Vec<_>>());
    }

    if let Some(battery) = frame.battery_voltage {
        values.push(battery.to_string());
    }

    if let Some(cpu) = frame.cpu_load {
        values.push(cpu.to_string());
    }

    values.join(",")
}

/// Export analysis to CSV format
async fn export_to_csv(
    analysis: &AnalysisResult,
    output_path: &PathBuf,
    include_raw: bool,
    _include_fft: bool,
) -> Result<()> {
    let mut content = String::new();

    // Header
    content.push_str("# FPV Drone Tuner Analysis Export\n");
    content.push_str(&format!("# File: {}\n", analysis.file_path.display()));
    content.push_str(&format!(
        "# Analysis Time: {:.2}s\n",
        analysis.analysis_time.as_secs_f32()
    ));
    content.push_str(&format!(
        "# Tune Quality: {:.1}\n",
        analysis.report.tune_quality_score
    ));
    content.push('\n');

    // Raw telemetry data if requested
    if include_raw {
        content.push_str("# Raw Gyro Data\n");
        content.push_str("time,gyro_x,gyro_y,gyro_z\n");

        let sample_rate = analysis.session.telemetry.sample_rate;
        for i in 0..analysis.session.telemetry.gyro.len() {
            let time = i as f32 / sample_rate;
            if let Some(gyro) = analysis.session.telemetry.gyro.get(i) {
                content.push_str(&format!(
                    "{:.6},{:.6},{:.6},{:.6}\n",
                    time, gyro.x, gyro.y, gyro.z
                ));
            }
        }
        content.push('\n');
    }

    // Recommendations
    content.push_str("# PID Recommendations\n");
    content.push_str("axis,term,current_value,recommended_value,priority,reason\n");
    for rec in &analysis.report.pid_recommendations {
        content.push_str(&format!(
            "{:?},{:?},{},{},{:?},\"{}\"\n",
            rec.axis, rec.term, rec.current_value, rec.recommended_value, rec.priority, rec.reason
        ));
    }

    std::fs::write(output_path, content)?;
    Ok(())
}

/// Export analysis to JSON format
async fn export_to_json(
    analysis: &AnalysisResult,
    output_path: &PathBuf,
    include_raw: bool,
    _include_fft: bool,
) -> Result<()> {
    let mut export_data = serde_json::Map::new();

    export_data.insert(
        "file_path".to_string(),
        serde_json::Value::String(analysis.file_path.display().to_string()),
    );
    export_data.insert(
        "analysis_time_s".to_string(),
        serde_json::Value::Number(
            serde_json::Number::from_f64(analysis.analysis_time.as_secs_f64()).unwrap(),
        ),
    );
    export_data.insert(
        "tune_quality".to_string(),
        serde_json::Value::Number(
            serde_json::Number::from_f64(analysis.report.tune_quality_score as f64).unwrap(),
        ),
    );

    // PID recommendations
    let pid_recs: Vec<serde_json::Value> = analysis
        .report
        .pid_recommendations
        .iter()
        .map(|rec| {
            serde_json::json!({
                "axis": format!("{:?}", rec.axis),
                "term": format!("{:?}", rec.term),
                "current_value": rec.current_value,
                "recommended_value": rec.recommended_value,
                "priority": format!("{:?}", rec.priority),
                "reason": rec.reason
            })
        })
        .collect();
    export_data.insert(
        "pid_recommendations".to_string(),
        serde_json::Value::Array(pid_recs),
    );

    // Raw data if requested
    if include_raw {
        let gyro_data: Vec<serde_json::Value> = (0..analysis.session.telemetry.gyro.len())
            .filter_map(|i| {
                let time = i as f32 / analysis.session.telemetry.sample_rate;
                analysis.session.telemetry.gyro.get(i).map(|gyro| {
                    serde_json::json!({ "time": time, "x": gyro.x, "y": gyro.y, "z": gyro.z })
                })
            })
            .collect();
        export_data.insert("gyro_data".to_string(), serde_json::Value::Array(gyro_data));
    }

    let json_str = serde_json::to_string_pretty(&serde_json::Value::Object(export_data))?;
    std::fs::write(output_path, json_str)?;
    Ok(())
}

/// Export analysis to MATLAB format
async fn export_to_matlab(
    analysis: &AnalysisResult,
    output_path: &PathBuf,
    include_raw: bool,
    _include_fft: bool,
) -> Result<()> {
    let mut content = String::new();

    content.push_str("% FPV Drone Tuner Analysis Export\n");
    content.push_str(&format!("% File: {}\n", analysis.file_path.display()));
    content.push_str(&format!(
        "% Analysis Time: {:.2}s\n",
        analysis.analysis_time.as_secs_f32()
    ));
    content.push('\n');

    content.push_str(&format!(
        "tune_quality = {:.1};\n",
        analysis.report.tune_quality_score
    ));
    content.push_str(&format!(
        "sample_rate = {:.1};\n",
        analysis.session.telemetry.sample_rate
    ));
    content.push_str(&format!(
        "duration_ms = {};\n",
        analysis.session.metadata.duration_ms
    ));
    content.push('\n');

    if include_raw {
        content.push_str("% Gyro data\n");
        content.push_str("gyro_data = [\n");
        for i in 0..analysis.session.telemetry.gyro.len() {
            if let Some(gyro) = analysis.session.telemetry.gyro.get(i) {
                content.push_str(&format!("  {:.6}, {:.6}, {:.6};\n", gyro.x, gyro.y, gyro.z));
            }
        }
        content.push_str("];\n\n");

        content.push_str("% Time vector\n");
        content.push_str(&format!(
            "t = (0:{})/sample_rate;\n\n",
            analysis.session.telemetry.gyro.len() - 1
        ));
    }

    std::fs::write(output_path, content)?;
    Ok(())
}

/// Export analysis to Python format
async fn export_to_python(
    analysis: &AnalysisResult,
    output_path: &PathBuf,
    include_raw: bool,
    _include_fft: bool,
) -> Result<()> {
    let mut content = String::new();

    content.push_str("# FPV Drone Tuner Analysis Export\n");
    content.push_str(&format!("# File: {}\n", analysis.file_path.display()));
    content.push_str(&format!(
        "# Analysis Time: {:.2}s\n",
        analysis.analysis_time.as_secs_f32()
    ));
    content.push('\n');
    content.push_str("import numpy as np\n");
    content.push_str("import matplotlib.pyplot as plt\n\n");

    content.push_str(&format!(
        "tune_quality = {:.1}\n",
        analysis.report.tune_quality_score
    ));
    content.push_str(&format!(
        "sample_rate = {:.1}\n",
        analysis.session.telemetry.sample_rate
    ));
    content.push_str(&format!(
        "duration_ms = {}\n",
        analysis.session.metadata.duration_ms
    ));
    content.push('\n');

    if include_raw {
        content.push_str("# Gyro data\n");
        content.push_str("gyro_data = np.array([\n");
        for i in 0..analysis.session.telemetry.gyro.len() {
            if let Some(gyro) = analysis.session.telemetry.gyro.get(i) {
                content.push_str(&format!(
                    "    [{:.6}, {:.6}, {:.6}],\n",
                    gyro.x, gyro.y, gyro.z
                ));
            }
        }
        content.push_str("])\n\n");

        content.push_str("# Time vector\n");
        content.push_str(&format!(
            "t = np.arange({}) / sample_rate\n\n",
            analysis.session.telemetry.gyro.len()
        ));

        content.push_str("# Example plot\n");
        content.push_str("plt.figure(figsize=(12, 4))\n");
        content.push_str("plt.plot(t, gyro_data[:, 0], label='Roll')\n");
        content.push_str("plt.plot(t, gyro_data[:, 1], label='Pitch')\n");
        content.push_str("plt.plot(t, gyro_data[:, 2], label='Yaw')\n");
        content.push_str("plt.xlabel('Time (s)')\n");
        content.push_str("plt.ylabel('Gyro (deg/s)')\n");
        content.push_str("plt.legend()\n");
        content.push_str("plt.title('Gyro Data')\n");
        content.push_str("plt.grid(True)\n");
        content.push_str("plt.show()\n");
    }

    std::fs::write(output_path, content)?;
    Ok(())
}

#[cfg(test)]
mod craft_name_tests {
    use super::*;

    #[test]
    fn sanitises_simple_name() {
        assert_eq!(sanitize_filename_part("Jeno"), Some("Jeno".to_string()));
    }

    #[test]
    fn collapses_spaces_to_underscore() {
        assert_eq!(
            sanitize_filename_part("TBS Source One"),
            Some("TBS_Source_One".to_string())
        );
    }

    #[test]
    fn drops_punctuation_and_emoji() {
        assert_eq!(
            sanitize_filename_part("My/Quad! 🚁"),
            Some("MyQuad".to_string())
        );
    }

    #[test]
    fn returns_none_for_empty_or_pure_junk() {
        assert_eq!(sanitize_filename_part(""), None);
        assert_eq!(sanitize_filename_part("///"), None);
        assert_eq!(sanitize_filename_part("   "), None);
    }

    #[test]
    fn trims_underscores_at_edges() {
        assert_eq!(
            sanitize_filename_part("  my quad  "),
            Some("my_quad".to_string())
        );
    }

    #[test]
    fn collapses_consecutive_separators() {
        assert_eq!(
            sanitize_filename_part("foo   bar"),
            Some("foo_bar".to_string())
        );
    }
}

#[cfg(test)]
mod phase1_backup_tests {
    use super::*;
    use drone_tuner_core::realtime::{
        AdvancedConfigSnapshot, FlightControllerConnection, MockTransport, MspSimulator,
        PidAdvancedSnapshot, RcTuningSnapshot,
    };

    #[test]
    fn read_optional_payload_returns_none_when_field_missing() {
        let json = serde_json::json!({"other": "value"});
        let out = read_optional_payload(&json, "missing").unwrap();
        assert_eq!(out, None);
    }

    #[test]
    fn read_optional_payload_returns_none_when_field_null() {
        let json = serde_json::json!({"missing": null});
        let out = read_optional_payload(&json, "missing").unwrap();
        assert_eq!(out, None);
    }

    #[test]
    fn read_optional_payload_round_trips_byte_array() {
        let json = serde_json::json!({"payload": [1u8, 2, 3, 254, 255]});
        let out = read_optional_payload(&json, "payload").unwrap();
        assert_eq!(out, Some(vec![1u8, 2, 3, 254, 255]));
    }

    #[test]
    fn read_optional_payload_rejects_oversized_byte() {
        let json = serde_json::json!({"payload": [256]});
        let err = read_optional_payload(&json, "payload")
            .expect_err("byte > 255 must be rejected to avoid silent truncation");
        assert!(err.to_string().contains("u8 range"));
    }

    #[test]
    fn read_optional_payload_rejects_non_array() {
        let json = serde_json::json!({"payload": "hello"});
        let err = read_optional_payload(&json, "payload")
            .expect_err("non-array must be rejected — caller can't recover meaningfully");
        assert!(err.to_string().contains("not an array"));
    }

    /// End-to-end Phase-1 round-trip: read all three surfaces from a
    /// simulator, serialize to v2 JSON shape, parse back via the same
    /// readers `restore_pid_from_backup` uses, write to the FC, re-read,
    /// and assert byte-equal preservation across every surface.
    #[tokio::test]
    async fn v2_backup_json_round_trips_phase1_surfaces_via_simulator() {
        let (client, server) = MockTransport::pair();
        let sim = MspSimulator::new(Box::new(server));
        let state = sim.state.clone();
        let _sim_handle = tokio::spawn(sim.run());
        let mut fc = FlightControllerConnection::from_transport(Box::new(client))
            .await
            .expect("simulator handshake");

        // Capture the FC's pre-restore state across the three surfaces.
        let extras = read_phase1_extras(&mut fc).await;
        assert!(extras.pid_advanced.is_some());
        assert!(extras.rc_tuning.is_some());
        assert!(extras.advanced_config.is_some());
        let original_pid_adv = state.lock().unwrap().pid_advanced.clone();
        let original_rc_tune = state.lock().unwrap().rc_tuning.clone();
        let original_adv_cfg = state.lock().unwrap().advanced_config.clone();

        // Serialize as the runtime would for a v2 backup file.
        let json_blob = serde_json::json!({
            "schema": BACKUP_SCHEMA_V2,
            "pid_advanced_payload": extras.pid_advanced.as_ref().map(|s| s.as_payload().to_vec()),
            "rc_tuning_payload": extras.rc_tuning.as_ref().map(|s| s.as_payload().to_vec()),
            "advanced_config_payload": extras.advanced_config.as_ref().map(|s| s.as_payload().to_vec()),
        });

        // Mutate the FC's state to simulate a Configurator session
        // changing things between backup and restore. After the restore
        // we expect the bytes to match the BACKUP, not these mutations.
        {
            let mut s = state.lock().unwrap();
            s.pid_advanced = vec![0xAA; 49];
            s.rc_tuning = vec![0xBB; 30];
            s.advanced_config = vec![0xCC; 21];
        }

        // Parse the v2 JSON back through the same helpers
        // restore_pid_from_backup uses, then write to the FC.
        let mut restored = Phase1Extras::default();
        if let Some(bytes) = read_optional_payload(&json_blob, "pid_advanced_payload").unwrap() {
            restored.pid_advanced = Some(PidAdvancedSnapshot::from_payload(bytes).unwrap());
        }
        if let Some(bytes) = read_optional_payload(&json_blob, "rc_tuning_payload").unwrap() {
            restored.rc_tuning = Some(RcTuningSnapshot::from_payload(bytes).unwrap());
        }
        if let Some(bytes) = read_optional_payload(&json_blob, "advanced_config_payload").unwrap() {
            restored.advanced_config = Some(AdvancedConfigSnapshot::from_payload(bytes).unwrap());
        }
        let written = write_phase1_extras(&mut fc, &restored).await;
        assert_eq!(written, 3, "all three surfaces should restore");

        // FC's state must now match the pre-mutation original bytes.
        let after = state.lock().unwrap();
        assert_eq!(
            after.pid_advanced, original_pid_adv,
            "PidAdvanced must round-trip through v2 backup JSON"
        );
        assert_eq!(after.rc_tuning, original_rc_tune);
        assert_eq!(after.advanced_config, original_adv_cfg);
    }

    /// v1 backups must still be accepted — they simply skip the Phase-1
    /// extras, matching pre-Phase-1 behaviour. Pinning this prevents a
    /// future schema-bump that drops v1 silently.
    #[test]
    fn v1_backup_schema_still_recognised() {
        let zeroes: Vec<u8> = vec![0; 30];
        let json = serde_json::json!({
            "schema": BACKUP_SCHEMA_V1,
            "pid_payload": zeroes,
            "roll": [42, 85, 35],
            "pitch": [46, 90, 38],
            "yaw": [45, 90, 0],
        });
        let schema = json.get("schema").and_then(|v| v.as_str()).unwrap();
        assert!(schema == BACKUP_SCHEMA_V1 || schema == BACKUP_SCHEMA_V2);
    }
}

#[cfg(test)]
mod port_discovery_tests {
    use super::*;
    use serialport::{SerialPortInfo, SerialPortType, UsbPortInfo};

    fn usb(name: &str, vid: u16, pid: u16) -> SerialPortInfo {
        SerialPortInfo {
            port_name: name.to_string(),
            port_type: SerialPortType::UsbPort(UsbPortInfo {
                vid,
                pid,
                serial_number: None,
                manufacturer: None,
                product: None,
            }),
        }
    }

    fn non_usb(name: &str) -> SerialPortInfo {
        SerialPortInfo {
            port_name: name.to_string(),
            port_type: SerialPortType::Unknown,
        }
    }

    #[test]
    fn picks_lone_stm32_vcp() {
        let pick = pick_fc_port_from(vec![usb("/dev/ttyACM0", 0x0483, 0x5740)]).unwrap();
        assert_eq!(pick.port_name, "/dev/ttyACM0");
    }

    #[test]
    fn prefers_stm32_when_usb_to_uart_also_present() {
        // STM32 VCP wins even when a CH340 (1a86) bridge is also plugged in.
        let pick = pick_fc_port_from(vec![
            usb("/dev/ttyUSB0", 0x1a86, 0x7523),
            usb("/dev/ttyACM0", 0x0483, 0x5740),
        ])
        .unwrap();
        assert_eq!(pick.port_name, "/dev/ttyACM0");
    }

    #[test]
    fn falls_back_to_any_usb_when_no_stm32() {
        let pick = pick_fc_port_from(vec![usb("/dev/ttyUSB0", 0x1a86, 0x7523)]).unwrap();
        assert_eq!(pick.port_name, "/dev/ttyUSB0");
    }

    #[test]
    fn errors_when_two_stm32_vcps_are_present() {
        let err = pick_fc_port_from(vec![
            usb("/dev/ttyACM0", 0x0483, 0x5740),
            usb("/dev/ttyACM1", 0x0483, 0x5740),
        ])
        .unwrap_err();
        let msg = format!("{err}");
        assert!(msg.contains("/dev/ttyACM0"));
        assert!(msg.contains("/dev/ttyACM1"));
        assert!(msg.contains("--connection"));
    }

    #[test]
    fn errors_when_no_usb_serial() {
        let err = pick_fc_port_from(vec![non_usb("/dev/ttyS0")]).unwrap_err();
        let msg = format!("{err}");
        assert!(msg.to_lowercase().contains("no usb serial"));
    }

    #[test]
    fn ignores_non_usb_ports_when_picking() {
        let pick = pick_fc_port_from(vec![
            non_usb("/dev/ttyS0"),
            usb("/dev/ttyACM0", 0x0483, 0x5740),
        ])
        .unwrap();
        assert_eq!(pick.port_name, "/dev/ttyACM0");
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use drone_tuner_core::domain::{Axis, PidRecommendation, PidTerm, Priority};
    use drone_tuner_core::realtime::PidSnapshot;

    fn rec(axis: Axis, term: PidTerm, value: f32, priority: Priority) -> PidRecommendation {
        PidRecommendation {
            axis,
            term,
            current_value: 0.0,
            recommended_value: value,
            reason: "test".to_string(),
            priority,
        }
    }

    fn fresh_snapshot() -> PidSnapshot {
        // Build a 30-byte zeroed payload — PidSnapshot has no public ctor.
        PidSnapshot::from_payload(vec![0u8; 30]).unwrap()
    }

    fn args(auto: bool, all: bool) -> TuneArgs {
        TuneArgs {
            input: Some(PathBuf::from("dummy")),
            connection: None,
            pull_bbl: false,
            keep_bbl: None,
            erase_after_pull: false,
            pull_chunk_size: None,
            dry_run: false,
            backup: None,
            auto_apply_safe: auto,
            apply_all: all,
            save_eeprom: false,
            skip_filters: false,
            session_strategy: None,
            restore: None,
            baseline: None,
        }
    }

    #[test]
    fn no_flags_applies_nothing() {
        let mut snap = fresh_snapshot();
        let recs = vec![
            rec(Axis::Roll, PidTerm::P, 50.0, Priority::Low),
            rec(Axis::Pitch, PidTerm::I, 100.0, Priority::High),
        ];
        let count = apply_pid_recs_to_snapshot(&mut snap, &recs, &args(false, false));
        assert_eq!(count, 0);
        assert_eq!(snap.roll(), (0, 0, 0));
        assert_eq!(snap.pitch(), (0, 0, 0));
    }

    #[test]
    fn auto_apply_safe_filters_to_low_medium() {
        let mut snap = fresh_snapshot();
        let recs = vec![
            rec(Axis::Roll, PidTerm::P, 40.0, Priority::Low),
            rec(Axis::Pitch, PidTerm::I, 80.0, Priority::Medium),
            rec(Axis::Yaw, PidTerm::D, 120.0, Priority::High),
            rec(Axis::Roll, PidTerm::I, 200.0, Priority::Critical),
        ];
        let count = apply_pid_recs_to_snapshot(&mut snap, &recs, &args(true, false));
        assert_eq!(count, 2);
        assert_eq!(snap.roll().0, 40);
        assert_eq!(snap.roll().1, 0); // Critical I rec was skipped
        assert_eq!(snap.pitch().1, 80);
        assert_eq!(snap.yaw(), (0, 0, 0)); // High skipped
    }

    #[test]
    fn apply_all_takes_every_priority() {
        let mut snap = fresh_snapshot();
        let recs = vec![
            rec(Axis::Roll, PidTerm::P, 40.0, Priority::Low),
            rec(Axis::Pitch, PidTerm::I, 80.0, Priority::High),
            rec(Axis::Yaw, PidTerm::D, 120.0, Priority::Critical),
        ];
        let count = apply_pid_recs_to_snapshot(&mut snap, &recs, &args(false, true));
        assert_eq!(count, 3);
        assert_eq!(snap.roll().0, 40);
        assert_eq!(snap.pitch().1, 80);
        assert_eq!(snap.yaw().2, 120);
    }

    #[test]
    fn out_of_range_values_clamp() {
        let mut snap = fresh_snapshot();
        let recs = vec![
            rec(Axis::Roll, PidTerm::P, -50.0, Priority::Low),
            rec(Axis::Pitch, PidTerm::I, 999.5, Priority::Low),
        ];
        let count = apply_pid_recs_to_snapshot(&mut snap, &recs, &args(true, false));
        assert_eq!(count, 2);
        assert_eq!(snap.roll().0, 0); // clamped from -50
        assert_eq!(snap.pitch().1, 255); // clamped from 999.5
    }

    #[test]
    fn f_term_recommendations_are_skipped() {
        let mut snap = fresh_snapshot();
        let recs = vec![rec(Axis::Roll, PidTerm::F, 100.0, Priority::Low)];
        let count = apply_pid_recs_to_snapshot(&mut snap, &recs, &args(false, true));
        assert_eq!(count, 0, "F-term doesn't fit in MSP_PID's first 9 bytes");
    }

    #[test]
    fn writes_preserve_other_axes() {
        let mut snap = fresh_snapshot();
        snap.set_yaw(11, 22, 33);
        let recs = vec![rec(Axis::Roll, PidTerm::P, 50.0, Priority::Low)];
        apply_pid_recs_to_snapshot(&mut snap, &recs, &args(true, false));
        assert_eq!(snap.yaw(), (11, 22, 33));
    }
}
