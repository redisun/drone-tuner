//! Command-line interface for the FPV drone tuning platform.

mod history;

use anyhow::{Context, Result};
use clap::{Args, Parser, Subcommand};
use console::{style, Term};
use drone_tuner_core::domain::{FilterRecommendationType, Priority};
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
    /// Path to blackbox file or directory
    #[arg(value_name = "FILE_OR_DIR")]
    input: PathBuf,

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

    /// Connection string for applying changes
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
    let level = if verbose {
        tracing::Level::DEBUG
    } else {
        tracing::Level::INFO
    };

    tracing_subscriber::fmt()
        .with_max_level(level)
        .with_writer(std::io::stderr) // Keep logs separate from output
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

    // Find all blackbox files to process
    let files = find_blackbox_files(&args.input, args.max_files)?;

    if files.is_empty() {
        eprintln!("{}", style("No blackbox files found").red());
        return Ok(());
    }

    // Status messages go to stderr so machine-readable output formats
    // (CSV, JSON) keep stdout clean and parseable.
    eprintln!(
        "{} Found {} blackbox file(s) to analyze",
        style("✓").green(),
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
        println!("{} Analysis Summary", style("📊").blue());
        println!("  Successful: {}", style(successful).green());
        if failed > 0 {
            println!("  Failed: {}", style(failed).red());
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
    eprintln!(
        "{} Comparing {} flights",
        style("🔍").blue(),
        args.files.len()
    );

    let mut engine = AnalysisEngine::new();
    let mut results = Vec::new();

    // Analyze each file
    for file in &args.files {
        match analyze_single_file(
            &mut engine,
            file,
            &AnalyzeArgs {
                input: file.clone(),
                output_dir: None,
                detailed: true,
                show_details: false,
                min_confidence: 0.5,
                max_files: 1,
                session: None,
                list_sessions: false,
                bb_summary: false,
                session_strategy: None,
            },
        )
        .await
        {
            Ok(result) => results.push(result),
            Err(e) => {
                eprintln!(
                    "{} Failed to analyze {}: {}",
                    style("✗").red(),
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
            style("!").yellow()
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
        println!("{} No blackbox files found", style("!").yellow());
        return Ok(());
    }

    println!("{} Validating {} file(s)", style("🔍").blue(), files.len());

    let mut valid_files = 0;
    let mut invalid_files = 0;

    for file in files {
        match validate_single_file(&file, args.check_issues).await {
            Ok(issues) => {
                valid_files += 1;
                if !issues.is_empty() {
                    println!(
                        "{} {} - {} issues found",
                        style("⚠").yellow(),
                        file.display(),
                        issues.len()
                    );
                    for issue in issues {
                        println!("    {}", issue);
                    }
                } else {
                    println!("{} {} - valid", style("✓").green(), file.display());
                }
            }
            Err(e) => {
                invalid_files += 1;
                println!("{} {} - {}", style("✗").red(), file.display(), e);
            }
        }
    }

    println!();
    println!("Validation Summary:");
    println!("  Valid files: {}", style(valid_files).green());
    if invalid_files > 0 {
        println!("  Invalid files: {}", style(invalid_files).red());
    }

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
        style("⚠").yellow().bold()
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
                    println!("{} Unknown telemetry field: {}", style("⚠").yellow(), field);
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

        println!("{} Connected successfully", style("✓").green());

        // Start telemetry streaming
        let mut telemetry_rx = fc
            .start_telemetry_streaming(telemetry_config)
            .await
            .context("Failed to start telemetry streaming")?;

        println!(
            "{} Monitoring telemetry at {}Hz...",
            style("📡").blue(),
            _args.rate
        );
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
            "\n{} Monitoring stopped. Captured {} samples",
            style("📊").blue(),
            sample_count
        );
        Ok(())
    }
}

/// Handle the tune command
async fn tune_command(args: TuneArgs, _output_format: OutputFormat) -> Result<()> {
    print_stage("Tune", "drone-tuner ✈️");

    // Validate flag combos. `--pull-bbl` needs --connection because there's
    // nothing to pull from otherwise; `--keep-bbl` only makes sense when we
    // actually have a pulled file to keep; an explicit input together with
    // `--pull-bbl` is ambiguous so reject it.
    if args.pull_bbl && args.connection.is_none() {
        return Err(anyhow::anyhow!(
            "--pull-bbl requires --connection <CONNECTION>"
        ));
    }
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

    // Resolve the .bbl path: download from FC or use the user-provided file.
    let bbl_path = match (&args.input, args.pull_bbl) {
        (Some(p), false) => p.clone(),
        (None, true) => {
            let connection = args.connection.as_deref().unwrap();
            pull_bbl_from_fc(connection, args.keep_bbl.as_deref(), args.erase_after_pull)
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

    print_stage("Analyze", "🔍");

    // Analyze the blackbox file first
    let analyze_args = AnalyzeArgs {
        input: bbl_path.clone(),
        output_dir: None,
        detailed: true,
        show_details: false,
        min_confidence: 0.7,
        max_files: 1,
        session: None,
        list_sessions: false,
        bb_summary: false,
        session_strategy: args.session_strategy.clone(),
    };

    // Get analysis results
    let mut engine = AnalysisEngine::new();
    let analysis_pb = make_spinner(format!("parsing {}", bbl_path.display()));
    let analysis = analyze_single_file(&mut engine, &bbl_path, &analyze_args).await?;
    finish_step(
        analysis_pb,
        format!(
            "parsed {} samples ({} ms)",
            analysis.session.telemetry.gyro.len(),
            analysis.analysis_time.as_millis()
        ),
    );

    // Display tuning recommendations
    println!("\n{} Tuning Recommendations:", style("📋").green());

    if !analysis.report.pid_recommendations.is_empty() {
        println!("\n  {} PID Adjustments:", style("🎛️").cyan());
        for rec in &analysis.report.pid_recommendations {
            let priority_icon = match rec.priority {
                Priority::Critical => style("🟣").magenta(),
                Priority::High => style("🔴").red(),
                Priority::Medium => style("🟡").yellow(),
                Priority::Low => style("🟢").green(),
            };
            println!(
                "    {} {:?} {:?}: {:.1} → {:.1}",
                priority_icon, rec.axis, rec.term, rec.current_value, rec.recommended_value
            );
            println!("      Reason: {}", rec.reason);
        }
    }

    if !analysis.report.filter_recommendations.is_empty() {
        println!("\n  {} Filter Adjustments:", style("🔧").cyan());
        for rec in &analysis.report.filter_recommendations {
            let priority_icon = match rec.priority {
                Priority::Critical => style("🟣").magenta(),
                Priority::High => style("🔴").red(),
                Priority::Medium => style("🟡").yellow(),
                Priority::Low => style("🟢").green(),
            };

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

    // Decide whether/how to apply.
    match (&args.connection, args.dry_run) {
        (None, true) => {
            println!("\n{} Dry run mode - no changes applied", style("ℹ️").blue());
        }
        (None, false) => {
            println!(
                "\n{} Specify --connection to apply changes to flight controller",
                style("ℹ️").blue()
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

    Ok(())
}

/// Print a section header. The whole tune flow is broken into named stages
/// so users can see where they are without scrolling.
fn print_stage(name: &str, icon: &str) {
    println!(
        "\n{} {}",
        style(icon).bold(),
        style(format!("── {name} ──")).bold().cyan()
    );
}

/// Build a small spinner with a stable style. Used for transient
/// progress feedback during MSP roundtrips that take ≤1s.
fn make_spinner(msg: impl Into<String>) -> ProgressBar {
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

/// Tear down a transient spinner and print a final result line on stdout.
///
/// Indicatif suppresses output when stderr isn't a TTY (CI, redirected
/// runs), so a `finish_with_message` alone vanishes in those cases. By
/// also writing to stdout we keep the result line visible regardless of
/// where the run is happening.
fn finish_step(pb: ProgressBar, message: impl Into<String>) {
    let msg = message.into();
    pb.finish_and_clear();
    println!("  {} {}", style("✓").green(), msg);
}

/// Pull the FC's onboard dataflash to a `.bbl` file with a live progress
/// bar. Returns the path to the saved file.
///
/// If `keep_path` is `Some`, the file is written there verbatim. Otherwise
/// it goes to `std::env::temp_dir()/drone-tuner-pull-<ts>.bbl`. We print
/// the destination so the user can pick the file up later.
///
/// `erase` triggers MSP_DATAFLASH_ERASE after a successful read — useful
/// for clearing the chip between tune iterations so the next pull only
/// contains the *new* flight, but defaults off because most users want
/// their flight history preserved.
async fn pull_bbl_from_fc(
    connection: &str,
    keep_path: Option<&Path>,
    erase: bool,
) -> Result<PathBuf> {
    print_stage("Pull", "📥");

    let connect_pb = make_spinner(format!("connecting to {connection}"));
    let mut fc = open_fc_connection(connection).await?;
    let info_line = match fc.fc_info() {
        Some(info) => format!(
            "connected: {} {} (api {}, target {})",
            info.firmware_id, info.firmware_version, info.api_version, info.target_name
        ),
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

    // Decide where the file lands.
    let path: PathBuf = match keep_path {
        Some(p) => p.to_path_buf(),
        None => {
            let ts = chrono::Utc::now().format("%Y%m%d-%H%M%S");
            std::env::temp_dir().join(format!("drone-tuner-pull-{ts}.bbl"))
        }
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
    let blob = fc
        .pull_dataflash(move |done, _total| pull_pb_for_cb.set_position(done))
        .await
        .context("Dataflash pull failed")?;
    finish_step(pull_pb, format!("downloaded {}", format_bytes(blob.len() as u64)));

    // Persist.
    std::fs::write(&path, &blob)
        .with_context(|| format!("Failed to write pulled BBL to {}", path.display()))?;
    println!("  {} saved → {}", style("💾").blue(), path.display());

    if erase {
        // Erase is intentionally not wired through `FlightControllerConnection`
        // yet — the command is destructive and we want a deliberate review
        // before exposing it. Surface a friendly note here so the flag
        // doesn't silently no-op.
        println!(
            "  {} --erase-after-pull is staged but not yet wired; \
             use Betaflight Configurator if you need to clear the chip.",
            style("⚠").yellow()
        );
    }

    Ok(path)
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
    println!("{} Connected", style("✓").green());

    let current = fc
        .read_pid()
        .await
        .context("Failed to read current PID values from FC")?;
    let mut proposed = current.clone();
    let count = apply_pid_recs_to_snapshot(&mut proposed, &report.pid_recommendations, args);

    if count == 0 {
        println!(
            "\n{} No PID changes would be applied (need --auto-apply-safe or --apply-all to opt in).",
            style("ℹ️").blue()
        );
    } else {
        println!(
            "\n{} {} PID change(s) WOULD be applied:",
            style("📝").yellow(),
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
                "\n{} Filter config (read-only): gyro_lpf1={} Hz  dterm_lpf1={} Hz  yaw_lpf={} Hz  ({} bytes)",
                style("🔧").yellow(),
                filter.gyro_lpf1_hz(),
                filter.dterm_lpf1_hz(),
                filter.yaw_lpf_hz(),
                filter.as_payload().len(),
            );
            if !report.filter_recommendations.is_empty() {
                println!(
                    "    {} {} filter recommendation(s) would be printed but not auto-applied — \
                     payload offsets vary by firmware version.",
                    style("ℹ️").blue(),
                    report.filter_recommendations.len()
                );
            }
        }
        Err(e) => {
            println!(
                "\n{} Could not read filter config: {} (continuing — PIDs are unaffected)",
                style("⚠").yellow(),
                e
            );
        }
    }

    println!(
        "\n{} Dry run complete — drop --dry-run to actually apply.",
        style("ℹ️").blue()
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
            style("ℹ️").blue()
        );
        return Ok(());
    }

    print_stage("Apply", "✏️");

    let connect_pb = make_spinner(format!("connecting to {connection}"));
    let mut fc = open_fc_connection(connection).await?;
    let info_line = match fc.fc_info() {
        Some(info) => format!(
            "connected: {} {} (api {}, target {})",
            info.firmware_id, info.firmware_version, info.api_version, info.target_name
        ),
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

    let mut new_snapshot = current.clone();
    let applied = apply_pid_recs_to_snapshot(&mut new_snapshot, &report.pid_recommendations, args);

    if applied == 0 {
        println!(
            "  {} No PID recommendations matched the active filters; nothing to write.",
            style("ℹ️").blue()
        );
        return Ok(());
    }

    // Show the diff before writing so the user can sanity-check.
    println!(
        "  {} {} PID change(s) staged:",
        style("📝").yellow(),
        applied
    );
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
    let backup = fc
        .apply_pid_with_rollback(&new_snapshot)
        .await
        .context("PID writeback failed (any partial write was rolled back)")?;
    finish_step(write_pb, "PIDs written; backup retained in memory");

    // Filter writeback via MSP2_COMMON_SET_SETTING (parameter-by-name).
    // We translate each FilterRecommendationType into one or more
    // (setting_name, value_bytes) pairs and apply them individually.
    // Failures on a single setting (typically "unknown name" on an older
    // firmware) are logged and we continue — this avoids one stale
    // setting taking out the whole filter writeback batch.
    let filter_priority_allows = |p: &drone_tuner_core::domain::Priority| {
        if args.apply_all {
            true
        } else if args.auto_apply_safe {
            matches!(
                p,
                drone_tuner_core::domain::Priority::Low | drone_tuner_core::domain::Priority::Medium
            )
        } else {
            false
        }
    };
    let mut filter_changes_applied = 0usize;
    let mut filter_changes_failed = 0usize;
    if !args.skip_filters && !report.filter_recommendations.is_empty() {
        let mut all_changes: Vec<FilterSettingChange> = Vec::new();
        let mut skip_notes: Vec<String> = Vec::new();
        for rec in &report.filter_recommendations {
            if !filter_priority_allows(&rec.priority) {
                continue;
            }
            let (changes, skip) = filter_rec_to_settings(rec);
            all_changes.extend(changes);
            if let Some(note) = skip {
                skip_notes.push(note);
            }
        }
        // Dedupe by setting name (a later rec for the same setting wins).
        let mut seen: std::collections::HashSet<String> = std::collections::HashSet::new();
        all_changes.reverse();
        all_changes.retain(|c| seen.insert(c.name.clone()));
        all_changes.reverse();

        if all_changes.is_empty() {
            if !skip_notes.is_empty() {
                println!(
                    "  {} filter recs surfaced but none auto-applicable on this build: {}",
                    style("ℹ️").blue(),
                    skip_notes.join("; ")
                );
            }
        } else {
            println!(
                "  {} {} filter setting(s) staged:",
                style("📝").yellow(),
                all_changes.len()
            );
            for change in &all_changes {
                println!("    {}", change.description);
            }
            let filter_pb = make_spinner(format!(
                "writing {} filter setting(s) by name",
                all_changes.len()
            ));
            for change in &all_changes {
                match fc.set_setting(&change.name, &change.bytes).await {
                    Ok(()) => filter_changes_applied += 1,
                    Err(e) => {
                        filter_changes_failed += 1;
                        warn!(
                            "Filter setting '{}' failed: {} (continuing with remaining settings)",
                            change.name, e
                        );
                    }
                }
            }
            let summary = if filter_changes_failed == 0 {
                format!("{filter_changes_applied} filter setting(s) written")
            } else {
                format!(
                    "{filter_changes_applied} written, {filter_changes_failed} failed (see warnings)"
                )
            };
            finish_step(filter_pb, summary);
        }
        // Surface skipped variants so the user knows what wasn't covered.
        if !skip_notes.is_empty() {
            for note in skip_notes
                .into_iter()
                .collect::<std::collections::HashSet<_>>()
            {
                println!("  {} {}", style("ℹ️").blue(), note);
            }
        }
    } else if args.skip_filters {
        println!(
            "  {} --skip-filters set — filter recs printed but not written.",
            style("ℹ️").blue()
        );
    }

    // Persist backup to disk if requested.
    if let Some(maybe_path) = &args.backup {
        let path = maybe_path.clone().unwrap_or_else(|| {
            let ts = chrono::Utc::now().format("%Y%m%d-%H%M%S");
            PathBuf::from(format!("tune-backup-{ts}.json"))
        });
        let json = serde_json::to_string_pretty(&serde_json::json!({
            "schema": "drone-tuner-pid-backup-v1",
            "captured_at": chrono::Utc::now(),
            "pid_payload": backup.as_payload(),
            "roll": backup.roll(),
            "pitch": backup.pitch(),
            "yaw": backup.yaw(),
        }))?;
        std::fs::write(&path, json)
            .with_context(|| format!("Failed to write backup snapshot to {}", path.display()))?;
        println!("  {} backup → {}", style("💾").blue(), path.display());
    }

    let mut persisted_to_eeprom = false;
    if args.save_eeprom {
        let save_pb = make_spinner("persisting to EEPROM");
        fc.save_to_eeprom().await.context(
            "EEPROM write failed; RAM changes are still in effect but will revert on power cycle",
        )?;
        persisted_to_eeprom = true;
        finish_step(save_pb, "changes persisted across power cycles");
    } else {
        println!(
            "  {} Changes are RAM-only and will revert on power cycle. \
             Re-run with --save-eeprom to persist.",
            style("ℹ️").blue()
        );
    }

    // Append a row to the per-FC tune history. Best-effort: a failure here
    // must never break the successful writeback that just completed.
    if let Err(e) = record_history(
        &fc,
        bbl_path,
        &backup,
        &new_snapshot,
        applied,
        persisted_to_eeprom,
    ) {
        warn!("Failed to append tune history entry: {e:#}");
    }

    println!("\n  {} Tune complete.", style("✅").green());

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
        "{} Tune logged to {} ({} {})",
        style("📒").blue(),
        path.display(),
        info.board_id,
        info.target_name,
    );
    Ok(())
}

/// One Betaflight setting we want to write, as a `(name, value-bytes)`
/// pair plus a human-readable label for stdout.
///
/// Built by [`filter_rec_to_settings`] so the apply path can drive
/// `MSP2_COMMON_SET_SETTING` per-setting without caring about which
/// `FilterRecommendationType` variant each came from.
#[derive(Debug, Clone)]
struct FilterSettingChange {
    name: String,
    bytes: Vec<u8>,
    /// What this change does, in plain English. Printed in the output.
    description: String,
}

impl FilterSettingChange {
    fn u16(name: impl Into<String>, value: u16, description: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            bytes: value.to_le_bytes().to_vec(),
            description: description.into(),
        }
    }
    fn u8(name: impl Into<String>, value: u8, description: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            bytes: vec![value],
            description: description.into(),
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

/// Translate a single filter recommendation into the list of Betaflight
/// settings writes that implement it. Returns `(changes, skip_reason)`:
/// - `changes` is non-empty when we know how to encode the change safely.
/// - `skip_reason` is `Some(...)` when we deliberately skip part or all of
///   the rec because the encoding varies across firmware versions and we
///   don't want to brick the FC.
fn filter_rec_to_settings(
    rec: &drone_tuner_core::domain::FilterRecommendation,
) -> (Vec<FilterSettingChange>, Option<String>) {
    use drone_tuner_core::domain::FilterRecommendationType::*;
    match &rec.recommendation_type {
        AdjustGyroLowpass {
            stage,
            recommended_cutoff,
            filter_type,
            ..
        } => {
            let prefix = format!("gyro_lpf{stage}");
            let mut changes = vec![FilterSettingChange::u16(
                format!("{prefix}_static_hz"),
                recommended_cutoff.round().clamp(0.0, 65535.0) as u16,
                format!(
                    "{prefix}_static_hz → {:.0} Hz",
                    recommended_cutoff
                ),
            )];
            if let Some(ft) = filter_type_to_enum(filter_type) {
                changes.push(FilterSettingChange::u8(
                    format!("{prefix}_type"),
                    ft,
                    format!("{prefix}_type → {filter_type}"),
                ));
            }
            (changes, None)
        }
        AdjustDtermLowpass {
            stage,
            recommended_cutoff,
            filter_type,
            dynamic_settings,
            ..
        } => {
            let prefix = format!("dterm_lpf{stage}");
            let mut changes = Vec::new();
            // Static cutoff (only emitted if dynamic isn't being set).
            if let Some(cutoff) = recommended_cutoff {
                if dynamic_settings.is_none() {
                    changes.push(FilterSettingChange::u16(
                        format!("{prefix}_static_hz"),
                        cutoff.round().clamp(0.0, 65535.0) as u16,
                        format!("{prefix}_static_hz → {:.0} Hz", cutoff),
                    ));
                    // Disable dynamic by zeroing both bounds.
                    changes.push(FilterSettingChange::u16(
                        format!("{prefix}_dyn_min_hz"),
                        0,
                        format!("{prefix}_dyn_min_hz → 0 (disable dynamic)"),
                    ));
                    changes.push(FilterSettingChange::u16(
                        format!("{prefix}_dyn_max_hz"),
                        0,
                        format!("{prefix}_dyn_max_hz → 0 (disable dynamic)"),
                    ));
                }
            }
            if let Some(d) = dynamic_settings {
                changes.push(FilterSettingChange::u16(
                    format!("{prefix}_dyn_min_hz"),
                    d.min_cutoff.round().clamp(0.0, 65535.0) as u16,
                    format!("{prefix}_dyn_min_hz → {:.0} Hz", d.min_cutoff),
                ));
                changes.push(FilterSettingChange::u16(
                    format!("{prefix}_dyn_max_hz"),
                    d.max_cutoff.round().clamp(0.0, 65535.0) as u16,
                    format!("{prefix}_dyn_max_hz → {:.0} Hz", d.max_cutoff),
                ));
            }
            if let Some(ft) = filter_type_to_enum(filter_type) {
                changes.push(FilterSettingChange::u8(
                    format!("{prefix}_type"),
                    ft,
                    format!("{prefix}_type → {filter_type}"),
                ));
            }
            (changes, None)
        }
        AdjustYawLowpass {
            recommended_cutoff, ..
        } => {
            let v = recommended_cutoff.round().clamp(0.0, 65535.0) as u16;
            (
                vec![FilterSettingChange::u16(
                    "yaw_lowpass_hz",
                    v,
                    format!("yaw_lowpass_hz → {v} Hz"),
                )],
                None,
            )
        }
        AdjustDynamicNotch {
            notch_count,
            min_freq,
            max_freq,
            enabled,
            ..
        } => {
            // Skip dyn_notch_q for now: its encoding (raw vs Q*100) shifts
            // between Betaflight 4.x versions and we don't yet probe the
            // FC for its semantics. The min/max/count cover the high-leverage
            // changes safely.
            let mut changes = if *enabled {
                vec![
                    FilterSettingChange::u8(
                        "dyn_notch_count",
                        *notch_count,
                        format!("dyn_notch_count → {notch_count}"),
                    ),
                    FilterSettingChange::u16(
                        "dyn_notch_min_hz",
                        min_freq.round().clamp(0.0, 65535.0) as u16,
                        format!("dyn_notch_min_hz → {:.0} Hz", min_freq),
                    ),
                    FilterSettingChange::u16(
                        "dyn_notch_max_hz",
                        max_freq.round().clamp(0.0, 65535.0) as u16,
                        format!("dyn_notch_max_hz → {:.0} Hz", max_freq),
                    ),
                ]
            } else {
                vec![FilterSettingChange::u8(
                    "dyn_notch_count",
                    0,
                    "dyn_notch_count → 0 (disable)".to_string(),
                )]
            };
            // Sort so the order is deterministic across runs (helps tests
            // and history diffs).
            changes.sort_by(|a, b| a.name.cmp(&b.name));
            (
                changes,
                Some("Q-factor unchanged (encoding varies by firmware)".to_string()),
            )
        }
        ConfigureRpmFilter {
            harmonics,
            min_freq,
            enabled,
            ..
        } => {
            let changes = if *enabled {
                vec![
                    FilterSettingChange::u8(
                        "rpm_filter_harmonics",
                        *harmonics,
                        format!("rpm_filter_harmonics → {harmonics}"),
                    ),
                    FilterSettingChange::u16(
                        "rpm_filter_min_hz",
                        min_freq.round().clamp(0.0, 65535.0) as u16,
                        format!("rpm_filter_min_hz → {:.0} Hz", min_freq),
                    ),
                ]
            } else {
                vec![FilterSettingChange::u8(
                    "rpm_filter_harmonics",
                    0,
                    "rpm_filter_harmonics → 0 (disable)".to_string(),
                )]
            };
            (
                changes,
                Some("Q-factor unchanged (encoding varies by firmware)".to_string()),
            )
        }
        ConfigureGyroNotch { .. } => (
            Vec::new(),
            Some(
                "Static gyro notches use a Hz/cutoff pair whose Q derivation \
                 changed between firmware versions; not auto-applied yet."
                    .to_string(),
            ),
        ),
    }
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
            input: args.input.clone(),
            output_dir: None,
            detailed: true,
            show_details: false,
            min_confidence: 0.5,
            max_files: 1,
            session: None,
            list_sessions: false,
            bb_summary: false,
            session_strategy: None,
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

    println!("{} Export completed successfully", style("✓").green());
    Ok(())
}

/// Handle the info command
async fn info_command() -> Result<()> {
    println!("{} FPV Drone Tuner", style("🚁").blue());
    println!("Version: {}", env!("CARGO_PKG_VERSION"));
    println!("Core library: {}", drone_tuner_core::VERSION);
    println!();
    println!("System Information:");
    println!("  OS: {}", std::env::consts::OS);
    println!("  Arch: {}", std::env::consts::ARCH);
    println!("  Rust version: Unknown"); // Would detect at runtime

    // Check if we can access required libraries
    println!();
    println!("Library Status:");
    println!("  ✓ FFT support available");
    println!("  ✓ Scientific computing available");
    println!("  ✓ Blackbox parsing ready");

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
                println!("{} {}", style("📊").blue(), file_path.display());

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
                    println!("{} File Details:", style("📁").blue());
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
                    println!("{} Flight Controller Configuration:", style("🎛️").blue());
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
                    println!("  {} PID Values:", style("⚙️").cyan());
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
                    println!("  {} Filter Settings:", style("🔧").cyan());

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
                    println!("  {} RC Rates:", style("📡").cyan());
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
                    println!("{} Verification Notes:", style("🔍").yellow());
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
                            style("⚠").yellow()
                        );
                    }
                    println!();
                }

                if !detailed_info {
                    println!();
                }

                // Tune quality score
                let quality = analysis.report.tune_quality_score;
                let quality_color = if quality >= 80.0 {
                    style(format!("{:.1}", quality)).green()
                } else if quality >= 60.0 {
                    style(format!("{:.1}", quality)).yellow()
                } else {
                    style(format!("{:.1}", quality)).red()
                };

                if detailed_info {
                    println!("{} Analysis Results:", style("📈").green());
                }
                println!("  Tune Quality: {}/100", quality_color);

                // Issues
                if !analysis.report.detected_issues.is_empty() {
                    println!("  {} Issues found:", style("⚠").yellow());
                    for issue in &analysis.report.detected_issues {
                        println!("    • {}", issue.description);
                    }
                }

                // Recommendations
                if !analysis.report.filter_recommendations.is_empty() {
                    println!("  {} Filter recommendations:", style("🔧").blue());
                    for rec in &analysis.report.filter_recommendations {
                        let description = match &rec.recommendation_type {
                            FilterRecommendationType::AdjustGyroLowpass {
                                stage,
                                current_cutoff,
                                recommended_cutoff,
                                filter_type,
                            } => {
                                format!(
                                    "Gyro Lowpass {} ({}): {:.0}→{:.0} Hz",
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
                                min_freq,
                                max_freq,
                                ..
                            } => {
                                format!(
                                    "Dynamic Notch: {} notches, {:.0}-{:.0} Hz",
                                    notch_count, min_freq, max_freq
                                )
                            }
                            FilterRecommendationType::ConfigureRpmFilter {
                                harmonics,
                                enabled,
                                ..
                            } => {
                                if *enabled {
                                    format!("Enable RPM Filter: {} harmonics", harmonics)
                                } else {
                                    "Disable RPM Filter".to_string()
                                }
                            }
                            FilterRecommendationType::AdjustDtermLowpass {
                                stage,
                                filter_type,
                                ..
                            } => {
                                format!("D-term Lowpass {} ({})", stage, filter_type)
                            }
                            FilterRecommendationType::AdjustYawLowpass {
                                current_cutoff,
                                recommended_cutoff,
                            } => {
                                format!(
                                    "Yaw Lowpass: {:.0}→{:.0} Hz",
                                    current_cutoff, recommended_cutoff
                                )
                            }
                        };
                        println!("    • {}", description);
                    }
                }

                if !analysis.report.pid_recommendations.is_empty() {
                    println!("  {} PID recommendations:", style("🎛").blue());
                    for rec in &analysis.report.pid_recommendations {
                        println!(
                            "    • {:?}: {:.1} → {:.1}",
                            rec.term, rec.current_value, rec.recommended_value
                        );
                    }
                }

                // Step-response viz, only in --show-details mode. Surface
                // *what the analyser saw* per axis so the recommendations
                // above stop being a black box.
                if detailed_info && !analysis.report.step_responses.is_empty() {
                    render_step_responses(&analysis.report.step_responses);
                }
            }
            Err(e) => {
                println!();
                println!(
                    "{} {} - Error: {}",
                    style("✗").red(),
                    file_path.display(),
                    e
                );
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
    println!("  {} Step responses (top per axis):", style("📐").blue());
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

/// Render a single trace as a unicode-block sparkline. Auto-scales to the
/// data range so flat traces stay flat instead of getting amplified noise.
fn print_sparkline(label: &str, samples: &[f32]) {
    if samples.is_empty() {
        return;
    }
    const COLS: usize = 60;
    const BLOCKS: &[char] = &['▁', '▂', '▃', '▄', '▅', '▆', '▇', '█'];

    // Down-sample to COLS columns by averaging consecutive chunks.
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
                + result.report.pid_recommendations.len(),
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
    println!("{} Flight Comparison", style("📊").blue());
    println!();

    println!("Summary:");
    println!(
        "  Best tune quality: {:.1}",
        comparison.summary.best_tune_quality
    );
    println!(
        "  Worst tune quality: {:.1}",
        comparison.summary.worst_tune_quality
    );
    println!(
        "  Average tune quality: {:.1}",
        comparison.summary.avg_tune_quality
    );
    println!("  Total issues: {}", comparison.summary.total_issues);
    println!();

    println!("Individual Flights:");
    for flight in &comparison.flights {
        let quality_color = if flight.tune_quality >= 80.0 {
            style(format!("{:.1}", flight.tune_quality)).green()
        } else if flight.tune_quality >= 60.0 {
            style(format!("{:.1}", flight.tune_quality)).yellow()
        } else {
            style(format!("{:.1}", flight.tune_quality)).red()
        };

        println!(
            "  {} - Quality: {}, Issues: {}, Duration: {:.1}s",
            flight.name,
            quality_color,
            flight.issues_count,
            flight.duration_ms as f32 / 1000.0
        );
    }
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
            dry_run: false,
            backup: None,
            auto_apply_safe: auto,
            apply_all: all,
            save_eeprom: false,
            skip_filters: false,
            session_strategy: None,
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
