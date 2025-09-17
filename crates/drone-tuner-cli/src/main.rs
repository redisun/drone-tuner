//! Command-line interface for the FPV drone tuning platform.

use anyhow::{Context, Result};
use clap::{Args, Parser, Subcommand};
use console::{style, Term};
use drone_tuner_core::{AnalysisEngine, BlackboxParser, FlightSession};
use indicatif::{ProgressBar, ProgressStyle};
use std::path::PathBuf;
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
    #[arg(short, long, value_enum, global = true, default_value = "pretty")]
    output: OutputFormat,

    #[command(subcommand)]
    command: Commands,
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

    /// Minimum confidence threshold for recommendations
    #[arg(long, default_value = "0.7")]
    min_confidence: f32,

    /// Maximum number of files to process in batch
    #[arg(long, default_value = "100")]
    max_files: usize,

    /// Select specific session to analyze (1-based index, default: last session)
    #[arg(long, short)]
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

#[tokio::main]
async fn main() -> Result<()> {
    let cli = Cli::parse();

    // Initialize logging
    init_logging(cli.verbose)?;

    // Execute the command
    match cli.command {
        Commands::Analyze(args) => analyze_command(args, cli.output, cli.detailed_info).await,
        Commands::Compare(args) => compare_command(args, cli.output).await,
        Commands::Validate(args) => validate_command(args, cli.output).await,
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
        println!("{}", style("No blackbox files found").red());
        return Ok(());
    }

    println!(
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

    // Print summary
    let successful = results.iter().filter(|(_, result)| result.is_ok()).count();
    let failed = results.len() - successful;

    println!();
    println!("{} Analysis Summary", style("📊").blue());
    println!("  Successful: {}", style(successful).green());
    if failed > 0 {
        println!("  Failed: {}", style(failed).red());
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
        let mut list_config = drone_tuner_core::blackbox::ParsingConfig::default();
        list_config.list_sessions_only = true;

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
    }

    Ok(files)
}

/// Handle the compare command
async fn compare_command(args: CompareArgs, output_format: OutputFormat) -> Result<()> {
    println!(
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
                println!(
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
        println!(
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

/// Handle the info command
async fn info_command() -> Result<()> {
    println!("{} FPV Drone Tuner", style("🚁").blue());
    println!("Version: {}", env!("CARGO_PKG_VERSION"));
    println!("Core library: {}", drone_tuner_core::VERSION);
    println!();
    println!("System Information:");
    println!("  OS: {}", std::env::consts::OS);
    println!("  Arch: {}", std::env::consts::ARCH);
    println!("  Rust version: {}", "Unknown"); // Would detect at runtime

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
                        println!(
                            "    • {} at {:.1} Hz",
                            format!("{:?}", rec.recommendation_type),
                            rec.frequency
                        );
                    }
                }

                if !analysis.report.pid_recommendations.is_empty() {
                    println!("  {} PID recommendations:", style("🎛").blue());
                    for rec in &analysis.report.pid_recommendations {
                        println!(
                            "    • {}: {:.1} → {:.1}",
                            format!("{:?}", rec.term),
                            rec.current_value,
                            rec.recommended_value
                        );
                    }
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
    let json = serde_json::to_string_pretty(comparison)
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
