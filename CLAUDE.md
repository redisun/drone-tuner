# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an FPV Tuning Intelligence Platform built in Rust - a desktop application that analyzes drone blackbox logs to provide intelligent tuning recommendations. The project aims to democratize professional-grade drone tuning by transforming complex flight data into actionable insights.

## Architecture

The system follows a hybrid architecture:
- **Core Analysis Engine**: Pure Rust library handling FFT analysis, oscillation detection, and filter optimization
- **Desktop Application**: Tauri-based app with React frontend and Rust backend
- **CLI Tool**: Command-line interface for power users and automation
- **Real-time Connectivity**: Live flight controller communication via USB/Bluetooth/WiFi

Key components:
- Blackbox parser for various log formats
- Frequency domain analysis using FFT
- Oscillation detection and categorization
- Filter optimization algorithms
- ML inference for pattern recognition
- Real-time MSP (MultiWii Serial Protocol) communication

## Development Status

Functional and validated on hardware. The CLI analyses real Betaflight blackbox files end-to-end (parse → FFT → oscillation detection → filter/PID recommendations) and applies the resulting PID changes over MSP with backup/rollback + EEPROM persistence. The MSP path has been exercised on two real flight controllers (Jeno STM32H743, TBS Source One STM32F7x2) across multiple tune iterations and survives power cycles. The Tauri desktop app and ML inference are still aspirational — none of that code exists yet.

All features are default-on. There are no Cargo feature flags: `tune`, `monitor`, MSP serial, and the in-process `simulator://` MSP simulator are always available.

See `docs/PROJECT_ASSESSMENT.md` for the current shortcomings list and prioritised path forward.

Reference docs:
- `docs/technical-development-document.md` — technical architecture & implementation patterns
- `docs/prd.md` — product requirements
- `docs/PROJECT_ASSESSMENT.md` — current state vs. ambition + roadmap

## Project Structure (actual)

Cargo workspace, two crates:

```
crates/
├── drone-tuner-core/         # analysis library
│   └── src/
│       ├── lib.rs            # public API surface
│       ├── domain.rs         # FlightSession, TelemetryData, recommendations
│       ├── analysis.rs       # FFT + oscillation detection + filter optimiser + PID analysis
│       ├── analysis/         # test modules (tests, realistic_tests, debug_tests, debug_d_term)
│       ├── blackbox/
│       │   ├── mod.rs        # parsing config / stats / field mappings
│       │   ├── simple_parser.rs  # custom Betaflight BBL parser (the working one)
│       │   └── converter.rs  # raw-int → physical-unit conversion
│       ├── filters.rs        # Butterworth / notch / biquad design
│       ├── error.rs          # DronetunerError + Result
│       └── realtime.rs       # MSP scaffolding (feature-gated, not production-ready)
└── drone-tuner-cli/          # binary `drone-tuner`
    ├── src/main.rs           # 7 subcommands: info, analyze, compare, validate, monitor, tune, export
    └── tests/                # integration + command_specific + performance suites

docs/                         # PRD, technical doc, project assessment
test_data/                    # real .bbl files for integration testing
examples/                     # standalone example binaries
```

Tauri/desktop and ML directories are **not yet present** despite being mentioned in the PRD.

## Key Technical Decisions

- **Language**: Rust for memory safety, performance, and multi-platform compilation
- **Desktop Framework**: Tauri for lightweight native apps with web UI
- **Signal Processing**: rustfft for frequency analysis
- **ML Framework**: candle or ONNX runtime for model inference
- **Communication**: tokio async runtime for real-time FC connectivity

## Core Dependencies (actual)

Workspace dependencies live in the root `Cargo.toml`. Highlights:

- **Signal processing:** `rustfft 6.2`, `ndarray 0.16`, `nalgebra 0.34`, `num-complex 0.4`
- **Blackbox parsing:** `blackbox-log 0.4` (reference) + custom `SimpleBlackboxParser`
- **CLI:** `clap 4.4`, `console`, `indicatif`
- **Async / realtime (feature-gated):** `tokio 1.35`, `serialport 4.2`, `async-trait`
- **Serialisation:** `serde`, `serde_json`, `bincode 2.0`
- **Errors:** `thiserror`, `anyhow`
- **Tracing:** `tracing`, `tracing-subscriber`
- **Tests / bench:** `criterion 0.7`, `proptest`, `tokio-test`

ML deps (`candle`, ONNX runtime) are **not** added yet.

## Development Phases

1. **Core Foundation** — Blackbox parser and basic FFT analysis. ✅ Done.
2. **Analysis Engine** — Oscillation detection and filter recommendations. ✅ Working; calibration golden tests in place.
3. **Desktop Application** — Tauri app with visualisation. ❌ Not started.
4. **Real-time Integration** — Live FC communication and auto-tune. ✅ Validated end-to-end on two real flight controllers; EEPROM persistence verified across power cycles.
5. **ML Enhancement** — Pattern recognition and anomaly detection. ❌ Not started; deferred until labelled dataset exists.
6. **Community Features** — Tune marketplace and sharing. ❌ Not started.

For the prioritised cleanup roadmap see `docs/PROJECT_ASSESSMENT.md`.

## Performance Requirements

- Blackbox parsing: < 500ms for 100MB file
- FFT analysis: < 1 second for 5-minute flight
- Memory usage: < 500MB for typical session
- Startup time: < 2 seconds to interactive state

## Safety Considerations

This system provides tuning recommendations that can affect flight safety. All recommendations must include:
- Confidence levels and safety warnings
- Backup configuration preservation
- Easy rollback mechanisms
- Validation against safe parameter ranges