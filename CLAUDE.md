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

This is an early-stage project with comprehensive planning documents but no implementation yet. The codebase currently contains only documentation:
- `docs/technical-development-document.md` - Detailed technical architecture and implementation patterns
- `docs/prd.md` - Product requirements and feature specifications

## Planned Project Structure

```
src/
├── lib.rs              # Main library entry point
├── domain/             # Core domain models (FlightSession, TelemetryData, etc.)
├── analysis/           # Analysis engine (FFT, oscillation detection)
├── blackbox/           # Blackbox parser implementation
├── filters/            # Filter optimization algorithms
├── realtime/           # Flight controller communication
├── ml/                 # Machine learning components
└── cli/                # Command-line interface

desktop/                # Tauri desktop application
├── src-tauri/         # Rust backend
└── src/               # React frontend
```

## Key Technical Decisions

- **Language**: Rust for memory safety, performance, and multi-platform compilation
- **Desktop Framework**: Tauri for lightweight native apps with web UI
- **Signal Processing**: rustfft for frequency analysis
- **ML Framework**: candle or ONNX runtime for model inference
- **Communication**: tokio async runtime for real-time FC connectivity

## Core Dependencies (Planned)

```toml
# Scientific computing
ndarray = "0.15"
rustfft = "6.0" 
nalgebra = "0.32"

# Machine learning
candle = "0.3"

# Serialization
serde = { version = "1.0", features = ["derive"] }
bincode = "1.3"

# Async runtime
tokio = { version = "1.0", features = ["full"] }

# Parsing
nom = "7.0"
```

## Development Phases

1. **Core Foundation**: Blackbox parser and basic FFT analysis
2. **Analysis Engine**: Oscillation detection and filter recommendations  
3. **Desktop Application**: Tauri app with visualization
4. **Real-time Integration**: Live FC communication and auto-tune
5. **ML Enhancement**: Pattern recognition and anomaly detection
6. **Community Features**: Tune marketplace and sharing

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