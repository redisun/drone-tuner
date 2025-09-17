# FPV Drone Tuning Platform

A high-performance Rust-based platform for analyzing FPV drone blackbox logs and providing intelligent tuning recommendations.

## Project Structure

This project follows a workspace-based architecture with separate crates for modularity and reusability:

```
drone-tuner/
├── crates/
│   ├── drone-tuner-core/    # Core analysis library
│   └── drone-tuner-cli/     # Command-line interface
├── examples/                # Usage examples
├── docs/                   # Technical documentation
└── README.md              # This file
```

## Features

### Core Library (`drone-tuner-core`)

- **Blackbox Parsing**: Support for Betaflight and other flight controller log formats
- **FFT Analysis**: High-performance frequency domain analysis using Welch's method
- **Oscillation Detection**: Automated detection of P-term, D-term, and mechanical oscillations
- **Filter Optimization**: Intelligent filter configuration recommendations
- **Real-time Communication**: Live flight controller connectivity (with `realtime` feature)
- **Domain Models**: Comprehensive data structures representing flight sessions and analysis results

### Command-Line Interface (`drone-tuner-cli`)

- **File Analysis**: Analyze single blackbox files or entire directories
- **Batch Processing**: Process multiple flights with progress tracking
- **Multiple Output Formats**: Pretty-printed, JSON, and CSV output options
- **Flight Comparison**: Compare multiple flights and identify patterns
- **Validation Tools**: Verify blackbox file integrity and detect common issues

## Quick Start

### Installation

```bash
# Clone the repository
git clone <repository-url>
cd drone-tuner

# Build the project
cargo build --release
```

### Basic Usage

```bash
# Show system information and capabilities
cargo run --bin drone-tuner info

# Analyze a single blackbox file
cargo run --bin drone-tuner analyze path/to/flight.bbl

# Analyze multiple files with JSON output
cargo run --bin drone-tuner analyze --output json logs/

# Compare multiple flights
cargo run --bin drone-tuner compare flight1.bbl flight2.bbl flight3.bbl

# Validate blackbox files
cargo run --bin drone-tuner validate --check-issues logs/
```

### Library Usage

```rust
use drone_tuner_core::{AnalysisEngine, BlackboxParser};

// Parse a blackbox file
let data = std::fs::read("flight.bbl")?;
let mut parser = BlackboxParser::new();
let session = parser.parse_file(&data)?;

// Analyze for oscillations and get recommendations
let mut engine = AnalysisEngine::new();
let report = engine.analyze(&session)?;

println!("Tune quality: {:.1}/100", report.tune_quality_score);
println!("Issues found: {}", report.detected_issues.len());
```

## Architecture Overview

### Domain-Driven Design

The core library is built around a rich domain model that captures the essential concepts of drone tuning:

- **FlightSession**: Complete flight data with metadata, telemetry, and analysis results
- **TelemetryData**: Time-series sensor data (gyro, accelerometer, motors, etc.)
- **AnalysisReport**: Comprehensive analysis results with confidence scores
- **FilterConfiguration**: Current and recommended filter settings
- **HardwareConfiguration**: Drone setup information

### Performance Characteristics

- **FFT Analysis**: Optimized using RustFFT for blazing-fast frequency analysis
- **Memory Efficiency**: Zero-copy parsing where possible, efficient data structures
- **Concurrent Processing**: Designed for batch processing of multiple flights
- **Streaming**: Real-time analysis capabilities for live tuning sessions

### Safety and Reliability

- **Memory Safety**: Rust's ownership system prevents memory corruption
- **Error Handling**: Comprehensive error types with context information
- **Input Validation**: Robust parsing that handles corrupted or malformed data
- **Testing**: Extensive unit tests, integration tests, and benchmarks

## Development

### Prerequisites

- Rust 1.70+ (2021 edition)
- Cargo for dependency management

### Building

```bash
# Debug build
cargo build

# Release build (optimized)
cargo build --release

# Run tests
cargo test

# Run benchmarks
cargo bench

# Check code quality
cargo clippy
cargo fmt
```

### Project Features

The core library supports optional features:

- `realtime`: Enables real-time flight controller communication (requires `serialport`)

Enable features during build:

```bash
cargo build --features realtime
```

### Benchmarking

Performance benchmarks are available for critical components:

```bash
# Run all benchmarks
cargo bench

# Run specific benchmark
cargo bench fft_analysis
cargo bench blackbox_parser
```

### Testing

The project includes comprehensive testing:

```bash
# Run all tests
cargo test

# Run tests with output
cargo test -- --nocapture

# Run tests for specific crate
cargo test -p drone-tuner-core
```

## Technical Details

### Supported Formats

- **Blackbox Logs**: Betaflight BBL format (compressed and uncompressed)
- **Data Fields**: Gyro, accelerometer, motors, PID errors, RC commands
- **Firmware**: Betaflight, INAV, ArduPilot (with appropriate field mappings)

### Analysis Capabilities

- **Frequency Analysis**: Power spectral density using Welch's method
- **Peak Detection**: Automatic identification of problematic frequencies
- **Oscillation Classification**: P-term, D-term, mechanical, and motor noise
- **Filter Design**: Butterworth, notch, and dynamic notch filters
- **Performance Metrics**: Tune quality scoring and confidence assessment

### Real-time Features (Optional)

- **MSP Protocol**: MultiWii Serial Protocol for flight controller communication
- **Live Telemetry**: Real-time data streaming and analysis
- **Parameter Management**: Read/write flight controller parameters
- **Auto-tuning**: Automated tuning sequences with safety checks

## Output Examples

### Pretty Format (Default)

```
📊 /path/to/flight.bbl
  Duration: 45.2s
  Samples: 45187
  Sample rate: 1000 Hz
  Analysis time: 0.85s

  Tune Quality: 73.5/100

  ⚠ Issues found:
    • P-term oscillation detected at 52.3 Hz. Consider reducing P gain.
    • D-term oscillation detected at 180.1 Hz. Consider reducing D gain or lowering D-term filter cutoff.

  🔧 Filter recommendations:
    • AddNotchFilter at 52.3 Hz
    • AdjustLowPassCutoff at 180.1 Hz

  📈 Frequency Analysis:
    Top frequency peaks:
      1. 52.3 Hz (amplitude: 2.15)
      2. 180.1 Hz (amplitude: 1.87)
      3. 340.5 Hz (amplitude: 0.92)
```

### JSON Format

```json
{
  "version": "0.1.0",
  "timestamp": "2024-01-15T10:30:00Z",
  "results": [
    {
      "file": "/path/to/flight.bbl",
      "status": "success",
      "tune_quality": 73.5,
      "duration_ms": 45200,
      "sample_rate": 1000.0,
      "samples": 45187,
      "analysis_time_ms": 850,
      "issues": 2,
      "filter_recommendations": 2,
      "pid_recommendations": 0
    }
  ]
}
```

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Make your changes following Rust best practices
4. Add tests for new functionality
5. Run `cargo test` and `cargo clippy`
6. Commit your changes (`git commit -m 'Add amazing feature'`)
7. Push to the branch (`git push origin feature/amazing-feature`)
8. Open a Pull Request

## License

This project is licensed under the MIT OR Apache-2.0 license.

## Acknowledgments

- RustFFT for high-performance FFT implementation
- The FPV community for domain expertise and testing
- Betaflight project for blackbox format documentation