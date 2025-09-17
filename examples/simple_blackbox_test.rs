//! Simple example demonstrating blackbox-log crate integration

use drone_tuner_core::{blackbox::utils, BlackboxParser};
use std::fs;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Try to load the sample blackbox file
    let sample_file = "/home/flo/workspace/personal/drone-tuner/btfl_010.bbl";

    println!("🚁 Drone Tuner - Blackbox Integration Test");
    println!("==========================================");

    if let Ok(data) = fs::read(sample_file) {
        println!(
            "✅ Loaded blackbox file: {:.2} MB",
            data.len() as f64 / 1_000_000.0
        );

        // Test file format detection
        if utils::is_blackbox_file(&data) {
            println!("✅ File format validated as blackbox log");
        } else {
            println!("❌ File format validation failed");
            return Ok(());
        }

        // Test basic info extraction
        match utils::extract_basic_info(&data) {
            Ok(info) => {
                println!("📋 Basic file information:");
                for (key, value) in &info {
                    println!("   - {}: {}", key, value);
                }
            }
            Err(e) => {
                println!("⚠️  Could not extract basic info: {}", e);
            }
        }

        // Test parsing with our simple parser
        println!("\n🔍 Parsing with simple blackbox parser...");
        let mut parser = BlackboxParser::new();

        match parser.parse_file(&data) {
            Ok(session) => {
                println!("✅ Successfully parsed blackbox file!");
                println!("📊 Flight session details:");
                println!("   - Session ID: {}", session.metadata.session_id);
                println!("   - Duration: {}ms", session.metadata.duration_ms);
                println!("   - Sample rate: {:.1}Hz", session.telemetry.sample_rate);
                println!("   - Gyro samples: {}", session.telemetry.gyro.len());
                println!("   - Accel samples: {}", session.telemetry.accel.len());
                println!("   - Motor traces: {}", session.telemetry.motor.len());
                println!("   - Events: {}", session.events.len());

                println!("\n⚡ Hardware configuration:");
                println!(
                    "   - Firmware: {} {}",
                    session.metadata.hardware.flight_controller.firmware,
                    session.metadata.hardware.flight_controller.version
                );
                println!(
                    "   - Target: {}",
                    session.metadata.hardware.flight_controller.target
                );
                println!(
                    "   - Loop rate: {}Hz",
                    session.metadata.hardware.flight_controller.loop_rate
                );

                let stats = parser.stats();
                println!("\n📈 Parsing statistics:");
                println!("   - Parse duration: {}ms", stats.parse_duration_ms);
                println!("   - Bytes processed: {}", stats.bytes_processed);
                println!("   - Total frames: {}", stats.total_frames);

                println!("\n🎯 Integration test completed successfully!");
            }
            Err(e) => {
                println!("❌ Failed to parse blackbox file: {}", e);
                return Err(e.into());
            }
        }
    } else {
        println!("⚠️  Sample blackbox file not found at {}", sample_file);
        println!("   This example requires a blackbox log file to demonstrate parsing.");
        println!("   Place a .bbl file at the specified path to test the integration.");
    }

    Ok(())
}
