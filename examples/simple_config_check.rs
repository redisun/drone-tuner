//! Simple configuration check to verify header extraction is working

use drone_tuner_core::BlackboxParser;
use std::fs;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let sample_file = "/home/flo/workspace/personal/drone-tuner/btfl_010.bbl";

    println!("🔧 Simple Configuration Check");
    println!("=============================");

    let data = fs::read(sample_file)?;
    println!("✅ Loaded file: {:.2} MB", data.len() as f64 / 1_000_000.0);

    let mut parser = BlackboxParser::new();
    let session = parser.parse_file(&data)?;

    println!("\n📡 Flight Controller:");
    println!(
        "   - Firmware: {}",
        session.metadata.hardware.flight_controller.firmware
    );
    println!(
        "   - Version: {}",
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

    println!("\n🎯 PID Values:");
    println!(
        "   Roll  PID: P={:.1}, I={:.1}, D={:.1}",
        session.metadata.hardware.pid_config.roll.p,
        session.metadata.hardware.pid_config.roll.i,
        session.metadata.hardware.pid_config.roll.d
    );
    println!(
        "   Pitch PID: P={:.1}, I={:.1}, D={:.1}",
        session.metadata.hardware.pid_config.pitch.p,
        session.metadata.hardware.pid_config.pitch.i,
        session.metadata.hardware.pid_config.pitch.d
    );
    println!(
        "   Yaw   PID: P={:.1}, I={:.1}, D={:.1}",
        session.metadata.hardware.pid_config.yaw.p,
        session.metadata.hardware.pid_config.yaw.i,
        session.metadata.hardware.pid_config.yaw.d
    );

    println!("\n🔧 Filter Info:");
    println!(
        "   Gyro filters: {} configured",
        session.metadata.hardware.filter_config.gyro_filters.len()
    );
    println!(
        "   D-term filters: {} configured",
        session.metadata.hardware.filter_config.dterm_filters.len()
    );

    if let Some(dynamic_notch) = &session.metadata.hardware.filter_config.dynamic_notch {
        println!(
            "   Dynamic notch: {:.0}Hz - {:.0}Hz",
            dynamic_notch.min_freq, dynamic_notch.max_freq
        );
    } else {
        println!("   Dynamic notch: Not configured");
    }

    // Check if we're getting real data vs defaults
    let has_real_config = session.metadata.hardware.pid_config.roll.p != 100.0
        || !session
            .metadata
            .hardware
            .flight_controller
            .firmware
            .contains("Unknown");

    if has_real_config {
        println!("\n✅ Configuration extracted successfully from blackbox headers!");
    } else {
        println!("\n❌ Still using default configuration - header extraction may not be working");
    }

    println!("\n✅ Configuration check complete!");
    Ok(())
}
