//! Configuration inspection tool to verify configuration extraction from blackbox headers

use drone_tuner_core::BlackboxParser;
use std::fs;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let sample_file = "/home/flo/workspace/personal/drone-tuner/btfl_010.bbl";

    println!("🔧 Configuration Inspection Tool");
    println!("=================================");

    let data = fs::read(sample_file)?;
    println!("✅ Loaded file: {:.2} MB", data.len() as f64 / 1_000_000.0);

    let mut parser = BlackboxParser::new();
    let session = parser.parse_file(&data)?;

    println!("\n🎛️ Hardware Configuration:");

    // Flight Controller Info
    println!("📡 Flight Controller:");
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

    // PID Configuration
    println!("\n🎯 PID Configuration:");
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

    // Show RC rates from PID settings
    println!("\n🎮 RC Configuration:");
    println!(
        "   RC Rates: [{:.1}, {:.1}, {:.1}]",
        session.metadata.hardware.pid_config.settings.rates.roll_rate,
        session.metadata.hardware.pid_config.settings.rates.pitch_rate,
        session.metadata.hardware.pid_config.settings.rates.yaw_rate
    );

    // Filter Configuration
    println!("\n🔧 Filter Configuration:");
    println!(
        "   Gyro filters: {} configured",
        session.metadata.hardware.filter_config.gyro_filters.len()
    );
    println!(
        "   D-term filters: {} configured",
        session.metadata.hardware.filter_config.dterm_filters.len()
    );
    println!(
        "   Notch filters: {} configured",
        session.metadata.hardware.filter_config.notch_filters.len()
    );

    if let Some(dynamic_notch) = &session.metadata.hardware.filter_config.dynamic_notch {
        println!(
            "   Dynamic notch: {:.0}Hz - {:.0}Hz (Q: {:.0}, enabled: {})",
            dynamic_notch.min_freq,
            dynamic_notch.max_freq,
            dynamic_notch.q_factor,
            dynamic_notch.enabled
        );
    }

    println!("\n✅ Configuration inspection complete!");
    Ok(())
}
