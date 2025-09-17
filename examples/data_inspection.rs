//! Data inspection tool to verify blackbox parsing accuracy

use drone_tuner_core::BlackboxParser;
use std::fs;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let sample_file = "/home/flo/workspace/personal/drone-tuner/btfl_010.bbl";

    println!("🔍 Data Inspection Tool");
    println!("======================");

    let data = fs::read(sample_file)?;
    println!("✅ Loaded file: {:.2} MB", data.len() as f64 / 1_000_000.0);

    let mut parser = BlackboxParser::new();
    let session = parser.parse_file(&data)?;

    println!("\n📊 Sample Data Values:");

    // Check first 10 gyro samples
    println!("🌀 First 10 Gyro samples (rad/s):");
    for i in 0..10.min(session.telemetry.gyro.len()) {
        if let Some(gyro) = session.telemetry.gyro.get(i) {
            println!("  {}: x={:.6}, y={:.6}, z={:.6}", i, gyro.x, gyro.y, gyro.z);
        }
    }

    // Check accelerometer values
    println!("\n📈 First 10 Accelerometer samples (m/s²):");
    for i in 0..10.min(session.telemetry.accel.len()) {
        if let Some(accel) = session.telemetry.accel.get(i) {
            let magnitude = (accel.x * accel.x + accel.y * accel.y + accel.z * accel.z).sqrt();
            println!(
                "  {}: x={:.3}, y={:.3}, z={:.3} (mag: {:.3}g)",
                i,
                accel.x,
                accel.y,
                accel.z,
                magnitude / 9.81
            );
        }
    }

    // Check RC commands
    println!("\n🎮 RC Command availability:");
    println!(
        "  Roll: {} samples",
        session.telemetry.rc_commands.roll.len()
    );
    println!(
        "  Pitch: {} samples",
        session.telemetry.rc_commands.pitch.len()
    );
    println!("  Yaw: {} samples", session.telemetry.rc_commands.yaw.len());
    println!(
        "  Throttle: {} samples",
        session.telemetry.rc_commands.throttle.len()
    );

    if !session.telemetry.rc_commands.roll.is_empty() {
        println!("\n🎮 First 10 RC Command samples:");
        for i in 0..10.min(session.telemetry.rc_commands.roll.len()) {
            println!(
                "  {}: roll={:.0}, pitch={:.0}, yaw={:.0}, throttle={:.0}",
                i,
                session.telemetry.rc_commands.roll[i],
                session.telemetry.rc_commands.pitch[i],
                session.telemetry.rc_commands.yaw[i],
                session.telemetry.rc_commands.throttle[i],
            );
        }
    }

    // Statistical analysis
    println!("\n📈 Data Statistics:");

    if !session.telemetry.gyro.is_empty() {
        let gyro_x_values = &session.telemetry.gyro.x;
        let mean_x = gyro_x_values.iter().sum::<f32>() / gyro_x_values.len() as f32;
        let max_x = gyro_x_values
            .iter()
            .fold(f32::NEG_INFINITY, |a, &b| a.max(b));
        let min_x = gyro_x_values.iter().fold(f32::INFINITY, |a, &b| a.min(b));

        println!(
            "  Gyro X: mean={:.3}, min={:.3}, max={:.3} rad/s",
            mean_x, min_x, max_x
        );
    }

    if !session.telemetry.accel.is_empty() {
        let accel_magnitudes = session.telemetry.accel.magnitude();
        let mean_mag = accel_magnitudes.iter().sum::<f32>() / accel_magnitudes.len() as f32;
        let max_mag = accel_magnitudes
            .iter()
            .fold(f32::NEG_INFINITY, |a, &b| a.max(b));
        let min_mag = accel_magnitudes
            .iter()
            .fold(f32::INFINITY, |a, &b| a.min(b));

        println!(
            "  Accel magnitude: mean={:.3}g, min={:.3}g, max={:.3}g",
            mean_mag / 9.81,
            min_mag / 9.81,
            max_mag / 9.81
        );
    }

    println!("\n✅ Data inspection complete!");
    Ok(())
}
