//! Basic usage example for the FPV drone tuning platform.

use drone_tuner_core::{AnalysisEngine, BlackboxParser, Result};
use std::path::Path;

fn main() -> Result<()> {
    // Initialize logging
    tracing_subscriber::fmt::init();

    // Example usage: analyze a blackbox file
    let example_file = Path::new("examples/sample_flight.bbl");
    
    if !example_file.exists() {
        println!("Sample flight file not found at: {}", example_file.display());
        println!("This example requires a blackbox file to analyze.");
        println!("You can:");
        println!("  1. Copy a real blackbox file to examples/sample_flight.bbl");
        println!("  2. Use the CLI tool to analyze your own files");
        return Ok(());
    }

    println!("FPV Drone Tuning Analysis Example");
    println!("=====================================");
    
    // Read the blackbox file
    println!("📖 Reading blackbox file: {}", example_file.display());
    let data = std::fs::read(example_file)?;
    println!("   File size: {} KB", data.len() / 1024);
    
    // Parse the blackbox data
    println!("Parsing blackbox data...");
    let mut parser = BlackboxParser::new();
    let session = parser.parse_file(&data)?;
    
    println!("   Flight duration: {:.1} seconds", session.metadata.duration_ms as f32 / 1000.0);
    println!("   Sample rate: {:.0} Hz", session.telemetry.sample_rate);
    println!("   Total samples: {}", session.telemetry.gyro.len());
    println!("   Firmware: {} {}", 
             session.metadata.hardware.flight_controller.firmware,
             session.metadata.hardware.flight_controller.version);
    
    // Perform analysis
    println!("Analyzing flight data...");
    let mut engine = AnalysisEngine::new();
    let report = engine.analyze(&session)?;
    
    // Display results
    println!("\nAnalysis Results");
    println!("==================");
    
    println!("Tune Quality Score: {:.1}/100", report.tune_quality_score);
    
    if report.tune_quality_score >= 80.0 {
        println!("   Excellent tune quality!");
    } else if report.tune_quality_score >= 60.0 {
        println!("    Good tune, but could be improved");
    } else {
        println!("   ❌ Tune needs significant improvement");
    }
    
    // Show detected issues
    if !report.detected_issues.is_empty() {
        println!("\n🚨 Issues Detected:");
        for (i, issue) in report.detected_issues.iter().enumerate() {
            let severity_icon = match issue.severity {
                drone_tuner_core::domain::Severity::Critical => "",
                drone_tuner_core::domain::Severity::High => "🟠",
                drone_tuner_core::domain::Severity::Medium => "",
                drone_tuner_core::domain::Severity::Low => "",
            };
            println!("   {}. {} {}", i + 1, severity_icon, issue.description);
        }
    } else {
        println!("\nNo significant issues detected!");
    }
    
    // Show filter recommendations
    if !report.filter_recommendations.is_empty() {
        println!("\nFilter Recommendations:");
        for (i, rec) in report.filter_recommendations.iter().enumerate() {
            let priority_icon = match rec.priority {
                drone_tuner_core::domain::Priority::Critical => "",
                drone_tuner_core::domain::Priority::High => "🟠",
                drone_tuner_core::domain::Priority::Medium => "",
                drone_tuner_core::domain::Priority::Low => "",
            };
            println!("   {}. {} {:?} at {:.1} Hz", 
                    i + 1, priority_icon, rec.recommendation_type, rec.frequency);
            println!("      Expected: {}", rec.expected_improvement);
        }
    }
    
    // Show PID recommendations
    if !report.pid_recommendations.is_empty() {
        println!("\n PID Recommendations:");
        for (i, rec) in report.pid_recommendations.iter().enumerate() {
            let priority_icon = match rec.priority {
                drone_tuner_core::domain::Priority::Critical => "",
                drone_tuner_core::domain::Priority::High => "🟠",
                drone_tuner_core::domain::Priority::Medium => "",
                drone_tuner_core::domain::Priority::Low => "",
            };
            println!("   {}. {} {:?} {:?}: {:.1} → {:.1}", 
                    i + 1, priority_icon, rec.axis, rec.term, 
                    rec.current_value, rec.recommended_value);
            println!("      Reason: {}", rec.reason);
        }
    }
    
    // Show frequency analysis summary
    println!("\nFrequency Analysis:");
    println!("   Spectral peaks found: {}", report.frequency_analysis.peaks.len());
    println!("   Noise floor: {:.3}", report.frequency_analysis.noise_floor);
    
    if !report.frequency_analysis.peaks.is_empty() {
        println!("   Top frequency peaks:");
        for (i, peak) in report.frequency_analysis.peaks.iter().take(5).enumerate() {
            println!("     {}. {:.1} Hz (amplitude: {:.2})", 
                    i + 1, peak.frequency, peak.amplitude);
        }
    }
    
    // Show confidence scores
    println!("\nConfidence Scores:");
    println!("   Overall: {:.1}%", report.confidence_scores.overall * 100.0);
    println!("   Oscillation detection: {:.1}%", 
             report.confidence_scores.oscillation_detection * 100.0);
    println!("   Filter recommendations: {:.1}%", 
             report.confidence_scores.filter_recommendations * 100.0);
    println!("   PID recommendations: {:.1}%", 
             report.confidence_scores.pid_recommendations * 100.0);
    
    println!("\nAnalysis complete! Use the recommendations above to improve your tune.");
    
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_example_compiles() {
        // This test just ensures the example compiles correctly
        // Real testing would require sample data files
        assert!(true);
    }
}