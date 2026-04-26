//! Per-FC tune iteration log.
//!
//! Each successful `tune` writeback appends a line of JSON to
//! `~/.local/share/drone-tuner/history.jsonl` (or
//! `$XDG_DATA_HOME/drone-tuner/history.jsonl`). One file across all FCs;
//! `fc.board_id` plus `fc.target_name` partition entries by hardware.
//!
//! Format is JSONL so the file is append-only, grep-able from the shell,
//! and survives a crash mid-write without corrupting prior entries.

use std::collections::hash_map::DefaultHasher;
use std::fs::{self, OpenOptions};
use std::hash::{Hash, Hasher};
use std::io::{Read, Write};
use std::path::{Path, PathBuf};

use anyhow::{Context, Result};
use chrono::{DateTime, Utc};
use serde::Serialize;

/// One row of the tune history log.
#[derive(Debug, Serialize)]
pub struct TuneHistoryEntry<'a> {
    /// Schema tag so future readers can detect format changes.
    pub schema: &'static str,
    /// When the tune was written, in UTC.
    pub timestamp: DateTime<Utc>,
    /// Flight controller identity. `board_id` + `target_name` are the
    /// stable join key across iterations on the same hardware.
    pub fc: FcIdentity<'a>,
    /// Source blackbox file the recommendations came from.
    pub bbl: BblIdentity,
    /// PID values before the writeback.
    pub pids_before: PidTriples,
    /// PID values after the writeback.
    pub pids_after: PidTriples,
    /// How many recommendations were applied (some may have been filtered
    /// by --auto-apply-safe).
    pub recommendations_applied: usize,
    /// Whether the change was committed to non-volatile memory. If false,
    /// the FC will revert to `pids_before` on the next power cycle.
    pub persisted_to_eeprom: bool,
}

/// FC identity fields lifted from `FlightControllerInfo`.
#[derive(Debug, Serialize)]
pub struct FcIdentity<'a> {
    pub board_id: &'a str,
    pub target_name: &'a str,
    pub firmware_id: &'a str,
    pub firmware_version: &'a str,
}

/// Source bbl identity. Path is captured for cross-reference;
/// `fingerprint_hex` lets the log distinguish two .bbls with the same
/// filename without committing a real cryptographic hash dependency.
#[derive(Debug, Serialize)]
pub struct BblIdentity {
    pub path: String,
    pub size_bytes: u64,
    pub fingerprint_hex: String,
}

/// 3-axis × 3-term PID triple, mirroring `PidSnapshot::{roll,pitch,yaw}`.
#[derive(Debug, Serialize)]
pub struct PidTriples {
    pub roll: [u8; 3],
    pub pitch: [u8; 3],
    pub yaw: [u8; 3],
}

impl PidTriples {
    pub fn from_snapshot(snap: &drone_tuner_core::realtime::PidSnapshot) -> Self {
        let (rp, ri, rd) = snap.roll();
        let (pp, pi, pd) = snap.pitch();
        let (yp, yi, yd) = snap.yaw();
        Self {
            roll: [rp, ri, rd],
            pitch: [pp, pi, pd],
            yaw: [yp, yi, yd],
        }
    }
}

impl BblIdentity {
    /// Build an identity record for `path`. Uses `std::hash::DefaultHasher`
    /// over the first 16 KB of the file — enough to distinguish bbls in
    /// practice without pulling in `sha2`.
    pub fn from_path(path: &Path) -> Result<Self> {
        let metadata = fs::metadata(path)
            .with_context(|| format!("Failed to stat {} for history entry", path.display()))?;
        let mut file = fs::File::open(path)
            .with_context(|| format!("Failed to open {} for fingerprint", path.display()))?;
        let mut buf = vec![0u8; 16 * 1024];
        let n = file.read(&mut buf).unwrap_or(0);
        let mut hasher = DefaultHasher::new();
        metadata.len().hash(&mut hasher);
        buf[..n].hash(&mut hasher);
        let fp = hasher.finish();
        Ok(Self {
            path: path.display().to_string(),
            size_bytes: metadata.len(),
            fingerprint_hex: format!("{fp:016x}"),
        })
    }
}

/// Resolve the on-disk history file path. Honours `$XDG_DATA_HOME`,
/// falls back to `$HOME/.local/share`. Creates the parent directory.
pub fn history_path() -> Result<PathBuf> {
    let base = std::env::var_os("XDG_DATA_HOME")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".local").join("share")))
        .context("Cannot resolve a data directory: neither $XDG_DATA_HOME nor $HOME is set")?;
    let dir = base.join("drone-tuner");
    fs::create_dir_all(&dir)
        .with_context(|| format!("Failed to create history dir at {}", dir.display()))?;
    Ok(dir.join("history.jsonl"))
}

/// Append `entry` as one JSON line to the history file. Best-effort: a
/// failure here must never break a successful tune writeback.
pub fn append(entry: &TuneHistoryEntry) -> Result<PathBuf> {
    let path = history_path()?;
    let mut line = serde_json::to_string(entry).context("Failed to serialize history entry")?;
    line.push('\n');
    let mut f = OpenOptions::new()
        .create(true)
        .append(true)
        .open(&path)
        .with_context(|| format!("Failed to open history file {}", path.display()))?;
    f.write_all(line.as_bytes())
        .with_context(|| format!("Failed to append to {}", path.display()))?;
    Ok(path)
}
