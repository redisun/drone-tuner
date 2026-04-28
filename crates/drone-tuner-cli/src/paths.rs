//! Central data directory layout for the CLI.
//!
//! Everything the tool generates lives under one root so users can find,
//! back up, or wipe drone-tuner state with a single path. Layout:
//!
//! ```text
//! $XDG_DATA_HOME/drone-tuner/   (or $HOME/.local/share/drone-tuner/)
//! ├── backups/        tune-backup-<craft>-<ts>.json (auto-saved before writes)
//! ├── baselines/      pinned per-craft anchors (user-curated)
//! ├── pulls/          dataflash .bbl files pulled from FCs
//! └── history.jsonl   cross-tune history for the convergence detector
//! ```
//!
//! `history.jsonl` already lives at `<root>/history.jsonl` via
//! `history::history_path()` — this module shares the same `<root>` and
//! adds the three companion subdirectories.
//!
//! Explicit `--backup <PATH>` / `--keep-bbl <PATH>` flags still override
//! the defaults so one-off runs can land anywhere.

use anyhow::{Context, Result};
use std::fs;
use std::path::PathBuf;

/// Resolve `$XDG_DATA_HOME/drone-tuner` (or `$HOME/.local/share/drone-tuner`).
/// Does *not* create the directory — callers ask for the specific subdir.
pub fn data_root() -> Result<PathBuf> {
    let base = std::env::var_os("XDG_DATA_HOME")
        .map(PathBuf::from)
        .or_else(|| {
            std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".local").join("share"))
        })
        .context(
            "Cannot resolve a data directory: neither $XDG_DATA_HOME nor $HOME is set",
        )?;
    Ok(base.join("drone-tuner"))
}

fn ensure(dir: PathBuf) -> Result<PathBuf> {
    fs::create_dir_all(&dir)
        .with_context(|| format!("Failed to create directory at {}", dir.display()))?;
    Ok(dir)
}

/// `<root>/backups/` — tune-backup snapshots written by `--backup`.
pub fn backups_dir() -> Result<PathBuf> {
    ensure(data_root()?.join("backups"))
}

/// `<root>/pulls/` — `.bbl` files pulled from a flight controller.
pub fn pulls_dir() -> Result<PathBuf> {
    ensure(data_root()?.join("pulls"))
}

/// `<root>/baselines/` — user-curated per-craft baseline anchors.
/// Currently nothing writes here automatically; the directory is created
/// so the tool has a stable place for users to drop pinned baselines
/// (see the `--baseline` flag).
#[allow(dead_code)]
pub fn baselines_dir() -> Result<PathBuf> {
    ensure(data_root()?.join("baselines"))
}
