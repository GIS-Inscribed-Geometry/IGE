//! Combined LER + LIR solver.
//!
//! Solves both Largest Empty Rectangle and Largest Inscribed Rectangle
//! in a single pass, which can be more efficient than running them separately.

pub mod combined;

use crate::shared::{Rectangle, Result};
use geo_types::Polygon;

/// Configuration for combined LER + LIR solvers.
#[derive(Debug, Clone)]
pub struct LerLirOptions {
    /// Max aspect ratio for rectangles (longer/shorter side); 0.0 = unconstrained.
    pub max_ratio: f64,
    /// Min aspect ratio for rectangles; 0.0 = unconstrained.
    pub min_ratio: f64,
    /// Grid resolution for coarse search.
    pub grid_coarse: usize,
    /// Number of top candidates to refine.
    pub top_k: usize,
    /// If true, return best-effort result even if certification fails.
    pub always_return: bool,
    /// If true, solve for axis-aligned rectangles only.
    pub axis_aligned_only: bool,
}

impl Default for LerLirOptions {
    fn default() -> Self {
        Self {
            max_ratio: 0.0,
            min_ratio: 0.0,
            grid_coarse: 60,
            top_k: 5,
            always_return: true,
            axis_aligned_only: false,
        }
    }
}

/// Result of a combined LER + LIR solve.
#[derive(Debug, Clone)]
pub struct LerLirResult {
    /// LIR: Largest Inscribed Rectangle (axis-aligned bounding box).
    pub lir_rect: Option<Rectangle>,
    /// LIR: The inscribed rectangle as a polygon.
    pub lir_polygon: Option<Polygon<f64>>,
    /// LIR: Area of the inscribed rectangle.
    pub lir_area: f64,
    /// LIR: Rotation angle in degrees.
    pub lir_angle_deg: f64,

    /// LER: Largest Empty Rectangle (axis-aligned bounding box).
    pub ler_rect: Option<Rectangle>,
    /// LER: The empty rectangle as a polygon.
    pub ler_polygon: Option<Polygon<f64>>,
    /// LER: Area of the empty rectangle.
    pub ler_area: f64,
    /// LER: Rotation angle in degrees.
    pub ler_angle_deg: f64,

    /// True if LIR result is best-effort.
    pub lir_best_effort: bool,
    /// True if LER result is best-effort.
    pub ler_best_effort: bool,
}

impl LerLirResult {
    pub fn empty() -> Self {
        Self {
            lir_rect: None,
            lir_polygon: None,
            lir_area: 0.0,
            lir_angle_deg: 0.0,
            ler_rect: None,
            ler_polygon: None,
            ler_area: 0.0,
            ler_angle_deg: 0.0,
            lir_best_effort: false,
            ler_best_effort: false,
        }
    }
}

impl Default for LerLirResult {
    fn default() -> Self {
        Self::empty()
    }
}

/// Solve combined LER + LIR using parallel approach.
///
/// # Arguments
/// * `poly` - Input polygon
/// * `obstacles` - Optional obstacle polygons for LER
/// * `options` - Solver configuration
///
/// # Returns
/// A `LerLirResult` with both LER and LIR results.
pub fn solve_ler_lir(
    _poly: &Polygon<f64>,
    _obstacles: &[Polygon<f64>],
    _options: &LerLirOptions,
) -> Result<LerLirResult> {
    Err(crate::shared::LirError::NotSupported(
        "LER+LIR combined not yet implemented".to_string(),
    ))
}

/// Solve combined LER + LIR axis-aligned only.
///
/// This is simpler and faster than the oriented version.
///
/// # Arguments
/// * `poly` - Input polygon
/// * `obstacles` - Optional obstacle polygons for LER
/// * `options` - Solver configuration
///
/// # Returns
/// A `LerLirResult` with both LER and LIR results.
pub fn solve_ler_lir_axis_aligned(
    _poly: &Polygon<f64>,
    _obstacles: &[Polygon<f64>],
    _options: &LerLirOptions,
) -> Result<LerLirResult> {
    Err(crate::shared::LirError::NotSupported(
        "LER+LIR axis-aligned not yet implemented".to_string(),
    ))
}
