//! Nesting - Largest polygon inside polygon.
//!
//! Finds the largest polygon that can be inscribed within a given polygon.
//! This is useful for nesting problems in manufacturing and layout.

pub mod convex;
pub mod general;

use crate::shared::Result;
use geo_types::Polygon;

/// Configuration for nesting solvers.
#[derive(Debug, Clone)]
pub struct NestingOptions {
    /// Max aspect ratio for bounding box (longer/shorter side); 0.0 = unconstrained.
    pub max_ratio: f64,
    /// Min aspect ratio for bounding box; 0.0 = unconstrained.
    pub min_ratio: f64,
    /// Max vertices in output polygon (simplification).
    pub max_vertices: usize,
    /// Grid resolution for coarse search.
    pub grid_coarse: usize,
    /// If true, prefer convex solutions.
    pub prefer_convex: bool,
}

impl Default for NestingOptions {
    fn default() -> Self {
        Self {
            max_ratio: 0.0,
            min_ratio: 0.0,
            max_vertices: 100,
            grid_coarse: 60,
            prefer_convex: true,
        }
    }
}

/// Result of a nesting solve.
#[derive(Debug, Clone)]
pub struct NestingResult {
    /// The largest inscribed polygon.
    pub polygon: Option<Polygon<f64>>,
    /// Area of the inscribed polygon.
    pub area: f64,
    /// Centroid of the inscribed polygon.
    pub centroid: Option<geo_types::Point<f64>>,
    /// Fill ratio (area of inscribed / area of container).
    pub fill_ratio: f64,
}

impl NestingResult {
    pub fn empty() -> Self {
        Self {
            polygon: None,
            area: 0.0,
            centroid: None,
            fill_ratio: 0.0,
        }
    }
}

impl Default for NestingResult {
    fn default() -> Self {
        Self::empty()
    }
}

/// Solve largest polygon inside polygon (general case).
///
/// # Arguments
/// * `container` - The containing polygon
/// * `options` - Solver configuration
///
/// # Returns
/// A `NestingResult` with the largest inscribed polygon.
pub fn solve_nesting(
    _container: &Polygon<f64>,
    _options: &NestingOptions,
) -> Result<NestingResult> {
    Err(crate::shared::LirError::NotSupported(
        "Nesting not yet implemented".to_string(),
    ))
}

/// Solve largest convex polygon inside convex polygon.
///
/// This is a simpler case that can be solved more efficiently.
///
/// # Arguments
/// * `container` - The containing convex polygon
/// * `options` - Solver configuration
///
/// # Returns
/// A `NestingResult` with the largest inscribed convex polygon.
pub fn solve_nesting_convex(
    _container: &Polygon<f64>,
    _options: &NestingOptions,
) -> Result<NestingResult> {
    Err(crate::shared::LirError::NotSupported(
        "Nesting convex not yet implemented".to_string(),
    ))
}
