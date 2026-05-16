//! Oriented Bounding Box (OBB) solver.
//!
//! Finds the minimal area oriented bounding box that encloses a polygon.
//! Unlike LIR which finds the largest rectangle INSIDE a polygon,
//! OBB finds the smallest bounding box that CONTAINS the polygon.
//!
//! This is useful for collision detection, packing, and shape analysis.

pub mod solver;

use crate::shared::Result;
use geo_types::{Point, Polygon};

/// Configuration for OBB solvers.
#[derive(Debug, Clone)]
pub struct ObbOptions {
    /// Max aspect ratio (longer/shorter side); 0.0 = unconstrained.
    pub max_ratio: f64,
    /// Min aspect ratio; 0.0 = unconstrained.
    pub min_ratio: f64,
    /// Number of angle samples for search.
    pub angle_samples: usize,
    /// If true, use PCA for initial angle guess.
    pub use_pca: bool,
    /// If true, enable refinement after initial find.
    pub use_refinement: bool,
    /// Convergence tolerance for refinement (degrees).
    pub xatol_deg: f64,
}

impl Default for ObbOptions {
    fn default() -> Self {
        Self {
            max_ratio: 0.0,
            min_ratio: 0.0,
            angle_samples: 90,
            use_pca: true,
            use_refinement: true,
            xatol_deg: 0.1,
        }
    }
}

/// Result of an OBB solve.
#[derive(Debug, Clone)]
pub struct ObbResult {
    /// The oriented bounding box as a polygon.
    pub polygon: Option<Polygon<f64>>,
    /// Area of the bounding box.
    pub area: f64,
    /// Perimeter of the bounding box.
    pub perimeter: f64,
    /// Rotation angle in degrees.
    pub angle_deg: f64,
    /// Width of the bounding box.
    pub width: f64,
    /// Height of the bounding box.
    pub height: f64,
    /// Centroid of the bounding box.
    pub centroid: Option<Point<f64>>,
    /// Aspect ratio (width/height or height/width, whichever is larger).
    pub aspect_ratio: f64,
    /// Fill ratio (polygon area / OBB area).
    pub fill_ratio: f64,
}

impl ObbResult {
    pub fn empty() -> Self {
        Self {
            polygon: None,
            area: 0.0,
            perimeter: 0.0,
            angle_deg: 0.0,
            width: 0.0,
            height: 0.0,
            centroid: None,
            aspect_ratio: 1.0,
            fill_ratio: 0.0,
        }
    }
}

impl Default for ObbResult {
    fn default() -> Self {
        Self::empty()
    }
}

/// Solve for the minimal oriented bounding box.
///
/// # Arguments
/// * `poly` - Input polygon
/// * `options` - Solver configuration
///
/// # Returns
/// An `ObbResult` with the minimal bounding box.
pub fn solve_obb(_poly: &Polygon<f64>, _options: &ObbOptions) -> Result<ObbResult> {
    Err(crate::shared::LirError::NotSupported(
        "OBB not yet implemented".to_string(),
    ))
}

/// Solve for minimal OBB with aspect ratio constraints.
///
/// # Arguments
/// * `poly` - Input polygon
/// * `options` - Solver configuration (max_ratio and min_ratio will be applied)
///
/// # Returns
/// An `ObbResult` with the constrained bounding box.
pub fn solve_obb_constrained(_poly: &Polygon<f64>, _options: &ObbOptions) -> Result<ObbResult> {
    Err(crate::shared::LirError::NotSupported(
        "OBB constrained not yet implemented".to_string(),
    ))
}
