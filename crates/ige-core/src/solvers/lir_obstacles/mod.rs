pub mod axis_aligned;
pub mod combined;
pub mod oriented;

use crate::shared::{ObstacleInput, Rectangle, Result};
use crate::tuning;
use geo_types::Polygon;

#[derive(Debug, Clone)]
pub struct LirObstaclesOptions {
    pub max_ratio: f64,
    pub min_ratio: f64,
    pub grid_coarse: usize,
    pub top_k: usize,
    pub always_return: bool,
    pub axis_aligned_only: bool,
    pub line_thickness: f64,
    pub cert_eps: f64,
    pub cert_max_shrink: f64,
}

impl Default for LirObstaclesOptions {
    fn default() -> Self {
        Self {
            max_ratio: 0.0,
            min_ratio: 0.0,
            grid_coarse: tuning::GRID_COARSE,
            top_k: tuning::TOP_K,
            always_return: true,
            axis_aligned_only: false,
            line_thickness: 0.0,
            cert_eps: tuning::CERT_EPS,
            cert_max_shrink: tuning::CERT_MAX_SHRINK,
        }
    }
}

#[derive(Debug, Clone)]
pub struct LirObstaclesResult {
    pub rect: Option<Rectangle>,
    pub rect_polygon: Option<Polygon<f64>>,
    pub area: f64,
    pub angle_deg: f64,
    pub best_effort: bool,
}

impl LirObstaclesResult {
    pub fn empty() -> Self {
        Self {
            rect: None,
            rect_polygon: None,
            area: 0.0,
            angle_deg: 0.0,
            best_effort: false,
        }
    }
}

impl Default for LirObstaclesResult {
    fn default() -> Self {
        Self::empty()
    }
}

pub fn solve_lir_obstacles(
    poly: &Polygon<f64>,
    obstacles: &[ObstacleInput],
    options: &LirObstaclesOptions,
) -> Result<LirObstaclesResult> {
    if options.axis_aligned_only {
        return axis_aligned::solve_lir_obstacles_axis_aligned(poly, obstacles, options);
    }
    oriented::solve_lir_obstacles_oriented(poly, obstacles, options)
}

pub fn solve_lir_obstacles_axis_aligned(
    poly: &Polygon<f64>,
    obstacles: &[ObstacleInput],
    options: &LirObstaclesOptions,
) -> Result<LirObstaclesResult> {
    axis_aligned::solve_lir_obstacles_axis_aligned(poly, obstacles, options)
}

pub fn solve_lir_obstacles_oriented(
    poly: &Polygon<f64>,
    obstacles: &[ObstacleInput],
    options: &LirObstaclesOptions,
) -> Result<LirObstaclesResult> {
    oriented::solve_lir_obstacles_oriented(poly, obstacles, options)
}
