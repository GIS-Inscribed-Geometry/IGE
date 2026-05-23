pub mod axis_aligned;
pub mod oriented;

use crate::shared::Result;
use geo_types::Polygon;

#[derive(Debug, Clone)]
pub struct NestedOptions {
    pub max_ratio: f64,
    pub min_ratio: f64,
    pub max_vertices: usize,
    pub grid_coarse: usize,
    pub prefer_convex: bool,
}

impl Default for NestedOptions {
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

#[derive(Debug, Clone)]
pub struct NestedResult {
    pub polygon: Option<Polygon<f64>>,
    pub area: f64,
    pub centroid: Option<geo_types::Point<f64>>,
    pub fill_ratio: f64,
}

impl NestedResult {
    pub fn empty() -> Self {
        Self {
            polygon: None,
            area: 0.0,
            centroid: None,
            fill_ratio: 0.0,
        }
    }
}

impl Default for NestedResult {
    fn default() -> Self {
        Self::empty()
    }
}

pub fn solve_nested(_container: &Polygon<f64>, _options: &NestedOptions) -> Result<NestedResult> {
    Err(crate::shared::LirError::NotSupported(
        "nested not yet implemented".to_string(),
    ))
}

pub fn solve_nested_convex(
    _container: &Polygon<f64>,
    _options: &NestedOptions,
) -> Result<NestedResult> {
    Err(crate::shared::LirError::NotSupported(
        "nested convex not yet implemented".to_string(),
    ))
}
