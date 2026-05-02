use geo::Contains;
use geo_types::{Point, Polygon};

use crate::mic::input::HostPolygon;

/// Point-in-polygon accelerator for MIC candidate filtering.
#[derive(Debug, Clone)]
pub struct PipIndex {
    polygon: Polygon<f64>,
}

impl PipIndex {
    pub fn new(host: &HostPolygon) -> Self {
        Self {
            polygon: host.polygon.clone(),
        }
    }

    pub fn contains_strict_xy(&self, x: f64, y: f64) -> bool {
        self.polygon.contains(&Point::new(x, y))
    }
}
