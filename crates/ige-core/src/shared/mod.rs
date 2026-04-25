//! Shared types for LIR algorithms.
//!
//! These types are used across all solver implementations.

use geo_types::{Coord, LineString, Polygon};
use thiserror::Error;

#[derive(Debug, Clone)]
pub struct Rectangle {
    pub x_min: f64,
    pub y_min: f64,
    pub x_max: f64,
    pub y_max: f64,
}

impl Rectangle {
    pub fn area(&self) -> f64 {
        (self.x_max - self.x_min) * (self.y_max - self.y_min)
    }
    
    pub fn to_polygon(&self) -> Polygon<f64> {
        Polygon::new(
            LineString::from(vec![
                Coord { x: self.x_min, y: self.y_min },
                Coord { x: self.x_max, y: self.y_min },
                Coord { x: self.x_max, y: self.y_max },
                Coord { x: self.x_min, y: self.y_max },
                Coord { x: self.x_min, y: self.y_min },
            ]),
            vec![],
        )
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PolygonType {
    ConvexNoHoles,
    ConvexWithHoles,
    ConcaveNoHoles,
    ConcaveWithHoles,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SolverBackend {
    Cpu,
    #[cfg(feature = "gpu")]
    Gpu,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AlgorithmCategory {
    AxisAligned,
    Oriented,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AlgorithmPrecision {
    Exact,
    Approx,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AlgorithmSpeed {
    Standard,
    Fast,
}

#[derive(Debug, Error)]
pub enum LirError {
    #[error("Invalid polygon: {0}")]
    InvalidPolygon(String),
    #[error("No rectangle found")]
    NoRectangleFound,
    #[error("GPU error: {0}")]
    GpuError(String),
    #[error("Algorithm not supported: {0}")]
    NotSupported(String),
    #[error("Internal error: {0}")]
    Internal(String),
}

pub type Result<T> = std::result::Result<T, LirError>;

#[derive(Debug, Clone)]
pub struct SolverOptions {
    pub rotation_degrees: f64,
    pub prefer_gpu: bool,
    pub force_cpu: bool,
    pub max_aspect_ratio: f64,
    pub gpu_threshold: usize,
}

impl Default for SolverOptions {
    fn default() -> Self {
        Self {
            rotation_degrees: 0.0,
            prefer_gpu: false,
            force_cpu: true,
            max_aspect_ratio: 0.0,
            gpu_threshold: 1000,
        }
    }
}