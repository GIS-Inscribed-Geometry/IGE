//! Core algorithm trait for LIR solvers.
//!
//! All solver implementations must implement this trait.

use geo_types::Polygon;
use crate::shared::{AlgorithmCategory, AlgorithmPrecision, AlgorithmSpeed, PolygonType, SolverOptions, Result, Rectangle};

pub trait LirAlgorithm: Send + Sync {
    fn name(&self) -> &'static str;
    fn category(&self) -> AlgorithmCategory;
    fn precision(&self) -> AlgorithmPrecision;
    fn speed(&self) -> AlgorithmSpeed;
    fn polygon_type(&self) -> Option<PolygonType>;
    fn solve(&self, polygon: &Polygon<f64>, options: &SolverOptions) -> Result<Rectangle>;
}