//! Inscribed Geometry Engine (IGE) - Largest Inscribed Rectangle algorithms

pub mod algorithms;
pub mod axis_aligned;
pub mod cpu;
pub mod geometry;
pub mod shared;
pub mod tuning;

#[cfg(feature = "gpu")]
pub mod gpu;
pub mod bcrs;

pub use cpu::{solve_bcrs_parallel, solve_oriented_lir, solve_axis_aligned, AxisAlignedOptions, Rectangle, SolverOptions, detect_polygon_type, rotate_polygon};

pub use shared::{PolygonType, LirError, Result};
pub use shared::{AlgorithmCategory, AlgorithmPrecision, AlgorithmSpeed, SolverBackend};