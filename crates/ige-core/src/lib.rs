//! Inscribed Geometry Engine (IGE) - Largest Inscribed Rectangle algorithms

#![cfg_attr(feature = "simd", feature(portable_simd))]

pub mod algorithms;
pub mod shared;
pub mod tuning;

pub mod solvers;

#[cfg(feature = "gpu")]
pub mod gpu;
mod prelude;

pub use algorithms::LirSolver;

// LIR solvers
pub use solvers::lir::axis_aligned::{detect_polygon_type, solve_vertex_grid, AxisAlignedOptions};
pub use solvers::lir::axis_aligned::{
    solve_axis_rect_bcrs_with_backend, solve_axis_rect_grid_with_backend, MaskBackend,
};
pub use solvers::lir::oriented::parallel::solve_lir_oriented_parallel;
pub use solvers::lir::oriented::{solve_lir_oriented, LirOrientedOptions, LirOrientedResult};

// MIC solvers
pub use solvers::mic::{
    maximum_inscribed_circle, maximum_inscribed_circle_multipolygon, MicEngine, MicError,
    MicOptions, MicResult, MicUsedEngine, RobustMode,
};

// LER solvers
pub use solvers::ler::axis_aligned::ObstacleInput;
pub use solvers::ler::{
    solve_ler_axis_aligned, solve_ler_axis_aligned_mixed, solve_ler_axis_aligned_mixed_sweep,
    solve_ler_axis_aligned_points_dc, solve_ler_axis_aligned_points_sweep,
    solve_ler_axis_aligned_with_lines, solve_ler_axis_aligned_with_lines_exact, solve_ler_oriented,
    LerOptions, LerResult,
};

// Nesting solvers
pub use solvers::nesting::{solve_nesting, solve_nesting_convex, NestingOptions, NestingResult};

// LER + LIR combined solvers
pub use solvers::ler_lir::{
    solve_ler_lir, solve_ler_lir_axis_aligned, LerLirOptions, LerLirResult,
};

// OBB solvers
pub use solvers::obb::{solve_obb, solve_obb_constrained, ObbOptions, ObbResult};

pub use shared::{
    rotate_polygon, AlgorithmCategory, AlgorithmPrecision, AlgorithmSpeed, LirError, PolygonType,
    Rectangle, Result, SolverBackend, SolverOptions,
};

pub use geo_types::Polygon;

pub fn solve_oriented_lir(poly: &Polygon<f64>) -> Option<Rectangle> {
    solve_lir_oriented(poly, &LirOrientedOptions::default())
        .ok()
        .and_then(|r| r.rect)
}

pub fn solve_axis_aligned(poly: &Polygon<f64>, options: &AxisAlignedOptions) -> Option<Rectangle> {
    solve_vertex_grid(poly, options)
}
