//! CPU-based solvers for oriented largest inscribed rectangles.

use geo_types::Polygon;

pub use crate::shared::{Rectangle, SolverOptions};

pub use crate::axis_aligned::{
    AxisAlignedOptions,
    detect_polygon_type,
};

pub use crate::geometry::rotate_polygon;

pub use crate::bcrs::solve_bcrs;
pub use crate::bcrs::{BcrsOptions, BcrsResult};

/// Convenience wrapper: solve axis-aligned with default options.
/// For full control use `AxisAlignedOptions` with `axis_aligned::solve_vertex_grid`.
pub fn solve_oriented_lir(poly: &Polygon<f64>) -> Option<Rectangle> {
    crate::axis_aligned::solve_vertex_grid(poly, &AxisAlignedOptions::default())
}

/// Solve the largest axis-aligned rectangle in a polygon.
/// Mirrors the BCRS options pattern.
pub fn solve_axis_aligned(poly: &Polygon<f64>, options: &AxisAlignedOptions) -> Option<Rectangle> {
    crate::axis_aligned::solve_vertex_grid(poly, options)
}