#[cfg(feature = "shewchuk")]
pub mod common;
pub mod ler;
pub mod lir;
pub mod lir_obstacles;
pub mod mic;
pub mod nested;
mod nested_obstacles;
pub mod obb;

pub use lir::axis_aligned::solve_axis_rect_fine;
