//! Standard prelude for LIRiAP

pub use crate::shared::{
    rotate_polygon, AlgorithmCategory, AlgorithmPrecision, AlgorithmSpeed, LirError, PolygonType,
    Rectangle, Result, SolverOptions,
};
pub use crate::solvers::lir::axis_aligned::solve_vertex_grid;
pub use crate::solvers::mic::{
    maximum_inscribed_circle, maximum_inscribed_circle_multipolygon, MicEngine, MicError,
    MicOptions, MicResult, MicUsedEngine, RobustMode,
};

#[cfg(feature = "gpu")]
pub use crate::gpu;
