//! Standard prelude for LIRiAP

#[allow(unused_imports)]
pub use crate::shared::{
    rotate_polygon, AlgorithmCategory, AlgorithmPrecision, AlgorithmSpeed, LirError, PolygonType,
    Rectangle, Result, SolverOptions,
};
#[allow(unused_imports)]
pub use crate::solvers::lir::axis_aligned::solve_vertex_grid;
#[allow(unused_imports)]
pub use crate::solvers::mic::{
    maximum_inscribed_circle, maximum_inscribed_circle_multipolygon, MicEngine, MicError,
    MicOptions, MicResult, MicUsedEngine, RobustMode,
};

#[cfg(feature = "gpu")]
#[allow(unused_imports)]
pub use crate::gpu;
