//! Oriented Largest Empty Rectangle solver.
//!
//! Finds the largest rectangle with free orientation that fits in the free space
//! of a polygon while avoiding obstacles.

use geo_types::Polygon;
use crate::shared::Result;
use super::{LerOptions, LerResult};

/// Solve oriented LER using parallel angle sweep.
///
/// This is a placeholder implementation.
pub fn solve_ler_oriented_parallel(
    _poly: &Polygon<f64>,
    _obstacles: &[Polygon<f64>],
    _options: &LerOptions,
) -> Result<LerResult> {
    Err(crate::shared::LirError::NotSupported("LER oriented not yet implemented".to_string()))
}

/// Solve oriented LER using coarse-to-fine refinement.
///
/// This is a placeholder implementation.
pub fn solve_ler_oriented_refine(
    _poly: &Polygon<f64>,
    _obstacles: &[Polygon<f64>],
    _options: &LerOptions,
) -> Result<LerResult> {
    Err(crate::shared::LirError::NotSupported("LER oriented refine not yet implemented".to_string()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo_types::{coord, LineString};

    fn sample_polygon() -> Polygon<f64> {
        Polygon::new(
            LineString::from(vec![
                coord! {x:0.0, y:0.0},
                coord! {x:10.0, y:0.0},
                coord! {x:10.0, y:10.0},
                coord! {x:0.0, y:10.0},
                coord! {x:0.0, y:0.0},
            ]),
            vec![],
        )
    }

    #[test]
    fn placeholder_not_implemented() {
        let poly = sample_polygon();
        let result = solve_ler_oriented_parallel(&poly, &[], &LerOptions::default());
        assert!(result.is_err());
    }
}