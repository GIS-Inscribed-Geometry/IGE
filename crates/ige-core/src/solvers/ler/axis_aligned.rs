//! Axis-aligned Largest Empty Rectangle solver.
//!
//! Finds the largest axis-aligned rectangle that fits in the free space
//! of a polygon while avoiding obstacles.

use geo_types::Polygon;
use crate::shared::Result;
use super::{LerOptions, LerResult};

/// Solve axis-aligned LER using vertex-grid approach.
///
/// This is a placeholder implementation.
pub fn solve_ler_axis_aligned_grid(
    _poly: &Polygon<f64>,
    _obstacles: &[Polygon<f64>],
    _options: &LerOptions,
) -> Result<LerResult> {
    Err(crate::shared::LirError::NotSupported("LER axis-aligned not yet implemented".to_string()))
}

/// Solve axis-aligned LER using exact approach.
///
/// This is a placeholder implementation.
pub fn solve_ler_axis_aligned_exact(
    _poly: &Polygon<f64>,
    _obstacles: &[Polygon<f64>],
    _options: &LerOptions,
) -> Result<LerResult> {
    Err(crate::shared::LirError::NotSupported("LER axis-aligned exact not yet implemented".to_string()))
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
        let result = solve_ler_axis_aligned_grid(&poly, &[], &LerOptions::default());
        assert!(result.is_err());
    }
}