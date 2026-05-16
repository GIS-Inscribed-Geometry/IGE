//! Convex polygon nesting solver.
//!
//! Solves the largest convex polygon inside a convex container.
//! This is simpler than the general case.

use super::{NestingOptions, NestingResult};
use crate::shared::Result;
use geo_types::Polygon;

/// Solve convex nesting using polygon offset approach.
///
/// This is a placeholder implementation.
pub fn solve_nesting_convex_offset(
    _container: &Polygon<f64>,
    _options: &NestingOptions,
) -> Result<NestingResult> {
    Err(crate::shared::LirError::NotSupported(
        "Nesting convex not yet implemented".to_string(),
    ))
}

/// Solve convex nesting using vertex insertion approach.
///
/// This is a placeholder implementation.
pub fn solve_nesting_convex_vertex(
    _container: &Polygon<f64>,
    _options: &NestingOptions,
) -> Result<NestingResult> {
    Err(crate::shared::LirError::NotSupported(
        "Nesting convex vertex not yet implemented".to_string(),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo_types::{coord, LineString};

    fn sample_convex_polygon() -> Polygon<f64> {
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
        let poly = sample_convex_polygon();
        let result = solve_nesting_convex_offset(&poly, &NestingOptions::default());
        assert!(result.is_err());
    }
}
