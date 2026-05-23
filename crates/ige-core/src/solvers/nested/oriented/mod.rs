use super::{NestedOptions, NestedResult};
use crate::shared::Result;
use geo_types::Polygon;

pub fn solve_nested_oriented_offset(
    _container: &Polygon<f64>,
    _options: &NestedOptions,
) -> Result<NestedResult> {
    Err(crate::shared::LirError::NotSupported(
        "nested oriented offset not yet implemented".to_string(),
    ))
}

pub fn solve_nested_oriented_vertex(
    _container: &Polygon<f64>,
    _options: &NestedOptions,
) -> Result<NestedResult> {
    Err(crate::shared::LirError::NotSupported(
        "nested oriented vertex not yet implemented".to_string(),
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
        let result = solve_nested_oriented_offset(&poly, &NestedOptions::default());
        assert!(result.is_err());
    }
}
