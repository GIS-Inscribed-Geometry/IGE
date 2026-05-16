//! Fast-path solver for perfect rectangles (4 vertices, no holes, right angles).
//! Certifies the identity polygon directly without going through the full pipeline.
//! All other shapes fall through to the main solver.

use geo_types::Polygon;

/// Try the convex fast path. Returns `(certified_polygon, area, angle_deg, ratio)` or `None`.
pub fn maybe_fast_path(
    poly: &Polygon<f64>,
    max_ratio: f64,
    min_ratio: f64,
) -> Option<(Polygon<f64>, f64, f64, f64)> {
    let ext = &poly.exterior().0;
    let nv = if ext.len() > 1 { ext.len() - 1 } else { 0 };
    let has_holes = !poly.interiors().is_empty();

    // Rectangle (identity) -- 4 vertices, no holes
    if nv == 4 && !has_holes {
        for i in 0..4 {
            let p0 = ext[i];
            let p1 = ext[(i + 1) % 4];
            let p2 = ext[(i + 2) % 4];
            let v1 = (p1.x - p0.x, p1.y - p0.y);
            let v2 = (p2.x - p1.x, p2.y - p1.y);
            let n1 = (v1.0 * v1.0 + v1.1 * v1.1).sqrt();
            let n2 = (v2.0 * v2.0 + v2.1 * v2.1).sqrt();
            if n1 > 0.0 && n2 > 0.0 {
                let dot = v1.0 * v2.0 + v1.1 * v2.1;
                if (dot / (n1 * n2)).abs() > 1e-6 {
                    break;
                }
            }
            if i == 3 {
                let wp = ((ext[1].x - ext[0].x).powi(2) + (ext[1].y - ext[0].y).powi(2)).sqrt();
                let hp = ((ext[2].x - ext[1].x).powi(2) + (ext[2].y - ext[1].y).powi(2)).sqrt();

                let e0 = (ext[1].x - ext[0].x, ext[1].y - ext[0].y);
                let e1 = (ext[2].x - ext[1].x, ext[2].y - ext[1].y);
                let ang = if wp >= hp {
                    e0.1.atan2(e0.0).to_degrees() % 90.0
                } else {
                    e1.1.atan2(e1.0).to_degrees() % 90.0
                };

                let rect_poly = Polygon::new(
                    geo_types::LineString::from(vec![ext[0], ext[1], ext[2], ext[3], ext[0]]),
                    vec![],
                );
                if let Some((cert_poly, cert_area)) = super::certify_and_adjust(
                    poly,
                    &rect_poly,
                    max_ratio,
                    crate::tuning::CERT_EPS,
                    crate::tuning::CERT_MAX_SHRINK,
                ) {
                    let corners: Vec<_> = cert_poly.exterior().0.iter().collect();
                    let w = ((corners[1].x - corners[0].x).powi(2)
                        + (corners[1].y - corners[0].y).powi(2))
                    .sqrt();
                    let h = ((corners[2].x - corners[1].x).powi(2)
                        + (corners[2].y - corners[1].y).powi(2))
                    .sqrt();
                    let rat = if w.min(h) > 0.0 {
                        w.max(h) / w.min(h)
                    } else {
                        1.0
                    };
                    return Some((cert_poly, cert_area, ang, rat));
                }
                return None;
            }
        }
    }

    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo_types::{coord, LineString};

    #[test]
    fn rectangle_is_fast_path() {
        let poly = Polygon::new(
            LineString::from(vec![
                coord! {x:0.0, y:0.0},
                coord! {x:10.0, y:0.0},
                coord! {x:10.0, y:5.0},
                coord! {x:0.0, y:5.0},
                coord! {x:0.0, y:0.0},
            ]),
            vec![],
        );
        let result = maybe_fast_path(&poly, 0.0, 0.0);
        assert!(result.is_some());
        let (_, area, _, _) = result.unwrap();
        assert!((area - 50.0).abs() < 1.0);
    }

    #[test]
    fn rectangle_fast_path_respects_max_ratio() {
        let poly = Polygon::new(
            LineString::from(vec![
                coord! {x:0.0, y:0.0},
                coord! {x:20.0, y:0.0},
                coord! {x:20.0, y:5.0},
                coord! {x:0.0, y:5.0},
                coord! {x:0.0, y:0.0},
            ]),
            vec![],
        );
        let (_, area, _, ratio) = maybe_fast_path(&poly, 2.0, 0.0).unwrap();
        assert!(area > 45.0 && area < 55.0, "area={area}");
        assert!(ratio <= 2.0 + 1e-9, "ratio={ratio}");
    }

    #[test]
    fn complex_shape_not_fast_path() {
        let poly = Polygon::new(
            LineString::from(vec![
                coord! {x:0.0, y:0.0},
                coord! {x:10.0, y:0.0},
                coord! {x:10.0, y:3.0},
                coord! {x:5.0, y:5.0},
                coord! {x:0.0, y:10.0},
                coord! {x:0.0, y:0.0},
            ]),
            vec![],
        );
        assert!(maybe_fast_path(&poly, 0.0, 0.0).is_none());
    }
}
