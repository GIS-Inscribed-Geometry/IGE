//! Oriented Largest Empty Rectangle solver.
//!
//! Finds the largest rectangle with free orientation that avoids all obstacle
//! points within the container's bounding box.
//!
//! Key difference from LIR: the rectangle can CROSS the container polygon walls
//! — it only needs to stay within the container's **bounding box** and contain
//! NO obstacle points in its interior.
//!
//! Pipeline:
//! 1. Generate candidate angles (edge-aligned + regular fill).
//! 2. Coarse sweep — uniform grid, mark cells with obstacle points as blocked, LRIH.
//! 3. Top-k refinement — vertex grid from obstacle coordinates, LRIH (exact).
//! 4. Return best empty rectangle.

pub mod mask;

use geo::{Area, BoundingRect, Centroid};
use geo_types::{Coord, LineString, Point, Polygon};

use crate::shared::{LirError, Rectangle, Result};
use crate::solvers::lir::axis_aligned::histogram::{lrih, lrih_vp};
use crate::solvers::lir::oriented::candidates::edge_candidate_angles;
use rayon::prelude::*;

use super::{LerOptions, LerResult};

// --- Internal candidate type ---

#[derive(Debug, Clone, Copy)]
struct LerCandidate {
    angle: f64,
    area: f64,
    rect_rot: (f64, f64, f64, f64),
}

// --- Angle generation ---

fn generate_angles(container: &Polygon<f64>, min_angles: usize) -> Vec<f64> {
    let mut angles = edge_candidate_angles(container, 4.0, 12);

    if angles.len() < min_angles {
        let step = 5usize.max(1);
        for step_deg in (step..90).step_by(step) {
            let a = step_deg as f64;
            if !angles.iter().any(|&ea| (ea - a).abs() < 0.5) {
                angles.push(a);
            }
        }
    }
    angles.sort_by(|a, b| a.partial_cmp(b).unwrap());
    angles.dedup_by(|a, b| (*a - *b).abs() < 0.1);
    angles
}

// --- Coarse sweep ---

fn coarse_evaluate_angle(
    obs_points: &[Coord<f64>],
    bbox: &RotatedBbox,
    angle: f64,
    coarse_steps: usize,
    max_ratio: f64,
    min_ratio: f64,
) -> Option<LerCandidate> {
    let (minx, miny, maxx, maxy) = bbox.extent;
    if maxx <= minx || maxy <= miny || coarse_steps < 2 {
        return None;
    }

    let mut xs = Vec::with_capacity(coarse_steps);
    let mut ys = Vec::with_capacity(coarse_steps);
    for i in 0..coarse_steps {
        let t = i as f64 / (coarse_steps - 1) as f64;
        xs.push(minx + (maxx - minx) * t);
        ys.push(miny + (maxy - miny) * t);
    }

    let free_mask = mask::build_free_mask(obs_points, &xs, &ys, Some(&bbox.quad_corners));
    let n_cols = xs.len().saturating_sub(1);
    let n_rows = ys.len().saturating_sub(1);
    if n_cols == 0 || n_rows == 0 {
        return None;
    }

    let mut heights = vec![0usize; n_cols];
    let mut best_local: Option<(f64, f64, f64, f64, f64)> = None;

    for r in 0..n_rows {
        let base = r * n_cols;
        for c in 0..n_cols {
            if free_mask[base + c] {
                heights[c] += 1;
            } else {
                heights[c] = 0;
            }
        }
        let (x0, y0, x1, y1, area) = lrih(&heights, &xs, &ys, r, max_ratio, min_ratio);
        if area > 0.0 {
            best_local = match best_local {
                Some((_, _, _, _, a)) if area > a => Some((x0, y0, x1, y1, area)),
                None => Some((x0, y0, x1, y1, area)),
                _ => best_local,
            };
        }
    }

    best_local.map(|(x0, y0, x1, y1, area)| LerCandidate {
        angle,
        area,
        rect_rot: (x0, y0, x1, y1),
    })
}

// --- Fine solve ---

fn fine_solve_angle(
    obs_points: &[Coord<f64>],
    bbox: &RotatedBbox,
    coarse: &LerCandidate,
    max_ratio: f64,
    min_ratio: f64,
) -> Option<LerCandidate> {
    let (minx, miny, maxx, maxy) = bbox.extent;
    let span_x = maxx - minx;
    let span_y = maxy - miny;
    if span_x <= 0.0 || span_y <= 0.0 {
        return Some(*coarse);
    }

    // Collect unique x/y from obstacle points + bbox extremes
    let mut xs_raw: Vec<f64> = obs_points.iter().map(|c| c.x).collect();
    let mut ys_raw: Vec<f64> = obs_points.iter().map(|c| c.y).collect();
    xs_raw.push(minx);
    xs_raw.push(maxx);
    ys_raw.push(miny);
    ys_raw.push(maxy);

    xs_raw.sort_by(|a, b| a.partial_cmp(b).unwrap());
    ys_raw.sort_by(|a, b| a.partial_cmp(b).unwrap());
    xs_raw.dedup_by(|a, b| (*a - *b).abs() < 1e-14);
    ys_raw.dedup_by(|a, b| (*a - *b).abs() < 1e-14);

    // Ensure minimum grid resolution (at least 16 cells per dimension)
    // This prevents overestimating area when the fine grid has too few cells
    // (e.g. zero obstacle points means only 2 xs/ys → 1 giant cell).
    const MIN_FINE_CELLS: usize = 16;
    if xs_raw.len() <= MIN_FINE_CELLS {
        let n_extra = MIN_FINE_CELLS + 1 - xs_raw.len();
        for i in 0..n_extra {
            let t = (i + 1) as f64 / (n_extra + 1) as f64;
            xs_raw.push(minx + span_x * t);
        }
        xs_raw.sort_by(|a, b| a.partial_cmp(b).unwrap());
        xs_raw.dedup_by(|a, b| (*a - *b).abs() < 1e-14);
    }
    if ys_raw.len() <= MIN_FINE_CELLS {
        let n_extra = MIN_FINE_CELLS + 1 - ys_raw.len();
        for i in 0..n_extra {
            let t = (i + 1) as f64 / (n_extra + 1) as f64;
            ys_raw.push(miny + span_y * t);
        }
        ys_raw.sort_by(|a, b| a.partial_cmp(b).unwrap());
        ys_raw.dedup_by(|a, b| (*a - *b).abs() < 1e-14);
    }

    let n_cols = xs_raw.len().saturating_sub(1);
    let n_rows = ys_raw.len().saturating_sub(1);
    if n_cols < 1 || n_rows < 1 {
        return None;
    }

    let free_mask = mask::build_free_mask(obs_points, &xs_raw, &ys_raw, Some(&bbox.quad_corners));

    let mut heights = vec![0usize; n_cols];
    let mut best_local: Option<(f64, f64, f64, f64, f64)> = None;

    // Seed with coarse candidate
    let (sx0, sy0, sx1, sy1) = coarse.rect_rot;
    if sx1 > sx0 && sy1 > sy0 {
        best_local = Some((sx0, sy0, sx1, sy1, (sx1 - sx0) * (sy1 - sy0)));
    }

    for r in 0..n_rows {
        let base = r * n_cols;
        for c in 0..n_cols {
            if free_mask[base + c] {
                heights[c] += 1;
            } else {
                heights[c] = 0;
            }
        }
        let (x0, y0, x1, y1, area) = lrih_vp(&heights, &xs_raw, &ys_raw, r, max_ratio, min_ratio);
        if area > 0.0 {
            best_local = match best_local {
                Some((_, _, _, _, a)) if area > a => Some((x0, y0, x1, y1, area)),
                None => Some((x0, y0, x1, y1, area)),
                _ => best_local,
            };
        }
    }

    let (x0, y0, x1, y1, area) = best_local?;
    Some(LerCandidate {
        angle: coarse.angle,
        area,
        rect_rot: (x0, y0, x1, y1),
    })
}

// --- Rotate utilities ---

/// Extract obstacle vertex points from obstacle polygons.
fn extract_obstacle_points(obstacles: &[Polygon<f64>]) -> Vec<Coord<f64>> {
    let mut pts = Vec::new();
    for obs in obstacles {
        for c in obs.exterior().coords() {
            pts.push(*c);
        }
        for hole in obs.interiors() {
            for c in hole.coords() {
                pts.push(*c);
            }
        }
    }
    // Remove near-duplicates
    pts.sort_by(|a, b| {
        a.x.partial_cmp(&b.x)
            .unwrap()
            .then(a.y.partial_cmp(&b.y).unwrap())
    });
    pts.dedup_by(|a, b| (a.x - b.x).abs() < 1e-12 && (a.y - b.y).abs() < 1e-12);
    pts
}

/// Rotate a set of points around the container centroid.
fn rotate_points(pts: &[Coord<f64>], centroid: Point<f64>, angle_deg: f64) -> Vec<Coord<f64>> {
    let (cx, cy) = (centroid.x(), centroid.y());
    let rad = -angle_deg.to_radians();
    let (cos_a, sin_a) = (rad.cos(), rad.sin());

    pts.iter()
        .map(|c| {
            let dx = c.x - cx;
            let dy = c.y - cy;
            Coord {
                x: cx + dx * cos_a - dy * sin_a,
                y: cy + dx * sin_a + dy * cos_a,
            }
        })
        .collect()
}

/// Bounding box of the container in the rotated frame, including the
/// axis-aligned extent and the 4 corners of the rotated quadrilateral.
struct RotatedBbox {
    /// AABB of the rotated boundary: (min_x, min_y, max_x, max_y)
    extent: (f64, f64, f64, f64),
    /// The 4 corners of the original world bounding box rotated into
    /// the rotated frame, forming the feasible quadrilateral.
    quad_corners: [Coord<f64>; 4],
}

fn rotate_bbox(poly: &Polygon<f64>, centroid: Point<f64>, angle_deg: f64) -> RotatedBbox {
    let bb = poly.bounding_rect().unwrap();
    let corners = [
        Coord {
            x: bb.min().x,
            y: bb.min().y,
        },
        Coord {
            x: bb.max().x,
            y: bb.min().y,
        },
        Coord {
            x: bb.max().x,
            y: bb.max().y,
        },
        Coord {
            x: bb.min().x,
            y: bb.max().y,
        },
    ];
    let rotated = rotate_points(&corners, centroid, angle_deg);
    let mut minx = f64::MAX;
    let mut miny = f64::MAX;
    let mut maxx = f64::MIN;
    let mut maxy = f64::MIN;
    for c in &rotated {
        if c.x < minx {
            minx = c.x
        }
        if c.x > maxx {
            maxx = c.x
        }
        if c.y < miny {
            miny = c.y
        }
        if c.y > maxy {
            maxy = c.y
        }
    }
    RotatedBbox {
        extent: (minx, miny, maxx, maxy),
        quad_corners: [rotated[0], rotated[1], rotated[2], rotated[3]],
    }
}

fn rotate_point(x: f64, y: f64, angle_deg: f64, origin: &Point<f64>) -> Coord<f64> {
    let rad = angle_deg.to_radians();
    let cos_a = rad.cos();
    let sin_a = rad.sin();
    let dx = x - origin.x();
    let dy = y - origin.y();
    Coord {
        x: origin.x() + dx * cos_a - dy * sin_a,
        y: origin.y() + dx * sin_a + dy * cos_a,
    }
}

// --- Public entry point ---

/// Solve the largest empty rectangle with free orientation.
///
/// The rectangle must stay within the container's **bounding box** and must
/// contain NO obstacle points in its interior.  It may CROSS the container
/// polygon's walls.
///
/// # Arguments
/// * `container` - Input polygon defining the free space (uses bounding box)
/// * `obstacles` - Obstacle polygons (vertex points are used as obstacle points)
/// * `options` - Solver configuration
///
/// # Returns
/// A `LerResult` with the largest empty rectangle (oriented).
pub fn solve_ler_oriented(
    container: &Polygon<f64>,
    obstacles: &[Polygon<f64>],
    options: &LerOptions,
) -> Result<LerResult> {
    if container.exterior().0.len() < 3
        || container.bounding_rect().is_none()
        || container.unsigned_area() < 1e-12
    {
        return Err(LirError::InvalidPolygon(
            "Polygon has <3 vertices or zero area".to_string(),
        ));
    }

    let centroid: Point<f64> = container
        .centroid()
        .map(|c| c.into())
        .unwrap_or(Point::new(0.0, 0.0));

    // Extract obstacle vertex points
    let obs_points = extract_obstacle_points(obstacles);

    let coarse_steps = options.grid_coarse.max(8);
    let top_k = options.top_k.max(1);

    let all_angles = generate_angles(container, 30);

    // Coarse evaluate all angles in parallel
    let mut candidates: Vec<LerCandidate> = all_angles
        .par_iter()
        .filter_map(|&angle| {
            let rot_pts = rotate_points(&obs_points, centroid, angle);
            let bbox = rotate_bbox(container, centroid, angle);
            coarse_evaluate_angle(
                &rot_pts,
                &bbox,
                angle,
                coarse_steps,
                options.max_ratio,
                options.min_ratio,
            )
        })
        .collect();

    if candidates.is_empty() {
        return Err(LirError::NoRectangleFound);
    }

    candidates.sort_by(|a, b| {
        b.area
            .partial_cmp(&a.area)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    // Refinement: +/-1 deg around top 3
    let best_angles: Vec<f64> = candidates.iter().map(|c| c.angle).take(3).collect();
    let refinement: Vec<f64> = best_angles
        .iter()
        .flat_map(|&base| vec![base - 1.0, base + 1.0])
        .filter(|&a| a >= 0.0 && a <= 90.0)
        .filter(|a| !all_angles.iter().any(|ta| (ta - a).abs() < 0.5))
        .collect();

    for &angle in &refinement {
        let rot_pts = rotate_points(&obs_points, centroid, angle);
        let bbox = rotate_bbox(container, centroid, angle);
        if let Some(c) = coarse_evaluate_angle(
            &rot_pts,
            &bbox,
            angle,
            coarse_steps,
            options.max_ratio,
            options.min_ratio,
        ) {
            candidates.push(c);
        }
    }

    candidates.sort_by(|a, b| {
        b.area
            .partial_cmp(&a.area)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    let mut seen: Vec<f64> = Vec::new();
    candidates.retain(|c| {
        if seen.iter().any(|&s| (c.angle - s).abs() < 2.0) {
            false
        } else {
            seen.push(c.angle);
            true
        }
    });

    let top_k = candidates.len().min(top_k);
    if top_k == 0 {
        return Err(LirError::NoRectangleFound);
    }

    // Fine solve top candidates in parallel
    let fine_results: Vec<Option<LerCandidate>> = candidates[..top_k]
        .par_iter()
        .map(|cand| {
            let rot_pts = rotate_points(&obs_points, centroid, cand.angle);
            let bbox = rotate_bbox(container, centroid, cand.angle);
            fine_solve_angle(&rot_pts, &bbox, cand, options.max_ratio, options.min_ratio)
        })
        .collect();

    let best = fine_results
        .into_iter()
        .flatten()
        .max_by(|a, b| {
            a.area
                .partial_cmp(&b.area)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .ok_or(LirError::NoRectangleFound)?;

    // Build result
    let raw_poly = Polygon::new(
        LineString::from(vec![
            rotate_point(best.rect_rot.0, best.rect_rot.1, best.angle, &centroid),
            rotate_point(best.rect_rot.2, best.rect_rot.1, best.angle, &centroid),
            rotate_point(best.rect_rot.2, best.rect_rot.3, best.angle, &centroid),
            rotate_point(best.rect_rot.0, best.rect_rot.3, best.angle, &centroid),
            rotate_point(best.rect_rot.0, best.rect_rot.1, best.angle, &centroid),
        ]),
        vec![],
    );

    let bb = raw_poly.bounding_rect().unwrap();
    Ok(LerResult {
        rect: Some(Rectangle {
            x_min: bb.min().x,
            y_min: bb.min().y,
            x_max: bb.max().x,
            y_max: bb.max().y,
        }),
        rect_polygon: Some(raw_poly),
        area: best.area,
        angle_deg: best.angle,
        best_effort: false,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo_types::{coord, LineString};

    fn square_container() -> Polygon<f64> {
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

    fn sample_obstacles() -> Vec<Polygon<f64>> {
        vec![Polygon::new(
            LineString::from(vec![
                coord! {x:4.0, y:4.0},
                coord! {x:6.0, y:4.0},
                coord! {x:6.0, y:6.0},
                coord! {x:4.0, y:6.0},
                coord! {x:4.0, y:4.0},
            ]),
            vec![],
        )]
    }

    #[test]
    fn no_obstacles_fills_box() {
        let poly = square_container();
        let result = solve_ler_oriented(&poly, &[], &LerOptions::default()).unwrap();
        // Without obstacles, should fill the entire 10x10 bounding box
        assert!(result.area > 95.0, "area too small: {}", result.area);
        assert!(result.rect.is_some());
    }

    #[test]
    fn central_obstacle_finds_gap() {
        let poly = square_container();
        let obs = sample_obstacles();
        let result = solve_ler_oriented(&poly, &obs, &LerOptions::default()).unwrap();
        // A 2x2 central obstacle at (4,4)-(6,6) max possible area is 40 (10x4 half)
        assert!(result.area > 38.0, "area too small: {}", result.area);
        assert!(result.rect_polygon.is_some());
    }

    #[test]
    fn multiple_obstacles() {
        let poly = square_container();
        let obs = vec![
            Polygon::new(
                LineString::from(vec![
                    coord! {x:1.0, y:1.0},
                    coord! {x:3.0, y:1.0},
                    coord! {x:3.0, y:3.0},
                    coord! {x:1.0, y:3.0},
                    coord! {x:1.0, y:1.0},
                ]),
                vec![],
            ),
            Polygon::new(
                LineString::from(vec![
                    coord! {x:7.0, y:7.0},
                    coord! {x:9.0, y:7.0},
                    coord! {x:9.0, y:9.0},
                    coord! {x:7.0, y:9.0},
                    coord! {x:7.0, y:7.0},
                ]),
                vec![],
            ),
        ];
        let result = solve_ler_oriented(&poly, &obs, &LerOptions::default()).unwrap();
        assert!(result.area > 40.0, "area too small: {}", result.area);
    }

    #[test]
    fn degenerate_polygon() {
        let flat = Polygon::new(
            LineString::from(vec![
                coord! {x:0.0, y:0.0},
                coord! {x:5.0, y:0.0},
                coord! {x:0.0, y:0.0},
            ]),
            vec![],
        );
        let result = solve_ler_oriented(&flat, &[], &LerOptions::default());
        assert!(result.is_err());
    }

    #[test]
    fn obstacle_on_boundary() {
        // Obstacle in a corner: the largest empty rect avoids it
        let poly = square_container();
        let obs = vec![Polygon::new(
            LineString::from(vec![
                coord! {x:0.0, y:0.0},
                coord! {x:3.0, y:0.0},
                coord! {x:3.0, y:3.0},
                coord! {x:0.0, y:3.0},
                coord! {x:0.0, y:0.0},
            ]),
            vec![],
        )];
        let result = solve_ler_oriented(&poly, &obs, &LerOptions::default()).unwrap();
        assert!(result.area > 40.0, "area too small: {}", result.area);
    }

    #[test]
    fn rect_avoids_vertices() {
        // Place obstacle points in the middle of each edge.
        // The largest empty rectangle should be a 5x5 centered rect rotated
        // to avoid the edge midpoints.
        let poly = Polygon::new(
            LineString::from(vec![
                coord! {x:0.0, y:0.0},
                coord! {x:10.0, y:0.0},
                coord! {x:10.0, y:10.0},
                coord! {x:0.0, y:10.0},
                coord! {x:0.0, y:0.0},
            ]),
            vec![],
        );
        // Two small obstacles near center
        let obs = vec![Polygon::new(
            LineString::from(vec![
                coord! {x:4.8, y:4.8},
                coord! {x:5.2, y:4.8},
                coord! {x:5.2, y:5.2},
                coord! {x:4.8, y:5.2},
                coord! {x:4.8, y:4.8},
            ]),
            vec![],
        )];
        let result = solve_ler_oriented(&poly, &obs, &LerOptions::default()).unwrap();
        assert!(result.area > 40.0, "area too small: {}", result.area);
    }

    #[test]
    fn max_ratio_constraint() {
        let poly = square_container();
        let mut opts = LerOptions::default();
        opts.max_ratio = 1.0;
        let result = solve_ler_oriented(&poly, &[], &opts).unwrap();
        if let Some(rect) = &result.rect {
            let w = rect.x_max - rect.x_min;
            let h = rect.y_max - rect.y_min;
            let ratio = w.max(h) / w.min(h);
            assert!(ratio <= 1.02, "ratio {} > 1.0", ratio);
        }
    }

    #[test]
    fn rectangle_crosses_polygon_edges() {
        // Arrowhead polygon — narrower at the base, wider at the tip.
        // The largest empty rect should be close to the bounding box size,
        // crossing the polygon edges.
        let arrow = Polygon::new(
            LineString::from(vec![
                coord! {x:0.0, y:0.0},
                coord! {x:10.0, y:5.0},
                coord! {x:0.0, y:10.0},
                coord! {x:0.0, y:0.0},
            ]),
            vec![],
        );
        let result = solve_ler_oriented(&arrow, &[], &LerOptions::default()).unwrap();
        // Bounding box is 10x10=100. With no obstacles, rect should be ~100.
        assert!(
            result.area > 90.0,
            "area={} should fill most of bounding box",
            result.area
        );
    }

    #[test]
    fn square_no_obstacles_exact_coords() {
        let poly = square_container();
        let result = solve_ler_oriented(&poly, &[], &LerOptions::default()).unwrap();
        // Without obstacles, should fill 10x10 bbox at angle 0°
        assert_eq!(result.angle_deg, 0.0, "angle should be 0");
        assert!(
            (result.area - 100.0).abs() < 1.0,
            "area should be ~100, got {}",
            result.area
        );
        if let Some(rp) = &result.rect_polygon {
            let coords: Vec<_> = rp.exterior().coords().collect();
            assert!((coords[0].x - 0.0).abs() < 1e-6, "x0={}", coords[0].x);
            assert!((coords[0].y - 0.0).abs() < 1e-6, "y0={}", coords[0].y);
            assert!((coords[1].x - 10.0).abs() < 1e-6, "x1={}", coords[1].x);
            assert!((coords[1].y - 0.0).abs() < 1e-6, "y1={}", coords[1].y);
            assert!((coords[2].x - 10.0).abs() < 1e-6, "x2={}", coords[2].x);
            assert!((coords[2].y - 10.0).abs() < 1e-6, "y2={}", coords[2].y);
            assert!((coords[3].x - 0.0).abs() < 1e-6, "x3={}", coords[3].x);
            assert!((coords[3].y - 10.0).abs() < 1e-6, "y3={}", coords[3].y);
        }
    }

    #[test]
    fn l_shape_no_obstacles_fills_bbox() {
        let lshape = Polygon::new(
            LineString::from(vec![
                coord! {x:0.0, y:0.0},
                coord! {x:10.0, y:0.0},
                coord! {x:10.0, y:3.0},
                coord! {x:3.0, y:3.0},
                coord! {x:3.0, y:10.0},
                coord! {x:0.0, y:10.0},
                coord! {x:0.0, y:0.0},
            ]),
            vec![],
        );
        // No obstacles — should fill the 10x10 bounding box (area ~100)
        let result = solve_ler_oriented(&lshape, &[], &LerOptions::default()).unwrap();
        assert!(
            result.area > 90.0,
            "area={} should fill bounding box",
            result.area
        );
    }
}
