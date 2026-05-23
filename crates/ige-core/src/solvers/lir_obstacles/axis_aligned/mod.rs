use geo::{BoundingRect, Contains};
use geo_types::{Point, Polygon};
use ordered_float::OrderedFloat;
use std::collections::HashSet;

use super::combined::{
    build_obstacle_mask_aa, group_obstacles, rect_avoids_obstacles, shrink_away_from_points,
    shrink_away_from_polygons,
};
use super::{LirObstaclesOptions, LirObstaclesResult};
use crate::shared::{LirError, ObstacleInput, Rectangle, Result};
use crate::solvers::lir::axis_aligned::containment::contract_rect_to_boundary;
use crate::solvers::lir::axis_aligned::histogram::lrih_vp;
use crate::solvers::lir::axis_aligned::vertex_grid::compute_row_intervals;

const EPS: f64 = 1e-9;

fn clamp_aspect_ratio(
    x0: f64,
    y0: f64,
    x1: f64,
    y1: f64,
    max_ratio: f64,
    min_ratio: f64,
) -> (f64, f64, f64, f64) {
    let rw = x1 - x0;
    let rh = y1 - y0;
    if rw <= 0.0 || rh <= 0.0 {
        return (x0, y0, x1, y1);
    }
    let ls = rw.max(rh);
    let ss = rw.min(rh);
    let current_ratio = ls / ss;
    if max_ratio > 0.0 && current_ratio > max_ratio {
        let nl = ss * max_ratio;
        if rw >= rh {
            let cx = (x0 + x1) * 0.5;
            (cx - nl * 0.5, y0, cx + nl * 0.5, y1)
        } else {
            let cy = (y0 + y1) * 0.5;
            (x0, cy - nl * 0.5, x1, cy + nl * 0.5)
        }
    } else if min_ratio > 0.0 && current_ratio < min_ratio {
        let nl = ss * min_ratio;
        if rw >= rh {
            let cx = (x0 + x1) * 0.5;
            (cx - nl * 0.5, y0, cx + nl * 0.5, y1)
        } else {
            let cy = (y0 + y1) * 0.5;
            (x0, cy - nl * 0.5, x1, cy + nl * 0.5)
        }
    } else {
        (x0, y0, x1, y1)
    }
}

fn obstacle_overlaps_x_range(g: &super::combined::GroupedObstacles, x0: f64, x1: f64) -> bool {
    for pt in &g.points {
        if pt.x >= x0 && pt.x <= x1 {
            return true;
        }
    }
    for l in &g.lines {
        let lx0 = l.ax.min(l.bx);
        let lx1 = l.ax.max(l.bx);
        if lx0 <= x1 && lx1 >= x0 {
            return true;
        }
    }
    for p in &g.polygons {
        if p.bbox.0 <= x1 && p.bbox.2 >= x0 {
            return true;
        }
    }
    false
}

fn obstacle_overlaps_y_range(g: &super::combined::GroupedObstacles, y0: f64, y1: f64) -> bool {
    for pt in &g.points {
        if pt.y >= y0 && pt.y <= y1 {
            return true;
        }
    }
    for l in &g.lines {
        let ly0 = l.ay.min(l.by);
        let ly1 = l.ay.max(l.by);
        if ly0 <= y1 && ly1 >= y0 {
            return true;
        }
    }
    for p in &g.polygons {
        if p.bbox.1 <= y1 && p.bbox.3 >= y0 {
            return true;
        }
    }
    false
}

pub fn solve_lir_obstacles_axis_aligned(
    poly: &Polygon<f64>,
    obstacles: &[ObstacleInput],
    options: &LirObstaclesOptions,
) -> Result<LirObstaclesResult> {
    let bb = poly
        .bounding_rect()
        .ok_or_else(|| LirError::InvalidPolygon("degenerate polygon".into()))?;
    let bx0 = bb.min().x;
    let by0 = bb.min().y;
    let bx1 = bb.max().x;
    let by1 = bb.max().y;

    if bx1 - bx0 < EPS || by1 - by0 < EPS {
        return Ok(LirObstaclesResult::empty());
    }

    let g = group_obstacles(obstacles);

    let mut x_coords: HashSet<OrderedFloat<f64>> = HashSet::new();
    let mut y_coords: HashSet<OrderedFloat<f64>> = HashSet::new();

    for coord in poly.exterior().0.iter() {
        x_coords.insert(OrderedFloat(coord.x));
        y_coords.insert(OrderedFloat(coord.y));
    }
    for interior in poly.interiors() {
        for coord in interior.0.iter() {
            x_coords.insert(OrderedFloat(coord.x));
            y_coords.insert(OrderedFloat(coord.y));
        }
    }
    for pt in &g.points {
        x_coords.insert(OrderedFloat(pt.x));
        y_coords.insert(OrderedFloat(pt.y));
    }
    for line in &g.lines {
        x_coords.insert(OrderedFloat(line.ax));
        x_coords.insert(OrderedFloat(line.bx));
        y_coords.insert(OrderedFloat(line.ay));
        y_coords.insert(OrderedFloat(line.by));
    }
    for p in &g.polygons {
        x_coords.insert(OrderedFloat(p.bbox.0));
        x_coords.insert(OrderedFloat(p.bbox.2));
        y_coords.insert(OrderedFloat(p.bbox.1));
        y_coords.insert(OrderedFloat(p.bbox.3));
    }

    let mut xs: Vec<f64> = x_coords.into_iter().map(|f| f.into_inner()).collect();
    let mut ys: Vec<f64> = y_coords.into_iter().map(|f| f.into_inner()).collect();
    xs.sort_by(|a, b| a.partial_cmp(b).unwrap());
    ys.sort_by(|a, b| a.partial_cmp(b).unwrap());
    xs.dedup_by(|a, b| (*a - *b).abs() < EPS);
    ys.dedup_by(|a, b| (*a - *b).abs() < EPS);

    // Obstacle-driven non-uniform grid: each cell that overlaps an obstacle
    // is subdivided so obstacles create fine cells. Open space stays coarse.
    // This makes obstacles into actual grid features the LRIH must route around.
    const N_SUB: usize = 6;
    let xs_sparse = xs.clone();
    let ys_sparse = ys.clone();
    let x_span = xs_sparse.last().unwrap() - xs_sparse[0];
    let y_span = ys_sparse.last().unwrap() - ys_sparse[0];

    xs.clear();
    ys.clear();
    for w in xs_sparse.windows(2) {
        xs.push(w[0]);
        let sub = obstacle_overlaps_x_range(&g, w[0], w[1]);
        if sub && w[1] - w[0] > x_span * 0.001 {
            for k in 1..=N_SUB {
                xs.push(w[0] + (w[1] - w[0]) * k as f64 / (N_SUB + 1) as f64);
            }
        }
    }
    xs.push(*xs_sparse.last().unwrap());
    for w in ys_sparse.windows(2) {
        ys.push(w[0]);
        let sub = obstacle_overlaps_y_range(&g, w[0], w[1]);
        if sub && w[1] - w[0] > y_span * 0.001 {
            for k in 1..=N_SUB {
                ys.push(w[0] + (w[1] - w[0]) * k as f64 / (N_SUB + 1) as f64);
            }
        }
    }
    ys.push(*ys_sparse.last().unwrap());

    let n_cols = xs.len().saturating_sub(1);
    let n_rows = ys.len().saturating_sub(1);
    if n_cols == 0 || n_rows == 0 {
        return Ok(LirObstaclesResult::empty());
    }

    let mut poly_mask = vec![false; n_cols * n_rows];
    let total_cells = n_cols * n_rows;
    if total_cells <= 4096 {
        for row in 0..n_rows {
            let cy = (ys[row] + ys[row + 1]) * 0.5;
            for col in 0..n_cols {
                let cx = (xs[col] + xs[col + 1]) * 0.5;
                poly_mask[row * n_cols + col] = poly.contains(&Point::new(cx, cy));
            }
        }
    } else {
        let row_intervals = compute_row_intervals(poly, &xs, &ys);
        for row in 0..n_rows {
            for (col_start, col_end) in &row_intervals[row] {
                for col in *col_start..*col_end {
                    if col < n_cols {
                        poly_mask[row * n_cols + col] = true;
                    }
                }
            }
        }
    }

    let obs_mask = build_obstacle_mask_aa(&g, &xs, &ys);

    let mut combined = vec![false; n_cols * n_rows];
    for i in 0..combined.len() {
        combined[i] = poly_mask[i] && obs_mask[i];
    }

    let mut heights = vec![0usize; n_cols];
    let mut best_area = 0.0;
    let mut best_rect: Option<Rectangle> = None;

    for row in 0..n_rows {
        for col in 0..n_cols {
            if combined[row * n_cols + col] {
                heights[col] += 1;
            } else {
                heights[col] = 0;
            }
        }
        let (x0, y0, x1, y1, area) = lrih_vp(
            &heights,
            &xs,
            &ys,
            row,
            options.max_ratio,
            options.min_ratio,
        );
        if area > best_area {
            best_area = area;
            let (cx0, cy0, cx1, cy1) =
                clamp_aspect_ratio(x0, y0, x1, y1, options.max_ratio, options.min_ratio);
            best_rect = Some(Rectangle {
                x_min: cx0,
                y_min: cy0,
                x_max: cx1,
                y_max: cy1,
            });
        }
    }

    let contracted = best_rect.and_then(|r| {
        contract_rect_to_boundary(poly, r.x_min, r.y_min, r.x_max, r.y_max).and_then(
            |(x0, y0, x1, y1)| {
                let (sx0, sy0, sx1, sy1) = shrink_away_from_points(&g, x0, y0, x1, y1);
                let (sx0, sy0, sx1, sy1) = shrink_away_from_polygons(&g, sx0, sy0, sx1, sy1);
                if rect_avoids_obstacles(&g, sx0, sy0, sx1, sy1) {
                    Some(Rectangle {
                        x_min: sx0,
                        y_min: sy0,
                        x_max: sx1,
                        y_max: sy1,
                    })
                } else {
                    None
                }
            },
        )
    });

    let bb_candidate = poly.bounding_rect().and_then(|bb| {
        let x0 = bb.min().x;
        let y0 = bb.min().y;
        let x1 = bb.max().x;
        let y1 = bb.max().y;
        if x1 - x0 < EPS || y1 - y0 < EPS {
            return None;
        }
        contract_rect_to_boundary(poly, x0, y0, x1, y1).and_then(|(x0, y0, x1, y1)| {
            if rect_avoids_obstacles(&g, x0, y0, x1, y1) {
                Some(Rectangle {
                    x_min: x0,
                    y_min: y0,
                    x_max: x1,
                    y_max: y1,
                })
            } else {
                None
            }
        })
    });

    let final_rect = match (contracted, bb_candidate) {
        (Some(vg), Some(bbv)) => {
            if bbv.area() > vg.area() {
                bbv
            } else {
                vg
            }
        }
        (Some(vg), None) => vg,
        (None, Some(bbv)) => bbv,
        (None, None) => return Ok(LirObstaclesResult::empty()),
    };

    let (x0, y0, x1, y1) = clamp_aspect_ratio(
        final_rect.x_min,
        final_rect.y_min,
        final_rect.x_max,
        final_rect.y_max,
        options.max_ratio,
        options.min_ratio,
    );

    let rect = Rectangle {
        x_min: x0,
        y_min: y0,
        x_max: x1,
        y_max: y1,
    };
    let area = rect.area();
    let rect_poly = rect.to_polygon();

    Ok(LirObstaclesResult {
        rect: Some(rect),
        rect_polygon: Some(rect_poly),
        area,
        angle_deg: 0.0,
        best_effort: false,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo_types::{coord, LineString};

    fn rp(x0: f64, y0: f64, x1: f64, y1: f64) -> Polygon<f64> {
        Polygon::new(
            LineString::from(vec![
                coord! { x: x0, y: y0 },
                coord! { x: x1, y: y0 },
                coord! { x: x1, y: y1 },
                coord! { x: x0, y: y1 },
                coord! { x: x0, y: y0 },
            ]),
            vec![],
        )
    }

    fn opts() -> LirObstaclesOptions {
        LirObstaclesOptions::default()
    }

    #[test]
    fn no_obstacles_square() {
        let poly = rp(0.0, 0.0, 10.0, 10.0);
        let result = solve_lir_obstacles_axis_aligned(&poly, &[], &opts()).unwrap();
        assert!((result.area - 100.0).abs() < 1.0);
    }

    #[test]
    fn point_obstacle_center() {
        let poly = rp(0.0, 0.0, 10.0, 10.0);
        let obs = vec![ObstacleInput::Point(coord! { x: 5.0, y: 5.0 })];
        let result = solve_lir_obstacles_axis_aligned(&poly, &obs, &opts()).unwrap();
        assert!(result.area > 20.0);
        assert!(result.area < 100.0);
    }

    #[test]
    fn polygon_obstacle_center() {
        let poly = rp(0.0, 0.0, 10.0, 10.0);
        let obs = vec![ObstacleInput::Polygon(rp(4.0, 4.0, 6.0, 6.0))];
        let result = solve_lir_obstacles_axis_aligned(&poly, &obs, &opts()).unwrap();
        assert!(result.area > 30.0);
        assert!(result.area < 100.0);
    }

    #[test]
    fn vertical_line_obstacle() {
        let poly = rp(0.0, 0.0, 10.0, 10.0);
        let line = LineString::from(vec![coord! { x: 5.0, y: 0.0 }, coord! { x: 5.0, y: 10.0 }]);
        let obs = vec![ObstacleInput::Line(line)];
        let result = solve_lir_obstacles_axis_aligned(&poly, &obs, &opts()).unwrap();
        assert!(result.area > 40.0);
        if let Some(rect) = &result.rect {
            assert!(rect.x_max <= 5.0 || rect.x_min >= 5.0);
        }
    }

    #[test]
    fn obstacle_covers_all() {
        let poly = rp(0.0, 0.0, 10.0, 10.0);
        let obs = vec![ObstacleInput::Polygon(rp(0.0, 0.0, 10.0, 10.0))];
        let result = solve_lir_obstacles_axis_aligned(&poly, &obs, &opts()).unwrap();
        assert!(result.area < 1.0);
    }

    #[test]
    fn mixed_obstacles() {
        let poly = rp(0.0, 0.0, 10.0, 10.0);
        let line = LineString::from(vec![coord! { x: 7.0, y: 0.0 }, coord! { x: 7.0, y: 10.0 }]);
        let obs = vec![
            ObstacleInput::Point(coord! { x: 2.0, y: 2.0 }),
            ObstacleInput::Line(line),
            ObstacleInput::Polygon(rp(4.0, 4.0, 5.0, 5.0)),
        ];
        let result = solve_lir_obstacles_axis_aligned(&poly, &obs, &opts()).unwrap();
        assert!(result.area > 0.0);
    }

    #[test]
    fn triangular_polygon_with_obstacle() {
        let poly = Polygon::new(
            LineString::from(vec![
                coord! { x: 0.0, y: 0.0 },
                coord! { x: 10.0, y: 0.0 },
                coord! { x: 0.0, y: 10.0 },
                coord! { x: 0.0, y: 0.0 },
            ]),
            vec![],
        );
        let obs = vec![ObstacleInput::Point(coord! { x: 3.0, y: 3.0 })];
        let result = solve_lir_obstacles_axis_aligned(&poly, &obs, &opts()).unwrap();
        assert!(result.area > 0.0);
    }

    #[test]
    fn aspect_ratio_constraint() {
        let mut o = opts();
        o.max_ratio = 1.5;
        let poly = rp(0.0, 0.0, 10.0, 10.0);
        let obs = vec![ObstacleInput::Point(coord! { x: 5.0, y: 5.0 })];
        let result = solve_lir_obstacles_axis_aligned(&poly, &obs, &o).unwrap();
        if let Some(rect) = &result.rect {
            let w = rect.x_max - rect.x_min;
            let h = rect.y_max - rect.y_min;
            assert!(w.max(h) / w.min(h) <= 1.52);
        }
    }
}
