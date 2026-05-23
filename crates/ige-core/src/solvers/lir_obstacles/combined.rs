use geo::{BoundingRect, Contains, Intersects};
use geo_types::{Coord, LineString, Point, Polygon};

use crate::shared::ObstacleInput;

const EPS: f64 = 1e-9;

#[derive(Clone, Debug)]
pub(crate) struct PointObs {
    pub x: f64,
    pub y: f64,
}

#[derive(Clone, Debug)]
pub(crate) struct LineObs {
    pub ax: f64,
    pub ay: f64,
    pub bx: f64,
    pub by: f64,
}

#[derive(Clone, Debug)]
pub(crate) struct PolyObs {
    pub poly: Polygon<f64>,
    pub bbox: (f64, f64, f64, f64),
}

#[derive(Clone, Debug, Default)]
pub(crate) struct GroupedObstacles {
    pub points: Vec<PointObs>,
    pub lines: Vec<LineObs>,
    pub polygons: Vec<PolyObs>,
}

pub(crate) fn group_obstacles(inputs: &[ObstacleInput]) -> GroupedObstacles {
    let mut points = Vec::new();
    let mut lines = Vec::new();
    let mut polygons = Vec::new();

    for inp in inputs {
        match inp {
            ObstacleInput::Point(c) => {
                points.push(PointObs { x: c.x, y: c.y });
            }
            ObstacleInput::Line(ls) => {
                let coords: Vec<Coord<f64>> = ls.coords().copied().collect();
                if coords.len() >= 2 {
                    let (ax, ay) = (coords[0].x, coords[0].y);
                    let (bx, by) = (coords[1].x, coords[1].y);
                    lines.push(LineObs { ax, ay, bx, by });
                }
            }
            ObstacleInput::Polygon(p) => {
                if let Some(bb) = p.bounding_rect() {
                    polygons.push(PolyObs {
                        poly: p.clone(),
                        bbox: (bb.min().x, bb.min().y, bb.max().x, bb.max().y),
                    });
                }
            }
        }
    }

    GroupedObstacles {
        points,
        lines,
        polygons,
    }
}

fn line_intersects_cell(
    ax: f64,
    ay: f64,
    bx: f64,
    by: f64,
    cx0: f64,
    cx1: f64,
    cy0: f64,
    cy1: f64,
) -> bool {
    let dx = bx - ax;
    let dy = by - ay;
    let mut t_min: f64 = 0.0;
    let mut t_max: f64 = 1.0;
    let edges = [
        (-dx, ax - cx0),
        (dx, cx1 - ax),
        (-dy, ay - cy0),
        (dy, cy1 - ay),
    ];
    for &(p, q) in &edges {
        if p.abs() < 1e-15 {
            if q < -EPS {
                return false;
            }
        } else {
            let t = q / p;
            if p < 0.0 {
                if t > t_min {
                    t_min = t;
                }
            } else if t < t_max {
                t_max = t;
            }
        }
    }
    t_min <= t_max
}

pub(crate) fn build_obstacle_mask_aa(g: &GroupedObstacles, xs: &[f64], ys: &[f64]) -> Vec<bool> {
    let n_cols = xs.len().saturating_sub(1);
    let n_rows = ys.len().saturating_sub(1);
    if n_cols == 0 || n_rows == 0 {
        return Vec::new();
    }

    let total = n_cols * n_rows;
    let mut mask = vec![true; total];

    // Points sit on grid line intersections; block adjacent cells.
    for pt in &g.points {
        let mut ci = n_cols;
        for c in 0..n_cols {
            if (pt.x - xs[c]).abs() < EPS || (pt.x - xs[c + 1]).abs() < EPS {
                ci = c;
                break;
            }
        }
        let mut ri = n_rows;
        for r in 0..n_rows {
            if (pt.y - ys[r]).abs() < EPS || (pt.y - ys[r + 1]).abs() < EPS {
                ri = r;
                break;
            }
        }
        let r0 = ri.saturating_sub(1);
        let r1 = (ri + 1).min(n_rows);
        let c0 = ci.saturating_sub(1);
        let c1 = (ci + 1).min(n_cols);
        for r in r0..r1 {
            for c in c0..c1 {
                mask[r * n_cols + c] = false;
            }
        }
    }

    // Polygon obstacles: mark cells whose centre is inside the polygon.

    for line in &g.lines {
        let lminx = line.ax.min(line.bx);
        let lmaxx = line.ax.max(line.bx);
        let lminy = line.ay.min(line.by);
        let lmaxy = line.ay.max(line.by);

        let mut c_start = n_cols;
        let mut c_end = 0;
        for c in 0..n_cols {
            if xs[c + 1] >= lminx && xs[c] <= lmaxx {
                if c < c_start {
                    c_start = c;
                }
                c_end = c + 1;
            }
        }
        if c_start >= c_end {
            continue;
        }

        let mut r_start = n_rows;
        let mut r_end = 0;
        for r in 0..n_rows {
            if ys[r + 1] >= lminy && ys[r] <= lmaxy {
                if r < r_start {
                    r_start = r;
                }
                r_end = r + 1;
            }
        }
        if r_start >= r_end {
            continue;
        }

        for r in r_start..r_end {
            for c in c_start..c_end {
                let idx = r * n_cols + c;
                if !mask[idx] {
                    continue;
                }
                if line_intersects_cell(
                    line.ax,
                    line.ay,
                    line.bx,
                    line.by,
                    xs[c],
                    xs[c + 1],
                    ys[r],
                    ys[r + 1],
                ) {
                    mask[idx] = false;
                }
            }
        }
    }

    for p in &g.polygons {
        let (pminx, pminy, pmaxx, pmaxy) = p.bbox;

        let mut c_start = n_cols;
        let mut c_end = 0;
        for c in 0..n_cols {
            if xs[c + 1] > pminx && xs[c] < pmaxx {
                if c < c_start {
                    c_start = c;
                }
                c_end = c + 1;
            }
        }
        if c_start >= c_end {
            continue;
        }

        let mut r_start = n_rows;
        let mut r_end = 0;
        for r in 0..n_rows {
            if ys[r + 1] > pminy && ys[r] < pmaxy {
                if r < r_start {
                    r_start = r;
                }
                r_end = r + 1;
            }
        }
        if r_start >= r_end {
            continue;
        }

        for r in r_start..r_end {
            let cy = (ys[r] + ys[r + 1]) * 0.5;
            for c in c_start..c_end {
                let idx = r * n_cols + c;
                if !mask[idx] {
                    continue;
                }
                let cx = (xs[c] + xs[c + 1]) * 0.5;
                if p.poly.contains(&Point::new(cx, cy)) {
                    mask[idx] = false;
                }
            }
        }
    }

    mask
}

pub(crate) fn rect_avoids_obstacles(
    g: &GroupedObstacles,
    x0: f64,
    y0: f64,
    x1: f64,
    y1: f64,
) -> bool {
    if x1 - x0 < 1e-12 || y1 - y0 < 1e-12 {
        return false;
    }

    for pt in &g.points {
        if pt.x > x0 + EPS && pt.x < x1 - EPS && pt.y > y0 + EPS && pt.y < y1 - EPS {
            return false;
        }
    }

    for line in &g.lines {
        if line_intersects_cell(line.ax, line.ay, line.bx, line.by, x0, x1, y0, y1) {
            return false;
        }
    }

    for p in &g.polygons {
        if p.bbox.0 < x1 - EPS && p.bbox.2 > x0 + EPS && p.bbox.1 < y1 - EPS && p.bbox.3 > y0 + EPS
        {
            if p.poly.intersects(&rect_to_poly(x0, y0, x1, y1)) {
                return false;
            }
        }
    }

    true
}

/// Shrink a rectangle so it excludes every point obstacle inside its interior.
/// Iteratively shrinks the smallest side to push the nearest point outside,
/// repeating until no points remain inside.
pub(crate) fn shrink_away_from_points(
    g: &GroupedObstacles,
    x0: f64,
    y0: f64,
    x1: f64,
    y1: f64,
) -> (f64, f64, f64, f64) {
    let mut l = x0;
    let mut b = y0;
    let mut r = x1;
    let mut t = y1;
    loop {
        let mut best_dist = f64::MAX;
        let mut best_edge = 0u8; // 0=left, 1=right, 2=bottom, 3=top
        let mut best_val = 0.0f64;
        for pt in &g.points {
            if pt.x > l + EPS && pt.x < r - EPS && pt.y > b + EPS && pt.y < t - EPS {
                let left = pt.x - l;
                let right = r - pt.x;
                let bottom = pt.y - b;
                let top = t - pt.y;
                let min = left.min(right).min(bottom).min(top);
                if min > 0.0 && min < best_dist {
                    best_dist = min;
                    if (left - min).abs() < EPS {
                        best_edge = 0;
                        best_val = pt.x;
                    } else if (right - min).abs() < EPS {
                        best_edge = 1;
                        best_val = pt.x;
                    } else if (bottom - min).abs() < EPS {
                        best_edge = 2;
                        best_val = pt.y;
                    } else {
                        best_edge = 3;
                        best_val = pt.y;
                    }
                }
            }
        }
        if best_dist == f64::MAX {
            break;
        }
        match best_edge {
            0 => l = best_val + EPS,
            1 => r = best_val - EPS,
            2 => b = best_val + EPS,
            3 => t = best_val - EPS,
            _ => unreachable!(),
        }
    }
    (l, b, r, t)
}

/// Shrink a rectangle to exclude all polygon obstacles that overlap it.
/// Finds the polygon-edge pair that requires the smallest shrink and applies
/// just that edge push, iterating until no more overlap.
pub(crate) fn shrink_away_from_polygons(
    g: &GroupedObstacles,
    x0: f64,
    y0: f64,
    x1: f64,
    y1: f64,
) -> (f64, f64, f64, f64) {
    let mut l = x0;
    let mut b = y0;
    let mut r = x1;
    let mut t = y1;
    loop {
        let mut best_dist = f64::MAX;
        let mut best_edge = 0u8;
        let mut best_val = 0.0f64;
        for p in &g.polygons {
            if p.bbox.0 < r - EPS && p.bbox.2 > l + EPS && p.bbox.1 < t - EPS && p.bbox.3 > b + EPS
            {
                let left_gap = r - p.bbox.0 + EPS;
                let right_gap = p.bbox.2 - l + EPS;
                let bottom_gap = t - p.bbox.1 + EPS;
                let top_gap = p.bbox.3 - b + EPS;
                let min_gap = left_gap.min(right_gap).min(bottom_gap).min(top_gap);
                if min_gap > 0.0 && min_gap < best_dist {
                    best_dist = min_gap;
                    if (left_gap - min_gap).abs() < EPS {
                        best_edge = 1;
                        best_val = p.bbox.0 - EPS;
                    } else if (right_gap - min_gap).abs() < EPS {
                        best_edge = 0;
                        best_val = p.bbox.2 + EPS;
                    } else if (bottom_gap - min_gap).abs() < EPS {
                        best_edge = 3;
                        best_val = p.bbox.1 - EPS;
                    } else {
                        best_edge = 2;
                        best_val = p.bbox.3 + EPS;
                    }
                }
            }
        }
        if best_dist == f64::MAX {
            break;
        }
        match best_edge {
            0 => l = l.max(best_val),
            1 => r = r.min(best_val),
            2 => b = b.max(best_val),
            3 => t = t.min(best_val),
            _ => unreachable!(),
        }
        if r - l < EPS || t - b < EPS {
            break;
        }
    }
    (l, b, r, t)
}

fn rect_to_poly(x0: f64, y0: f64, x1: f64, y1: f64) -> Polygon<f64> {
    Polygon::new(
        LineString::from(vec![
            Coord { x: x0, y: y0 },
            Coord { x: x1, y: y0 },
            Coord { x: x1, y: y1 },
            Coord { x: x0, y: y1 },
            Coord { x: x0, y: y0 },
        ]),
        vec![],
    )
}

pub(crate) fn buffer_lines_to_polygons(lines: &[LineObs], thickness: f64) -> Vec<PolyObs> {
    if thickness <= 0.0 {
        return Vec::new();
    }
    let half = thickness / 2.0;
    lines
        .iter()
        .map(|l| {
            let x0 = l.ax.min(l.bx) - half;
            let x1 = l.ax.max(l.bx) + half;
            let y0 = l.ay.min(l.by) - half;
            let y1 = l.ay.max(l.by) + half;
            let rect = Polygon::new(
                LineString::from(vec![
                    Coord { x: x0, y: y0 },
                    Coord { x: x1, y: y0 },
                    Coord { x: x1, y: y1 },
                    Coord { x: x0, y: y1 },
                    Coord { x: x0, y: y0 },
                ]),
                vec![],
            );
            let bb = rect.bounding_rect().unwrap();
            PolyObs {
                poly: rect,
                bbox: (bb.min().x, bb.min().y, bb.max().x, bb.max().y),
            }
        })
        .collect()
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

    #[test]
    fn test_rect_avoids_point() {
        let g = group_obstacles(&[ObstacleInput::Point(coord! { x: 5.0, y: 5.0 })]);
        assert!(rect_avoids_obstacles(&g, 0.0, 0.0, 4.0, 10.0));
        assert!(!rect_avoids_obstacles(&g, 0.0, 0.0, 10.0, 10.0));
    }

    #[test]
    fn test_rect_avoids_line() {
        let line = LineString::from(vec![coord! { x: 5.0, y: 0.0 }, coord! { x: 5.0, y: 10.0 }]);
        let g = group_obstacles(&[ObstacleInput::Line(line)]);
        assert!(rect_avoids_obstacles(&g, 0.0, 0.0, 4.0, 10.0));
        assert!(!rect_avoids_obstacles(&g, 0.0, 0.0, 10.0, 10.0));
    }

    #[test]
    fn test_rect_avoids_polygon() {
        let obs = rp(4.0, 4.0, 6.0, 6.0);
        let g = group_obstacles(&[ObstacleInput::Polygon(obs)]);
        assert!(rect_avoids_obstacles(&g, 0.0, 0.0, 3.0, 10.0));
        assert!(!rect_avoids_obstacles(&g, 0.0, 0.0, 10.0, 10.0));
    }

    #[test]
    fn test_empty_mask_all_free() {
        let g = GroupedObstacles::default();
        let xs = vec![0.0, 5.0, 10.0];
        let ys = vec![0.0, 5.0, 10.0];
        let mask = build_obstacle_mask_aa(&g, &xs, &ys);
        assert_eq!(mask.len(), 4);
        assert!(mask.iter().all(|&v| v));
    }
}
