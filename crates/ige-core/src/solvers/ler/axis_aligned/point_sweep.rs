//! O(n log n) plane-sweep for Largest Empty Rectangle amidst point obstacles.
//!
//! Uses a doubly-linked list over `ys` to avoid O(n) `Vec::remove` shifting.
//! Kills gaps permanently when they can never beat `best_area`.

use super::{LerOptions, LerResult};
use crate::shared::{LirError, Rectangle, Result};
use geo::BoundingRect;
use geo_types::{Coord, Polygon, Rect};

const EPS: f64 = 1e-9;

pub fn solve_ler_points_sweep(
    poly: &Polygon<f64>,
    points: &[Coord<f64>],
    options: &LerOptions,
) -> Result<LerResult> {
    let bb = poly
        .bounding_rect()
        .ok_or_else(|| LirError::InvalidPolygon("degenerate".into()))?;
    let (bx0, by0, bx1, by1) = (bb.min().x, bb.min().y, bb.max().x, bb.max().y);

    if bx1 - bx0 < EPS || by1 - by0 < EPS {
        return Ok(LerResult::empty());
    }

    if points.is_empty() {
        if aspect_ok(bx1 - bx0, by1 - by0, options) {
            let r = Rectangle {
                x_min: bx0,
                y_min: by0,
                x_max: bx1,
                y_max: by1,
            };
            return Ok(LerResult {
                area: r.area(),
                rect: Some(r),
                rect_polygon: Some(
                    Rect::new(Coord { x: bx0, y: by0 }, Coord { x: bx1, y: by1 }).to_polygon(),
                ),
                angle_deg: 0.0,
                best_effort: false,
            });
        }
        return Ok(LerResult::empty());
    }

    // Group points by x, skip boundary points, dedup (x,y)
    let mut x_groups: Vec<(f64, Vec<f64>)> = Vec::new();
    let mut sorted: Vec<(f64, f64)> = points.iter().map(|c| (c.x, c.y)).collect();
    sorted.sort_by(|a, b| {
        a.0.partial_cmp(&b.0)
            .unwrap()
            .then(a.1.partial_cmp(&b.1).unwrap())
    });
    sorted.dedup_by(|a, b| (a.0 - b.0).abs() < EPS && (a.1 - b.1).abs() < EPS);
    for (x, y) in sorted {
        if x <= bx0 + EPS || x >= bx1 - EPS || y <= by0 + EPS || y >= by1 - EPS {
            continue;
        }
        if let Some(last) = x_groups.last_mut() {
            if (last.0 - x).abs() < EPS {
                last.1.push(y);
                continue;
            }
        }
        x_groups.push((x, vec![y]));
    }

    if x_groups.is_empty() {
        let r = Rectangle {
            x_min: bx0,
            y_min: by0,
            x_max: bx1,
            y_max: by1,
        };
        return Ok(LerResult {
            area: r.area(),
            rect: Some(r),
            rect_polygon: Some(
                Rect::new(Coord { x: bx0, y: by0 }, Coord { x: bx1, y: by1 }).to_polygon(),
            ),
            angle_deg: 0.0,
            best_effort: false,
        });
    }

    // Build sorted ys array
    let mut ys: Vec<f64> =
        Vec::with_capacity(x_groups.iter().map(|g| g.1.len()).sum::<usize>() + 2);
    ys.push(by0);
    for (_, gys) in &x_groups {
        ys.extend_from_slice(gys);
    }
    ys.push(by1);
    ys.sort_by(|a, b| a.partial_cmp(b).unwrap());

    let n = ys.len();

    // Doubly-linked list over ys indices (active points only)
    let mut prev: Vec<usize> = Vec::with_capacity(n);
    let mut next: Vec<usize> = Vec::with_capacity(n);
    for i in 0..n {
        prev.push(if i == 0 { 0 } else { i - 1 });
        next.push(if i == n - 1 { n - 1 } else { i + 1 });
    }
    let mut active: Vec<bool> = vec![true; n];

    // left[i] = left barrier for gap (prev[i], i), stored at top of gap
    let mut left: Vec<f64> = vec![bx0; n];

    // Records for final bx1 pass: (y_lo, y_hi, left_x) saved at gap creation
    let mut records: Vec<(f64, f64, f64)> = Vec::new();

    // Initial gaps
    for i in 1..n {
        records.push((ys[i - 1], ys[i], bx0));
    }

    let mut best_area = 0.0;
    let mut best_rect: Option<Rectangle> = None;

    #[inline]
    fn record(
        rect: &Rectangle,
        area: f64,
        best_area: &mut f64,
        best_rect: &mut Option<Rectangle>,
        opts: &LerOptions,
    ) {
        if area > *best_area + EPS {
            let w = rect.x_max - rect.x_min;
            let h = rect.y_max - rect.y_min;
            if w > EPS && h > EPS {
                let (s, l) = (w.min(h), w.max(h));
                let r = l / s;
                let ratio_ok = opts.max_ratio <= 0.0 || r <= opts.max_ratio * 1.000001;
                let min_ok = opts.min_ratio <= 0.0 || r >= opts.min_ratio * 0.999999;
                if ratio_ok && min_ok {
                    *best_area = area;
                    *best_rect = Some(rect.clone());
                }
            }
        }
    }

    // Sweep left to right
    for &(cur_x, ref gys) in &x_groups {
        // Evaluate all active gaps at cur_x
        let mut gap_top = next[0];
        while gap_top < n - 1 {
            let gap_bot = prev[gap_top];
            let h = ys[gap_top] - ys[gap_bot];
            let l = left[gap_top];
            if cur_x > l + EPS {
                let area = (cur_x - l) * h;
                if area > best_area + EPS {
                    let rect = Rectangle {
                        x_min: l,
                        y_min: ys[gap_bot],
                        x_max: cur_x,
                        y_max: ys[gap_top],
                    };
                    record(&rect, area, &mut best_area, &mut best_rect, options);
                }
            }
            gap_top = next[gap_top];
        }

        // Remove points at this x
        for &y in gys {
            let Ok(idx) = ys.binary_search_by(|a| a.partial_cmp(&y).unwrap()) else {
                continue;
            };
            if idx == 0 || idx == n - 1 || !active[idx] {
                continue;
            }

            let p = prev[idx];
            let nxt = next[idx];

            active[idx] = false;
            let new_left = left[idx].max(left[nxt]).max(cur_x);

            next[p] = nxt;
            prev[nxt] = p;
            left[nxt] = new_left;

            // Save record for the new merged gap
            records.push((ys[p], ys[nxt], new_left));
        }
    }

    // Final pass: right wall = bx1. Check all recorded gap states.
    for &(y_lo, y_hi, left_x) in &records {
        if bx1 > left_x + EPS {
            let h = y_hi - y_lo;
            let w = bx1 - left_x;
            if w * h > best_area + EPS {
                let rect = Rectangle {
                    x_min: left_x,
                    y_min: y_lo,
                    x_max: bx1,
                    y_max: y_hi,
                };
                record(&rect, rect.area(), &mut best_area, &mut best_rect, options);
            }
        }
    }

    match best_rect {
        Some(r) => {
            let area = r.area();
            Ok(LerResult {
                area,
                rect: Some(r.clone()),
                rect_polygon: Some(
                    Rect::new(
                        Coord {
                            x: r.x_min,
                            y: r.y_min,
                        },
                        Coord {
                            x: r.x_max,
                            y: r.y_max,
                        },
                    )
                    .to_polygon(),
                ),
                angle_deg: 0.0,
                best_effort: false,
            })
        }
        None => Ok(LerResult::empty()),
    }
}

fn aspect_ok(w: f64, h: f64, opts: &LerOptions) -> bool {
    if w < EPS || h < EPS {
        return false;
    }
    let (s, l) = (w.min(h), w.max(h));
    let r = l / s;
    if opts.max_ratio > 0.0 && r > opts.max_ratio * 1.000001 {
        return false;
    }
    if opts.min_ratio > 0.0 && r < opts.min_ratio * 0.999999 {
        return false;
    }
    true
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
    fn opts() -> LerOptions {
        LerOptions::default()
    }

    #[test]
    fn no_points_fills_box() {
        let poly = rp(0., 0., 10., 10.);
        let r = solve_ler_points_sweep(&poly, &[], &opts()).unwrap();
        assert!(r.area > 99.0);
    }
    #[test]
    fn single_center_point() {
        let poly = rp(0., 0., 10., 10.);
        let pts = vec![coord! { x: 5., y: 5. }];
        let r = solve_ler_points_sweep(&poly, &pts, &opts()).unwrap();
        assert!(r.area > 20.0 && r.area < 80.0);
    }
    #[test]
    fn four_corner_points() {
        let poly = rp(0., 0., 10., 10.);
        let pts = vec![
            coord! { x: 2., y: 2. },
            coord! { x: 8., y: 2. },
            coord! { x: 2., y: 8. },
            coord! { x: 8., y: 8. },
        ];
        let r = solve_ler_points_sweep(&poly, &pts, &opts()).unwrap();
        assert!(r.area > 0.0);
    }
    #[test]
    fn vertical_line_of_points() {
        let poly = rp(0., 0., 10., 10.);
        let pts: Vec<_> = (1..10).map(|i| coord! { x: 5., y: i as f64 }).collect();
        let r = solve_ler_points_sweep(&poly, &pts, &opts()).unwrap();
        assert!(r.area > 0.0);
    }
    #[test]
    fn many_random_points() {
        let poly = rp(0., 0., 100., 100.);
        let pts: Vec<_> = (0..300)
            .map(|i| coord! { x: ((i * 157) % 99 + 1) as f64, y: ((i * 271) % 99 + 1) as f64 })
            .collect();
        let r = solve_ler_points_sweep(&poly, &pts, &opts()).unwrap();
        assert!(r.area > 0.0);
    }

    #[test]
    fn matches_sweep_line_on_simple() {
        use super::super::solve_ler_axis_aligned_exact;
        let poly = rp(0., 0., 10., 10.);
        let pts = vec![coord! { x: 5., y: 5. }];
        let obs: Vec<Polygon<f64>> = pts
            .iter()
            .map(|c| {
                Polygon::new(
                    LineString::from(vec![
                        coord! { x: c.x - 0.01, y: c.y - 0.01 },
                        coord! { x: c.x + 0.01, y: c.y - 0.01 },
                        coord! { x: c.x + 0.01, y: c.y + 0.01 },
                        coord! { x: c.x - 0.01, y: c.y + 0.01 },
                        coord! { x: c.x - 0.01, y: c.y - 0.01 },
                    ]),
                    vec![],
                )
            })
            .collect();
        let r_old = solve_ler_axis_aligned_exact(&poly, &obs, &opts()).unwrap();
        let r_new = solve_ler_points_sweep(&poly, &pts, &opts()).unwrap();
        assert!(
            (r_old.area - r_new.area).abs() < 5.0,
            "old={:.2} new={:.2} differ too much",
            r_old.area,
            r_new.area
        );
    }
}
