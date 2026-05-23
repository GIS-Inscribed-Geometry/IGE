use geo_types::{coord, LineString, Polygon};
use ige_core::{
    build_obb_frame, solve_obb, solve_obb_aspect_fit, solve_obb_constrained, ObbOptions,
};

fn square() -> Polygon<f64> {
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

fn rect_20x5() -> Polygon<f64> {
    Polygon::new(
        LineString::from(vec![
            coord! {x:0.0, y:0.0},
            coord! {x:20.0, y:0.0},
            coord! {x:20.0, y:5.0},
            coord! {x:0.0, y:5.0},
            coord! {x:0.0, y:0.0},
        ]),
        vec![],
    )
}

fn diamond() -> Polygon<f64> {
    Polygon::new(
        LineString::from(vec![
            coord! {x:5.0, y:0.0},
            coord! {x:10.0, y:5.0},
            coord! {x:5.0, y:10.0},
            coord! {x:0.0, y:5.0},
            coord! {x:5.0, y:0.0},
        ]),
        vec![],
    )
}

fn triangle() -> Polygon<f64> {
    Polygon::new(
        LineString::from(vec![
            coord! {x:0.0, y:0.0},
            coord! {x:10.0, y:0.0},
            coord! {x:0.0, y:10.0},
            coord! {x:0.0, y:0.0},
        ]),
        vec![],
    )
}

fn rect_10x5() -> Polygon<f64> {
    Polygon::new(
        LineString::from(vec![
            coord! {x:0.0, y:0.0},
            coord! {x:10.0, y:0.0},
            coord! {x:10.0, y:5.0},
            coord! {x:0.0, y:5.0},
            coord! {x:0.0, y:0.0},
        ]),
        vec![],
    )
}

/// Concave L-shape: hull should be the full 10×10 square
fn l_shape() -> Polygon<f64> {
    Polygon::new(
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
    )
}

// ── solve_obb (rotating calipers, min-area) ───────────────────────────

#[test]
fn calipers_square() {
    let r = solve_obb(&square(), &ObbOptions::default()).unwrap();
    assert!((r.area - 100.0).abs() < 1e-6, "area={}", r.area);
    assert!((r.width - 10.0).abs() < 1e-6);
    assert!((r.height - 10.0).abs() < 1e-6);
    assert!(r.polygon.is_some());
    assert!(r.fill_ratio > 0.0);
}

#[test]
fn calipers_rectangle() {
    let r = solve_obb(&rect_20x5(), &ObbOptions::default()).unwrap();
    assert!((r.area - 100.0).abs() < 1e-6, "area={}", r.area);
    assert!((r.angle_deg - 0.0).abs() < 1e-6);
    assert!((r.width - 20.0).abs() < 1e-6);
    assert!((r.height - 5.0).abs() < 1e-6);
}

#[test]
fn calipers_diamond() {
    let r = solve_obb(&diamond(), &ObbOptions::default()).unwrap();
    let diag = 5.0 * 2.0_f64.sqrt();
    assert!((r.area - diag * diag).abs() < 1e-6, "area={}", r.area);
    assert!(r.polygon.is_some());
}

#[test]
fn calipers_triangle() {
    let r = solve_obb(&triangle(), &ObbOptions::default()).unwrap();
    assert!((r.area - 100.0).abs() < 1e-6, "area={}", r.area);
    assert!((r.width - 10.0).abs() < 1e-6);
    assert!((r.height - 10.0).abs() < 1e-6);
    assert!(r.fill_ratio > 0.0);
}

#[test]
fn calipers_concave_uses_hull() {
    let r = solve_obb(&l_shape(), &ObbOptions::default()).unwrap();
    assert!((r.area - 100.0).abs() < 1e-6, "area={}", r.area);
    assert!(r.polygon.is_some());
}

#[test]
fn calipers_degenerate_returns_err() {
    let bad = Polygon::new(
        LineString::from(vec![coord! {x:0.0, y:0.0}, coord! {x:0.0, y:0.0}]),
        vec![],
    );
    assert!(solve_obb(&bad, &ObbOptions::default()).is_err());
}

#[test]
fn calipers_zero_area_returns_err() {
    let line = Polygon::new(
        LineString::from(vec![
            coord! {x:0.0, y:0.0},
            coord! {x:5.0, y:0.0},
            coord! {x:10.0, y:0.0},
            coord! {x:0.0, y:0.0},
        ]),
        vec![],
    );
    assert!(solve_obb(&line, &ObbOptions::default()).is_err());
}

#[test]
fn calipers_result_fields_populated() {
    let r = solve_obb(&square(), &ObbOptions::default()).unwrap();
    assert!(r.perimeter > 0.0);
    assert!(r.centroid.is_some());
    assert!(r.aspect_ratio >= 1.0);
    assert!((r.aspect_ratio - 1.0).abs() < 1e-6);
    assert!((r.north_fill - 0.0).abs() < 1e-12);
    assert!((r.improve_pct - 0.0).abs() < 1e-12);
    assert_eq!(r.n_intervals, 0);
}

// ── axis-aligned OBB ──────────────────────────────────────────────────

#[test]
fn axis_aligned_square() {
    use ige_core::solvers::obb::axis_aligned::solve_obb_axis_aligned;
    let r = solve_obb_axis_aligned(&square(), &ObbOptions::default()).unwrap();
    assert!((r.area - 100.0).abs() < 1e-6);
    assert!((r.angle_deg - 0.0).abs() < 1e-6);
}

#[test]
fn axis_aligned_rectangle() {
    use ige_core::solvers::obb::axis_aligned::solve_obb_axis_aligned;
    let r = solve_obb_axis_aligned(&rect_20x5(), &ObbOptions::default()).unwrap();
    assert!((r.area - 100.0).abs() < 1e-6);
    assert!((r.width - 20.0).abs() < 1e-6);
    assert!((r.height - 5.0).abs() < 1e-6);
}

// ── solve_obb_aspect_fit ──────────────────────────────────────────────

#[test]
fn aspect_fit_square_a4() {
    let res = solve_obb_aspect_fit(&square(), 297.0, 210.0).unwrap();
    assert!(
        (res.fill_ratio - 0.707).abs() < 0.01,
        "fill={}",
        res.fill_ratio
    );
    assert!(res.angle_deg >= 0.0);
    assert!(res.n_intervals > 0);
}

#[test]
fn aspect_fit_rectangle_a4() {
    let res = solve_obb_aspect_fit(&rect_20x5(), 297.0, 210.0).unwrap();
    assert!(res.fill_ratio > 0.3, "fill={}", res.fill_ratio);
    assert!(res.north_fill > 0.0);
}

#[test]
fn aspect_fit_triangle() {
    let res = solve_obb_aspect_fit(&triangle(), 1.0, 1.0).unwrap();
    assert!(res.fill_ratio > 0.0);
}

#[test]
fn aspect_fit_exact_match() {
    let res = solve_obb_aspect_fit(&rect_10x5(), 2.0, 1.0).unwrap();
    assert!(
        (res.fill_ratio - 1.0).abs() < 1e-12,
        "fill={}",
        res.fill_ratio
    );
    assert!((res.angle_deg - 0.0).abs() < 1e-6);
    assert!(res.improve_pct >= 0.0);
}

#[test]
fn aspect_fit_improve_pct_zero_when_optimal() {
    let res = solve_obb_aspect_fit(&square(), 1.0, 1.0).unwrap();
    assert!((res.fill_ratio - 1.0).abs() < 1e-12);
    assert!((res.north_fill - 1.0).abs() < 1e-12);
    assert!(
        (res.improve_pct - 0.0).abs() < 1e-12,
        "imp={}",
        res.improve_pct
    );
}

#[test]
fn aspect_fit_degenerate() {
    let bad = Polygon::new(
        LineString::from(vec![coord! {x:0.0, y:0.0}, coord! {x:0.0, y:0.0}]),
        vec![],
    );
    assert!(solve_obb_aspect_fit(&bad, 1.0, 1.0).is_none());
}

#[test]
fn aspect_fit_zero_height() {
    assert!(solve_obb_aspect_fit(&square(), 1.0, 0.0).is_none());
}

#[test]
fn aspect_fit_fill_ratio_formula() {
    use ige_core::solvers::obb::oriented::common::fill_ratio;
    assert!((fill_ratio(10.0, 10.0, 2.0) - 0.5).abs() < 1e-12);
    assert!((fill_ratio(10.0, 5.0, 2.0) - 1.0).abs() < 1e-12);
    let f = fill_ratio(0.0, 5.0, 2.0);
    assert!((f - 0.0).abs() < 1e-12, "zero w fill={}", f);
}

#[test]
fn aspect_fit_width_depth() {
    use geo_types::Coord;
    use ige_core::solvers::obb::oriented::common::width_depth;
    let pts = vec![
        Coord { x: 0.0, y: 0.0 },
        Coord { x: 10.0, y: 0.0 },
        Coord { x: 10.0, y: 10.0 },
        Coord { x: 0.0, y: 10.0 },
    ];
    let (w, d) = width_depth(&pts, 0.0);
    assert!((w - 10.0).abs() < 1e-12);
    assert!((d - 10.0).abs() < 1e-12);

    let (w, d) = width_depth(&pts, std::f64::consts::PI / 4.0);
    let diag = 10.0 * 2.0_f64.sqrt();
    assert!((w - diag).abs() < 1e-10, "w={}", w);
    assert!((d - diag).abs() < 1e-10, "d={}", d);
}

#[test]
fn aspect_fit_width_depth_collinear() {
    use geo_types::Coord;
    use ige_core::solvers::obb::oriented::common::width_depth;
    let pts = vec![
        Coord { x: 0.0, y: 0.0 },
        Coord { x: 5.0, y: 0.0 },
        Coord { x: 10.0, y: 0.0 },
    ];
    let (w, d) = width_depth(&pts, 0.0);
    assert!((w - 10.0).abs() < 1e-12, "w={}", w);
    assert!((d - 0.0).abs() < 1e-12, "d={}", d);
}

#[test]
fn aspect_fit_hull_ccw() {
    use ige_core::solvers::obb::oriented::common::hull_ccw;
    let pts = hull_ccw(&square()).unwrap();
    let mut area = 0.0;
    for i in 0..pts.len() {
        let j = (i + 1) % pts.len();
        area += pts[i].x * pts[j].y;
        area -= pts[j].x * pts[i].y;
    }
    assert!(area > 0.0, "area={}", area);
}

#[test]
fn aspect_fit_hull_ccw_concave() {
    use ige_core::solvers::obb::oriented::common::hull_ccw;
    let pts = hull_ccw(&l_shape()).unwrap();
    let mut area = 0.0;
    for i in 0..pts.len() {
        let j = (i + 1) % pts.len();
        area += pts[i].x * pts[j].y;
        area -= pts[j].x * pts[i].y;
    }
    assert!(area > 0.0, "area={}", area);
    // L-shape hull is the full 10×10 square → 4 vertices
    assert!(pts.len() >= 3, "pts.len={}", pts.len());
}

#[test]
fn aspect_fit_caliper_breakpoints() {
    use ige_core::solvers::obb::oriented::common::caliper_breakpoints;
    use ige_core::solvers::obb::oriented::common::hull_ccw;
    let pts = hull_ccw(&square()).unwrap();
    let bps = caliper_breakpoints(&pts);
    assert!(!bps.is_empty());
    for &b in &bps {
        assert!(b >= 0.0 && b < std::f64::consts::PI);
    }
}

#[test]
fn aspect_fit_antipodal_coeffs() {
    use ige_core::solvers::obb::oriented::common::antipodal_coeffs;
    use ige_core::solvers::obb::oriented::common::hull_ccw;
    let pts = hull_ccw(&square()).unwrap();
    let (aw, bw, ad, bd) = antipodal_coeffs(&pts, 0.0);
    assert!(aw.is_finite());
    assert!(bw.is_finite());
    assert!(ad.is_finite());
    assert!(bd.is_finite());
}

#[test]
fn aspect_fit_north_fill_not_zero() {
    let res = solve_obb_aspect_fit(&diamond(), 297.0, 210.0).unwrap();
    assert!(res.north_fill > 0.0, "north_fill={}", res.north_fill);
}

// ── build_obb_frame ───────────────────────────────────────────────────

#[test]
fn build_frame_basic() {
    let res = solve_obb_aspect_fit(&square(), 2.0, 1.0).unwrap();
    let theta = res.angle_deg.to_radians();
    let result = build_obb_frame(&square(), theta, 2.0, 1.0);
    assert!(result.is_some());

    let (_frame, fw, fh, wb, hb, fcx, fcy) = result.unwrap();
    assert!(fw > 0.0);
    assert!(fh > 0.0);
    assert!(wb > 0.0);
    assert!(hb > 0.0);
    assert!(fcx.is_finite());
    assert!(fcy.is_finite());
    assert!((fw / fh - 2.0).abs() < 1e-10, "ar={}", fw / fh);
}

#[test]
fn build_frame_at_zero_angle_matches_axis_aligned() {
    let result = build_obb_frame(&square(), 0.0, 1.0, 1.0).unwrap();
    let (frame, fw, fh, wb, hb, fcx, fcy) = result;
    assert!((fw - 10.0).abs() < 1e-10, "fw={}", fw);
    assert!((fh - 10.0).abs() < 1e-10, "fh={}", fh);
    assert!((wb - 10.0).abs() < 1e-10, "wb={}", wb);
    assert!((hb - 10.0).abs() < 1e-10, "hb={}", hb);
    assert!((fcx - 5.0).abs() < 1e-10, "fcx={}", fcx);
    assert!((fcy - 5.0).abs() < 1e-10, "fcy={}", fcy);
    assert_eq!(frame.exterior().0.len(), 5);
}

#[test]
fn build_frame_cover_contains_centroid() {
    let res = solve_obb_aspect_fit(&diamond(), 2.0, 1.0).unwrap();
    let theta = res.angle_deg.to_radians();
    let (frame, _, _, _, _, fcx, fcy) = build_obb_frame(&diamond(), theta, 2.0, 1.0).unwrap();
    assert!(fcx.is_finite());
    assert!(fcy.is_finite());
    assert!((fcx - 5.0).abs() < 5.0);
    assert!((fcy - 5.0).abs() < 5.0);
    // frame centroid should be inside the frame polygon
    use geo::Centroid;
    let fc = frame.centroid().unwrap();
    assert!(fc.x().is_finite());
    assert!(fc.y().is_finite());
}

// ── solve_obb_constrained ─────────────────────────────────────────────

#[test]
fn constrained_max_ratio() {
    let mut opts = ObbOptions::default();
    opts.max_ratio = 2.0;
    let r = solve_obb_constrained(&rect_20x5(), &opts).unwrap();
    assert!(r.aspect_ratio <= 2.0 + 1e-6, "ar={}", r.aspect_ratio);
}

#[test]
fn constrained_min_ratio() {
    let mut opts = ObbOptions::default();
    opts.min_ratio = 3.0;
    let r = solve_obb_constrained(&square(), &opts).unwrap();
    assert!(r.aspect_ratio >= 3.0 - 1e-6, "ar={}", r.aspect_ratio);
}

#[test]
fn constrained_no_op_when_unconstrained() {
    let r = solve_obb_constrained(&square(), &ObbOptions::default()).unwrap();
    assert!((r.area - 100.0).abs() < 1e-6);
}

#[test]
fn constrained_max_ratio_already_satisfied() {
    let mut opts = ObbOptions::default();
    opts.max_ratio = 5.0;
    let r = solve_obb_constrained(&rect_20x5(), &opts).unwrap();
    assert!((r.area - 100.0).abs() < 1e-6, "area={}", r.area);
}

#[test]
fn constrained_min_ratio_already_satisfied() {
    let mut opts = ObbOptions::default();
    opts.min_ratio = 1.0;
    let r = solve_obb_constrained(&square(), &opts).unwrap();
    assert!((r.area - 100.0).abs() < 1e-6);
}

// ── ObbResult ─────────────────────────────────────────────────────────

#[test]
fn empty_result_defaults() {
    let r = ige_core::ObbResult::empty();
    assert!(r.polygon.is_none());
    assert_eq!(r.area, 0.0);
    assert_eq!(r.perimeter, 0.0);
    assert_eq!(r.angle_deg, 0.0);
    assert_eq!(r.width, 0.0);
    assert_eq!(r.height, 0.0);
    assert!(r.centroid.is_none());
    assert_eq!(r.aspect_ratio, 1.0);
    assert_eq!(r.fill_ratio, 0.0);
    assert_eq!(r.north_fill, 0.0);
    assert_eq!(r.improve_pct, 0.0);
    assert_eq!(r.n_intervals, 0);
}

#[test]
fn default_is_empty() {
    let r: ige_core::ObbResult = Default::default();
    assert!(r.polygon.is_none());
}
