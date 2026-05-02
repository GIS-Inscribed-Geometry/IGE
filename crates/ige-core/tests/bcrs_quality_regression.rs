use geo::Area;
use geo_types::{coord, LineString, Polygon};
use ige_core::bcrs::{solve_bcrs, BcrsOptions};

fn compare_parallel_quality(poly: &Polygon<f64>, min_ratio: f64) {
    let mut std_opts = BcrsOptions::default();
    std_opts.use_parallel_field = false;
    let std_res = solve_bcrs(poly, &std_opts).expect("standard bcrs solve failed");

    let mut par_opts = BcrsOptions::default();
    par_opts.use_parallel_field = true;
    let par_res = solve_bcrs(poly, &par_opts).expect("parallel bcrs solve failed");

    assert!(std_res.area > 0.0, "standard area must be positive");
    assert!(par_res.area > 0.0, "parallel area must be positive");
    let ratio = par_res.area / std_res.area.max(1e-12);
    assert!(
        ratio >= min_ratio,
        "parallel quality regression: std_area={} par_area={} ratio={}",
        std_res.area,
        par_res.area,
        ratio
    );
}

fn square_with_hole() -> Polygon<f64> {
    Polygon::new(
        LineString::from(vec![
            coord! {x:0.0, y:0.0},
            coord! {x:12.0, y:0.0},
            coord! {x:12.0, y:12.0},
            coord! {x:0.0, y:12.0},
            coord! {x:0.0, y:0.0},
        ]),
        vec![LineString::from(vec![
            coord! {x:4.0, y:4.0},
            coord! {x:8.0, y:4.0},
            coord! {x:8.0, y:8.0},
            coord! {x:4.0, y:8.0},
            coord! {x:4.0, y:4.0},
        ])],
    )
}

#[test]
fn parallel_quality_on_representative_shapes() {
    let shapes = vec![
        Polygon::new(
            LineString::from(vec![
                coord! {x:0.0, y:0.0},
                coord! {x:20.0, y:0.0},
                coord! {x:20.0, y:5.0},
                coord! {x:0.0, y:5.0},
                coord! {x:0.0, y:0.0},
            ]),
            vec![],
        ),
        Polygon::new(
            LineString::from(vec![
                coord! {x:0.0, y:0.0},
                coord! {x:8.0, y:0.0},
                coord! {x:8.0, y:2.0},
                coord! {x:3.0, y:2.0},
                coord! {x:3.0, y:8.0},
                coord! {x:0.0, y:8.0},
                coord! {x:0.0, y:0.0},
            ]),
            vec![],
        ),
        Polygon::new(
            LineString::from(vec![
                coord! {x:0.0, y:0.0},
                coord! {x:10.0, y:1.0},
                coord! {x:11.0, y:8.0},
                coord! {x:4.0, y:11.0},
                coord! {x:-1.0, y:5.0},
                coord! {x:0.0, y:0.0},
            ]),
            vec![],
        ),
        Polygon::new(
            LineString::from(vec![
                coord! {x:0.0, y:0.0},
                coord! {x:10.0, y:0.0},
                coord! {x:10.0, y:10.0},
                coord! {x:6.0, y:10.0},
                coord! {x:6.0, y:3.0},
                coord! {x:4.0, y:3.0},
                coord! {x:4.0, y:10.0},
                coord! {x:0.0, y:10.0},
                coord! {x:0.0, y:0.0},
            ]),
            vec![],
        ),
        square_with_hole(),
    ];

    for poly in &shapes {
        assert!(poly.unsigned_area() > 0.0);
        compare_parallel_quality(poly, 0.75);
    }
}
