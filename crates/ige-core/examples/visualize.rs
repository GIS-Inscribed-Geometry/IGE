//! IGE Visual Preview Tool
//!
//! Generates HTML files to visualize polygons and their largest inscribed rectangles.
//! Output goes to `target/ige_output/` directory.
//!
//! Run with: cargo run --package ige-core --example visualize

use geo::Area;
use geo_types::{Coord, LineString, Polygon};
use ige_core::solve_oriented_lir;
use serde_json::Value;
use std::fs;
use std::path::PathBuf;

fn parse_ring(value: &Value) -> Option<Vec<Coord<f64>>> {
    let ring = value.as_array()?;
    let mut coords = Vec::new();
    for point in ring {
        let pt = point.as_array()?;
        if pt.len() >= 2 {
            let x = pt[0].as_f64()?;
            let y = pt[1].as_f64()?;
            coords.push(Coord { x, y });
        }
    }
    Some(coords)
}

fn parse_polygon(geom: &Value) -> Option<Polygon<f64>> {
    let coords = geom.get("coordinates")?;
    let arr = coords.as_array()?;
    let ext_ring = arr.get(0)?;
    let exterior = parse_ring(ext_ring)?;
    if exterior.len() < 3 {
        return None;
    }
    let exterior_ls = LineString::from(exterior);
    let holes: Vec<LineString<f64>> = arr[1..]
        .iter()
        .filter_map(|ring| parse_ring(ring))
        .filter(|ls| ls.len() >= 3)
        .map(LineString::from)
        .collect();

    if holes.is_empty() {
        Some(Polygon::new(exterior_ls, vec![]))
    } else {
        Some(Polygon::new(exterior_ls, holes))
    }
}

fn load_polygons() -> Vec<(usize, Polygon<f64>)> {
    let content = include_str!("../tests/real_world_data/realworld.geojson");
    let json: Value = serde_json::from_str(content).expect("Failed to parse realworld.geojson");

    let features = json.get("features").expect("No features");
    let arr = features.as_array().expect("Features is not array");

    arr.iter()
        .filter_map(|f| {
            let id = f.get("properties")?.get("fid")?.as_u64()? as usize;
            let geom = f.get("geometry")?;
            let poly = parse_polygon(geom)?;
            Some((id, poly))
        })
        .collect()
}

fn make_l_shape(cx: f64, cy: f64, size: f64) -> Polygon<f64> {
    Polygon::new(
        LineString::from(vec![
            Coord { x: cx - size, y: cy - size },
            Coord { x: cx + size, y: cy - size },
            Coord { x: cx + size, y: cy - size * 0.3 },
            Coord { x: cx + size * 0.3, y: cy - size * 0.3 },
            Coord { x: cx + size * 0.3, y: cy + size },
            Coord { x: cx - size, y: cy + size },
            Coord { x: cx - size, y: cy - size },
        ]),
        vec![],
    )
}

fn make_u_shape(cx: f64, cy: f64, size: f64) -> Polygon<f64> {
    Polygon::new(
        LineString::from(vec![
            Coord { x: cx - size, y: cy - size },
            Coord { x: cx + size, y: cy - size },
            Coord { x: cx + size, y: cy + size },
            Coord { x: cx + size * 0.4, y: cy + size },
            Coord { x: cx + size * 0.4, y: cy },
            Coord { x: cx - size * 0.4, y: cy },
            Coord { x: cx - size * 0.4, y: cy + size },
            Coord { x: cx - size, y: cy + size },
            Coord { x: cx - size, y: cy - size },
        ]),
        vec![],
    )
}

fn make_zigzag(cx: f64, cy: f64, size: f64) -> Polygon<f64> {
    Polygon::new(
        LineString::from(vec![
            Coord { x: cx - size, y: cy - size },
            Coord { x: cx - size * 0.6, y: cy - size },
            Coord { x: cx - size * 0.2, y: cy },
            Coord { x: cx + size * 0.2, y: cy },
            Coord { x: cx + size * 0.6, y: cy - size },
            Coord { x: cx + size, y: cy - size },
            Coord { x: cx + size, y: cy + size },
            Coord { x: cx + size * 0.6, y: cy + size },
            Coord { x: cx + size * 0.2, y: cy },
            Coord { x: cx - size * 0.2, y: cy },
            Coord { x: cx - size * 0.6, y: cy + size },
            Coord { x: cx - size, y: cy + size },
            Coord { x: cx - size, y: cy - size },
        ]),
        vec![],
    )
}

fn is_valid_polygon(poly: &Polygon<f64>) -> bool {
    if poly.unsigned_area() <= 0.0 {
        return false;
    }
    for coord in poly.exterior().0.iter() {
        if !coord.x.is_finite() || !coord.y.is_finite() {
            return false;
        }
    }
    true
}

fn get_polygon_bounds(poly: &Polygon<f64>) -> (f64, f64, f64, f64) {
    let mut min_x = f64::MAX;
    let mut min_y = f64::MAX;
    let mut max_x = f64::MIN;
    let mut max_y = f64::MIN;

    for coord in poly.exterior().0.iter() {
        min_x = min_x.min(coord.x);
        min_y = min_y.min(coord.y);
        max_x = max_x.max(coord.x);
        max_y = max_y.max(coord.y);
    }

    (min_x, min_y, max_x, max_y)
}

fn generate_svg_for_polygon(id: &str, poly: &Polygon<f64>, rect: Option<&ige_core::Rectangle>, time_ms: f64) -> String {
    let (min_x, min_y, max_x, max_y) = get_polygon_bounds(poly);
    let poly_area = poly.unsigned_area();
    let rect_area = rect.map(|r| r.area()).unwrap_or(0.0);
    
    let size = 200.0;
    let padding = 10.0;
    let draw_size = size - 2.0 * padding;
    
    let width = max_x - min_x;
    let height = max_y - min_y;
    let scale = if width > 0.0 && height > 0.0 {
        (draw_size / width).min(draw_size / height)
    } else {
        draw_size
    };
    
    let offset_x = padding + (draw_size - width * scale) / 2.0;
    let offset_y = padding + (draw_size - height * scale) / 2.0;
    
    let to_svg = |x: f64, y: f64| -> (f64, f64) {
        (offset_x + (x - min_x) * scale, offset_y + (y - min_y) * scale)
    };
    
    let exterior_points: String = poly.exterior().0.iter()
        .map(|c| {
            let (sx, sy) = to_svg(c.x, c.y);
            format!("{:.1},{:.1}", sx, sy)
        })
        .collect::<Vec<_>>()
        .join(" ");
    
    let holes_svg: String = poly.interiors().iter()
        .map(|hole| {
            let points: String = hole.0.iter()
                .map(|c| {
                    let (sx, sy) = to_svg(c.x, c.y);
                    format!("{:.1},{:.1}", sx, sy)
                })
                .collect::<Vec<_>>()
                .join(" ");
            format!(r#"<polygon class="hole" points="{}"/>"#, points)
        })
        .collect();
    
    let rect_svg = if let Some(r) = rect {
        let (x0, y0) = to_svg(r.x_min, r.y_min);
        let (x1, y1) = to_svg(r.x_max, r.y_max);
        format!(
            r#"<rect class="rect" x="{:.1}" y="{:.1}" width="{:.1}" height="{:.1}"/>"#,
            x0, y0, x1 - x0, y1 - y0
        )
    } else {
        String::new()
    };
    
    let fill_ratio = if poly_area > 0.0 { rect_area / poly_area * 100.0 } else { 0.0 };
    
    format!(
        r#"<div class="card">
            <svg viewBox="0 0 {:.0} {:.0}">
                <polygon class="polygon" points="{}"/>
                {}
                {}
            </svg>
            <div class="info">
                <strong>{}</strong><br/>
                Polygon: {:.1}<br/>
                Rectangle: {:.1}<br/>
                Fill: {:.1}%<br/>
                Time: {:.2}ms
            </div>
        </div>"#,
        size, size,
        exterior_points,
        holes_svg,
        rect_svg,
        id,
        poly_area,
        rect_area,
        fill_ratio,
        time_ms
    )
}

pub fn generate_preview_html(output_dir: &PathBuf, max_polygons: Option<usize>) -> std::io::Result<()> {
    let mut all_polygons: Vec<(String, Polygon<f64>)> = Vec::new();
    
    all_polygons.push(("Square 10x10".to_string(), Polygon::new(
        LineString::from(vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 10.0, y: 0.0 },
            Coord { x: 10.0, y: 10.0 },
            Coord { x: 0.0, y: 10.0 },
            Coord { x: 0.0, y: 0.0 },
        ]),
        vec![],
    )));
    
    all_polygons.push(("Rectangle 10x1".to_string(), Polygon::new(
        LineString::from(vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 10.0, y: 0.0 },
            Coord { x: 10.0, y: 1.0 },
            Coord { x: 0.0, y: 1.0 },
            Coord { x: 0.0, y: 0.0 },
        ]),
        vec![],
    )));
    
    all_polygons.push(("Triangle".to_string(), Polygon::new(
        LineString::from(vec![
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 10.0, y: 0.0 },
            Coord { x: 5.0, y: 10.0 },
            Coord { x: 0.0, y: 0.0 },
        ]),
        vec![],
    )));
    
    all_polygons.push(("L-Shape".to_string(), make_l_shape(5.0, 5.0, 5.0)));
    
    all_polygons.push(("U-Shape".to_string(), make_u_shape(5.0, 5.0, 5.0)));
    
    all_polygons.push(("Zigzag".to_string(), make_zigzag(5.0, 5.0, 5.0)));
    
    let real_polygons = load_polygons();
    for (id, poly) in real_polygons {
        let vertex_count = poly.exterior().0.len() - 1;
        all_polygons.push((format!("Real #{} ({}v)", id, vertex_count), poly));
        if let Some(n) = max_polygons {
            if all_polygons.len() >= n {
                break;
            }
        }
    }
    
    let output_dir = output_dir.join("ige_output");
    fs::create_dir_all(&output_dir)?;

    let mut html = String::from(r#"<!DOCTYPE html>
<html>
<head>
    <title>IGE Visual Preview</title>
    <style>
        body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Arial, sans-serif; margin: 20px; background: #1a1a2e; color: #eee; }
        h1 { color: #eee; margin-bottom: 10px; }
        .grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(220px, 1fr)); gap: 15px; }
        .card { background: #16213e; border-radius: 8px; padding: 10px; box-shadow: 0 2px 8px rgba(0,0,0,0.3); }
        svg { width: 100%; height: 200px; background: #0f0f23; border-radius: 4px; }
        .polygon { fill: #e94560; stroke: #ff6b6b; stroke-width: 1; }
        .rect { fill: rgba(66, 133, 244, 0.4); stroke: #4285f4; stroke-width: 2; }
        .hole { fill: none; stroke: #666; stroke-width: 1; stroke-dasharray: 3; }
        .info { margin-top: 8px; font-size: 11px; color: #aaa; line-height: 1.4; }
        .stats { background: #16213e; padding: 20px; border-radius: 8px; margin-bottom: 20px; box-shadow: 0 2px 8px rgba(0,0,0,0.3); }
        .stats p { margin: 5px 0; color: #ccc; }
        .stats strong { color: #fff; }
    </style>
</head>
<body>
    <h1>IGE - Largest Inscribed Rectangle Preview</h1>
    <div class="stats">
"#);

    let mut success_count = 0;
    let mut failed_count = 0;
    let mut total_rect_area = 0.0;
    let mut total_poly_area: f64 = 0.0;
    let mut total_time_ms = 0.0;
    let mut cards_html = String::new();
    
    for (id, poly) in &all_polygons {
        let poly_area = poly.unsigned_area();
        total_poly_area += poly_area;
        
        let start = std::time::Instant::now();
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| solve_oriented_lir(poly)))
            .unwrap_or(None);
        let elapsed = start.elapsed().as_secs_f64() * 1000.0;
        total_time_ms += elapsed;
        
        if result.is_some() {
            success_count += 1;
        } else {
            failed_count += 1;
        }
        
        if let Some(ref rect) = result {
            total_rect_area += rect.area();
        }
        
        cards_html.push_str(&generate_svg_for_polygon(id, poly, result.as_ref(), elapsed));
    }

    let fill_ratio = if total_poly_area > 0.0 { total_rect_area / total_poly_area * 100.0 } else { 0.0 };
    
html.push_str(&format!(
        r#"
        <p><strong>Total shapes:</strong> {}</p>
        <p><strong>Successfully processed:</strong> {} ({:.1}%)</p>
        <p><strong>Failed:</strong> {}</p>
        <p><strong>Total polygon area:</strong> {:.0}</p>
        <p><strong>Total inscribed area:</strong> {:.0} ({:.1}%)</p>
        <p><strong>Total processing time:</strong> {:.1}ms ({:.2}ms avg per shape)</p>
        "#,
        all_polygons.len(),
        success_count,
        success_count as f64 / all_polygons.len() as f64 * 100.0,
        failed_count,
        total_poly_area,
        total_rect_area,
        fill_ratio,
        total_time_ms,
        total_time_ms / all_polygons.len() as f64
    ));

    html.push_str(r#"
    </div>
    <div class="grid">
"#);
    
    html.push_str(&cards_html);
    
    html.push_str(r#"
    </div>
</body>
</html>
"#);

    let output_path = output_dir.join("index.html");
    fs::write(&output_path, &html)?;
    
    println!("Generated preview: {}", output_path.display());
    
    Ok(())
}

fn main() {
    let real_polygons = load_polygons();
    eprintln!("Loaded {} polygons from realworld.geojson", real_polygons.len());
    generate_preview_html(&std::env::current_dir().unwrap().join("target"), None).unwrap();
}