use ige_core::{solve_oriented_lir, SolverOptions};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use geo_types::{Polygon, LineString, Coord};

#[pyclass]
#[derive(Clone)]
pub struct PyOrientedLirResult {
    #[pyo3(get)]
    pub x_min: f64,
    #[pyo3(get)]
    pub y_min: f64,
    #[pyo3(get)]
    pub x_max: f64,
    #[pyo3(get)]
    pub y_max: f64,
    #[pyo3(get)]
    pub area: f64,
}

#[pyfunction(signature = (exterior, rotation_degrees=None))]
pub fn solve_oriented_lir_py(
    exterior: Vec<(f64, f64)>,
    rotation_degrees: Option<f64>,
) -> PyResult<PyOrientedLirResult> {
    if exterior.len() < 3 {
        return Err(PyValueError::new_err("polygon exterior must contain at least 3 points"));
    }

    let coords: Vec<Coord<f64>> = exterior
        .into_iter()
        .map(|(x, y)| Coord { x, y })
        .collect();
    let exterior_ls = LineString::from(coords);
    let polygon = Polygon::new(exterior_ls, vec![]);

    let mut _opts = SolverOptions::default();
    if let Some(deg) = rotation_degrees {
        _opts.rotation_degrees = deg;
    }

    let result = solve_oriented_lir(&polygon)
        .ok_or_else(|| PyValueError::new_err("solve failed"))?;

    Ok(PyOrientedLirResult {
        x_min: result.x_min,
        y_min: result.y_min,
        x_max: result.x_max,
        y_max: result.y_max,
        area: result.area(),
    })
}

#[pyfunction]
fn oriented_lir_demo() -> PyResult<String> {
    let result = solve_oriented_lir_py(
        vec![(0.0, 0.0), (8.0, 1.0), (7.0, 7.0), (2.0, 8.0), (-1.0, 4.0)],
        Some(0.0),
    )?;
    Ok(format!(
        "area={:.3}, bounds=({:.3}, {:.3}, {:.3}, {:.3})",
        result.area, result.x_min, result.y_min, result.x_max, result.y_max
    ))
}

#[pymodule]
fn _native(_py: Python, m: &Bound<PyModule>) -> PyResult<()> {
    m.add_class::<PyOrientedLirResult>()?;
    m.add_function(wrap_pyfunction!(solve_oriented_lir_py, m)?)?;
    m.add_function(wrap_pyfunction!(oriented_lir_demo, m)?)?;
    Ok(())
}