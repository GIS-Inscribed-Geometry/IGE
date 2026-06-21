[![Rust](https://img.shields.io/badge/Rust-1.85+-orange)](https://www.rust-lang.org/)
[![MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.10+-blue)](https://www.python.org/)
[![WebGPU](https://img.shields.io/badge/GPU-WebGPU-green)](https://www.w3.org/TR/webgpu/)
[![CI](https://img.shields.io/github/actions/workflow/status/GIS-Inscribed-Geometry/IGE/wheels.yml?label=CI)](https://github.com/GIS-Inscribed-Geometry/IGE/actions)
[![Status: Experimental](https://img.shields.io/badge/Status-Experimental-orange)]()

# IGE — Inscribed Geometry Engine

A Rust library for computing inscribed geometry problems relevant to GIS and spatial analysis. Provides approximate and exact solvers for the largest inscribed rectangle (LIR), maximum inscribed circle (MIC), largest empty rectangle (LER), oriented bounding boxes (OBB), and polygon nesting — with axis-aligned and oriented (rotated) variants, optional GPU acceleration via WebGPU (WGSL), and integration with QGIS and Python/numpy.

```mermaid
flowchart LR
    A[Input polygon] --> B{Inscribed problem}
    B --> C[LIR: LargestInscribed Rectangle]
    B --> D[MIC: MaximumInscribed Circle]
    B --> E[LER: LargestEmpty Rectangle]
    B --> F[OBB: OrientedBounding Box]
    B --> G[Nested: Insidecontainer polygon]
    
    C --> H[Axis-aligned]
    C --> I[Oriented]
    D --> J[Exact / Grid / GEOS]
    E --> K[Point / Line / Polygon obstacles]
    F --> L[Min-area / Aspect-fit / Constrained]
    G --> M[Convex / General]

    H & I --> N[CPU parallel]
    I --> O[GPU compute(WebGPU / WGSL)]
```

## Solvers

### LIR — Largest Inscribed Rectangle

Finds the largest rectangle (by area) that fits entirely inside a polygon.

| Variant | Methods | Backend |
|---|---|---|
| Axis-aligned | Vertex grid, SDF, histogram, exact (B.C.R.S.) | CPU |
| Oriented (rotated) | Parallel candidate evaluation, edge-anchored, gradient expand, Brent polish, bootstrap seeds, PCA axes | CPU |
| Oriented (rotated) | Grid-batch, SDF (WGSL shaders) | GPU (WebGPU) |
| With obstacles | Axis-aligned + oriented, combined solver | CPU |

### MIC — Maximum Inscribed Circle

Finds the largest circle that fits inside a polygon.

| Method | Description |
|---|---|
| Exact (native Rust) | Polygon-specialised Voronoi-based solver |
| Grid approximate | Uniform grid sampling with gradient ascent |
| GEOS fallback | Optional GEOS engine for cross-validation |

### LER — Largest Empty Rectangle

Finds the largest axis-aligned rectangle that avoids a set of obstacle points, line segments, or polygons.

| Variant | Methods |
|---|---|
| Axis-aligned | Points (DC / sweep), lines (exact / mixed) |
| Oriented | Fixed rotation, mixed obstacle types |

### OBB — Oriented Bounding Box

Finds the optimal rotation for an enclosing rectangle.

| Variant | Method |
|---|---|
| Min-area OBB | Rotating calipers — O(n) on convex hull |
| Aspect-ratio fit (AFR) | Exact closed-form via caliper support function — O(n log n) |
| Constrained | Min-area with min/max aspect ratio bounds |

### Nesting

Determines whether one polygon fits inside another (stub).

## GPU acceleration

Optional WebGPU compute shaders (WGSL) for oriented LIR. Enabled via the `gpu` feature flag. Supports batch candidate evaluation and signed-distance-field expansion.

## Bindings

| Language | Crate | Description |
|---|---|---|
| Rust | `ige-core` | Core library (`ige_core`) |
| C | `ige-c` | C FFI via `libc` |
| Python | `ige-py` | Python wheel via PyO3 |

## QGIS integration

Processing scripts in `gis/qgis/scripts/`:

| Script | Solver |
|---|---|
| `axis_aligned_lir.py` | Largest axis-aligned inscribed rectangle |
| `oriented_lir.py` | Largest oriented (rotated) inscribed rectangle |

## Installation

### Rust

```toml
[dependencies]
ige-core = { git = "https://github.com/GIS-Inscribed-Geometry/IGE" }
```

### Python

```bash
pip install ige-py
```

Or from source:

```bash
cd crates/ige-py
pip install -e .
```

### C

```c
#include "ige.h"
// link against libige_c.a
```

## Usage (Rust)

```rust
use ige_core::prelude::*;
use geo_types::Polygon;

let poly: Polygon<f64> = /* ... */;

// Largest axis-aligned inscribed rectangle
let opts = AxisAlignedOptions::default();
let rect = solve_vertex_grid(&poly, &opts);

// Largest oriented (rotated) inscribed rectangle
let opts = LirOrientedOptions::default();
let result = solve_lir_oriented(&poly, &opts).unwrap();
if let Some(rect) = result.rect {
    println!("angle: {} deg, area: {}", result.angle_deg, rect.area());
}

// Maximum inscribed circle
let opts = MicOptions::default();
let mic = maximum_inscribed_circle(&poly, &opts).unwrap();

// Oriented bounding box — min area
let opts = ObbOptions::default();
let obb = solve_obb(&poly, &opts).unwrap();

// Oriented bounding box — aspect-ratio fit (AFR)
let afr = solve_obb_aspect_fit(&poly, 297.0, 210.0).unwrap();
```

## Architecture

```
crates/
  ige-core/     — Core algorithms, types, GPU module
    src/
      solvers/  — LIR, LER, MIC, OBB, Nested
      algorithms/ — Unified LirSolver trait
      gpu/        — WebGPU compute shaders
      shared/     — Rectangle, errors, rotation helpers
  ige-c/        — C FFI bindings
  ige-py/       — Python bindings (PyO3)
gis/
  qgis/         — QGIS Processing scripts
docs/
  book/         — mdBook documentation
```

## Tuning

Parameter evolution via the `openevolve/` framework:

```bash
openevolve run_ige_evolution.py
```

## Benchmarks

Criterion benchmarks in `crates/ige-core/benches/`:

| Benchmark | Scope |
|---|---|
| `lir_axis_aligned_bench` | Vertex grid, SDF, histogram methods |
| `lir_oriented_bench` | Oriented solver variants |
| `mic_bench` | Exact, grid, fallback |
| `ler_axis_aligned_bench` | Empty rectangle solvers |
| `real_world_bench` | Real-world polygon data |

Run:

```bash
cargo bench
```

## Limitations

- Oriented LIR and LER solvers are approximate (grid-based heuristic + local polish). Only axis-aligned LIR has an exact variant (B.C.R.S. backend).
- Nesting solvers are stubs — not yet implemented.
- GPU acceleration is experimental and requires a WebGPU-compatible device.
- Concave polygons with many holes may degrade performance on some methods.

## Documentation

Full API reference and developer guide available as an mdBook:

```bash
cd docs/book
mdbook serve
```

## License

MIT. See [LICENSE](LICENSE).
