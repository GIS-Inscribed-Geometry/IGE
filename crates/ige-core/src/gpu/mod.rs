//! GPU-accelerated rectangle candidate evaluation using WGSL compute shaders.
//!
//! This module provides optional GPU acceleration for evaluating large batches
//! of rectangle candidates in parallel. Falls back gracefully to CPU when GPU
//! is unavailable or disabled.

use anyhow::Result;
use bytemuck::{Pod, Zeroable, AnyBitPattern};
use geo_types::Polygon;
use wgpu::util::DeviceExt;

/// Maximum polygon vertices supported by GPU shader
const MAX_VERTICES: usize = 2048;

/// Workgroup size (must match shader)
const WORKGROUP_SIZE: u32 = 256;

/// Polygon data structure matching WGSL layout
#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
struct PolygonDataGpu {
    vertex_count: u32,
    _pad0: u32,
    _pad1: u32,
    _pad2: u32,
    vertices: [f32; MAX_VERTICES],
}

/// Rectangle candidate structure matching WGSL layout
#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
struct RectCandidateGpu {
    x_min: f32,
    y_min: f32,
    x_max: f32,
    y_max: f32,
}

/// Candidate result structure matching WGSL layout
#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
struct CandidateResultGpu {
    area: f32,
    is_valid: u32,
    _pad0: u32,
    _pad1: u32,
}

/// Result from GPU candidate evaluation
#[derive(Debug, Clone)]
pub struct CandidateResult {
    pub area: f32,
    pub is_valid: bool,
}

/// Rectangle candidate for evaluation
#[derive(Debug, Clone, Copy)]
pub struct RectCandidate {
    pub x_min: f64,
    pub y_min: f64,
    pub x_max: f64,
    pub y_max: f64,
}

impl RectCandidate {
    pub fn new(x_min: f64, y_min: f64, x_max: f64, y_max: f64) -> Self {
        Self { x_min, y_min, x_max, y_max }
    }

    pub fn area(&self) -> f64 {
        (self.x_max - self.x_min) * (self.y_max - self.y_min)
    }
}

/// GPU compute context for rectangle evaluation
pub struct GpuContext {
    device: wgpu::Device,
    queue: wgpu::Queue,
    pipeline: wgpu::ComputePipeline,
    bind_group_layout: wgpu::BindGroupLayout,
}

impl GpuContext {
    /// Initialize GPU context (blocking)
    pub fn new() -> Result<Self> {
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
            backends: wgpu::Backends::all(),
            ..Default::default()
        });

        let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            compatible_surface: None,
            force_fallback_adapter: false,
        }))
            .ok_or_else(|| anyhow::anyhow!("Failed to find suitable GPU adapter"))?;

        let (device, queue) = pollster::block_on(adapter.request_device(
            &wgpu::DeviceDescriptor {
                label: Some("LIRiAP GPU Device"),
                required_features: wgpu::Features::empty(),
                required_limits: wgpu::Limits::default(),
            },
            None,
        ))?;

        // Load and compile shader
        let shader_source = include_str!("shaders/oriented_lir.wgsl");
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Oriented LIR Shader"),
            source: wgpu::ShaderSource::Wgsl(shader_source.into()),
        });

        // Create bind group layout
        let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("Oriented LIR Bind Group Layout"),
            entries: &[
                // Polygon data
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // Candidates
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // Results
                wgpu::BindGroupLayoutEntry {
                    binding: 2,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: false },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
            ],
        });

        // Create pipeline layout
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Oriented LIR Pipeline Layout"),
            bind_group_layouts: &[&bind_group_layout],
            push_constant_ranges: &[],
        });

        // Create compute pipeline
        let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("Oriented LIR Pipeline"),
            layout: Some(&pipeline_layout),
            module: &shader,
            entry_point: "main",
            compilation_options: Default::default(),
        });

        Ok(Self {
            device,
            queue,
            pipeline,
            bind_group_layout,
        })
    }

    /// Evaluate rectangle candidates on GPU
    pub fn evaluate_candidates(
        &self,
        polygon: &Polygon<f64>,
        candidates: &[RectCandidate],
    ) -> Result<Vec<CandidateResult>> {
        // Prepare polygon data
        let coords = polygon.exterior().0.clone();
        if coords.len() > MAX_VERTICES / 2 {
            anyhow::bail!("Polygon has too many vertices for GPU (max {})", MAX_VERTICES / 2);
        }

        let mut poly_data = PolygonDataGpu {
            vertex_count: coords.len() as u32,
            _pad0: 0,
            _pad1: 0,
            _pad2: 0,
            vertices: [0.0; MAX_VERTICES],
        };

        for (i, coord) in coords.iter().enumerate() {
            poly_data.vertices[i * 2] = coord.x as f32;
            poly_data.vertices[i * 2 + 1] = coord.y as f32;
        }

        // Convert candidates
        let gpu_candidates: Vec<RectCandidateGpu> = candidates
            .iter()
            .map(|c| RectCandidateGpu {
                x_min: c.x_min as f32,
                y_min: c.y_min as f32,
                x_max: c.x_max as f32,
                y_max: c.y_max as f32,
            })
            .collect();

        let num_candidates = gpu_candidates.len();

        // Create GPU buffers
        let polygon_buffer = self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Polygon Buffer"),
            contents: bytemuck::cast_slice(&[poly_data]),
            usage: wgpu::BufferUsages::STORAGE,
        });

        let candidate_buffer = self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Candidate Buffer"),
            contents: bytemuck::cast_slice(&gpu_candidates),
            usage: wgpu::BufferUsages::STORAGE,
        });

        let result_buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Result Buffer"),
            size: (num_candidates * std::mem::size_of::<CandidateResultGpu>()) as u64,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });

        let staging_buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Staging Buffer"),
            size: (num_candidates * std::mem::size_of::<CandidateResultGpu>()) as u64,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });

        // Create bind group
        let bind_group = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Oriented LIR Bind Group"),
            layout: &self.bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: polygon_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: candidate_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: result_buffer.as_entire_binding(),
                },
            ],
        });

        // Dispatch compute shader
        let mut encoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("Oriented LIR Encoder"),
        });

        {
            let mut compute_pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("Oriented LIR Compute Pass"),
                timestamp_writes: None,
            });
            compute_pass.set_pipeline(&self.pipeline);
            compute_pass.set_bind_group(0, &bind_group, &[]);

            let workgroups = (num_candidates as u32 + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE;
            compute_pass.dispatch_workgroups(workgroups, 1, 1);
        }

        // Copy results to staging buffer
        encoder.copy_buffer_to_buffer(
            &result_buffer,
            0,
            &staging_buffer,
            0,
            (num_candidates * std::mem::size_of::<CandidateResultGpu>()) as u64,
        );

        self.queue.submit(Some(encoder.finish()));

        // Read back results
        let buffer_slice = staging_buffer.slice(..);
        let (tx, rx) = std::sync::mpsc::channel();
        buffer_slice.map_async(wgpu::MapMode::Read, move |result| {
            tx.send(result).unwrap();
        });

        self.device.poll(wgpu::Maintain::Wait);
        rx.recv().unwrap()?;

        let data = buffer_slice.get_mapped_range();
        let gpu_results: &[CandidateResultGpu] = bytemuck::cast_slice(&data);

        let results = gpu_results
            .iter()
            .map(|r| CandidateResult {
                area: r.area,
                is_valid: r.is_valid != 0,
            })
            .collect();

        drop(data);
        staging_buffer.unmap();

        Ok(results)
    }
}

/// Try to create GPU context, returning None if unavailable
pub fn try_create_gpu_context() -> Option<GpuContext> {
    GpuContext::new().ok()
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo_types::coord;

    #[test]
    fn test_gpu_context_creation() {
        let _ctx = try_create_gpu_context();
    }
}