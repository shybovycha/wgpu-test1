use winit::{
    event::*,
    event_loop::{ControlFlow, EventLoop},
    window::WindowBuilder,
};

use pollster::FutureExt as _;
use wgpu::util::DeviceExt;
use std::time::Instant;
use image::GenericImageView;

#[repr(C)]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
struct Vertex {
    position: [f32; 3],
    uv: [f32; 2],
}

impl Vertex {
    fn desc<'a>() -> wgpu::VertexBufferLayout<'a> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Vertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &[
                // position
                wgpu::VertexAttribute {
                    offset: 0,
                    shader_location: 0,
                    format: wgpu::VertexFormat::Float32x3,
                },

                // UV
                wgpu::VertexAttribute {
                    offset: std::mem::size_of::<[f32; 3]>() as wgpu::BufferAddress,
                    shader_location: 1,
                    format: wgpu::VertexFormat::Float32x2,
                }
            ]
        }
    }
}

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
struct CameraUniform {
    // We can't use Matrix4 with bytemuck directly so we'll have
    // to convert the Matrix4 into a 4x4 f32 array
    view: [f32; 16],
    projection: [f32; 16],
}

impl CameraUniform {
    fn new() -> Self {
        Self {
            view: *glam::Mat4::IDENTITY.as_ref(),
            projection: *glam::Mat4::IDENTITY.as_ref(),
        }
    }
}

struct Camera {
    position: glam::Vec3,
    up: glam::Vec3,
    right: glam::Vec3,
    forward: glam::Vec3,

    fov_y: f32,
    aspect_ratio: f32,

    z_near: f32,
    z_far: f32,

    view_matrix: glam::Mat4,
    projection_matrix: glam::Mat4,
}

impl Camera {
    fn set_aspect_ratio(&mut self, aspect_ratio: f32) -> &mut Self {
        self.aspect_ratio = aspect_ratio;

        self.update_matrices();

        self
    }

    fn translate(&mut self, delta_pos: glam::Vec3) -> &mut Self {
        self.position += delta_pos;

        self.update_matrices();

        self
    }

    fn rotate(&mut self, horizontal_angle: f32, vertical_angle: f32) -> &mut Self {
        let rotation_matrix_vertical = glam::Mat3::from_axis_angle(self.right.normalize(), f32::to_radians(vertical_angle));
        let rotation_matrix_horizontal = glam::Mat3::from_axis_angle(self.up.normalize(), f32::to_radians(horizontal_angle));

        self.forward = rotation_matrix_vertical * rotation_matrix_horizontal * self.forward;

        self.right = rotation_matrix_horizontal * self.right;

        self.update_matrices();

        self
    }

    fn update_matrices(&mut self) -> &mut Self {
        self.view_matrix = glam::Mat4::look_at_rh(self.position, self.position + self.forward, self.up);
        self.projection_matrix = glam::Mat4::perspective_rh_gl(f32::to_radians(self.fov_y), self.aspect_ratio, self.z_near, self.z_far);

        self
    }
}

impl Default for Camera {
    fn default() -> Camera {
        Camera {
            position: glam::Vec3::ZERO,
            up: glam::Vec3::Y,
            right: glam::Vec3::X,
            forward: glam::Vec3::NEG_Z,
            fov_y: 45.0,
            aspect_ratio: 1.0,
            z_near: 0.01,
            z_far: 100.0,
            view_matrix: glam::Mat4::IDENTITY,
            projection_matrix: glam::Mat4::IDENTITY,
        }
    }
}

fn create_rtt_target_texture(config: &wgpu::SurfaceConfiguration, device: &wgpu::Device) -> wgpu::Texture {
    device.create_texture(
        &wgpu::TextureDescriptor {
            size: wgpu::Extent3d {
                width: config.width,
                height: config.height,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Bgra8UnormSrgb,
            usage: wgpu::TextureUsages::TEXTURE_BINDING
                | wgpu::TextureUsages::COPY_DST
                | wgpu::TextureUsages::RENDER_ATTACHMENT,
            label: Some("RTT target texture"),
        }
    )
}

fn create_depth_stencil_texture(config: &wgpu::SurfaceConfiguration, device: &wgpu::Device) -> wgpu::TextureView {
    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("Depth & stencil buffer"),
        size: wgpu::Extent3d {
            width: config.width,
            height: config.height,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Depth32Float,
        usage: wgpu::TextureUsages::TEXTURE_BINDING
            | wgpu::TextureUsages::COPY_DST
            | wgpu::TextureUsages::RENDER_ATTACHMENT,
    });

    texture.create_view(&wgpu::TextureViewDescriptor::default())
}

fn main() {
    env_logger::init();

    let event_loop = EventLoop::new();

    let mut window_builder = WindowBuilder::new();

    window_builder = window_builder.with_title("Sample 7: Framebuffer");
    window_builder = window_builder.with_min_inner_size(winit::dpi::LogicalSize::new(1024.0, 768.0));

    let window = window_builder.build(&event_loop).unwrap();

    let window_size = window.inner_size();

    let instance = wgpu::Instance::new(wgpu::Backends::all());

    let surface = unsafe { instance.create_surface(&window) };

    let adapter = instance.request_adapter(
        &wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::default(),
            compatible_surface: Some(&surface),
            force_fallback_adapter: false,
        },
    ).block_on().unwrap();

    let adapter_info = adapter.get_info();

    println!("Adapter:\n\tname: {}\n\tdriver: {}\n\tbackend: {:?}\n", adapter_info.name, adapter_info.driver, adapter_info.backend);

    window.set_cursor_grab(winit::window::CursorGrabMode::Confined)
        .or_else(|_e| window.set_cursor_grab(winit::window::CursorGrabMode::Locked))
        .unwrap();

    window.set_cursor_position(winit::dpi::PhysicalPosition::new(window_size.width as f32 / 2.0, window_size.height as f32 / 2.0)).unwrap();

    window.set_cursor_visible(false);

    let (device, queue) = adapter.request_device(
        &wgpu::DeviceDescriptor {
            features: wgpu::Features::empty(),
            limits: wgpu::Limits::default(),
            label: None,
        },
        None,
    ).block_on().unwrap();

    let mut config = wgpu::SurfaceConfiguration {
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
        format: surface.get_supported_formats(&adapter)[0],
        width: window_size.width,
        height: window_size.height,
        present_mode: wgpu::PresentMode::Fifo,
        alpha_mode: wgpu::CompositeAlphaMode::Auto,
    };

    surface.configure(&device, &config);

    let triangle_texture_bytes = include_bytes!("texture.jpg");
    let triangle_texture_image = image::load_from_memory(triangle_texture_bytes).unwrap();
    let triangle_texture_rgba = triangle_texture_image.to_rgba8();

    let triangle_texture_dimensions = triangle_texture_image.dimensions();

    let triangle_texture_size = wgpu::Extent3d {
        width: triangle_texture_dimensions.0,
        height: triangle_texture_dimensions.1,
        depth_or_array_layers: 1,
    };

    let triangle_texture = device.create_texture(
        &wgpu::TextureDescriptor {
            size: triangle_texture_size,
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8UnormSrgb,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            label: Some("Triangle texture"),
        }
    );

    queue.write_texture(
        wgpu::ImageCopyTexture {
            texture: &triangle_texture,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        &triangle_texture_rgba,
        wgpu::ImageDataLayout {
            offset: 0,
            bytes_per_row: std::num::NonZeroU32::new(4 * triangle_texture_dimensions.0),
            rows_per_image: std::num::NonZeroU32::new(triangle_texture_dimensions.1),
        },
        triangle_texture_size,
    );

    let triangle_texture_view = triangle_texture.create_view(&wgpu::TextureViewDescriptor::default());

    let triangle_texture_sampler = device.create_sampler(&wgpu::SamplerDescriptor {
        address_mode_u: wgpu::AddressMode::ClampToEdge,
        address_mode_v: wgpu::AddressMode::ClampToEdge,
        address_mode_w: wgpu::AddressMode::ClampToEdge,
        mag_filter: wgpu::FilterMode::Linear,
        min_filter: wgpu::FilterMode::Nearest,
        mipmap_filter: wgpu::FilterMode::Nearest,
        ..Default::default()
    });

    let texture_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        entries: &[
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    multisampled: false,
                    view_dimension: wgpu::TextureViewDimension::D2,
                    sample_type: wgpu::TextureSampleType::Float { filterable: true },
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::FRAGMENT,
                // This should match the filterable field of the
                // corresponding Texture entry above.
                ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                count: None,
            },
        ],
        label: Some("Triangle bind group layout"),
    });

    let triangle_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        layout: &texture_bind_group_layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::TextureView(&triangle_texture_view),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: wgpu::BindingResource::Sampler(&triangle_texture_sampler),
            }
        ],
        label: Some("Triangle bind group"),
    });

    let rtt_texture = create_rtt_target_texture(&config, &device);
    let rtt_texture_view1 = rtt_texture.create_view(&wgpu::TextureViewDescriptor::default());
    let rtt_texture_view2 = rtt_texture.create_view(&wgpu::TextureViewDescriptor::default());

    let mut depth_view = create_depth_stencil_texture(&config, &device);

    let rtt_sampler = device.create_sampler(&wgpu::SamplerDescriptor {
        address_mode_u: wgpu::AddressMode::ClampToEdge,
        address_mode_v: wgpu::AddressMode::ClampToEdge,
        address_mode_w: wgpu::AddressMode::ClampToEdge,
        mag_filter: wgpu::FilterMode::Linear,
        min_filter: wgpu::FilterMode::Nearest,
        mipmap_filter: wgpu::FilterMode::Nearest,
        ..Default::default()
    });

    let rtt_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        entries: &[
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    multisampled: false,
                    view_dimension: wgpu::TextureViewDimension::D2,
                    sample_type: wgpu::TextureSampleType::Float { filterable: true },
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::FRAGMENT,
                // This should match the filterable field of the
                // corresponding Texture entry above.
                ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                count: None,
            },
        ],
        label: Some("RTT texture bind group layout"),
    });

    let mut rtt_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        layout: &rtt_bind_group_layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::TextureView(&rtt_texture_view2),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: wgpu::BindingResource::Sampler(&rtt_sampler),
            }
        ],
        label: Some("RTT bind group"),
    });

    const TRIANGLE_VERTICES: &[Vertex] = &[
        Vertex { position: [ 0.0, 0.5, 0.0 ], uv: [ 0.5, 0.0 ] },
        Vertex { position: [ -0.5, -0.5, 0.0 ], uv: [ 0.0, 1.0 ] },
        Vertex { position: [ 0.5, -0.5, 0.0 ], uv: [ 1.0, 1.0 ] },
    ];

    const TRIANGLE_INDICES: &[u16] = &[
        2, 1, 0,
    ];

    let triangle_vertex_buffer = device.create_buffer_init(
        &wgpu::util::BufferInitDescriptor {
            label: Some("Triangle vertex buffer"),
            contents: bytemuck::cast_slice(TRIANGLE_VERTICES),
            usage: wgpu::BufferUsages::VERTEX,
        }
    );

    let triangle_index_buffer = device.create_buffer_init(
        &wgpu::util::BufferInitDescriptor {
            label: Some("Triangle vertex buffer"),
            contents: bytemuck::cast_slice(TRIANGLE_INDICES),
            usage: wgpu::BufferUsages::INDEX,
        }
    );

    const CUBE_VERTICES: &[Vertex] = &[
        Vertex { position: [ -0.5, 0.5, -0.5 ], uv: [ 0.0, 0.0 ] },
        Vertex { position: [ -0.5, -0.5, -0.5 ], uv: [ 0.0, 1.0 ] },
        Vertex { position: [ 0.5, -0.5, -0.5 ], uv: [ 1.0, 1.0 ] },
        Vertex { position: [ 0.5, 0.5, -0.5 ], uv: [ 1.0, 0.0 ] },

        Vertex { position: [ -0.5, 0.5, 0.5 ], uv: [ 0.0, 0.0 ] },
        Vertex { position: [ -0.5, -0.5, 0.5 ], uv: [ 0.0, 1.0 ] },
        Vertex { position: [ 0.5, -0.5, 0.5 ], uv: [ 1.0, 1.0 ] },
        Vertex { position: [ 0.5, 0.5, 0.5 ], uv: [ 1.0, 0.0 ] },
    ];

    const CUBE_INDICES: &[u16] = &[
        // front
        0, 1, 3,
        3, 1, 2,

        // top
        0, 3, 7,
        7, 4, 0,

        // back
        7, 6, 5,
        5, 4, 7,

        // left
        0, 5, 1,
        0, 4, 5,

        // right
        3, 2, 6,
        6, 7, 3,

        // bottom
        1, 2, 6,
        6, 5, 1
    ];

    let cube_vertex_buffer = device.create_buffer_init(
        &wgpu::util::BufferInitDescriptor {
            label: Some("Cube vertex buffer"),
            contents: bytemuck::cast_slice(CUBE_VERTICES),
            usage: wgpu::BufferUsages::VERTEX,
        }
    );

    let cube_index_buffer = device.create_buffer_init(
        &wgpu::util::BufferInitDescriptor {
            label: Some("Cube vertex buffer"),
            contents: bytemuck::cast_slice(CUBE_INDICES),
            usage: wgpu::BufferUsages::INDEX,
        }
    );

    let mut camera_uniform = CameraUniform::new();

    // set the render to texture camera as the "default" camera settings
    let rtt_camera_uniform = CameraUniform::new();

    let camera_buffer = device.create_buffer_init(
        &wgpu::util::BufferInitDescriptor {
            label: Some("Camera buffer"),
            contents: bytemuck::cast_slice(&[camera_uniform]),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        }
    );

    let camera_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        entries: &[
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }
        ],
        label: Some("Camera bind group layout"),
    });

    let camera_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        layout: &camera_bind_group_layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: camera_buffer.as_entire_binding(),
            }
        ],
        label: Some("Camera bind group"),
    });

    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("Shader"),
        source: wgpu::ShaderSource::Wgsl(include_str!("shader.wgsl").into()),
    });

    let rtt_render_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("RTT render pipeline layout"),
        bind_group_layouts: &[
            &camera_bind_group_layout,
            &rtt_bind_group_layout,
        ],
        push_constant_ranges: &[],
    });

    let rtt_render_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some("RTT render pipeline"),
        layout: Some(&rtt_render_pipeline_layout),
        vertex: wgpu::VertexState {
            module: &shader,
            entry_point: "vs_main",
            buffers: &[
                Vertex::desc(),
            ],
        },
        fragment: Some(wgpu::FragmentState {
            module: &shader,
            entry_point: "fs_main",
            targets: &[Some(wgpu::ColorTargetState {
                format: config.format,
                blend: Some(wgpu::BlendState::REPLACE),
                write_mask: wgpu::ColorWrites::ALL,
            })],
        }),
        primitive: wgpu::PrimitiveState {
            topology: wgpu::PrimitiveTopology::TriangleList,
            front_face: wgpu::FrontFace::Ccw,
            cull_mode: Some(wgpu::Face::Front),
            ..Default::default()
        },
        depth_stencil: Some(wgpu::DepthStencilState {
            format: wgpu::TextureFormat::Depth32Float,
            depth_write_enabled: true,
            depth_compare: wgpu::CompareFunction::Less,
            stencil: wgpu::StencilState::default(),
            bias: wgpu::DepthBiasState::default(),
        }),
        multisample: wgpu::MultisampleState::default(),
        multiview: None,
    });

    let mut camera = Camera {
        position: glam::Vec3::new(0.0, 0.0, 1.5),
        right: glam::Vec3::X,
        up: glam::Vec3::Y,
        forward: glam::Vec3::NEG_Z,
        fov_y: 45.0,
        z_near: 0.01,
        z_far: 100.0,
        ..Default::default()
    };

    const CAMERA_ROTATE_SPEED: f32 = 100.0;
    const CAMERA_MOVE_SPEED: f32 = 0.1;

    let mut last_frame_inst = Instant::now();

    const LEFT: u32 = 1 << 1;
    const RIGHT: u32 = 1 << 2;
    const UP: u32 = 1 << 3;
    const DOWN: u32 = 1 << 4;

    let mut movement_input: u32 = 0x00;

    event_loop.run(move |event, _, control_flow| match event {
        Event::WindowEvent {
            ref event,
            window_id,
        } if window_id == window.id() => match event {
            WindowEvent::CloseRequested => *control_flow = ControlFlow::Exit,

            WindowEvent::Resized(physical_size) => {
                let new_size = *physical_size;

                if new_size.width > 0 && new_size.height > 0 {
                    config.width = new_size.width;
                    config.height = new_size.height;
                    surface.configure(&device, &config);

                    camera.set_aspect_ratio((config.width / config.height) as f32);

                    let rtt_texture = create_rtt_target_texture(&config, &device);
                    let rtt_texture_view = rtt_texture.create_view(&wgpu::TextureViewDescriptor::default());

                    let rtt_sampler = device.create_sampler(&wgpu::SamplerDescriptor {
                        address_mode_u: wgpu::AddressMode::ClampToEdge,
                        address_mode_v: wgpu::AddressMode::ClampToEdge,
                        address_mode_w: wgpu::AddressMode::ClampToEdge,
                        mag_filter: wgpu::FilterMode::Linear,
                        min_filter: wgpu::FilterMode::Nearest,
                        mipmap_filter: wgpu::FilterMode::Nearest,
                        ..Default::default()
                    });

                    rtt_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
                        layout: &rtt_bind_group_layout,
                        entries: &[
                            wgpu::BindGroupEntry {
                                binding: 0,
                                resource: wgpu::BindingResource::TextureView(&rtt_texture_view),
                            },
                            wgpu::BindGroupEntry {
                                binding: 1,
                                resource: wgpu::BindingResource::Sampler(&rtt_sampler),
                            }
                        ],
                        label: Some("RTT bind group"),
                    });

                    depth_view = create_depth_stencil_texture(&config, &device);
                }
            },

            WindowEvent::ScaleFactorChanged { scale_factor, new_inner_size, .. } => {
                let new_size = **new_inner_size;

                if new_size.width > 0 && new_size.height > 0 {
                    config.width = new_size.width * (*scale_factor as u32);
                    config.height = new_size.height * (*scale_factor as u32);

                    surface.configure(&device, &config);

                    camera.set_aspect_ratio((config.width / config.height) as f32 * (*scale_factor as f32));

                    let rtt_texture = create_rtt_target_texture(&config, &device);
                    let rtt_texture_view = rtt_texture.create_view(&wgpu::TextureViewDescriptor::default());

                    let rtt_sampler = device.create_sampler(&wgpu::SamplerDescriptor {
                        address_mode_u: wgpu::AddressMode::ClampToEdge,
                        address_mode_v: wgpu::AddressMode::ClampToEdge,
                        address_mode_w: wgpu::AddressMode::ClampToEdge,
                        mag_filter: wgpu::FilterMode::Linear,
                        min_filter: wgpu::FilterMode::Nearest,
                        mipmap_filter: wgpu::FilterMode::Nearest,
                        ..Default::default()
                    });

                    rtt_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
                        layout: &rtt_bind_group_layout,
                        entries: &[
                            wgpu::BindGroupEntry {
                                binding: 0,
                                resource: wgpu::BindingResource::TextureView(&rtt_texture_view),
                            },
                            wgpu::BindGroupEntry {
                                binding: 1,
                                resource: wgpu::BindingResource::Sampler(&rtt_sampler),
                            }
                        ],
                        label: Some("RTT bind group"),
                    });

                    depth_view = create_depth_stencil_texture(&config, &device);
                }
            },

            WindowEvent::KeyboardInput {
                input: KeyboardInput {
                    state,
                    virtual_keycode: Some(key_code),
                    ..
                },
                ..
            } => {
                if *state == ElementState::Pressed {
                    match key_code {
                        VirtualKeyCode::W => {
                            movement_input |= UP;
                        },

                        VirtualKeyCode::S => {
                            movement_input |= DOWN;
                        },

                        VirtualKeyCode::A => {
                            movement_input |= LEFT;
                        },

                        VirtualKeyCode::D => {
                            movement_input |= RIGHT;
                        },

                        _ => (),
                    }
                } else if *state == ElementState::Released {
                    match key_code {
                        VirtualKeyCode::W => {
                            movement_input &= !UP;
                        },

                        VirtualKeyCode::S => {
                            movement_input &= !DOWN;
                        },

                        VirtualKeyCode::A => {
                            movement_input &= !LEFT;
                        },

                        VirtualKeyCode::D => {
                            movement_input &= !RIGHT;
                        },

                        _ => (),
                    }
                }
            },

            WindowEvent::CursorMoved {
                position: mouse_position,
                ..
            } => {
                let delta_time = last_frame_inst.elapsed().as_secs_f32();

                last_frame_inst = Instant::now();

                let screen_center = glam::Vec2::new(window_size.width as f32 / 2.0, window_size.height as f32 / 2.0);
                let current_mouse_position = glam::Vec2::new(mouse_position.x as f32, mouse_position.y as f32);
                let mouse_delta = screen_center - current_mouse_position;

                window.set_cursor_position(winit::dpi::PhysicalPosition::new(screen_center.x, screen_center.y)).unwrap();

                let horizontal_angle = (mouse_delta.x / window_size.width as f32 / 2.0) * delta_time * CAMERA_ROTATE_SPEED * camera.fov_y;
                let vertical_angle = (mouse_delta.y / window_size.height as f32 / 2.0) * delta_time * CAMERA_ROTATE_SPEED * camera.fov_y;

                camera.rotate(horizontal_angle, vertical_angle);
            },

            _ => {}
        },

        Event::RedrawRequested(window_id) if window_id == window.id() => {
            // update movement
            if movement_input & UP > 0 {
                camera.translate(camera.forward * CAMERA_MOVE_SPEED);
            }

            if movement_input & DOWN > 0 {
                camera.translate(-camera.forward * CAMERA_MOVE_SPEED);
            }

            if movement_input & LEFT > 0 {
                camera.translate(-camera.right * CAMERA_MOVE_SPEED);
            }

            if movement_input & RIGHT > 0 {
                camera.translate(camera.right * CAMERA_MOVE_SPEED);
            }

            // render
            let output = surface.get_current_texture().unwrap();

            let view = output.texture.create_view(&wgpu::TextureViewDescriptor::default());

            let mut command_encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("Render command encoder"),
            });

            camera_uniform.projection = *camera.projection_matrix.as_ref();
            camera_uniform.view = *camera.view_matrix.as_ref();

            {
                let mut render_pass = command_encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                    label: Some("First render pass"),

                    color_attachments: &[
                        Some(wgpu::RenderPassColorAttachment {
                            view: &rtt_texture_view1,
                            resolve_target: None,
                            ops: wgpu::Operations {
                                load: wgpu::LoadOp::Clear(
                                    wgpu::Color {
                                        r: 1.0,
                                        g: 1.0,
                                        b: 1.0,
                                        a: 1.0,
                                    }
                                ),
                                store: true,
                            },
                        }
                    )],

                    depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                        view: &depth_view,
                        depth_ops: Some(wgpu::Operations {
                            load: wgpu::LoadOp::Clear(1.0),
                            store: true,
                        }),
                        stencil_ops: None,
                    }),
                });

                queue.write_buffer(&camera_buffer, 0, bytemuck::bytes_of(&rtt_camera_uniform));

                render_pass.set_pipeline(&rtt_render_pipeline);

                render_pass.set_bind_group(0, &camera_bind_group, &[]);
                render_pass.set_bind_group(1, &triangle_bind_group, &[]);

                render_pass.set_vertex_buffer(0, triangle_vertex_buffer.slice(..));
                render_pass.set_index_buffer(triangle_index_buffer.slice(..), wgpu::IndexFormat::Uint16);

                render_pass.draw_indexed(0..(TRIANGLE_INDICES.len() as u32), 0, 0..1);
            }

            {
                let mut render_pass = command_encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                    label: Some("Final render pass"),

                    color_attachments: &[
                        // This is what @location(0) in the fragment shader targets
                        Some(wgpu::RenderPassColorAttachment {
                            view: &view,
                            resolve_target: None,
                            ops: wgpu::Operations {
                                load: wgpu::LoadOp::Clear(
                                    wgpu::Color {
                                        r: 0.1,
                                        g: 0.2,
                                        b: 0.3,
                                        a: 1.0,
                                    }
                                ),
                                store: true,
                            },
                        }
                    )],

                    depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                        view: &depth_view,
                        depth_ops: Some(wgpu::Operations {
                            load: wgpu::LoadOp::Clear(1.0),
                            store: true,
                        }),
                        stencil_ops: None,
                    }),
                });

                queue.write_buffer(&camera_buffer, 0, bytemuck::bytes_of(&camera_uniform));

                render_pass.set_pipeline(&rtt_render_pipeline);

                render_pass.set_bind_group(0, &camera_bind_group, &[]);

                render_pass.set_bind_group(1, &rtt_bind_group, &[]);

                render_pass.set_vertex_buffer(0, cube_vertex_buffer.slice(..));
                render_pass.set_index_buffer(cube_index_buffer.slice(..), wgpu::IndexFormat::Uint16);

                render_pass.draw_indexed(0..(CUBE_INDICES.len() as u32), 0, 0..1);
            }

            queue.submit(Some(command_encoder.finish()));

            output.present();
        },

        Event::MainEventsCleared => {
            window.request_redraw();
        },

        _ => {}
    });
}
