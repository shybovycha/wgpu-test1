use winit::{
    event::*,
    event_loop::{ControlFlow, EventLoop},
    window::WindowBuilder,
};

use pollster::FutureExt as _;

fn main() {
    env_logger::init();

    let event_loop = EventLoop::new();

    let mut window_builder = WindowBuilder::new();

    window_builder = window_builder.with_title("Sample 2: Init WGPU");
    window_builder = window_builder.with_min_inner_size(winit::dpi::PhysicalSize::new(800, 600));

    let window = window_builder.build(&event_loop).unwrap();

    let size = window.inner_size();

    // The instance is a handle to our GPU
    // Backends::all => Vulkan + Metal + DX12 + Browser WebGPU
    let instance = wgpu::Instance::new(wgpu::Backends::all());

    let surface = unsafe { instance.create_surface(&window) };

    let adapter = instance.request_adapter(
        &wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::default(),
            compatible_surface: Some(&surface),
            force_fallback_adapter: false,
        },
    ).block_on().unwrap();

    let (device, queue) = adapter.request_device(
        &wgpu::DeviceDescriptor {
            features: wgpu::Features::empty(),
            limits: wgpu::Limits::default(),
            label: None,
        },
        None, // Trace path
    ).block_on().unwrap();

    let mut config = wgpu::SurfaceConfiguration {
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
        format: surface.get_supported_formats(&adapter)[0],
        width: size.width,
        height: size.height,
        present_mode: wgpu::PresentMode::Fifo,
        alpha_mode: wgpu::CompositeAlphaMode::Auto,
    };

    surface.configure(&device, &config);

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
                }
            },

            WindowEvent::ScaleFactorChanged { new_inner_size, .. } => {
                let new_size = **new_inner_size;

                if new_size.width > 0 && new_size.height > 0 {
                    config.width = new_size.width;
                    config.height = new_size.height;
                    surface.configure(&device, &config);
                }
            },

            _ => {}
        },

        Event::RedrawRequested(window_id) if window_id == window.id() => {
            // render
            let output = surface.get_current_texture().unwrap();

            let view = output.texture.create_view(&wgpu::TextureViewDescriptor::default());

            let mut command_encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("Render Encoder"),
            });

            command_encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("Render Pass"),

                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.1,
                            g: 0.2,
                            b: 0.3,
                            a: 1.0,
                        }),
                        store: true,
                    },
                })],

                depth_stencil_attachment: None,
            });

            queue.submit(std::iter::once(command_encoder.finish()));

            output.present();
        },

        Event::MainEventsCleared => {
            // RedrawRequested will only trigger once, unless manually requested
            window.request_redraw();
        },

        _ => {}
    });
}
