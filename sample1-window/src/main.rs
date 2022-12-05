use winit::{
    event::*,
    event_loop::{ControlFlow, EventLoop},
    window::WindowBuilder,
};

fn main() {
    env_logger::init();

    let event_loop = EventLoop::new();

    let mut window_builder = WindowBuilder::new();

    window_builder = window_builder.with_title("Sample 4: Init window");
    window_builder = window_builder.with_min_inner_size(winit::dpi::PhysicalSize::new(800, 600));

    let window = window_builder.build(&event_loop).unwrap();

    event_loop.run(move |event, _, control_flow| match event {
        Event::WindowEvent {
            ref event,
            window_id,
        } if window_id == window.id() => match event {
            WindowEvent::CloseRequested => *control_flow = ControlFlow::Exit,

            _ => {}
        },

        _ => {}
    });
}
