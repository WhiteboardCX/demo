use std::{f32::MIN, f64::consts::FRAC_PI_2, num::NonZeroUsize, sync::Arc, time::Instant, vec};

use anyhow::Context;
use ink_stroke_modeler_rs::{ModelerError, ModelerInput, ModelerInputEventType, ModelerParams, StrokeModeler};
use log::{debug, error, info, warn};
use path::PointPath;
use vello::{
    kurbo::{Affine, BezPath, Circle, Ellipse, Line, Point, RoundedRect, Stroke, Vec2},
    peniko::{color::palette, Color, Fill},
    util::{RenderContext, RenderSurface},
    wgpu, AaConfig, Renderer, RendererOptions, Scene,
};
use winit::{
    application::ApplicationHandler,
    dpi::{LogicalSize, PhysicalPosition},
    event::{ButtonSource, ElementState, MouseButton, MouseScrollDelta, PointerSource, ToolButton, WindowEvent},
    event_loop::{ActiveEventLoop, EventLoop},
    keyboard::{self, Key, NamedKey},
    window::{Window, WindowAttributes, WindowId},
};

mod path;

struct DemoApp<'s> {
    context: RenderContext,
    state: RenderState<'s>,
    renderers: Vec<Option<Renderer>>,
    scene: Scene,
    canvas: Scene,
    transform: Affine,
    prior_position: Option<Vec2>,
    mouse_down: bool,
    stroke: StrokeBuilder,
}

#[derive(Default)]
struct StrokeBuilder {
    scene: Scene,
    modeler: StrokeModeler,
    start_time: Option<Instant>,
    path: PointPath,
    done: bool,
}

impl StrokeBuilder {
    fn new() -> Self {
        let mut b = Self::default();
        b.modeler
            .reset_w_params(ModelerParams {
                sampling_min_output_rate: 120.,
                sampling_end_of_stroke_stopping_distance: 0.01,
                ..ModelerParams::suggested()
            })
            .unwrap();
        b
    }

    fn reset(&mut self) {
        self.scene.reset();
        self.modeler.reset();
        self.path.reset();
        self.start_time = None;
        self.done = false;
    }

    fn add_point(&mut self, x: f64, y: f64, pressure: f64, done: bool) {
        if self.done {
            self.reset();
            if done {
                return;
            }
        }

        let now = Instant::now();
        let (time, event_type) = if let Some(start_time) = self.start_time {
            (
                now.duration_since(start_time).as_secs_f64(),
                if done {
                    self.done = true;
                    ModelerInputEventType::Up
                } else {
                    ModelerInputEventType::Move
                },
            )
        } else {
            self.start_time = Some(now);
            (0., ModelerInputEventType::Down)
        };
        let input = ModelerInput {
            event_type,
            time,
            pressure,
            pos: (x, y),
        };
        // info!("{input:?}");
        match self.modeler.update(input) {
            Ok(results) => {
                const MIN_PRESSURE: f64 = 0.05;
                let mut start = 0;
                while start < results.len() && results[start].pressure < MIN_PRESSURE {
                    start += 1;
                }
                let mut end = results.len();
                while end > 0 && results[end - 1].pressure < MIN_PRESSURE {
                    end -= 1;
                }
                for i in start..end {
                    let res = &results[i];
                    self.path.add_point(res.pos.0, res.pos.1, res.pressure * 5.);
                }
            }
            Err(ModelerError::Element { src }) => {
                self.done = true;
                warn!("ink-stroke-modeler error: {src}");
            }
            Err(e) => {
                warn!("Other error: {e}");
            }
        }
    }

    fn draw(&mut self) {
        // for p in self.path.points.iter() {
        //     print!("({}, {}, {}), ", p.point.x, p.point.y, p.r);
        // }
        // print!("\n");
        let bez = self.path.bez_path();
        // TODO append predicted points?
        self.scene.reset();
        let color = palette::css::BLACK;
        // let stroke = Stroke::new(1.);
        // self.scene
        //     .stroke(&stroke, Affine::IDENTITY, color, None, &bez);
        self.scene.fill(Fill::NonZero, Affine::IDENTITY, color, None, &bez);
    }

    fn in_progress(&self) -> bool {
        self.start_time.is_some() && !self.done
    }
}

/// Simple struct to hold the state of the renderer
pub struct ActiveRenderState<'s> {
    // The fields MUST be in this order, so that the surface is dropped before the window
    surface: RenderSurface<'s>,
    window: Arc<dyn Window>,
}

enum RenderState<'s> {
    Active(ActiveRenderState<'s>),
    // Cache a window so that it can be reused when the app is resumed after being suspended
    Suspended(Option<Arc<dyn Window>>),
}

fn create_winit_window(event_loop: &dyn ActiveEventLoop) -> Arc<dyn Window> {
    let attr = WindowAttributes::default()
        .with_surface_size(LogicalSize::new(1044, 800))
        .with_resizable(true)
        .with_title("Drawing App Demo");
    Arc::from(event_loop.create_window(attr).unwrap())
}

/// Helper function that creates a vello `Renderer` for a given `RenderContext` and `RenderSurface`
fn create_vello_renderer(render_cx: &RenderContext, surface: &RenderSurface<'_>) -> Renderer {
    let backend: wgpu::Backend = render_cx.devices[surface.dev_id].adapter().get_info().backend;
    info!("Creating vello renderer using backend `{backend}`.");
    Renderer::new(&render_cx.devices[surface.dev_id].device, RendererOptions {
        surface_format: Some(surface.format),
        use_cpu: false,
        antialiasing_support: vello::AaSupport::area_only(),
        num_init_threads: NonZeroUsize::new(default_threads()),
    })
    .expect("Couldn't create renderer")
}

fn default_threads() -> usize {
    #[cfg(target_os = "macos")]
    return 1;
    #[cfg(not(target_os = "macos"))]
    return 0;
}

impl ApplicationHandler for DemoApp<'_> {
    fn can_create_surfaces(&mut self, event_loop: &dyn ActiveEventLoop) {
        let RenderState::Suspended(cached_window) = &mut self.state else {
            return;
        };

        // Get the winit window cached in a previous Suspended event or else create a new window
        let window = cached_window.take().unwrap_or_else(|| create_winit_window(event_loop));

        // Create a vello Surface
        let size = window.surface_size();
        let surface_future =
            self.context
                .create_surface(window.clone(), size.width, size.height, wgpu::PresentMode::AutoVsync);
        let surface = pollster::block_on(surface_future).expect("Error creating surface");

        // Create a vello Renderer for the surface (using its device id)
        self.renderers.resize_with(self.context.devices.len(), || None);
        self.renderers[surface.dev_id].get_or_insert_with(|| create_vello_renderer(&self.context, &surface));

        // Save the Window and Surface to a state variable
        self.state = RenderState::Active(ActiveRenderState { window, surface });
    }

    fn window_event(
        &mut self, event_loop: &dyn ActiveEventLoop, window_id: WindowId, event: winit::event::WindowEvent,
    ) {
        // Ignore the event (return from the function) if
        //   - we have no render_state
        //   - OR the window id of the event doesn't match the window id of our render_state
        //
        // Else extract a mutable reference to the render state from its containing option for use below
        let render_state = match &mut self.state {
            RenderState::Active(state) if state.window.id() == window_id => state,
            _ => return,
        };

        match event {
            // Exit the event loop when a close is requested (e.g. window's close button is pressed)
            WindowEvent::CloseRequested => event_loop.exit(),

            WindowEvent::SurfaceResized(size) => {
                self.context
                    .resize_surface(&mut render_state.surface, size.width, size.height);
            }

            WindowEvent::MouseWheel { delta, .. } => {
                let e = match delta {
                    MouseScrollDelta::LineDelta(_, y) => y as f64,
                    MouseScrollDelta::PixelDelta(pd) => pd.y / 20.,
                };
                let Some(prior_position) = self.prior_position else {
                    return;
                };
                let scale = 1.05_f64.powf(e);
                self.transform = Affine::translate(prior_position)
                    * Affine::scale(scale)
                    * Affine::translate(-prior_position)
                    * self.transform;
                render_state.window.request_redraw();
            }

            WindowEvent::PointerMoved {
                position,
                source: PointerSource::Mouse,
                ..
            } => {
                let position = Vec2::new(position.x, position.y);
                if self.mouse_down {
                    if let Some(prior) = self.prior_position {
                        self.transform = Affine::translate(position - prior) * self.transform;
                        render_state.window.request_redraw();
                    }
                }
                self.prior_position = Some(position);
            }

            WindowEvent::PointerMoved {
                position,
                source: PointerSource::Tool(state),
                ..
            } => {
                // println!("{state:?}");
                if !self.stroke.in_progress() && !state.contact {
                    return;
                }
                let done = !state.contact;
                let point = self.transform.inverse() * Point::new(position.x, position.y);
                self.stroke
                    .add_point(point.x, point.y, state.force.normalized(state.angle), done);
                if self.stroke.done {
                    self.canvas.append(&self.stroke.scene, None);
                    self.stroke.reset();
                }

                render_state.window.request_redraw();
            }

            WindowEvent::PointerButton {
                state,
                button: ButtonSource::Mouse(MouseButton::Left),
                ..
            } => {
                self.mouse_down = state.is_pressed();
            }

            WindowEvent::KeyboardInput { event, .. } => {
                if !event.state.is_pressed() {
                    return;
                }
                match event.logical_key {
                    Key::Named(NamedKey::Space) => {
                        self.transform = Affine::IDENTITY;
                        render_state.window.request_redraw();
                    }
                    _ => {}
                }
            }

            WindowEvent::RedrawRequested => {
                self.scene.reset();
                self.scene.append(&self.canvas, Some(self.transform));

                self.stroke.draw();
                self.scene.append(&self.stroke.scene, Some(self.transform));

                // Get the RenderSurface (surface + config)
                let surface = &render_state.surface;

                // Get the window size
                let width = surface.config.width;
                let height = surface.config.height;

                // Get a handle to the device
                let device_handle = &self.context.devices[surface.dev_id];

                // Get the surface's texture
                let surface_texture = surface
                    .surface
                    .get_current_texture()
                    .expect("failed to get surface texture");

                // Render to the surface's texture
                self.renderers[surface.dev_id]
                    .as_mut()
                    .unwrap()
                    .render_to_surface(
                        &device_handle.device,
                        &device_handle.queue,
                        &self.scene,
                        &surface_texture,
                        &vello::RenderParams {
                            base_color: palette::css::WHITE,
                            width,
                            height,
                            antialiasing_method: AaConfig::Area,
                        },
                    )
                    .expect("failed to render to surface");

                // Queue the texture to be presented on the surface
                surface_texture.present();

                device_handle.device.poll(wgpu::Maintain::Poll);
            }
            _ => {}
        }
    }

    fn destroy_surfaces(&mut self, _event_loop: &dyn ActiveEventLoop) {
        if let RenderState::Active(state) = &self.state {
            self.state = RenderState::Suspended(Some(state.window.clone()));
        }
    }
}

fn main() -> anyhow::Result<()> {
    env_logger::builder()
        .format_timestamp(Some(env_logger::TimestampPrecision::Millis))
        .filter_module("demo", log::LevelFilter::Info)
        .filter_level(log::LevelFilter::Warn)
        .init();
    let event_loop = EventLoop::builder().build()?;
    let context = RenderContext::new();
    info!("Hello, world!");
    let app = DemoApp {
        context,
        state: RenderState::Suspended(None),
        renderers: vec![None],
        scene: Scene::new(),
        canvas: Scene::new(),
        stroke: StrokeBuilder::new(),
        transform: Affine::IDENTITY,
        prior_position: None,
        mouse_down: false,
    };
    event_loop.run_app(app).context("Could not run event loop")?;

    Ok(())
}

/*
Adapted from the vello examples (https://github.com/linebender/vello/examples), which is licensed as

    Copyright 2020 the Vello Authors

    Permission is hereby granted, free of charge, to any
    person obtaining a copy of this software and associated
    documentation files (the "Software"), to deal in the
    Software without restriction, including without
    limitation the rights to use, copy, modify, merge,
    publish, distribute, sublicense, and/or sell copies of
    the Software, and to permit persons to whom the Software
    is furnished to do so, subject to the following
    conditions:

    The above copyright notice and this permission notice
    shall be included in all copies or substantial portions
    of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF
    ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
    TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
    PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
    SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
    CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
    OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
    IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
    DEALINGS IN THE SOFTWARE.
*/
