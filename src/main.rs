use std::{f64::consts::FRAC_PI_2, num::NonZeroUsize, sync::Arc, time::Instant, vec};

use anyhow::Context;
use ink_stroke_modeler_rs::{ModelerError, ModelerInput, ModelerInputEventType, ModelerParams, StrokeModeler};
use log::{debug, error, info, warn};
use path::PointPath;
use vello::{
    kurbo::{Affine, BezPath, Circle, Ellipse, Line, RoundedRect, Stroke, Vec2},
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
        info!("{input:?}");
        match self.modeler.update(input) {
            Ok(results) => {
                for res in results {
                    println!("ModelerResult {res:?}");
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
        println!("{}", self.path.points.len());
        for p in self.path.points.iter() {
            print!("({}, {}, {}), ", p.point.x, p.point.y, p.r);
        }
        print!("\n");
        let bez = self.path.bez_path();
        // TODO using prediction
        self.scene.reset();
        let color = palette::css::BLACK;
        // let stroke = Stroke::new(1.);
        // self.scene
        //     .stroke(&stroke, Affine::IDENTITY, color, None, &bez);
        self.scene.fill(Fill::NonZero, Affine::IDENTITY, color, None, &bez);

        // let c = Circle::new((200., 200.), 50.);
        // self.scene
        //     .fill(Fill::NonZero, Affine::IDENTITY, palette::css::RED, None, &c);
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
                if state.contact && self.stroke.done {
                    self.stroke.reset();
                }
                let done = !state.contact;
                self.stroke
                    .add_point(position.x, position.y, state.force.normalized(state.angle), done);
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
                self.canvas.reset();
                add_shapes_to_scene(&mut self.canvas);
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

fn add_shapes_to_scene(scene: &mut Scene) {
    // Draw an outlined rectangle
    // let stroke = Stroke::new(6.0);
    // let rect = RoundedRect::new(10.0, 10.0, 240.0, 240.0, 20.0);
    // let rect_stroke_color: vello::peniko::color::AlphaColor<vello::peniko::color::Srgb> = Color::new([0.9804, 0.702, 0.5294, 1.]);
    // scene.stroke(&stroke, Affine::IDENTITY, rect_stroke_color, None, &rect);

    // // Draw a filled circle
    // let circle = Circle::new((420.0, 200.0), 120.0);
    // let circle_fill_color = Color::new([0.9529, 0.5451, 0.6588, 1.]);
    // scene.fill(
    //     vello::peniko::Fill::NonZero,
    //     Affine::IDENTITY,
    //     circle_fill_color,
    //     None,
    //     &circle,
    // );

    // // Draw a filled ellipse
    // let ellipse = Ellipse::new((250.0, 420.0), (100.0, 160.0), -90.0);
    // let ellipse_fill_color = Color::new([0.7961, 0.651, 0.9686, 1.]);
    // scene.fill(
    //     vello::peniko::Fill::NonZero,
    //     Affine::IDENTITY,
    //     ellipse_fill_color,
    //     None,
    //     &ellipse,
    // );

    // // Draw a straight line
    // let line = Line::new((260.0, 20.0), (620.0, 100.0));
    // let line_stroke_color = Color::new([0.5373, 0.7059, 0.9804, 1.]);
    // scene.stroke(&stroke, Affine::IDENTITY, line_stroke_color, None, &line);

    let stroke = Stroke::new(1.);

    let mut path = PointPath::new();
    let points = vec![
        // (367.4396017956852, 498.7727405310446, 0.7410734453246421),
        // (367.62719555713886, 498.6595550071471, 0.8411730567026174),
        // (368.22488577879864, 498.06262510147525, 0.9887222644293494),
        // (368.94881139625, 497.2928617680625, 1.0621944234392773),
        // (368.95387611006055, 497.2874749447513, 1.062625394059713),
        // (368.955813344427, 497.28541430151114, 1.0627902477670155),
        // (370.1157348863287, 496.0241036690352, 1.1309017735747027),
        // (371.3369064862009, 494.69288795061345, 1.1907873531794542),
        (371.34411105335437, 494.68503493524383, 1.19112009978267),
        (371.3466948540594, 494.6822188260312, 1.1912394281571845),
        (373.0780699921055, 492.8722951734045, 1.2645217313815085),
        // (374.8168651363856, 491.0906309640158, 1.3309955476511857),
        // (374.8290858530321, 491.07811043868946, 1.3314564237840125),
        // (374.8341466575077, 491.0729258702457, 1.3316472730492044),
        // (374.8382939513386, 491.0686775425021, 1.3318036654246632),
        // (377.53179853433255, 488.3757511792601, 1.4104307534580829),
        // (380.2728119797418, 485.6457623764467, 1.4835178608429442),
        // (380.2915226359081, 485.62712708637304, 1.4840446716456266),
        //     (380.2989783351232, 485.6197012296065, 1.4842545943889212),
        //     (382.3931146972412, 483.5178709430329, 1.5409379163749342),
        //     (384.51031608106945, 481.3805722935763, 1.5925005499271583),
        //     (386.51985949230584, 479.34129705353325, 1.6367815306042983),
        //     (386.543189580488, 479.31761934632635, 1.637237324902355),
        //     (386.5512420416042, 479.30944637568865, 1.6373946490060645),
        //     (390.20891707493166, 475.5122315579436, 1.7069746920958568),
        //     (393.7151435715422, 471.8352500713966, 1.7660341306144702),
        //     (393.7367893530981, 471.8125488339745, 1.766412701873981),
        //     (393.7455521130753, 471.8033584892414, 1.7665659598197943),
        //     (393.75535749783546, 471.7930740396405, 1.7667374582663065),
        //     (396.7516026657946, 468.61212636885443, 1.8202150511456052),
        //     (399.92394976874306, 465.2253822470199, 1.8854758440920028),
        //     (403.01298377163715, 461.91715053271554, 1.9504276061559862),
        //     (403.0481288851543, 461.87951067341726, 1.9511631405344165),
        //     (403.05983323019814, 461.866975483657, 1.9514080951050978),
        //     (408.8379064648509, 455.68262533331125, 2.078274949362732),
        //     (414.37332239178676, 449.7661271401624, 2.185717447604806),
        //     (414.4070513817854, 449.73007657452916, 2.186250368092412),
        //     (414.4182011483145, 449.71815949836457, 2.1864265341714653),
        //     (418.5807966326407, 445.2894150436576, 2.25023273031699),
        //     (422.7825220936163, 440.83483619227343, 2.3052502854329444),
        //     (426.7778190870542, 436.6128300047051, 2.349870513257535),
        //     (426.83356419956846, 436.5539257923155, 2.3504694750802626),
        //     (426.85382123408925, 436.5325219058775, 2.3506871238331923),
        //     (426.873368816884, 436.5118692573345, 2.3508971412855786),
        //     (435.634469294423, 427.4213531305285, 2.4547311631044693),
        //     (444.4034616073128, 418.3745214910448, 2.589549088627276),
        //     (444.45755705021367, 418.3187128791555, 2.5903870328527256),
        //     (444.47677570353017, 418.29888565511874, 2.5906847313058634),
        //     (454.72714326744523, 407.73331187052815, 2.7396863163725445),
        //     (464.10990703284233, 398.0587923024845, 2.833152948659394),
        //     (464.1693375563781, 397.99751330859357, 2.8336212459121546),
        //     (464.1889089386809, 397.97733309514814, 2.833775463775064),
        //     (474.6942868859345, 387.1108552436361, 2.905717812101986),
        //     (484.2313686313334, 377.22482252893997, 2.9579771253260128),
        //     (484.28896160959863, 377.1651214204964, 2.958277612636973),
        //     (484.30928381132304, 377.1440551762467, 2.958383642737413),
        //     (484.3276593322975, 377.12500669782605, 2.958479516614649),
        //     (496.3365541178365, 364.62527834630527, 3.0083959567992795),
        //     (507.7872370009437, 352.69086952349573, 3.025162373514955),
        //     (507.8566783334721, 352.6184945100833, 3.0252173170753247),
        //     (507.8818486953329, 352.592260797188, 3.0252372324302366),
        //     (516.1654826799396, 343.96391904473046, 3.031583930653592),
        //     (524.1350845608193, 335.6689187580461, 3.0387629035701478),
        //     (531.3995281077204, 328.1148219508966, 3.053252377736719),
        //     (531.4742357122957, 328.0371372007525, 3.053480910844944),
        //     (531.4967351481528, 328.01374148845997, 3.053549736865446),
        //     (543.3191756026242, 315.82131330296863, 3.093448645699643),
        //     (553.8872634069353, 304.9775964043257, 3.139890083693626),
        //     (553.9510088550549, 304.9121907931098, 3.1402067777603726),
        //     (553.9735865406418, 304.88902561759073, 3.140318944791301),
        //     (553.994487109468, 304.86758189667273, 3.140422778094658),
        //     (566.1369472986995, 292.61577020070513, 3.207192168675614),
        //     (577.2054105996239, 281.54448045391985, 3.278164310099414),
        //     (577.2727705522882, 281.4771070954737, 3.27861466812759),
        //     (577.2967047894385, 281.4531689036864, 3.2787746859686955),
        //     (584.8987076631731, 273.9439319884098, 3.324614611274236),
        //     (591.9574346283513, 267.0375754909108, 3.3575267154041866),
        //     (598.2078537510872, 260.97309215032914, 3.382077608748142),
        //     (598.2666795832685, 260.9160211958219, 3.382274423443495),
        //     (598.286661860384, 260.89663581408115, 3.382341277220513),
        //     (598.3067162068165, 260.8771814411555, 3.3824083705871004),
        //     (608.9309817087474, 250.7738703915118, 3.4121137416935774),
        //     (618.4883071107467, 241.75427383610995, 3.4338742110575633),
        //     (618.5446711205121, 241.7010815371401, 3.4340109210934466),
        //     (618.565133072955, 241.68177100888803, 3.4340605512542095),
        //     (628.2780130359396, 232.5023878569452, 3.4443184243878395),
        //     (636.5373725889965, 224.68111193868913, 3.434074432376172),
        //     (636.5892985126671, 224.6319395093702, 3.4339981117336817),
        //     (636.6070013997675, 224.61517526833572, 3.4339720919900962),
        //     (644.7151106240673, 216.91447490462852, 3.426043982910794),
        //     (651.6853010202917, 210.28284642505085, 3.4272374247284327),
        //     (651.726261659308, 210.24387531378815, 3.4272756854190876),
        //     (651.7414986325614, 210.22937842347042, 3.4272899180381633),
        //     (651.7551129853424, 210.21642534545987, 3.4273026349904336),
        //     (656.9253359904426, 205.2976719585433, 3.436758513385248),
        //     (661.9354626201643, 200.53113102887298, 3.4498016247841456),
        //     (666.4935514561603, 196.19430635685495, 3.4574256134541304),
        //     (666.5397686886145, 196.15033327729594, 3.457486843885502),
        //     (666.5543303125672, 196.13647891094163, 3.4575061355770593),
        //     (673.2557964750998, 189.82112780674163, 3.461972822960333),
        //     (678.9857113252818, 184.4641947521702, 3.4597284183626487),
        //     (679.0196448282941, 184.43247226522544, 3.459628300150129),
        //     (679.0321666628108, 184.42076691023757, 3.4595913562997422),
        //     (682.8278000022408, 180.94195095259664, 3.4483895179256603),
        //     (686.2669496025842, 177.84246968941017, 3.4369311433896894),
        //     (689.252111099402, 175.19534854338565, 3.4252695587074653),
        //     (689.2808553026936, 175.16986430492366, 3.425173088047954),
        //     (689.2905435384134, 175.16127574006472, 3.4251405740793968),
        //     (689.2990930129613, 175.15369763221145, 3.4251118833823573),
        //     (693.9954820866509, 171.28472684685715, 3.4142907703101772),
        //     (698.0458884157487, 168.08909072188237, 3.4116782734779774),
        //     (698.0704171378263, 168.06974256931517, 3.4116190339458137),
        //     (698.0792186347013, 168.06280087175017, 3.41159777840355),
        //     (701.7977443817075, 165.35719626852662, 3.367417279488334),
        //     (704.8067703355824, 163.30356869513525, 3.244589861217217),
        //     (704.8255471596027, 163.29075901417738, 3.2429391898082036),
        //     (704.8320940533764, 163.28629370275996, 3.2423636925298864),
        //     (706.7345233363642, 162.10011272420513, 3.061530981715121),
        //     (708.3995622532505, 161.14212985494683, 2.794945250838588),
        //     (709.8000309918481, 160.39786292739973, 2.4555661525725134),
        //     (709.8136105776279, 160.3906510979695, 2.4514336315364478),
        //     (709.8185670056674, 160.3880193416452, 2.4499253504704175),
        //     (709.8225030352845, 160.38592942708814, 2.448727588275258),
        //     (711.4970573435314, 159.32487677807848, 1.7992911064424832),
        //     (712.6485634723349, 158.32441658623753, 1.0872285571478555),
        //     (713.3624648135636, 157.53688872968175, 0.025427492593083845),
        //     (713.7472690529626, 157.00259565939726, 0.013779753256614423),
        //     (713.9217244935707, 156.6788837225431, 0.006302071628627648),
        //     (713.9763563472653, 156.50561804432672, 0.002021938871577299),
    ];
    for (x, y, r) in points.iter() {
        let x = x - points[0].0 + 20.;
        let y = y - points[0].1 + 20.;
        let scale = 10.;
        path.add_point(x * scale, y * scale, r * scale);
    }
    println!("Path points = {}", path.points.len());

    let bez = path.bez_path();
    // let line_stroke_color = palette::css::GREEN;
    // scene.stroke(&stroke, Affine::IDENTITY, line_stroke_color, None, &aux);

    let line_stroke_color = palette::css::BLACK;
    scene.stroke(&stroke, Affine::IDENTITY, line_stroke_color, None, &bez);
    // scene.fill(Fill::NonZero, Affine::IDENTITY, line_stroke_color, None, &bez);

    // let arc = vello::kurbo::Arc::new((200., 200.), (100., 100.), 0., FRAC_PI_2, 0.);
    // scene.stroke(&stroke, Affine::IDENTITY, line_stroke_color, None, &arc);
    // let line_stroke_color = palette::css::RED;
    // for p in path.points.iter() {
    //     let c = Circle::new(p.point, p.r);
    //     scene.stroke(&stroke, Affine::IDENTITY, line_stroke_color, None, &c);
    // }

    // let arc = vello::kurbo::Arc::new((200., 200.), (100., 100.), 0., -FRAC_PI_2, 0.);
    // scene.stroke(&stroke, Affine::IDENTITY, line_stroke_color, None, &arc);
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
