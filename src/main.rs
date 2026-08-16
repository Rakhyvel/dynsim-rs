pub mod sim;

use macroquad::prelude::*;

use crate::sim::Sim;

pub const WIDTH: usize = 1920 / 2;
pub const HEIGHT: usize = 1080 / 2;

fn window_conf() -> Conf {
    Conf {
        window_title: "ODE Visualizer".to_owned(),
        window_width: WIDTH as i32,
        window_height: HEIGHT as i32,
        window_resizable: false,
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let mut sim = Sim::new(1000, 42, WIDTH, HEIGHT);

    let mut image = Image::gen_image_color(WIDTH as u16, HEIGHT as u16, BLACK);
    let texture = Texture2D::from_image(&image);
    texture.set_filter(FilterMode::Nearest);

    loop {
        if is_key_pressed(KeyCode::Escape) {
            break;
        }

        for _ in 0..10 {
            sim.update();
        }
        sim.render();

        image.bytes.copy_from_slice(sim.pixels());
        texture.update(&image);

        clear_background(BLACK);
        draw_texture(&texture, 0.0, 0.0, WHITE);

        next_frame().await;
    }
}
