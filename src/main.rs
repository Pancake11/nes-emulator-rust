pub mod bus;
pub mod cpu;
pub mod opcodes;
pub mod rom;
pub mod trace;

use bus::Bus;
use cpu::Mem;
use cpu::CPU;
use trace::trace;
// use rand::Rng;
use rom::Rom;
use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::pixels::Color;
use sdl2::pixels::PixelFormatEnum;
use sdl2::EventPump;

#[macro_use]
extern crate lazy_static;

#[macro_use]
extern crate bitflags;

fn handle_user_input(cpu: &mut CPU, event_pump: &mut EventPump) {
    for event in event_pump.poll_iter() {
        match event {
            Event::Quit { .. }
            | Event::KeyDown {
                keycode: Some(Keycode::Escape),
                ..
            } => {
                std::process::exit(0);
            }
            Event::KeyDown {
                keycode: Some(Keycode::W),
                ..
            } => {
                cpu.mem_write_u8(0xff, 0x77);
            }
            Event::KeyDown {
                keycode: Some(Keycode::S),
                ..
            } => {
                cpu.mem_write_u8(0xff, 0x73);
            }
            Event::KeyDown {
                keycode: Some(Keycode::A),
                ..
            } => {
                cpu.mem_write_u8(0xff, 0x61);
            }
            Event::KeyDown {
                keycode: Some(Keycode::D),
                ..
            } => {
                cpu.mem_write_u8(0xff, 0x64);
            }
            _ => { /* nothing */ }
        }
    }
}

fn color(byte: u8) -> Color {
    match byte {
        0 => sdl2::pixels::Color::BLACK,
        1 => sdl2::pixels::Color::BLACK,
        2 | 9 => sdl2::pixels::Color::BLACK,
        3 | 10 => sdl2::pixels::Color::BLACK,
        4 | 11 => sdl2::pixels::Color::BLACK,
        5 | 12 => sdl2::pixels::Color::BLACK,
        6 | 13 => sdl2::pixels::Color::BLACK,
        7 | 14 => sdl2::pixels::Color::BLACK,
        _ => sdl2::pixels::Color::CYAN,
    }
}

fn read_screen_state(cpu: &CPU, frame: &mut [u8; 32 * 3 * 32]) -> bool {
    let mut frame_idx = 0;
    let mut update = false;

    for i in 0x200..0x600 {
        let color_idx = cpu.mem_read_u8(i as u16);
        let (b1, b2, b3) = color(color_idx).rgb();
        if frame[frame_idx] != b1 || frame[frame_idx + 1] != b2 || frame[frame_idx + 2] != b3 {
            frame[frame_idx] = b1;
            frame[frame_idx + 1] = b2;
            frame[frame_idx + 2] = b3;
            update = true;
        }
        frame_idx += 3;
    }
    update
}

fn main() {
    let sdl_ctx = sdl2::init().unwrap();
    let video_subsystem = sdl_ctx.video().unwrap();
    let window = video_subsystem
        .window("Snake Game", (32.0 * 10.0) as u32, (32.0 * 10.0) as u32)
        .position_centered()
        .build()
        .unwrap();

    let mut canvas = window.into_canvas().present_vsync().build().unwrap();
    let mut event_pump = sdl_ctx.event_pump().unwrap();
    canvas.set_scale(10.0, 10.0).unwrap();

    let creator = canvas.texture_creator();
    let mut texture = creator
        .create_texture_target(PixelFormatEnum::RGB24, 32, 32)
        .unwrap();

    let mut screen_state = [0 as u8; 32 * 3 * 32];
    let mut rng = rand::thread_rng();

    //loading the rom
    let data: Vec<u8> = std::fs::read("nestest.nes").unwrap();
    let rom = Rom::new(&data).unwrap();

    let bus = Bus::new(rom);
    let mut cpu = CPU::new(bus);
    cpu.reset();
    cpu.pc = 0xC000;

    cpu.run_with_callback(move |cpu| {
        println!("{}", trace(cpu));
        // handle_user_input(cpu, &mut event_pump);
        // cpu.mem_write_u8(0xfe, rng.gen_range(1, 16));

        // if read_screen_state(cpu, &mut screen_state) {
        // texture.update(None, &screen_state, 32 * 3).unwrap();
        // canvas.copy(&texture, None, None).unwrap();
        // canvas.present();
        // }

        // ::std::thread::sleep(std::time::Duration::new(0, 70_000));
    })
}
