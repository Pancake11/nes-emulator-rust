use crate::rom::Mirroring;

use registers::addr::AddrRegister;
use registers::control::ControlRegister;

pub mod registers;

pub struct NesPPU {
    pub chr_rom: Vec<u8>,
    pub palette_table: [u8; 32],
    pub vram: [u8; 2048],
    pub oam_data: [u8; 256],

    pub mirroring: Mirroring,
    addr: AddrRegister,
    data_buf: u8,
    pub ctrl: ControlRegister,
}

impl NesPPU {
    pub fn new(chr_rom: Vec<u8>, mirroring: Mirroring) -> Self {
        NesPPU {
            chr_rom,
            mirroring,
            addr: AddrRegister::new(),
            ctrl: ControlRegister::new(),

            vram: [0; 2048],
            oam_data: [0; 64 * 4],
            palette_table: [0; 32],
            data_buf: 0,
        }
    }

    fn write_to_ppu(&mut self, value: u8) {
        self.addr.update(value);
    }

    fn write_to_ctrl(&mut self, value: u8) {
        self.ctrl.update(value);
    }

    fn increment_vram_addr(&mut self) {
        self.addr.increment(self.ctrl.vram_addr_increment());
    }

    pub fn mirror_vram_addr(&self, addr: u16) -> u16 {
        let mirrored_vram = addr & 0b10_1111_1111_1111;
        let vram_idx = mirrored_vram - 0x200;
        let name_table = vram_idx / 0x400;
        match (&self.mirroring, name_table) {
            (Mirroring::VERTICAL, 2) | (Mirroring::VERTICAL, 3) => vram_idx - 0x800,
            (Mirroring::HORIZONTAL, 2) => vram_idx - 0x400,
            (Mirroring::HORIZONTAL, 1) => vram_idx - 0x400,
            (Mirroring::HORIZONTAL, 3) => vram_idx - 0x800,
            _ => vram_idx,
        }
    }

    pub fn read_data(&mut self) -> u8 {
        let addr = self.addr.get();
        self.increment_vram_addr();

        match addr {
            0..=0x1FFF => {
                let res = self.data_buf;
                self.data_buf = self.chr_rom[addr as usize];
                res
            }
            0x2000..=0x2FFF => {
                let res = self.data_buf;
                self.data_buf = self.vram[self.mirror_vram_addr(addr) as usize];
                res
            }
            0x3000..=0x3EFF => panic!(
                "addr space 0x3000..0x3EFF is not expected to be used, requested = {} ",
                addr
            ),
            0x3F00..=0x3FFF => self.palette_table[(addr - 0x3f00) as usize],
            _ => panic!("unexpected access to mirrored space {}", addr),
        }
    }
}
