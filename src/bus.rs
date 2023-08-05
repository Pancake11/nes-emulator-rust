use crate::ppu::NesPPU;
use crate::rom::Rom;
use crate::Mem;

pub struct Bus {
    cpu_vram: [u8; 2048],
    prg_rom: Vec<u8>,
    ppu: NesPPU,
}

const RAM: u16 = 0x0000;
const RAM_MIRRORS_END: u16 = 0x1FFF;
const PPU_REGISTERS: u16 = 0x2000;
const PPU_REGISTERS_MIRRORS_END: u16 = 0x3FFF;

impl Bus {
    pub fn new(rom: Rom) -> Self {
        let ppu = NesPPU::new(rom.chr_rom, rom.mirroring);

        Bus {
            cpu_vram: [0; 2048],
            prg_rom: rom.prg_rom,
            ppu,
        }
    }

    fn read_prg_rom(&self, mut addr: u16) -> u8 {
        addr -= 0x8000;
        if self.prg_rom.len() == 0x4000 && addr >= 0x4000 {
            // mirroring
            addr = addr % 0x4000;
        }
        self.prg_rom[addr as usize]
    }
}

impl Mem for Bus {
    fn mem_read_u8(&mut self, addr: u16) -> u8 {
        match addr {
            RAM..=RAM_MIRRORS_END => {
                let mirror_down_addr = addr & 0b111_11111111;
                self.cpu_vram[mirror_down_addr as usize]
            }
            0x2000..=0x2006 | 0x4014 => {
                panic!("Attemp to read from wo PPU address {:x}", addr);
            }
            0x2007 => self.ppu.read_data(),
            0x2008..=PPU_REGISTERS_MIRRORS_END => {
                let mirror_down_addr = addr & 0b0010_0000_00000111;
                self.mem_read_u8(mirror_down_addr)
            }
            0x8000..=0xFFFF => self.read_prg_rom(addr),
            _ => {
                println!("ignoring mem access at 0x{:x?}", addr);
                0
            }
        }
    }

    fn mem_write_u8(&mut self, addr: u16, data: u8) {
        match addr {
            RAM..=RAM_MIRRORS_END => {
                let mirror_down_addr = addr & 0b111_1111_1111;
                self.cpu_vram[mirror_down_addr as usize] = data;
            }
            0x8000..=0xFFFF => panic!("Trying to write in ROM at addr : 0x{:x?}", addr),
            _ => {
                println!("ignoring mem access at 0x{:x?}", addr);
            }
        }
    }
}
