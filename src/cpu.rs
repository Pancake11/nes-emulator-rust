use std::collections::HashMap;
use crate::opcodes;

bitflags! {
    pub struct CpuFlags: u8 {
        const C     = 0b00000001;
        const Z     = 0b00000010;
        const I     = 0b00000100;
        const D     = 0b00001000;
        const B     = 0b00010000;
        const B2    = 0b00100000;
        const V     = 0b01000000;
        const N     = 0b10000000;
    }
}

const STACK: u16 = 0x0100;
const STACK_RESET: u8 = 0xfd;

pub struct CPU {
    // registers
    pub ra: u8,
    pub rx: u8,
    pub ry: u8,
    // special registers
    pub sp: u8,
    pub pc: u16,
    // flags
    pub flags: CpuFlags,
    // memory
    memory: [u8; 0xFFFF]
}

#[derive(Debug, PartialEq, Eq)]
#[allow(non_camel_case_types)]
pub enum AddressingMode {
    Immediate,
    ZeroPage,
    ZeroPage_X,
    ZeroPage_Y,
    Absolute,
    Absolute_X,
    Absolute_Y,
    Indirect_X,
    Indirect_Y,
    None,
}

trait Mem {
    fn mem_read_u8(&self, addr: u16) -> u8;
    fn mem_write_u8(&mut self, addr:u16, data: u8);

    // TODO: use rusts native little endian conversion (from_le|ne_bytes)
    fn mem_read_u16(&mut self, addr: u16) -> u16 {
        let lo = self.mem_read_u8(addr) as u16;
        let hi = self.mem_read_u8(addr + 1) as u16;
        // should be (hi << 8) | (lo as u16) but we already converted ?
        (hi << 8) | lo
    }

    // TODO: use rusts native little endian conversion (from_le|ne_bytes)
    fn mem_write_u16(&mut self, addr: u16, data: u16) {
        let hi = (data >> 8) as u8;
        let lo = (data & 0xFF) as u8;
        self.mem_write_u8(addr, lo);
        self.mem_write_u8(addr + 1, hi);
    }
}

impl Mem for CPU {
    fn mem_read_u8(&self, addr: u16) -> u8 {
        self.memory[addr as usize]
    }

    fn mem_write_u8(&mut self, addr: u16, data: u8) {
        self.memory[addr as usize] = data;
    }
}

impl CPU {
    pub fn new() -> Self {
        CPU {
            ra: 0,
            rx: 0,
            ry: 0,
            sp: STACK_RESET,
            pc: 0,
            flags: CpuFlags::from_bits_truncate(0b100100),
            memory: [0; 0xFFFF],
        }
    }

    fn get_operand_address(&mut self, mode: &AddressingMode) -> u16 {
        match mode {
            AddressingMode::Immediate => self.pc,

            AddressingMode::ZeroPage  =>
                self.mem_read_u8(self.pc) as u16,

            AddressingMode::Absolute =>
                self.mem_read_u16(self.pc),

            AddressingMode::ZeroPage_X => {
                let pos = self.mem_read_u8(self.pc);
                let addr = pos.wrapping_add(self.rx) as u16;
                addr
            }
            AddressingMode::ZeroPage_Y => {
                let pos = self.mem_read_u8(self.pc);
                let addr = pos.wrapping_add(self.ry) as u16;
                addr
            }

            AddressingMode::Absolute_X => {
                let base = self.mem_read_u16(self.pc);
                let addr = base.wrapping_add(self.rx as u16);
                addr
            }
            AddressingMode::Absolute_Y => {
                let base = self.mem_read_u16(self.pc);
                let addr = base.wrapping_add(self.ry as u16);
                addr
            }

            AddressingMode::Indirect_X => {
                let base = self.mem_read_u8(self.pc);

                let ptr: u8 = (base as u8).wrapping_add(self.rx);
                let lo = self.mem_read_u8(ptr as u16);
                let hi = self.mem_read_u8(ptr.wrapping_add(1) as u16);
                (hi as u16) << 8 | (lo as u16)
            }
            AddressingMode::Indirect_Y => {
                let base = self.mem_read_u8(self.pc);

                let lo = self.mem_read_u8(base as u16);
                let hi = self.mem_read_u8((base as u8).wrapping_add(1) as u16);
                let deref_base = (hi as u16) << 8 | (lo as u16);
                let deref = deref_base.wrapping_add(self.ry as u16);
                deref
            }

            AddressingMode::None => {
                panic!("mode {:?} is not supported", mode);
            }
        }
    }

    fn load(&mut self, program: Vec<u8>) {
        // program memory is in 0x8000 to 0xFFFF
        self.memory[0x8000..(0x8000 + program.len())]
            .copy_from_slice(&program[..]);

        self.mem_write_u16(0xFFFC, 0x8000);
    }

    fn reset(&mut self) {
        self.ra = 0;
        self.rx = 0;
        self.ry = 0;
        self.sp = STACK_RESET;
        self.flags = CpuFlags::from_bits_truncate(0b100100);

        self.pc = self.mem_read_u16(0xFFFC);
    }

    fn stack_pop(&mut self) -> u8 {
        self.sp = self.sp.wrapping_add(1);
        self.mem_read_u8((STACK as u16) + self.sp as u16)
    }

    fn stack_push_u8(&mut self, data: u8) {
        self.mem_write_u8((STACK as u16) + self.sp as u16, data);
        self.sp = self.sp.wrapping_sub(1);
    }

    fn stack_push_u16(&mut self, data: u16) {
        let hi = (data >> 8) as u8;
        let lo = (data & 0xFF) as u8;

        self.stack_push_u8(hi);
        self.stack_push_u8(lo);
    }

    fn load_and_run(&mut self, program: Vec<u8>) {
        self.load(program);
        self.reset();
        self.run();
    }

    fn set_ra(&mut self, data: u8) {
        self.ra = data;
        self.update_zn_flags(self.ra);
    }

    fn set_carry_flag(&mut self) {
        self.flags.insert(CpuFlags::C);
    }

    fn clear_carry_flag(&mut self) {
        self.flags.remove(CpuFlags::C);
    }

    fn lda(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(&mode);
        let value = self.mem_read_u8(addr);

        self.set_ra(value);
    }

    fn ldx(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(&mode);
        let value = self.mem_read_u8(addr);

        self.rx = value;
        self.update_zn_flags(self.rx);
    }

    fn ldy(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(&mode);
        let value = self.mem_read_u8(addr);

        self.ry = value;
        self.update_zn_flags(self.ry);
    }

    fn sta(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(&mode);
        self.mem_write_u8(addr, self.ra);
    }

    fn tax(&mut self) {
        self.rx = self.ra;
        self.update_zn_flags(self.rx);
    }


    fn and(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let data = self.mem_read_u8(addr);

        self.set_ra(data & self.ra);
    }

    fn asl_acc(&mut self) {
        let mut data = self.ra;

        if data >> 7 == 1 {
            self.set_carry_flag();
        } else {
            self.clear_carry_flag();
        }

        data = data << 1;
        self.set_ra(data);
    }

    fn asl_mem(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let mut data = self.mem_read_u8(addr);

        if data >> 7 == 1 {
            self.set_carry_flag();
        } else {
            self.clear_carry_flag();
        }

        data = data << 1;
        self.mem_write_u8(addr, data);
        self.update_zn_flags(data);
    }

    fn asl(&mut self, mode: &AddressingMode) {
        if mode == &AddressingMode::None {
            self.asl_acc();
        } else {
            self.asl_mem(&mode);
        }
    }

    fn bit(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let data = self.mem_read_u8(addr);

        let res = self.ra & data;
        if res > 0 {
            self.flags.insert(CpuFlags::Z);
        } else {
            self.flags.remove(CpuFlags::Z);
        }

        self.flags.set(CpuFlags::N, data & 0b1000_0000 > 0);
        self.flags.set(CpuFlags::V, data & 0b0100_0000 > 0);
    }

    fn branch(&mut self, cond: bool) {
        if cond {
            let indx: i8 = self.mem_read_u8(self.pc) as i8;
            let addr = self.pc.wrapping_add(1 + indx as u16);
            self.pc = addr;
        }
    }

    fn cmp(&mut self, mode: &AddressingMode, lhs: u8) {
        let addr = self.get_operand_address(mode);
        let data = self.mem_read_u8(addr);

        if lhs >= data {
            self.set_carry_flag();
        } else {
            self.clear_carry_flag();
        }

        self.update_zn_flags(lhs.wrapping_sub(data));
    }

    fn inc(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let mut data = self.mem_read_u8(addr);

        data = data.wrapping_add(1);
        self.mem_write_u8(addr, data);
        self.update_zn_flags(data);
    }

    fn inx(&mut self) {
        self.rx = self.rx.wrapping_add(1);
        self.update_zn_flags(self.rx);
    }

    fn iny(&mut self) {
        self.ry = self.ry.wrapping_add(1);
        self.update_zn_flags(self.ry);
    }

    fn dec(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let mut data = self.mem_read_u8(addr);

        data = data.wrapping_sub(1);
        self.mem_write_u8(addr, data);
        self.update_zn_flags(data);
    }

    fn dex(&mut self) {
        self.rx = self.rx.wrapping_sub(1);
        self.update_zn_flags(self.rx);
    }

    fn dey(&mut self) {
        self.ry = self.ry.wrapping_sub(1);
        self.update_zn_flags(self.ry);
    }

    fn eor(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let data = self.mem_read_u8(addr);

        self.ra = self.ra ^ data;
        self.update_zn_flags(self.ra);
    }

    fn jmp(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let mem_addr = self.mem_read_u16(addr);
        self.pc = mem_addr;
    }

    fn jsr(&mut self) {
        self.stack_push_u16(self.pc + 2 - 1);
        let addr = self.mem_read_u16(self.pc);
        self.pc = addr;
    }

    fn lsr(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let mut data = self.mem_read_u8(addr);

        if data & 1 == 1 {
            self.set_carry_flag();
        } else {
            self.clear_carry_flag();
        }

        data = data >> 1;
        self.mem_write_u8(addr, data);
        self.update_zn_flags(data);
    }

    fn ora(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let data = self.mem_read_u8(addr);

        self.set_ra(self.ra | data);
    }

    fn php(&mut self) {
        let mut flags = self.flags.clone();
        flags.insert(CpuFlags::B);
        flags.insert(CpuFlags::B2);
        self.stack_push_u8(flags.bits());
    }

    fn pla(&mut self) {
        let data = self.stack_pop();
        self.set_ra(data);
    }

    fn plp(&mut self) {
        self.flags.bits = self.stack_pop();
        self.flags.remove(CpuFlags::B);
        self.flags.remove(CpuFlags::B2);
    }

    fn rol_acc(&mut self) {
        let mut data = self.ra;
        let old_c = self.flags.contains(CpuFlags::C);

        if data >> 7 == 1 {
            self.set_carry_flag();
        } else {
            self.clear_carry_flag();
        }

        data = data << 1;
        if old_c {
            data = data | 1;
        }

        self.set_ra(data);
    }

    fn rol_mem(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let mut data = self.mem_read_u8(addr);
        let old_c = self.flags.contains(CpuFlags::C);

        if data >> 7 == 1 {
            self.set_carry_flag();
        } else {
            self.clear_carry_flag();
        }

        data = data << 1;
        if old_c {
            data = data | 1;
        }

        self.mem_write_u8(addr, data);
        self.update_zn_flags(data);
    }

    fn rol(&mut self, mode: &AddressingMode) {
        if mode == &AddressingMode::None {
            self.rol_acc();
        } else {
            self.rol_mem(mode);
        }
    }

    fn ror(&mut self, mode: &AddressingMode) {
        if mode == &AddressingMode::None {
            self.ror_acc();
        } else {
            self.ror_mem(mode);
        }
    }

    fn update_zn_flags(&mut self, result: u8) {
        if result == 0 {
            self.flags = self.flags | CpuFlags::Z;
        } else {
            self.flags = self.flags & !CpuFlags::Z;
        }

        if result & 0b1000_0000 != 0 {
            self.flags = self.flags | CpuFlags::N;
        } else {
            self.flags = self.flags & !CpuFlags::N;
        }
    }

    pub fn run(&mut self) {
        let ref opcodes: HashMap<u8, &'static opcodes::OpCode> =
            *opcodes::OPCODES_MAP;

        loop {
            let code = self.mem_read_u8(self.pc);
            self.pc += 1;
            let pc_state = self.pc;

            let opcode = opcodes.get(&code)
                .expect(&format!("OpCode {:x} is not recognised", code));

            dbg!(opcode);
            match opcode.opcode {
                //opcodes::Code::ADC => self.adc(&opcode.mode),
                opcodes::Code::AND => self.and(&opcode.mode),
                opcodes::Code::ASL => self.asl(&opcode.mode),
                opcodes::Code::BIT => self.bit(&opcode.mode),
                opcodes::Code::BRK => return,

                opcodes::Code::BCC => self.branch(!self.flags.contains(CpuFlags::C)),
                opcodes::Code::BCS => self.branch(self.flags.contains(CpuFlags::C)),
                opcodes::Code::BEQ => self.branch(self.flags.contains(CpuFlags::Z)),
                opcodes::Code::BMI => self.branch(self.flags.contains(CpuFlags::N)),
                opcodes::Code::BNE => self.branch(!self.flags.contains(CpuFlags::Z)),
                opcodes::Code::BPL => self.branch(!self.flags.contains(CpuFlags::N)),
                opcodes::Code::BVC => self.branch(!self.flags.contains(CpuFlags::V)),
                opcodes::Code::BVS => self.branch(self.flags.contains(CpuFlags::V)),

                opcodes::Code::CLC => self.flags.remove(CpuFlags::C),
                opcodes::Code::CLD => self.flags.remove(CpuFlags::D),
                opcodes::Code::CLI => self.flags.remove(CpuFlags::I),
                opcodes::Code::CLV => self.flags.remove(CpuFlags::V),

                opcodes::Code::CMP => self.cmp(&opcode.mode, self.ra),
                opcodes::Code::CPX => self.cmp(&opcode.mode, self.rx),
                opcodes::Code::CPY => self.cmp(&opcode.mode, self.ry),

                opcodes::Code::DEC => self.dec(&opcode.mode),
                opcodes::Code::DEX => self.dex(),
                opcodes::Code::DEY => self.dey(),

                opcodes::Code::EOR => self.eor(&opcode.mode),

                opcodes::Code::INC => self.inc(&opcode.mode),
                opcodes::Code::INX => self.inx(),
                opcodes::Code::INY => self.iny(),

                opcodes::Code::JMP => self.jmp(&opcode.mode),
                opcodes::Code::JSR => self.jsr(),

                opcodes::Code::LDA => self.lda(&opcode.mode),
                opcodes::Code::LDX => self.ldx(&opcode.mode),
                opcodes::Code::LDY => self.ldy(&opcode.mode),

                opcodes::Code::LSR => self.lsr(&opcode.mode),

                opcodes::Code::NOP => { /* nothing */ },

                opcodes::Code::ORA => self.ora(&opcode.mode),

                opcodes::Code::PHA => self.stack_push_u8(self.ra),
                opcodes::Code::PHP => self.php(),
                opcodes::Code::PLA => self.pla(),
                opcodes::Code::PLP => self.plp(),

                opcodes::Code::STA => self.sta(&opcode.mode),
                opcodes::Code::TAX => self.tax(),
                _ => todo!(),
            }

            if pc_state == self.pc {
                self.pc += (opcode.len - 1) as u16;
            }
        }
    }
}

#[cfg(test)]
mod test {
   use super::*;

   #[test]
   fn test_0xa9_lda_immediate_load_data() {
       let mut cpu = CPU::new();
       cpu.load_and_run(vec![0xa9, 0x05, 0x00]);
       assert_eq!(cpu.ra, 0x05);
       assert!(cpu.flags.bits() & 0b0000_0010 == 0b00);
       assert!(cpu.flags.bits() & 0b1000_0000 == 0);
   }

    #[test]
    fn test_0xa9_lda_zero_flag() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0x00, 0x00]);
        assert!(cpu.flags.bits() & 0b0000_0010 == 0b10);
    }

    #[test]
    fn text_0xaa_tax_move_a_to_x() {
        let mut cpu = CPU::new();
        cpu.ra = 10;
        cpu.load_and_run(vec![0xa9, 0x0A, 0xAA, 0x00]);

        assert_eq!(cpu.rx, 10);
    }

    #[test]
    fn test_5_op_working_together() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0xc0, 0xaa, 0xe8, 0x00]);

        assert_eq!(cpu.rx, 0xc1);
    }

    #[test]
    fn test_inx_overflow() {
        let mut cpu = CPU::new();
        cpu.rx = 0xff;
        cpu.load_and_run(vec![0xa9, 0xff, 0xaa, 0xe8, 0xe8, 0x00]);

        assert_eq!(cpu.rx, 0x01);
    }

    #[test]
    fn test_lda_from_memory() {
        let mut cpu = CPU::new();
        cpu.mem_write_u8(0x10, 0x55);

        cpu.load_and_run(vec![0xa5, 0x10, 0x00]);

        assert_eq!(cpu.ra, 0x55);
    }
}
