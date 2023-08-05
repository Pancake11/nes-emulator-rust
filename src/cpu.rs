use crate::bus::Bus;
use crate::opcodes;
use std::collections::HashMap;

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
    pub bus: Bus,
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

pub trait Mem {
    fn mem_read_u8(&mut self, addr: u16) -> u8;
    fn mem_write_u8(&mut self, addr: u16, data: u8);

    // TODO: use rusts native little endian conversion (from_le|ne_bytes)
    fn mem_read_u16(&mut self, addr: u16) -> u16 {
        let lo = self.mem_read_u8(addr) as u16;
        let hi = self.mem_read_u8(addr + 1) as u16;
        (hi << 8) | (lo as u16)
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
    fn mem_read_u8(&mut self, addr: u16) -> u8 {
        self.bus.mem_read_u8(addr)
    }

    fn mem_write_u8(&mut self, addr: u16, data: u8) {
        self.bus.mem_write_u8(addr, data);
    }

    fn mem_read_u16(&mut self, addr: u16) -> u16 {
        self.bus.mem_read_u16(addr)
    }

    fn mem_write_u16(&mut self, addr: u16, data: u16) {
        self.bus.mem_write_u16(addr, data);
    }
}

impl CPU {
    pub fn new(bus: Bus) -> Self {
        CPU {
            ra: 0,
            rx: 0,
            ry: 0,
            sp: STACK_RESET,
            pc: 0,
            flags: CpuFlags::from_bits_truncate(0b100100),
            bus,
        }
    }

    fn get_operand_address(&mut self, mode: &AddressingMode) -> u16 {
        match mode {
            AddressingMode::Immediate => self.pc,
            _ => self.get_absolute_address(mode, self.pc),
        }
    }

    pub fn get_absolute_address(&mut self, mode: &AddressingMode, addr: u16) -> u16 {
        match mode {
            AddressingMode::Immediate => addr,

            AddressingMode::ZeroPage => self.mem_read_u8(addr) as u16,

            AddressingMode::Absolute => self.mem_read_u16(addr),

            AddressingMode::ZeroPage_X => {
                let pos = self.mem_read_u8(addr);
                let addr = pos.wrapping_add(self.rx) as u16;
                addr
            }
            AddressingMode::ZeroPage_Y => {
                let pos = self.mem_read_u8(addr);
                let addr = pos.wrapping_add(self.ry) as u16;
                addr
            }

            AddressingMode::Absolute_X => {
                let base = self.mem_read_u16(addr);
                let addr = base.wrapping_add(self.rx as u16);
                addr
            }
            AddressingMode::Absolute_Y => {
                let base = self.mem_read_u16(addr);
                let addr = base.wrapping_add(self.ry as u16);
                addr
            }

            AddressingMode::Indirect_X => {
                let base = self.mem_read_u8(addr);

                let ptr: u8 = (base as u8).wrapping_add(self.rx);
                let lo = self.mem_read_u8(ptr as u16);
                let hi = self.mem_read_u8(ptr.wrapping_add(1) as u16);
                (hi as u16) << 8 | (lo as u16)
            }
            AddressingMode::Indirect_Y => {
                let base = self.mem_read_u8(addr);

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

    pub fn load(&mut self, program: Vec<u8>) {
        // program memory is in 0x8000 to 0xFFFF
        for i in 0..(program.len() as u16) {
            self.mem_write_u8(0x0600 + i, program[i as usize]);
        }

        // self.mem_write_u16(0xFFFC, 0x0600);
    }

    pub fn load_and_run(&mut self, program: Vec<u8>) {
        self.load(program);
        self.reset();
        self.pc = 0x0600;
        self.run();
    }

    pub fn reset(&mut self) {
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

    fn stack_pop_u16(&mut self) -> u16 {
        let lo = self.stack_pop() as u16;
        let hi = self.stack_pop() as u16;

        hi << 8 | lo
    }

    fn stack_push_u8(&mut self, data: u8) {
        self.mem_write_u8((STACK as u16) + self.sp as u16, data);
        self.sp = self.sp.wrapping_sub(1);
    }

    fn stack_push_u16(&mut self, data: u16) {
        let hi = (data >> 8) as u8;
        let lo = (data & 0xff) as u8;

        self.stack_push_u8(hi);
        self.stack_push_u8(lo);
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
        let addr = self.get_operand_address(mode);
        self.mem_write_u8(addr, self.ra);
    }

    fn stx(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(&mode);
        self.mem_write_u8(addr, self.rx);
    }

    fn sty(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(&mode);
        self.mem_write_u8(addr, self.ry);
    }

    fn tax(&mut self) {
        self.rx = self.ra;
        self.update_zn_flags(self.rx);
    }

    fn tay(&mut self) {
        self.ry = self.ra;
        self.update_zn_flags(self.ry);
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
        if res == 0 {
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

    fn jmp_abs(&mut self) {
        let addr = self.mem_read_u16(self.pc);
        self.pc = addr;
    }

    fn jmp_ind(&mut self) {
        let addr = self.mem_read_u16(self.pc);
        // 6502 bug with page boundary
        let indirect_ref = if addr & 0x00FF == 0x00FF {
            let lo = self.mem_read_u8(addr);
            let hi = self.mem_read_u8(addr & 0xFF00);
            (hi as u16) << 8 | (lo as u16)
        } else {
            self.mem_read_u16(addr)
        };

        self.pc = indirect_ref;
    }

    fn jsr(&mut self) {
        self.stack_push_u16(self.pc + 2 - 1);
        let addr = self.mem_read_u16(self.pc);
        self.pc = addr;
    }

    fn lsr_acc(&mut self) {
        let mut data = self.ra;

        if data & 1 == 1 {
            self.set_carry_flag();
        } else {
            self.clear_carry_flag();
        }

        data = data >> 1;
        self.set_ra(data);
    }

    fn lsr_mem(&mut self, mode: &AddressingMode) {
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

    fn lsr(&mut self, mode: &AddressingMode) {
        if mode == &AddressingMode::None {
            self.lsr_acc();
        } else {
            self.lsr_mem(mode);
        }
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
        self.flags.insert(CpuFlags::B2);
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

    fn ror_mem(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(mode);
        let mut data = self.mem_read_u8(addr);
        let old_c = self.flags.contains(CpuFlags::C);

        if data & 1 == 1 {
            self.set_carry_flag();
        } else {
            self.clear_carry_flag();
        }

        data = data >> 1;
        if old_c {
            data = data | 0b1000_0000;
        }

        self.mem_write_u8(addr, data);
        self.update_zn_flags(data);
    }

    fn ror_acc(&mut self) {
        let mut data = self.ra;
        let old_c = self.flags.contains(CpuFlags::C);

        if data & 1 == 1 {
            self.set_carry_flag();
        } else {
            self.clear_carry_flag();
        }

        data = data >> 1;
        if old_c {
            data = data | 0b1000_0000;
        }

        self.set_ra(data);
    }

    fn ror(&mut self, mode: &AddressingMode) {
        if mode == &AddressingMode::None {
            self.ror_acc();
        } else {
            self.ror_mem(mode);
        }
    }

    fn rti(&mut self) {
        self.flags.bits = self.stack_pop();
        self.flags.remove(CpuFlags::B);
        self.flags.insert(CpuFlags::B2);

        self.pc = self.stack_pop_u16();
    }

    fn tsx(&mut self) {
        self.rx = self.sp;
        self.update_zn_flags(self.rx);
    }

    fn txs(&mut self) {
        self.sp = self.rx;
    }

    fn add_to_ra(&mut self, data: u8) {
        let sum = self.ra as u16 + data as u16 + self.flags.contains(CpuFlags::C) as u16;

        let carry = sum > 0xff;

        if carry {
            self.flags.insert(CpuFlags::C);
        } else {
            self.flags.remove(CpuFlags::C);
        }

        let res = sum as u8;

        if (data ^ res) & (res ^ self.ra) & 0x80 != 0 {
            self.flags.insert(CpuFlags::V);
        } else {
            self.flags.remove(CpuFlags::V);
        }

        self.set_ra(res);
    }

    fn adc(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(&mode);
        let data = self.mem_read_u8(addr);

        self.add_to_ra(data);
    }

    fn sbc(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(&mode);
        let data = self.mem_read_u8(addr);

        self.add_to_ra(((data as i8).wrapping_neg().wrapping_sub(1)) as u8);
    }

    fn rts(&mut self) {
        self.pc = self.stack_pop_u16() + 1;
    }

    fn lax(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(&mode);
        let data = self.mem_read_u8(addr);

        self.set_ra(data);
        self.rx = self.ra;
    }

    fn sax(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(&mode);
        let data = self.ra & self.rx;

        self.mem_write_u8(addr, data);
    }

    fn dcp(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(&mode);
        let mut data = self.mem_read_u8(addr);

        data = data.wrapping_sub(1);
        self.mem_write_u8(addr, data);

        if data <= self.ra {
            self.flags.insert(CpuFlags::C);
        }

        self.update_zn_flags(self.ra.wrapping_sub(data));
    }

    fn isb(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(&mode);
        self.inc(mode);

        let data = self.mem_read_u8(addr);
        self.add_to_ra(((data).wrapping_neg().wrapping_sub(1)) as u8);
    }

    fn slo(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(&mode);
        self.asl(mode);

        let data = self.mem_read_u8(addr);
        self.set_ra(data | self.ra);
    }

    fn rla(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(&mode);
        self.rol(mode);

        let data = self.mem_read_u8(addr);
        self.set_ra(data & self.ra);
    }

    fn sre(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(&mode);
        self.lsr(mode);

        let data = self.mem_read_u8(addr);
        self.set_ra(data ^ self.ra);
    }

    fn rra(&mut self, mode: &AddressingMode) {
        let addr = self.get_operand_address(&mode);
        self.ror(mode);

        let data = self.mem_read_u8(addr);
        self.add_to_ra(data);
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
        self.run_with_callback(|_| {});
    }

    pub fn run_with_callback<F>(&mut self, mut callback: F)
    where
        F: FnMut(&mut CPU),
    {
        let ref opcodes: HashMap<u8, &'static opcodes::OpCode> = *opcodes::OPCODES_MAP;

        loop {
            callback(self);

            let code = self.mem_read_u8(self.pc);
            self.pc += 1;
            let pc_state = self.pc;

            let opcode = opcodes
                .get(&code)
                .expect(&format!("OpCode {:x} is not recognised", code));

            match opcode.opcode {
                opcodes::Code::ADC => self.adc(&opcode.mode),
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

                opcodes::Code::JMP => {
                    // no clean way to differentiate the two while keeping a clean trace
                    // kinda sad
                    if opcode.code == 0x4C {
                        self.jmp_abs();
                    } else {
                        self.jmp_ind();
                    }
                }
                opcodes::Code::JSR => self.jsr(),

                opcodes::Code::LDA => self.lda(&opcode.mode),
                opcodes::Code::LDX => self.ldx(&opcode.mode),
                opcodes::Code::LDY => self.ldy(&opcode.mode),

                opcodes::Code::LSR => self.lsr(&opcode.mode),

                opcodes::Code::NOP | opcodes::Code::DOP | opcodes::Code::TOP => { /* nothing */ }

                opcodes::Code::ORA => self.ora(&opcode.mode),

                opcodes::Code::PHA => self.stack_push_u8(self.ra),
                opcodes::Code::PHP => self.php(),
                opcodes::Code::PLA => self.pla(),
                opcodes::Code::PLP => self.plp(),

                opcodes::Code::ROR => self.ror(&opcode.mode),
                opcodes::Code::ROL => self.rol(&opcode.mode),

                opcodes::Code::RTI => self.rti(),
                opcodes::Code::RTS => self.rts(),

                opcodes::Code::SBC => self.sbc(&opcode.mode),

                opcodes::Code::SEC => self.set_carry_flag(),
                opcodes::Code::SED => self.flags.insert(CpuFlags::D),
                opcodes::Code::SEI => self.flags.insert(CpuFlags::I),

                opcodes::Code::STA => self.sta(&opcode.mode),
                opcodes::Code::STX => self.stx(&opcode.mode),
                opcodes::Code::STY => self.sty(&opcode.mode),

                opcodes::Code::TAX => self.tax(),
                opcodes::Code::TAY => self.tay(),

                opcodes::Code::TSX => self.tsx(),
                opcodes::Code::TXA => self.set_ra(self.rx),
                opcodes::Code::TXS => self.txs(),
                opcodes::Code::TYA => self.set_ra(self.ry),

                // unofficials
                opcodes::Code::LAX => self.lax(&opcode.mode),
                opcodes::Code::SAX => self.sax(&opcode.mode),
                opcodes::Code::DCP => self.dcp(&opcode.mode),
                opcodes::Code::ISB => self.isb(&opcode.mode),
                opcodes::Code::SLO => self.slo(&opcode.mode),
                opcodes::Code::RLA => self.rla(&opcode.mode),
                opcodes::Code::SRE => self.sre(&opcode.mode),
                opcodes::Code::RRA => self.rra(&opcode.mode),
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
    use crate::rom::test;

    #[test]
    fn test_0xa9_lda_immediate_load_data() {
        let bus = Bus::new(test::test_rom());
        let mut cpu = CPU::new(bus);
        cpu.load_and_run(vec![0xa9, 0x05, 0x00]);
        assert_eq!(cpu.ra, 0x05);
        assert!(cpu.flags.bits() & 0b0000_0010 == 0b00);
        assert!(cpu.flags.bits() & 0b1000_0000 == 0);
    }

    #[test]
    fn test_0xa9_lda_zero_flag() {
        let bus = Bus::new(test::test_rom());
        let mut cpu = CPU::new(bus);
        cpu.load_and_run(vec![0xa9, 0x00, 0x00]);
        assert!(cpu.flags.bits() & 0b0000_0010 == 0b10);
    }

    #[test]
    fn text_0xaa_tax_move_a_to_x() {
        let bus = Bus::new(test::test_rom());
        let mut cpu = CPU::new(bus);
        cpu.ra = 10;
        cpu.load_and_run(vec![0xa9, 0x0A, 0xAA, 0x00]);

        assert_eq!(cpu.rx, 10);
    }

    #[test]
    fn test_5_op_working_together() {
        let bus = Bus::new(test::test_rom());
        let mut cpu = CPU::new(bus);
        cpu.load_and_run(vec![0xa9, 0xc0, 0xaa, 0xe8, 0x00]);

        assert_eq!(cpu.rx, 0xc1);
    }

    #[test]
    fn test_inx_overflow() {
        let bus = Bus::new(test::test_rom());
        let mut cpu = CPU::new(bus);
        cpu.rx = 0xff;
        cpu.load_and_run(vec![0xa9, 0xff, 0xaa, 0xe8, 0xe8, 0x00]);

        assert_eq!(cpu.rx, 0x01);
    }

    #[test]
    fn test_lda_from_memory() {
        let bus = Bus::new(test::test_rom());
        let mut cpu = CPU::new(bus);
        cpu.mem_write_u8(0x10, 0x55);

        cpu.load_and_run(vec![0xa5, 0x10, 0x00]);

        assert_eq!(cpu.ra, 0x55);
    }
}
