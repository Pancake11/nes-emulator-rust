use crate::cpu::AddressingMode;
use std::collections::HashMap;

#[derive(Debug)]
pub enum Code {
    ADC,
    AND,
    ASL,
    BCC,
    BCS,
    BEQ,
    BIT,
    BMI,
    BNE,
    BPL,
    BRK,
    BVC,
    BVS,
    CLC,
    CLD,
    CLI,
    CLV,
    CMP,
    CPX,
    CPY,
    DEC,
    DEX,
    DEY,
    EOR,
    INC,
    INX,
    INY,
    JMP,
    JSR,
    LDA,
    LDX,
    LDY,
    LSR,
    NOP,
    ORA,
    PHA,
    PHP,
    PLA,
    PLP,
    ROL,
    ROR,
    RTI,
    RTS,
    SBC,
    SEC,
    SED,
    SEI,
    STA,
    STX,
    STY,
    TAX,
    TAY,
    TSX,
    TXA,
    TXS,
    TYA,
}

#[derive(Debug)]
pub struct OpCode {
    pub code: u8,
    pub opcode: Code,
    pub mnemonic: &'static str,
    pub len: u8,
    pub cycles: u8,
    pub mode: AddressingMode,
}

impl OpCode {
    fn new (code: u8, opcode: Code, mnemonic: &'static str, len: u8, 
            cycles: u8, mode: AddressingMode) -> Self {
        OpCode {
            code,
            opcode,
            mnemonic,
            len,
            cycles,
            mode,
        }
    }
}

lazy_static! {
    pub static ref CPU_OPS_CODES: Vec<OpCode> = vec![
        OpCode::new(0x69, Code::ADC, "ADC", 2, 2, AddressingMode::Immediate),
        OpCode::new(0x65, Code::ADC, "ADC", 2, 3, AddressingMode::ZeroPage),
        OpCode::new(0x75, Code::ADC, "ADC", 2, 4, AddressingMode::ZeroPage_X),
        OpCode::new(0x6D, Code::ADC, "ADC", 3, 4, AddressingMode::Absolute),
        OpCode::new(0x7D, Code::ADC, "ADC", 3, 4, AddressingMode::Absolute_X),
        OpCode::new(0x79, Code::ADC, "ADC", 3, 4, AddressingMode::Absolute_Y),
        OpCode::new(0x61, Code::ADC, "ADC", 2, 6, AddressingMode::Indirect_X),
        OpCode::new(0x71, Code::ADC, "ADC", 2, 5, AddressingMode::Indirect_Y),

        OpCode::new(0x29, Code::AND, "AND", 2, 2, AddressingMode::Immediate),
        OpCode::new(0x25, Code::AND, "AND", 2, 3, AddressingMode::ZeroPage),
        OpCode::new(0x35, Code::AND, "AND", 2, 4, AddressingMode::ZeroPage_X),
        OpCode::new(0x2D, Code::AND, "AND", 3, 4, AddressingMode::Absolute),
        OpCode::new(0x3D, Code::AND, "AND", 3, 4, AddressingMode::Absolute_X),
        OpCode::new(0x39, Code::AND, "AND", 3, 4, AddressingMode::Absolute_Y),
        OpCode::new(0x21, Code::AND, "AND", 2, 6, AddressingMode::Indirect_X),
        OpCode::new(0x31, Code::AND, "AND", 2, 5, AddressingMode::Indirect_Y),

        OpCode::new(0x0A, Code::ASL, "ASL", 1, 2, AddressingMode::None),
        OpCode::new(0x06, Code::ASL, "ASL", 2, 5, AddressingMode::ZeroPage),
        OpCode::new(0x16, Code::ASL, "ASL", 2, 6, AddressingMode::ZeroPage_X),
        OpCode::new(0x0E, Code::ASL, "ASL", 3, 6, AddressingMode::Absolute),
        OpCode::new(0x1E, Code::ASL, "ASL", 3, 7, AddressingMode::Absolute_X),

        OpCode::new(0x90, Code::BCC, "BCC", 2, 2, AddressingMode::None),
        
        OpCode::new(0xB0, Code::BCS, "BCS", 2, 2, AddressingMode::None),

        OpCode::new(0xF0, Code::BEQ, "BEQ", 2, 2, AddressingMode::None),

        OpCode::new(0x24, Code::BIT, "BIT", 2, 3, AddressingMode::ZeroPage),
        OpCode::new(0x2C, Code::BIT, "BIT", 3, 4, AddressingMode::Absolute),

        OpCode::new(0x30, Code::BMI, "BMI", 2, 2, AddressingMode::None),

        OpCode::new(0xD0, Code::BNE, "BNE", 2, 2, AddressingMode::None),

        OpCode::new(0x10, Code::BPL, "BPL", 2, 2, AddressingMode::None),

        OpCode::new(0x00, Code::BRK, "BRK", 1, 7, AddressingMode::None),

        OpCode::new(0x50, Code::BVC, "BVC", 2, 2, AddressingMode::None),

        OpCode::new(0x70, Code::BVS, "BVS", 2, 2, AddressingMode::None),

        OpCode::new(0x18, Code::CLC, "CLC", 1, 2, AddressingMode::None),

        OpCode::new(0xD8, Code::CLD, "CLD", 1, 2, AddressingMode::None),

        OpCode::new(0x58, Code::CLI, "CLI", 1, 2, AddressingMode::None),

        OpCode::new(0xB8, Code::CLV, "CLV", 1, 2, AddressingMode::None),

        OpCode::new(0xC9, Code::CMP, "CMP", 2, 2, AddressingMode::Immediate),
        OpCode::new(0xC5, Code::CMP, "CMP", 2, 3, AddressingMode::ZeroPage),
        OpCode::new(0xD5, Code::CMP, "CMP", 2, 4, AddressingMode::ZeroPage_X),
        OpCode::new(0xCD, Code::CMP, "CMP", 3, 4, AddressingMode::Absolute),
        OpCode::new(0xDD, Code::CMP, "CMP", 3, 4, AddressingMode::Absolute_X),
        OpCode::new(0xD9, Code::CMP, "CMP", 3, 4, AddressingMode::Absolute_Y),
        OpCode::new(0xC1, Code::CMP, "CMP", 2, 6, AddressingMode::Indirect_X),
        OpCode::new(0xD1, Code::CMP, "CMP", 2, 5, AddressingMode::Indirect_Y),

        OpCode::new(0xE0, Code::CPX, "CPX", 2, 2, AddressingMode::Immediate),
        OpCode::new(0xE4, Code::CPX, "CPX", 2, 3, AddressingMode::ZeroPage),
        OpCode::new(0xEC, Code::CPX, "CPX", 3, 4, AddressingMode::Absolute),

        OpCode::new(0xC0, Code::CPY, "CPY", 2, 2, AddressingMode::Immediate),
        OpCode::new(0xC4, Code::CPY, "CPY", 2, 3, AddressingMode::ZeroPage),
        OpCode::new(0xCC, Code::CPY, "CPY", 3, 4, AddressingMode::Absolute),

        OpCode::new(0xC6, Code::DEC, "DEC", 2, 5, AddressingMode::ZeroPage),
        OpCode::new(0xD6, Code::DEC, "DEC", 2, 6, AddressingMode::ZeroPage_X),
        OpCode::new(0xCE, Code::DEC, "DEC", 3, 6, AddressingMode::Absolute),
        OpCode::new(0xDE, Code::DEC, "DEC", 3, 7, AddressingMode::Absolute_X),

        OpCode::new(0xCA, Code::DEX, "DEX", 1, 2, AddressingMode::None),

        OpCode::new(0x88, Code::DEY, "DEY", 1, 2, AddressingMode::None),

        OpCode::new(0x49, Code::EOR, "EOR", 2, 2, AddressingMode::Immediate),
        OpCode::new(0x45, Code::EOR, "EOR", 2, 3, AddressingMode::ZeroPage),
        OpCode::new(0x55, Code::EOR, "EOR", 2, 4, AddressingMode::ZeroPage_X),
        OpCode::new(0x4D, Code::EOR, "EOR", 3, 4, AddressingMode::Absolute),
        OpCode::new(0x5D, Code::EOR, "EOR", 3, 4, AddressingMode::Absolute_X),
        OpCode::new(0x59, Code::EOR, "EOR", 3, 4, AddressingMode::Absolute_Y),
        OpCode::new(0x41, Code::EOR, "EOR", 2, 6, AddressingMode::Indirect_X),
        OpCode::new(0x51, Code::EOR, "EOR", 2, 5, AddressingMode::Indirect_Y),

        OpCode::new(0xE6, Code::INC, "INC", 2, 5, AddressingMode::ZeroPage),
        OpCode::new(0xF6, Code::INC, "INC", 2, 6, AddressingMode::ZeroPage_X),
        OpCode::new(0xEE, Code::INC, "INC", 3, 6, AddressingMode::Absolute),
        OpCode::new(0xFE, Code::INC, "INC", 3, 7, AddressingMode::Absolute_X),

        OpCode::new(0xE8, Code::INX, "INX", 1, 2, AddressingMode::None),

        OpCode::new(0xC8, Code::INY, "INY", 1, 2, AddressingMode::None),

        OpCode::new(0x4C, Code::JMP, "JMP", 3, 3, AddressingMode::Absolute),
        OpCode::new(0x6C, Code::JMP, "JMP", 3, 5, AddressingMode::None),

        OpCode::new(0x20, Code::JSR, "JSR", 3, 6, AddressingMode::Absolute),
        
        OpCode::new(0xA9, Code::LDA, "LDA", 2, 2, AddressingMode::Immediate),
        OpCode::new(0xA5, Code::LDA, "LDA", 2, 3, AddressingMode::ZeroPage),
        OpCode::new(0xB5, Code::LDA, "LDA", 2, 4, AddressingMode::ZeroPage_X),
        OpCode::new(0xAD, Code::LDA, "LDA", 3, 4, AddressingMode::Absolute),
        OpCode::new(0xBD, Code::LDA, "LDA", 3, 4, AddressingMode::Absolute_X),
        OpCode::new(0xB9, Code::LDA, "LDA", 3, 4, AddressingMode::Absolute_Y),
        OpCode::new(0xA1, Code::LDA, "LDA", 2, 6, AddressingMode::Indirect_X),
        OpCode::new(0xB1, Code::LDA, "LDA", 2, 5, AddressingMode::Indirect_Y),

        OpCode::new(0xA2, Code::LDX, "LDX", 2, 2, AddressingMode::Immediate),
        OpCode::new(0xA6, Code::LDX, "LDX", 2, 3, AddressingMode::ZeroPage),
        OpCode::new(0xB6, Code::LDX, "LDX", 2, 4, AddressingMode::ZeroPage_Y),
        OpCode::new(0xAE, Code::LDX, "LDX", 3, 4, AddressingMode::Absolute),
        OpCode::new(0xBE, Code::LDX, "LDX", 3, 4, AddressingMode::Absolute_Y),
        
        OpCode::new(0xA0, Code::LDY, "LDY", 2, 2, AddressingMode::Immediate),
        OpCode::new(0xA4, Code::LDY, "LDY", 2, 3, AddressingMode::ZeroPage),
        OpCode::new(0xB4, Code::LDY, "LDY", 2, 4, AddressingMode::ZeroPage_Y),
        OpCode::new(0xAC, Code::LDY, "LDY", 3, 4, AddressingMode::Absolute),
        OpCode::new(0xBC, Code::LDY, "LDY", 3, 4, AddressingMode::Absolute_Y),

        OpCode::new(0x4A, Code::LSR, "LSR", 1, 2, AddressingMode::None),
        OpCode::new(0x46, Code::LSR, "LSR", 2, 5, AddressingMode::ZeroPage),
        OpCode::new(0x56, Code::LSR, "LSR", 2, 6, AddressingMode::ZeroPage_X),
        OpCode::new(0x4E, Code::LSR, "LSR", 3, 6, AddressingMode::Absolute),
        OpCode::new(0x5E, Code::LSR, "LSR", 3, 7, AddressingMode::Absolute_X),

        OpCode::new(0xEA, Code::NOP, "NOP", 1, 2, AddressingMode::None),

        OpCode::new(0x09, Code::ORA, "ORA", 2, 2, AddressingMode::Immediate), 
        OpCode::new(0x05, Code::ORA, "ORA", 2, 3, AddressingMode::ZeroPage), 
        OpCode::new(0x15, Code::ORA, "ORA", 2, 4, AddressingMode::ZeroPage_X), 
        OpCode::new(0x0D, Code::ORA, "ORA", 3, 4, AddressingMode::Absolute),
        OpCode::new(0x1D, Code::ORA, "ORA", 3, 4, AddressingMode::Absolute_X), 
        OpCode::new(0x19, Code::ORA, "ORA", 3, 4, AddressingMode::Absolute_Y), 
        OpCode::new(0x01, Code::ORA, "ORA", 2, 6, AddressingMode::Indirect_X), 
        OpCode::new(0x11, Code::ORA, "ORA", 2, 5, AddressingMode::Indirect_Y), 

        OpCode::new(0x48, Code::PHA, "PHA", 1, 3, AddressingMode::None),

        OpCode::new(0x08, Code::PHP, "PHP", 1, 3, AddressingMode::None),

        OpCode::new(0x68, Code::PLA, "PLA", 1, 4, AddressingMode::None),

        OpCode::new(0x28, Code::PLP, "PLP", 1, 4, AddressingMode::None),

        OpCode::new(0x2A, Code::ROL, "ROL", 1, 2, AddressingMode::None),
        OpCode::new(0x26, Code::ROL, "ROL", 2, 5, AddressingMode::ZeroPage),
        OpCode::new(0x36, Code::ROL, "ROL", 2, 6, AddressingMode::ZeroPage_X),
        OpCode::new(0x2E, Code::ROL, "ROL", 3, 6, AddressingMode::Absolute),
        OpCode::new(0x3E, Code::ROL, "ROL", 3, 7, AddressingMode::Absolute_X),

        OpCode::new(0x6A, Code::ROR, "ROR", 1, 2, AddressingMode::None),
        OpCode::new(0x66, Code::ROR, "ROR", 2, 5, AddressingMode::ZeroPage),
        OpCode::new(0x76, Code::ROR, "ROR", 2, 6, AddressingMode::ZeroPage_X),
        OpCode::new(0x6E, Code::ROR, "ROR", 3, 6, AddressingMode::Absolute),
        OpCode::new(0x7E, Code::ROR, "ROR", 3, 7, AddressingMode::Absolute_X),

        OpCode::new(0x40, Code::RTI, "RTI", 1, 6, AddressingMode::None),

        OpCode::new(0x60, Code::RTS, "RTS", 1, 6, AddressingMode::None),

        OpCode::new(0xE9, Code::SBC, "SBC", 2, 2, AddressingMode::None),
        OpCode::new(0xE5, Code::SBC, "SBC", 2, 3, AddressingMode::ZeroPage),
        OpCode::new(0xF5, Code::SBC, "SBC", 2, 4, AddressingMode::ZeroPage_X),
        OpCode::new(0xED, Code::SBC, "SBC", 3, 4, AddressingMode::Absolute),
        OpCode::new(0xFD, Code::SBC, "SBC", 3, 4, AddressingMode::Absolute_X),
        OpCode::new(0xF9, Code::SBC, "SBC", 3, 4, AddressingMode::Absolute_Y),
        OpCode::new(0xE1, Code::SBC, "SBC", 2, 6, AddressingMode::Indirect_X),
        OpCode::new(0xF1, Code::SBC, "SBC", 2, 5, AddressingMode::Indirect_Y),

        OpCode::new(0x38, Code::SEC, "SEC", 1, 2, AddressingMode::None),

        OpCode::new(0xF8, Code::SED, "SED", 1, 2, AddressingMode::None),

        OpCode::new(0x78, Code::SEI, "SEI", 1, 2, AddressingMode::None),

        OpCode::new(0x85, Code::STA, "STA", 2, 3, AddressingMode::ZeroPage),
        OpCode::new(0x95, Code::STA, "STA", 2, 4, AddressingMode::ZeroPage_X),
        OpCode::new(0x8d, Code::STA, "STA", 3, 4, AddressingMode::Absolute),
        OpCode::new(0x9d, Code::STA, "STA", 3, 5, AddressingMode::Absolute_X),
        OpCode::new(0x99, Code::STA, "STA", 3, 5, AddressingMode::Absolute_Y),
        OpCode::new(0x81, Code::STA, "STA", 2, 6, AddressingMode::Indirect_X),
        OpCode::new(0x91, Code::STA, "STA", 2, 6, AddressingMode::Indirect_Y),

        OpCode::new(0x86, Code::STX, "STX", 2, 3, AddressingMode::ZeroPage),
        OpCode::new(0x96, Code::STX, "STX", 2, 4, AddressingMode::ZeroPage_X),
        OpCode::new(0x8E, Code::STX, "STX", 3, 4, AddressingMode::Absolute),

        OpCode::new(0x84, Code::STY, "STY", 2, 3, AddressingMode::ZeroPage),
        OpCode::new(0x94, Code::STY, "STY", 2, 4, AddressingMode::ZeroPage_X),
        OpCode::new(0x8C, Code::STY, "STY", 3, 4, AddressingMode::Absolute),

        OpCode::new(0xAA, Code::TAX, "TAX", 1, 2, AddressingMode::None),

        OpCode::new(0xA8, Code::TAY, "TAY", 1, 2, AddressingMode::None),

        OpCode::new(0xBA, Code::TSX, "TSX", 1, 2, AddressingMode::None),

        OpCode::new(0x8A, Code::TXA, "TXA", 1, 2, AddressingMode::None),

        OpCode::new(0x9A, Code::TXS, "TXS", 1, 2, AddressingMode::None),

        OpCode::new(0x98, Code::TYA, "TYA", 1, 2, AddressingMode::None),
    ];

    pub static ref OPCODES_MAP: HashMap<u8, &'static OpCode> = {
            let mut map = HashMap::new();
            for cpuop in &*CPU_OPS_CODES {
                map.insert(cpuop.code, cpuop);
            }
            map
    };
}