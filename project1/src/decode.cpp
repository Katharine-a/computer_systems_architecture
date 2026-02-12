// Copyright 2025 Blaise Tine
//
// Licensed under the Apache License;
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include <string>
#include <stdlib.h>
#include <string.h>
#include <iomanip>
#include <vector>
#include <unordered_map>
#include <util.h>
#include "debug.h"
#include "types.h"
#include "core.h"
#include "instr.h"

using namespace tinyrv;

namespace tinyrv {

static const std::unordered_map<Opcode, InstType> sc_instTable = {
  {Opcode::R,     InstType::R},
  {Opcode::L,     InstType::I},
  {Opcode::I,     InstType::I},
  {Opcode::S,     InstType::S},
  {Opcode::B,     InstType::B},
  {Opcode::LUI,   InstType::U},
  {Opcode::AUIPC, InstType::U},
  {Opcode::JAL,   InstType::J},
  {Opcode::JALR,  InstType::I},
  {Opcode::SYS,   InstType::I},
  {Opcode::FENCE, InstType::I},
};

enum Constants {
  width_opcode= 7,
  width_reg   = 5,
  width_func3 = 3,
  width_func7 = 7,
  width_i_imm = 12,
  width_j_imm = 20,

  shift_opcode= 0,
  shift_rd    = width_opcode,
  shift_func3 = shift_rd + width_reg,
  shift_rs1   = shift_func3 + width_func3,
  shift_rs2   = shift_rs1 + width_reg,
  shift_func2 = shift_rs2 + width_reg,
  shift_func7 = shift_rs2 + width_reg,

  mask_opcode = (1 << width_opcode)- 1,
  mask_reg    = (1 << width_reg)   - 1,
  mask_func3  = (1 << width_func3) - 1,
  mask_func7  = (1 << width_func7) - 1,
  mask_i_imm  = (1 << width_i_imm) - 1,
  mask_j_imm  = (1 << width_j_imm) - 1,
};

static const char* op_string(const Instr &instr) {
  auto opcode = instr.getOpcode();
  auto func3  = instr.getFunc3();
  auto func7  = instr.getFunc7();
  auto imm    = instr.getImm();

  switch (opcode) {
  case Opcode::LUI:   return "LUI";
  case Opcode::AUIPC: return "AUIPC";
  case Opcode::R:
    switch (func3) {
    case 0:
        switch (func7) {
        case 0x00:
            return "ADD";
        case 0x20:
            return "SUB";
        case 0x01:
            return "MUL";
        }
        std::abort();
    case 1: return (func7 & 0x1) ? "MULH" : "SLL";
    case 2: return (func7 & 0x1) ? "MULHSU" : "SLT";
    case 3: return (func7 & 0x1) ? "MULHU" : "SLTU";
    case 4: return "XOR";
    case 5: return (func7 & 0x20) ? "SRA" : "SRL";
    case 6: return "OR";
    case 7: return "AND";
    default:
      std::abort();
    }
  case Opcode::I:
    switch (func3) {
    case 0: return "ADDI";
    case 1: return "SLLI";
    case 2: return "SLTI";
    case 3: return "SLTIU";
    case 4: return "XORI";
    case 5: return (func7 & 0x20) ? "SRAI" : "SRLI";
    case 6: return "ORI";
    case 7: return "ANDI";
    default:
      std::abort();
    }
  case Opcode::B:
    switch (func3) {
    case 0: return "BEQ";
    case 1: return "BNE";
    case 4: return "BLT";
    case 5: return "BGE";
    case 6: return "BLTU";
    case 7: return "BGEU";
    default:
      std::abort();
    }
  case Opcode::JAL:  return "JAL";
  case Opcode::JALR: return "JALR";
  case Opcode::L:
    switch (func3) {
    case 0: return "LB";
    case 1: return "LH";
    case 2: return "LW";
    case 3: return "LD";
    case 4: return "LBU";
    case 5: return "LHU";
    case 6: return "LWU";
    default:
      std::abort();
    }
  case Opcode::S:
    switch (func3) {
    case 0: return "SB";
    case 1: return "SH";
    case 2: return "SW";
    case 3: return "SD";
    default:
      std::abort();
    }
  case Opcode::SYS:
    switch (func3) {
    case 0:
      switch (imm) {
      case 0x000: return "ECALL";
      case 0x001: return "EBREAK";
      case 0x002: return "URET";
      case 0x102: return "SRET";
      case 0x302: return "MRET";
      default:
        std::abort();
      }
    case 1: return "CSRRW";
    case 2: return "CSRRS";
    case 3: return "CSRRC";
    case 5: return "CSRRWI";
    case 6: return "CSRRSI";
    case 7: return "CSRRCI";
    default:
      std::abort();
    }
  case Opcode::FENCE:
    return "FENCE";
  default:
    std::abort();
  }
}

std::ostream &operator<<(std::ostream &os, const Instr &instr) {
  os << op_string(instr);
  int sep = 0;

  auto exec_flags = instr.getExeFlags();

  if (exec_flags.use_rd) {
    if (sep++ != 0) { os << ", "; } else { os << " "; }
    os << "x" << std::dec << instr.getRd();
  }

  if (exec_flags.use_rs1) {
    if (sep++ != 0) { os << ", "; } else { os << " "; }
    os << "x" << std::dec << instr.getRs1();
  }

  if (exec_flags.use_rs2) {
    if (sep++ != 0) { os << ", "; } else { os << " "; }
    os << "x" << std::dec << instr.getRs2();
  }

  if (exec_flags.use_imm) {
    if (sep++ != 0) { os << ", "; } else { os << " "; }
    os << "0x" << std::hex << instr.getImm();
  }

  os << std::dec << ", alu_op=" << instr.getAluOp()
      << ", br_op=" << instr.getBrOp()
     << ", exe_flags=" << instr.getExeFlags();

  return os;
}

}

std::shared_ptr<Instr> Core::decode(uint32_t instr_code) const {
  auto instr = std::make_shared<Instr>();
  auto opcode = Opcode((instr_code >> shift_opcode) & mask_opcode);

  auto func3 = (instr_code >> shift_func3) & mask_func3;
  auto func7 = (instr_code >> shift_func7) & mask_func7;

  auto rd  = (instr_code >> shift_rd)  & mask_reg;
  auto rs1 = (instr_code >> shift_rs1) & mask_reg;
  auto rs2 = (instr_code >> shift_rs2) & mask_reg;

  auto op_it = sc_instTable.find(opcode);
  if (op_it == sc_instTable.end()) {
    std::cout << std::hex << "Error: invalid opcode: 0x" << static_cast<int>(opcode) << std::endl;
    return nullptr;
  }

  ExeFlags exe_flags;
  memset(&exe_flags, 0, sizeof(ExeFlags));
  uint32_t imm = 0x0;

  // instruction type decoding

  auto inst_type = op_it->second;
  switch (inst_type) {
  case InstType::R:
    exe_flags.use_rd  = 1;
    exe_flags.use_rs1 = 1;
    exe_flags.use_rs2 = 1;
    break;

  case InstType::I: {
    switch (opcode) {
    case Opcode::I:
      exe_flags.use_rd  = 1;
      exe_flags.use_rs1 = 1;
      exe_flags.use_imm = 1;
      exe_flags.alu_s2_imm = 1;
      imm = (uint32_t)((int32_t)instr_code >> 20);  // I-type: imm[11:0] sign-extended
      break;
    case Opcode::L:
    case Opcode::JALR: {
      exe_flags.use_rd  = 1;
      exe_flags.use_rs1 = 1;
      exe_flags.use_imm = 1;
      exe_flags.alu_s2_imm = 1;
      imm = (uint32_t)((int32_t)instr_code >> 20);  // I-type: imm[11:0] sign-extended
    } break;
    case Opcode::SYS: {
      exe_flags.use_imm = 1;
      if (func3 != 0) {
        // CSR instructions
        exe_flags.use_rd = 1;
        if (func3 < 5) {
          exe_flags.use_rs1 = 1;
        }
      }
      imm = (instr_code >> 20) & mask_i_imm;  // 12-bit unsigned (CSR address / ecode)
    } break;
    case Opcode::FENCE:
      break;
    default:
      std::abort();
      break;
    }
  } break;
  case InstType::S: {
    // S-type: imm[11:5] at 31-25, imm[4:0] at 11-7, 12-bit signed
    exe_flags.use_rs1 = 1;
    exe_flags.use_rs2 = 1;
    exe_flags.use_imm = 1;
    exe_flags.alu_s2_imm = 1;
    {
      uint32_t s_imm12 = ((instr_code >> 25) << 5) | ((instr_code >> 7) & 0x1f);
      imm = (uint32_t)((int32_t)(s_imm12 << 20) >> 20);
    }
  } break;

  case InstType::B: {
    // B-type: imm[12]|imm[10:5]|imm[4:1]|imm[11] at 31,30-25,11-8,7, 13-bit signed
    exe_flags.use_rs1 = 1;
    exe_flags.use_rs2 = 1;
    exe_flags.use_imm = 1;
    exe_flags.alu_s2_imm = 1;
    {
      uint32_t b_imm13 = (instr_code >> 31) << 12 | ((instr_code >> 25) & 0x3f) << 5
                       | ((instr_code >> 8) & 0xf) << 1 | ((instr_code >> 7) & 1) << 11;
      imm = (uint32_t)((int32_t)(b_imm13 << 19) >> 19);
    }
  } break;

  case InstType::U: {
    // U-type: imm[31:12] at 31-12, 20-bit upper, lower 12 bits zero
    exe_flags.use_rd  = 1;
    exe_flags.use_imm = 1;
    exe_flags.alu_s2_imm = 1;
    imm = (instr_code >> 12) << 12;
  } break;

  case InstType::J: {
    // J-type: imm[20]|imm[10:1]|imm[11]|imm[19:12] at 31,30-21,20,19-12, 21-bit signed
    exe_flags.use_rd  = 1;
    exe_flags.use_imm = 1;
    exe_flags.alu_s2_imm = 1;
    {
      uint32_t j_imm21 = (instr_code >> 31) << 20 | ((instr_code >> 21) & 0x3ff) << 1
                       | ((instr_code >> 20) & 1) << 11 | ((instr_code >> 12) & 0xff) << 12;
      imm = (uint32_t)((int32_t)(j_imm21 << 11) >> 11);
    }
  } break;

  default:
    std::abort();
  }

  // instruction opcode decoding

  AluOp alu_op = AluOp::NONE;
  BrOp br_op = BrOp::NONE;

  switch (opcode) {
  case Opcode::LUI: {
    // RV32I: LUI — rd = imm (upper 20 bits); ALU computes 0 + imm
    alu_op = AluOp::ADD;
    break;
  }
  case Opcode::AUIPC: {
    // RV32I: AUIPC — rd = PC + imm
    alu_op = AluOp::ADD;
    exe_flags.alu_s1_PC = 1;
    break;
  }
  case Opcode::R: {
    switch (func3) {
    case 0:
      switch (func7) {
      case 0x00: alu_op = AluOp::ADD; break;
      case 0x20: alu_op = AluOp::SUB; break;
      case 0x01: alu_op = AluOp::MUL; break;
      default: std::abort();
      }
      break;
    case 1: alu_op = (func7 & 0x1) ? AluOp::MULH  : AluOp::SLL;  break;
    case 2: alu_op = (func7 & 0x1) ? AluOp::MULHSU : AluOp::LTI;  break;
    case 3: alu_op = (func7 & 0x1) ? AluOp::MULHU : AluOp::LTU;  break;
    case 4: alu_op = AluOp::XOR; break;
    case 5: alu_op = (func7 & 0x20) ? AluOp::SRA : AluOp::SRL; break;
    case 6: alu_op = AluOp::OR; break;
    case 7: alu_op = AluOp::AND; break;
    default: std::abort();
    }
    break;
  }
  case Opcode::I: {
    switch (func3) {
    case 0: alu_op = AluOp::ADD; break;   // ADDI
    case 1: alu_op = AluOp::SLL; break;   // SLLI
    case 2: alu_op = AluOp::LTI; break;   // SLTI
    case 3: alu_op = AluOp::LTU; break;   // SLTIU
    case 4: alu_op = AluOp::XOR; break;   // XORI
    case 5: alu_op = (func7 & 0x20) ? AluOp::SRA : AluOp::SRL; break;  // SRAI / SRLI
    case 6: alu_op = AluOp::OR; break;    // ORI
    case 7: alu_op = AluOp::AND; break;   // ANDI
    default: std::abort();
    }
    break;
  }
  case Opcode::B: {
    exe_flags.alu_s1_PC = 1;
    alu_op = AluOp::ADD;  // branch target = PC + imm
    switch (func3) {
    case 0: br_op = BrOp::BEQ;  break;
    case 1: br_op = BrOp::BNE;  break;
    case 4: br_op = BrOp::BLT;  break;
    case 5: br_op = BrOp::BGE;  break;
    case 6: br_op = BrOp::BLTU; break;
    case 7: br_op = BrOp::BGEU; break;
    default: std::abort();
    }
    break;
  }
  case Opcode::JAL: {
    exe_flags.alu_s1_PC = 1;
    alu_op = AluOp::ADD;  // target = PC + imm
    br_op = BrOp::JAL;
    break;
  }
  case Opcode::JALR: {
    alu_op = AluOp::ADD;   // target = rs1 + imm
    br_op = BrOp::JALR;
    break;
  }
  case Opcode::L: {
    // RV32I: LB, LH, LW, LBU, LHU
    alu_op = AluOp::ADD;   // load address = rs1 + imm
    exe_flags.is_load = 1;
    break;
  }
  case Opcode::S: {
    // RV32I: SB, SH, SW — store address = rs1 + imm (S-type imm decoded above)
    alu_op = AluOp::ADD;
    exe_flags.is_store = 1;
    break;
  }
  case Opcode::SYS: {
    if (func3 == 0) {
      alu_op = AluOp::ADD;
      switch (imm) {
      case 0x000: // RV32I: ECALL
      case 0x001: // RV32I: EBREAK
        exe_flags.is_exit = 1;
        break;
      case 0x002: // RV32I: URET
      case 0x102: // RV32I: SRET
      case 0x302: // RV32I: MRET
        break;
      default:
        std::abort();
      }
    } else {
      exe_flags.is_csr = 1;
      exe_flags.alu_s2_csr = 1;
      switch (func3) {
      case 1: {
        // RV32I: CSRRW
        alu_op = AluOp::ADD;
        break;
      }
      case 2: {
        // RV32I: CSRRS
        alu_op = AluOp::OR;
        break;
      }
      case 3: {
        // RV32I: CSRRC
        alu_op = AluOp::AND;
        exe_flags.alu_s1_inv = 1;
        break;
      }
      case 5: {
        // RV32I: CSRRWI
        alu_op = AluOp::ADD;
        exe_flags.alu_s1_rs1 = 1;
        break;
      }
      case 6: {
        // RV32I: CSRRSI;
        alu_op = AluOp::OR;
        exe_flags.alu_s1_rs1 = 1;
        break;
      }
      case 7: {
        // RV32I: CSRRCI
        alu_op = AluOp::AND;
        exe_flags.alu_s1_inv = 1;
        exe_flags.alu_s1_rs1 = 1;
        break;
      }
      default:
        std::abort();
      }
    }
    break;
  }
  case Opcode::FENCE: {
    // RV32I: FENCE
    break;
  }
  default:
    std::abort();
  }

  instr->setOpcode(opcode);
  instr->setRd(rd);
  instr->setSrc1(rs1);
  instr->setSrc2(rs2);
  instr->setImm(imm);
  instr->setFunc3(func3);
  instr->setFunc7(func7);
  instr->setAluOp(alu_op);
  instr->setBrOp(br_op);
  instr->setExeFlags(exe_flags);

  return instr;
}
