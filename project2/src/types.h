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

#pragma once

#include <stdint.h>
#include <bitset>
#include <queue>
#include <unordered_map>
#include <stringutil.h>
#include <simobject.h>
#include <util.h>
#include "debug.h"
#include "config.h"

namespace tinyrv {

typedef uint8_t  Byte;
typedef uint32_t Word;
typedef int32_t  WordI;

typedef std::bitset<NUM_REGS> RegMask;

enum class AddrType {
  Global,
  IO
};

inline std::ostream &operator<<(std::ostream &os, const AddrType& type) {
  switch (type) {
  case AddrType::Global: os << "Global"; break;
  case AddrType::IO:     os << "IO"; break;
  default: assert(false);
  }
  return os;
}

inline AddrType get_addr_type(uint64_t addr) {
  if (addr >= IO_BASE_ADDR) {
     return AddrType::IO;
  }
  return AddrType::Global;
}

///////////////////////////////////////////////////////////////////////////////

enum class AluOp {
  NONE,
  ADD,
  SUB,
  AND,
  OR,
  XOR,
  SLL,
  SRL,
  SRA,
  LTI,
  LTU
};

inline std::ostream &operator<<(std::ostream &os, const AluOp& op) {
  switch (op) {
  case AluOp::NONE: os << "NONE"; break;
  case AluOp::ADD:  os << "ADD"; break;
  case AluOp::SUB:  os << "SUB"; break;
  case AluOp::AND:  os << "AND"; break;
  case AluOp::OR:   os << "OR";  break;
  case AluOp::XOR:  os << "XOR"; break;
  case AluOp::SLL:  os << "SLL"; break;
  case AluOp::SRL:  os << "SRL"; break;
  case AluOp::SRA:  os << "SRA"; break;
  case AluOp::LTI:  os << "LTI"; break;
  case AluOp::LTU:  os << "LTU"; break;
  default: assert(false);
  }
  return os;
}

///////////////////////////////////////////////////////////////////////////////

enum class BrOp {
  NONE,
  JAL,
  JALR,
  BEQ,
  BNE,
  BLT,
  BGE,
  BLTU,
  BGEU
};

inline std::ostream &operator<<(std::ostream &os, const BrOp& op) {
  switch (op) {
  case BrOp::NONE:  os << "NONE"; break;
  case BrOp::JAL:   os << "JAL";  break;
  case BrOp::JALR:  os << "JALR";  break;
  case BrOp::BEQ:   os << "BEQ";  break;
  case BrOp::BNE:   os << "BNE";  break;
  case BrOp::BLT:   os << "BLT";  break;
  case BrOp::BGE:   os << "BGE";  break;
  case BrOp::BLTU:  os << "BLTU"; break;
  case BrOp::BGEU:  os << "BGEU"; break;
  default: assert(false);
  }
  return os;
}

///////////////////////////////////////////////////////////////////////////////

struct ExeFlags {
  uint32_t use_rd      : 1;   // do write-back
  uint32_t use_rs1     : 1;   // use rs1 register
  uint32_t use_rs2     : 1;   // use rs2 register
  uint32_t use_imm     : 1;   // use immmediate value
  uint32_t is_load     : 1;   // is LDAD instruction
  uint32_t is_store    : 1;   // is STORE instruction
  uint32_t is_csr      : 1;   // is CSR instruction
  uint32_t is_exit     : 1;   // is exit instruction
  uint32_t alu_s1_inv  : 1;   // alu source1 is inverted
  uint32_t alu_s1_rs1  : 1;   // alu source1 is rs1
  uint32_t alu_s1_PC   : 1;   // alu source1 is PC
  uint32_t alu_s2_imm  : 1;   // alu source2 is immediate
  uint32_t alu_s2_csr  : 1;   // alu source2 is CSR
};

inline std::ostream &operator<<(std::ostream &os, const ExeFlags& flags) {
  os << "{"
    << "use_rd=" << flags.use_rd
    << ", use_rs1=" << flags.use_rs1
    << ", use_rs2=" << flags.use_rs2
    << ", use_imm=" << flags.use_imm
    << ", is_load=" << flags.is_load
    << ", is_store=" << flags.is_store
    << ", is_csr=" << flags.is_csr
    << ", is_exit=" << flags.is_exit
    << ", alu_s1_inv=" << flags.alu_s1_inv
    << ", alu_s1_rs1=" << flags.alu_s1_rs1
    << ", alu_s1_PC=" << flags.alu_s1_PC
    << ", alu_s2_imm=" << flags.alu_s2_imm
    << "}";
  return os;
}

}