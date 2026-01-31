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
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <bitset>
#include <climits>
#include <sys/types.h>
#include <sys/stat.h>
#include <assert.h>
#include <util.h>
#include "core.h"

using namespace tinyrv;

uint32_t Core::alu_unit(const Instr &instr, uint32_t rs1_data, uint32_t rs2_data, uint32_t PC) {
  auto exe_flags  = instr.getExeFlags();
  auto alu_op     = instr.getAluOp();

  uint32_t alu_s1 = exe_flags.alu_s1_PC ? PC : (exe_flags.alu_s1_rs1 ? instr.getRs1() :  rs1_data);
  uint32_t alu_s2 = exe_flags.alu_s2_imm ? instr.getImm() : rs2_data;

  if (exe_flags.alu_s1_inv) {
    alu_s1 = ~alu_s1;
  }

  uint32_t rd_data = 0;

  switch (alu_op) {
  case AluOp::NONE:
    break;
  case AluOp::ADD: {
    rd_data = // TODO:
    break;
  }
  case AluOp::SUB: {
    rd_data = // TODO:
    break;
  }
  case AluOp::MUL: {
    rd_data = // TODO;
    break;
  }
  case AluOp::MULH: {
    rd_data = // TODO;
    break;
  }
  case AluOp::MULHSU: {
    rd_data = // TODO;
    break;
  }
  case AluOp::MULHU: {
    rd_data = // TODO;
    break;
  }
  case AluOp::AND: {
    rd_data = // TODO:
    break;
  }
  case AluOp::OR: {
    rd_data = // TODO:
    break;
  }
  case AluOp::XOR: {
    rd_data = // TODO:
    break;
  }
  case AluOp::SLL: {
    rd_data = // TODO:
    break;
  }
  case AluOp::SRL: {
    rd_data = // TODO:
    break;
  }
  case AluOp::SRA: {
    rd_data = // TODO:
    break;
  }
  case AluOp::LTI: {
    rd_data = // TODO:
    break;
  }
  case AluOp::LTU: {
    rd_data = // TODO:
    break;
  }
  default:
    std::abort();
  }

  return rd_data;
}

uint32_t Core::branch_unit(const Instr &instr, uint32_t rs1_data, uint32_t rs2_data, uint32_t rd_data, uint32_t PC) {
  auto br_op = instr.getBrOp();

  bool br_taken = false;

  switch (br_op) {
  case BrOp::NONE:
    break;
  case BrOp::JAL:
  case BrOp::JALR: {
    br_taken = // TODO:
    break;
  }
  case BrOp::BEQ: {
    br_taken = // TODO:
    break;
  }
  case BrOp::BNE: {
    br_taken = // TODO:
    break;
  }
  case BrOp::BLT: {
    br_taken = // TODO:
    break;
  }
  case BrOp::BGE: {
    br_taken = // TODO:
    break;
  }
  case BrOp::BLTU: {
    br_taken = // TODO:
    break;
  }
  case BrOp::BGEU: {
    br_taken = // TODO:
    break;
  }
  default:
    std::abort();
  }

  // resolve branches
  if (br_op != BrOp::NONE) {
    auto br_target = rd_data;
    if (br_taken) {
      uint32_t next_PC = PC + 4;
      if (br_op == BrOp::JAL || br_op == BrOp::JALR) {
        rd_data = // TODO:
      }
      // check misprediction
      if (br_op != BrOp::JAL && br_target != next_PC) {
        PC_ = // TODO:
        // flush pipeline
        if_id_.reset();
        fetch_stalled_ = false;
        DT(2, "*** Branch misprediction: (#" << id_ex_.data().uuid << ")");
      }
    }
    DT(2, "Branch: " << (br_taken ? "taken" : "not-taken") << ", target=0x" << std::hex << br_target << std::dec << " (#" << id_ex_.data().uuid << ")");
  }

  return rd_data;
}

uint32_t Core::mem_access(const Instr &instr, uint32_t rd_data, uint32_t rs2_data) {
  auto exe_flags = instr.getExeFlags();
  auto func3     = instr.getFunc3();

  // handle loads
  if (exe_flags.is_load) {
    uint64_t mem_addr = rd_data;
    uint32_t data_bytes = 1 << (func3 & 0x3);
    uint32_t data_width = 8 * data_bytes;
    uint32_t read_data = 0;
    this->dmem_read(&read_data, mem_addr, data_bytes);
    switch (func3) {
    case 0: // RV32I: LB
    case 1: // RV32I: LH
      rd_data = sext(read_data, data_width);
      break;
    case 2: // RV32I: LW
      rd_data = // TODO:
      break;
    case 4: // RV32I: LBU
    case 5: // RV32I: LHU
      rd_data = // TODO:
      break;
    default:
      std::abort();
    }
  }

  // handle stores
  if (exe_flags.is_store) {
    uint64_t mem_addr = rd_data;
    uint32_t data_bytes = 1 << (func3 & 0x3);
    switch (func3) {
    case 0:
    case 1:
    case 2:
      this->dmem_write(&rs2_data, mem_addr, data_bytes);
      break;
    default:
      std::abort();
    }
  }

  // handle CSR write
  if (exe_flags.is_csr) {
    if (rs2_data != rd_data) {
      this->set_csr(instr.getImm(), rd_data);
    }
    rd_data = rs2_data;
  }

  return rd_data;
}

void Core::dmem_read(void *data, uint64_t addr, uint32_t size) {
  auto type = get_addr_type(addr);
  __unused (type);
  mmu_.read(data, addr, size, 0);
  DTH(2, "Mem Read: addr=0x" << std::hex << addr << ", data=0x" << ByteStream(data, size) << " (size=" << size << ", type=" << type << ")");
}

void Core::dmem_write(const void* data, uint64_t addr, uint32_t size) {
  auto type = get_addr_type(addr);
  __unused (type);
  if (addr >= uint64_t(IO_COUT_ADDR)
   && addr < (uint64_t(IO_COUT_ADDR) + IO_COUT_SIZE)) {
     this->writeToStdOut(data);
  } else {
    mmu_.write(data, addr, size, 0);
  }
  DTH(2, "Mem Write: addr=0x" << std::hex << addr << ", data=0x" << ByteStream(data, size) << " (size=" << size << ", type=" << type << ")");
}

uint32_t Core::get_csr(uint32_t addr) {
  switch (addr) {
  case VX_CSR_MHARTID:
  case VX_CSR_SATP:
  case VX_CSR_PMPCFG0:
  case VX_CSR_PMPADDR0:
  case VX_CSR_MSTATUS:
  case VX_CSR_MISA:
  case VX_CSR_MEDELEG:
  case VX_CSR_MIDELEG:
  case VX_CSR_MIE:
  case VX_CSR_MTVEC:
  case VX_CSR_MEPC:
  case VX_CSR_MNSTATUS:
    return 0;
  case VX_CSR_MCYCLE: // NumCycles
    return perf_stats_.cycles & 0xffffffff;
  case VX_CSR_MCYCLE_H: // NumCycles
    return (uint32_t)(perf_stats_.cycles >> 32);
  case VX_CSR_MINSTRET: // NumInsts
    return perf_stats_.instrs & 0xffffffff;
  case VX_CSR_MINSTRET_H: // NumInsts
    return (uint32_t)(perf_stats_.instrs >> 32);
  default:
    std::cout << std::hex << "Error: invalid CSR read addr=0x" << addr << std::endl;
    std::abort();
    return 0;
  }
}

void Core::set_csr(uint32_t addr, uint32_t value) {
  switch (addr) {
  case VX_CSR_SATP:
  case VX_CSR_MSTATUS:
  case VX_CSR_MEDELEG:
  case VX_CSR_MIDELEG:
  case VX_CSR_MIE:
  case VX_CSR_MTVEC:
  case VX_CSR_MEPC:
  case VX_CSR_PMPCFG0:
  case VX_CSR_PMPADDR0:
  case VX_CSR_MNSTATUS:
    break;
  default: {
      std::cout << std::hex << "Error: invalid CSR write addr=0x" << addr << ", value=0x" << value << std::endl;
      std::abort();
    }
  }
}
