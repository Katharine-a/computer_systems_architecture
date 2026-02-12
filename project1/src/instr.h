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

#include "types.h"

namespace tinyrv {

enum class InstType {
  R,
  I,
  S,
  B,
  U,
  J,
};

enum class Opcode {
  NONE  = 0x0,
  R     = 0x33,
  L     = 0x3,
  I     = 0x13,
  S     = 0x23,
  B     = 0x63,
  LUI   = 0x37,
  AUIPC = 0x17,
  JAL   = 0x6f,
  JALR  = 0x67,
  SYS   = 0x73,
  FENCE = 0x0f,
};

class Instr {
public:
  Instr()
    : opcode_(Opcode::NONE)
    , rd_(0)
    , rs1_(0)
    , rs2_(0)
    , imm_(0)
    , func3_(0)
    , func7_(0)
    , alu_op_(AluOp::ADD)
    , exe_flags_(ExeFlags{})
  {}

  void setOpcode(Opcode opcode)  {
    opcode_ = opcode;
  }

  void setRd(uint32_t value) {
    rd_ = value;
  }

  void setSrc1(uint32_t value) {
    rs1_ = value;
  }

  void setSrc2(uint32_t value) {
    rs2_ = value;
  }

  void setImm(uint32_t value) {
    imm_ = value;
  }

  void setFunc3(uint32_t value) {
    func3_ = value;
  }

  void setFunc7(uint32_t value) {
    func7_ = value;
  }

  void setAluOp(AluOp value) {
    alu_op_ = value;
  }

  void setBrOp(BrOp value) {
    br_op_ = value;
  }

  void setExeFlags(ExeFlags value) {
    exe_flags_ = value;
  }

  Opcode   getOpcode() const { return opcode_; }
  uint32_t getRd() const { return rd_; }
  uint32_t getRs1() const { return rs1_; }
  uint32_t getRs2() const { return rs2_; }
  uint32_t getImm() const { return imm_; }
  uint32_t getFunc3() const { return func3_; }
  uint32_t getFunc7() const { return func7_; }
  AluOp    getAluOp() const { return alu_op_; };
  BrOp     getBrOp() const { return br_op_; };
  ExeFlags getExeFlags() const { return exe_flags_; }

private:

  Opcode    opcode_;
  uint32_t  rd_;
  uint32_t  rs1_;
  uint32_t  rs2_;
  uint32_t  imm_;
  uint32_t  func3_;
  uint32_t  func7_;

  AluOp     alu_op_;
  BrOp      br_op_;
  ExeFlags  exe_flags_;

  friend std::ostream &operator<<(std::ostream &, const Instr&);
};

}