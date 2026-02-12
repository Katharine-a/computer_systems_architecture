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

#include <string>
#include <vector>
#include <list>
#include <stack>
#include <queue>
#include <unordered_map>
#include <sstream>
#include <memory>
#include <set>
#include <simobject.h>
#include <mem.h>
#include "debug.h"
#include "types.h"
#include "pipeline.h"
#include "instr.h"

namespace tinyrv {

class ProcessorImpl;
class Instr;
class RAM;

class Core : public SimObject<Core> {
public:
  struct PerfStats {
    uint64_t cycles;
    uint64_t instrs;

    PerfStats()
      : cycles(0)
      , instrs(0)
    {}
  };

  Core(const SimContext& ctx, uint32_t core_id, ProcessorImpl* processor);
  ~Core();

  void reset();

  void tick();

  void attach_ram(RAM* ram);

  bool running() const;

  bool check_exit(Word* exitcode, bool riscv_test) const;

  void showStats();

private:

  std::shared_ptr<Instr> decode(uint32_t instr_code) const;

  bool check_data_hazards(const Instr &instr);

  bool data_forwarding(uint32_t reg, uint32_t* rs2_data);

  void regfile_read(const Instr &instr, uint32_t* rs1_data, uint32_t* rs2_data);

  void regfile_write(const Instr &instr, uint32_t alu_result);

  uint32_t alu_unit(const Instr &instr, uint32_t rs1_data, uint32_t rs2_data, uint32_t PC);

  uint32_t branch_unit(const Instr &instr, uint32_t rs1_data, uint32_t rs2_data, uint32_t rd_data, uint32_t PC);

  uint32_t mem_access(const Instr &instr, uint32_t rd_data, uint32_t rs2_data);

  void set_csr(uint32_t addr, uint32_t value);

  uint32_t get_csr(uint32_t addr);

  void dmem_read(void* data, uint64_t addr, uint32_t size);

  void dmem_write(const void* data, uint64_t addr, uint32_t size);

  void writeToStdOut(const void* data);

  void cout_flush();

  struct if_id_t {
    uint32_t instr_code;
    Word     PC;
    uint64_t uuid;
  };

  struct id_ex_t {
    std::shared_ptr<Instr> instr;
    uint32_t rs1_data;
    uint32_t rs2_data;
    Word     PC;
    uint64_t uuid;
  };

  struct ex_mem_t {
    std::shared_ptr<Instr> instr;
    uint32_t rs1_data;
    uint32_t rs2_data;
    uint32_t result;
    Word     PC;
    uint64_t uuid;
  };

  struct mem_wb_t {
    std::shared_ptr<Instr> instr;
    uint32_t result;
    Word     PC;
    uint64_t uuid;
  };

  void if_stage();
  void id_stage();
  void ex_stage();
  void mem_stage();
  void wb_stage();

  uint32_t core_id_;
  ProcessorImpl* processor_;
  MemoryUnit mmu_;

  std::vector<Word> reg_file_;
  Word PC_;

  PipelineReg<if_id_t>  if_id_;
  PipelineReg<id_ex_t>  id_ex_;
  PipelineReg<ex_mem_t> ex_mem_;
  PipelineReg<mem_wb_t> mem_wb_;

  bool fetch_stalled_;
  bool exited_;

  std::stringstream cout_buf_;

  uint64_t uuid_ctr_;

  PerfStats perf_stats_;
  uint64_t fetched_instrs_;

  friend class Emulator;
};

} // namespace tinyrv
