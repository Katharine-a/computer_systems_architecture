//
// Copyright 2026 Blaise Tine
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

#pragma once

#include <cstdint>
#include <map>
#include <sys/types.h>
#include <vector>

namespace tinyrv {

class BranchPredictor {
  public:
    virtual ~BranchPredictor() {}

    virtual uint32_t predict(uint32_t PC) { return PC + 4; };

    virtual void update(uint32_t PC, uint32_t next_PC, bool taken) {
        (void)PC;
        (void)next_PC;
        (void)taken;
    };
};

class GShare : public BranchPredictor {
  public:
    GShare(uint32_t BTB_size, uint32_t BHR_size);

    ~GShare() override;

    uint32_t predict(uint32_t PC) override;
    void update(uint32_t PC, uint32_t next_PC, bool taken) override;

    // TODO: Add your own methods here
    void update_bht(uint32_t PC, bool taken);
    void update_btb(uint32_t PC, uint32_t next_PC);
    void update_bhr(bool taken);
    uint32_t inc_two_bit_counter(uint32_t count);
    uint32_t dec_two_bit_counter(uint32_t count);

  private:
    struct BTB_entry_t {
        bool valid;
        uint32_t tag;
        uint32_t target;
    };
    std::vector<BTB_entry_t> BTB_;
    std::vector<uint32_t> BHT_;
    uint32_t BHR_;
    uint32_t BTB_shift_;
    uint32_t BTB_mask_;
    uint32_t BHR_mask_;
};

class GSharePlus : public BranchPredictor {
  public:
    GSharePlus(uint32_t BTB_size, uint32_t BHR_size);

    ~GSharePlus() override;

    uint32_t predict(uint32_t PC) override;
    void update(uint32_t PC, uint32_t next_PC, bool taken) override;

    // TODO: extra credit component
};

} // namespace tinyrv
