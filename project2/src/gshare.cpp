//
// Copyright 2026 Blaise Tine
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
//

#include "gshare.h"
#include "core.h"
#include "debug.h"
#include "types.h"
#include <algorithm>
#include <assert.h>
#include <cstdint>
#include <iostream>
#include <sys/types.h>
#include <util.h>

using namespace tinyrv;

////////////////////////////////////////////////////////////////////////////////

GShare::GShare(uint32_t BTB_size, uint32_t BHR_size)
    : BTB_(BTB_size, BTB_entry_t{false, 0x0, 0x0}), BHT_((1 << BHR_size), 0x0),
      BHR_(0x0), BTB_shift_(log2ceil(BTB_size)), BTB_mask_(BTB_size - 1),
      BHR_mask_((1 << BHR_size) - 1) {
    //--
}

GShare::~GShare() {
    //--
}

uint32_t GShare::predict(uint32_t PC) {
    uint32_t next_PC = PC + 4;
    bool predict_taken = false;

    // TODO:
    // branch prediction
    uint32_t bht_index = ((PC >> 2) ^ BHR_) & BHR_mask_;
    predict_taken = (BHT_[bht_index] >= 2) ? 1 : 0;
    std::cout << "bht_index: " << bht_index
              << "\npredict_taken: " << predict_taken << "\n"
              << std::endl;

    // branch target
    if (predict_taken) {
        uint32_t btb_index = (PC >> 2) & BTB_mask_;
        uint32_t btb_tag = (PC >> (BTB_shift_ + 2));
        BTB_entry_t btb_row = BTB_[btb_index];
        if (btb_row.valid && btb_row.tag == btb_tag) {
            next_PC = btb_row.target;
        }
    }

    DT(3, "*** GShare: predict PC=0x"
              << std::hex << PC << std::dec << ", next_PC=0x" << std::hex
              << next_PC << std::dec << ", predict_taken=" << predict_taken);
    return next_PC;
}

/*
 * PC is PC of current instruction
 * next_PC is true (computed) next PC
 * taken is prediction ground truth
 */
void GShare::update(uint32_t PC, uint32_t next_PC, bool taken) {
    DT(3, "*** GShare: update PC=0x" << std::hex << PC << std::dec
                                     << ", next_PC=0x" << std::hex << next_PC
                                     << std::dec << ", taken=" << taken);

    // TODO:
    update_bht(PC, taken);
    if (taken) {
        update_btb(PC, next_PC);
    }
    update_bhr(taken);
}

void GShare::update_bht(uint32_t PC, bool taken) {
    uint32_t bht_index =
        ((PC >> 2) ^ BHR_) & BHR_mask_; // make sure bhr is current
    if (taken) {
        BHT_[bht_index] = inc_two_bit_counter(BHT_[bht_index]);
    } else {
        BHT_[bht_index] = dec_two_bit_counter(BHT_[bht_index]);
    }
}

void GShare::update_btb(uint32_t PC, uint32_t next_PC) {
    uint32_t btb_index = (PC >> 2) & BTB_mask_;
    BTB_entry_t *entry = &BTB_[btb_index];
    entry->valid = true;
    entry->target = next_PC;
    entry->tag = (PC >> (BTB_shift_ + 2));
}

void GShare::update_bhr(bool taken) { BHR_ = (BHR_ << 1 | taken) & BHR_mask_; }

uint32_t GShare::inc_two_bit_counter(uint32_t count) {
    return count == 3 ? count : ++count;
}
uint32_t GShare::dec_two_bit_counter(uint32_t count) {
    return count == 0 ? count : --count;
}

///////////////////////////////////////////////////////////////////////////////

GSharePlus::GSharePlus(uint32_t BTB_size, uint32_t BHR_size) {
    (void)BTB_size;
    (void)BHR_size;
}

GSharePlus::~GSharePlus() {
    //--
}

uint32_t GSharePlus::predict(uint32_t PC) {
    uint32_t next_PC = PC + 4;
    bool predict_taken = false;
    (void)PC;
    (void)next_PC;
    (void)predict_taken;

    // TODO: extra credit component

    DT(3, "*** GShare+: predict PC=0x"
              << std::hex << PC << std::dec << ", next_PC=0x" << std::hex
              << next_PC << std::dec << ", predict_taken=" << predict_taken);
    return next_PC;
}

void GSharePlus::update(uint32_t PC, uint32_t next_PC, bool taken) {
    (void)PC;
    (void)next_PC;
    (void)taken;

    DT(3, "*** GShare+: update PC=0x" << std::hex << PC << std::dec
                                      << ", next_PC=0x" << std::hex << next_PC
                                      << std::dec << ", taken=" << taken);

    // TODO: extra credit component
}
