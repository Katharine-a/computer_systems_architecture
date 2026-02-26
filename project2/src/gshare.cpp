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

#include <iostream>
#include <assert.h>
#include <util.h>
#include "types.h"
#include "core.h"
#include "debug.h"

using namespace tinyrv;

#define clamp_max(x, max) ( x > max ? max : x)
#define clamp_min(x, min) (x < min ? min: x)
////////////////////////////////////////////////////////////////////////////////

GShare::GShare(uint32_t BTB_size, uint32_t BHR_size)
  : BTB_(BTB_size, BTB_entry_t{false, 0x0, 0x0})
  , BHT_((1 << BHR_size), 0x0)
  , BHR_(0x0)
  , BTB_shift_(log2ceil(BTB_size))
  , BTB_mask_(BTB_size-1)
  , BHR_mask_((1 << BHR_size)-1) {
  //--
}

GShare::~GShare() {
  //--
}

uint32_t GShare::btb_index(uint32_t PC){
  return (PC >> 2) & BTB_mask_;
}

uint32_t GShare::pc_tag(uint32_t PC){
  return (PC >> 2) >> BTB_shift_;
}

uint32_t GShare::bht_index(uint32_t PC){
  return ((PC >> 2) ^ BHR_) & BHR_mask_;
}

uint32_t GShare::BTB_lookup(uint32_t PC) {
  uint32_t index = btb_index(PC);
  uint32_t tag = pc_tag(PC);

  BTB_entry_t btb_entry = BTB_[index];
  return (btb_entry.valid && btb_entry.tag == tag) ? btb_entry.target : PC + 4;
 }


/* Prediction. */
uint32_t GShare::predict(uint32_t PC) {
  uint32_t index = bht_index(PC);
  bool predict_taken = BHT_[index] >= 2;
  uint32_t next_PC = predict_taken ? BTB_lookup(PC) : PC + 4;

  DT(3, "*** GShare: predict PC=0x" << std::hex << PC << std::dec
        << ", next_PC=0x" << std::hex << next_PC << std::dec
        << ", predict_taken=" << predict_taken);
  return next_PC;
}

/* Update after prediction resolution. */
void GShare::update_BHT(uint32_t index, bool taken){
  BHT_[index] = taken ? 
  clamp_max(BHT_[index] + 1, 0b11) :
  clamp_min(BHT_[index] - 1, 0b00);
}

void GShare::update_BHR(bool taken){
  BHR_ = ((BHR_ << 1) | (taken ? 1 : 0)) & BHR_mask_;
}

void GShare::update_BTB(uint32_t PC, uint32_t next_PC){
  uint32_t index = btb_index(PC);
  uint32_t tag = pc_tag(PC);

  BTB_[index] = {true, tag, next_PC};
}

void GShare::update(uint32_t PC, uint32_t next_PC, bool taken) {
  DT(3, "*** GShare: update PC=0x" << std::hex << PC << std::dec
        << ", next_PC=0x" << std::hex << next_PC << std::dec
        << ", taken=" << taken);

  update_BHT(bht_index(PC), taken);
  update_BHR(taken);
  if (taken) {
    update_BTB(PC, next_PC);
  }

}

///////////////////////////////////////////////////////////////////////////////

GSharePlus::GSharePlus(uint32_t BTB_size, uint32_t BHR_size) {
  (void) BTB_size;
  (void) BHR_size;
}

GSharePlus::~GSharePlus() {
  //--
}

uint32_t GSharePlus::predict(uint32_t PC) {
  uint32_t next_PC = PC + 4;
  bool predict_taken = false;
  (void) PC;
  (void) next_PC;
  (void) predict_taken;

  // TODO: extra credit component

  DT(3, "*** GShare+: predict PC=0x" << std::hex << PC << std::dec
        << ", next_PC=0x" << std::hex << next_PC << std::dec
        << ", predict_taken=" << predict_taken);
  return next_PC;
}

void GSharePlus::update(uint32_t PC, uint32_t next_PC, bool taken) {
  (void) PC;
  (void) next_PC;
  (void) taken;

  DT(3, "*** GShare+: update PC=0x" << std::hex << PC << std::dec
        << ", next_PC=0x" << std::hex << next_PC << std::dec
        << ", taken=" << taken);

  // TODO: extra credit component
}


