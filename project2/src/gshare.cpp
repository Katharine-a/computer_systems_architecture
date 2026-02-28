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

#define CLAMP_MAX(x, max) ( x > max ? max : x)
#define CLAMP_MIN(x, min) (x < min ? min: x)
////////////////////////////////////////////////////////////////////////////////

GShare::GShare(uint32_t BTB_size, uint32_t BHR_size)
  : BHR_(0x0)
  , BTB_(BTB_size, BTB_entry_t{false, 0x0, 0x0})
  , BHT_((1 << BHR_size), 0x0)
  , BTB_shift_(log2ceil(BTB_size))
  , BTB_mask_(BTB_size-1)
  , BHR_mask_((1 << BHR_size)-1) {
  //--
}

GShare::~GShare() {
  //--
}

uint32_t GShare::btb_index(uint32_t PC) const {
  return (PC >> 2) & BTB_mask_;
}

uint32_t GShare::pc_tag(uint32_t PC) const {
  return (PC >> 2) >> BTB_shift_;
}

uint32_t GShare::bht_index(uint32_t PC) const {
  return ((PC >> 2) ^ BHR_) & BHR_mask_;
}

bool GShare::gshare_predict_taken(uint32_t PC) const {
  return BHT_.at(bht_index(PC)) >= 2;
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
  CLAMP_MAX(BHT_[index] + 1, 0b11) :
  CLAMP_MIN(BHT_[index] - 1, 0b00);
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

GSharePlus::GSharePlus(uint32_t BTB_size, uint32_t BHR_size)
  : GShare(BTB_size, BHR_size)
  , perceptron_table_((1 << BHR_size), std::array<int32_t,PARAM_SIZE>{0})
  , selector_((1 << BHR_size), 0)   /* 0 = prefer gshare; 2-bit counters, 0-1 gshare, 2-3 perceptron */
  , choice_((1 << BHR_size), false)
  , selector_mask_((1 << BHR_size) - 1)
  , input_mask_((1 << BHR_size) - 1)  /* 8-bit index for perceptron table */
  , predict_taken_(true)
  {
  (void) BTB_size;
  (void) BHR_size;
}

GSharePlus::~GSharePlus() {
  //--
}

uint32_t GSharePlus::perceptron_table_index_(uint32_t PC){
  return (PC >> 2) & input_mask_;
}

uint32_t GSharePlus::selector_index_(uint32_t PC){
  return (PC >> 2) & selector_mask_;
}

bool GSharePlus::run_perceptron_model_(uint32_t index, std::array<int32_t, PARAM_SIZE> input){
  int32_t result = 0;
  std::array<int32_t,PARAM_SIZE> weights = perceptron_table_[index];
  for (uint8_t i = 0; i < weights.size(); i++) {
    result += weights[i] * input[i];
  }
  return result >= 0;
}

std::array<int32_t, PARAM_SIZE> GSharePlus::get_input_(uint32_t BHR){
  uint32_t input_val = ((BHR << 1) | 0b1) & ((1u << PARAM_SIZE) - 1);
  std::array<int32_t, PARAM_SIZE> input = {0};
  /* Bias is element 0 of the array. */
  for (uint8_t i = 0; i < input.size(); i++) {
    input[i] = (input_val & (1u << i)) ? 1 : 0;
  }
  return input;
}

uint32_t GSharePlus::predict(uint32_t PC) {
  bool gshare_taken = gshare_predict_taken(PC);
  uint32_t pidx = perceptron_table_index_(PC);
  bool perceptron_taken = run_perceptron_model_(pidx, get_input_(BHR_));

  uint32_t sel_idx = selector_index_(PC);
  bool use_gshare = (selector_[sel_idx] < 2);  /* 0,1 -> gshare; 2,3 -> perceptron */
  choice_[sel_idx] = !use_gshare;               /* true if we used perceptron */
  predict_taken_ = use_gshare ? gshare_taken : perceptron_taken;

  uint32_t next_PC = predict_taken_ ? BTB_lookup(PC) : PC + 4;
  DT(3, "*** GShare+: predict PC=0x" << std::hex << PC << std::dec
        << ", next_PC=0x" << std::hex << next_PC << std::dec
        << ", predict_taken=" << predict_taken_ << " ("
        << (use_gshare ? "gshare" : "perceptron") << ")");
  return next_PC;
}

void GSharePlus::update_perceptron_table_(uint32_t index, bool taken, std::array<int32_t, PARAM_SIZE> input){
  if (predict_taken_ != taken) {
    for (uint8_t i = 0; i < PARAM_SIZE; i++) {
      int32_t delta = (taken ? 1 : -1) * input[i];
      int32_t w = perceptron_table_[index][i] + delta;
      if (w > PERCEPTRON_WEIGHT_MAX) w = PERCEPTRON_WEIGHT_MAX;
      if (w < PERCEPTRON_WEIGHT_MIN) w = PERCEPTRON_WEIGHT_MIN;
      perceptron_table_[index][i] = static_cast<int32_t>(w);
    }
  }
}

void GSharePlus::update_selector_(uint32_t sel_idx, bool used_perceptron, bool pred_correct){
  int8_t& c = selector_[sel_idx];
  if (used_perceptron) {
    if (pred_correct)
      c = (c < 3) ? c + 1 : 3;
    else
      c = (c > 0) ? c - 1 : 0;
  } else {
    if (pred_correct)
      c = (c > 0) ? c - 1 : 0;
    else
      c = (c < 3) ? c + 1 : 3;
  }
}

void GSharePlus::update(uint32_t PC, uint32_t next_PC, bool taken) {
  DT(3, "*** GShare+: update PC=0x" << std::hex << PC << std::dec
        << ", next_PC=0x" << std::hex << next_PC << std::dec
        << ", taken=" << taken);

  uint32_t sel_idx = selector_index_(PC);
  bool used_perceptron = choice_[sel_idx];
  bool gshare_pred = gshare_predict_taken(PC);
  bool perceptron_pred = run_perceptron_model_(perceptron_table_index_(PC), get_input_(BHR_));
  bool pred_correct = used_perceptron ? (perceptron_pred == taken) : (gshare_pred == taken);
  update_selector_(sel_idx, used_perceptron, pred_correct);

  predict_taken_ = used_perceptron ? perceptron_pred : gshare_pred;
  std::array<int32_t, PARAM_SIZE> input = get_input_(BHR_);

  GShare::update(PC, next_PC, taken);
  update_perceptron_table_(perceptron_table_index_(PC), taken, input);
}


