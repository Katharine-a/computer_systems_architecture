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
  , perceptron_table_((1 << BHR_size), std::array<int32_t,9>{0}) // initialize all weights and bias to 0, default taken prediction
  , input_mask_((1 << PARAM_SIZE) - 1) // mask for 9 LSBs of BHR
  , predict_taken_(true)
  {
  (void) BTB_size;
  (void) BHR_size;
}

GSharePlus::~GSharePlus() {
  //--
}

uint32_t GSharePlus::perceptron_table_index_(uint32_t PC){
  return (PC >> 2) & input_mask_; // 8 bit index
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
  uint32_t input_val =((BHR << 1) | 0b1 );
  std::array<int32_t, PARAM_SIZE> input = {0};
  /* Note that bias is element 0 of the array. */
  for (uint8_t i = 0 ; i < input.size(); i++) {
    input[i] = (input_val & (0b1 << i)) ? 1 : 0;
  }
  return input;
}

uint32_t GSharePlus::predict(uint32_t PC) {
  uint32_t index = perceptron_table_index_(PC);
  std::array<int32_t, PARAM_SIZE> input = get_input_(BHR_);
  bool predict_taken = run_perceptron_model_(index, input);
  predict_taken_ = predict_taken;
  uint32_t next_PC = predict_taken ? BTB_lookup(PC) : PC + 4;
  // TODO: extra credit component

  DT(3, "*** GShare+: predict PC=0x" << std::hex << PC << std::dec
        << ", next_PC=0x" << std::hex << next_PC << std::dec
        << ", predict_taken=" << predict_taken);
  return next_PC;
}

void GSharePlus::update_perceptron_table_(uint32_t index, bool taken, std::array<int32_t, PARAM_SIZE> input){
  if (predict_taken_ != taken) { /* only update if there was a misprediction */
    std::array<int32_t, PARAM_SIZE> weights = perceptron_table_[index];
    for (uint8_t i = 0; i < weights.size(); i++) {
      weights[i] += weights[i] + (taken ? 1 : -1)*input[i];
  }
  }
}

void GSharePlus::update(uint32_t PC, uint32_t next_PC, bool taken) {
  (void) PC;
  (void) next_PC;
  (void) taken;

  DT(3, "*** GShare+: update PC=0x" << std::hex << PC << std::dec
        << ", next_PC=0x" << std::hex << next_PC << std::dec
        << ", taken=" << taken);

  // TODO: extra credit component
  update_perceptron_table_(perceptron_table_index_(PC), taken, get_input_(PC));
  update_BHR(taken);
  if (taken) {
    update_BTB(PC, next_PC);
  }
}


