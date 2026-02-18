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

#include <memory>
#include <iostream>
#include <util.h>
#include "types.h"

namespace tinyrv {

template <typename T>
class PipelineReg : public SimObject<PipelineReg<T>> {
public:
  PipelineReg(const SimContext& ctx, const char* name)
    : SimObject<PipelineReg<T>>(ctx, name)
    , valid_(false)
    , valid_next_(false)
  {}

  ~PipelineReg() {}

  bool valid() const {
    return valid_;
  }

  const T& data() {
    return data_;
  }

  void push(const T& data) {
    data_next_ = data;
    valid_next_ = true;
  }

  void pop() {
    valid_next_ = false;
  }

  void reset() {
    valid_ = false;
    valid_next_ = false;
  }

  void tick() {
    data_ = data_next_;
    valid_ = valid_next_;
  }

protected:
  T data_;
  T data_next_;
  bool valid_;
  bool valid_next_;
};

}