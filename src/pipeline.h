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
class PipelineReg {
public:
  PipelineReg()
    : valid_(false)
  {}

  ~PipelineReg() {}

  bool empty() const {
    return !valid_;
  }

  const T& data() {
    return data_;
  }

  void push(const T& data) {
    assert(!valid_ && "PipelineReg: push() called on non-empty register");
    data_ = data;
    valid_ = true;
  }

  void pop() {
    valid_ = false;
  }

  void reset() {
    valid_ = false;
  }

protected:
  T data_;
  bool valid_;
};

}