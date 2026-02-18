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

#include "core.h"

namespace tinyrv {

class ProcessorImpl {
public:

  ProcessorImpl();
  ~ProcessorImpl();

  void attach_ram(RAM* mem);

  int run(bool riscv_test);

  void showStats();

private:
  void reset();

  Core::Ptr core_;
};

}
