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

#include <vector>

namespace tinyrv {

class BranchPredictor {
public:
  virtual ~BranchPredictor() {}

  virtual uint32_t predict(uint32_t PC) {
      return PC + 4;
  };

  virtual void update(uint32_t PC, uint32_t next_PC, bool taken) {
      (void) PC;
      (void) next_PC;
      (void) taken;
  };
};

class GShare : public BranchPredictor {
public:
  GShare(uint32_t BTB_size, uint32_t BHR_size);

  ~GShare() override;

  uint32_t predict(uint32_t PC) override;
  void update(uint32_t PC, uint32_t next_PC, bool taken) override;

  // TODO: Add your own methods here
};

class GSharePlus : public BranchPredictor {
public:
  GSharePlus(uint32_t BTB_size, uint32_t BHR_size);

  ~GSharePlus() override;

  uint32_t predict(uint32_t PC) override;
  void update(uint32_t PC, uint32_t next_PC, bool taken) override;

  // TODO: extra credit component
};

}
