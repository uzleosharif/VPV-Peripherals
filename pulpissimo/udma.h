/*
 * Copyright (c) 2019 -2022 MINRES Technologies GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "gen/udma.h"
//
#include "scc/memory.h"
#include "scc/tlm_target.h"

namespace vpvper::pulpissimo {
namespace gen {
class udma_regs;
}

class udma : public sc_core::sc_module, public scc::tlm_target<> {
 protected:
  class SPIM final {
   public:
    SPIM(gen::spi_channel_regs *);
    void spim_regs_cb();
    bool isCMDCFGOk();
    void printCMDCFG();
    int handleCommands();

   private:
    const unsigned kTX{0};
    const unsigned kRX{1};
    const unsigned kCMD{2};

    gen::spi_channel_regs *regs_{nullptr};
    // channel-wise
    std::array<bool, 3> is_enabled_{{false, false, false}};
  };

 public:
  sc_core::sc_in<sc_core::sc_time> clk_i{"clk_i"};
  sc_core::sc_in<bool> rst_i{"rst_i"};

  udma(sc_core::sc_module_name nm, scc::memory<512_kB, 32> *l2_mem);
  virtual ~udma() override;

 protected:
  sc_core::sc_time clk;
  std::unique_ptr<gen::udma_regs> regs;
  scc::memory<512_kB, 32> *l2_mem_{nullptr};
  SPIM spim_{&regs->i_spi};

  void clock_cb();
  void reset_cb();
};

}  // namespace vpvper::pulpissimo
