/*
 * Copyright (c) 2019 -2022 MINRES Technologies GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "gen/udma.h"
#include "util.h"
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
    SPIM(gen::spi_channel_regs *, l2mem_t *);
    void spim_regs_cb();

   private:
    const unsigned kTX{0};
    const unsigned kRX{1};
    const unsigned kCMD{2};

    gen::spi_channel_regs *regs_{nullptr};
    l2mem_t *l2_mem_{nullptr};
    gen::spi_channel_regs::SPIM_CMD_CFG_t current_cfg_{};
    // // channel-wise
    // std::array<bool, 3> is_enabled_{{false, false, false}};
    bool transfer_started_{false};
    size_t chip_select_{0};

    void printCMDCFG();
    // bool isCMDCFGOk();
    int handleCommands();
  };

 public:
  sc_core::sc_in<sc_core::sc_time> clk_i{"clk_i"};
  sc_core::sc_in<bool> rst_i{"rst_i"};

  udma(sc_core::sc_module_name nm, l2mem_t *l2_mem);
  ~udma() override;
  // even though below are not needed but they
  // (i) make my intent clear that this should not be defined in future to avoid slicing problem
  // (ii) nice compiler message if copy/move are done
  udma(const udma &) = delete;
  udma &operator=(const udma &) = delete;
  udma(udma &&) = delete;
  udma &operator=(udma &&) = delete;
  // in case copying is needed then do a clone function
  // virtual std::unique_ptr<udma> clone() const;

 protected:
  sc_core::sc_time clk;
  std::unique_ptr<gen::udma_regs> regs;
  l2mem_t *l2_mem_{nullptr};
  SPIM spim_{&regs->i_spi, l2_mem_};

  void clock_cb();
  void reset_cb();
};

}  // namespace vpvper::pulpissimo
