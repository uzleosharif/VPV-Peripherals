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

class udma final : public sc_core::sc_module, public scc::tlm_target<> {
 private:
  class SPIM final {
   public:
    // raw pointers here are ok as underlying resources aren't dynamically allocated by owner so no risk of me
    // deallocating these resources
    SPIM(gen::spi_channel_regs *, SoC *, std::array<tlm::tlm_initiator_socket<> *, 4> *);
    // as no class can inherit from this class hence no need to provide a virtual destructor (even though its provided
    // by default) and no need to suppress copy/move stuff (they are also implictly defaulted)
    // this simplifies as per rule-of-zero

    void spim_regs_cb();

   private:
    const unsigned kTX{0};
    const unsigned kRX{1};
    const unsigned kCMD{2};

    gen::spi_channel_regs *regs_{nullptr};
    SoC *soc_{nullptr};
    std::array<tlm::tlm_initiator_socket<> *, 4> *spim_sockets_{nullptr};
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

  // raw pointers here are ok as underlying resources aren't dynamically allocated by owner so no risk of me
  // deallocating these resources
  // further passing array as by value is ok as its just 4 pointers
  udma(sc_core::sc_module_name nm, SoC *, std::array<tlm::tlm_initiator_socket<> *, 4>);
  // as no class can inherit from this class hence no need to provide a virtual destructor (even though its provided
  // by default) and no need to suppress copy/move stuff (they are also implictly defaulted)
  // this simplifies as per rule-of-zero

 private:
  sc_core::sc_time clk;
  std::unique_ptr<gen::udma_regs> regs;
  SoC *soc_{nullptr};
  std::array<tlm::tlm_initiator_socket<> *, 4> spim_sockets_{{nullptr, nullptr, nullptr, nullptr}};
  SPIM spim_{&regs->i_spi, soc_, &spim_sockets_};

  void clock_cb();
  void reset_cb();
};

}  // namespace vpvper::pulpissimo
