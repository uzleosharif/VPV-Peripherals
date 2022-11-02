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
  class SPIM final : public sc_core::sc_module {
    SC_HAS_PROCESS(SPIM);

   public:
    // passing raw pointer is dangerous but rest assured I am not going to manage this memory
    // rather the guy who owns it ( allocates it ) is responsible for releasing it
    SPIM(gen::spi_channel_regs *, SoC *);
    // as no class can inherit from this class hence no need to provide a virtual destructor (even though its provided
    // by default) and no need to suppress copy/move stuff (they are also implictly defaulted)
    // this simplifies as per rule-of-zero

    void regs_cb();

   private:
    const unsigned kTX{0};
    const unsigned kRX{1};
    const unsigned kCMD{2};
    const sc_core::sc_time kEOTDelay{10, SC_US};

    gen::spi_channel_regs *regs_{nullptr};
    SoC *soc_{nullptr};
    gen::spi_channel_regs::SPIM_CMD_CFG_t current_cfg_{};
    // // channel-wise
    // std::array<bool, 3> is_enabled_{{false, false, false}};
    bool transfer_started_{false};
    size_t chip_select_{0};
    sc_core::sc_event eot_event_{};

    void printCMDCFG();
    // bool isCMDCFGOk();
    int handleCommands();
    void notifyEventGenerator();
  };

  class I2S final : public sc_core::sc_module {
    SC_HAS_PROCESS(I2S);

   public:
    // passing raw pointer is dangerous but rest assured I am not going to manage this memory
    // rather the guy who owns it ( allocates it ) is responsible for releasing it
    I2S(gen::i2s_channel_regs *, SoC *);
    // as no class can inherit from this class hence no need to provide a virtual destructor (even though its provided
    // by default) and no need to suppress copy/move stuff (they are also implictly defaulted)
    // this simplifies as per rule-of-zero

    void regs_cb();

   private:
    // const unsigned kTX{0};
    // const unsigned kRX{1};
    // const unsigned kCMD{2};
    // const sc_core::sc_time kEOTDelay{10, SC_US};

    gen::i2s_channel_regs *regs_{nullptr};
    SoC *soc_{nullptr};
    // gen::spi_channel_regs::SPIM_CMD_CFG_t current_cfg_{};
    // // channel-wise
    // std::array<bool, 3> is_enabled_{{false, false, false}};
    // bool transfer_started_{false};
    // size_t chip_select_{0};
    // sc_core::sc_event eot_event_{};

    // void printCMDCFG();
    // // bool isCMDCFGOk();
    // int handleCommands();
    void notifyEventGenerator();
  };

 public:
  sc_core::sc_in<sc_core::sc_time> clk_i{"clk_i"};
  sc_core::sc_in<bool> rst_i{"rst_i"};

  // passing raw pointer is dangerous but rest assured I am not going to manage this memory
  // rather the guy who owns it ( allocates it ) is responsible for releasing it
  udma(sc_core::sc_module_name, SoC *);
  // as no class can inherit from this class hence no need to provide a virtual destructor (even though its provided
  // by default) and no need to suppress copy/move stuff (they are also implictly defaulted)
  // this simplifies as per rule-of-zero

 private:
  sc_core::sc_time clk;
  std::unique_ptr<gen::udma_regs> regs;
  SoC *soc_{nullptr};
  SPIM spim_{&regs->i_spi, soc_};
  I2S i2s_{&regs->i_i2s, soc_};

  void clock_cb();
  void reset_cb();
};

}  // namespace vpvper::pulpissimo
