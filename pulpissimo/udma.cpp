/*
 * Copyright (c) 2019 -2022 MINRES Technologies GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "udma.h"

#include "gen/udma.h"
#include "scc/utilities.h"
#include "util.h"

// TODO: chip select logic -> what chip select is live and associated error handling

namespace vpvper::pulpissimo {
SC_HAS_PROCESS(udma);  // NOLINT

udma::udma(sc_core::sc_module_name nm, scc::memory<512_kB, 32> *l2_mem)
    : sc_core::sc_module(nm), scc::tlm_target<>(clk), NAMEDD(regs, gen::udma_regs), l2_mem_{l2_mem} {
  regs->registerResources(*this);

  SC_METHOD(clock_cb);
  sensitive << clk_i;
  SC_METHOD(reset_cb);
  sensitive << rst_i;

  // CTRL_CFG_CG register
  regs->CTRL_CFG_CG.set_read_cb(vpvper::pulpissimo::simple_read);
  regs->CTRL_CFG_CG.set_write_cb(vpvper::pulpissimo::simple_write);

  spim_.spim_regs_cb();
}

udma::~udma() {}  // NOLINT

void udma::clock_cb() { this->clk = clk_i.read(); }

void udma::reset_cb() {
  if (rst_i.read()) {
    regs->reset_start();
  } else {
    regs->reset_stop();
  }
}

udma::SPIM::SPIM(gen::spi_channel_regs *regs) : regs_{regs} {}

void udma::SPIM::spim_regs_cb() {
  // SPIM_RX_SADDR register
  // right now the UDMA moel is instantaneous i.e. the whole txn data is received atomically from external device
  // hence this reg should always be 0 (possibly by some other part of the model)
  regs_->SPIM_RX_SADDR.set_read_cb(vpvper::pulpissimo::simple_read);
  regs_->SPIM_RX_SADDR.set_write_cb([this](scc::sc_register<uint32_t> &reg, uint32_t v, sc_core::sc_time d) -> bool {
    if (v >= 0x1c000000 && v < 0x1c080000) {
      reg.put(v);
      return true;
    } else {
      return false;
    }
  });

  // SPIM_RX_SIZE register
  // similar to above read_cb of RX_SPIM_SADDR, we expect to return 0 as 'size left' of rx-buffer here
  regs_->SPIM_RX_SIZE.set_read_cb(vpvper::pulpissimo::simple_read);
  regs_->SPIM_RX_SIZE.set_write_cb([this](scc::sc_register<uint32_t> &reg, uint32_t v, sc_core::sc_time d) -> bool {
    if (v <= 1048576) {
      reg.put(v);
      return true;
    } else {
      return false;
    }
  });

  // SPIM_RX_CFG register
  regs_->SPIM_RX_CFG.set_read_cb(vpvper::pulpissimo::simple_read);
  regs_->SPIM_RX_CFG.set_write_cb(vpvper::pulpissimo::simple_write);

  // SPIM_CMD_SADDR register
  regs_->SPIM_CMD_SADDR.set_read_cb(vpvper::pulpissimo::simple_read);
  regs_->SPIM_CMD_SADDR.set_write_cb([this](scc::sc_register<uint32_t> &reg, uint32_t v, sc_core::sc_time d) -> bool {
    if (v >= 0x1c000000 && v < 0x1c080000) {
      reg.put(v);
      return true;
    } else {
      return false;
    }
  });

  // SPIM_CMD_SIZE register
  regs_->SPIM_CMD_SIZE.set_read_cb(vpvper::pulpissimo::simple_read);
  regs_->SPIM_CMD_SIZE.set_write_cb([this](scc::sc_register<uint32_t> &reg, uint32_t v, sc_core::sc_time d) -> bool {
    if (v <= 1048576) {
      reg.put(v);
      return true;
    } else {
      return false;
    }
  });

  // SPIM_CMD_CFG register
  regs_->SPIM_CMD_CFG.set_read_cb(vpvper::pulpissimo::simple_read);
  regs_->SPIM_CMD_CFG.set_write_cb([this](scc::sc_register<uint32_t> &reg, uint32_t v, sc_core::sc_time d) {
    reg.put(v);

    // make sure that command configurations programmed by CPU are valid before inspecting the CMD buffer
    bool status{isCMDCFGOk()};
    if (!status) {
      return false;
    }
    exit(1);
    // now inspecting the command buffer to update state
    status = handleCommands();
    if (!status) {
      return false;
    }

    return true;
  });
}

bool udma::SPIM::isCMDCFGOk() {
  printCMDCFG();
  return true;
}

void udma::SPIM::printCMDCFG() {
  gen::spi_channel_regs::SPIM_CMD_CFG_t current_cfg{regs_->SPIM_CMD_CFG.get()};
  std::cout << "EN = " << current_cfg.EN << "\t";
  std::cout << "PENDING = " << current_cfg.PENDING << "\t";
  std::cout << "CLR = " << current_cfg.CLR << "\t";
  std::cout << "DATASIZE = " << current_cfg.DATASIZE << "\t";
  std::cout << "CONTINOUS = " << current_cfg.CONTINOUS << "\n";
}

int udma::SPIM::handleCommands() {
  // 0x0000_0032 : sets the configuration for the SPIM IP t0 0x32 -> clock divided by 50 !! -> HW info not relevant
  // 0x1000_0001 : sets the chip select -> select CSN1 -> HW info not relevant
  // 0x7007_000f : receive data
  // 0x9000_0001 : clear the chip select -> clear CSN1 -> HW info not relevant

  return true;
}

}  // namespace vpvper::pulpissimo
