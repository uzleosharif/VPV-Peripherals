/*
 * Copyright (c) 2019 -2021 MINRES Technologies GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _PRCI_H_
#define _PRCI_H_

#include "scc/tlm_target.h"

namespace vpvper {
namespace sifive {

class prci_regs;

class prci : public sc_core::sc_module, public scc::tlm_target<> {
public:
    SC_HAS_PROCESS(prci);// NOLINT
    sc_core::sc_port<sc_core::sc_signal_in_if<sc_core::sc_time>, 1, sc_core::SC_ZERO_OR_MORE_BOUND> hfxosc_i;
    sc_core::sc_in<bool> rst_i;
    sc_core::sc_out<sc_core::sc_time> hfclk_o;
    prci(sc_core::sc_module_name nm);
    virtual ~prci() override; // need to keep it in source file because of fwd declaration of prci_regs

protected:
    void hfxosc_cb();
    void reset_cb();
    void hfrosc_en_cb();
    void hfxosc_en_cb();
    void update_hfclk();
    sc_core::sc_time hfxosc_clk, hfrosc_clk, pll_clk, hfclk;
    std::unique_ptr<prci_regs> regs;
    sc_core::sc_event hfrosc_en_evt, hfxosc_en_evt;
};

} /* namespace sifive */
} /* namespace vpvper */

#endif /* _GPIO_H_ */
