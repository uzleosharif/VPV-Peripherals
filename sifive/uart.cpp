/*
 * Copyright (c) 2019 -2021 MINRES Technologies GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "uart.h"
#include "gen/uart_regs.h"
#include <generic/tlm_extensions.h>


#include <scc/report.h>
#include <scc/utilities.h>

using namespace std;

namespace vpvper {
namespace sifive {
using namespace sc_core;

SC_HAS_PROCESS(uart);// NOLINT

uart::uart(sc_core::sc_module_name const& nm)
: sc_core::sc_module(nm)
, tlm_target<>(clk)
, NAMED(clk_i)
, NAMED(rst_i)
, NAMED(tx_o)
, NAMED(rx_i)
, NAMED(irq_o)
, NAMED(bit_true_transfer, false)
, NAMEDD(regs, uart_regs)
, NAMED(rx_fifo, 8)
, NAMED(tx_fifo, 8) {
    regs->registerResources(*this);
    SC_METHOD(clock_cb);
    sensitive << clk_i;
    SC_METHOD(reset_cb);
    sensitive << rst_i;
    dont_initialize();
    SC_THREAD(transmit_data);
    rx_i.register_nb_transport(
        [this](tlm::scc::tlm_signal_gp<bool> &gp, tlm::tlm_phase &phase, sc_core::sc_time &delay) -> tlm::tlm_sync_enum {
            this->receive_data(gp, delay);
            return tlm::TLM_COMPLETED;
        });
    regs->txdata.set_write_cb([this](scc::sc_register<uint32_t> &reg, uint32_t data, sc_core::sc_time d) -> bool {
        if (!this->regs->in_reset()) {
            reg.put(data);
            tx_fifo.nb_write(static_cast<uint8_t>(regs->r_txdata.data));
            regs->r_txdata.full = tx_fifo.num_free() == 0;
            regs->r_ip.txwm = regs->r_txctrl.txcnt <= (7 - tx_fifo.num_free()) ? 1 : 0;
            update_irq();
        }
        return true;
    });
    regs->rxdata.set_read_cb([this](const scc::sc_register<uint32_t> &reg, uint32_t &data, sc_core::sc_time d) -> bool {
        if (!this->regs->in_reset()) {
            uint8_t val;
            if (rx_fifo.nb_read(val)) {
                regs->r_rxdata.data = val;
                if (regs->r_rxctrl.rxcnt <= rx_fifo.num_available()) {
                    regs->r_ip.rxwm = 1;
                    update_irq();
                }
            }
            data = reg.get() & reg.rdmask;
        }
        return true;
    });
    regs->ie.set_write_cb([this](scc::sc_register<uint32_t> &reg, uint32_t data, sc_core::sc_time d) -> bool {
        update_irq();
        return true;
    });
    regs->ip.set_write_cb([this](scc::sc_register<uint32_t> &reg, uint32_t data, sc_core::sc_time d) -> bool {
        update_irq();
        return true;
    });
}

uart::~uart() = default;

void uart::update_irq() {
    irq_o = (regs->r_ip.rxwm == 1 && regs->r_ie.rxwm == 1) || (regs->r_ip.txwm == 1 && regs->r_ie.txwm == 1);
}

void uart::clock_cb() { this->clk = clk_i.read(); }

void uart::reset_cb() {
    if (rst_i.read())
        regs->reset_start();
    else
        regs->reset_stop();
}

void uart::transmit_data() {
    uint8_t txdata;
    tlm::tlm_phase phase(tlm::BEGIN_REQ);
    sc_core::sc_time delay(SC_ZERO_TIME);
    sc_core::sc_time bit_duration(SC_ZERO_TIME);
    sc_core::sc_time start_time;

    auto set_bit = [&](bool val) {
        auto *gp = tlm::scc::tlm_signal_gp<>::create();
        auto *ext = new vpvper::generic::tlm_signal_uart_extension();
        ext->tx.data_bits = 8;
        ext->tx.parity = false;
        ext->start_time = start_time;
        ext->tx.baud_rate = static_cast<unsigned>(1 / bit_duration.to_seconds());
        ext->tx.stop_bits = 1 + regs->r_txctrl.nstop;
        ext->tx.data = txdata;
        gp->set_extension(ext);
        gp->set_command(tlm::TLM_WRITE_COMMAND);
        gp->set_value(val);
        gp->acquire();
        phase = tlm::BEGIN_REQ;
        delay = SC_ZERO_TIME;
        tx_o->nb_transport_fw(*gp, phase, delay);
        gp->release();
        if (delay < bit_duration) wait(bit_duration - delay);
    };
    wait(delay);
    while (true) {
        set_bit(true);
        wait(tx_fifo.data_written_event());
        while (tx_fifo.nb_read(txdata)) {
            regs->r_txdata.full = tx_fifo.num_free() == 0;
            regs->r_ip.txwm = regs->r_txctrl.txcnt <= (7 - tx_fifo.num_free()) ? 1 : 0;
            bit_duration = (regs->r_div.div + 1) * clk;
            start_time = sc_core::sc_time_stamp();
            set_bit(false); // start bit
            if (bit_true_transfer.get_value()) {
                for (int i = 8; i > 0; --i) set_bit(txdata & (1 << (i - 1))); // 8 data bits, MSB first
                if (regs->r_txctrl.nstop) set_bit(true);                      // stop bit 1
            } else
                wait(8 * bit_duration);
            set_bit(true); // stop bit 1/2
        }
    }
}

void uart::receive_data(tlm::scc::tlm_signal_gp<> &gp, sc_core::sc_time &delay) {
    vpvper::generic::tlm_signal_uart_extension *ext{nullptr};
    gp.get_extension(ext);
    if (ext && ext->start_time != rx_last_start) {
        auto data = static_cast<uint8_t>(ext->tx.data);
        if (ext->tx.parity || ext->tx.data_bits != 8) data = rand(); // random value if wrong config
        rx_fifo.write(data);
        if (regs->r_rxctrl.rxcnt <= rx_fifo.num_available()) {
            regs->r_ip.rxwm = 1;
            update_irq();
        }
        rx_last_start = ext->start_time; // omit repeated handling of signal changes
    }
    gp.set_response_status(tlm::TLM_OK_RESPONSE);
}

} /* namespace sifive */
} /* namespace vpvper */
