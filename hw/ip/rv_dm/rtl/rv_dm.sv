// Copyright lowRISC contributors (OpenTitan project).
// Portions Copyright (c) 2025 Maximilian Kirschner
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Top-level debug module (DM)
//
// This module implements the RISC-V debug specification version 0.13,
//
// This toplevel wraps the PULP debug module available from
// https://github.com/pulp-platform/riscv-dbg to match the needs of
// the TL-UL-based lowRISC chip design.

`include "prim_assert.sv"

module rv_dm
  import rv_dm_reg_pkg::*;
#(
  parameter logic [31:0]          IdcodeValue            = 32'h 0000_0001
) (
  input  logic                clk_i,       // clock
  input  logic                rst_ni,      // asynchronous reset active low, connect PoR
                                           // here, not the system reset
  input  logic [31:0]         next_dm_addr_i, // static word address of the next debug module.

  output logic                ndmreset_req_o,  // non-debug module reset
  output logic                dmactive_o,  // debug module is active
  output logic [NrHarts-1:0]  debug_req_o, // async debug request
  input  logic [NrHarts-1:0]  unavailable_i, // communicate whether the hart is unavailable
                                             // (e.g.: power down)

  // bus device with debug memory, for an execution based technique
  input  tlul_pkg::tl_h2d_t  mem_tl_d_i,
  output tlul_pkg::tl_d2h_t  mem_tl_d_o,

  // bus host, for system bus accesses
  output tlul_pkg::tl_h2d_t  sba_tl_h_o,
  input  tlul_pkg::tl_d2h_t  sba_tl_h_i,

  // JTAG TAP
  input  jtag_pkg::jtag_req_t jtag_i,
  output jtag_pkg::jtag_rsp_t jtag_o
);

  ///////////////////////////
  // Parameter Definitions //
  ///////////////////////////

  // import lc_ctrl_pkg::lc_tx_test_true_strict;

  `ASSERT_INIT(paramCheckNrHarts, NrHarts > 0)

  // static debug hartinfo
  localparam dm::hartinfo_t DebugHartInfo = '{
    zero1:      '0,
    nscratch:   2, // Debug module needs at least two scratch regs
    zero0:      0,
    dataaccess: 1'b1, // data registers are memory mapped in the debugger
    datasize:   dm::DataCount,
    dataaddr:   dm::DataAddr
  };

  dm::hartinfo_t [NrHarts-1:0]      hartinfo;
  for (genvar i = 0; i < NrHarts; i++) begin : gen_dm_hart_ctrl
    assign hartinfo[i] = DebugHartInfo;
  end

  // Currently only 32 bit busses are supported by our TL-UL IP
  localparam int BusWidth = 32;
  // all harts have contiguous IDs
  localparam logic [NrHarts-1:0] SelectableHarts = {NrHarts{1'b1}};


  dm::dmi_req_t  dmi_req;
  dm::dmi_resp_t dmi_rsp;
  logic dmi_req_valid, dmi_req_ready;
  logic dmi_rsp_valid, dmi_rsp_ready;
  logic dmi_rst_n;

  ////////////////////////
  // NDM Reset Tracking //
  ////////////////////////

  // logic reset_req_en;
  // logic ndmreset_req, ndmreset_ack;
  // logic ndmreset_req_qual;
  // // SEC_CM: DM_EN.CTRL.LC_GATED
  // assign reset_req_en = lc_tx_test_true_strict(lc_hw_debug_en_gated[LcEnResetReq]);
  // assign ndmreset_req_o = ndmreset_req_qual & reset_req_en;

  // // Sample the processor reset to detect lc reset assertion.
  // logic lc_rst_asserted_async;
  // prim_flop_2sync #(
  //   .Width(1),
  //   .ResetValue(1) // Resets to 1 to indicate assertion.
  // ) u_prim_flop_2sync_lc_rst_assert (
  //   .clk_i, // Use RV_DM clock
  //   .rst_ni(rst_lc_ni), // Use LC reset here that resets the entire system except the RV_DM.
  //   .d_i(1'b0), // Set to 0 to indicate deassertion.
  //   .q_o(lc_rst_asserted_async)
  // );

  // // Note that the output of the above flops can be metastable at reset assertion, since the reset
  // // signal is coming from a different clock domain and has not been synchronized with clk_i.
  // logic lc_rst_asserted;
  // prim_flop_2sync #(
  //   .Width(1)
  // ) u_prim_flop_2sync_lc_rst_sync (
  //   .clk_i,
  //   .rst_ni,
  //   .d_i(lc_rst_asserted_async),
  //   .q_o(lc_rst_asserted)
  // );

  // // The acknowledgement pulse sets the dmstatus.allhavereset / dmstatus.anyhavereset registers in
  // // RV_DM. It should only be asserted once an NDM reset request has been fully completed.
  // logic ndmreset_pending_q;
  // logic lc_rst_pending_q;
  // always_ff @(posedge clk_i or negedge rst_ni) begin : p_ndm_reset
  //   if (!rst_ni) begin
  //     ndmreset_pending_q <= 1'b0;
  //     lc_rst_pending_q <= 1'b0;
  //   end else begin
  //     // Only set this if there was no previous pending NDM request.
  //     if (ndmreset_req && !ndmreset_pending_q) begin
  //       ndmreset_pending_q <= 1'b1;
  //     end else if (ndmreset_ack && ndmreset_pending_q) begin
  //       ndmreset_pending_q <= 1'b0;
  //     end
  //     // We only track lc resets that are asserted during an active ndm reset request..
  //     if (ndmreset_pending_q && lc_rst_asserted) begin
  //       lc_rst_pending_q <= 1'b1;
  //     end else if (ndmreset_ack && lc_rst_pending_q) begin
  //       lc_rst_pending_q <= 1'b0;
  //     end
  //   end
  // end

  // // In order to ACK the following conditions must be met
  // // 1) an NDM reset request was asserted and is pending
  // // 2) a lc reset was asserted after the NDM reset request
  // // 3) the NDM reset request was deasserted
  // // 4) the NDM lc request was deasserted
  // // 5) the debug module has been ungated for operation (depending on LC state, OTP config and CSR)
  // assign ndmreset_ack = ndmreset_pending_q &&
  //                       lc_rst_pending_q &&
  //                       !ndmreset_req &&
  //                       !lc_rst_asserted &&
  //                       reset_req_en;

  // JTAG TAP
  dmi_jtag #(
    .IdcodeValue    (IdcodeValue),
    .NumDmiWordAbits(7)
  ) dap (
    .clk_i            (clk_i),
    .rst_ni           (rst_ni),
    .testmode_i       (1'b0),
    .test_rst_ni      (1'b1),

    .dmi_rst_no       (dmi_rst_n),
    .dmi_req_o        (dmi_req),
    .dmi_req_valid_o  (dmi_req_valid),
    .dmi_req_ready_i  (dmi_req_ready),

    .dmi_resp_i       (dmi_rsp      ),
    .dmi_resp_ready_o (dmi_rsp_ready),
    .dmi_resp_valid_i (dmi_rsp_valid),

    //JTAG
    .tck_i            (jtag_i.tck),
    .tms_i            (jtag_i.tms),
    .trst_ni          (1'b1      ), // This signal needs to be held high, otherwise dmi_cdc would be optimized out.
    .td_i             (jtag_i.tdi),
    .td_o             (jtag_o.tdo),
    .tdo_oe_o         (jtag_o.tdo_oe)
  );

  /////////////////////////////////////////
  // System Bus Access Port (TL-UL Host) //
  /////////////////////////////////////////

  logic                   host_req;
  logic   [BusWidth-1:0]  host_add;
  logic                   host_we;
  logic   [BusWidth-1:0]  host_wdata;
  logic [BusWidth/8-1:0]  host_be;
  logic                   host_gnt;
  logic                   host_r_valid;
  logic   [BusWidth-1:0]  host_r_rdata;
  logic                   host_r_err;
  logic                   host_r_other_err;

  tlul_adapter_host #(
    .MAX_REQS(1),
    .EnableDataIntgGen(1),
    .EnableRspDataIntgCheck(1)
  ) tl_adapter_host_sba (
    .clk_i,
    .rst_ni,
    .req_i        (host_req),
    .instr_type_i (prim_mubi_pkg::MuBi4False),
    .gnt_o        (host_gnt),
    .addr_i       (host_add),
    .we_i         (host_we),
    .wdata_i      (host_wdata),
    .wdata_intg_i ('0),
    .be_i         (host_be),
    .user_rsvd_i  ('0),
    .valid_o      (host_r_valid),
    .rdata_o      (host_r_rdata),
    .rdata_intg_o (),
    .err_o        (host_r_err),
    .intg_err_o   (host_r_other_err),
    .tl_o         (sba_tl_h_o),
    .tl_i         (sba_tl_h_i)
  );

  //////////////////////////////////////
  // Debug Memory Port (TL-UL Device) //
  //////////////////////////////////////

  logic                         device_req;
  logic                         device_we;
  logic                         device_re;
  logic [BusWidth/8-1:0]        device_be;
  logic   [BusWidth-1:0]        device_wdata;
  logic   [BusWidth-1:0]        device_rdata;
  logic                         device_err;

  tlul_adapter_reg #(
    .CmdIntgCheck     (1),
    .EnableRspIntgGen (1),
    .EnableDataIntgGen(1),
    .RegAw            (MemAw),
    .RegDw            (BusWidth),
    .AccessLatency    (1)
  ) i_tlul_adapter_reg (
    .clk_i,
    .rst_ni,
    .tl_i        (mem_tl_d_i),
    .tl_o        (mem_tl_d_o),
    // SEC_CM: EXEC.CTRL.MUBI
    .en_ifetch_i (prim_mubi_pkg::MuBi4True),
    // SEC_CM: BUS.INTEGRITY
    .intg_error_o(),
    .re_o        (device_re),
    .we_o        (device_we),
    .addr_o      (device_addr),
    .wdata_o     (device_wdata),
    .be_o        (device_be),
    .busy_i      (1'b0),
    .rdata_i     (device_rdata),
    .error_i     (device_err)
  );

  assign device_req = device_we || device_re;

  ///////////////////////////
  // Debug Module Instance //
  ///////////////////////////

  dm_top #(
    .NrHarts        (NrHarts),
    .BusWidth       (BusWidth),
    .SelectableHarts(SelectableHarts),
    // The debug module provides a simplified ROM for systems that map the debug ROM to offset 0x0
    // on the system bus. In that case, only one scratch register has to be implemented in the core.
    // However, we require that the DM can be placed at arbitrary offsets in the system, which
    // requires the generalized debug ROM implementation and two scratch registers. We hence set
    // this parameter to a non-zero value (inside dm_mem, this just feeds into a comparison with 0).
    .DmBaseAddress  (1)
  ) u_dm_top (
    .clk_i,
    .rst_ni,
    .next_dm_addr_i,
    .testmode_i            (1'b0                ),
    .ndmreset_o            (ndmreset_req        ),
    .ndmreset_ack_i        (                    ), // TODO Not sure if this can be left unconnected, probably not need some tracking logic
    .dmactive_o,
    .debug_req_o           (debug_req_o          ),
    .unavailable_i,
    .hartinfo_i            (hartinfo            ),
    .slave_req_i           (device_req          ),
    .slave_we_i            (device_we           ),
    .slave_addr_i          (device_addr         ),
    .slave_be_i            (device_be           ),
    .slave_wdata_i         (device_wdata        ),
    .slave_rdata_o         (device_rdata        ),
    .slave_err_o           (device_err          ),
    .master_req_o          (host_req            ),
    .master_add_o          (host_add            ),
    .master_we_o           (host_we             ),
    .master_wdata_o        (host_wdata          ),
    .master_be_o           (host_be             ),
    .master_gnt_i          (host_gnt            ),
    .master_r_valid_i      (host_r_valid        ),
    .master_r_err_i        (host_r_err          ),
    .master_r_other_err_i  (host_r_other_err    ),
    .master_r_rdata_i      (host_r_rdata        ),
    .dmi_rst_ni            (dmi_rst_n           ),
    .dmi_req_valid_i       (dmi_req_valid       ),
    .dmi_req_ready_o       (dmi_req_ready       ),
    .dmi_req_i             (dmi_req             ),
    .dmi_resp_valid_o      (dmi_rsp_valid       ),
    .dmi_resp_ready_i      (dmi_rsp_ready       ),
    .dmi_resp_o            (dmi_rsp             )
  );

endmodule
