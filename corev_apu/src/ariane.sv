// Copyright 2017-2019 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Author: Florian Zaruba, ETH Zurich
// Date: 19.03.2017
// Description: Ariane Top-level module


module ariane import ariane_pkg::*; import ara_pkg::*;#(
  // RVV Parameters
  parameter int unsigned NrLanes                = 2,                               // Number of parallel vector lanes.
  // Support for floating-point data types
  parameter fpu_support_e FPUSupport            = FPUSupportHalfSingleDouble,
  // External support for vfrec7, vfrsqrt7
  parameter fpext_support_e FPExtSupport        = FPExtSupportEnable,
  // Support for fixed-point data types
  parameter fixpt_support_e FixPtSupport        = FixedPointEnable,
  parameter ariane_pkg::ariane_cfg_t ArianeCfg  = ariane_pkg::ArianeDefaultConfig,
  parameter int unsigned AxiAddrWidth           = ariane_axi::AddrWidth,
  parameter int unsigned AxiDataWidth           = ariane_axi::DataWidth,
  parameter int unsigned AxiIdWidth             = ariane_axi::IdWidth,
  parameter int unsigned AxiWideDataWidth       = 64*NrLanes/2,
  parameter type ariane_axi_ar_chan_t           = ariane_axi::ar_chan_t,
  parameter type ariane_axi_r_chan_t           = ariane_axi::r_chan_t,
  parameter type ariane_axi_aw_chan_t           = ariane_axi::aw_chan_t,
  parameter type ariane_axi_w_chan_t            = ariane_axi::w_chan_t,
  parameter type ariane_axi_b_chan_t            = ariane_axi::b_chan_t,
  parameter type ariane_axi_req_t               = ariane_axi::req_t,
  parameter type ariane_axi_rsp_t               = ariane_axi::resp_t,

  parameter type ara_axi_ar_chan_t              = ariane_axi::ar_chan_t,
  parameter type ara_axi_r_chan_t              = ariane_axi::r_chan_t,
  parameter type ara_axi_aw_chan_t              = ariane_axi::aw_chan_t,
  parameter type ara_axi_w_chan_t               = ariane_axi::w_chan_t,
  parameter type ara_axi_b_chan_t               = ariane_axi::b_chan_t,
  parameter type ara_axi_req_t                  = ariane_axi::req_t,
  parameter type ara_axi_rsp_t                  = ariane_axi::resp_t,

  parameter type axi_ar_chan_t                  = ariane_axi::ar_chan_t,
  parameter type axi_r_chan_t                  = ariane_axi::r_chan_t,
  parameter type axi_aw_chan_t                  = ariane_axi::aw_chan_t,
  parameter type axi_w_chan_t                   = ariane_axi::w_chan_t,
  parameter type axi_b_chan_t                   = ariane_axi::b_chan_t,
  parameter type axi_req_t                      = ariane_axi::req_t,
  parameter type axi_rsp_t                      = ariane_axi::resp_t
) (
  input  logic                         clk_i,
  input  logic                         rst_ni,
  // Core ID, Cluster ID and boot address are considered more or less static
  input  logic [riscv::VLEN-1:0]       boot_addr_i,  // reset boot address
  input  logic [riscv::XLEN-1:0]       hart_id_i,    // hart id in a multicore environment (reflected in a CSR)

  // Interrupt inputs
  input  logic [1:0]                   irq_i,        // level sensitive IR lines, mip & sip (async)
  input  logic                         ipi_i,        // inter-processor interrupts (async)
  // Timer facilities
  input  logic                         time_irq_i,   // timer interrupt in (async)
  input  logic                         debug_req_i,  // debug request (async)
`ifdef RVFI_PORT
  // RISC-V formal interface port (`rvfi`):
  // Can be left open when formal tracing is not needed.
  output rvfi_port_t                   rvfi_o,
`endif
`ifdef PITON_ARIANE
  // L15 (memory side)
  output wt_cache_pkg::l15_req_t       l15_req_o,
  input  wt_cache_pkg::l15_rtrn_t      l15_rtrn_i
`else
  // memory side, AXI Master
  output axi_req_t                     axi_req_o,
  input  axi_rsp_t                     axi_resp_i
`endif
);


  ///////////
  //  AXI  //
  ///////////
  localparam NrAXIMasters = 1;
  localparam AxiSocIdWidth  = AxiIdWidth - $clog2(NrAXIMasters);
  localparam AxiCoreIdWidth = AxiSocIdWidth - 1;

  ariane_axi_req_t  ariane_narrow_axi_req;
  ariane_axi_rsp_t ariane_narrow_axi_resp;
  ara_axi_req_t     ariane_axi_req, ara_axi_req_inval, ara_axi_req;
  ara_axi_rsp_t    ariane_axi_resp, ara_axi_resp_inval, ara_axi_resp;

  //////////////////////
  //  Ara and Ariane  //
  //////////////////////

  import acc_pkg::accelerator_req_t;
  import acc_pkg::accelerator_resp_t;

  // Accelerator ports
  accelerator_req_t                     acc_req;
  accelerator_resp_t                    acc_resp;
  logic                                 acc_resp_valid;
  logic                                 acc_resp_ready;
  logic                                 acc_cons_en;
  logic              [AxiAddrWidth-1:0] inval_addr;
  logic                                 inval_valid;
  logic                                 inval_ready;

  // Support max 8 cores, for now
  logic [63:0] hart_id;
  assign hart_id = {'0, hart_id_i};

  // Pack invalidation interface into acc interface
  accelerator_resp_t                    acc_resp_pack;
  always_comb begin : pack_inval
    acc_resp_pack             = acc_resp;
    acc_resp_pack.inval_valid = inval_valid;
    acc_resp_pack.inval_addr  = inval_addr;
    inval_ready               = acc_req.inval_ready;
    acc_cons_en               = acc_req.acc_cons_en;
  end
  cva6 #(
    .ArianeCfg    ( ArianeCfg                 ),
    .cvxif_req_t  (acc_pkg::accelerator_req_t ),
    .cvxif_resp_t (acc_pkg::accelerator_resp_t),
    .AxiAddrWidth ( AxiAddrWidth              ),
    .AxiDataWidth ( AxiDataWidth              ),
    .AxiIdWidth   ( AxiIdWidth                ),
    .axi_ar_chan_t (ariane_axi_ar_chan_t      ),
    .axi_aw_chan_t (ariane_axi_aw_chan_t      ),
    .axi_w_chan_t (ariane_axi_w_chan_t        ),
    .axi_req_t    (ariane_axi_req_t           ),
    .axi_rsp_t    (ariane_axi_rsp_t           )
  ) i_cva6 (
    .clk_i                ( clk_i                     ),
    .rst_ni               ( rst_ni                    ),
    .boot_addr_i          ( boot_addr_i               ),
    .hart_id_i            ( hart_id                   ),
    .irq_i                ( irq_i                     ),
    .ipi_i                ( ipi_i                     ),
    .time_irq_i           ( time_irq_i                ),
    .debug_req_i          ( debug_req_i               ),
`ifdef RVFI_PORT
    .rvfi_o               ( rvfi_o                    ),
`else
    .rvfi_o               (                           ),
`endif
    .cvxif_req_o          ( acc_req                   ),
    .cvxif_resp_i         ( acc_resp_pack             ),
`ifdef PITON_ARIANE
    .l15_req_o            ( l15_req_o                 ),
    .l15_rtrn_i           ( l15_rtrn_i                ),
    .axi_req_o            (                           ),
    .axi_resp_i           ( '0                        )
`else
    .l15_req_o            (                           ),
    .l15_rtrn_i           ( '0                        ),
    .axi_req_o            ( ariane_narrow_axi_req     ),
    .axi_resp_i           ( ariane_narrow_axi_resp    )
`endif
  );

  axi_dw_converter #(
    .AxiSlvPortDataWidth(AxiDataWidth         ),
    .AxiMstPortDataWidth(AxiWideDataWidth     ),
    .AxiAddrWidth       (AxiAddrWidth         ),
    .AxiIdWidth         (AxiIdWidth           ),
    .AxiMaxReads        (2                    ),
    .ar_chan_t          (ariane_axi_ar_chan_t ),
    .mst_r_chan_t       (ara_axi_r_chan_t     ),
    .slv_r_chan_t       (ariane_axi_r_chan_t  ),
    .aw_chan_t          (ariane_axi_aw_chan_t ),
    .b_chan_t           (ariane_axi_b_chan_t  ),
    .mst_w_chan_t       (ara_axi_w_chan_t     ),
    .slv_w_chan_t       (ariane_axi_w_chan_t  ),
    .axi_mst_req_t      (ara_axi_req_t        ),
    .axi_mst_resp_t     (ara_axi_rsp_t        ),
    .axi_slv_req_t      (ariane_axi_req_t     ),
    .axi_slv_resp_t     (ariane_axi_rsp_t     )
  ) i_ariane_axi_dwc (
    .clk_i     (clk_i                 ),
    .rst_ni    (rst_ni                ),
    .slv_req_i (ariane_narrow_axi_req ),
    .slv_resp_o(ariane_narrow_axi_resp),
    .mst_req_o (ariane_axi_req        ),
    .mst_resp_i(ariane_axi_resp       )
  );

  axi_inval_filter #(
    .MaxTxns    (4                              ),
    .AddrWidth  (AxiAddrWidth                   ),
    .L1LineWidth(ariane_pkg::DCACHE_LINE_WIDTH/8),
    .aw_chan_t  (ara_axi_aw_chan_t              ),
    .req_t      (ara_axi_req_t                  ),
    .resp_t     (ara_axi_rsp_t                  )
  ) i_axi_inval_filter (
    .clk_i        (clk_i             ),
    .rst_ni       (rst_ni            ),
    .en_i         (acc_cons_en       ),
    .slv_req_i    (ara_axi_req       ),
    .slv_resp_o   (ara_axi_resp      ),
    .mst_req_o    (ara_axi_req_inval ),
    .mst_resp_i   (ara_axi_resp_inval),
    .inval_addr_o (inval_addr        ),
    .inval_valid_o(inval_valid       ),
    .inval_ready_i(inval_ready       )
  );

  ara #(
    .NrLanes     (NrLanes         ),
    .FPUSupport  (FPUSupport      ),
    .FPExtSupport(FPExtSupport    ),
    .FixPtSupport(FixPtSupport    ),
    .AxiDataWidth(AxiWideDataWidth),
    .AxiAddrWidth(AxiAddrWidth    ),
    .axi_ar_t    (ara_axi_ar_chan_t    ),
    .axi_r_t     (ara_axi_r_chan_t     ),
    .axi_aw_t    (ara_axi_aw_chan_t    ),
    .axi_w_t     (ara_axi_w_chan_t     ),
    .axi_b_t     (ara_axi_b_chan_t     ),
    .axi_req_t   (ara_axi_req_t        ),
    .axi_resp_t  (ara_axi_rsp_t        )
  ) i_ara (
    .clk_i           (clk_i         ),
    .rst_ni          (rst_ni        ),
    .scan_enable_i   (1'b0          ),
    .scan_data_i     (1'b0          ),
    .scan_data_o     (/* Unused */  ),
    .acc_req_i       (acc_req       ),
    .acc_resp_o      (acc_resp      ),
    .axi_req_o       (ara_axi_req   ),
    .axi_resp_i      (ara_axi_resp  )
  );

  axi_mux #(
    .SlvAxiIDWidth(AxiCoreIdWidth   ),
    .slv_ar_chan_t(ara_axi_ar_chan_t),
    .slv_aw_chan_t(ara_axi_aw_chan_t),
    .slv_b_chan_t (ara_axi_b_chan_t ),
    .slv_r_chan_t (ara_axi_r_chan_t ),
    .slv_req_t    (ara_axi_req_t    ),
    .slv_resp_t   (ara_axi_rsp_t    ),
    .mst_ar_chan_t(axi_ar_chan_t    ),
    .mst_aw_chan_t(axi_aw_chan_t    ),
    .w_chan_t     (axi_w_chan_t     ),
    .mst_b_chan_t (axi_b_chan_t     ),
    .mst_r_chan_t (axi_r_chan_t     ),
    .mst_req_t    (axi_req_t        ),
    .mst_resp_t   (axi_rsp_t        ),
    .NoSlvPorts   (2                ),
    .SpillAr      (1'b1             ),
    .SpillR       (1'b1             ),
    .SpillAw      (1'b1             ),
    .SpillW       (1'b1             ),
    .SpillB       (1'b1             )
  ) i_system_mux (
    .clk_i      (clk_i                                ),
    .rst_ni     (rst_ni                               ),
    .test_i     (1'b0                                 ),
    .slv_reqs_i ({ara_axi_req_inval, ariane_axi_req}  ),
    .slv_resps_o({ara_axi_resp_inval, ariane_axi_resp}),
    .mst_req_o  (axi_req_o                            ),
    .mst_resp_i (axi_resp_i                           )
  );

endmodule // ariane
