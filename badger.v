/*
 * Copyright 2020, Lawrence Berkeley National Laboratory
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS
 * AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Minimal wrapper around Bedrock/Badger Ethernet in fabric
 * Assumes RGMII hardware, and no access to Badger UDP port plugins.
 * Only interface used is the host MAC.
 */
module badger (
    input              sysClk,
    input       [31:0] sysGPIO_OUT,
    input              sysConfigStrobe,
    input              sysTxStrobe,
    input              sysRxStrobe,
    input              sysRxDataStrobe,
    output wire [31:0] sysTxStatus,
    output wire [31:0] sysRxStatus,
    output wire [31:0] sysRxData,

    // Two phases of 125 MHz clock, created by on-board reference
    input            refClk125,
    input            refClk125d90,

    // Diagnostic outputs (e.g. to frequency counters)
    output wire      rx_clk,
    output wire      tx_clk,

    // RGMII pins
    input            RGMII_RX_CLK,
    input            RGMII_RX_CTRL,
    input      [3:0] RGMII_RXD,
    output wire      RGMII_TX_CLK,
    output wire      RGMII_TX_CTRL,
    output wire[3:0] RGMII_TXD);

assign tx_clk = refClk125;
wire tx_clk90 = refClk125d90;

///////////////////////////////////////////////////////////////////////////////
// Run-time configuration
localparam CONFIG_ADDR_WIDTH = 4;
localparam CONFIG_DATA_WIDTH = 8;
wire [CONFIG_DATA_WIDTH-1:0] sysConfigData = sysGPIO_OUT[0+:CONFIG_DATA_WIDTH];
wire [CONFIG_ADDR_WIDTH-1:0] sysConfigAddr = sysGPIO_OUT[CONFIG_DATA_WIDTH+:
                                                             CONFIG_ADDR_WIDTH];
reg sysEnableRx = 0;
always @(posedge sysClk) begin
    if (sysConfigStrobe && sysGPIO_OUT[31]) begin
        sysEnableRx <= sysGPIO_OUT[30];
    end
end
(*ASYNC_REG="true"*) reg enable_rx_m = 0;
reg enable_rx = 0;
always @(posedge rx_clk) begin
    enable_rx_m <= sysEnableRx;
    enable_rx   <= enable_rx_m;
end

///////////////////////////////////////////////////////////////////////////////
// Handle incoming frames and make available to processor
wire [7:0] rx_mac_status_d;
wire       rx_mac_status_s;
reg rx_isARP;
reg rx_mac_accept;
always @(posedge rx_clk) begin
    // Is the processor possibly interested in this packet?
    if (rx_mac_status_s)  begin
        rx_mac_accept <= rx_mac_status_d[2]     // Require valid CRC
                    &&   rx_mac_status_d[3]     // Require directed packet
                             // Accept stuff to which the firmware won't respond
                    && (rx_mac_status_d[1:0] == 2'd0);
        rx_isARP = rx_mac_status_d[1:0] == 2'd1;
    end
end

// Show that the processor has consumed the packet
reg sysHbank = 0;
always @(posedge sysClk) begin
    if (sysRxStrobe && sysGPIO_OUT[1]) begin
        sysHbank <= !sysHbank;
    end
end
wire [1:0] rx_mac_buf_status;
(*ASYNC_REG="true"*) reg rx_mac_hbank_m = 0;
reg rx_mac_hbank = 0;
always @(posedge rx_clk) begin
    rx_mac_hbank_m <= sysHbank;
    rx_mac_hbank   <= rx_mac_hbank_m;
end
assign sysRxStatus = { {30{1'b0}}, rx_mac_buf_status };

// Dual port packet buffer
localparam RX_MAC_ADDR_WIDTH = 12;
localparam RX_MAC_DATA_WIDTH = 8;
localparam RX_WORD_ADDR_WIDTH = RX_MAC_ADDR_WIDTH - 2;
reg [7:0] rxByteBuf00 [0:(1<<RX_WORD_ADDR_WIDTH)-1], rxByteBuf00Q;
reg [7:0] rxByteBuf01 [0:(1<<RX_WORD_ADDR_WIDTH)-1], rxByteBuf01Q;
reg [7:0] rxByteBuf10 [0:(1<<RX_WORD_ADDR_WIDTH)-1], rxByteBuf10Q;
reg [7:0] rxByteBuf11 [0:(1<<RX_WORD_ADDR_WIDTH)-1], rxByteBuf11Q;
wire [RX_MAC_ADDR_WIDTH-1:0] rx_mac_a;
wire [1:0] rx_mac_bsel = rx_mac_a[1:0];
wire [RX_WORD_ADDR_WIDTH-1:0] rx_mac_waddr = rx_mac_a[2+:RX_WORD_ADDR_WIDTH];
wire [RX_MAC_DATA_WIDTH-1:0] rx_mac_d;
wire                         rx_mac_wen;
always @(posedge rx_clk) begin
    if (rx_mac_wen) begin
        if (rx_mac_bsel == 2'b00) rxByteBuf00[rx_mac_waddr] <= rx_mac_d;
        if (rx_mac_bsel == 2'b01) rxByteBuf01[rx_mac_waddr] <= rx_mac_d;
        if (rx_mac_bsel == 2'b10) rxByteBuf10[rx_mac_waddr] <= rx_mac_d;
        if (rx_mac_bsel == 2'b11) rxByteBuf11[rx_mac_waddr] <= rx_mac_d;
    end
end

localparam SYS_RX_INDEX_WIDTH = RX_WORD_ADDR_WIDTH - 1;
reg [SYS_RX_INDEX_WIDTH-1:0] sysRxIndex;
wire [RX_WORD_ADDR_WIDTH-1:0] sysRxAddress = { sysHbank, sysRxIndex };
always @(posedge sysClk) begin
    if (sysRxDataStrobe) begin
        sysRxIndex <= sysGPIO_OUT[0+:SYS_RX_INDEX_WIDTH];
    end
    rxByteBuf00Q <= rxByteBuf00[sysRxAddress];
    rxByteBuf01Q <= rxByteBuf01[sysRxAddress];
    rxByteBuf10Q <= rxByteBuf10[sysRxAddress];
    rxByteBuf11Q <= rxByteBuf11[sysRxAddress];
end
assign sysRxData = { rxByteBuf11Q, rxByteBuf10Q, rxByteBuf01Q, rxByteBuf00Q };

///////////////////////////////////////////////////////////////////////////////
// Processor generation of outgoing frames
localparam PK_TXBUF_ADDR_WIDTH = 10;
localparam PK_TXBUF_DATA_WIDTH = 16;
wire [PK_TXBUF_DATA_WIDTH-1:0] sysTxData = sysGPIO_OUT[0+:PK_TXBUF_DATA_WIDTH];
wire [PK_TXBUF_ADDR_WIDTH-1:0] sysTxAddress = sysGPIO_OUT[PK_TXBUF_DATA_WIDTH+:
                                                           PK_TXBUF_ADDR_WIDTH];
reg [PK_TXBUF_DATA_WIDTH-1:0] txBuf [0:(1<<PK_TXBUF_ADDR_WIDTH)-1];
reg [PK_TXBUF_DATA_WIDTH-1:0] txBufQ;
wire [PK_TXBUF_ADDR_WIDTH-1:0] txBufBadgerAddress;
reg sysTxToggle = 0;
(*ASYNC_REG="true"*) reg txToggle_m;
reg  txToggle, txToggle_d;
reg  tx_mac_start;
wire tx_mac_done;
always @(posedge sysClk) begin
    if (sysTxStrobe) begin
        if (sysGPIO_OUT[31]) begin
            sysTxToggle <= !sysTxToggle;
        end
        else begin
            txBuf[sysTxAddress] <= sysTxData;
        end
    end
end
assign sysTxStatus = {tx_mac_start, sysTxToggle, txToggle_d, {29{1'b0}}};
always @(posedge tx_clk) begin
    txToggle_m <= sysTxToggle;
    txToggle   <= txToggle_m;
    txToggle_d <= txToggle;
    if (txToggle != txToggle_d) begin
        tx_mac_start <= 1;
    end
    else if (tx_mac_done) begin
        tx_mac_start <= 0;
    end
    txBufQ <= txBuf[txBufBadgerAddress];
end

// Double-data-rate conversion
wire [7:0] vgmii_txd, vgmii_rxd;
wire vgmii_tx_en, vgmii_tx_er, vgmii_rx_dv, vgmii_rx_er;
gmii_to_rgmii #(.in_phase_tx_clk(1'b1)) gmii_to_rgmii_i(
    .rgmii_txd(RGMII_TXD),
    .rgmii_tx_ctl(RGMII_TX_CTRL),
    .rgmii_tx_clk(RGMII_TX_CLK),
    .rgmii_rxd(RGMII_RXD),
    .rgmii_rx_ctl(RGMII_RX_CTRL),
    .rgmii_rx_clk(RGMII_RX_CLK),

    .gmii_tx_clk(tx_clk),
    .gmii_tx_clk90(tx_clk90),
    .gmii_txd(vgmii_txd),
    .gmii_tx_en(vgmii_tx_en),
    .gmii_tx_er(vgmii_tx_er),
    .gmii_rxd(vgmii_rxd),
    .gmii_rx_clk(rx_clk),
    .gmii_rx_dv(vgmii_rx_dv),
    .gmii_rx_er(vgmii_rx_er)
);

// Machine-generated Ethernet support
wire rx_mon, tx_mon, blob_in_use;
rtefi_blob #(.mac_aw(PK_TXBUF_ADDR_WIDTH))
  rtefi(
    .rx_clk(rx_clk), .rxd(vgmii_rxd),
    .rx_dv(vgmii_rx_dv), .rx_er(vgmii_rx_er),
    .tx_clk(tx_clk) , .txd(vgmii_txd),
    .tx_en(vgmii_tx_en),  // no vgmii_tx_er

    .enable_rx(enable_rx),
    .config_clk(sysClk),
    .config_s(sysConfigStrobe && ~sysGPIO_OUT[31]),
    .config_p(1'b0),    // Stick with default UDP port
    .config_d(sysConfigData),
    .config_a(sysConfigAddr),

    .host_raddr(txBufBadgerAddress),
    .host_rdata(txBufQ),
    .buf_start_addr({PK_TXBUF_ADDR_WIDTH{1'b0}}),
    .tx_mac_start(tx_mac_start),
    .tx_mac_done(tx_mac_done),

    .rx_mac_status_d(rx_mac_status_d),
    .rx_mac_status_s(rx_mac_status_s),
    .rx_mac_accept(rx_mac_accept),
    .rx_mac_buf_status(rx_mac_buf_status),
    .rx_mac_hbank(rx_mac_hbank),
    .rx_mac_d(rx_mac_d),
    .rx_mac_a(rx_mac_a),
    .rx_mac_wen(rx_mac_wen)
`ifdef NOTDEF
    .ibadge_stb(ibadge_stb), .ibadge_data(ibadge_data),
    .obadge_stb(obadge_stb), .obadge_data(obadge_data),
    .xdomain_fault(xdomain_fault),
    .p2_nomangle(1'b0),
    .p3_addr(lb_addr), .p3_control_strobe(lb_control_strobe),
    .p3_control_rd(lb_control_rd), .p3_control_rd_valid(lb_control_rd_valid),
    .p3_data_out(lb_data_out), .p3_data_in(p3_lb_data_in),
    .p4_spi_clk(boot_clk), .p4_spi_cs(boot_cs),
    .p4_spi_mosi(boot_mosi), .p4_spi_miso(boot_miso),
    .p4_busy(boot_busy),
    .rx_mon(rx_mon), .tx_mon(tx_mon), .in_use(blob_in_use)
`endif
);

endmodule
