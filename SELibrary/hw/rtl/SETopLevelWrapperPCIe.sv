//
// Copyright (c) 2020, Intel Corporation
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
//
// Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// Neither the name of the Intel Corporation nor the names of its contributors
// may be used to endorse or promote products derived from this software
// without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

`include "ofs_plat_if.vh"
`include "afu_json_info.vh"

//
// CCI-P version of hello world AFU example.
//

module ofs_plat_afu
   (
    // All platform wires, wrapped in one interface.
    ofs_plat_if plat_ifc
    );

    // ====================================================================
    //
    //  Get a CCI-P port from the platform.
    //
    // ====================================================================

    // Instance of a CCI-P interface. The interface wraps usual CCI-P
    // sRx and sTx structs as well as the associated clock and reset.
    ofs_plat_host_ccip_if host_ccip();

    // Use the platform-provided module to map the primary host interface
    // to CCI-P. The "primary" interface is the port that includes the
    // main OPAE-managed MMIO connection. This primary port is always
    // index 0 of plat_ifc.host_chan.ports, indepedent of the platform
    // and the native protocol of the host channel.
    ofs_plat_host_chan_as_ccip
        #(
        .ADD_CLOCK_CROSSING(1)
        ) primary_ccip
       (
        .to_fiu(plat_ifc.host_chan.ports[0]),
        .to_afu(host_ccip),

        // These ports would be used if the PIM is told to cross to
        // a different clock. In this example, host_ccip is instantiated
        // with the native pClk.
        .afu_clk(plat_ifc.clocks.pClkDiv2.clk),
        .afu_reset_n(plat_ifc.clocks.pClkDiv2.reset_n)
        );


    // Each interface names its associated clock and reset.
    logic clk;
    assign clk =plat_ifc.clocks.pClkDiv2.clk;
    logic reset_n;
    assign reset_n =plat_ifc.clocks.pClkDiv2.reset_n;


    // ====================================================================
    //
    //  Tie off unused ports.
    //
    // ====================================================================

    // The PIM ties off unused devices, controlled by the AFU indicating
    // which devices it is using. This way, an AFU must know only about
    // the devices it uses. Tie-offs are thus portable, with the PIM
    // managing devices unused by and unknown to the AFU.
    ofs_plat_if_tie_off_unused
      #(
        // Host channel group 0 port 0 is connected. The mask is a
        // bit vector of indices used by the AFU.
        .HOST_CHAN_IN_USE_MASK(1)
        )
        tie_off(plat_ifc);


    // =========================================================================
    //
    //   CSR (MMIO) handling.
    //
    // =========================================================================

    // The AFU ID is a unique ID for a given program.  Here we generated
    // one with the "uuidgen" program and stored it in the AFU's JSON file.
    // ASE and synthesis setup scripts automatically invoke afu_json_mgr
    // to extract the UUID into afu_json_info.vh.
    logic [127:0] afu_id = `AFU_ACCEL_UUID;

    //
    // A valid AFU must implement a device feature list, starting at MMIO
    // address 0.  Every entry in the feature list begins with 5 64-bit
    // words: a device feature header, two AFU UUID words and two reserved
    // words.
    //

    // Is a CSR read request active this cycle?
    logic is_csr_read;
    assign is_csr_read = host_ccip.sRx.c0.mmioRdValid;

    // Is a CSR write request active this cycle?
    logic is_csr_write;
    assign is_csr_write = host_ccip.sRx.c0.mmioWrValid;

    // The MMIO request header is overlayed on the normal c0 memory read
    // response data structure.  Cast the c0Rx header to an MMIO request
    // header.
    t_ccip_c0_ReqMmioHdr mmio_req_hdr;
    assign mmio_req_hdr = t_ccip_c0_ReqMmioHdr'(host_ccip.sRx.c0.hdr);


    //
    // Implement the device feature list by responding to MMIO reads.
    //

    always_ff @(posedge clk)
    begin
        if (!reset_n)
        begin
            host_ccip.sTx.c2.mmioRdValid <= 1'b0;
        end
        else
        begin
            // Always respond with something for every read request
            host_ccip.sTx.c2.mmioRdValid <= is_csr_read;

            // The unique transaction ID matches responses to requests
            host_ccip.sTx.c2.hdr.tid <= mmio_req_hdr.tid;

            // Addresses are of 32-bit objects in MMIO space.  Addresses
            // of 64-bit objects are thus multiples of 2.
            case (mmio_req_hdr.address)
              0: // AFU DFH (device feature header)
                begin
                    // Here we define a trivial feature list.  In this
                    // example, our AFU is the only entry in this list.
                    host_ccip.sTx.c2.data <= t_ccip_mmioData'(0);
                    // Feature type is AFU
                    host_ccip.sTx.c2.data[63:60] <= 4'h1;
                    // End of list (last entry in list)
                    host_ccip.sTx.c2.data[40] <= 1'b1;
                end

              // AFU_ID_L
              2: host_ccip.sTx.c2.data <= afu_id[63:0];

              // AFU_ID_H
              4: host_ccip.sTx.c2.data <= afu_id[127:64];

              // DFH_RSVD0
              6: host_ccip.sTx.c2.data <= t_ccip_mmioData'(0);

              // DFH_RSVD1
              8: host_ccip.sTx.c2.data <= t_ccip_mmioData'(0);

              default: host_ccip.sTx.c2.data <= t_ccip_mmioData'(0);
            endcase
        end
    end


    //
    // CSR write handling.  Host software must tell the AFU the memory address
    // to which it should be writing.  The address is set by writing a CSR.
    //

    // We use MMIO address 0 to set the memory address.  The read and
    // write MMIO spaces are logically separate so we are free to use
    // whatever we like.  This may not be good practice for cleanly
    // organizing the MMIO address space, but it is legal.
    logic is_mem_csr_write;
    assign is_mem_csr_write = is_csr_write &&
                                   (mmio_req_hdr.address == t_ccip_mmioAddr'(0));
    logic rd_addr_valid;
    logic wr_addr_valid;
    logic data_rd_req_issued;
    // Memory address to which this AFU will write.
    t_ccip_clAddr rd_mem_addr;
    t_ccip_clAddr wr_mem_addr;


    always_ff @(posedge clk) 
    begin
        if(!reset_n) // reset
        begin
            rd_mem_addr <= 'b0;
            rd_addr_valid <= 'b0;
            data_rd_req_issued <= 'b0;
        end
        else if(is_mem_csr_write) // when we receive the information about the new req
        begin
            $display("writing rd_mem_addr: %x",host_ccip.sRx.c0.data[CCIP_CLADDR_WIDTH-1:0]);
            rd_mem_addr <= host_ccip.sRx.c0.data[CCIP_CLADDR_WIDTH-1:0];
            rd_addr_valid <= 'b1;
            data_rd_req_issued <= 'b0;
        end
        else if (!host_ccip.sRx.c0TxAlmFull) begin
            if(rd_addr_valid && (!data_rd_req_issued)) begin
                data_rd_req_issued <= 'b1;
            end
        end

    end

    t_ccip_c0_ReqMemHdr rd_hdr;
    always_comb begin
        rd_hdr.vc_sel = eVC_VA;
        rd_hdr.cl_len = eCL_LEN_1;
        rd_hdr.req_type = eREQ_RDLINE_I;
        rd_hdr.address = rd_mem_addr;
        rd_hdr.mdata = 0;
    end
    assign host_ccip.sTx.c0.hdr = rd_hdr;


    always_ff @( posedge clk ) 
    begin
        if(!reset_n) begin
            host_ccip.sTx.c0.valid = 1'b0;
        end
        else if (!host_ccip.sRx.c0TxAlmFull) begin
            if(rd_addr_valid && (!data_rd_req_issued)) begin
                host_ccip.sTx.c0.valid <= 'b1;
                $display("read request issued: %x", host_ccip.sTx.c0.hdr.address);
            end 
            else begin
                host_ccip.sTx.c0.valid <= 'b0;
            end
        end
    end
    // =========================================================================
    //
    //   Main AFU logic
    //
    // =========================================================================
    logic          se_ctrl_out_ready;
    // logic         se_ctrl_out_valid;
    // logic         se_ctrl_in_ready;
    logic          se_ctrl_in_valid;
    logic  [127:0] se_in_cond;
    logic  [7:0]   se_in_op2_is_a_byte;
    logic  [7:0] se_in_op2_encrypted;
    logic  [127:0] se_in_op2;
    logic  [7:0]    se_in_op1_is_a_byte;
    logic  [7:0]  se_in_op1_encrypted;
    logic  [127:0] se_in_op1;
    logic  [7:0]   se_in_inst;
    logic [127:0] se_out_result;

    t_ccip_clData input_data;
    always_ff @( posedge clk ) begin   
        if(!reset_n) begin
            wr_mem_addr <= 'b0;
        end
        else if(host_ccip.sRx.c0.rspValid) begin
            $display("write address: %x",input_data[423+CCIP_CLADDR_WIDTH:424]);
            wr_mem_addr <= input_data[465:424];
        end
    end

    assign se_ctrl_out_ready = !host_ccip.sRx.c1TxAlmFull;
    assign input_data = t_ccip_clData'(host_ccip.sRx.c0.data);
    assign {se_in_cond, se_in_op2_is_a_byte, se_in_op2_encrypted,se_in_op2, 
            se_in_op1_is_a_byte, se_in_op1_encrypted, se_in_op1,se_in_inst } = input_data[423:0];
    SE SEFU(clk, !reset_n, se_ctrl_out_ready, se_ctrl_out_valid, se_ctrl_in_ready,
        se_ctrl_in_valid, se_in_cond, se_in_op2_is_a_byte[0], se_in_op2_encrypted[0],
        se_in_op2, se_in_op1_is_a_byte[0], se_in_op1_encrypted[0], 
        se_in_op1, se_in_inst, se_out_result);
    assign se_ctrl_in_valid = host_ccip.sRx.c0.rspValid;
    always_comb begin
        if(se_ctrl_in_valid)begin
            $display("data read: %x", input_data);
            $display("se_in_cond: %x", se_in_cond);
            $display("se_in_inst: %x", se_in_inst);
            $display("se_in_op1: %x", se_in_op1);
            $display("se_in_op2: %x", se_in_op2);
            $display("se_in_op1_is_a_byte: %x", se_in_op1_is_a_byte);
            $display("se_in_op1_encrypted: %x",se_in_op1_encrypted);
            $display("se_in_op2_is_a_byte: %x", se_in_op2_is_a_byte);
            $display("se_in_op2_encrypted: %x",se_in_op2_encrypted);
        end
    end

    //
    // Write "Hello world!" to memory when in STATE_RUN.
    //

    // Construct a memory write request header.  For this AFU it is always
    // the same, since we write to only one address.
    t_ccip_c1_ReqMemHdr wr_hdr;
    always_comb
    begin
        // Zero works for most write request header fields in this example
        wr_hdr = t_ccip_c1_ReqMemHdr'(0);
        // Set the write address
        wr_hdr.address = wr_mem_addr;
        // Start of packet is always set for single beat writes
        wr_hdr.sop = 1'b1;
    end

    // Data to write to memory: little-endian ASCII encoding of "Hello world!"
    assign host_ccip.sTx.c1.data = t_ccip_clData'({1'b1,se_out_result});

    // Control logic for memory writes
    always_ff @(posedge clk)
    begin
        if (!reset_n)
        begin
            host_ccip.sTx.c1.valid <= 1'b0;
        end
        else
        begin
            // Request the write as long as the channel isn't full.
            host_ccip.sTx.c1.valid <= se_ctrl_out_valid && se_ctrl_out_ready;
            if(host_ccip.sTx.c1.valid) begin
                $display("wrote to mem %x",host_ccip.sTx.c1.data);
            end
        end

        host_ccip.sTx.c1.hdr <= wr_hdr;
    end


endmodule
