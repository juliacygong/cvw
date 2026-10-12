///////////////////////////////////////////
// instrfields.sv
//
// Written: October 2026
//
// Purpose: Report the register fields of one instruction, 16- or 32-bit, for the superscalar
//          dependency check in issue.sv.
//
//          issue.sv runs in Fetch and has only raw bytes, so it cannot use the decompressor's
//          output -- that is a stage later.  It therefore has to read the register fields itself,
//          and a compressed instruction keeps them in different places from a 32-bit one.
//
//          Recognized is the safety property.  A wrong Rd would let issue.sv permit a dependent
//          pair, and that produces wrong data silently: unlike an illegal instruction there is no
//          fault to replay on.  So every encoding whose fields are not decoded here reports
//          Recognized = 0, and issue.sv refuses to pair an unrecognized slot.  A gap therefore
//          costs pairing opportunities and never correctness, and new formats can be added one at a
//          time with the testbench cross-check confirming each one.
//
//          Fields that are not registers report x0, which cannot alias a real dependency.
//
// A component of the CORE-V-WALLY configurable RISC-V project.
// https://github.com/openhwgroup/cvw
//
// Copyright (C) 2021-23 Harvey Mudd College & Oklahoma State University
//
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
//
// Licensed under the Solderpad Hardware License v 2.1 (the “License”); you may not use this file
// except in compliance with the License, or, at your option, the Apache License version 2.0. You
// may obtain a copy of the License at
//
// https://solderpad.org/licenses/SHL-2.1/
//
// Unless required by applicable law or agreed to in writing, any work distributed under the
// License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
// either express or implied. See the License for the specific language governing permissions
// and limitations under the License.
////////////////////////////////////////////////////////////////////////////////////////////////

module instrfields import cvw::*;  #(parameter cvw_t P) (
  input  logic [31:0] Instr,        // Raw instruction; a compressed one occupies Instr[15:0]
  output logic [4:0]  Rd,           // Register written, or x0 if none
  output logic [4:0]  Rs1, Rs2,     // Registers read, or x0 if none
  output logic [1:0]  MemRW,        // Uses the load/store unit: [1] reads, [0] writes
  output logic        IsBranch,     // Conditional branch: may sit in slot 0, but not yet in slot 1
  output logic        Recognized    // The fields above are known to be correct for this encoding
);

  logic       CompressedI;
  logic [1:0] Quadrant;             // Instr[1:0]: 00, 01, 10 select the three compressed quadrants
  logic [2:0] Funct3;               // Instr[15:13]
  logic [4:0] CRs2;                 // Instr[6:2], the CR-format source

  assign CompressedI = ~&Instr[1:0];
  assign Quadrant    = Instr[1:0];
  assign Funct3      = Instr[15:13];
  assign CRs2        = Instr[6:2];

  always_comb begin
    // A 32-bit instruction keeps its fields in fixed positions.  They are reported whether or not
    // the format actually uses them -- for a store or a lui some are immediate bits -- which is what
    // issue.sv has always done.  Treating an immediate as a register number is conservative: it can
    // refuse a legal pair, never permit an illegal one.
    Rd  = Instr[11:7];
    Rs1 = Instr[19:15];
    Rs2 = Instr[24:20];
    Recognized = 1'b1;
    // Which opcodes occupy the load/store unit.  The floating-point and atomic forms are included
    // because they use it just as much, even though only LOAD and STORE may appear in slot 1.
    case (Instr[6:0])
      7'b0000011, 7'b0000111: MemRW = 2'b10;   // load, flw/fld
      7'b0100011, 7'b0100111: MemRW = 2'b01;   // store, fsw/fsd
      7'b0101111:             MemRW = 2'b11;   // lr, sc and the AMOs: both
      default:                MemRW = 2'b00;
    endcase
    IsBranch = (Instr[6:0] == 7'b1100011);

    if (CompressedI) begin
      // Default for compressed: nothing decoded, so refuse to pair.  Each arm below replaces this.
      Rd  = 5'b0;
      Rs1 = 5'b0;
      Rs2 = 5'b0;
      MemRW = 2'b00;
      IsBranch = 1'b0;
      Recognized = 1'b0;

      if (P.ZCA_SUPPORTED)
        case ({Quadrant, Funct3})
          // c.addi and c.nop: rd and rs1 are the same register, at the 32-bit rd position.
          // c.nop is the rd = x0 case and falls out of the same expression.
          5'b01_000: begin Rd = Instr[11:7]; Rs1 = Instr[11:7]; Recognized = 1'b1; end
          // c.li: writes rd from an immediate, reads nothing.
          5'b01_010: begin Rd = Instr[11:7];                    Recognized = 1'b1; end
          // c.slli: rd and rs1 are the same register.
          5'b10_000: begin Rd = Instr[11:7]; Rs1 = Instr[11:7]; Recognized = 1'b1; end
          // Quadrant 2 funct3 100 holds four instructions, separated by Instr[12] and whether the
          // CR source is x0.  c.jr and c.jalr are control transfers, so they stay unrecognized and
          // never pair; c.mv and c.add are ordinary ALU operations.
          5'b10_100:
            if (CRs2 != 5'b0) begin
              Rs2 = CRs2;
              Rd  = Instr[11:7];
              if (Instr[12]) Rs1 = Instr[11:7];   // c.add reads rd as well; c.mv does not
              Recognized = 1'b1;
            end
          // Loads and stores.  The 3-bit forms name x8-x15, so the register number is
          // {2'b01, field} -- not a zero-extend.  The format itself fixes the access width, so
          // unlike the 32-bit arms in issue.sv there is no funct3 to check: being recognized here
          // is the legality check.
          5'b00_010: begin                                        // c.lw
            Rd  = {2'b01, Instr[4:2]};
            Rs1 = {2'b01, Instr[9:7]};
            MemRW = 2'b10; Recognized = 1'b1;
          end
          5'b00_110: begin                                        // c.sw
            Rs1 = {2'b01, Instr[9:7]};
            Rs2 = {2'b01, Instr[4:2]};
            MemRW = 2'b01; Recognized = 1'b1;
          end
          // The stack-pointer forms read x2 without naming it anywhere in the encoding.  Reporting
          // that implicit source is essential: without it a dependency on sp is invisible and the
          // pair would issue with stale data.
          5'b10_010: begin                                        // c.lwsp
            Rd  = Instr[11:7];
            Rs1 = 5'd2;
            MemRW = 2'b10; Recognized = 1'b1;
          end
          5'b10_110: begin                                        // c.swsp
            Rs1 = 5'd2;
            Rs2 = Instr[6:2];
            MemRW = 2'b01; Recognized = 1'b1;
          end
          5'b00_000:                                              // c.addi4spn: ALU, but reads sp
            if (Instr[12:5] != 8'b0) begin   // all-zero immediate is the illegal encoding
              Rd  = {2'b01, Instr[4:2]};
              Rs1 = 5'd2;
              Recognized = 1'b1;
            end
          // RV64 doubleword forms.  On RV32 these same keys are c.flw/c.fsw/c.flwsp/c.fswsp, which
          // are floating point and must stay unrecognized -- slot 1 has no second FP write port.
          5'b00_011: if (P.XLEN == 64) begin                      // c.ld
            Rd  = {2'b01, Instr[4:2]};
            Rs1 = {2'b01, Instr[9:7]};
            MemRW = 2'b10; Recognized = 1'b1;
          end
          5'b00_111: if (P.XLEN == 64) begin                      // c.sd
            Rs1 = {2'b01, Instr[9:7]};
            Rs2 = {2'b01, Instr[4:2]};
            MemRW = 2'b01; Recognized = 1'b1;
          end
          5'b10_011: if (P.XLEN == 64) begin                      // c.ldsp
            Rd  = Instr[11:7];
            Rs1 = 5'd2;
            MemRW = 2'b10; Recognized = 1'b1;
          end
          5'b10_111: if (P.XLEN == 64) begin                      // c.sdsp
            Rs1 = 5'd2;
            Rs2 = Instr[6:2];
            MemRW = 2'b01; Recognized = 1'b1;
          end
          // Quadrant 1 funct3 100 holds the register-register and shift-immediate ALU forms, all
          // using the 3-bit x8-x15 encoding and separated by Instr[11:10].  The reserved
          // combinations are left unrecognized rather than trusted: slot 1's decompressor flags an
          // illegal compressed instruction through IllegalCompInstr2D, which nothing reads, so an
          // illegal encoding admitted here would execute instead of trapping.
          5'b01_100: begin
            Rd  = {2'b01, Instr[9:7]};
            Rs1 = {2'b01, Instr[9:7]};
            case (Instr[11:10])
              2'b00, 2'b01: Recognized = (P.XLEN == 64) | ~Instr[12];  // c.srli, c.srai -- on RV32 shamt[5] must be 0
              2'b10:        Recognized = 1'b1;                         // c.andi
              2'b11: begin
                Rs2 = {2'b01, Instr[4:2]};
                if (~Instr[12]) Recognized = 1'b1;                     // c.sub, c.xor, c.or, c.and
                // c.subw and c.addw are RV64 only, and Instr[6:5] of 10 or 11 is reserved.
                else            Recognized = (P.XLEN == 64) & ~Instr[6];
              end
            endcase
          end
          // Conditional branches.  These are recognized so that one in SLOT 0 can carry a partner,
          // exactly as a 32-bit beq already does -- it writes nothing, and if it resolves taken
          // Kill2E squashes slot 1.  issue.sv still keeps them out of slot 1, which needs a second
          // comparator and a prediction for slot 1's PC.
          5'b01_110, 5'b01_111: begin                             // c.beqz, c.bnez
            Rs1 = {2'b01, Instr[9:7]};
            IsBranch = 1'b1; Recognized = 1'b1;
          end
          // c.lui and c.addi16sp share a key and are told apart by rd.  Both are ALU operations, so
          // either slot may hold them.  rd = x0 is a HINT and a zero immediate is reserved, so
          // neither is claimed here.
          5'b01_011:
            if ((Instr[11:7] != 5'b0) & ({Instr[12], Instr[6:2]} != 6'b0)) begin
              Rd = Instr[11:7];
              if (Instr[11:7] == 5'd2) Rs1 = 5'd2;                // c.addi16sp reads sp
              Recognized = 1'b1;                                  // otherwise c.lui, no source
            end
          // Compressed jumps are deliberately left unrecognized.  c.j, c.jal, c.jr and c.ret must
          // block pairing in either slot -- the instruction after a jump is not the next one
          // executed -- and leaving them out of the table achieves exactly that.
          default: ;   // every other compressed encoding stays unrecognized
        endcase
    end
  end

endmodule
