///////////////////////////////////////////
// datapath.sv
//
// Written: David_Harris@hmc.edu, Sarah.Harris@unlv.edu
// Created: 9 January 2021
// Modified:
//
// Purpose: Wally Integer Datapath
//
// Documentation: RISC-V System on Chip Design
//
// A component of the CORE-V-WALLY configurable RISC-V project.
// https://github.com/openhwfoundation/cvw
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

module datapath import cvw::*;  #(parameter cvw_t P) (
  input  logic              clk, reset,
  // Decode stage signals
  input  logic [2:0]        ImmSrcD,                 // Selects type of immediate extension
  input  logic [31:0]       InstrD,                  // Instruction in Decode stage
  input  logic [4:0]        Rs1D, Rs2D, Rs2E,        // Source registers
  input  logic [2:0]        ImmSrc2D,                // Slot 1 (superscalar): immediate format
  input  logic [31:0]       Instr2D,                 // Slot 1: instruction in Decode stage
  input  logic [4:0]        Rs1_2D, Rs2_2D,          // Slot 1: source registers
  input  logic [4:0]        Rs2_2E,                  // Slot 1: source register 2 in Execute (ALU bit-manipulation)
  input  logic [2:0]        Forward2AE, Forward2BE,  // Slot 1: forwarding selects
  input  logic              ALUSrcA2E, ALUSrcB2E,    // Slot 1: ALU operands
  input  logic              ALUResultSrc2E,          // Slot 1: selects ALU result or immediate
  input  logic [2:0]        ALUSelect2E,             // Slot 1: ALU mux select
  input  logic [2:0]        Funct3_2E,               // Slot 1: funct3
  input  logic [6:0]        Funct7_2E,               // Slot 1: funct7
  input  logic              W64_2E, UW64_2E,         // Slot 1: RV64 W-type / .uw-type
  input  logic              SubArith2E,              // Slot 1: subtract or arithmetic shift
  input  logic [3:0]        BSelect2E, ZBBSelect2E,  // Slot 1: bit-manipulation selects
  input  logic [2:0]        BALUControl2E,           // Slot 1: bit-manipulation ALU control
  input  logic              BMUActive2E,             // Slot 1: bit-manipulation instruction active
  input  logic [1:0]        CZero2E,                 // Slot 1: czero.* active
  input  logic              BranchSigned2E,          // Slot 1: branch comparison is signed
  output logic [1:0]        Flags2E,                 // Slot 1: comparison flags ({eq, lt})
  output logic [P.XLEN-1:0] IEUAdr2E,                // Slot 1: ALU sum, which is its branch target
  output logic [P.XLEN-1:0] IEUResult2M,             // Slot 1: result in Memory; a branch target for a slot 1 branch
  input  logic              RegWrite2W,              // Slot 1: commits a register write this cycle
  input  logic [4:0]        Rd2W,                    // Slot 1: destination register in Writeback
  input  logic              Mem2E,                   // Slot 1: holds the bundle's memory operation
  input  logic [2:0]        ResultSrc2W,             // Slot 1: selects source of its writeback value
  input  logic              Mdu2E,                   // Slot 1: holds the bundle's multiply
  output logic [P.XLEN-1:0] MDUSrcAE, MDUSrcBE,      // Operands to the MDU, from whichever slot holds its operation
  // Execute stage signals
  input  logic [P.XLEN-1:0] PCE,                     // PC in Execute stage
  input  logic [P.XLEN-1:0] PCLinkE,                 // PC + 4 (of instruction in Execute stage)
  input  logic [2:0]        Funct3E,                 // Funct3 field of instruction in Execute stage
  input  logic [6:0]        Funct7E,                 // Funct7 field of instruction in Execute stage
  input  logic              StallE, FlushE,          // Stall, flush Execute stage
  input  logic [2:0]        ForwardAE, ForwardBE,    // Forward ALU operands from later stages
  input  logic              W64E, UW64E,             // W64/.uw-type instruction
  input  logic              SubArithE,               // Subtraction or arithmetic shift
  input  logic              ALUSrcAE, ALUSrcBE,      // ALU operands
  input  logic              ALUResultSrcE,           // Selects result to pass on to Memory stage
  input  logic [2:0]        ALUSelectE,              // ALU mux select signal
  input  logic              JumpE,                   // Jump (jal or jalr) instruction
  input  logic              BranchSignedE,           // Branch comparison operands are signed (if it's a branch)
  input  logic [3:0]        BSelectE,                // BMU result select (binary encoded; see bitmanipalu)
  input  logic [3:0]        ZBBSelectE,              // ZBB mux select signal
  input  logic [2:0]        BALUControlE,            // ALU Control signals for B instructions in Execute Stage
  input  logic              BMUActiveE,              // Bit manipulation instruction being executed
  input  logic [1:0]        CZeroE,                  // {czero.nez, czero.eqz} instructions active
  output logic [1:0]        FlagsE,                  // Comparison flags ({eq, lt})
  output logic [P.XLEN-1:0] IEUAdrE,                 // Address computed by ALU: slot 0's branch/jump target
  output logic [P.XLEN-1:0] LSUAdrE,                 // Address computed by ALU: memory address, from either slot
  output logic [P.XLEN-1:0] ForwardedSrcAE, ForwardedSrcBE, // ALU sources before the mux chooses between them and PCE to put in srcA/B
  // Memory stage signals
  input  logic              StallM, FlushM,          // Stall, flush Memory stage
  input  logic              FWriteIntM, FCvtIntW,    // FPU writes integer register file, FPU converts float to int
  input  logic [P.XLEN-1:0] FIntResM,                // FPU integer result
  output logic [P.XLEN-1:0] SrcAM,                   // ALU's Source A in Memory stage to privilege unit for CSR writes
  output logic [P.XLEN-1:0] WriteDataM,              // Write data in Memory stage
  // Writeback stage signals
  input  logic              StallW, FlushW,          // Stall, flush Writeback stage
  input  logic              RegWriteW, IntDivW,      // Write register file, integer divide instruction
  input  logic              SquashSCW,               // Squash a store conditional when a conflict arose
  input  logic [2:0]        ResultSrcW,              // Select source of result to write back to register file
  input  logic [P.XLEN-1:0] FCvtIntResW,             // FPU convert fp to integer result
  input  logic [P.XLEN-1:0] ReadDataW,               // Read data from LSU
  input  logic [P.XLEN-1:0] CSRReadValW,             // CSR read result
  input  logic [P.XLEN-1:0] MDUResultW,              // MDU (Multiply/divide unit) result
  input  logic [P.XLEN-1:0] FIntDivResultW,          // FPU's integer divide result
  input  logic [4:0]        RdW                      // Destination register
);

  // Decode stage signals
  logic [P.XLEN-1:0] R1D, R2D;                       // Read data from Rs1 (RD1), Rs2 (RD2)
  logic [P.XLEN-1:0] ImmExtD;                        // Extended immediate in Decode stage
  logic [P.XLEN-1:0] R1_2D, R2_2D;                   // Slot 1: read data from Rs1_2D, Rs2_2D
  logic [P.XLEN-1:0] ImmExt2D;                       // Slot 1: extended immediate in Decode stage
  logic [P.XLEN-1:0] R1_2E, R2_2E, ImmExt2E;         // Slot 1: the same, in Execute
  logic [P.XLEN-1:0] ForwardedSrc2AE, ForwardedSrc2BE; // Slot 1: operands after forwarding
  logic [P.XLEN-1:0] SrcA2E, SrcB2E;                 // Slot 1: ALU inputs
  logic [P.XLEN-1:0] ALUResult2E;                    // Slot 1: ALU output.  The sum, IEUAdr2E, needs no bit-0
                                                     //   mask as slot 0's does, because slot 1 cannot jump.
  logic [P.XLEN-1:0] AltResult2E, IEUResult2E;       // Slot 1: result in Execute
  logic [P.XLEN-1:0] IEUResult2W;                    // Slot 1: result in Writeback
  logic [P.XLEN-1:0] ResultW2;                       // Slot 1: value written back
  // Execute stage signals
  logic [P.XLEN-1:0] R1E, R2E;                       // Source operands read from register file
  logic [P.XLEN-1:0] ImmExtE;                        // Extended immediate in Execute stage
  logic [P.XLEN-1:0] SrcAE, SrcBE;                   // ALU operands
  logic [P.XLEN-1:0] ALUResultE, AltResultE, IEUResultE; // ALU result, Alternative result (ImmExtE or PC+4), result of execution stage
  logic [P.XLEN-1:0] IEUAdrRawE;                     // ALU sum before clearing bit 0 of a jump target
  logic [P.XLEN-1:0] WriteDataE;                     // Store data of whichever slot holds the memory operation
  // Memory stage signals
  logic [P.XLEN-1:0] IEUResultM;                     // Result from execution stage
  logic [P.XLEN-1:0] IFResultM;                      // Result from either IEU or single-cycle FPU op writing an integer register
  // Writeback stage signals
  logic [P.XLEN-1:0] SCResultW;                      // Store Conditional result
  logic [P.XLEN-1:0] ResultW;                        // Result to write to register file
  logic [P.XLEN-1:0] IFResultW;                      // Result from either IEU or single-cycle FPU op writing an integer register
  logic [P.XLEN-1:0] IFCvtResultW;                   // Result from IEU, single-cycle FPU op, or 2-cycle FCVT float to int
  logic [P.XLEN-1:0] MulDivResultW;                  // Multiply always comes from MDU.  Divide could come from MDU or FPU (when using fdivsqrt for integer division)

  // Decode stage
  regfile #(P.XLEN, P.E_SUPPORTED) regf(clk, reset, RegWriteW, RegWrite2W, Rs1D, Rs2D, RdW,
    Rs1_2D, Rs2_2D, Rd2W, ResultW, ResultW2, R1D, R2D, R1_2D, R2_2D);
  extend #(P)        ext(.InstrD(InstrD[31:7]), .ImmSrcD, .ImmExtD);
  extend #(P)        ext2(.InstrD(Instr2D[31:7]), .ImmSrcD(ImmSrc2D), .ImmExtD(ImmExt2D));   // slot 1

  // Execute stage pipeline register and logic
  //
  // Operand capture.  The Execute stage can be held while Memory and Writeback keep moving: a divide
  // holds it for its whole iteration (DivBusyE, FDivBusyE) while older instructions drain and bubbles
  // follow them.  An operand that was being forwarded from Memory or Writeback when the stall began
  // then loses its source as the producer retires, and R1E/R2E still hold the register file value read
  // in Decode, before the producer wrote it.  Single-issue Wally never meets this, because the only
  // instruction in Execute during such a stall is the divide itself, which latches its operands when
  // it starts.  With two slots the other instruction of the bundle sits there too, so the registers
  // reload the forwarded operand every cycle of the stall, and hold the right value once forwarding
  // stops.  The two cases cannot overlap with a flush: StallE without StallM means StallECause, which
  // is already gated by ~FlushECause, so FlushE is low whenever CaptureE is high.
  logic              CaptureE;                       // Execute held while Memory advances: keep forwarded operands
  logic [P.XLEN-1:0] R1ENext, R2ENext, R1_2ENext, R2_2ENext;
  assign CaptureE = StallE & ~StallM;
  mux2  #(P.XLEN)  r1enextmux(R1D, ForwardedSrcAE, CaptureE, R1ENext);
  mux2  #(P.XLEN)  r2enextmux(R2D, ForwardedSrcBE, CaptureE, R2ENext);
  flopenrc #(P.XLEN) RD1EReg(clk, reset, FlushE, ~StallE | CaptureE, R1ENext, R1E);
  flopenrc #(P.XLEN) RD2EReg(clk, reset, FlushE, ~StallE | CaptureE, R2ENext, R2E);
  flopenrc #(P.XLEN) ImmExtEReg(clk, reset, FlushE, ~StallE, ImmExtD, ImmExtE);

  // Forwarding sources in the order each lane's select encodes them: no forward, own Writeback,
  // own Memory, other Writeback, other Memory.  The encoding is relative to the lane, so lane 2's
  // muxes below take the same four sources with the pairs swapped.  See controller.sv for priority.
  mux5  #(P.XLEN)  faemux(R1E, ResultW, IFResultM, ResultW2, IEUResult2M, ForwardAE, ForwardedSrcAE);
  mux5  #(P.XLEN)  fbemux(R2E, ResultW, IFResultM, ResultW2, IEUResult2M, ForwardBE, ForwardedSrcBE);
  comparator #(P.XLEN) comp(ForwardedSrcAE, ForwardedSrcBE, BranchSignedE, FlagsE);
  mux2  #(P.XLEN)  srcamux(ForwardedSrcAE, PCE, ALUSrcAE, SrcAE);
  mux2  #(P.XLEN)  srcbmux(ForwardedSrcBE, ImmExtE, ALUSrcBE, SrcBE);
  alu   #(P)       alu(SrcAE, SrcBE, W64E, UW64E, SubArithE, ALUSelectE, BSelectE, ZBBSelectE, Funct3E, Funct7E, Rs2E, BALUControlE, BMUActiveE, CZeroE, ALUResultE, IEUAdrRawE);
  // jalr sets the least significant bit of the target address to zero.  jal and branch targets are
  // always even, so masking on JumpE alone is sufficient and leaves load/store addresses untouched.
  assign IEUAdrE = {IEUAdrRawE[P.XLEN-1:1], IEUAdrRawE[0] & ~JumpE};
  mux2  #(P.XLEN)  altresultmux(ImmExtE, PCLinkE, JumpE, AltResultE);
  mux2  #(P.XLEN)  ieuresultmux(ALUResultE, AltResultE, ALUResultSrcE, IEUResultE);

  // Slot 1 (superscalar) Execute stage, mirroring slot 0 above.  A conditional branch in slot 1
  // compares with its own comparator, and its target is its ALU's sum of slot 1's PC and immediate.
  mux2  #(P.XLEN)  r1_2enextmux(R1_2D, ForwardedSrc2AE, CaptureE, R1_2ENext);   // operand capture, as for slot 0
  mux2  #(P.XLEN)  r2_2enextmux(R2_2D, ForwardedSrc2BE, CaptureE, R2_2ENext);
  flopenrc #(P.XLEN) RD1_2EReg(clk, reset, FlushE, ~StallE | CaptureE, R1_2ENext, R1_2E);
  flopenrc #(P.XLEN) RD2_2EReg(clk, reset, FlushE, ~StallE | CaptureE, R2_2ENext, R2_2E);
  flopenrc #(P.XLEN) ImmExt2EReg(clk, reset, FlushE, ~StallE, ImmExt2D, ImmExt2E);

  mux5  #(P.XLEN)  faemux2(R1_2E, ResultW2, IEUResult2M, ResultW, IFResultM, Forward2AE, ForwardedSrc2AE);
  mux5  #(P.XLEN)  fbemux2(R2_2E, ResultW2, IEUResult2M, ResultW, IFResultM, Forward2BE, ForwardedSrc2BE);
  comparator #(P.XLEN) comp2(ForwardedSrc2AE, ForwardedSrc2BE, BranchSigned2E, Flags2E);
  // Slot 1 sits one instruction after slot 0, so its PC is PCLinkE (= PCE + 4, since the issue
  // rules require an uncompressed slot 0).  Using PCE here would make every paired auipc off by 4.
  mux2  #(P.XLEN)  srcamux2(ForwardedSrc2AE, PCLinkE, ALUSrcA2E, SrcA2E);
  mux2  #(P.XLEN)  srcbmux2(ForwardedSrc2BE, ImmExt2E, ALUSrcB2E, SrcB2E);
  alu   #(P)       alu2(SrcA2E, SrcB2E, W64_2E, UW64_2E, SubArith2E, ALUSelect2E, BSelect2E, ZBBSelect2E,
                        Funct3_2E, Funct7_2E, Rs2_2E, BALUControl2E, BMUActive2E, CZero2E, ALUResult2E, IEUAdr2E);
  // Slot 0's altresultmux also selects PCLinkE as a jump's link value.  Slot 1 may branch but cannot
  // jump under the issue rules, and its link value would be PCLinkE + 4 rather than PCLinkE, so that input is left
  // out rather than wired to a value that is wrong but currently unreachable.
  assign AltResult2E = ImmExt2E;                     // lui writes its immediate straight through
  mux2  #(P.XLEN)  ieuresultmux2(ALUResult2E, AltResult2E, ALUResultSrc2E, IEUResult2E);

  // The address handed to the load/store unit, selected from whichever slot holds the bundle's one
  // memory operation.  Each slot's ALU already computes its own sum, so this is only a choice of which
  // one the LSU listens to.
  //
  // It is deliberately a separate output from IEUAdrE rather than a mux on it.  A single instruction
  // is never both a control transfer and a memory access -- srcamux steers its one adder to PC+imm or
  // to rs1+imm -- which is why slot 0 can send one net to both the IFU and the LSU.  That stops being
  // true across slots: a taken branch in slot 0 needs its target at the IFU in the very cycle slot 1
  // needs its address at the LSU.  Keeping the nets apart lets both happen, and leaves the IFU's
  // redirect path exactly as it was.
  mux2  #(P.XLEN)  lsuadrmux(IEUAdrE, IEUAdr2E, Mem2E, LSUAdrE);

  // The multiplier's operands, from whichever slot holds the bundle's one MDU operation.  These are
  // the forwarded register values, as slot 0's ForwardedSrcAE/BE always were.  They are separate
  // outputs rather than a mux on ForwardedSrcAE/BE because the FPU reads those too, for slot 0.
  mux2  #(P.XLEN)  mdusrcamux(ForwardedSrcAE, ForwardedSrc2AE, Mdu2E, MDUSrcAE);
  mux2  #(P.XLEN)  mdusrcbmux(ForwardedSrcBE, ForwardedSrc2BE, Mdu2E, MDUSrcBE);

  // Slot 1 Memory and Writeback.  Slot 0 needs a mux5 in Writeback to choose between the ALU, a load,
  // a CSR read, the multiply/divide unit and a store-conditional.  Slot 1 can only be an ALU operation,
  // a load or a multiply/divide, so it needs just three of those, selected by the same ResultSrc
  // encoding slot 0 uses.  A slot 1 MDU result is always the MDU's own: the FPU only ever divides, and
  // issue.sv admits slot 1 divides only in configurations whose divider is the MDU's.
  flopenrc #(P.XLEN) IEUResult2MReg(clk, reset, FlushM, ~StallM, IEUResult2E, IEUResult2M);
  flopenrc #(P.XLEN) IEUResult2WReg(clk, reset, FlushW, ~StallW, IEUResult2M, IEUResult2W);
  always_comb
    case (ResultSrc2W)
      3'b001:  ResultW2 = ReadDataW;
      3'b011:  ResultW2 = MDUResultW;
      default: ResultW2 = IEUResult2W;
    endcase

  // Memory stage pipeline register
  flopenrc #(P.XLEN) SrcAMReg(clk, reset, FlushM, ~StallM, SrcAE, SrcAM);
  flopenrc #(P.XLEN) IEUResultMReg(clk, reset, FlushM, ~StallM, IEUResultE, IEUResultM);
  // A store's data is rs2 after forwarding, so it comes from whichever slot's forwarding muxes hold
  // the memory operation.  The lane choice is made before the register, so Memory sees one value.
  mux2  #(P.XLEN)  writedatamux(ForwardedSrcBE, ForwardedSrc2BE, Mem2E, WriteDataE);
  flopenrc #(P.XLEN) WriteDataMReg(clk, reset, FlushM, ~StallM, WriteDataE, WriteDataM);

  // Writeback stage pipeline register and logic
  flopenrc #(P.XLEN) IFResultWReg(clk, reset, FlushW, ~StallW, IFResultM, IFResultW);

  // floating point inputs: FIntResM comes from fclass, fcmp, fmv; FCvtIntResW comes from fcvt
  if (P.F_SUPPORTED) begin : fpmux
    mux2  #(P.XLEN)  resultmuxM(IEUResultM, FIntResM, FWriteIntM, IFResultM);
    mux2  #(P.XLEN)  cvtresultmuxW(IFResultW, FCvtIntResW, FCvtIntW, IFCvtResultW);
    if (P.IDIV_ON_FPU & P.F_SUPPORTED) begin
      mux2  #(P.XLEN)  divresultmuxW(MDUResultW, FIntDivResultW, IntDivW, MulDivResultW);
    end else begin
      assign MulDivResultW = MDUResultW;
    end
  end else begin : fpmux
    assign IFResultM = IEUResultM;
    assign IFCvtResultW = IFResultW;
    assign MulDivResultW = MDUResultW;
  end
  mux5  #(P.XLEN) resultmuxW(IFCvtResultW, ReadDataW, CSRReadValW, MulDivResultW, SCResultW, ResultSrcW, ResultW);

  // handle Store Conditional result if atomic extension supported
  if (P.ZALRSC_SUPPORTED) assign SCResultW = {{(P.XLEN-1){1'b0}}, SquashSCW};
  else                    assign SCResultW = '0;
endmodule
