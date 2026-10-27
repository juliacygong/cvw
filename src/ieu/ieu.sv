///////////////////////////////////////////
// ieu.sv
//
// Written: David_Harris@hmc.edu 9 January 2021
// Modified:
//
// Purpose: Integer Execution Unit: datapath and controller
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

module ieu import cvw::*;  #(parameter cvw_t P) (
  input  logic              clk, reset,
  // Decode stage signals
  input  logic [31:0]       InstrD,                           // Instruction
  input  logic [31:0]       Instr2D,                          // Second instruction (superscalar slot 1), decoded but not issued
  input  logic              Issue2D,                          // Slot 1 is paired with slot 0 this cycle
  input  logic              Issue2M, Issue2W,                 // ... and still committing in Memory / Writeback
  input  logic              LateA2D, LateB2D,                 // Slot 1's rs1 / rs2 field matches slot 0's rd
  input  logic              LateOK2D,                         // Slot 1 is an operation the late ALU implements
  input  logic              PredTakenE,                       // Fetch went to slot 0's branch target (predicted taken)
  output logic              LateBrM,                          // Slot 0 is a branch whose direction is checked in Memory
  output logic              LateBrTakenM,                     // ... its real direction
  output logic              LateBrWrongM,                     // ... and fetch went the other way: redirect from Memory
  output logic              Mem2M,                            // The bundle's memory operation is slot 1's
  output logic              Illegal2M,                        // Slot 1 is an illegal instruction; replay it alone
  input  logic [1:0]        STATUS_FS,                        // is FPU enabled?
  input  logic [3:0]        ENVCFG_CBE,                       // Cache block operation enables
  input  logic              IllegalIEUFPUInstrD,              // Illegal instruction
  output logic              IllegalBaseInstrD,                // Illegal base integer instruction, or illegal RV32E access to upper 16 registers
  // Execute stage signals
  input  logic [P.XLEN-1:0] PCE,                              // PC
  input  logic [P.XLEN-1:0] PCLinkE,                          // PC + 4
  output logic              PCSrcE,                           // Select next PC (between PC+4 and IEUAdrE)
  input  logic              FWriteIntE, FCvtIntE,             // FPU writes to integer register file, FPU converts float to int
  output logic [P.XLEN-1:0] IEUAdrE,                          // Branch / jump target, from slot 0
  output logic [P.XLEN-1:0] LSUAdrE,                          // Memory address, from whichever slot holds one
  output logic              IntDivE, W64E,                    // Integer divide, RV64 W-type instruction
  output logic [2:0]        Funct3E,                          // Funct3 instruction field
  output logic [P.XLEN-1:0] ForwardedSrcAE, ForwardedSrcBE,   // ALU src inputs before the mux choosing between them and PCE to put in srcA/B
  output logic [4:0]        RdE,                              // Destination register
  output logic              MDUActiveE,                       // Mul/Div instruction being executed, in either slot
  output logic [P.XLEN-1:0] MDUSrcAE, MDUSrcBE,               // MDU operands, from whichever slot holds the bundle's multiply
  output logic [2:0]        MDUFunct3E, MDUFunct3M,           // MDU operation, from that slot
  output logic              MDUW64E,                          // MDU W-type, from that slot
  output logic              MDUIntDivE,                       // MDU integer divide, from that slot
  output logic [3:0]        CMOpM,                            // 1: cbo.inval; 2: cbo.clean; 4: cbo.flush; 8: cbo.zero
  output logic              IFUPrefetchE,                     // instruction prefetch
  output logic              LSUPrefetchM,                     // data prefetch
  // Memory stage signals
  input  logic              SquashSCW,                        // Squash store conditional, from LSU
  output logic [1:0]        MemRWE,                           // Read/write control goes to LSU
  output logic [1:0]        MemRWM,                           // Read/write control goes to LSU
  output logic [1:0]        AtomicM,                          // Atomic control goes to LSU
  output logic [P.XLEN-1:0] WriteDataM,                       // Write data to LSU
  output logic [2:0]        Funct3M,                          // Funct3 of slot 0, to the FPU
  output logic [2:0]        MemFunct3M,                       // Funct3 (size and signedness) to LSU, from either slot
  output logic [P.XLEN-1:0] SrcAM,                            // ALU SrcA to Privileged unit and FPU
  output logic [4:0]        RdM,                              // Destination register
  input  logic [P.XLEN-1:0] FIntResM,                         // Integer result from FPU (fmv, fclass, fcmp)
  output logic              InvalidateICacheM, FlushDCacheM,  // Invalidate I$, flush D$
  output logic              InstrValidD, InstrValidE, InstrValidM, // Instruction is valid
  output logic              BranchD, BranchE,
  output logic              JumpD, JumpE,
  // Writeback stage signals
  input  logic [P.XLEN-1:0] FIntDivResultW,                   // Integer divide result from FPU (fdivsqrt)
  input  logic [P.XLEN-1:0] CSRReadValW,                      // CSR read value,
  input  logic [P.XLEN-1:0] MDUResultW,                       // multiply/divide unit result
  input  logic [P.XLEN-1:0] FCvtIntResW,                      // FPU's float to int conversion result
  input  logic              FCvtIntW,                         // FPU converts float to int
  output logic [4:0]        RdW,                              // Destination register
  input  logic [P.XLEN-1:0] ReadDataW,                        // LSU's read data
  // Hazard unit signals
  input  logic              StallD, StallE, StallM, StallW,   // Stall signals from hazard unit
  input  logic              FlushD, FlushE, FlushM, FlushW,   // Flush signals
  output logic              StructuralStallD,                 // IEU detects structural hazard in Decode stage
  output logic              LoadStallD,                       // Structural stalls for load, sent to performance counters
  output logic              StoreStallD,                      // load after store hazard
  output logic              CSRReadM, CSRWriteM, PrivilegedM, // CSR read, CSR write, is privileged instruction
  output logic              CSRWriteFenceM                    // CSR write or fence instruction needs to flush subsequent instructions
);

  logic [2:0] ImmSrcD;                                        // Select type of immediate extension
  logic [1:0] FlagsE;                                         // Comparison flags ({eq, lt})
  logic       ALUSrcAE, ALUSrcBE;                             // ALU source operands
  logic [2:0] ResultSrcW;                                     // Selects result in Writeback stage
  logic       ALUResultSrcE;                                  // Selects ALU result to pass on to Memory stage
  logic [2:0] ALUSelectE;                                     // ALU select mux signal
  logic       FWriteIntM;                                     // FPU writing to integer register file
  logic       IntDivW;                                        // Integer divide instruction
  logic [3:0] BSelectE;                                       // BMU result select (binary encoded; see bitmanipalu)
  logic [3:0] ZBBSelectE;                                     // ZBB Result Select Signal in Execute Stage
  logic [2:0] BALUControlE;                                   // ALU Control signals for B instructions in Execute Stage
  logic       SubArithE;                                      // Subtraction or arithmetic shift
  logic       UW64E;                                          // .uw-type instruction

  logic [6:0] Funct7E;

  // Forwarding signals
  logic [4:0] Rs1D, Rs2D;
  logic [4:0] Rs2E;                                           // Source registers

  // Slot 1 (superscalar) Decode-stage outputs: the same signals the controller produces in D for slot 0.
  // Slot 1 is decoded alongside slot 0 but does not issue yet, so these are observed by the testbench only.
  logic [4:0] Rs1_2D, Rs2_2D;                                 // Slot 1 source registers
  logic [2:0] ImmSrc2D;                                       // Slot 1 immediate format
  logic       IllegalBaseInstr2D;                             // Slot 1 is an illegal base instruction
  logic       IllegalBaseInstr2MRaw;                          // ... carried to Memory, before the commit gate
  logic       Branch2D, Jump2D;                               // Slot 1 is a branch / jump
  logic       InstrValid2D;                                   // Slot 1 controller valid bit (not yet qualified by the IFU's Instr2ValidD)
  logic       StructuralStall0D;                              // Slot 0's own structural hazards
  logic [1:0] MemRW0D, MemRW0M;                               // Slot 0 memory read/write in Decode / Memory
  logic       StructuralStall2D;                              // Slot 1 depends on a result not ready to forward
  logic       LoadStall2D;                                    // A slot 1 load's data is not ready to forward yet
  logic       MDUStall2D;                                     // A slot 1 multiply's product is not ready to forward yet
  logic       BundleStoreStallD;                              // The bundle's load would collide with a store on the memory's one port
  logic       RegWriteM;                                      // Slot 0 writes a register in Memory
  logic       RegWrite2MRaw, RegWrite2WRaw;                   // What slot 1's controller decoded
  logic       RegWrite2M, RegWrite2W;                         // ... once it is known slot 1 still commits
  logic [4:0] Rd2M, Rd2W;                                     // Slot 1 destination register in Memory / Writeback

  // Slot 1 Execute-stage signals.  Slot 1 computes a result but does not commit it yet.
  logic [4:0] Rs1_2E, Rs2_2E;                                 // Slot 1 source registers in Execute
  logic [4:0] Rd2E;                                           // Slot 1 destination register in Execute
  logic [2:0] Forward2AE, Forward2BE;                         // Lane 2 forwarding selects (from controller c2)
  logic       ALUSrcA2E, ALUSrcB2E, ALUResultSrc2E;           // Slot 1 ALU operand and result selects
  logic [2:0] ALUSelect2E;                                    // Slot 1 ALU operation
  logic [2:0] Funct3_2E;                                      // Slot 1 funct3
  logic [6:0] Funct7_2E;                                      // Slot 1 funct7
  logic       W64_2E, UW64_2E, SubArith2E;                    // Slot 1 W-type, .uw-type, subtract/arithmetic-shift
  logic       Jump2E;                                         // Slot 1 is a jump (always 0 under the issue rules)
  logic [3:0] BSelect2E, ZBBSelect2E;                         // Slot 1 bit-manipulation selects
  logic [2:0] BALUControl2E;                                  // Slot 1 bit-manipulation ALU control
  logic       BMUActive2E;                                    // Slot 1 bit-manipulation instruction active
  logic [1:0] CZero2E;                                        // Slot 1 czero.* active

  // Slot 1 late ALU.  A slot 1 that reads slot 0's result executes one stage later, in Memory, where
  // slot 0's result is already registered in IEUResultM.  See issue.sv for which pairs qualify.
  logic       LateA2RawE, LateB2RawE;                         // issue.sv's field matches, in Execute
  logic [1:0] LateWA2D, LateWB2D;                             // Slot 1 operand produced too late by the bundle ahead
  logic [1:0] LateWA2RawE, LateWB2RawE;                       // ... in Execute
  logic [1:0] LateSelA2E, LateSelB2E;                         // Where the late ALU takes each operand from
  logic       Late2E;                                         // Slot 1 executes on the late ALU
  logic       SlowResult0E, Slow2E;                           // Slot 0 / slot 1 in Execute has a result not ready in Memory
  logic       CaptureE;                                       // Execute held while Memory advances

  // Late branch: a slot 0 conditional branch whose operand the bundle ahead produces too late to compare
  // in Execute.  Its direction is checked by the late ALU in Memory instead of stalling it in Decode.
  logic [1:0] LateWA0D, LateWB0D;                             // Slot 0 operand produced too late (as LateWA2D, for slot 0)
  logic       LateBrD, LateBrE;                               // Slot 0 is a late branch
  logic       AssumedTakenM;                                  // The direction it was given in Execute
  logic [2:0] LateFlagsM;                                     // {eq, lt, ltu} from the late ALU
  logic       StructuralStall0GD;                             // Slot 0's own stalls, less those a late branch avoids
  logic       LateStall2D;                                    // A late slot 1's result is not ready to forward yet

  // Slot 1 Memory/Writeback-stage signals.  The core has one load/store unit, so at most one slot of
  // a bundle may hold a memory operation; Mem2E/Mem2M say it is slot 1's and steer the LSU's inputs
  // to lane 2.  c2 pipelines its own control to M and W with the real StallM/FlushM, exactly as c does.
  logic [1:0] MemRW2D, MemRW2E, MemRW2M, MemRW2MRaw;          // Slot 1 memory read/write, before and after the commit gate
  logic       Mem2E;                                          // The bundle's memory operation is in slot 1, in Execute
  logic [2:0] Funct3_2M;                                      // Slot 1 access size and signedness
  logic [2:0] ResultSrc2W;                                    // Slot 1 writeback source select

  // Slot 1 multiply/divide.  The core has one multiply/divide unit, so at most one slot of a bundle may
  // hold an MDU operation; Mdu2E/Mdu2M say it is slot 1's and steer the MDU's inputs to lane 2.
  logic       MDUActive0E, MDUActive2E;                       // Slot 0 / slot 1 holds a multiply or divide
  logic       IntDiv2E;                                       // Slot 1 holds a divide
  logic       Mdu2E, Mdu2M;                                   // The bundle's MDU operation is slot 1's

  logic [2:0] ForwardAE, ForwardBE;                           // Select signals for forwarding multiplexers
  logic       RegWriteW;                                      // Register will be written in Writeback stage
  logic       BranchSignedE;                                  // Branch does signed comparison on operands
  logic       BMUActiveE;                                     // Bit manipulation instruction being executed
  logic [1:0] CZeroE;                                         // {czero.nez, czero.eqz} instructions active

  controller #(P) c(
    .clk, .reset, .StallD, .FlushD, .InstrD, .STATUS_FS, .ENVCFG_CBE, .ImmSrcD,
    .IllegalIEUFPUInstrD, .IllegalBaseInstrD,
    .StructuralStallD(StructuralStall0D), .LoadStallD, .StoreStallD, .Rs1D, .Rs2D, .Rs2E, .MemRWD(MemRW0D),
    .Rs1_2D, .Rs2_2D, .StructuralStall2D,                     // lane 2's hazard, computed here against the real RdE
    .SlowResultE(SlowResult0E), .LateBrE, .PredTakenE,
    .StallE, .FlushE, .FlagsE, .FWriteIntE,
    .PCSrcE, .ALUSrcAE, .ALUSrcBE, .ALUResultSrcE, .ALUSelectE,
    .Funct3E, .Funct7E, .IntDivE, .W64E, .UW64E, .SubArithE, .BranchD, .BranchE, .JumpD, .JumpE,
    .BranchSignedE, .BSelectE, .ZBBSelectE, .BALUControlE, .BMUActiveE, .CZeroE, .MDUActiveE(MDUActive0E),
    .FCvtIntE, .ForwardAE, .ForwardBE, .CMOpM, .IFUPrefetchE, .LSUPrefetchM,
    // MemRWM and Funct3M are the only two LSU inputs that can differ by lane, so they are taken to
    // local names here and muxed against slot 1's below.  The rest reach the LSU and the privileged
    // unit straight from slot 0, and which lane they came from never has to be asked: issue.sv's
    // whitelist admits only LUI/AUIPC/OP-IMM/OP (and the RV64 W forms) to slot 1, so a bundle whose
    // slot 1 is an atomic, a CSR access, a privileged instruction, cbo.* or fence.i cannot pair at
    // all, leaving slot 1's copy of those signals structurally zero.  MemRWE is simply a dead port.
    .StallM, .FlushM, .MemRWE, .MemRWM(MemRW0M), .CSRReadM, .CSRWriteM, .PrivilegedM, .AtomicM, .Funct3M,
    // Slot 0's illegal instruction already reaches the privileged unit through IllegalBaseInstrD, which
    // privpiperegs.sv pipelines, so the Memory-stage copy is only needed for slot 1's replay.
    .FlushDCacheM, .InstrValidM, .InstrValidE, .InstrValidD, .FWriteIntM, .IllegalBaseInstrM(),
    .RegWriteM, .RegWriteOtherM(RegWrite2M),                  // lane 1 forwards from lane 2
    .StallW, .FlushW, .RegWriteW, .IntDivW, .ResultSrcW, .CSRWriteFenceM, .InvalidateICacheM,
    .RegWriteOtherW(RegWrite2W),
    .RdW, .RdE, .RdM, .RdOtherM(Rd2M), .RdOtherW(Rd2W));

  // Slot 1 controller (superscalar).  A second copy of the controller decodes Instr2D in the Decode stage.
  // Only its Decode-stage outputs are used; slot 1 does not issue, so its Execute/Memory/Writeback
  // outputs are left unconnected and its Execute-stage feedback inputs are tied off.
  controller #(P, 0) c2(
    .clk, .reset, .StallD, .FlushD, .InstrD(Instr2D), .STATUS_FS, .ENVCFG_CBE, .ImmSrcD(ImmSrc2D),
    .IllegalIEUFPUInstrD(1'b0), .IllegalBaseInstrD(IllegalBaseInstr2D),
    // The load-use hazard cannot be computed here: it depends on the Execute-stage control of the
    // real pipeline (MemReadE, CSRReadE, MDUE), which only c has.  c computes it for both lanes.
    .StructuralStallD(), .LoadStallD(), .StoreStallD(),
    .Rs1_2D(5'b0), .Rs2_2D(5'b0), .StructuralStall2D(), .SlowResultE(), .LateBrE(1'b0), .PredTakenE(1'b0),
    .Rs1D(Rs1_2D), .Rs2D(Rs2_2D), .Rs2E(Rs2_2E), .MemRWD(MemRW2D),
    // An unpaired slot 1 is flushed on its way into Execute rather than being allowed down the
    // pipeline and suppressed at each consumer.  Its control signals, RegWrite2M/W and Rd2M/W then
    // come out as zero on their own, so nothing downstream needs to know about pairing.
    .StallE, .FlushE(FlushE | ~Issue2D), .FlagsE(2'b00), .FWriteIntE(1'b0),
    .PCSrcE(), .ALUSrcAE(ALUSrcA2E), .ALUSrcBE(ALUSrcB2E), .ALUResultSrcE(ALUResultSrc2E), .ALUSelectE(ALUSelect2E),
    .Funct3E(Funct3_2E), .Funct7E(Funct7_2E), .IntDivE(IntDiv2E), .W64E(W64_2E), .UW64E(UW64_2E), .SubArithE(SubArith2E),
    .BranchD(Branch2D), .BranchE(), .JumpD(Jump2D), .JumpE(Jump2E),
    .BranchSignedE(), .BSelectE(BSelect2E), .ZBBSelectE(ZBBSelect2E), .BALUControlE(BALUControl2E),
    .BMUActiveE(BMUActive2E), .CZeroE(CZero2E), .MDUActiveE(MDUActive2E),
    .FCvtIntE(1'b0), .ForwardAE(Forward2AE), .ForwardBE(Forward2BE), .CMOpM(), .IFUPrefetchE(), .LSUPrefetchM(),
    // Only the two signals the LSU reads per-lane are connected.  AtomicM, CSRReadM, CSRWriteM and
    // PrivilegedM stay open because the issue rules keep atomics, CSR accesses and privileged
    // instructions in slot 0.  CMOpM and FlushDCacheM stay open because cbo.* and fence.i are
    // MISC_MEM, which slot 1's whitelist never admits, so they are constant zero here.  And
    // LSUPrefetchM, like MemRWE, is a dead input on the LSU: declared, never read.
    // A slot 1 killed in Execute by a taken branch in slot 0 is flushed on its way into Memory, for the
    // same reason.  Gating RegWrite2M/W below is not enough on its own: c2's forwarding logic reads its
    // own RegWriteM/W, so a dead slot 1 would still forward its result to the next bundle's slot 1.
    // A correctly predicted taken branch puts its target in Execute the very next cycle, so this
    // happens whenever slot 1 of the target bundle reads the fall-through's destination.
    .StallM, .FlushM(FlushM | PCSrcE), .MemRWE(MemRW2E), .MemRWM(MemRW2MRaw), .CSRReadM(), .CSRWriteM(), .PrivilegedM(), .AtomicM(), .Funct3M(Funct3_2M),
    .FlushDCacheM(), .InstrValidM(), .InstrValidE(), .InstrValidD(InstrValid2D), .FWriteIntM(),
    .IllegalBaseInstrM(IllegalBaseInstr2MRaw),
    .RegWriteM(RegWrite2MRaw), .RegWriteOtherM(RegWriteM),    // lane 2 forwards from lane 1
    .StallW, .FlushW, .RegWriteW(RegWrite2WRaw), .IntDivW(), .ResultSrcW(ResultSrc2W), .CSRWriteFenceM(), .InvalidateICacheM(),
    .RegWriteOtherW(RegWriteW),
    .RdW(Rd2W), .RdE(Rd2E), .RdM(Rd2M), .RdOtherM(RdM), .RdOtherW(RdW));

  // Slot 1 can be abandoned in Execute, after its controller's Execute register has already loaded --
  // when slot 0 turns out to be a taken branch and slot 1 was the fall-through.  c2's FlushM above
  // clears that case, and Issue2W adds the Memory-stage replay, so qualify the register-write signals
  // with Issue2M/Issue2W here: these feed both the second write port and lane 1's forwarding network,
  // so an abandoned slot 1 neither writes nor is bypassed.
  assign RegWrite2M = RegWrite2MRaw & Issue2M;
  assign RegWrite2W = RegWrite2WRaw & Issue2W;

  // A slot 1 store reaches memory in the Memory stage, so it needs the same commit gate: without it
  // an abandoned slot 1 would still write memory, which no later redirect can undo.  Gating here
  // rather than in Execute is also the cheaper placement -- both inputs are register outputs, where
  // Kill2E (and so PCSrcE, out of the branch comparator) arrives late in Execute.
  assign MemRW2M = MemRW2MRaw & {2{Issue2M}};

  // Slot 1 holds an encoding that is not a legal instruction in this configuration.  Fetch cannot tell
  // that apart from a legal one -- sh1add and an illegal encoding are the same bits, separated only by
  // P.ZBA_SUPPORTED -- so the issue rules let slot 1 take any encoding its ALU could execute and leave
  // the verdict to c2's own decoder, which already reaches it.  Slot 1 never executes it: the commit
  // gate stops the register write, and the replay discards it and refetches it as slot 0, the only
  // lane the privileged unit can describe.
  assign Illegal2M = IllegalBaseInstr2MRaw & Issue2M;

  // Which lane owns the single load/store unit.  The issue rules permit at most one memory operation
  // per bundle, so "slot 1 has one" is the same statement as "slot 0 does not", and one select does
  // for both.  Mem2E steers the Execute-stage inputs -- the address and the store data, in
  // datapath.sv -- and Mem2M the control the LSU reads in Memory.  Neither needs a register of its
  // own, because c2 already pipelines its control from Execute to Memory alongside c.
  // ~PCSrcE is the same condition as ifu.sv's Kill2E: a taken branch in slot 0 leaves slot 1 as the
  // fall-through, which must not act.  MemRW2M already stops the access, so this is not needed for
  // correctness -- it keeps the LSU from latching a dead slot 1's address into IEUAdrM, which the
  // branch predictor reads when it updates a target.  It is free because, with the address reaching
  // the LSU on its own net, nothing downstream of this select in Execute is more than a register.
  assign Mem2E = |MemRW2E & ~PCSrcE;
  assign Mem2M = |MemRW2M;

  // MemRWM reaches nothing but the load/store unit, so it is muxed in place.  Funct3M is not that
  // private: the MDU selects its result from it (mul vs mulh vs div vs rem) and the FPU's integer
  // divide reads it too.  Muxing it would hand an MDU operation in slot 0 the funct3 of a load in
  // slot 1 and silently return the wrong product, so the memory funct3 goes out on its own net
  // instead, exactly as LSUAdrE does for the address.
  mux2 #(2) memrwmmux(MemRW0M, MemRW2M, Mem2M, MemRWM);
  mux2 #(3) memfunct3mmux(Funct3M, Funct3_2M, Mem2M, MemFunct3M);

  // Which lane owns the single multiply/divide unit, by the same reasoning as the load/store unit
  // above: the issue rules permit at most one MDU operation per bundle.  Unlike Mem2E this select
  // needs no ~PCSrcE.  The multiplier has no enable and no side effect -- it multiplies whatever it is
  // given every cycle, and only the selected, committed result is ever written -- so a slot 1 multiply
  // killed by a taken branch in slot 0 computes a product that RegWrite2M/W, gated by Issue2M/W, never
  // write.  The divider does act when it starts, but issue.sv never pairs a slot 1 divide with a slot
  // 0 branch, so a slot 1 divide that reaches Execute always commits.  MDUActive2E comes straight out
  // of c2's Execute register, so the select adds no logic ahead of the operand muxes in datapath.sv.
  //
  // The MDU's funct3, W64 and divide start go out on their own nets, as MemFunct3M does: slot 0's
  // Funct3E/M, W64E and IntDivE also feed the FPU's integer divide and conversions, which must keep
  // seeing slot 0.
  // Which of slot 1's operands the late ALU replaces with slot 0's result.  issue.sv matched raw
  // register fields, which for an immediate or a PC operand are not registers at all, so the bits are
  // cleared here for an operand the ALU does not take from the register file, and for lui, whose
  // result is its immediate.  What remains is exactly the set of real dependencies, because issue.sv
  // never pairs a dependent slot 1 it could not mark.  The register flushes like c2's Execute stage,
  // so an unpaired slot 1 is never late.
  // The Decode-stage compare against the decompressed fields is belt and braces: instrfields may name a
  // register that a compressed HINT's decompressed form replaces with x0, and the late ALU must never
  // substitute slot 0's result for an operand that reads some other register.
  logic LateA2DQ, LateB2DQ;
  assign LateA2DQ = LateA2D & (Rs1_2D == InstrD[11:7]);
  assign LateB2DQ = LateB2D & (Rs2_2D == InstrD[11:7]);
  flopenrc #(2) Late2EReg(clk, reset, FlushE | ~Issue2D, ~StallE, {LateA2DQ, LateB2DQ}, {LateA2RawE, LateB2RawE});

  // The late ALU also takes an operand that the bundle now in Execute produces too late to forward --
  // a load, a multiply or divide, a CSR read or an FPU-to-integer move in slot 0, or a load, multiply
  // or late operation in slot 1.  Instead of stalling the bundle in Decode, slot 1 goes late: when it
  // reaches Memory its producer is in Writeback, and the value is in ResultW or ResultW2.  Slot 1 of
  // the producing bundle is younger, so it is checked first; when it writes the register but is fast,
  // ordinary forwarding from Memory is correct and nothing is needed.  Only for a slot 1 the late ALU
  // can execute (LateOK2D); otherwise the stalls below still apply.
  //
  // This depends on the producer being exactly one stage ahead when slot 1 reaches Memory, which the
  // pipeline guarantees except when Execute is held while Memory advances (CaptureE): then the producer
  // retires first, but datapath.sv's operand capture has meanwhile picked its value up in Execute, so
  // these bits are cleared and slot 1 takes its operands as captured.
  assign Slow2E   = MemRW2E[1] | MDUActive2E | Late2E;
  assign LateWA2D = ((Rd2E != 5'b0) & (Rs1_2D == Rd2E)) ? {2{Slow2E}} :
                    ((RdE  != 5'b0) & (Rs1_2D == RdE) & SlowResult0E) ? 2'b10 : 2'b00;
  assign LateWB2D = ((Rd2E != 5'b0) & (Rs2_2D == Rd2E)) ? {2{Slow2E}} :
                    ((RdE  != 5'b0) & (Rs2_2D == RdE) & SlowResult0E) ? 2'b10 : 2'b00;
  assign CaptureE = StallE & ~StallM;

  // Late branch.  Slot 0 is a conditional branch with an operand produced too late, exactly as LateWA2D
  // describes for slot 1, so rather than stall it in Decode it passes through Execute and the late ALU
  // compares its operands in Memory, taking the late one from ResultW or ResultW2.  Until then it goes
  // the way fetch went (PredTakenE): Execute neither redirects on its direction nor trains the predictor
  // with it, but still checks and corrects its target.  If Memory finds the other direction, ifu.sv
  // redirects from there and hazard.sv flushes what was fetched behind it, as for a replay.  That costs
  // one cycle more than an Execute-stage misprediction, against the stall cycle it saves every time.
  //
  // The late ALU serves one instruction per bundle, so a slot 1 that needs it, or that holds the
  // memory operation (whose access would happen beside a branch not yet known to fall through, and
  // which might need a replay of its own), keeps the bundle on the ordinary stalling path.  The
  // redirect uses the Memory-stage path that CSR writes and fences use, and the target misalignment
  // check stays in Execute, so a late branch also needs those and compressed instructions.
  assign LateWA0D = ((Rd2E != 5'b0) & (Rs1D == Rd2E)) ? {2{Slow2E}} :
                    ((RdE  != 5'b0) & (Rs1D == RdE) & SlowResult0E) ? 2'b10 : 2'b00;
  assign LateWB0D = ((Rd2E != 5'b0) & (Rs2D == Rd2E)) ? {2{Slow2E}} :
                    ((RdE  != 5'b0) & (Rs2D == RdE) & SlowResult0E) ? 2'b10 : 2'b00;
  assign LateBrD  = (P.ZICSR_SUPPORTED | P.ZIFENCEI_SUPPORTED) & P.ZCA_SUPPORTED &
                    BranchD & (|LateWA0D | |LateWB0D) &
                    ~(Issue2D & (|MemRW2D | LateA2DQ | LateB2DQ | |LateWA2D | |LateWB2D));
  flopenrc #(1) LateBrEReg(clk, reset, FlushE, ~StallE, LateBrD, LateBrE);

  // The late ALU's Writeback operand selects serve slot 1 normally and slot 0's branch when there is one.
  flopenrc #(4) LateW2EReg(clk, reset, FlushE | (~Issue2D & ~LateBrD) | CaptureE, ~StallE | CaptureE,
                           LateBrD ? {LateWA0D, LateWB0D} : ({LateWA2D, LateWB2D} & {4{LateOK2D}}),
                           {LateWA2RawE, LateWB2RawE});

  // Each operand's source for the late ALU: 00 as captured in Execute, 01 slot 0's result in Memory,
  // 10 ResultW, 11 ResultW2.  Slot 0 of the same bundle is the youngest producer, so it wins.  Nothing
  // applies to an operand that is not a register (the ALU takes the PC or an immediate there) or to lui.
  // A late branch's two operands are always registers.
  assign LateSelA2E = LateBrE ? LateWA2RawE :
                      (ALUSrcA2E | ALUResultSrc2E) ? 2'b00 : LateA2RawE ? 2'b01 : LateWA2RawE;
  assign LateSelB2E = LateBrE ? LateWB2RawE :
                      (ALUSrcB2E | ALUResultSrc2E) ? 2'b00 : LateB2RawE ? 2'b01 : LateWB2RawE;
  assign Late2E     = ~LateBrE & (|LateSelA2E | |LateSelB2E);   // slot 1 itself is late

  // In Memory: the branch's real direction from the late ALU's flags, by funct3 as the comparator does.
  flopenrc #(2) LateBrMReg(clk, reset, FlushM, ~StallM, {LateBrE, PCSrcE}, {LateBrM, AssumedTakenM});
  always_comb
    case (Funct3M)
      3'b000:  LateBrTakenM =  LateFlagsM[2];   // beq
      3'b001:  LateBrTakenM = ~LateFlagsM[2];   // bne
      3'b100:  LateBrTakenM =  LateFlagsM[1];   // blt
      3'b101:  LateBrTakenM = ~LateFlagsM[1];   // bge
      3'b110:  LateBrTakenM =  LateFlagsM[0];   // bltu
      3'b111:  LateBrTakenM = ~LateFlagsM[0];   // bgeu
      default: LateBrTakenM = 1'b0;
    endcase
  assign LateBrWrongM = LateBrM & (LateBrTakenM != AssumedTakenM);

  assign Mdu2E = MDUActive2E;
  flopenrc #(1) Mdu2MReg(clk, reset, FlushM, ~StallM, Mdu2E, Mdu2M);
  assign MDUActiveE = MDUActive0E | MDUActive2E;
  mux2 #(3) mdufunct3emux(Funct3E, Funct3_2E, Mdu2E, MDUFunct3E);
  mux2 #(3) mdufunct3mmux(Funct3M, Funct3_2M, Mdu2M, MDUFunct3M);
  mux2 #(1) mduw64emux(W64E, W64_2E, Mdu2E, MDUW64E);
  mux2 #(1) mduintdivemux(IntDivE, IntDiv2E, Mdu2E, MDUIntDivE);

  // Memory hazards for the bundle.  The controller states both of these against slot 0's Execute stage,
  // which covers every case where slot 0 holds the memory operation.  Once slot 1 may hold it instead,
  // the same two hazards have to be stated again with slot 1 as the producer.
  //
  // Load-use: a load's data is not ready in Memory, so a dependent instruction one bundle behind has
  // to wait.  Lane 2's Memory-stage forwarding source, IEUResult2M, is slot 1's ALU result, which for
  // a load is its address -- forwarding it would quietly supply the wrong value.  One stall cycle
  // pushes the consumer to forward from Writeback instead, where ResultW2 is the real load data.  All
  // four Decode-stage sources are checked, because either slot may be the consumer.
  //
  // In all three of these, a slot 1 consumer the late ALU can execute does not stall: it goes late and
  // takes the value from ResultW2 (see LateWA2D above).
  assign LoadStall2D = MemRW2E[1] & (Rd2E != 5'b0) &
                       (~LateBrD & ((Rs1D == Rd2E) | (Rs2D == Rd2E)) |
                        (Issue2D & ~LateOK2D & ((Rs1_2D == Rd2E) | (Rs2_2D == Rd2E))));

  // Multiply/divide-use, for the same reason: the result is not complete until the end of Memory, so
  // it first appears in ResultW2, and IEUResult2M holds slot 1's meaningless ALU output.  This is the
  // slot 1 counterpart of the controller's MDUStallD, which charges slot 0's MDU operations the same
  // cycle.  A divide's iterations happen while DivBusyE holds the whole pipeline, so from the
  // consumer's point of view a divide is no later than a multiply once it leaves Execute.
  assign MDUStall2D = MDUActive2E & (Rd2E != 5'b0) &
                      (~LateBrD & ((Rs1D == Rd2E) | (Rs2D == Rd2E)) |
                       (Issue2D & ~LateOK2D & ((Rs1_2D == Rd2E) | (Rs2_2D == Rd2E))));

  // Late-ALU-use, for the same reason again: a late slot 1 computes its result in Memory, so it first
  // appears in ResultW2, and IEUResult2M holds what slot 1's Execute ALU made of a stale operand.
  // Forwarding the late ALU's output to Execute instead would chain it into the next bundle's ALU,
  // which is the critical path the late ALU exists to avoid.
  assign LateStall2D = Late2E & (Rd2E != 5'b0) &
                       (~LateBrD & ((Rs1D == Rd2E) | (Rs2D == Rd2E)) |
                        (Issue2D & ~LateOK2D & ((Rs1_2D == Rd2E) | (Rs2_2D == Rd2E))));

  // Store-then-load: a structural conflict over the data memory's single SRAM port, not a data
  // dependency, so it ignores registers entirely.  A read is issued in Execute and a write happens in
  // Memory, so a load one stage behind a store would want the port in the very cycle the store writes
  // it.  The controller states this for slot 0 against slot 0, which is one of four combinations now
  // that the load and the store may sit in different slots.  A bundle holds at most one memory
  // operation, so each side is just an OR over the slots, and the controller's own term becomes
  // redundant rather than wrong.
  assign BundleStoreStallD = (MemRW0D[1] | (Issue2D & MemRW2D[1]))   // the bundle in Decode has a load
                           & (MemRWE[0]  | MemRW2E[0]);              // the bundle in Execute has a store

  // Stall the bundle when slot 1 depends on a result that cannot be forwarded yet, exactly as slot 0
  // already does.  Gated by Issue2D so a slot 1 hazard costs nothing on cycles where the two slots
  // were never going to issue together.  The two hazards above need no such gate: they concern the
  // bundle already in Execute, which has issued whatever it issued.  A slot 1 the late ALU can execute
  // takes the slow result from ResultW instead.
  // A late branch keeps only the store-then-load stall of slot 0's own: its data hazards are what it avoids.
  assign StructuralStall0GD = LateBrD ? StoreStallD : StructuralStall0D;
  assign StructuralStallD = StructuralStall0GD | (Issue2D & ~LateOK2D & StructuralStall2D) | LoadStall2D | MDUStall2D | LateStall2D | BundleStoreStallD;

  datapath #(P) dp(
    .clk, .reset, .ImmSrcD, .InstrD, .Rs1D, .Rs2D, .Rs2E, .StallE, .FlushE, .ForwardAE, .ForwardBE, .W64E, .UW64E, .SubArithE,
    .ImmSrc2D, .Instr2D, .Rs1_2D, .Rs2_2D, .Rs2_2E, .Forward2AE, .Forward2BE,                       // slot 1
    .ALUSrcA2E, .ALUSrcB2E, .ALUResultSrc2E, .ALUSelect2E, .Funct3_2E, .Funct7_2E,                  // slot 1
    .W64_2E, .UW64_2E, .SubArith2E, .BSelect2E, .ZBBSelect2E, .BALUControl2E, .BMUActive2E, .CZero2E, // slot 1
    .RegWrite2W, .Rd2W,                                                                             // slot 1 write port
    .Mem2E, .ResultSrc2W,                                                                           // slot 1 memory steering
    .Mdu2E, .MDUSrcAE, .MDUSrcBE,                                                                   // slot 1 multiply steering
    .LateSelA2E, .LateSelB2E, .LateBrE, .LateFlagsM,                                                // late ALU
    .Funct3E, .Funct7E, .ALUSrcAE, .ALUSrcBE, .ALUResultSrcE, .ALUSelectE, .JumpE, .BranchSignedE,
    .PCE, .PCLinkE, .FlagsE, .IEUAdrE, .LSUAdrE, .ForwardedSrcAE, .ForwardedSrcBE, .BSelectE, .ZBBSelectE, .BALUControlE, .BMUActiveE, .CZeroE,
    .StallM, .FlushM, .FWriteIntM, .FIntResM, .SrcAM, .WriteDataM, .FCvtIntW,
    .StallW, .FlushW, .RegWriteW, .IntDivW, .SquashSCW, .ResultSrcW, .ReadDataW, .FCvtIntResW,
    .CSRReadValW, .MDUResultW, .FIntDivResultW, .RdW);
endmodule
