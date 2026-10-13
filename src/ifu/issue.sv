///////////////////////////////////////////
// issue.sv
//
// Written: September 2026
//
// Purpose: Decide whether the two instructions in the fetch window may issue together.
//
//          This runs in the Fetch stage because the PC advance is computed there: when both
//          slots issue the PC must move by both their lengths in the same cycle.  Everything it
//          needs is a fixed bit field of an uncompressed instruction, so no decode is required.
//
//          The rule is a whitelist for pairing, not for execution.  When Issue2F is low the core
//          behaves exactly as the original single-issue Wally: slot 0 issues by itself and slot 1
//          is fetched again next cycle as the next slot 0.  No instruction is ever skipped.
//
//          Slot 1 is restricted by the hardware it has, not by legality.  It may hold anything its
//          own ALU can finish in one cycle -- including the Zb*/Zk*/Zicond encodings, since its
//          controller has its own bitmanip decoder -- plus one load and one multiply or divide,
//          because the single load/store unit and the single multiply/divide unit can each be
//          steered to either slot.  It may not hold a CSR access or a control transfer.  Stores are
//          excluded for a reason that is about replay rather than hardware; see the LOAD arm below.
//
//          Legality is left to the decoder that already knows it.  Where slot 1 turns out to hold an
//          instruction this configuration does not implement, or one whose memory access faults, the
//          Memory stage cannot describe the trap -- it holds slot 0's PC, cause and tval -- so slot 1
//          is discarded and refetched to run alone, and traps through the ordinary single-issue path.
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

module issue import cvw::*;  #(parameter cvw_t P) (
  input  logic [31:0] Instr0F,        // Slot 0 raw instruction (at PCF)
  input  logic [31:0] Instr1F,        // Slot 1 raw instruction, already aligned to bit 0
  input  logic        Instr2ValidF,   // Slot 1 holds a complete instruction from the I$
  input  logic        BPPCSrcF,       // Branch predictor says this fetch is a taken branch or jump
  output logic        Issue2F         // Both slots may issue together this cycle
);

  // Opcodes
  localparam [6:0] LUI      = 7'b0110111;
  localparam [6:0] AUIPC    = 7'b0010111;
  localparam [6:0] OP       = 7'b0110011;
  localparam [6:0] OP_IMM   = 7'b0010011;
  localparam [6:0] OP_32    = 7'b0111011;   // RV64 only: addw, subw, sllw, srlw, sraw
  localparam [6:0] OP_IMM_32= 7'b0011011;   // RV64 only: addiw, slliw, srliw, sraiw
  localparam [6:0] JAL      = 7'b1101111;
  localparam [6:0] JALR     = 7'b1100111;
  localparam [6:0] SYSTEM   = 7'b1110011;
  localparam [6:0] MISC_MEM = 7'b0001111;
  // LOAD is still named for the access-width check below.  The other memory opcodes -- STORE, the
  // floating-point loads and stores, and the atomics -- are no longer listed here: instrfields
  // reports which slots occupy the load/store unit, for compressed and 32-bit instructions alike.
  localparam [6:0] LOAD     = 7'b0000011;

  logic [6:0] Op0F, Op1F, Funct7_1F;
  logic [2:0] Funct3_1F;
  logic [4:0] Rd0F;                          // What slot 0 writes -- its sources do not matter, it runs first
  logic [4:0] Rd1F, Rs1_1F, Rs2_1F;          // Slot 1's register fields, whatever its length
  logic       Recognized0F, Recognized1F;    // instrfields decoded this slot's fields
  logic [1:0] MemRW0F, MemRW1F;              // ... and whether it uses the load/store unit
  logic       IsBranch0F, IsBranch1F;        // ... and whether each slot is a conditional branch
  logic       BothRecognizedF, Slot0CanPairF, Slot1CanPairF, NoDependencyF;
  logic       Slot0IsMemF, Slot1IsMemF, OneMemOpF;
  logic       MExtF;                      // Slot 1 is a multiply/divide, which needs the one MDU
  logic       MulF;                       // ... and specifically a multiply, which the MDU can take from slot 1
  logic       DivF;                       // ... or a divide, which it can take when the divider is the MDU's
  logic       MduOKF;                     // Slot 1's multiply or divide may pair
  logic       Slot0ReadsFPF;              // Slot 0 reads floating-point registers
  logic       Slot0IsMduF, Slot1IsMduF, OneMduOpF;

  assign Op0F      = Instr0F[6:0];
  assign Op1F      = Instr1F[6:0];
  assign Funct3_1F = Instr1F[14:12];
  assign Funct7_1F = Instr1F[31:25];

  // The register fields come from instrfields rather than from fixed bit positions, because a
  // compressed instruction keeps them elsewhere.  Each slot reports Recognized low for any encoding
  // whose fields are not decoded, and the dependency check below refuses to pair in that case.
  // Slot 0's sources are not connected: only what it writes can create a hazard with slot 1.  A
  // write-after-read cannot arise, because slot 0 read its operands in Decode and slot 1 does not
  // write until Writeback, and the two move together through the pipeline.
  instrfields #(P) fields0(.Instr(Instr0F), .Rd(Rd0F), .Rs1(), .Rs2(), .MemRW(MemRW0F), .IsBranch(IsBranch0F), .Recognized(Recognized0F));
  instrfields #(P) fields1(.Instr(Instr1F), .Rd(Rd1F), .Rs1(Rs1_1F), .Rs2(Rs2_1F), .MemRW(MemRW1F), .IsBranch(IsBranch1F), .Recognized(Recognized1F));

  // A slot is admitted only if its register fields are known.  For a 32-bit instruction that is
  // always true, so this reduces to the previous behaviour; for a compressed one it is instrfields'
  // verdict, which is low for every format not decoded there.  That is what makes a gap in
  // instrfields cost pairing opportunities rather than correctness -- and it covers slot 0 too,
  // whose Op0F comparisons below are meaningless when it is compressed.
  assign BothRecognizedF = Recognized0F & Recognized1F;

  // Which instructions may carry a partner in slot 1.  These all still execute normally on their
  // own when they cannot pair.
  //   ~BPPCSrcF is required unconditionally, and is about the PC mux rather than the opcode: when
  //     BPPCSrcF is high the next PC becomes the predicted target and the sequential PC is discarded,
  //     so a paired slot 1 would issue while the machine jumped somewhere else.  The predictor can
  //     say taken for an instruction that is not a branch at all (a class misprediction on a stale
  //     or aliased BTB entry), so this cannot be qualified by the opcode.
  //     With it, a conditional branch predicted not-taken DOES pair; if it later resolves taken,
  //     Kill2E in ifu.sv squashes slot 1 before it commits and the redirect re-fetches it.
  //   jal/jalr: never pair.  The instruction after an unconditional jump is never the next one
  //     executed, and BPJumpF forces BPPCSrcF high for them anyway.
  //   SYSTEM: mret and sret redirect the PC from the Memory stage without flushing Writeback, so a
  //     slot 1 sharing that stage would commit after the return had already been taken.  This one
  //     cannot be fixed by killing slot 1 later -- it is already past the point of no return.
  //   MISC_MEM: fence.i flushes the pipeline behind it.
  assign Slot0CanPairF = ~BPPCSrcF & (Op0F != JAL) & (Op0F != JALR) &
                         (Op0F != SYSTEM) & (Op0F != MISC_MEM);

  // What slot 1 may hold is a statement about hardware, not about legality.  Slot 1 has its own full
  // controller, including its own bmuctrl, so it already decodes every Zba/Zbb/Zbs/Zbc/Zbk/Zk/Zicond
  // encoding this configuration enables and flags the rest as illegal.  Fetch cannot make that
  // judgement and does not try: sh1add and an illegal encoding are the same bits, separated only by
  // P.ZBA_SUPPORTED, so an encoding that turns out illegal is replayed from the Memory stage and traps
  // as an unpaired instruction (see Replay2M).  That keeps the one copy of the encoding table in the
  // decoder that owns it, rather than a second copy here that could silently disagree with it.
  //
  // Multiplies and divides share the OP and OP-32 opcodes with the ALU instructions but need the one
  // multiply/divide unit, and the decoder calls them perfectly legal -- so nothing downstream would
  // catch one that slot 1 could not execute.  ieu.sv steers the MDU's operands, funct3, W64 and
  // divide start to whichever slot holds the bundle's MDU operation, and slot 1's writeback takes its
  // result -- the same arrangement as the one load/store unit.  OP-IMM needs no such check: the M
  // extension has no OP-IMM encoding, and bits 31:25 of an addi are immediate bits that may hold any
  // value.
  //
  // Multiplies (mul, mulh, mulhsu, mulhu, and mulw on RV64) may always pair.  The multiplier has no
  // enable and no side effect, so a slot 1 multiply that is later killed or replayed costs nothing:
  // its register write is stopped by the commit gate like any slot 1 ALU operation.  RV64 OP-32 with
  // funct7 = 0000001 is mulw only at funct3 000; the other multiply funct3 values are reserved there.
  //
  // Divides (div, divu, rem, remu, and the RV64 W forms) may pair when the divider is the MDU's own --
  // the same condition under which mdu.sv builds one.  With IDIV_ON_FPU the FPU divides instead, from
  // slot 0's operands, so slot 1 divides stay out there.  And not beside a conditional branch in slot
  // 0: unlike the multiplier, the divider acts as soon as it starts, taking DivBusyE high and holding
  // the pipeline for the whole divide.  A branch that resolved taken in the same cycle would have
  // started a divide for the fall-through, which must not execute, and hazard.sv deliberately keeps a
  // busy divider from being flushed by a misprediction.  Refusing the pair keeps the divider from ever
  // starting on a slot 1 that can be killed: the other way slot 1 is killed is a jump, which never
  // pairs, and a slot 0 predicted taken does not pair either.
  //
  // Nor beside a slot 0 that reads floating-point registers.  The divide holds the Execute stage for
  // its whole iteration while Memory and Writeback drain, so slot 0 sits there with operands whose
  // forwarding sources retire under it.  datapath.sv's operand capture keeps the integer operands
  // right through such a stall, but the FPU's Execute registers have no equivalent, so a slot 0 FP
  // store, FP arithmetic or fused multiply-add stays out.  Compressed FP loads and stores are not
  // recognized by instrfields, so only 32-bit encodings need checking.
  assign MExtF  = (Funct7_1F == 7'b0000001);
  assign MulF   = P.ZMMUL_SUPPORTED & MExtF & ~Funct3_1F[2];
  assign DivF   = P.M_SUPPORTED & ~(P.IDIV_ON_FPU & P.F_SUPPORTED) & MExtF & Funct3_1F[2];
  assign Slot0ReadsFPF = (&Instr0F[1:0]) &
                         ((Op0F == 7'b0100111) | (Op0F == 7'b1010011) |       // FP store, FP arithmetic
                          (Op0F[6:4] == 3'b100) & (Op0F[1:0] == 2'b11));      // fmadd, fmsub, fnmsub, fnmadd
  assign MduOKF = MulF | (DivF & ~IsBranch0F & ~Slot0ReadsFPF);

  always_comb
    // A compressed slot 1 cannot go through the opcode whitelist below -- Instr1F[6:0] is not an
    // opcode for a 16-bit instruction.  instrfields vouches for it instead: the formats it decodes
    // today are all single-cycle ALU operations, so recognition and class-legality coincide.  Adding
    // a non-ALU compressed format there for its register fields must come with a separate class
    // signal, or it would silently become pairable here.
    // A compressed slot 1 is admitted on instrfields' recognition, minus a store.  Stores stay in
    // slot 0 for the replay reason described in the LOAD arm below -- the same restriction the
    // 32-bit STORE opcode gets by having no arm at all.
    if (~&Instr1F[1:0]) Slot1CanPairF = Recognized1F & ~MemRW1F[0] & ~IsBranch1F;
    else case (Op1F)
      LUI, AUIPC: Slot1CanPairF = 1'b1;
      OP_IMM:     Slot1CanPairF = 1'b1;
      OP:         Slot1CanPairF = ~MExtF | MduOKF;
      OP_IMM_32:  Slot1CanPairF = (P.XLEN == 64);
      // On OP-32, funct3 001-011 with funct7 = 0000001 is reserved; mulw and the four W divides are not
      OP_32:      Slot1CanPairF = (P.XLEN == 64) & (~MExtF | (MduOKF & ((Funct3_1F == 3'b000) | Funct3_1F[2])));
      // A load may pair; a store may not, and the reason is replay rather than the load/store unit,
      // which handles either slot equally well.  A faulting store must leave memory untouched, and the
      // only thing that cancels a write in this design is FlushW, which a trap supplies.  A replay
      // withholds FlushW so that slot 0 can still commit, so a replayed slot 1 store would reach memory
      // and only then trap, leaving a handler to find state already modified.  Cancelling it needs the
      // write stopped downstream of the MMU -- at CacheRW, after the MMU has decided store-versus-load
      // page fault from WriteAccessM -- which is a change to the load/store unit that nothing currently
      // tests, since no test in the suite produces a faulting store in slot 1.  Until that is built and
      // proven, stores stay in slot 0, where a trap cancels them exactly as it always has.  The cost is
      // 0.31% of CoreMark cycles; loads are the rest of the benefit.
      //
      // Funct3 is checked even though replay would also catch an illegal width: the decoder's LFunctD
      // leaves ControlsD at its illegal default, which carries MemRW = 00, so no access is made either
      // way.  Checking here declines to pair rather than issuing and then undoing it, and keeps the
      // rule the same for a configuration whose decoder does not examine funct3 at all.
      LOAD:       case (Funct3_1F)
                    3'b011, 3'b110: Slot1CanPairF = (P.XLEN == 64);   // ld, lwu
                    3'b111:         Slot1CanPairF = 1'b0;             // reserved
                    default:        Slot1CanPairF = 1'b1;             // lb, lh, lw, lbu, lhu
                  endcase
      default:    Slot1CanPairF = 1'b0;
    endcase

  // Slot 1 must not depend on slot 0.  Forwarding slot 0's result to slot 1 within the same cycle
  // would chain one ALU into the other and roughly double the critical path, so such pairs simply
  // issue one at a time.  Rd0F is treated as a destination whenever it is nonzero, which is
  // conservative for stores (where those bits are part of the immediate) but never unsafe.
  assign NoDependencyF = (Rd0F == 5'b0) |
                         ((Rs1_1F != Rd0F) & (Rs2_1F != Rd0F) & (Rd1F != Rd0F));

  // The core has one load/store unit, so a bundle may contain at most one memory operation -- but it
  // may sit in either slot, and ieu.sv steers the LSU's inputs to whichever one holds it.  Slot 0's
  // list is much the wider: slot 1 may only ever hold a LOAD, while slot 0 may also hold a store, a
  // floating-point load or store, or an atomic, each of which occupies the LSU just the same.
  assign Slot0IsMemF = |MemRW0F;
  assign Slot1IsMemF = |MemRW1F;
  assign OneMemOpF   = ~(Slot0IsMemF & Slot1IsMemF);

  // Likewise there is one multiply/divide unit, so a bundle may hold at most one MDU operation.  Only
  // 32-bit encodings are MDU operations here: c.mul is not recognized by instrfields, so a compressed
  // slot never pairs as one, and for a compressed slot 0 bits 31:25 belong to the next instruction.
  assign Slot0IsMduF = (&Instr0F[1:0]) & ((Op0F == OP) | (Op0F == OP_32)) & (Instr0F[31:25] == 7'b0000001);
  assign Slot1IsMduF = (&Instr1F[1:0]) & ((Op1F == OP) | (Op1F == OP_32)) & MExtF;
  assign OneMduOpF   = ~(Slot0IsMduF & Slot1IsMduF);

  assign Issue2F = Instr2ValidF & BothRecognizedF & Slot0CanPairF & Slot1CanPairF &
                   NoDependencyF & OneMemOpF & OneMduOpF;

endmodule
