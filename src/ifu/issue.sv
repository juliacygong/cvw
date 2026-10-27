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
  output logic        Issue2F,        // Both slots may issue together this cycle
  output logic        LateA2F, LateB2F, // Slot 1's rs1 / rs2 is slot 0's rd: its operand comes from the late ALU's bypass
  output logic        LateOK2F          // Slot 1 is an operation the late ALU implements
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
  logic       BothRecognizedF, Slot0CanPairF, Slot1CanPairF, DependencyOKF;
  logic       Slot0EarlyF;                // Slot 0 is an integer ALU operation: its result is in IEUResultM
  logic       Slot1LateOKF;               // Slot 1 is an operation the late ALU implements
  logic       SrcDepF, SameRdF;           // Slot 1 reads / overwrites slot 0's destination (SameRdF: statistics only)
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

  // Slot 1 may read slot 0's result only through the late ALU in the Memory stage.  Forwarding it
  // within Execute would chain one ALU into the other and roughly double the critical path; one
  // stage later slot 0's result is already in its IEUResultM register, and the late ALU takes it
  // from there.  Rd0F is treated as a destination whenever it is nonzero, which is conservative for
  // stores and branches (where those bits are part of the immediate) but never unsafe.
  assign SrcDepF = (Rd0F != 5'b0) & ((Rs1_1F == Rd0F) | (Rs2_1F == Rd0F));
  assign SameRdF = (Rd0F != 5'b0) & (Rd1F == Rd0F);

  // A dependent pair needs slot 0's result at the start of Memory, so slot 0 must be an integer ALU
  // operation -- not a load, a multiply or divide, a CSR read or an FPU-to-integer move, whose results
  // arrive later.  The opcodes are the ones whose IEUResultE is final; a compressed slot 0 has already
  // been vouched for by instrfields, and with its memory operations and branches excluded the formats
  // left are all ALU operations.  OP and OP-32 carry the multiplies and divides at funct7 = 0000001.
  always_comb
    if (~&Instr0F[1:0]) Slot0EarlyF = ~|MemRW0F & ~IsBranch0F;
    else case (Op0F)
      LUI, AUIPC, OP_IMM: Slot0EarlyF = 1'b1;
      OP_IMM_32:          Slot0EarlyF = (P.XLEN == 64);
      OP:                 Slot0EarlyF = (Instr0F[31:25] != 7'b0000001);
      OP_32:              Slot0EarlyF = (P.XLEN == 64) & (Instr0F[31:25] != 7'b0000001);
      default:            Slot0EarlyF = 1'b0;
    endcase

  // The late ALU implements add, sub, the logical operations, shifts and set-less-than, with their RV64
  // W forms -- but no bit-manipulation, crypto or czero.  Those share the OP/OP-IMM opcodes and are
  // told apart here by funct7 (funct6 for an RV64 shift immediate), so a dependent bitmanip operation
  // simply does not pair.  An encoding admitted here that turns out illegal is still caught by c2 and
  // replayed, as for any slot 1.  Every compressed ALU format instrfields recognizes is a base
  // operation.  LUI and AUIPC read no register: their raw rs fields can match Rd0F only by accident,
  // so admitting them only removes false dependencies -- ieu.sv clears the late bits for an operand
  // that is not a register, and they execute in Execute as usual.
  always_comb
    if (~&Instr1F[1:0]) Slot1LateOKF = ~|MemRW1F & ~IsBranch1F;
    else case (Op1F)
      LUI, AUIPC: Slot1LateOKF = 1'b1;
      OP_IMM:     case (Funct3_1F)
                    3'b001:  Slot1LateOKF = (Instr1F[31:26] == 6'b000000);                                    // slli
                    3'b101:  Slot1LateOKF = (Instr1F[31:26] == 6'b000000) | (Instr1F[31:26] == 6'b010000);   // srli, srai
                    default: Slot1LateOKF = 1'b1;                                                             // addi, slti(u), xori, ori, andi
                  endcase
      OP_IMM_32:  case (Funct3_1F)
                    3'b000:  Slot1LateOKF = (P.XLEN == 64);                                                   // addiw
                    3'b001:  Slot1LateOKF = (P.XLEN == 64) & (Funct7_1F == 7'b0000000);                      // slliw
                    3'b101:  Slot1LateOKF = (P.XLEN == 64) & ((Funct7_1F == 7'b0000000) | (Funct7_1F == 7'b0100000)); // srliw, sraiw
                    default: Slot1LateOKF = 1'b0;
                  endcase
      // add, sll, slt, sltu, xor, srl, or, and at funct7 = 0; sub and sra at 0100000
      OP:         Slot1LateOKF = (Funct7_1F == 7'b0000000) |
                                 ((Funct7_1F == 7'b0100000) & ((Funct3_1F == 3'b000) | (Funct3_1F == 3'b101)));
      // addw, sllw, srlw at funct7 = 0; subw, sraw at 0100000
      OP_32:      Slot1LateOKF = (P.XLEN == 64) &
                                 (((Funct7_1F == 7'b0000000) & ((Funct3_1F == 3'b000) | (Funct3_1F == 3'b001) | (Funct3_1F == 3'b101))) |
                                  ((Funct7_1F == 7'b0100000) & ((Funct3_1F == 3'b000) | (Funct3_1F == 3'b101))));
      default:    Slot1LateOKF = 1'b0;
    endcase

  // Both slots may write the same register.  Slot 1 is the later instruction, so its value must win
  // everywhere: regfile.sv writes port 6 second, and controller.sv's forwarding prefers slot 1 over slot
  // 0 within a stage.  Every stall is keyed on each slot's own destination, so a younger slot 1 that
  // overwrites a slow slot 0 result (a load, say) can only cost an unneeded stall, never a stale value.
  //
  // A late pair that also shares its destination is still refused, for performance rather than
  // correctness.  Such pairs are almost always the first two links of a chain (addi a5,a5,1; slli
  // a5,a5,2; ...) whose next instruction reads the late result at once and stalls for it.  Issuing the
  // first link alone leaves the late ALU free for the second link, which gets the same work done one
  // instruction further on.  Admitting them cost rv64gc CoreMark 0.3%.
  assign DependencyOKF = ~SrcDepF | (Slot0EarlyF & Slot1LateOKF & ~SameRdF);

  // Which of slot 1's operands are slot 0's result.  These are the same fields the pairing decision
  // used, so a dependency that permitted the pair is always marked; ieu.sv clears a bit whose operand
  // is an immediate or the PC rather than a register.
  assign LateOK2F = Slot1LateOKF;
  assign LateA2F = (Rd0F != 5'b0) & (Rs1_1F == Rd0F);
  assign LateB2F = (Rd0F != 5'b0) & (Rs2_1F == Rd0F);

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
                   DependencyOKF & OneMemOpF & OneMduOpF;

endmodule
