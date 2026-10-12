///////////////////////////////////////////
// testbench.sv
//
// Written: David_Harris@hmc.edu 9 January 2021
//// Modified by sanarayanan@hmc.edu, May 2025
// Modified:
//
// Purpose: Wally Testbench and helper modules
//          Applies test programs from the riscv-arch-test and other custom tests
//
// A component of the Wally configurable RISC-V project.
//
// Copyright (C) 2021 Harvey Mudd College & Oklahoma State University
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

`include "config.vh"
`include "tests.vh"
`include "BranchPredictorType.vh"

`ifdef USE_IMPERAS_DV
    `include "idv/idv.svh"
`endif

import cvw::*;

module testbench;
  /* verilator lint_off WIDTHTRUNC */
  /* verilator lint_off WIDTHEXPAND */
  parameter DEBUG=0;
  parameter PrintHPMCounters=0;
  parameter BPRED_LOGGER=0;
  parameter I_CACHE_ADDR_LOGGER=0;
  parameter D_CACHE_ADDR_LOGGER=0;
  parameter RVVI_SYNTH_SUPPORTED=0;
  parameter MAKE_VCD=0;

  // TREK Requires a license for the Breker tool. See tests/breker/README.md for details
  `ifdef USE_TREK_DV
    event trek_start;
    always @(testbench.trek_start) begin
      trek_uvm_pkg::trek_uvm_events::do_backdoor_init();
    end
  `endif

  `ifdef USE_IMPERAS_DV
    import idvPkg::*;
    import rvviApiPkg::*;
    import idvApiPkg::*;
    `define ENABLE_RVVI_TRACE
  `endif

  `ifdef FCOV
    `define ENABLE_RVVI_TRACE
  `endif

  `ifdef VERILATOR
      import "DPI-C" function string getenvval(input string env_name);
      string       RISCV_DIR = getenvval("RISCV");
      string       WALLY_DIR = getenvval("WALLY"); // ~/cvw typical
  `elsif VCS
      import "DPI-C" function string getenv(input string env_name);
      string       RISCV_DIR = getenv("RISCV");
      string       WALLY_DIR = getenv("WALLY");
  `else
      string       RISCV_DIR = "$RISCV";
      string       WALLY_DIR = "$WALLY";
  `endif

  `include "parameter-defs.vh"

  logic        clk;
  logic        reset_ext, reset;
  logic        ResetMem;

  // Variables that can be overwritten with $value$plusargs at start of simulation
  string       TEST, ElfFile, sim_log_prefix;
  integer      INSTR_LIMIT;
  string       UART_LOG_FILE;
  integer      UART_LOG;

  // DUT signals
  logic [P.AHBW-1:0]    HRDATAEXT;
  logic                 HREADYEXT, HRESPEXT;
  logic [P.PA_BITS-1:0] HADDR;
  logic [P.AHBW-1:0]    HWDATA;
  logic [P.XLEN/8-1:0]  HWSTRB;
  logic                 HWRITE;
  logic [2:0]           HSIZE;
  logic [2:0]           HBURST;
  logic [3:0]           HPROT;
  logic [1:0]           HTRANS;
  logic                 HMASTLOCK;
  logic                 HCLK, HRESETn;

  logic [31:0] GPIOIN, GPIOOUT, GPIOEN;
  logic        UARTSin, UARTSout;
  logic        SPIIn, SPIOut;
  logic [3:0]  SPICS;
  logic        SPICLK;
  logic        SDCCmd;
  logic        SDCIn;
  logic [3:0]  SDCCS;
  logic        SDCCLK;
  logic [3:0]  PWMGPIO;

  logic        HREADY;
  logic        HSELEXT;


  string  ProgramAddrMapFile, ProgramLabelMapFile;
  integer ProgramAddrLabelArray [string];

  int test, i, errors, totalerrors;

  string outputfile;
  integer outputFilePointer;

  string tests[];
  logic DCacheFlushDone, DCacheFlushStart;
  logic riscofTest;
  logic Validate;
  logic SelectTest;
  logic TestComplete;
  logic PrevPCZero;
  logic RVVIStall;

  integer elfFD;
  byte header[0:4];
  byte readBytes;

  initial begin
    // look for arguments passed to simulation, or use defaults
    if (!$value$plusargs("TEST=%s", TEST))
      TEST = "none";
    if (!$value$plusargs("ElfFile=%s", ElfFile))
      ElfFile = "none";
    if (!$value$plusargs("INSTR_LIMIT=%d", INSTR_LIMIT))
      INSTR_LIMIT = 0;
    // Check if sim_log_prefix is passed as a command-line argument
    if (!$value$plusargs("sim_log_prefix=%s", sim_log_prefix)) begin
        sim_log_prefix = "";  // Assign default value if not passed
    end
    if (!$value$plusargs("UART_LOG=%d", UART_LOG))
      UART_LOG = (TEST == "buildroot"); // Default to true for buildroot test, false otherwise
    if (!$value$plusargs("UART_LOG_FILE=%s", UART_LOG_FILE))
      UART_LOG_FILE = {"logs/", TEST, "_uart.out"};
    //$display("TEST = %s ElfFile = %s", TEST, ElfFile);

    if (ElfFile != "none") begin // If Elf File passed in, check its bit width
      elfFD = $fopen(ElfFile, "rb");
      readBytes = $fread(header, elfFD);
      $fclose(elfFD);

      // If DUT and elf bit width misaligned, exit and return error message
      if (header[4] == 1 & integer'(P.XLEN) == 64) begin
        $display("Error: You can not run a 32 bit elf on a 64 bit DUT");
        $finish;
      end else if (header[4] == 2 & integer'(P.XLEN) == 32) begin
        $display("Error: You can not run a 64 bit elf on a 32 bit DUT");
        $finish;
      end
    end

    // pick tests based on modes supported
    //tests = '{};
    if (P.XLEN == 64) begin // RV64
      case (TEST)
        "arch64i":                                tests = arch64i;
        "arch64priv":                             tests = arch64priv;
        "arch64c":      if (P.ZCA_SUPPORTED)
                          if (P.ZICSR_SUPPORTED)
                            if (P.ZCD_SUPPORTED)  tests = {arch64c, arch64cpriv, arch64zcd};
                            else                  tests = {arch64c, arch64cpriv};
                          else                    tests = {arch64c};
        "arch64m":      if (P.M_SUPPORTED)        tests = arch64m;
        "arch64a_amo":      if (P.ZAAMO_SUPPORTED)        tests = arch64a_amo;
        "arch64f":      if (P.F_SUPPORTED)        tests = arch64f;
        "arch64d":      if (P.D_SUPPORTED)        tests = arch64d;
        "arch64f_fma":  if (P.F_SUPPORTED)        tests = arch64f_fma;
        "arch64d_fma":  if (P.D_SUPPORTED)        tests = arch64d_fma;
        "arch64f_divsqrt":  if (P.F_SUPPORTED)        tests = arch64f_divsqrt;
        "arch64d_divsqrt":  if (P.D_SUPPORTED)        tests = arch64d_divsqrt;
        "arch64zifencei":  if (P.ZIFENCEI_SUPPORTED) tests = arch64zifencei;
        "arch64zicond":  if (P.ZICOND_SUPPORTED)  tests = arch64zicond;
        "wally64q":     if (P.Q_SUPPORTED)        tests = wally64q;
        "wally64a_lrsc":     if (P.ZALRSC_SUPPORTED)        tests = wally64a_lrsc;
        "custom":                                 tests = custom;
        "wally64priv":                            tests = wally64priv;
        "wally64periph":                          tests = wally64periph;
        "coremark":                               tests = coremark;
        "fpga":                                   tests = fpga;
        "ahb64" :                                 tests = ahb64;
        "coverage64gc" :                          tests = coverage64gc;
        "arch64zba":     if (P.ZBA_SUPPORTED)     tests = arch64zba;
        "arch64zbb":     if (P.ZBB_SUPPORTED)     tests = arch64zbb;
        "arch64zbc":     if (P.ZBC_SUPPORTED)     tests = arch64zbc;
        "arch64zbs":     if (P.ZBS_SUPPORTED)     tests = arch64zbs;
        "arch64zicboz":  if (P.ZICBOZ_SUPPORTED)  tests = arch64zicboz;
        "arch64zcb":     if (P.ZCB_SUPPORTED)     tests = arch64zcb;
        "arch64zfh":     if (P.ZFH_SUPPORTED)
                           if (P.D_SUPPORTED)     tests = {arch64zfh, arch64zfh_d};
                           else                   tests = arch64zfh;
        "arch64zfh_fma": if (P.ZFH_SUPPORTED)     tests = arch64zfh_fma;
        "arch64zfh_divsqrt":     if (P.ZFH_SUPPORTED)     tests = arch64zfh_divsqrt;
        "arch64zfaf":    if (P.ZFA_SUPPORTED)     tests = arch64zfaf;
        "arch64zfad":    if (P.ZFA_SUPPORTED & P.D_SUPPORTED)  tests = arch64zfad;
        "buildroot":                              tests = buildroot;
        "arch64zbkb":    if (P.ZBKB_SUPPORTED)    tests = arch64zbkb;
        "arch64zbkc":    if (P.ZBKC_SUPPORTED)    tests = arch64zbkc;
        "arch64zbkx":    if (P.ZBKX_SUPPORTED)    tests = arch64zbkx;
        "arch64zknd":    if (P.ZKND_SUPPORTED)    tests = arch64zknd;
        "arch64zkne":    if (P.ZKNE_SUPPORTED)    tests = arch64zkne;
        "arch64zknh":    if (P.ZKNH_SUPPORTED)    tests = arch64zknh;
        "arch64pmp":     if (P.PMP_ENTRIES > 0)   tests = arch64pmp;
        "arch64vm_sv39": if (P.SV39_SUPPORTED)    tests = arch64vm_sv39;
        "arch64vm_sv48": if (P.SV48_SUPPORTED)    tests = arch64vm_sv48;
        "arch64vm_sv48_a": if (P.SV48_SUPPORTED)    tests = arch64vm_sv48_a;
        "arch64vm_sv48_b": if (P.SV48_SUPPORTED)    tests = arch64vm_sv48_b;
        "arch64vm_sv39_isolate":     if (P.SV39_SUPPORTED) tests = arch64vm_sv39_isolate;
        "arch64vm_sv48_mxr_isolate": if (P.SV48_SUPPORTED) tests = arch64vm_sv48_mxr_isolate;
        "arch64vm_sv57": if (P.SV57_SUPPORTED)    tests = arch64vm_sv57;
      endcase
    end else begin // RV32
      case (TEST)
        "arch32e":                                tests = arch32e;
        "arch32i":                                tests = arch32i;
        "arch32priv":                             tests = arch32priv;
        "arch32c":      if (P.C_SUPPORTED)
                          if (P.ZICSR_SUPPORTED)
                            if (P.ZCF_SUPPORTED)
                              if (P.ZCD_SUPPORTED)  tests = {arch32c, arch32cpriv, arch32zcf, arch32zcd};
                              else                tests = {arch32c, arch32cpriv, arch32zcf};
                            else                  tests = {arch32c, arch32cpriv};
                          else                    tests = {arch32c};
        "arch32m":      if (P.M_SUPPORTED)        tests = arch32m;
        "arch32a_amo":      if (P.ZAAMO_SUPPORTED)  tests = arch32a_amo;
        "arch32f":      if (P.F_SUPPORTED)        tests = arch32f;
        "arch32d":      if (P.D_SUPPORTED)        tests = arch32d;
        "arch32f_fma":  if (P.F_SUPPORTED)        tests = arch32f_fma;
        "arch32d_fma":  if (P.D_SUPPORTED)        tests = arch32d_fma;
        "arch32f_divsqrt":  if (P.F_SUPPORTED)        tests = arch32f_divsqrt;
        "arch32d_divsqrt":  if (P.D_SUPPORTED)        tests = arch32d_divsqrt;
        "arch32zifencei":     if (P.ZIFENCEI_SUPPORTED) tests = arch32zifencei;
        "arch32zicond":  if (P.ZICOND_SUPPORTED)  tests = arch32zicond;
        "wally32a_lrsc":     if (P.ZALRSC_SUPPORTED)        tests = wally32a_lrsc;
        "wally32priv":                            tests = wally32priv;
        "wally32periph":                          tests = wally32periph;
        "wally32periph_imc":                      tests = wally32periph_imc;
        "ahb32" :                                 tests = ahb32;
        "embench":                                tests = embench;
        "coremark":                               tests = coremark;
        "arch32zba":     if (P.ZBA_SUPPORTED)     tests = arch32zba;
        "arch32zbb":     if (P.ZBB_SUPPORTED)     tests = arch32zbb;
        "arch32zbc":     if (P.ZBC_SUPPORTED)     tests = arch32zbc;
        "arch32zbs":     if (P.ZBS_SUPPORTED)     tests = arch32zbs;
        "arch32zicboz":  if (P.ZICBOZ_SUPPORTED)  tests = arch32zicboz;
        "arch32zcb":     if (P.ZCB_SUPPORTED)     tests = arch32zcb;
        "arch32zfh":     if (P.ZFH_SUPPORTED)
                           if (P.D_SUPPORTED)     tests = {arch32zfh, arch32zfh_d};
                           else                   tests = arch32zfh;
        "arch32zfh_fma": if (P.ZFH_SUPPORTED)     tests = arch32zfh_fma;
        "arch32zfh_divsqrt":     if (P.ZFH_SUPPORTED)     tests = arch32zfh_divsqrt;
        "arch32zfaf":    if (P.ZFA_SUPPORTED)     tests = arch32zfaf;
        "arch32zfad":    if (P.ZFA_SUPPORTED & P.D_SUPPORTED)  tests = arch32zfad;
        "arch32zbkb":    if (P.ZBKB_SUPPORTED)    tests = arch32zbkb;
        "arch32zbkc":    if (P.ZBKC_SUPPORTED)    tests = arch32zbkc;
        "arch32zbkx":    if (P.ZBKX_SUPPORTED)    tests = arch32zbkx;
        "arch32zknd":    if (P.ZKND_SUPPORTED)    tests = arch32zknd;
        "arch32zkne":    if (P.ZKNE_SUPPORTED)    tests = arch32zkne;
        "arch32zknh":    if (P.ZKNH_SUPPORTED)    tests = arch32zknh;
        "arch32pmp":     if (P.PMP_ENTRIES > 0)   tests = arch32pmp;
        "arch32vm_sv32": if (P.SV32_SUPPORTED)    tests = arch32vm_sv32;
      endcase
    end
    if (tests.size() == 0 & ElfFile == "none") begin
      $display("TEST %s not supported in this configuration", TEST);
      $finish;
    end
    if (MAKE_VCD) begin
      $dumpfile("testbench.vcd");
      $dumpvars;
    end
  end // initial begin

  // Model the testbench as an fsm.
  // Do this in parts so it easier to verify
  // part 1: build a version which echos the same behavior as the below code, but does not drive anything
  // part 2: drive some of the controls
  // part 3: drive all logic and remove old initial and always @ negedge clk block

  typedef enum logic [3:0]{STATE_TESTBENCH_RESET,
                           STATE_INIT_TEST,
                           STATE_RESET_MEMORIES,
                           STATE_RESET_MEMORIES2,
                           STATE_LOAD_MEMORIES,
                           STATE_RESET_TEST,
                           STATE_RUN_TEST,
                           STATE_COPY_RAM,
                           STATE_CHECK_TEST,
                           STATE_CHECK_TEST_WAIT,
                           STATE_VALIDATE,
                           STATE_INCR_TEST} statetype;
  statetype CurrState, NextState;
  logic        TestBenchReset;
  logic [2:0]  ResetCount, ResetThreshold;
  logic        LoadMem;
  logic        ResetCntEn;
  logic        ResetCntRst;
  logic        CopyRAM;

  string  signame, elffilename, memfilename, bootmemfilename, uartoutfilename, pathname;
  integer begin_signature_addr, end_signature_addr, signature_size, selfcheck_record_addr;
  integer uartoutfile;


  assign ResetThreshold = 3'd5;

  initial begin
    TestBenchReset = 1'b1;
    # 100;
    TestBenchReset = 1'b0;
  end

  always_ff @(posedge clk)
    if (TestBenchReset) CurrState <= STATE_TESTBENCH_RESET;
    else CurrState <= NextState;

  // fsm next state logic
  always_comb begin
    // riscof tests have a different signature, tests[0] == "0" refers to RiscvArchTests
    // and tests[0] == "1" refers to WallyRiscvArchTests
    riscofTest = tests[0] == "0" | tests[0] == "1";
    pathname = tvpaths[tests[0].atoi()];

    case(CurrState)
      STATE_TESTBENCH_RESET:                      NextState = STATE_INIT_TEST;
      STATE_INIT_TEST:                            NextState = STATE_RESET_MEMORIES;
      STATE_RESET_MEMORIES:                       NextState = STATE_RESET_MEMORIES2;
      STATE_RESET_MEMORIES2:                      NextState = STATE_LOAD_MEMORIES;  // Give the reset enough time to ensure the bus is reset before loading the memories.
      STATE_LOAD_MEMORIES:                        NextState = STATE_RESET_TEST;
      STATE_RESET_TEST:      if(ResetCount < ResetThreshold) NextState = STATE_RESET_TEST;
                             else                 NextState = STATE_RUN_TEST;
      STATE_RUN_TEST:        if(TestComplete)     NextState = STATE_COPY_RAM;
                             else                 NextState = STATE_RUN_TEST;
      STATE_COPY_RAM:                             NextState = STATE_CHECK_TEST;
      STATE_CHECK_TEST:      if (DCacheFlushDone) NextState = STATE_VALIDATE;
                             else                 NextState = STATE_CHECK_TEST_WAIT;
      STATE_CHECK_TEST_WAIT: if(DCacheFlushDone)  NextState = STATE_VALIDATE;
                             else                 NextState = STATE_CHECK_TEST_WAIT;
      STATE_VALIDATE:                             NextState = STATE_INIT_TEST;
      STATE_INCR_TEST:                            NextState = STATE_INIT_TEST;
      default:                                    NextState = STATE_TESTBENCH_RESET;
    endcase
  end // always_comb
  // fsm output control logic
  assign reset_ext = CurrState == STATE_TESTBENCH_RESET | CurrState == STATE_INIT_TEST |
                     CurrState == STATE_RESET_MEMORIES | CurrState == STATE_RESET_MEMORIES2 |
                     CurrState == STATE_LOAD_MEMORIES | CurrState ==STATE_RESET_TEST;
  // this initialization is very expensive, only do it for coremark.
  assign ResetMem = (CurrState == STATE_RESET_MEMORIES | CurrState == STATE_RESET_MEMORIES2) & TEST == "coremark";
  assign LoadMem = CurrState == STATE_LOAD_MEMORIES;
  assign ResetCntRst = CurrState == STATE_INIT_TEST;
  assign ResetCntEn = CurrState == STATE_RESET_TEST;
  assign Validate = CurrState == STATE_VALIDATE;
  assign SelectTest = CurrState == STATE_INIT_TEST;
  assign CopyRAM = TestComplete & CurrState == STATE_RUN_TEST;
  assign DCacheFlushStart = CurrState == STATE_COPY_RAM;

  // fsm reset counter
  counter #(3) RstCounter(clk, ResetCntRst, ResetCntEn, ResetCount);

  ////////////////////////////////////////////////////////////////////////////////
  // Find the test vector files and populate the PC to function label converter
  ////////////////////////////////////////////////////////////////////////////////
  logic [P.XLEN-1:0] testadr;

  //VCS ignores the dynamic types while processing the implicit sensitivity lists of always @*, always_comb, and always_latch
  //procedural blocks. VCS supports the dynamic types in the implicit sensitivity list of always @* block as specified in the Section 9.2 of the IEEE Standard SystemVerilog Specification 1800-2012.
  //To support memory load and dump task verbosity: flag : -diag sys_task_mem
  always @(*) begin
    begin_signature_addr = ProgramAddrLabelArray["begin_signature"];
    end_signature_addr = ProgramAddrLabelArray["sig_end_canary"];
    signature_size = end_signature_addr - begin_signature_addr;
    selfcheck_record_addr = ProgramAddrLabelArray["selfcheck_record"]; // nonzero for self-checking tests
  end
  logic EcallFaultM;
  if (P.ZICSR_SUPPORTED)
    assign EcallFaultM = dut.core.priv.priv.EcallFaultM;
  else
    assign EcallFaultM = 0;

  always @(posedge clk) begin
    ////////////////////////////////////////////////////////////////////////////////
    // Verify the test ran correctly by checking the memory against a known signature.
    ////////////////////////////////////////////////////////////////////////////////
    if(TestBenchReset) test = 1;
    if (P.ZICSR_SUPPORTED & TEST == "coremark")
      if (EcallFaultM) begin
        $display("Benchmark: coremark is done.");
        $finish;   // not $stop: $stop skips final blocks, so the end-of-run statistics never print
      end
    if(SelectTest) begin
      if (riscofTest) begin
        memfilename = {pathname, tests[test], "/ref/ref.elf.memfile"};
        elffilename = {pathname, tests[test], "ref/ref.elf"};
        ProgramAddrMapFile = {pathname, tests[test], "/ref/ref.elf.objdump.addr"};
        ProgramLabelMapFile = {pathname, tests[test], "/ref/ref.elf.objdump.lab"};
      end else if(TEST == "buildroot") begin
        memfilename = {RISCV_DIR, "/linux-testvectors/ram.bin"};
        elffilename = "buildroot";
        bootmemfilename = {RISCV_DIR, "/linux-testvectors/bootmem.bin"};
        ProgramAddrMapFile = {RISCV_DIR, "/buildroot/output/images/disassembly/vmlinux.objdump.addr"};
        ProgramLabelMapFile = {RISCV_DIR, "/buildroot/output/images/disassembly/vmlinux.objdump.lab"};
      end else if(TEST == "fpga") begin
        bootmemfilename = {WALLY_DIR, "/fpga/src/boot.mem"};
        memfilename = {WALLY_DIR, "/fpga/src/data.mem"};
        ProgramAddrMapFile = {WALLY_DIR, "/fpga/zsbl/bin/boot.objdump.addr"};
        ProgramLabelMapFile = {WALLY_DIR, "/fpga/zsbl/bin/boot.objdump.lab"};
      end else if(ElfFile != "none") begin
        elffilename = ElfFile;
        memfilename = {ElfFile, ".memfile"};
        ProgramAddrMapFile = {ElfFile, ".objdump.addr"};
        ProgramLabelMapFile = {ElfFile, ".objdump.lab"};
      end else begin
        elffilename = {pathname, tests[test], ".elf"};
        memfilename = {pathname, tests[test], ".elf.memfile"};
        ProgramAddrMapFile = {pathname, tests[test], ".elf.objdump.addr"};
        ProgramLabelMapFile = {pathname, tests[test], ".elf.objdump.lab"};
      end
      // declare memory labels that interest us, the updateProgramAddrLabelArray task will find
      // the addr of each label and fill the array. To expand, add more elements to this array
      // and initialize them to zero (also initialize them to zero at the start of the next test)
      updateProgramAddrLabelArray(ProgramAddrMapFile, ProgramLabelMapFile, memfilename, WALLY_DIR, ProgramAddrLabelArray);
      // Open UART log file if enabled (buildroot defaults to on, override with +UART_LOG=1)
      if (UART_LOG) begin
        uartoutfilename = UART_LOG_FILE;
        uartoutfile = $fopen(uartoutfilename, "w");
      end else
        uartoutfile = 0;
    end
    if(Validate) begin
      if (PrevPCZero) totalerrors = totalerrors + 1; //  error if PC is stuck at zero
      if (uartoutfile)
        $fclose(uartoutfile);
      if (TEST == "embench") begin
        // Writes contents of begin_signature to .sim.output file
        // this contains instret and cycles for start and end of test run, used by embench
        // python speed script to calculate embench speed score.
        // also, begin_signature contains the results of the self checking mechanism,
        // which will be read by the python script for error checking
        $display("Embench Benchmark: %s is done.", tests[test]);
        if (riscofTest) outputfile = {pathname, tests[test], "/ref/ref.sim.output"};
        else outputfile = {pathname, tests[test], ".sim.output"};
        outputFilePointer = $fopen(outputfile, "w");
        i = 0;
        testadr = ($unsigned(begin_signature_addr))/(P.XLEN/8);
        while ($unsigned(i) < $unsigned(5'd5)) begin
          $fdisplayh(outputFilePointer, DCacheFlushFSM.ShadowRAM[testadr+i]);
          i = i + 1;
        end
        $fclose(outputFilePointer);
        $display("Embench Benchmark: created output file: %s", outputfile);
      end else if (TEST == "coverage64gc") begin
        $display("%s ran. Coverage tests don't get checked", tests[test]);
      end else if (ElfFile != "none") begin
        `ifdef USE_TREK_DV
          $display("Breker test is done.");
        `elsif FCOV
          $display("Functional coverage test complete.");
        `else
          if (selfcheck_record_addr != 0) CheckSelfCheck(ElfFile, selfcheck_record_addr, errors);
          else $display("Single Elf file tests are not signatured verified.");
        `endif
`ifdef QUESTA
        $stop;  // if this is changed to $finish for Questa, wally-batch.do does not go to the next step to run coverage, and wally.do terminates without allowing GUI debug
`else
        $finish;
`endif
      end else begin
        // self-checking tests record their own result; for other tests, read .signature.output file and compare to check for errors
        // clear signature to prevent contamination from previous tests
        if (!begin_signature_addr)
          $display("begin_signature addr not found in %s", ProgramLabelMapFile);
        else if (TEST != "embench") begin
          if (selfcheck_record_addr != 0) CheckSelfCheck(tests[test], selfcheck_record_addr, errors);
          else CheckSignature(pathname, tests[test], riscofTest, begin_signature_addr, errors);
          if(errors > 0) totalerrors = totalerrors + 1;
        end
      end
      test = test + 1;
      if (test == tests.size()) begin
        if (totalerrors == 0) $display("SUCCESS! All tests ran without failures.");
        else $display("FAIL: %d test programs had errors", totalerrors);
`ifdef QUESTA
        $stop;  // if this is changed to $finish for Questa, wally-batch.do does not go to the next step to run coverage, and wally.do terminates without allowing GUI debug
`else
        $finish;
`endif
      end
    end
  end


  ////////////////////////////////////////////////////////////////////////////////
  // load memories with program image
  ////////////////////////////////////////////////////////////////////////////////

  integer ShadowIndex;
  integer LogXLEN;
  integer StartIndex;
  integer EndIndex;
  integer BaseIndex;
  integer memFile, uncoreMemFile;
  integer readResult;
  if (P.SDC_SUPPORTED) begin
    always @(posedge clk) begin
      if (LoadMem) begin
        string romfilename, sdcfilename;
        romfilename = {"../tests/custom/fpga-test-sdc/bin/fpga-test-sdc.memfile"};
        sdcfilename = {"../testbench/sdc/ramdisk2.hex"};
        //$readmemh(romfilename, dut.uncoregen.uncore.bootrom.bootrom.memory.ROM);
        //$readmemh(sdcfilename, sdcard.sdcard.FLASHmem);
        // shorten sdc timers for simulation
        //dut.uncoregen.uncore.sdc.SDC.LimitTimers = 1;
      end
    end
  end else if (P.IROM_SUPPORTED) begin
    always @(posedge clk) begin
      if (LoadMem) begin
        $readmemh(memfilename, dut.core.ifu.irom.irom.rom.ROM);
      end
    end
  end else if (P.BUS_SUPPORTED) begin : bus_supported
    always @(posedge clk) begin
      if (LoadMem) begin
        if (TEST == "buildroot") begin
          memFile = $fopen(bootmemfilename, "rb");
          if (memFile == 0) begin
            $display("Error: Could not open file %s", memfilename);
            $finish;
          end
          if (P.BOOTROM_SUPPORTED)
            readResult = $fread(dut.uncoregen.uncore.bootrom.bootrom.memory.ROM, memFile);
          else begin
            $display("Buildroot test requires BOOTROM_SUPPORTED");
            $finish;
          end
          $fclose(memFile);
          memFile = $fopen(memfilename, "rb");
          if (memFile == 0) begin
            $display("Error: Could not open file %s", memfilename);
            $finish;
          end
          readResult = $fread(dut.uncoregen.uncore.ram.ram.memory.ram.RAM, memFile);
          $fclose(memFile);
        end else if (TEST == "fpga") begin
          memFile = $fopen(bootmemfilename, "rb");
          if (memFile == 0) begin
            $display("Error: Could not open file %s", memfilename);
            $finish;
          end
          if (P.BOOTROM_SUPPORTED) begin
            readResult = $fread(dut.uncoregen.uncore.bootrom.bootrom.memory.ROM, memFile);
          end
          $fclose(memFile);
          memFile = $fopen(memfilename, "rb");
          if (memFile == 0) begin
            $display("Error: Could not open file %s", memfilename);
            $finish;
          end
          readResult = $fread(dut.uncoregen.uncore.ram.ram.memory.ram.RAM, memFile);
          $fclose(memFile);
        end else begin
          uncoreMemFile = $fopen(memfilename, "r");  // Is there a better way to test if a file exists?
          if (uncoreMemFile == 0) begin
            $display("Error: Could not open file %s", memfilename);
            $finish;
          end else begin
            $fclose(uncoreMemFile);
            $readmemh(memfilename, dut.uncoregen.uncore.ram.ram.memory.ram.RAM);
            `ifdef USE_TREK_DV
              -> trek_start;
              $display("starting Trek....");
            `endif
          end
        end
        if (TEST == "embench") $display("Read memfile %s", memfilename);
      end
      if (CopyRAM) begin
        LogXLEN = (1 + P.XLEN/32); // 2 for rv32 and 3 for rv64
        StartIndex = begin_signature_addr >> LogXLEN;
        EndIndex = (end_signature_addr >> LogXLEN) + 8;
        BaseIndex = P.UNCORE_RAM_BASE >> LogXLEN;
        for(ShadowIndex = StartIndex; ShadowIndex <= EndIndex; ShadowIndex++) begin
          testbench.DCacheFlushFSM.ShadowRAM[ShadowIndex] = dut.uncoregen.uncore.ram.ram.memory.ram.RAM[ShadowIndex - BaseIndex];
        end
        if (selfcheck_record_addr != 0) begin // also copy the result record of a self-checking test
          StartIndex = selfcheck_record_addr >> LogXLEN;
          EndIndex = StartIndex + 7;
          for(ShadowIndex = StartIndex; ShadowIndex <= EndIndex; ShadowIndex++) begin
            testbench.DCacheFlushFSM.ShadowRAM[ShadowIndex] = dut.uncoregen.uncore.ram.ram.memory.ram.RAM[ShadowIndex - BaseIndex];
          end
        end
      end
    end
  end
  if (P.DTIM_SUPPORTED) begin
    always @(posedge clk) begin
      if (LoadMem) begin
        $readmemh(memfilename, dut.core.lsu.dtim.dtim.ram.ram.RAM);
      end
      if (CopyRAM) begin
        LogXLEN = (1 + P.XLEN/32); // 2 for rv32 and 3 for rv64
        StartIndex = begin_signature_addr >> LogXLEN;
        EndIndex = (end_signature_addr >> LogXLEN) + 8;
        BaseIndex = P.UNCORE_RAM_BASE >> LogXLEN;
        for(ShadowIndex = StartIndex; ShadowIndex <= EndIndex; ShadowIndex++) begin
          testbench.DCacheFlushFSM.ShadowRAM[ShadowIndex] = dut.core.lsu.dtim.dtim.ram.ram.RAM[ShadowIndex - BaseIndex];
        end
        if (selfcheck_record_addr != 0) begin // also copy the result record of a self-checking test
          StartIndex = selfcheck_record_addr >> LogXLEN;
          EndIndex = StartIndex + 7;
          for(ShadowIndex = StartIndex; ShadowIndex <= EndIndex; ShadowIndex++) begin
            testbench.DCacheFlushFSM.ShadowRAM[ShadowIndex] = dut.core.lsu.dtim.dtim.ram.ram.RAM[ShadowIndex - BaseIndex];
          end
        end
      end
    end
  end

  integer adrindex;
  if (P.UNCORE_RAM_SUPPORTED)
    always @(posedge clk)
      if (ResetMem)  // program memory is sometimes reset (e.g. for CoreMark, which needs zeroed memory)
        for (adrindex=0; adrindex<(P.UNCORE_RAM_RANGE>>1+(P.XLEN/32)); adrindex = adrindex+1)
          dut.uncoregen.uncore.ram.ram.memory.ram.RAM[adrindex] = '0;

  ////////////////////////////////////////////////////////////////////////////////
  // Actual hardware
  ////////////////////////////////////////////////////////////////////////////////

  // instantiate device to be tested
  assign GPIOIN = '0;
  assign UARTSin = 1'b1;
  assign SPIIn = 1'b0;

  if(P.EXT_MEM_SUPPORTED) begin
    ram_ahb #(.P(P), .RANGE(P.EXT_MEM_RANGE))
    ram (.HCLK, .HRESETn, .HADDR, .HWRITE, .HTRANS, .HWDATA, .HSELRam(HSELEXT),
      .HREADRam(HRDATAEXT), .HREADYRam(HREADYEXT), .HRESPRam(HRESPEXT), .HREADY, .HWSTRB);
  end else begin
    assign HREADYEXT = 1'b1;
    assign {HRESPEXT, HRDATAEXT} = '0;
  end

  if(P.SDC_SUPPORTED) begin : sdcard
    // JP: Add back sd card when sd card AHB implementation done
    /* -----\/----- EXCLUDED -----\/-----
    sdModel sdcard
      (.sdClk(SDCCLK),
       .cmd(SDCCmd),
       .dat(SDCDat));

    assign SDCCmd = SDCCmdOE ? SDCCmdOut : 1'bz;
    assign SDCCmdIn = SDCCmd;
    assign SDCDat = sd_dat_reg_t ? sd_dat_reg_o : sd_dat_i;
    assign SDCDatIn = SDCDat;
    -----/\----- EXCLUDED -----/\----- */
  end else begin
    assign SDCIn = 1'b1;

  end

  wallypipelinedsoc  #(P) dut(.clk, .reset_ext, .reset, .ExternalStall(RVVIStall),
    .HRDATAEXT, .HREADYEXT, .HRESPEXT, .HSELEXT,
    .HCLK, .HRESETn, .HADDR, .HWDATA, .HWSTRB, .HWRITE, .HSIZE, .HBURST, .HPROT,
    .HTRANS, .HMASTLOCK, .HREADY, .TIMECLK(1'b0), .GPIOIN, .GPIOOUT, .GPIOEN,
    .UARTSin, .UARTSout, .SPIIn, .SPIOut, .SPICS, .SPICLK, .SDCIn, .SDCCmd, .SDCCS, .SDCCLK, .PWMGPIO);

  // generate clock to sequence tests
  always begin
    clk = 1'b1; # 5; clk = 1'b0; # 5;
  end

  if(RVVI_SYNTH_SUPPORTED) begin : rvvi_synth
    localparam MAX_CSRS = 5;
    localparam logic [31:0] RVVI_INIT_TIME_OUT = 32'd4;
    localparam logic [31:0] RVVI_PACKET_DELAY = 32'd2;

    logic [3:0]                                       mii_txd;
    logic                                             mii_tx_en, mii_tx_er;

    rvvitbwrapper #(P, MAX_CSRS, RVVI_INIT_TIME_OUT, RVVI_PACKET_DELAY)
    rvvitbwrapper(.clk, .reset, .RVVIStall, .mii_tx_clk(clk), .mii_txd, .mii_tx_en, .mii_tx_er,
                  .mii_rx_clk(clk), .mii_rxd('0), .mii_rx_dv('0), .mii_rx_er('0));
  end else begin
    assign RVVIStall = '0;
  end


  /*
  // Print key info  each cycle for debugging
  always @(posedge clk) begin
    #2;
    $display("PCM: %x  InstrM: %x (%5s) WriteDataM: %x  IEUResultM: %x",
         dut.core.PCM, dut.core.InstrM, InstrMName, dut.core.WriteDataM, dut.core.ieu.dp.IEUResultM);
  end
  */

  ////////////////////////////////////////////////////////////////////////////////
  // Support logic
  ////////////////////////////////////////////////////////////////////////////////

  // Duplicate copy of pipeline registers that are optimized out of some configurations
  logic [31:0] NextInstrE, InstrM;
  mux2    #(32)     FlushInstrMMux(dut.core.ifu.InstrE, dut.core.ifu.nop, dut.core.ifu.FlushM, NextInstrE);
  flopenr #(32)     InstrMReg(clk, reset, ~dut.core.ifu.StallM, NextInstrE, InstrM);

  // Track names of instructions
  string InstrFName, InstrDName, InstrEName, InstrMName, InstrWName;
  logic [31:0] InstrW;
  flopenr #(32)    InstrWReg(clk, reset, ~dut.core.ieu.dp.StallW,  InstrM, InstrW);
  instrTrackerTB #(P.XLEN) it(clk, reset, dut.core.ieu.dp.FlushE,
                dut.core.ifu.InstrRawF[31:0],
                dut.core.ifu.InstrD, dut.core.ifu.InstrE,
                InstrM,  InstrW,
                InstrFName, InstrDName, InstrEName, InstrMName, InstrWName);

  // Superscalar: the front end decodes two instruction slots but issues only slot 0.  Nothing downstream
  // consumes slot 1 yet, so check it here.  Whenever Decode advances and execution is sequential, the
  // instruction that just arrived in Decode (slot 0) must be exactly what the previous Decode held in
  // slot 1: same PC, same raw and decompressed bits, and the same decode results (source registers,
  // immediate, register-file reads, branch/jump/illegal flags).
  // Remove this block once slot 1 issues: slot 1 then no longer reappears as the next slot 0.
  // instrfields reads slot 1's register fields in Fetch from raw bytes, duplicating knowledge that
  // decompress.sv also holds.  Duplication is only acceptable if a divergence is detected rather
  // than silent -- a wrong Rd would let issue.sv permit a dependent pair, which corrupts data with
  // no fault to catch it.  So compare the Fetch-stage fields against the Decode-stage decompressed
  // instruction, which is the authority, every time slot 1 actually issues.
  if (P.ICACHE_SUPPORTED) begin : fieldscheck
    logic [4:0] RdF, Rs1F, Rs2F;
    logic [1:0] MemRWF;
    logic       RecF;
    integer     FieldChecks = 0;
    // These must mirror Instr2RawD's register exactly, which is a flopenl whose load ignores en:
    // on a simultaneous flush and stall the instruction becomes a nop while Issue2D, a flopenrc
    // whose clear requires en, keeps its old value.  Clearing RecF on flush suppresses the check for
    // a slot that Fetch never examined.
    always_ff @(posedge clk)
      if (reset | dut.core.ifu.FlushD) RecF <= 1'b0;
      else if (~dut.core.ifu.StallD) begin
        RdF  <= dut.core.ifu.issue.Rd1F;
        Rs1F <= dut.core.ifu.issue.Rs1_1F;
        Rs2F   <= dut.core.ifu.issue.Rs2_1F;
        MemRWF <= dut.core.ifu.issue.MemRW1F;
        RecF   <= dut.core.ifu.issue.Recognized1F;
      end
    always_ff @(posedge clk)
      // Only compare when the decompressor actually recognised the instruction.  For a reserved or
      // HINT encoding -- c.slli with a zero shift amount, c.lwsp with rd = x0 -- it emits a default
      // rather than a decode, so there is nothing meaningful to compare against.  Those encodings
      // are handled instead by IllegalComp2M, which replays slot 1 so it traps as an unpaired
      // instruction; instrfields deliberately does not try to enumerate the HINT rules.
      if (~reset & dut.core.ifu.Issue2D & RecF & ~dut.core.ifu.IllegalCompInstr2D) begin
        FieldChecks <= FieldChecks + 1;
        // The property that matters is one-sided.  Naming a register the instruction does not touch
        // only makes NoDependencyF refuse a pair it could have taken -- harmless.  *Missing* one
        // permits a dependent pair, which corrupts data silently.  So a nonzero register in the
        // decompressed form must be reported exactly; x0 means no dependency and anything is safe.
        // This also accommodates the HINT encodings, which decompress.sv turns into a nop (see its
        // line 156): instrfields names their registers, the nop touches none.
        if ((dut.core.ifu.Instr2D[11:7] !== 5'b0) & (RdF !== dut.core.ifu.Instr2D[11:7]))
          $fatal(1, "fieldscheck: Rd under-reported at PC %h: Fetch said x%0d, decompressed writes x%0d (raw %h)",
                 dut.core.ifu.Instr2PCD, RdF, dut.core.ifu.Instr2D[11:7], dut.core.ifu.Instr2RawD);
        // Only compare rs1 for formats that have one.  LUI, AUIPC and JAL are U- and J-type, so
        // their [19:15] bits are immediate, not a register number -- c.lui decompresses to lui and
        // those bits can be any value.  Likewise [24:20] is rs2 only for R-type, already gated below.
        if (~(dut.core.ifu.Instr2D[6:0] inside {7'b0110111, 7'b0010111, 7'b1101111})
            & (dut.core.ifu.Instr2D[19:15] !== 5'b0) & (Rs1F !== dut.core.ifu.Instr2D[19:15]))
          $fatal(1, "fieldscheck: Rs1 under-reported at PC %h: Fetch said x%0d, decompressed reads x%0d (raw %h)",
                 dut.core.ifu.Instr2PCD, Rs1F, dut.core.ifu.Instr2D[19:15], dut.core.ifu.Instr2RawD);
        // Rs2 only exists as a register in the R-type expansion; for an OP-IMM expansion those bits
        // are immediate, so comparing them would be meaningless.
        if ((dut.core.ifu.Instr2D[6:0] == 7'b0110011) & (dut.core.ifu.Instr2D[24:20] !== 5'b0)
            & (Rs2F !== dut.core.ifu.Instr2D[24:20]))
          $fatal(1, "fieldscheck: Rs2 under-reported at PC %h: Fetch said x%0d, decompressed reads x%0d (raw %h)",
                 dut.core.ifu.Instr2PCD, Rs2F, dut.core.ifu.Instr2D[24:20], dut.core.ifu.Instr2RawD);
        // Memory-ness matters as much as the registers: OneMemOpF is the only thing keeping two
        // accesses off the single load/store unit, so a compressed load that Fetch failed to
        // recognise as memory would let a pair through that the LSU cannot serve.
        if (MemRWF !== {dut.core.ifu.Instr2D[6:0] == 7'b0000011, dut.core.ifu.Instr2D[6:0] == 7'b0100011})
          $fatal(1, "fieldscheck: MemRW mismatch at PC %h: Fetch said %b, decompressed opcode %h (raw %h)",
                 dut.core.ifu.Instr2PCD, MemRWF, dut.core.ifu.Instr2D[6:0], dut.core.ifu.Instr2RawD);
      end
    final $display("fieldscheck: %0d slot-1 field extractions verified against the decompressor", FieldChecks);
  end

  if (P.ICACHE_SUPPORTED) begin : decode2check
    logic              AdvanceD;              // Decode is loading a new instruction this cycle
    logic              CheckPending;          // Decode advanced last cycle, so compare now
    logic              Instr2ValidPrev;
    logic [P.XLEN-1:0] Instr2PCPrev, ImmExt2Prev, R1_2Prev, R2_2Prev;
    logic [31:0]       Instr2RawPrev, Instr2Prev;
    logic [4:0]        Rs1_2Prev, Rs2_2Prev;
    logic [2:0]        ImmSrc2Prev;
    logic              Branch2Prev, Jump2Prev, Illegal2Prev;
    logic              CompressedNow, Slot0Squashed, R1Written, R2Written;
    integer            ChecksDone = 0;

    assign AdvanceD = ~reset & ~dut.core.ifu.StallD & ~dut.core.ifu.FlushD;
    always_ff @(posedge clk) begin
      if (AdvanceD) begin                                       // remember slot 1 as Decode is replaced
        Instr2ValidPrev <= dut.core.ifu.Instr2ValidD;
        Instr2PCPrev    <= dut.core.ifu.Instr2PCD;
        Instr2RawPrev   <= dut.core.ifu.Instr2RawD;
        Instr2Prev      <= dut.core.ifu.Instr2D;
        Rs1_2Prev       <= dut.core.ieu.Rs1_2D;
        Rs2_2Prev       <= dut.core.ieu.Rs2_2D;
        ImmSrc2Prev     <= dut.core.ieu.ImmSrc2D;
        ImmExt2Prev     <= dut.core.ieu.dp.ImmExt2D;
        R1_2Prev        <= dut.core.ieu.dp.R1_2D;
        R2_2Prev        <= dut.core.ieu.dp.R2_2D;
        Branch2Prev     <= dut.core.ieu.Branch2D;
        Jump2Prev       <= dut.core.ieu.Jump2D;
        Illegal2Prev    <= dut.core.ieu.IllegalBaseInstr2D;
        CheckPending    <= 1'b1;
      end else CheckPending <= 1'b0;
      // fence.i / I$ invalidate: memory may have changed, so the remembered slot 1 is stale
      if (dut.core.ifu.CSRWriteFenceM | dut.core.ifu.InvalidateICacheM) Instr2ValidPrev <= 1'b0;
    end

    assign CompressedNow = ~&dut.core.ifu.InstrRawD[1:0];
    assign Slot0Squashed = dut.core.ifu.IllegalIEUFPUInstrD;    // slot 0's controls are zeroed for an illegal instruction; slot 1's are not
    // a writeback in between the two reads changes the register file, so skip that operand
    assign R1Written = ((dut.core.ieu.RegWriteW  & (dut.core.ieu.RdW  == Rs1_2Prev)) |
                        (dut.core.ieu.RegWrite2W & (dut.core.ieu.Rd2W == Rs1_2Prev))) & (Rs1_2Prev != 0);
    assign R2Written = ((dut.core.ieu.RegWriteW  & (dut.core.ieu.RdW  == Rs2_2Prev)) |
                        (dut.core.ieu.RegWrite2W & (dut.core.ieu.Rd2W == Rs2_2Prev))) & (Rs2_2Prev != 0);

    always_ff @(posedge clk)
      if (CheckPending & Instr2ValidPrev & (dut.core.ifu.PCD == Instr2PCPrev)) begin
        ChecksDone <= ChecksDone + 1;
        if (CompressedNow ? (Instr2RawPrev[15:0] != dut.core.ifu.InstrRawD[15:0]) : (Instr2RawPrev != dut.core.ifu.InstrRawD))
          $fatal(1, "decode2check: raw mismatch at PC %h: slot 1 %h, slot 0 %h", dut.core.ifu.PCD, Instr2RawPrev, dut.core.ifu.InstrRawD);
        if (Instr2Prev != dut.core.ifu.InstrD)
          $fatal(1, "decode2check: decompressed mismatch at PC %h: slot 1 %h, slot 0 %h", dut.core.ifu.PCD, Instr2Prev, dut.core.ifu.InstrD);
        if ({Rs1_2Prev, Rs2_2Prev} != {dut.core.ieu.Rs1D, dut.core.ieu.Rs2D})
          $fatal(1, "decode2check: Rs1/Rs2 mismatch at PC %h: slot 1 x%0d/x%0d, slot 0 x%0d/x%0d", dut.core.ifu.PCD, Rs1_2Prev, Rs2_2Prev, dut.core.ieu.Rs1D, dut.core.ieu.Rs2D);
        if ({Branch2Prev, Jump2Prev, Illegal2Prev} != {dut.core.ieu.BranchD, dut.core.ieu.JumpD, dut.core.ieu.IllegalBaseInstrD})
          $fatal(1, "decode2check: branch/jump/illegal mismatch at PC %h: slot 1 %b, slot 0 %b", dut.core.ifu.PCD, {Branch2Prev, Jump2Prev, Illegal2Prev}, {dut.core.ieu.BranchD, dut.core.ieu.JumpD, dut.core.ieu.IllegalBaseInstrD});
        if (~Slot0Squashed & ({ImmSrc2Prev, ImmExt2Prev} != {dut.core.ieu.ImmSrcD, dut.core.ieu.dp.ImmExtD}))
          $fatal(1, "decode2check: immediate mismatch at PC %h: slot 1 src %b ext %h, slot 0 src %b ext %h", dut.core.ifu.PCD, ImmSrc2Prev, ImmExt2Prev, dut.core.ieu.ImmSrcD, dut.core.ieu.dp.ImmExtD);
        if (~R1Written & (R1_2Prev != dut.core.ieu.dp.R1D))
          $fatal(1, "decode2check: R1 mismatch at PC %h (x%0d): slot 1 %h, slot 0 %h", dut.core.ifu.PCD, Rs1_2Prev, R1_2Prev, dut.core.ieu.dp.R1D);
        if (~R2Written & (R2_2Prev != dut.core.ieu.dp.R2D))
          $fatal(1, "decode2check: R2 mismatch at PC %h (x%0d): slot 1 %h, slot 0 %h", dut.core.ifu.PCD, Rs2_2Prev, R2_2Prev, dut.core.ieu.dp.R2D);
      end

    // FP decode: slot 1's fctrl outputs and FP register-file reads must match slot 0's the next cycle
    if (P.F_SUPPORTED) begin : fpcheck
      logic [4:0]        FAdr1_2Prev, FAdr2_2Prev, FAdr3_2Prev;
      logic              XEn2Prev, YEn2Prev, ZEn2Prev, FIllegal2Prev;
      logic [P.FLEN-1:0] FRD1_2Prev, FRD2_2Prev, FRD3_2Prev;
      logic              F1Written, F2Written, F3Written;
      always_ff @(posedge clk)
        if (AdvanceD) begin
          {FAdr1_2Prev, FAdr2_2Prev, FAdr3_2Prev} <= {dut.core.fpu.fpu.Adr1_2D, dut.core.fpu.fpu.Adr2_2D, dut.core.fpu.fpu.Adr3_2D};
          {XEn2Prev, YEn2Prev, ZEn2Prev}          <= {dut.core.fpu.fpu.XEn2D, dut.core.fpu.fpu.YEn2D, dut.core.fpu.fpu.ZEn2D};
          FIllegal2Prev                           <= dut.core.fpu.fpu.IllegalFPUInstr2D;
          {FRD1_2Prev, FRD2_2Prev, FRD3_2Prev}    <= {dut.core.fpu.fpu.FRD1_2D, dut.core.fpu.fpu.FRD2_2D, dut.core.fpu.fpu.FRD3_2D};
        end
      // an FP writeback in between the two reads changes the register file, so skip that operand
      assign F1Written = dut.core.fpu.fpu.FRegWriteW & (dut.core.fpu.fpu.RdW == FAdr1_2Prev);
      assign F2Written = dut.core.fpu.fpu.FRegWriteW & (dut.core.fpu.fpu.RdW == FAdr2_2Prev);
      assign F3Written = dut.core.fpu.fpu.FRegWriteW & (dut.core.fpu.fpu.RdW == FAdr3_2Prev);
      always_ff @(posedge clk)
        if (CheckPending & Instr2ValidPrev & (dut.core.ifu.PCD == Instr2PCPrev)) begin
          if ({FAdr1_2Prev, FAdr2_2Prev, FAdr3_2Prev} != {dut.core.fpu.fpu.Adr1D, dut.core.fpu.fpu.Adr2D, dut.core.fpu.fpu.Adr3D})
            $fatal(1, "decode2check: FP Adr mismatch at PC %h: slot 1 f%0d/f%0d/f%0d, slot 0 f%0d/f%0d/f%0d", dut.core.ifu.PCD,
                   FAdr1_2Prev, FAdr2_2Prev, FAdr3_2Prev, dut.core.fpu.fpu.Adr1D, dut.core.fpu.fpu.Adr2D, dut.core.fpu.fpu.Adr3D);
          if ({XEn2Prev, YEn2Prev, ZEn2Prev, FIllegal2Prev} != {dut.core.fpu.fpu.XEnD, dut.core.fpu.fpu.YEnD, dut.core.fpu.fpu.ZEnD, dut.core.fpu.fpu.IllegalFPUInstrD})
            $fatal(1, "decode2check: FP XYZEn/illegal mismatch at PC %h: slot 1 %b, slot 0 %b", dut.core.ifu.PCD,
                   {XEn2Prev, YEn2Prev, ZEn2Prev, FIllegal2Prev}, {dut.core.fpu.fpu.XEnD, dut.core.fpu.fpu.YEnD, dut.core.fpu.fpu.ZEnD, dut.core.fpu.fpu.IllegalFPUInstrD});
          if (~F1Written & (FRD1_2Prev != dut.core.fpu.fpu.FRD1D))
            $fatal(1, "decode2check: FRD1 mismatch at PC %h (f%0d): slot 1 %h, slot 0 %h", dut.core.ifu.PCD, FAdr1_2Prev, FRD1_2Prev, dut.core.fpu.fpu.FRD1D);
          if (~F2Written & (FRD2_2Prev != dut.core.fpu.fpu.FRD2D))
            $fatal(1, "decode2check: FRD2 mismatch at PC %h (f%0d): slot 1 %h, slot 0 %h", dut.core.ifu.PCD, FAdr2_2Prev, FRD2_2Prev, dut.core.fpu.fpu.FRD2D);
          if (~F3Written & (FRD3_2Prev != dut.core.fpu.fpu.FRD3D))
            $fatal(1, "decode2check: FRD3 mismatch at PC %h (f%0d): slot 1 %h, slot 0 %h", dut.core.ifu.PCD, FAdr3_2Prev, FRD3_2Prev, dut.core.fpu.fpu.FRD3D);
        end
    end
    final $display("decode2check: %0d slot-1 decodes verified against slot 0", ChecksDone);
  end

  // Report how often the two slots actually issue together.  This is the headline number for the
  // superscalar work: it bounds the speedup the issue rules currently allow.
  if (P.ICACHE_SUPPORTED) begin : issuerate
    integer Retired = 0, RetiredPaired = 0;
    always_ff @(posedge clk)
      if (~reset & dut.core.ieu.c.InstrValidM & ~dut.core.ifu.StallW & ~dut.core.ifu.FlushW) begin
        Retired <= Retired + 1;
        // Not Issue2M alone: a replayed slot 1 is still in Memory but has not retired, and will be
        // counted when it runs again on its own.
        if (dut.core.ifu.Issue2M & ~dut.core.Replay2M) RetiredPaired <= RetiredPaired + 1;
      end
    final
      if (Retired > 0)
        $display("issuerate: %0d instructions retired, %0d of them paired (%0d%% of cycles dual-issued)",
                 Retired + RetiredPaired, RetiredPaired, (100 * RetiredPaired) / Retired);
  end

  // Cycles that issue nothing are either stalls or flushes.  Attribute them to a cause so a change
  // that pairs more instructions but runs slower can be diagnosed rather than guessed at.
  if (P.ICACHE_SUPPORTED) begin : overhead
    integer Cycles = 0, StallSlot0 = 0, StallSlot1 = 0, StallOther = 0, FlushMispredict = 0, Kills = 0;
    integer Replays = 0, Mem2 = 0;   // slot 1 memory operations, and how many had to be replayed
    integer LoadUse2 = 0, BundleStore = 0;  // the two bundle-level memory hazards, to confirm they fire
    integer Replay2Ld = 0, Replay2St = 0;   // which kind of slot 1 memory operation had to be replayed
    integer BrWithMem2 = 0;                 // taken branch in slot 0 alongside a memory operation in slot 1
    integer ReplayStoreLeak = 0;            // a replayed slot 1 store that the LSU was not told to cancel

    // Why a pair was refused.  Each counter records cycles where one term of Issue2F was the ONLY
    // thing holding the pair back, so it reads as the marginal headroom of fixing that term alone
    // rather than as a raw tally of a term that was one blocker among several.
    integer PairOpportunity = 0;            // slot 1 held a complete instruction but the pair was refused
    integer BlockBranch1 = 0;               // ... solely because slot 1 was a branch or jump
    integer BlockBranch1Aligned = 0;        // ... and the bundle was 8-byte aligned, so a block-indexed BTB would see it
    integer BlockCompressed = 0;            // ... solely because either slot was compressed
    integer BlockDep = 0;                   // ... solely because slot 1 depended on slot 0
    integer BlockSlot0 = 0;                 // ... solely because of slot 0 (predicted taken, jump, SYSTEM, fence)
    integer BlockOneMem = 0;                // ... solely because both slots wanted the load/store unit
    integer AnyCompressed = 0;              // ... compressed was A blocker, not necessarily the only one
    integer AnyDep = 0;                     // ... likewise for the dependency check
    integer BlockJump1 = 0;                 // ... solely because slot 1 was a jal/jalr (out of scope: the RAS)
    integer BlockMDU1 = 0;                  // ... solely because slot 1 was a multiply or divide (one MDU)
    integer BlockUnrec0 = 0;                // ... solely because SLOT 0's encoding is not in instrfields
    integer BlockUnrec1 = 0;                // ... solely because SLOT 1's is
    logic Cfi1F, Br1F, Jmp1F, Refused;
    assign Br1F    = (dut.core.ifu.issue.Op1F == 7'b1100011);   // branch
    assign Jmp1F   = (dut.core.ifu.issue.Op1F == 7'b1101111)    // jal
                   | (dut.core.ifu.issue.Op1F == 7'b1100111);   // jalr
    assign Cfi1F   = Br1F | Jmp1F;
    assign Refused = dut.core.ifu.issue.Instr2ValidF & ~dut.core.ifu.Issue2F;

    logic Slot1Stall;
    // The slot 1 hazard only costs anything when slot 0 was not already stalling for its own reasons
    assign Slot1Stall = dut.core.ieu.Issue2D & dut.core.ieu.StructuralStall2D
                        & ~dut.core.ieu.StructuralStall0D;
    always_ff @(posedge clk)
      if (~reset) begin
        Cycles <= Cycles + 1;
        if (dut.core.StallD) begin
          if (dut.core.ieu.StructuralStall0D) StallSlot0 <= StallSlot0 + 1;
          else if (Slot1Stall)                StallSlot1 <= StallSlot1 + 1;
          else                                StallOther <= StallOther + 1;
        end
        // "Solely" means every other term of Issue2F was satisfied, so this one term is what cost
        // the pair.  A cycle blocked by two terms is counted in neither, which is the point: fixing
        // one of them would not have paired it.
        if (Refused) begin
          PairOpportunity <= PairOpportunity + 1;
          if (~dut.core.ifu.issue.Slot1CanPairF & Cfi1F
              & dut.core.ifu.issue.BothRecognizedF & dut.core.ifu.issue.Slot0CanPairF
              & dut.core.ifu.issue.NoDependencyF    & dut.core.ifu.issue.OneMemOpF) begin
            if (Br1F) begin
              BlockBranch1 <= BlockBranch1 + 1;
              if (~dut.core.ifu.PCF[2]) BlockBranch1Aligned <= BlockBranch1Aligned + 1;
            end else BlockJump1 <= BlockJump1 + 1;
          end
          if (~dut.core.ifu.issue.Slot1CanPairF & dut.core.ifu.issue.MExtF & ~Cfi1F
              & dut.core.ifu.issue.BothRecognizedF & dut.core.ifu.issue.Slot0CanPairF
              & dut.core.ifu.issue.NoDependencyF   & dut.core.ifu.issue.OneMemOpF)
            BlockMDU1 <= BlockMDU1 + 1;
          // A slot 1 multiply or divide is refused by ~MExtF inside Slot1CanPairF, so it needs its
          // own attribution: it is not a branch, a jump, compressed, a dependency or a memory clash.
          if (~dut.core.ifu.issue.Slot1CanPairF & dut.core.ifu.issue.MExtF & ~Cfi1F
              & dut.core.ifu.issue.BothRecognizedF & dut.core.ifu.issue.Slot0CanPairF
              & dut.core.ifu.issue.NoDependencyF   & dut.core.ifu.issue.OneMemOpF) begin
            BlockMDU1 <= BlockMDU1 + 1;
          end
          if (~dut.core.ifu.issue.BothRecognizedF) AnyCompressed <= AnyCompressed + 1;
          if (~dut.core.ifu.issue.NoDependencyF)      AnyDep        <= AnyDep + 1;
          if (~dut.core.ifu.issue.BothRecognizedF
              & dut.core.ifu.issue.Slot0CanPairF & dut.core.ifu.issue.Slot1CanPairF
              & dut.core.ifu.issue.NoDependencyF & dut.core.ifu.issue.OneMemOpF) begin
            BlockCompressed <= BlockCompressed + 1;
            // Which slot's encoding instrfields failed to recognise.  A gap in slot 0 is pure
            // conservatism -- slot 0 executes everything it always did -- so it bounds what the
            // class-split change would recover without any datapath work.
            if (~dut.core.ifu.issue.Recognized0F) BlockUnrec0 <= BlockUnrec0 + 1;
            if (~dut.core.ifu.issue.Recognized1F) BlockUnrec1 <= BlockUnrec1 + 1;
          end
          if (~dut.core.ifu.issue.NoDependencyF
              & dut.core.ifu.issue.BothRecognizedF & dut.core.ifu.issue.Slot0CanPairF
              & dut.core.ifu.issue.Slot1CanPairF    & dut.core.ifu.issue.OneMemOpF)
            BlockDep <= BlockDep + 1;
          if (~dut.core.ifu.issue.Slot0CanPairF
              & dut.core.ifu.issue.BothRecognizedF & dut.core.ifu.issue.Slot1CanPairF
              & dut.core.ifu.issue.NoDependencyF     & dut.core.ifu.issue.OneMemOpF)
            BlockSlot0 <= BlockSlot0 + 1;
          if (~dut.core.ifu.issue.OneMemOpF
              & dut.core.ifu.issue.BothRecognizedF & dut.core.ifu.issue.Slot0CanPairF
              & dut.core.ifu.issue.Slot1CanPairF     & dut.core.ifu.issue.NoDependencyF)
            BlockOneMem <= BlockOneMem + 1;
        end
        if (dut.core.ifu.BPWrongE)                             FlushMispredict <= FlushMispredict + 1;
        if (dut.core.ifu.Kill2E & dut.core.ifu.Issue2E)         Kills <= Kills + 1;
        if (dut.core.Mem2M & ~dut.core.StallM)                  Mem2 <= Mem2 + 1;
        if (dut.core.Replay2M & ~dut.core.StallM)               Replays <= Replays + 1;
        if (dut.core.ieu.LoadStall2D       & ~dut.core.StallE)  LoadUse2    <= LoadUse2 + 1;
        if (dut.core.ieu.BundleStoreStallD & ~dut.core.StallE)  BundleStore <= BundleStore + 1;
        if (dut.core.Replay2M & ~dut.core.StallM & dut.core.ieu.MemRW2M[1]) Replay2Ld <= Replay2Ld + 1;
        if (dut.core.Replay2M & ~dut.core.StallM & dut.core.ieu.MemRW2M[0]) Replay2St <= Replay2St + 1;
        if (dut.core.PCSrcE & (|dut.core.ieu.MemRW2E) & ~dut.core.StallE)   BrWithMem2 <= BrWithMem2 + 1;
        // A replayed slot 1 store must never reach memory: the instruction is about to be refetched and
        // trap, so the handler has to find state unmodified.  Nothing in the design cancels that write
        // -- FlushW does it for a trap, and a replay withholds FlushW so slot 0 can commit -- so
        // issue.sv keeps stores out of slot 1 and this must never fire.  It is here to catch anyone
        // re-admitting them without first building the cancel path described in issue.sv's LOAD arm.
        if (dut.core.Replay2M & dut.core.Mem2M & dut.core.ieu.MemRW2M[0] & ~dut.core.lsu.LSUFlushW) begin
          ReplayStoreLeak <= ReplayStoreLeak + 1;
          $error("replayed slot 1 store was not cancelled at the LSU: the write will reach memory");
        end
      end
    // minstret must equal the number of instructions actually committed.  Both count from reset and
    // use the same condition, so any difference is the counter being wrong -- which matters because
    // a slot 1 abandoned in Execute must not be counted as retired.
    integer Committed = 0;
    always_ff @(posedge clk)
      if (~reset & dut.core.ieu.c.InstrValidM & ~dut.core.StallW & ~dut.core.FlushW)
        Committed <= Committed + 1 + ((dut.core.ifu.Issue2M & ~dut.core.Replay2M) ? 1 : 0);

    final begin
      $display("overhead: %0d cycles | stall slot0=%0d slot1=%0d other=%0d | mispredict=%0d | slot1 kills=%0d | slot1 mem=%0d replays=%0d | hazards: loaduse2=%0d bundlestore=%0d | replay ld=%0d st=%0d | branch+slot1mem=%0d | replay store leaks=%0d (must be 0)",
               Cycles, StallSlot0, StallSlot1, StallOther, FlushMispredict, Kills, Mem2, Replays, LoadUse2, BundleStore, Replay2Ld, Replay2St, BrWithMem2, ReplayStoreLeak);
      $display("pairing refused: %0d opportunities | sole cause: branch in slot1=%0d (8B-aligned=%0d) jump in slot1=%0d mdu in slot1=%0d compressed=%0d dependency=%0d slot0=%0d onemem=%0d (unrec slot0=%0d slot1=%0d) | any-cause: compressed=%0d dependency=%0d",
               PairOpportunity, BlockBranch1, BlockBranch1Aligned, BlockJump1, BlockMDU1, BlockCompressed, BlockDep, BlockSlot0, BlockOneMem, BlockUnrec0, BlockUnrec1, AnyCompressed, AnyDep);
      if (P.ZICSR_SUPPORTED)
        $display("minstret check: committed=%0d minstret=%0d %s", Committed,
                 dut.core.priv.priv.csr.counters.HPMCOUNTER_REGW[2],
                 (Committed == dut.core.priv.priv.csr.counters.HPMCOUNTER_REGW[2]) ? "MATCH" : "*** MISMATCH ***");
    end
  end



  // watch for problems such as lockup, reading uninitialized memory, bad configs
`ifdef MEMPIPE_PROBE
  `include "mempipeprobe.svh"
`endif
  watchdog #(P.XLEN, 1000000) watchdog(.clk, .reset, .TEST);  // check if PCW is stuck
  ramxdetector #(P.XLEN, P.LLEN) ramxdetector(clk, dut.core.lsu.MemRWM[1], dut.core.lsu.LSULoadAccessFaultM, dut.core.lsu.ReadDataM,
                                      dut.core.ifu.PCM, InstrM, dut.core.lsu.IEUAdrM, dut.core.lsu.StallW, InstrMName);
  riscvassertions       #(P) riscvassertions();     // check assertions for a legal architectural configuration
  riscvassertions_wally #(P) riscvassertions_wally();  // check assertions for a legal microarchitectural configuration
  loggers #(P, PrintHPMCounters, I_CACHE_ADDR_LOGGER, D_CACHE_ADDR_LOGGER, BPRED_LOGGER)
    loggers (clk, reset, DCacheFlushStart, DCacheFlushDone, memfilename, sim_log_prefix, TEST);

  // track the current function or global label
  if (DEBUG > 0 | ((PrintHPMCounters | BPRED_LOGGER) & P.ZICNTR_SUPPORTED)) begin : functionName
    functionName #(P) functionName(.reset(reset_ext | TestBenchReset),
            .clk(clk), .ProgramAddrMapFile(ProgramAddrMapFile), .ProgramLabelMapFile(ProgramLabelMapFile));
  end

  // Optionally log UART output to file (always on for buildroot, enable with +UART_LOG=1)
  if (P.UART_SUPPORTED) begin: uart_logger
    always @(posedge clk) begin
      if (uartoutfile) begin
        if (~dut.uncoregen.uncore.uartgen.uart.MEMWb & dut.uncoregen.uncore.uartgen.uart.uartPC.A == 3'b000 & ~dut.uncoregen.uncore.uartgen.uart.uartPC.DLAB) begin
          $fwrite(uartoutfile, "%c", dut.uncoregen.uncore.uartgen.uart.uartPC.Din); // append characters one at a time so we see a consistent log appearing during the run
          $fflush(uartoutfile);
        end
      end
    end
  end

  // Termination condition
  // Terminate on
  // 1. jump to self loop (0x0000006f)
  // 2. a store word writes to the address "tohost"
  // 3. or PC is stuck at 0


  logic              Store2ToHostM;        // superscalar slot 1 is storing a word (to "tohost", qualified below)
  logic [P.XLEN-1:0] PCM;
  // PCM is not valid for configurations without ZICSR or branch predictor
  flopenr #(P.XLEN) PCMReg(clk, reset, ~dut.core.StallM, dut.core.PCE, PCM);
  // A store to "tohost" ends the test.  It is recognised from slot 0 by name, but there is no Instr2M
  // to name slot 1's instruction by, so a superscalar slot 1 store is recognised from the control the
  // LSU acts on: a word-sized write that Mem2M says belongs to slot 1.  That is unambiguous here
  // because the issue rules keep AMO and store-conditional, the only other funct3=010 writes, in
  // slot 0.  Without this the test would simply never finish once stores can issue in slot 1.
  // The jump-to-self check needs no equivalent: j is JAL, which slot 1 never holds.
  assign Store2ToHostM = dut.core.Mem2M & dut.core.MemRWM[0] & (dut.core.MemFunct3M == 3'b010);
  always @(posedge clk) begin
    TestComplete <= ((InstrM == 32'h6f) & dut.core.InstrValidM ) |
       ((dut.core.lsu.IEUAdrM == ProgramAddrLabelArray["tohost"] & dut.core.lsu.IEUAdrM != 0) & (InstrMName == "SW" | Store2ToHostM)); // |
    //   (functionName.PCM == 0 & dut.core.ifu.InstrM == 0 & dut.core.InstrValidM & PrevPCZero));
    if (reset) PrevPCZero <= 0;
    else if (dut.core.InstrValidM) PrevPCZero <= (PCM == 0 & dut.core.ifu.InstrM == 0);
    if (PCM == 0 & dut.core.ifu.InstrM == 0 & dut.core.InstrValidM & PrevPCZero) begin
      $error("Program fetched illegal instruction 0x00000000 from address 0x00000000 twice in a row.  Usually due to fault with no fault handler.");
      $fatal(1);
    end
  end

  DCacheFlushFSM #(P) DCacheFlushFSM(.clk, .start(DCacheFlushStart), .done(DCacheFlushDone));

  logic [P.XLEN-1:0] Minstret;
  // The counter only exists when the privileged unit does, so the hierarchical reference has to be
  // elaborated conditionally.  A procedural if would not do: the name is resolved either way.
  if (P.ZICSR_SUPPORTED) assign Minstret = testbench.dut.core.priv.priv.csr.counters.HPMCOUNTER_REGW[2];
  else                   assign Minstret = '0;   // INSTR_LIMIT is then simply never reached
  always @(negedge clk) begin
    if (INSTR_LIMIT > 0) begin
      if((Minstret != 0) & (Minstret % 'd100000 == 0)) $display("Reached %d instructions", Minstret);
      if((Minstret == INSTR_LIMIT) & (INSTR_LIMIT!=0)) begin $finish; end
    end
  end

// RVVI trace for functional coverage and lockstep
`ifdef ENABLE_RVVI_TRACE
  // RETIRE=2: the superscalar core can retire two instructions in one cycle, and each needs its own
  // RVVI record or the reference model falls a instruction behind and never catches up.
  rvviTrace #(.XLEN(P.XLEN), .FLEN(P.FLEN), .RETIRE(2)) rvvi();
  wallyTracer #(P) wallyTracer(rvvi);
`endif

// Functional coverage
`ifdef FCOV
  cvw_arch_verif cvw_arch_verif(rvvi);
`endif

  ////////////////////////////////////////////////////////////////////////////////
  // ImperasDV Co-simulator hooks
  ////////////////////////////////////////////////////////////////////////////////
`ifdef USE_IMPERAS_DV

  // enabling of comparison types
  trace2api #(.CMP_PC      (1),
              .CMP_INS     (1),
              .CMP_GPR     (1),
              .CMP_FPR     (P.F_SUPPORTED),
              .CMP_VR      (0),
              .CMP_CSR     (P.ZICSR_SUPPORTED)
              ) idv_trace2api(rvvi);
  trace2log idv_trace2log(rvvi); // enable Imperas tracer

  string filename;
  initial begin
    // imperasDV requires the elffile be defined at the beginning of the simulation.
    int iter;
    longint x64;
    int     x32[2];
    longint index;
    string  memfilenameImperasDV, bootmemfilenameImperasDV;
    #1;
    IDV_MAX_ERRORS = 3;
    elffilename = ElfFile;

    // Initialize REF (do this before initializing the DUT)
    if (!rvviVersionCheck(RVVI_API_VERSION)) begin
      $display($sformatf("%m @ t=%0t: Expecting RVVI API version %0d.", $time, RVVI_API_VERSION));
      $fatal;
    end

    void'(rvviRefConfigSetString(IDV_CONFIG_MODEL_VENDOR,            "riscv.ovpworld.org"));
    void'(rvviRefConfigSetString(IDV_CONFIG_MODEL_NAME,              "riscv"));
    void'(rvviRefConfigSetString(IDV_CONFIG_MODEL_VARIANT,           "RV64GCK"));
    void'(rvviRefConfigSetInt(IDV_CONFIG_MODEL_ADDRESS_BUS_WIDTH,     XLEN==64 ? 56 : 34));
    void'(rvviRefConfigSetInt(IDV_CONFIG_MAX_NET_LATENCY_RETIREMENTS, 6));

    if(elffilename == "buildroot") filename = "";
    else filename = elffilename;

    // use the ImperasDV rvviRefInit to load the reference model with an elf file
    if(elffilename != "none") begin
      if (!rvviRefInit(filename)) begin
        $display($sformatf("%m @ t=%0t: rvviRefInit failed", $time));
        $fatal;
      end
    end else begin // for buildroot use the binary instead to load the reference model.
      if (!rvviRefInit("")) begin // still have to call with nothing
        $display($sformatf("%m @ t=%0t: rvviRefInit failed", $time));
        $fatal;
      end

      memfilenameImperasDV = {RISCV_DIR, "/linux-testvectors/ram.bin"};
      bootmemfilenameImperasDV = {RISCV_DIR, "/linux-testvectors/bootmem.bin"};

      $display("RVVI Loading bootmem.bin");
      memFile = $fopen(bootmemfilenameImperasDV, "rb");
      index = 'h1000 - 8;
      while(!$feof(memFile)) begin
        index+=8;
        readResult = $fread(x64, memFile);
        if (x64 == 0) continue;
        x32[0] = x64 & 'hffffffff;
        x32[1] = x64 >> 32;
        rvviRefMemoryWrite(0, index+0, x32[0], 4);
        rvviRefMemoryWrite(0, index+4, x32[1], 4);
        //$display("boot %08X x32[0]=%08X x32[1]=%08X", index, x32[0], x32[1]);
      end
      $fclose(memFile);

      $display("RVVI Loading ram.bin");
      memFile = $fopen(memfilenameImperasDV, "rb");
      index = 'h80000000 - 8;
      while(!$feof(memFile)) begin
        index+=8;
        readResult = $fread(x64, memFile);
        if (x64 == 0) continue;
        x32[0] = x64 & 'hffffffff;
        x32[1] = x64 >> 32;
        rvviRefMemoryWrite(0, index+0, x32[0], 4);
        rvviRefMemoryWrite(0, index+4, x32[1], 4);
        //$display("ram  %08X x32[0]=%08X x32[1]=%08X", index, x32[0], x32[1]);
      end
      $fclose(memFile);

      $display("RVVI Loading Complete");

      void'(rvviRefPcSet(0, P.RESET_VECTOR)); // set BOOTROM address
    end

    // Volatile CSRs
    // Counter CSRs
    if (P.ZICNTR_SUPPORTED) begin
      void'(rvviRefCsrSetVolatile(0, 32'hC00));   // CYCLE
      void'(rvviRefCsrSetVolatile(0, 32'hB00));   // MCYCLE
      void'(rvviRefCsrSetVolatile(0, 32'hC02));   // INSTRET
      void'(rvviRefCsrSetVolatile(0, 32'hB02));   // MINSTRET
      void'(rvviRefCsrSetVolatile(0, 32'hC01));   // TIME
      if (P.XLEN == 32) begin
        void'(rvviRefCsrSetVolatile(0, 32'hC80));   // CYCLEH
        void'(rvviRefCsrSetVolatile(0, 32'hB80));   // MCYCLEH
        void'(rvviRefCsrSetVolatile(0, 32'hC82));   // INSTRETH
        void'(rvviRefCsrSetVolatile(0, 32'hB82));   // MINSTRETH
        void'(rvviRefCsrSetVolatile(0, 32'hC81));   // TIMEH
      end
      // HPM counters
      if (P.ZIHPM_SUPPORTED) begin
        // User HPMCOUNTER3 - HPMCOUNTER31
        if (P.U_SUPPORTED) begin
          for (iter='hC03; iter<='hC1F; iter++) begin
            void'(rvviRefCsrSetVolatile(0, iter));   // HPMCOUNTERx
            if (P.XLEN == 32)
              void'(rvviRefCsrSetVolatile(0, iter+128));   // HPMCOUNTERxH
          end
        end

        // Machine MHPMCOUNTER3 - MHPMCOUNTER31
        for (iter='hB03; iter<='hB1F; iter++) begin
          void'(rvviRefCsrSetVolatile(0, iter));   // MHPMCOUNTERx
          if (P.XLEN == 32)
            void'(rvviRefCsrSetVolatile(0, iter+128));   // MHPMCOUNTERxH
        end
      end
    end

    // cannot predict this register due to latency between
    // pending and taken
    void'(rvviRefCsrSetVolatile(0, 32'h344));   // MIP
    if (P.S_SUPPORTED) begin
      void'(rvviRefCsrSetVolatile(0, 32'h144));   // SIP
    end

    // Privileges for PMA are set in the imperas.ic
    // volatile (IO) regions are defined here
    // only real ROM/RAM areas are BOOTROM and UNCORE_RAM
    if (P.CLINT_SUPPORTED) begin
      void'(rvviRefMemorySetVolatile(P.CLINT_BASE, (P.CLINT_BASE + P.CLINT_RANGE)));
    end
    if (P.GPIO_SUPPORTED) begin
      void'(rvviRefMemorySetVolatile(P.GPIO_BASE, (P.GPIO_BASE + P.GPIO_RANGE)));
    end
    if (P.UART_SUPPORTED) begin
      void'(rvviRefMemorySetVolatile(P.UART_BASE, (P.UART_BASE + P.UART_RANGE)));
    end
    if (P.PLIC_SUPPORTED) begin
      void'(rvviRefMemorySetVolatile(P.PLIC_BASE, (P.PLIC_BASE + P.PLIC_RANGE)));
    end
    if (P.SDC_SUPPORTED) begin
      void'(rvviRefMemorySetVolatile(P.SDC_BASE, (P.SDC_BASE + P.SDC_RANGE)));
    end
    if (P.SPI_SUPPORTED) begin
      void'(rvviRefMemorySetVolatile(P.SPI_BASE, (P.SPI_BASE + P.SPI_RANGE)));
    end
  end

  if (P.ZICSR_SUPPORTED) begin
    always @(dut.core.priv.priv.csr.csri.MIP_REGW[7])   void'(rvvi.net_push("MTimerInterrupt",    dut.core.priv.priv.csr.csri.MIP_REGW[7]));
    always @(dut.core.priv.priv.csr.csri.MIP_REGW[11])  void'(rvvi.net_push("MExternalInterrupt", dut.core.priv.priv.csr.csri.MIP_REGW[11]));
    always @(dut.core.priv.priv.csr.csri.MIP_REGW[3])   void'(rvvi.net_push("MSWInterrupt",       dut.core.priv.priv.csr.csri.MIP_REGW[3]));
    if (P.S_SUPPORTED) begin
      always @(dut.core.priv.priv.csr.csri.MIP_REGW[5])   void'(rvvi.net_push("STimerInterrupt",    dut.core.priv.priv.csr.csri.MIP_REGW[5]));
      always @(dut.core.priv.priv.csr.csri.MIP_REGW[9])   void'(rvvi.net_push("SExternalInterrupt", dut.core.priv.priv.csr.csri.MIP_REGW[9]));
      // when ImperasDV is updated, restore the MIP-based logic commented out below and remove the ValidIntsM-based logic
      // See Fixed https://github.com/openhwgroup/cvw-arch-verif/issues/670
      //always @(dut.core.priv.priv.csr.csri.MIP_REGW[1])   void'(rvvi.net_push("SSWInterrupt",       dut.core.priv.priv.csr.csri.MIP_REGW[1]));
      always @(dut.core.priv.priv.trap.ValidIntsM[1])   void'(rvvi.net_push("SSWInterrupt",       dut.core.priv.priv.trap.ValidIntsM[1])); // dh 6/29/25 temporarily use ValidInts until Synopsys fixes level sensitive  https://github.com/openhwgroup/cvw-arch-verif/issues/670
    end
  end

  final begin
    void'(rvviRefShutdown());
  end

`endif
  ////////////////////////////////////////////////////////////////////////////////
  // END of ImperasDV Co-simulator hooks
  ////////////////////////////////////////////////////////////////////////////////

  task automatic CheckSignature;
    // This task must be declared inside this module as it needs access to parameter P.  There is
    // no way to pass P to the task unless we convert it to a module.

    input string  pathname;
    input string  TestName;
    input logic   riscofTest;
    input integer begin_signature_addr;
    output integer errors;
    int fd, code;
    string line;
    int siglines, sigentries;

    localparam SIGNATURESIZE = 5000000;
    integer        i;
    logic [31:0]   sig32[0:SIGNATURESIZE];
    logic [31:0]   parsed;
    logic [P.XLEN-1:0] signature[0:SIGNATURESIZE];
    string            signame;
    logic [P.XLEN-1:0] testadr, testadrNoBase;

    //$display("Invoking CheckSignature %s %s %0t", pathname, TestName, $time);

    // read .signature.output file and compare to check for errors
    if (riscofTest) signame = {pathname, TestName, "/ref/Reference-sail_c_simulator.signature"};
    else signame = {pathname, TestName, ".signature.output"};

    // read signature file from memory and count lines.  Can't use readmemh because we need the line count
    // $readmemh(signame, sig32);
    fd = $fopen(signame, "r");
    siglines = 0;
    if (fd == 0) $display("Unable to read %s", signame);
    else begin
      while (!$feof(fd)) begin
        code = $fgets(line, fd);
        if (code != 0) begin
          int errno;
          string errstr;
          errno = $ferror(fd, errstr);
          if (errno != 0) $display("Error %d (code %d) reading line %d of %s: %s", errno, code, siglines, signame, errstr);
          if (line.len() > 1) begin // skip blank lines
            if ($sscanf(line, "%x", parsed) != 0) begin
              sig32[siglines] = parsed;
              siglines = siglines + 1; // increment if line is not blank
            end
          end
        end
      end
      $fclose(fd);
    end

    // Check valid number of lines were read
    if (siglines == 0) begin
      errors = 1;
      $display("Error: empty test file %s", signame);
    end else if (P.XLEN == 64 & (siglines % 2)) begin
      errors = 1;
      $display("Error: RV64 signature has odd number of lines %s", signame);
    end else errors = 0;

    // copy lines into signature, converting to XLEN if necessary
    sigentries = (P.XLEN == 32) ? siglines : siglines/2; // number of signature entries
    for (i=0; i<sigentries; i++) begin
      signature[i] = (P.XLEN == 32) ? sig32[i] : {sig32[i*2+1], sig32[i*2]};
      //$display("XLEN = %d signature[%d] = %x", P.XLEN, i, signature[i]);
    end

    // Check errors
    testadr = ($unsigned(begin_signature_addr))/(P.XLEN/8);
    testadrNoBase = (begin_signature_addr - P.UNCORE_RAM_BASE)/(P.XLEN/8);
    for (i=0; i<sigentries; i++) begin
      if (signature[i] !== testbench.DCacheFlushFSM.ShadowRAM[testadr+i]) begin
        errors = errors+1;
        $display("  Error on test %s result %d: adr = %h sim (D$) %h signature = %h",
          TestName, i, (testadr+i)*(P.XLEN/8), testbench.DCacheFlushFSM.ShadowRAM[testadr+i], signature[i]);
        $stop; // if this is changed to $finish, wally-batch.do does not get to the next step to run coverage
      end
    end
    if (errors) $display("%s failed with %d errors. :(", TestName, errors);
    else $display("%s succeeded.  Brilliant!!!", TestName);
  endtask

  // Report the result of a self-checking test.  Such a test embeds its expected signature, compares
  // each entry as it is written, and leaves the outcome in selfcheck_record (XLEN-sized entries):
  //   0: status (0 = did not finish, 1 = passed, 2 = entry mismatch, 3 = wrong number of entries)
  //   1: entry index    2: entry address    3: expected value    4: actual value
  task automatic CheckSelfCheck;
    input string  TestName;
    input integer selfcheck_record_addr;
    output integer errors;
    logic [P.XLEN-1:0] status, index, adr, expected, actual;
    integer recadr;
    recadr = $unsigned(selfcheck_record_addr) / (P.XLEN/8); // $unsigned because integer is signed and RAM addresses have bit 31 set
    status   = testbench.DCacheFlushFSM.ShadowRAM[recadr];
    index    = testbench.DCacheFlushFSM.ShadowRAM[recadr+1];
    adr      = testbench.DCacheFlushFSM.ShadowRAM[recadr+2];
    expected = testbench.DCacheFlushFSM.ShadowRAM[recadr+3];
    actual   = testbench.DCacheFlushFSM.ShadowRAM[recadr+4];
    errors = 0;
    case (status)
      1: $display("%s succeeded.  Brilliant!!!", TestName);
      2: begin
        errors = 1;
        $display("  Error on test %s result %0d: adr = %h sim (D$) %h signature = %h", TestName, index, adr, actual, expected);
      end
      3: begin
        errors = 1;
        $display("  Error on test %s: wrote %0d signature entries but expected %0d (next adr = %h)", TestName, actual, expected, adr);
      end
      default: begin
        errors = 1;
        $display("  Error on test %s: halted without completing its self-check (status = %h)", TestName, status);
      end
    endcase
    if (errors) begin
      $display("%s failed with %d errors. :(", TestName, errors);
      $stop; // if this is changed to $finish, wally-batch.do does not get to the next step to run coverage
    end
  endtask

`ifdef PMP_COVERAGE
test_pmp_coverage #(P) pmp_inst(clk);
`endif
  /* verilator lint_on WIDTHTRUNC */
  /* verilator lint_on WIDTHEXPAND */

endmodule

/* verilator lint_on STMTDLY */
/* verilator lint_on WIDTH */

task automatic updateProgramAddrLabelArray;
  /* verilator lint_off WIDTHTRUNC */
  /* verilator lint_off WIDTHEXPAND */
  input string ProgramAddrMapFile, ProgramLabelMapFile, memfilename, WALLY_DIR;
  inout  integer ProgramAddrLabelArray [string];
  // Gets the memory location of begin_signature
  integer ProgramLabelMapFP, ProgramAddrMapFP;
  string cmd;

  // if memfile, label, or addr files are out of date or don't exist, generate them
  cmd = {"make -s -f ", WALLY_DIR, "/testbench/Makefile ", memfilename, " ", ProgramAddrMapFile};
  $system(cmd);

  ProgramLabelMapFP = $fopen(ProgramLabelMapFile, "r");
  ProgramAddrMapFP = $fopen(ProgramAddrMapFile, "r");

  if (ProgramLabelMapFP & ProgramAddrMapFP) begin // check we found both files
    ProgramAddrLabelArray["begin_signature"] = 0;
    ProgramAddrLabelArray["tohost"] = 0;
    ProgramAddrLabelArray["sig_end_canary"] = 0;
    ProgramAddrLabelArray["selfcheck_record"] = 0;
    while (!$feof(ProgramLabelMapFP)) begin
      string label, adrstr;
      integer returncode;
      returncode = $fscanf(ProgramLabelMapFP, "%s\n", label);
      returncode = $fscanf(ProgramAddrMapFP, "%s\n", adrstr);
      if (ProgramAddrLabelArray.exists(label)) ProgramAddrLabelArray[label] = adrstr.atohex();
    end
  end

//  if(ProgramAddrLabelArray["begin_signature"] == 0) $display("Couldn't find begin_signature in %s", ProgramLabelMapFile);
//  if(ProgramAddrLabelArray["sig_end_canary"] == 0) $display("Couldn't find sig_end_canary in %s", ProgramLabelMapFile);

  $fclose(ProgramLabelMapFP);
  $fclose(ProgramAddrMapFP);
  /* verilator lint_on WIDTHTRUNC */
  /* verilator lint_on WIDTHEXPAND */
endtask
