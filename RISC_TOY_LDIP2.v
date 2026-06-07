/*****************************************
    
    Team XX : 
        2023104125    ???
        2021103504    ???
     	2021104197	  ???

    [INTEGRATED VERSION]
    - IF / ID / EX / MEM / WB 5-stage ?????? RISC_TOY ?? ??? ??
    - ?? ?? ?? ?? ?? (REGFILE ? ?? model.v ??)
    - Forwarding (EX/MEM, MEM/WB ? EX), Load-Use Stall, Branch Flush ??

    [FIX 1] EX/MEM ????? ????? Branch_Taken flush ??
            ? Branch ? ??? ???? ????? MEM/WB ? WB?? ????
              ????? ????? ???? ?? ??
    [FIX 2] LD ????? rb == R[31]? ? ??? ??? 0?? ?? (????)
            ? ??? ?? ??: LD rb=R[31] ? zero-extend (???? ??)
    [FIX 3] LD/LDR ?? read latency ?? (R4=x ?? ??)
            ? DATA_RAM ? ?? read ???(?? ??? ?? ???? DRDATA valid).
              ?? ??? LD ??? 1?? ?? MEM/WB ? latch ?? load ?? x ?
              ???, ??? ?? ??(ADD)?? ?? ?????.
            ? LD ? MEM ??? ??? ? ???? 1?? Mem_Stall ? ????
              DRDATA ? valid ? ?? ??? MEM/WB ? ?? latch ?? ?.
            ? ?? stall(load-use + mem-stall) ?? ???? ???? ???
              IF ?? ?? ??(INSTR_reg)? stall ?? ????? ??.

    [FIX 4] ?? ? ? ?? flush ?? ?? (R5=99 ?? ??)
            ? ??/??? EX ???? ???? INST_MEM ? ?? read ???,
              ?? ?? delay-slot ? 2? ???(IF/ID ? ???? ??? ?? 2?).
              ?? ??? ID/EX ???? 1?? ?? ?? ???? ???? ???.
            ? Branch_Taken ?? + ? ?? 1??, ? 2?? ?? IF/ID ???
              NOP ?? ??(IFID_Flush) delay-slot 2?? ?? ???.
              (??? ??? ? 2?? ?? IF/ID ? ????? ????.)

*****************************************/


////////////////////////////////////
//  TOP MODULE
////////////////////////////////////
module RISC_TOY (
    input     wire              CLK,
    input     wire              RSTN,
    output    wire              IREQ,
    output    wire    [29:0]    IADDR,
    input     wire    [31:0]    INSTR,
    output    wire              DREQ,
    output    wire    [1:0]     DRW,
    output    wire    [29:0]    DADDR,
    output    wire    [31:0]    DWDATA,
    output    wire    [31:0]    CONSIG,
    input     wire    [31:0]    DRDATA
);

    // =========================================================================
    // ?? ?? : ?? ???? ???? ???? ????? ???
    // =========================================================================
    // WB ???? RegFile ? ??? ??
    wire [31:0] WB_data;
    wire [4:0]  WB_Rd;
    wire        WB_RegWrite;

    // Branch / Hazard ?? (EX ???? ??)
    wire        Branch_Taken;
    wire [31:0] Branch_Target;

    // Load-Use Hazard ? ?? Stall (ID ???? ??)
    wire        Stall_Flag;       // load-use (ID ??)
    wire        Mem_Stall;        // [FIX 3] LD ?? read latency ??? 1?? stall (MEM ??)
    wire        Pipe_Stall;       // ? stall ? OR : ??(PC/IF/ID/ID/EX) ??? ??
    assign      Pipe_Stall = Stall_Flag | Mem_Stall;
    // Branch taken ? IF/ID, ID/EX ? Flush
    wire        Flush_Flag;
    assign      Flush_Flag = Branch_Taken;

    // =========================================================================
    // [Flush ??] ??(J/Branch) ??
    //   ?? ?? ??(???? ?????):
    //     - J ? EX ?? ???? Branch_Taken=1 ? ?? ????,
    //       "??? ??? ? ??(?? ?? ?? ??)"? ?? IF/ID ? ??? ??.
    //     - ? ?? ??? IF/ID ? ???? ?? "?? ???"??? ??? ??.
    //   ??? ?? ??? ??? 1????, flush ? Branch_Taken 1???? ????.
    //   (??? Flush_Delay ?? ??? ????? ???? ??)
    //
    //   ? ??: Branch_Taken ??? ?? IF/ID ?? ?? ???
    //     "??? ??? ??? ?? IF/ID flush"?? ? ???.
    //     ? ? ??? ID/EX ? ???? ?? ID/EX ???? ??? ??(?? [4] ??).
    // =========================================================================
    wire        Extended_Flush = Branch_Taken;   // 1?? flush (ID/EX ???, ?? ??)

    // =========================================================================
    // [FIX 4] ?? ? IF/ID 2?? flush
    //   ??/??? EX ???? ????(Branch_Taken=1). INST_MEM ? ?? read ?
    //   ?? ?? ?? ???? ? "?? 2?? fetch" ? ?? ?? ??? ??
    //   (delay-slot 2?)?? IF/ID ? ?? ????.
    //   ?? ??? ID/EX ???? 1?? ??(?? ???), ??�??? ?????.
    //   ? Branch_Taken ?? 2?? ?? IF/ID ??? NOP ?? ?? ? ? ???.
    //     (??? ??? ? 2??? ?? ? IF/ID ? ????? ????.)
    reg [1:0]   br_flush_cnt;
    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN)
            br_flush_cnt <= 2'd0;
        else if (Branch_Taken)
            br_flush_cnt <= 2'd1;            // ?? ?? ? ?? 1?? ? IF/ID flush ??
                                             //  (?? ??? Branch_Taken ?? ?? ????
                                             //   delay-slot 2? = Branch_Taken(1) + cnt(1))
        else if (Pipe_Stall)
            br_flush_cnt <= br_flush_cnt;    // stall ??? ??? ??(?? ?? ??)
        else if (br_flush_cnt != 2'd0)
            br_flush_cnt <= br_flush_cnt - 2'd1;
    end
    wire        IFID_Flush = Branch_Taken | (br_flush_cnt != 2'd0);   // IF/ID ??? NOP ?? ?? ??

   


    // =========================================================================
    // [1] IF ?? (Instruction Fetch)
    // =========================================================================
    reg [31:0] PC;

    // =========================================================================
    // ?? [???] Stall ?? ? ???? ???? ???? ?? ??
    // =========================================================================
    reg [31:0]  INSTR_reg;
    reg         Was_Stalled;

    // ??? ?? ??(Was_Stalled=1)?? ????? ? ??? ?? ???, ???? ?? ???? ??!
    // (ModelSim ??: ??? assign ? ????, always ???? ?? ??)
    wire        use_buffer;
    wire [31:0] safe_INSTR;
    wire [31:0] safe_PC;
    assign use_buffer = Was_Stalled & ~Extended_Flush;
    assign safe_INSTR = use_buffer ? INSTR_reg : INSTR;
    assign safe_PC    = use_buffer ? (PC - 32'd4) : PC;

    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            INSTR_reg   <= 32'd0;
            Was_Stalled <= 1'b0;
        end else begin
            // [FIX 3] stall ? ??? ? ?? ???? ????? ??? ??.
            //   - ?? stall ?? ???: ??? ??? ???? ??
            //   - ?? stall ???: ??? ??? ? safe_INSTR ? ??? ??
            //     (PC ? ?? ?? ?? ??? ????? ??? ??? ???? ? ?)
            INSTR_reg   <= Pipe_Stall ? safe_INSTR : INSTR;
            Was_Stalled <= Pipe_Stall;  // ?? ? ??? (??) ?????? ???
        end
    end
    
    //=========================================================================


    // ?? PC ?? : branch ??, ??? PC+4
    wire [31:0] next_PC = Branch_Taken ? Branch_Target : (PC + 32'd4);

    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            PC <= 32'd0;            // ??? : ?? ? PC=0
        end else if (!Pipe_Stall) begin
            PC <= next_PC;
        end
        // Stall ??? PC ??
    end

    assign IREQ  = 1'b1;
    assign IADDR = PC[29:0];        // word address (30-bit)


    // --- IF/ID ????? ???? ---
    reg [31:0] IF_ID_PC;
    reg [31:0] IF_ID_Inst;
    reg        IF_ID_Valid;   // ?? [??] ? ??? ?? ??(1)??, ??? ??? ? ??(0)??

    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            IF_ID_PC    <= 32'd0;
            IF_ID_Inst  <= 32'd0;
            IF_ID_Valid <= 1'b0;
        end else if (Pipe_Stall) begin
            // Load-Use Stall / Mem-Stall : IF/ID ?? (?? ???? ? ? ? ?? ?)
            IF_ID_PC    <= IF_ID_PC;
            IF_ID_Inst  <= IF_ID_Inst;
            IF_ID_Valid <= IF_ID_Valid;
        end else if (IFID_Flush) begin
            // [FIX 4] ?? ?? delay-slot(???) ??? IF/ID ???? NOP ?? ???.
            IF_ID_PC    <= 32'd0;
            IF_ID_Inst  <= 32'd0;
            IF_ID_Valid <= 1'b0;
        end else begin
            IF_ID_PC    <= safe_PC;
            IF_ID_Inst  <= safe_INSTR;
            IF_ID_Valid <= 1'b1;
        end
    end


    // =========================================================================
    // [2] ID ?? (Instruction Decode)
    // =========================================================================
    // ??? ??? ?? ?? ??
    wire [4:0]  opcode = IF_ID_Inst[31:27];
    wire [4:0]  ra     = IF_ID_Inst[26:22];
    wire [4:0]  rb     = IF_ID_Inst[21:17];
    wire [4:0]  rc     = IF_ID_Inst[16:12];
    wire [16:0] imm17  = IF_ID_Inst[16:0];
    wire [21:0] imm22  = IF_ID_Inst[21:0];
    wire [2:0]  br_cond = IF_ID_Inst[2:0];      // BR/BRL ? cond ??

    // ?? ??
    wire [31:0] sign_ext_17 = {{15{imm17[16]}}, imm17};
    wire [31:0] sign_ext_22 = {{10{imm22[21]}}, imm22};

    // ???? ?? 17?? / 22?? Immediate ??
    //   imm22 ?? : J(17), JL(18), LDR(20), STR(22)
    wire use_imm22 = (opcode == 5'd17) || (opcode == 5'd18) ||
                     (opcode == 5'd20) || (opcode == 5'd22);
    wire [31:0] selected_imm = use_imm22 ? sign_ext_22 : sign_ext_17;

    // --- ??? (Control Unit) ---
    reg        Ctrl_RegWrite, Ctrl_MemRead, Ctrl_MemWrite, Ctrl_MemToReg, Ctrl_ALUSrc;
    reg        Ctrl_IPMem;            // ?? [LDIP/STIP] 1?? ??? ??? CPU ? ??? CUSTOM_IP ?
                                       //   (LDIP : ????IPIN / STIP : IPOUT????)
    reg [3:0]  Ctrl_ALUOp;             // EX_stage ? 4-bit ALU op encoding ?? ??
    reg [2:0]  Ctrl_BranchType;        // 000:none / 001:BR / 010:BRL / 011:J / 100:JL

    // ALU op encoding (EX ??? ??)
    //   0000:ADD 0001:SUB 0010:NEG 0011:NOT 0100:AND 0101:OR 0110:XOR
    //   0111:LSR 1000:ASR 1001:SHL 1010:ROR 1011:PASS_B 1101:LINK(PC+4)
    always @(*) begin
        // default
        Ctrl_RegWrite   = 1'b0;
        Ctrl_MemRead    = 1'b0;
        Ctrl_MemWrite   = 1'b0;
        Ctrl_MemToReg   = 1'b0;
        Ctrl_ALUSrc     = 1'b0;
        Ctrl_IPMem      = 1'b0;        // ?? ?? : ?? CPU ??? ??
        Ctrl_ALUOp      = 4'b0000;
        Ctrl_BranchType = 3'b000;

        case (opcode)
            // ---- I-type arithmetic / logical ----
            5'd0 : begin  // ADDI
                Ctrl_RegWrite = 1; Ctrl_ALUSrc = 1; Ctrl_ALUOp = 4'b0000;
            end
            5'd1 : begin  // ANDI
                Ctrl_RegWrite = 1; Ctrl_ALUSrc = 1; Ctrl_ALUOp = 4'b0100;
            end
            5'd2 : begin  // ORI
                Ctrl_RegWrite = 1; Ctrl_ALUSrc = 1; Ctrl_ALUOp = 4'b0101;
            end
            5'd3 : begin  // MOVI : R[ra] = imm
                Ctrl_RegWrite = 1; Ctrl_ALUSrc = 1; Ctrl_ALUOp = 4'b1011; // PASS_B
            end

            // ---- R-type arithmetic / logical / shift ----
            5'd4 : begin  // ADD
                Ctrl_RegWrite = 1; Ctrl_ALUSrc = 0; Ctrl_ALUOp = 4'b0000;
            end
            5'd5 : begin  // SUB
                Ctrl_RegWrite = 1; Ctrl_ALUSrc = 0; Ctrl_ALUOp = 4'b0001;
            end
            5'd6 : begin  // NEG : R[ra] = -R[rb] ? ALU ???? op_b = R[rb]
                Ctrl_RegWrite = 1; Ctrl_ALUSrc = 0; Ctrl_ALUOp = 4'b0010;
            end
            5'd7 : begin  // NOT : R[ra] = ~R[rb]
                Ctrl_RegWrite = 1; Ctrl_ALUSrc = 0; Ctrl_ALUOp = 4'b0011;
            end
            5'd8 : begin  // AND
                Ctrl_RegWrite = 1; Ctrl_ALUSrc = 0; Ctrl_ALUOp = 4'b0100;
            end
            5'd9 : begin  // OR
                Ctrl_RegWrite = 1; Ctrl_ALUSrc = 0; Ctrl_ALUOp = 4'b0101;
            end
            5'd10: begin  // XOR
                Ctrl_RegWrite = 1; Ctrl_ALUSrc = 0; Ctrl_ALUOp = 4'b0110;
            end
            5'd11: begin  // LSR
                Ctrl_RegWrite = 1; Ctrl_ALUSrc = 0; Ctrl_ALUOp = 4'b0111;
            end
            5'd12: begin  // ASR
                Ctrl_RegWrite = 1; Ctrl_ALUSrc = 0; Ctrl_ALUOp = 4'b1000;
            end
            5'd13: begin  // SHL
                Ctrl_RegWrite = 1; Ctrl_ALUSrc = 0; Ctrl_ALUOp = 4'b1001;
            end
            5'd14: begin  // ROR
                Ctrl_RegWrite = 1; Ctrl_ALUSrc = 0; Ctrl_ALUOp = 4'b1010;
            end

            // ---- Branch / Jump ----
            5'd15: begin  // BR  : PC = R[rb]
                Ctrl_BranchType = 3'b001;
            end
            5'd16: begin  // BRL : R[ra] = PC+4, PC = R[rb]
                Ctrl_BranchType = 3'b010;
                Ctrl_RegWrite   = 1;
                Ctrl_ALUOp      = 4'b1101;   // LINK : op_a + 4
            end
            5'd17: begin  // J   : PC = currentPC + signExt(imm22)
                Ctrl_BranchType = 3'b011;
            end
            5'd18: begin  // JL  : R[ra] = PC+4, PC = currentPC + signExt(imm22)
                Ctrl_BranchType = 3'b100;
                Ctrl_RegWrite   = 1;
                Ctrl_ALUOp      = 4'b1101;   // LINK : op_a + 4
            end

            // ---- Memory ----
            5'd19: begin  // LD  : R[ra] = M[R[rb] + signExt(imm17)] (or zeroExt if rb==31)
                Ctrl_RegWrite = 1; Ctrl_MemRead = 1; Ctrl_MemToReg = 1;
                Ctrl_ALUSrc   = 1; Ctrl_ALUOp = 4'b0000;
            end
            5'd20: begin  // LDR : R[ra] = M[currentPC + signExt(imm22)]
                Ctrl_RegWrite = 1; Ctrl_MemRead = 1; Ctrl_MemToReg = 1;
                Ctrl_ALUSrc   = 1; Ctrl_ALUOp = 4'b0000;
            end
            5'd21: begin  // ST  : M[R[rb] + signExt(imm17)] = R[ra]
                Ctrl_MemWrite = 1; Ctrl_ALUSrc = 1; Ctrl_ALUOp = 4'b0000;
            end
            5'd22: begin  // STR : M[currentPC + signExt(imm22)] = R[ra]
                Ctrl_MemWrite = 1; Ctrl_ALUSrc = 1; Ctrl_ALUOp = 4'b0000;
            end

            // ---- Custom IP Memory (ray-tracing accelerator I/O) ----
            //   ?? ??? LD/ST ? ???? R[rb] + signExt(imm17) ??.
            //   ?? ?? ??? ?????(??? ? CUSTOM_IP).
            //   ? opcode ??(23/24)? ???. inst.hex/???? ??? ?? ? ??? ??.
            5'd23: begin  // LDIP : M[R[rb]+signExt(imm17)] ? IPIN  (DRW=2'b11)
                          //   CPU ????? ?? ??(RegWrite=0). ??? ??? IP ??? ??.
                Ctrl_MemRead = 1; Ctrl_IPMem = 1;
                Ctrl_ALUSrc  = 1; Ctrl_ALUOp = 4'b0000;
            end
            5'd24: begin  // STIP : IPOUT ? M[R[rb]+signExt(imm17)]  (DRW=2'b01)
                          //   ?? ???? CUSTOM_IP ? IPOUT(=DATA_RAM.DI2). CPU ? ??? ??.
                Ctrl_MemWrite = 1; Ctrl_IPMem = 1;
                Ctrl_ALUSrc   = 1; Ctrl_ALUOp = 4'b0000;
            end

            default: ; // NOP
        endcase
    end

    // =========================================================================
    // ?? [??2 ??] IF_ID_Valid ? ???? ???
    //   IF/ID ? ?? ??? ??(??? ??? ? ??)?? ?? ????? 0??.
    //   ? ? ??? ID/EX ? ???? RegWrite/MemWrite ?? 0 ?? ?? ?? ??.
    //   ? ??? ???? ??? ? ?? ?? ????, Valid ??? ?? ????
    //     flush ???(2??/3??)? ??? ?? ??? ??.
    // =========================================================================
    wire        g_RegWrite   = Ctrl_RegWrite   & IF_ID_Valid;
    wire        g_MemRead     = Ctrl_MemRead    & IF_ID_Valid;
    wire        g_MemWrite    = Ctrl_MemWrite   & IF_ID_Valid;
    wire        g_MemToReg    = Ctrl_MemToReg   & IF_ID_Valid;
    wire        g_IPMem       = Ctrl_IPMem      & IF_ID_Valid;   // ?? ?? ??? IP ??? ??? ??
    wire        g_ALUSrc      = Ctrl_ALUSrc;                      // ??????, ???? ??
    wire [3:0]  g_ALUOp       = Ctrl_ALUOp;
    wire [2:0]  g_BranchType  = IF_ID_Valid ? Ctrl_BranchType : 3'b000; // ?? ??? ??? ??
    // =========================================================================
    //  - read ? ???, write ? ??
    //  - ???? R-type ? rb / rc ? ??? ??
    //  - Store(ST/STR) : ??? ?? ra ? ???? RA1 ?? ra ? ??? ?
    //  - Branch(BR/BRL) : ?? ??? rb (RA0), ?? ?? ??? rc (RA1)
    //  - LD ? ?? ??? ???? rb ? R[31] ?? ???? (zero-ext) ?? ??? ??? ???,
    //    EX ???? is_ld_abs ??? alu_op_a ? 0?? ???? ??? [FIX 2]
    //
    //  ? ?? ?? :
    //      Read_Addr_0 (RA0) = rb
    //      Read_Addr_1 (RA1) = (ST/STR ? ra : rc)

    wire [31:0] Rs1_Data_out, Rs2_Data_out;

    wire [4:0] Read_Addr_0 = rb;
    wire [4:0] Read_Addr_1 = (opcode == 5'd21 || opcode == 5'd22) ? ra : rc;

    // REGISTER FILE FOR GENERAL PURPOSE REGISTERS
    REGFILE  #(.AW(5), .ENTRY(32))  RegFile (
                    .CLK    (CLK),
                    .RSTN   (RSTN),
                    .WEN    (~WB_RegWrite),
                    .WA     (WB_Rd),
                    .DI     (WB_data),
                    .RA0    (Read_Addr_0),
                    .RA1    (Read_Addr_1),
                    .DOUT0  (Rs1_Data_out),
                    .DOUT1  (Rs2_Data_out),
                    .CONSIG (CONSIG)
    );


    // =========================================================================
    // [3.5] Load-Use Hazard Detection (Stall ??)
    //   ?? ???? LD/LDR (ID/EX.MemRead=1) ??, ? ?? ?????
    //   ?? ID ?? ???? ??? ?? 1 ??? stall.
    // =========================================================================
    //  ? ?? ID/EX ?? ?? ? detection ??? ?? ????? ?????,
    //    ??? ? ? ????? wire ? ?? ??? ? ?.
    wire        ID_EX_MemRead_w;
    wire [4:0]  ID_EX_Rd_num_w;

    // ?? ID ???? ?? ????? ??? (?? ?? ??)
    //  branch/store ?? ??? RA0 = rb, RA1 = ra(ST/STR) or rc
    //   RISC-TOY ? R0 ? zero register ? ?? ?? ???????
    //   ??? ??? 0??? stall ??? ?? ?.
    assign Stall_Flag = IF_ID_Valid && ID_EX_MemRead_w &&
                        ( (ID_EX_Rd_num_w == Read_Addr_0) ||
                          (ID_EX_Rd_num_w == Read_Addr_1) );


    // =========================================================================
    // [4] ID/EX ????? ????
    // =========================================================================
    reg [31:0] ID_EX_PC, ID_EX_Rs1_data, ID_EX_Rs2_data, ID_EX_Imm;
    reg [4:0]  ID_EX_Rs1_num, ID_EX_Rs2_num, ID_EX_Rd_num;
    reg [3:0]  ID_EX_ALUOp;
    reg [2:0]  ID_EX_BranchType, ID_EX_BR_Cond;
    reg        ID_EX_RegWrite, ID_EX_MemRead, ID_EX_MemWrite, ID_EX_MemToReg, ID_EX_ALUSrc;
    reg        ID_EX_IPMem;     // ?? [LDIP/STIP] ??? ??? CUSTOM_IP ??

    // ??? ?? ??? wire ?? ?? reg ??? ??
    //  [LDIP ??] LDIP ? RegWrite=0(????? ? ?)?? load-use ??? ??? ???.
    //  ? ??? ??? MemRead ? "????? ?? ?? load(LD/LDR)"? 1 ? ??? IPMem ? ??.
    assign ID_EX_MemRead_w = ID_EX_MemRead & ~ID_EX_IPMem;
    assign ID_EX_Rd_num_w  = ID_EX_Rd_num;

    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            ID_EX_PC        <= 32'd0;
            ID_EX_Rs1_data  <= 32'd0;
            ID_EX_Rs2_data  <= 32'd0;
            ID_EX_Imm       <= 32'd0;
            ID_EX_Rs1_num   <= 5'd0;
            ID_EX_Rs2_num   <= 5'd0;
            ID_EX_Rd_num    <= 5'd0;
            ID_EX_ALUOp     <= 4'b0000;
            ID_EX_BranchType<= 3'b000;
            ID_EX_BR_Cond   <= 3'b000;
            ID_EX_RegWrite  <= 1'b0;
            ID_EX_MemRead   <= 1'b0;
            ID_EX_MemWrite  <= 1'b0;
            ID_EX_MemToReg  <= 1'b0;
            ID_EX_ALUSrc    <= 1'b0;
            ID_EX_IPMem     <= 1'b0;
        end else if (Branch_Taken || Stall_Flag || Mem_Stall) begin
            // Branch_Taken(?? ?? ??) ? ID ??? ??? ?? ?? ??? (NOP)
            // Stall / Mem_Stall          ? ID/EX ? NOP (= bubble) ??
            // ? ?? ?? "??" ???? ???? IF_ID_Valid=0 ????? ?????
            //   ??? 3?? Extended_Flush ? ?? ???(?? ?? ?? ??).
            ID_EX_PC        <= 32'd0;
            ID_EX_Rs1_data  <= 32'd0;
            ID_EX_Rs2_data  <= 32'd0;
            ID_EX_Imm       <= 32'd0;
            ID_EX_Rs1_num   <= 5'd0;
            ID_EX_Rs2_num   <= 5'd0;
            ID_EX_Rd_num    <= 5'd0;
            ID_EX_ALUOp     <= 4'b0000;
            ID_EX_BranchType<= 3'b000;
            ID_EX_BR_Cond   <= 3'b000;
            ID_EX_RegWrite  <= 1'b0;
            ID_EX_MemRead   <= 1'b0;
            ID_EX_MemWrite  <= 1'b0;
            ID_EX_MemToReg  <= 1'b0;
            ID_EX_ALUSrc    <= 1'b0;
            ID_EX_IPMem     <= 1'b0;
        end else begin
            ID_EX_PC        <= IF_ID_PC;
            ID_EX_Rs1_data  <= Rs1_Data_out;
            ID_EX_Rs2_data  <= Rs2_Data_out;
            ID_EX_Imm       <= selected_imm;
            ID_EX_Rs1_num   <= Read_Addr_0;
            ID_EX_Rs2_num   <= Read_Addr_1;
            ID_EX_Rd_num    <= ra;                  // ???? ?? ra
            ID_EX_ALUOp     <= g_ALUOp;
            ID_EX_BranchType<= g_BranchType;        // ?? ?? ??? ?? ? ?
            ID_EX_BR_Cond   <= br_cond;
            ID_EX_RegWrite  <= g_RegWrite;          // ?? ?? ??? ???? ?? ? ?
            ID_EX_MemRead   <= g_MemRead;           // ??
            ID_EX_MemWrite  <= g_MemWrite;          // ??
            ID_EX_MemToReg  <= g_MemToReg;          // ??
            ID_EX_ALUSrc    <= g_ALUSrc;
            ID_EX_IPMem     <= g_IPMem;             // ?? LDIP/STIP ??
        end
    end


    // =========================================================================
    // [5] EX ?? (Execute) - Forwarding + ALU + Branch Unit
    // =========================================================================
    //  ??? ??? EX/MEM, MEM/WB ???? ??? ?? wire ? ??
    wire [31:0] EX_MEM_ALU_out;
    wire [4:0]  EX_MEM_Rd;
    wire        EX_MEM_Ctrl_RegWrite;
    wire [4:0]  MEM_WB_Rd;
    wire        MEM_WB_Ctrl_RegWrite;
    //  MEM/WB ? EX forwarding ??? WB_data ? ??? ??

    // ---- Forwarding Unit ----
    //   2'b10 : EX/MEM ? ALU ?? forward (?? ??)
    //   2'b01 : MEM/WB ? WB ?? forward
    //   2'b00 : ID/EX ?? ??? ?? ? ??
    wire [1:0] forward_A =
        ( EX_MEM_Ctrl_RegWrite && (EX_MEM_Rd == ID_EX_Rs1_num) ) ? 2'b10 :
        ( MEM_WB_Ctrl_RegWrite && (MEM_WB_Rd == ID_EX_Rs1_num) ) ? 2'b01 :
                                                                   2'b00 ;
    wire [1:0] forward_B =
        ( EX_MEM_Ctrl_RegWrite && (EX_MEM_Rd == ID_EX_Rs2_num) ) ? 2'b10 :
        ( MEM_WB_Ctrl_RegWrite && (MEM_WB_Rd == ID_EX_Rs2_num) ) ? 2'b01 :
                                                                   2'b00 ;

    // ---- Forwarding MUX ----
    reg [31:0] rs1_fwd, rs2_fwd;
    always @(*) begin
        case (forward_A)
            2'b10  : rs1_fwd = EX_MEM_ALU_out;
            2'b01  : rs1_fwd = WB_data;
            default: rs1_fwd = ID_EX_Rs1_data;
        endcase
        case (forward_B)
            2'b10  : rs2_fwd = EX_MEM_ALU_out;
            2'b01  : rs2_fwd = WB_data;
            default: rs2_fwd = ID_EX_Rs2_data;
        endcase
    end

    // ---- ALU ?? ?? ----
    //  LINK ?? (BRL/JL)  : op_a = currentPC (= ID_EX_PC) ? +4 ??? RegFile ? ?
    //  LD rb==R[31] (????) : op_a = 0  ? 0 + imm17 = ???? (??? ??)
    //  ? ?               : op_a = rs1_fwd
    wire is_link   = (ID_EX_ALUOp == 4'b1101);
    // [FIX 2] LD(opcode=19)?? ?? ???? ??? R[31]?? ???? ??
   wire is_ld_abs = (ID_EX_MemRead || ID_EX_MemWrite) && (ID_EX_Rs1_num == 5'd31);
    wire [31:0] alu_op_a = is_link   ? ID_EX_PC  :
                           is_ld_abs ? 32'd0      :
                                       rs1_fwd;
    wire [31:0] alu_op_b = ID_EX_ALUSrc ? ID_EX_Imm : rs2_fwd;

    // ---- ALU ----
    wire [4:0]  sh      = alu_op_b[4:0];
    wire [31:0] ror_val = (alu_op_a >> sh) | (alu_op_a << (32 - {27'd0, sh}));

    reg [31:0] alu_result;
    always @(*) begin
        case (ID_EX_ALUOp)
            4'b0000 : alu_result = alu_op_a + alu_op_b;                 // ADD
            4'b0001 : alu_result = alu_op_a - alu_op_b;                 // SUB
            4'b0010 : alu_result = ~alu_op_b + 32'd1;                   // NEG = -op_b
            4'b0011 : alu_result = ~alu_op_b;                           // NOT
            4'b0100 : alu_result = alu_op_a & alu_op_b;                 // AND
            4'b0101 : alu_result = alu_op_a | alu_op_b;                 // OR
            4'b0110 : alu_result = alu_op_a ^ alu_op_b;                 // XOR
            4'b0111 : alu_result = alu_op_a >> sh;                      // LSR
            4'b1000 : alu_result = $signed(alu_op_a) >>> sh;            // ASR
            4'b1001 : alu_result = alu_op_a << sh;                      // SHL
            4'b1010 : alu_result = (sh == 5'd0) ? alu_op_a : ror_val;   // ROR
            4'b1011 : alu_result = alu_op_b;                            // PASS_B (MOVI)
            4'b1100 : alu_result = alu_op_a;                            // PASS_A
            4'b1101 : alu_result = alu_op_a + 32'd4;                    // LINK : PC+4
            default : alu_result = 32'd0;
        endcase
    end

    // ---- Branch Unit ----
    //  branch_type ??? :
    //      000 : none / 001 : BR / 010 : BRL / 011 : J / 100 : JL
    //  br_cond :
    //      000 Never / 001 Always / 010 Zero / 011 Nonzero / 100 Plus / 101 Minus
    wire cond_ok = (ID_EX_BR_Cond == 3'b001) ? 1'b1 :
                   (ID_EX_BR_Cond == 3'b010) ? (rs2_fwd == 32'd0) :
                   (ID_EX_BR_Cond == 3'b011) ? (rs2_fwd != 32'd0) :
                   (ID_EX_BR_Cond == 3'b100) ? (rs2_fwd[31] == 1'b0) :
                   (ID_EX_BR_Cond == 3'b101) ? (rs2_fwd[31] == 1'b1) :
                                               1'b0;

    reg        branch_taken_r;
    reg [31:0] branch_target_r;
    always @(*) begin
        case (ID_EX_BranchType)
            3'b001, 3'b010 : begin                          // BR, BRL : PC = R[rb] = rs1_fwd
                branch_taken_r  = cond_ok;
                branch_target_r = rs1_fwd;
            end
            3'b011, 3'b100 : begin                          // J, JL : PC = currentPC + imm22
                branch_taken_r  = 1'b1;
                branch_target_r = ID_EX_PC + ID_EX_Imm;
            end
            default : begin
                branch_taken_r  = 1'b0;
                branch_target_r = 32'd0;
            end
        endcase
    end

    assign Branch_Taken   = branch_taken_r;
    assign Branch_Target  = branch_target_r;


    // =========================================================================
    // [6] EX/MEM ????? ????
    // =========================================================================
    reg [31:0] EX_MEM_ALU_out_r;
    reg [31:0] EX_MEM_Rs2_data_r;     // ST/STR ? ???? ? ? (forwarding ??)
    reg [4:0]  EX_MEM_Rd_r;
    reg        EX_MEM_Ctrl_RegWrite_r;
    reg        EX_MEM_Ctrl_MemRead_r;
    reg        EX_MEM_Ctrl_MemWrite_r;
    reg        EX_MEM_Ctrl_MemToReg_r;
    reg        EX_MEM_IPMem_r;        // ?? [LDIP/STIP]

    // [FIX 3] LD/LDR ?? read latency ??
    //   DATA_RAM ? ?? read(?? ??? ?? ???? DRDATA valid) + ATIME ??.
    //   LD ? MEM ??? ? ??? ???? DRDATA ? ?? ? ?(x) ???,
    //   ? ??? MEM/WB ? latch ?? load ??? ???.
    //   ? LD ? MEM ? ??? "?" ???? 1?? stall ? ??,
    //     ?? ??(=DRDATA valid)? MEM/WB ? ?? latch ?? ??.
    reg        Mem_Stall_done;   // ?? LD ? ?? ?? mem-stall ? 1? ???
    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) Mem_Stall_done <= 1'b0;
        else       Mem_Stall_done <= Mem_Stall; // ?? mem-stall ??? ?? ??? ??
    end
    //  [LDIP ??] LDIP ? MEM/WB ? ?? ????(???? ???) read latency ??
    //  stall ? ?? ??. ? IPMem ? read ? mem-stall ???? ??.
    assign Mem_Stall = EX_MEM_Ctrl_MemRead_r & ~EX_MEM_IPMem_r & ~Mem_Stall_done;
    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            EX_MEM_ALU_out_r       <= 32'd0;
            EX_MEM_Rs2_data_r      <= 32'd0;
            EX_MEM_Rd_r            <= 5'd0;
            EX_MEM_Ctrl_RegWrite_r <= 1'b0;
            EX_MEM_Ctrl_MemRead_r  <= 1'b0;
            EX_MEM_Ctrl_MemWrite_r <= 1'b0;
            EX_MEM_Ctrl_MemToReg_r <= 1'b0;
            EX_MEM_IPMem_r         <= 1'b0;
        // ?? [??2] ?? ??? ID/EX ???? ?? RegWrite/MemRead/MemWrite=0 ??
        //   ????????, ??? Extended_Flush ? ?? ??? ??? ??.
        //   (??? 3???? ??? Extended_Flush ? EX ??? "??" ???
        //    ?? ? ????, ?? ?? flush ??? ???? ??? ?????.)
        end else if (Mem_Stall) begin
            // [FIX 3] LD ? MEM ? ? ??? ?? : 1?? ? ??? ??.
            //   - ALU_out/Rd/RegWrite/MemToReg ? ?? (?? ?? MEM/WB ? latch)
            //   - MemRead ? 0 ?? ?? ?? ??? Mem_Stall ?? ??
            //     (read ??? ? ??? ?? DATA_RAM ?? ???)
            EX_MEM_ALU_out_r       <= EX_MEM_ALU_out_r;
            EX_MEM_Rs2_data_r      <= EX_MEM_Rs2_data_r;
            EX_MEM_Rd_r            <= EX_MEM_Rd_r;
            EX_MEM_Ctrl_RegWrite_r <= EX_MEM_Ctrl_RegWrite_r;
            EX_MEM_Ctrl_MemRead_r  <= 1'b0;
            EX_MEM_Ctrl_MemWrite_r <= 1'b0;
            EX_MEM_Ctrl_MemToReg_r <= EX_MEM_Ctrl_MemToReg_r;
            EX_MEM_IPMem_r         <= 1'b0;     // ??? ?? ???
        end else begin
            EX_MEM_ALU_out_r       <= alu_result;
            EX_MEM_Rs2_data_r      <= rs2_fwd;            // store data : forwarding ??
            EX_MEM_Rd_r            <= ID_EX_Rd_num;
            EX_MEM_Ctrl_RegWrite_r <= ID_EX_RegWrite;
            EX_MEM_Ctrl_MemRead_r  <= ID_EX_MemRead;
            EX_MEM_Ctrl_MemWrite_r <= ID_EX_MemWrite;
            EX_MEM_Ctrl_MemToReg_r <= ID_EX_MemToReg;
            EX_MEM_IPMem_r         <= ID_EX_IPMem;   // ?? LDIP/STIP ?? ??
        end
    end

    // EX ??? forwarding ??? wire ?? reg ?? ??
    assign EX_MEM_ALU_out       = EX_MEM_ALU_out_r;
    assign EX_MEM_Rd            = EX_MEM_Rd_r;
    assign EX_MEM_Ctrl_RegWrite = EX_MEM_Ctrl_RegWrite_r;


    // =========================================================================
    // [7] MEM ?? (Memory Access)
    // =========================================================================
    //   DATA_MEM ????? :
    //      DREQ = MemRead | MemWrite
    //      DRW  : 2'b10 LD(????CPU) / 2'b11 LDIP(????IP)
    //             2'b00 ST(CPU????) / 2'b01 STIP(IP IPOUT????)
    //      DADDR = ALU_out[31:2]  (word address)
    //      DWDATA = Rs2_data (store data, ST/STR ?? ? STIP ? DI2=IPOUT ??)
    assign DREQ   = EX_MEM_Ctrl_MemRead_r | EX_MEM_Ctrl_MemWrite_r;
    assign DRW    = (EX_MEM_Ctrl_MemRead_r  & ~EX_MEM_IPMem_r) ? 2'b10 :   // LD
                    (EX_MEM_Ctrl_MemRead_r  &  EX_MEM_IPMem_r) ? 2'b11 :   // LDIP
                    (EX_MEM_Ctrl_MemWrite_r & ~EX_MEM_IPMem_r) ? 2'b00 :   // ST
                    (EX_MEM_Ctrl_MemWrite_r &  EX_MEM_IPMem_r) ? 2'b01 :   // STIP
                                                                 2'b00 ;
    assign DADDR  = EX_MEM_ALU_out_r[29:0];
    assign DWDATA = EX_MEM_Rs2_data_r;

    // MEM ??? ????? EX_MEM ????? ??? ??? ????? ??
    //   ? ?? ???? ?? ?? MEM/WB ?????? latch


    // =========================================================================
    // [8] MEM/WB ????? ????
    // =========================================================================
    //   DRDATA ? ?? read ? ?? ??? valid ? MEM/WB ????? ?? ??? latch
    reg [31:0] MEM_WB_Mem_data_r;
    reg [31:0] MEM_WB_ALU_out_r;
    reg [4:0]  MEM_WB_Rd_r;
    reg        MEM_WB_Ctrl_RegWrite_r;
    reg        MEM_WB_Ctrl_MemToReg_r;

    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            MEM_WB_Mem_data_r       <= 32'd0;
            MEM_WB_ALU_out_r        <= 32'd0;
            MEM_WB_Rd_r             <= 5'd0;
            MEM_WB_Ctrl_RegWrite_r  <= 1'b0;
            MEM_WB_Ctrl_MemToReg_r  <= 1'b0;
        end else if (Mem_Stall) begin
            // [FIX 3] mem-stall ? ?? : DRDATA ?? ?? ? MEM/WB ? ??(WB ??).
            //   ?? ??(stall ??)? valid DRDATA + ??? LD ????? ?? latch ?.
            MEM_WB_Mem_data_r       <= 32'd0;
            MEM_WB_ALU_out_r        <= 32'd0;
            MEM_WB_Rd_r             <= 5'd0;
            MEM_WB_Ctrl_RegWrite_r  <= 1'b0;
            MEM_WB_Ctrl_MemToReg_r  <= 1'b0;
        end else begin
            MEM_WB_Mem_data_r       <= DRDATA;
            MEM_WB_ALU_out_r        <= EX_MEM_ALU_out_r;
            MEM_WB_Rd_r             <= EX_MEM_Rd_r;
            MEM_WB_Ctrl_RegWrite_r  <= EX_MEM_Ctrl_RegWrite_r;
            MEM_WB_Ctrl_MemToReg_r  <= EX_MEM_Ctrl_MemToReg_r;
        end
    end

    assign MEM_WB_Rd            = MEM_WB_Rd_r;
    assign MEM_WB_Ctrl_RegWrite = MEM_WB_Ctrl_RegWrite_r;


    // =========================================================================
    // [9] WB ?? (Write Back) - MUX
    // =========================================================================
    //   MemToReg = 0 ? ALU_out  (R-type, ADDI, BRL/JL link ?)
    //   MemToReg = 1 ? Mem_data (LD, LDR)
    assign WB_data     = MEM_WB_Ctrl_MemToReg_r ? MEM_WB_Mem_data_r : MEM_WB_ALU_out_r;
    assign WB_Rd       = MEM_WB_Rd_r;
    assign WB_RegWrite = MEM_WB_Ctrl_RegWrite_r;


endmodule
