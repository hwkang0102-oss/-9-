/*****************************************
    Team XX : 
        2023104125    장세은
        2021103504    이경태
        2021104197    강현우

    [INTEGRATED VERSION - FINAL]
    - IF / ID / EX / MEM / WB 5-stage 파이프라인
    - Forwarding, Load-Use Stall, Branch Flush (2-Cycle + Rescue Buffer)
    - Custom IP 명령어 (LDIP, STIP) 지원 추가 완료
*****************************************/

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
    // 미리 선언 : 뒤쪽 단계에서 만들어져 앞쪽으로 피드백되는 신호들
    // =========================================================================
    wire [31:0] WB_data;
    wire [4:0]  WB_Rd;
    wire        WB_RegWrite;

    wire        Branch_Taken;
    wire [31:0] Branch_Target;
    wire        Stall_Flag;
    
    // 🌟 2-Cycle Flush 로직
    reg         Flush_Delay;
    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            Flush_Delay <= 1'b0;
        end else begin
            Flush_Delay <= Branch_Taken;
        end
    end
    wire        Extended_Flush = Branch_Taken | Flush_Delay;

    // =========================================================================
    // [1] IF 단계 (Instruction Fetch) & 🌟 Stall 구출 버퍼
    // =========================================================================
    reg [31:0] PC;

    reg [31:0]  INSTR_reg;
    reg         Was_Stalled;
    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            INSTR_reg   <= 32'd0;
            Was_Stalled <= 1'b0;
        end else begin
            INSTR_reg   <= INSTR;
            Was_Stalled <= Stall_Flag;
        end
    end
    
    wire [31:0] safe_INSTR = Was_Stalled ? INSTR_reg : INSTR;
    wire [31:0] safe_PC    = Was_Stalled ? PC : PC;

    wire [31:0] next_PC = Branch_Taken ? Branch_Target : (PC + 32'd4);

    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            PC <= 32'd0;
        end else if (!Stall_Flag) begin
            PC <= next_PC;
        end
    end

    assign IREQ  = 1'b1;
    assign IADDR = PC[31:2];

    // --- IF/ID 파이프라인 레지스터 ---
    reg [31:0] IF_ID_PC;
    reg [31:0] IF_ID_Inst;

    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            IF_ID_PC   <= 32'd0;
            IF_ID_Inst <= 32'd0;
        end else if (Extended_Flush) begin   
            IF_ID_PC   <= 32'd0;
            IF_ID_Inst <= 32'd0;
        end else if (Stall_Flag) begin
            IF_ID_PC   <= IF_ID_PC;
            IF_ID_Inst <= IF_ID_Inst;
        end else begin
            IF_ID_PC   <= safe_PC;
            IF_ID_Inst <= safe_INSTR;
        end
    end


    // =========================================================================
    // [2] ID 단계 (Instruction Decode)
    // =========================================================================
    wire [4:0]  opcode = IF_ID_Inst[31:27];
    wire [4:0]  ra     = IF_ID_Inst[26:22];
    wire [4:0]  rb     = IF_ID_Inst[21:17];
    wire [4:0]  rc     = IF_ID_Inst[16:12];
    wire [16:0] imm17  = IF_ID_Inst[16:0];
    wire [21:0] imm22  = IF_ID_Inst[21:0];
    wire [2:0]  br_cond= IF_ID_Inst[2:0];

    wire [31:0] sign_ext_17 = {{15{imm17[16]}}, imm17};
    wire [31:0] sign_ext_22 = {{10{imm22[21]}}, imm22};

    wire use_imm22 = (opcode == 5'd17) || (opcode == 5'd18) ||
                     (opcode == 5'd20) || (opcode == 5'd22);
    wire [31:0] selected_imm = use_imm22 ? sign_ext_22 : sign_ext_17;

    // --- 제어부 (Control Unit) ---
    reg        Ctrl_RegWrite, Ctrl_MemRead, Ctrl_MemWrite, Ctrl_MemToReg, Ctrl_ALUSrc;
    reg [3:0]  Ctrl_ALUOp;
    reg [2:0]  Ctrl_BranchType;
    reg        Ctrl_LDIP, Ctrl_STIP;  // 🌟 [추가됨] Custom IP 제어 신호

    always @(*) begin
        Ctrl_RegWrite   = 1'b0;
        Ctrl_MemRead    = 1'b0;
        Ctrl_MemWrite   = 1'b0;
        Ctrl_MemToReg   = 1'b0;
        Ctrl_ALUSrc     = 1'b0;
        Ctrl_ALUOp      = 4'b0000;
        Ctrl_BranchType = 3'b000;
        Ctrl_LDIP       = 1'b0;      // 🌟 초기화
        Ctrl_STIP       = 1'b0;      // 🌟 초기화

        case (opcode)
            // ---- 기존 명령어들 ----
            5'd0 : begin Ctrl_RegWrite = 1; Ctrl_ALUSrc = 1; Ctrl_ALUOp = 4'b0000; end // ADDI
            5'd1 : begin Ctrl_RegWrite = 1; Ctrl_ALUSrc = 1; Ctrl_ALUOp = 4'b0100; end // ANDI
            5'd2 : begin Ctrl_RegWrite = 1; Ctrl_ALUSrc = 1; Ctrl_ALUOp = 4'b0101; end // ORI
            5'd3 : begin Ctrl_RegWrite = 1; Ctrl_ALUSrc = 1; Ctrl_ALUOp = 4'b1011; end // MOVI
            5'd4 : begin Ctrl_RegWrite = 1; Ctrl_ALUSrc = 0; Ctrl_ALUOp = 4'b0000; end // ADD
            5'd5 : begin Ctrl_RegWrite = 1; Ctrl_ALUSrc = 0; Ctrl_ALUOp = 4'b0001; end // SUB
            5'd6 : begin Ctrl_RegWrite = 1; Ctrl_ALUSrc = 0; Ctrl_ALUOp = 4'b0010; end // NEG
            5'd7 : begin Ctrl_RegWrite = 1; Ctrl_ALUSrc = 0; Ctrl_ALUOp = 4'b0011; end // NOT
            5'd8 : begin Ctrl_RegWrite = 1; Ctrl_ALUSrc = 0; Ctrl_ALUOp = 4'b0100; end // AND
            5'd9 : begin Ctrl_RegWrite = 1; Ctrl_ALUSrc = 0; Ctrl_ALUOp = 4'b0101; end // OR
            5'd10: begin Ctrl_RegWrite = 1; Ctrl_ALUSrc = 0; Ctrl_ALUOp = 4'b0110; end // XOR
            5'd11: begin Ctrl_RegWrite = 1; Ctrl_ALUSrc = 0; Ctrl_ALUOp = 4'b0111; end // LSR
            5'd12: begin Ctrl_RegWrite = 1; Ctrl_ALUSrc = 0; Ctrl_ALUOp = 4'b1000; end // ASR
            5'd13: begin Ctrl_RegWrite = 1; Ctrl_ALUSrc = 0; Ctrl_ALUOp = 4'b1001; end // SHL
            5'd14: begin Ctrl_RegWrite = 1; Ctrl_ALUSrc = 0; Ctrl_ALUOp = 4'b1010; end // ROR
            5'd15: begin Ctrl_BranchType = 3'b001; end // BR
            5'd16: begin Ctrl_BranchType = 3'b010; Ctrl_RegWrite = 1; Ctrl_ALUOp = 4'b1101; end // BRL
            5'd17: begin Ctrl_BranchType = 3'b011; end // J
            5'd18: begin Ctrl_BranchType = 3'b100; Ctrl_RegWrite = 1; Ctrl_ALUOp = 4'b1101; end // JL
            5'd19: begin Ctrl_RegWrite = 1; Ctrl_MemRead = 1; Ctrl_MemToReg = 1; Ctrl_ALUSrc = 1; Ctrl_ALUOp = 4'b0000; end // LD
            5'd20: begin Ctrl_RegWrite = 1; Ctrl_MemRead = 1; Ctrl_MemToReg = 1; Ctrl_ALUSrc = 1; Ctrl_ALUOp = 4'b0000; end // LDR
            5'd21: begin Ctrl_MemWrite = 1; Ctrl_ALUSrc = 1; Ctrl_ALUOp = 4'b0000; end // ST
            5'd22: begin Ctrl_MemWrite = 1; Ctrl_ALUSrc = 1; Ctrl_ALUOp = 4'b0000; end // STR

            // 🌟 [추가됨] Custom IP 명령어
            5'd23: begin // LDIP: 메모리에서 값을 읽어 IPIN으로 줌 (레지스터에는 저장 안 함)
                Ctrl_LDIP = 1; Ctrl_ALUSrc = 1; Ctrl_ALUOp = 4'b0000;
            end
            5'd24: begin // STIP: IPOUT의 값을 메모리에 씀
                Ctrl_STIP = 1; Ctrl_ALUSrc = 1; Ctrl_ALUOp = 4'b0000;
            end

            default: ; // NOP
        endcase
    end

    // =========================================================================
    // [3] 레지스터 파일 (REGFILE) 연결
    // =========================================================================
    wire [31:0] Rs1_Data_out, Rs2_Data_out;
    wire [4:0] Read_Addr_0 = rb;
    wire [4:0] Read_Addr_1 = (opcode == 5'd21 || opcode == 5'd22) ? ra : rc;

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
    // [3.5] Load-Use Hazard Detection
    // =========================================================================
    wire        ID_EX_MemRead_w;
    wire [4:0]  ID_EX_Rd_num_w;
    assign Stall_Flag = ID_EX_MemRead_w &&
                        ( (ID_EX_Rd_num_w == Read_Addr_0) ||
                          (ID_EX_Rd_num_w == Read_Addr_1) );


    // =========================================================================
    // [4] ID/EX 파이프라인 레지스터
    // =========================================================================
    reg [31:0] ID_EX_PC, ID_EX_Rs1_data, ID_EX_Rs2_data, ID_EX_Imm;
    reg [4:0]  ID_EX_Rs1_num, ID_EX_Rs2_num, ID_EX_Rd_num;
    reg [3:0]  ID_EX_ALUOp;
    reg [2:0]  ID_EX_BranchType, ID_EX_BR_Cond;
    reg        ID_EX_RegWrite, ID_EX_MemRead, ID_EX_MemWrite, ID_EX_MemToReg, ID_EX_ALUSrc;
    reg        ID_EX_LDIP, ID_EX_STIP; // 🌟 [추가됨]

    assign ID_EX_MemRead_w = ID_EX_MemRead;
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
            ID_EX_LDIP      <= 1'b0; // 🌟
            ID_EX_STIP      <= 1'b0; // 🌟
        end else if (Extended_Flush || Stall_Flag) begin
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
            ID_EX_LDIP      <= 1'b0; // 🌟
            ID_EX_STIP      <= 1'b0; // 🌟
        end else begin
            ID_EX_PC        <= IF_ID_PC;
            ID_EX_Rs1_data  <= Rs1_Data_out;
            ID_EX_Rs2_data  <= Rs2_Data_out;
            ID_EX_Imm       <= selected_imm;
            ID_EX_Rs1_num   <= Read_Addr_0;
            ID_EX_Rs2_num   <= Read_Addr_1;
            ID_EX_Rd_num    <= ra;
            ID_EX_ALUOp     <= Ctrl_ALUOp;
            ID_EX_BranchType<= Ctrl_BranchType;
            ID_EX_BR_Cond   <= br_cond;
            ID_EX_RegWrite  <= Ctrl_RegWrite;
            ID_EX_MemRead   <= Ctrl_MemRead;
            ID_EX_MemWrite  <= Ctrl_MemWrite;
            ID_EX_MemToReg  <= Ctrl_MemToReg;
            ID_EX_ALUSrc    <= Ctrl_ALUSrc;
            ID_EX_LDIP      <= Ctrl_LDIP; // 🌟
            ID_EX_STIP      <= Ctrl_STIP; // 🌟
        end
    end


    // =========================================================================
    // [5] EX 단계 (Execute) - Forwarding + ALU + Branch Unit
    // =========================================================================
    wire [31:0] EX_MEM_ALU_out;
    wire [4:0]  EX_MEM_Rd;
    wire        EX_MEM_Ctrl_RegWrite;
    wire [4:0]  MEM_WB_Rd;
    wire        MEM_WB_Ctrl_RegWrite;

    // ---- Forwarding Unit ----
    wire [1:0] forward_A =
        ( EX_MEM_Ctrl_RegWrite && (EX_MEM_Rd == ID_EX_Rs1_num) ) ? 2'b10 :
        ( MEM_WB_Ctrl_RegWrite && (MEM_WB_Rd == ID_EX_Rs1_num) ) ? 2'b01 : 2'b00;
    wire [1:0] forward_B =
        ( EX_MEM_Ctrl_RegWrite && (EX_MEM_Rd == ID_EX_Rs2_num) ) ? 2'b10 :
        ( MEM_WB_Ctrl_RegWrite && (MEM_WB_Rd == ID_EX_Rs2_num) ) ? 2'b01 : 2'b00;

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

    wire is_link = (ID_EX_ALUOp == 4'b1101);
    wire [31:0] alu_op_a = is_link ? ID_EX_PC : rs1_fwd;
    wire [31:0] alu_op_b = ID_EX_ALUSrc ? ID_EX_Imm : rs2_fwd;

    wire [4:0]  sh      = alu_op_b[4:0];
    wire [31:0] ror_val = (alu_op_a >> sh) | (alu_op_a << (32 - {27'd0, sh}));

    reg [31:0] alu_result;
    always @(*) begin
        case (ID_EX_ALUOp)
            4'b0000 : alu_result = alu_op_a + alu_op_b;
            4'b0001 : alu_result = alu_op_a - alu_op_b;
            4'b0010 : alu_result = ~alu_op_b + 32'd1;
            4'b0011 : alu_result = ~alu_op_b;
            4'b0100 : alu_result = alu_op_a & alu_op_b;
            4'b0101 : alu_result = alu_op_a | alu_op_b;
            4'b0110 : alu_result = alu_op_a ^ alu_op_b;
            4'b0111 : alu_result = alu_op_a >> sh;
            4'b1000 : alu_result = $signed(alu_op_a) >>> sh;
            4'b1001 : alu_result = alu_op_a << sh;
            4'b1010 : alu_result = (sh == 5'd0) ? alu_op_a : ror_val;
            4'b1011 : alu_result = alu_op_b;
            4'b1100 : alu_result = alu_op_a;
            4'b1101 : alu_result = alu_op_a + 32'd4;
            default : alu_result = 32'd0;
        endcase
    end

    // ---- Branch Unit ----
    wire cond_ok = (ID_EX_BR_Cond == 3'b001) ? 1'b1 :
                   (ID_EX_BR_Cond == 3'b010) ? (rs2_fwd == 32'd0) :
                   (ID_EX_BR_Cond == 3'b011) ? (rs2_fwd != 32'd0) :
                   (ID_EX_BR_Cond == 3'b100) ? (rs2_fwd[31] == 1'b0) :
                   (ID_EX_BR_Cond == 3'b101) ? (rs2_fwd[31] == 1'b1) : 1'b0;

    reg        branch_taken_r;
    reg [31:0] branch_target_r;
    always @(*) begin
        case (ID_EX_BranchType)
            3'b001, 3'b010 : begin branch_taken_r = cond_ok; branch_target_r = rs1_fwd; end
            3'b011, 3'b100 : begin branch_taken_r = 1'b1; branch_target_r = ID_EX_PC + ID_EX_Imm; end
            default : begin branch_taken_r = 1'b0; branch_target_r = 32'd0; end
        endcase
    end

    assign Branch_Taken  = branch_taken_r;
    assign Branch_Target = branch_target_r;


    // =========================================================================
    // [6] EX/MEM 파이프라인 레지스터
    // =========================================================================
    reg [31:0] EX_MEM_ALU_out_r;
    reg [31:0] EX_MEM_Rs2_data_r;
    reg [4:0]  EX_MEM_Rd_r;
    reg        EX_MEM_Ctrl_RegWrite_r;
    reg        EX_MEM_Ctrl_MemRead_r;
    reg        EX_MEM_Ctrl_MemWrite_r;
    reg        EX_MEM_Ctrl_MemToReg_r;
    reg        EX_MEM_Ctrl_LDIP_r, EX_MEM_Ctrl_STIP_r; // 🌟 [추가됨]

    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            EX_MEM_ALU_out_r       <= 32'd0;
            EX_MEM_Rs2_data_r      <= 32'd0;
            EX_MEM_Rd_r            <= 5'd0;
            EX_MEM_Ctrl_RegWrite_r <= 1'b0;
            EX_MEM_Ctrl_MemRead_r  <= 1'b0;
            EX_MEM_Ctrl_MemWrite_r <= 1'b0;
            EX_MEM_Ctrl_MemToReg_r <= 1'b0;
            EX_MEM_Ctrl_LDIP_r     <= 1'b0; // 🌟
            EX_MEM_Ctrl_STIP_r     <= 1'b0; // 🌟
        end else begin
            EX_MEM_ALU_out_r       <= alu_result;
            EX_MEM_Rs2_data_r      <= rs2_fwd;
            EX_MEM_Rd_r            <= ID_EX_Rd_num;
            EX_MEM_Ctrl_RegWrite_r <= ID_EX_RegWrite;
            EX_MEM_Ctrl_MemRead_r  <= ID_EX_MemRead;
            EX_MEM_Ctrl_MemWrite_r <= ID_EX_MemWrite;
            EX_MEM_Ctrl_MemToReg_r <= ID_EX_MemToReg;
            EX_MEM_Ctrl_LDIP_r     <= ID_EX_LDIP; // 🌟
            EX_MEM_Ctrl_STIP_r     <= ID_EX_STIP; // 🌟
        end
    end

    assign EX_MEM_ALU_out       = EX_MEM_ALU_out_r;
    assign EX_MEM_Rd            = EX_MEM_Rd_r;
    assign EX_MEM_Ctrl_RegWrite = EX_MEM_Ctrl_RegWrite_r;


    // =========================================================================
    // [7] MEM 단계 (Memory Access) - 🌟 LDIP / STIP 로직 반영 완벽 적용
    // =========================================================================
    // 메모리에 접근해야 하는 경우는 LD(Read), ST(Write), LDIP, STIP 총 4가지입니다.
    assign DREQ   = EX_MEM_Ctrl_MemRead_r | EX_MEM_Ctrl_MemWrite_r | 
                    EX_MEM_Ctrl_LDIP_r    | EX_MEM_Ctrl_STIP_r;

    // DRW 매뉴얼 규격: LDIP(11), LD(10), STIP(01), ST(00)
    assign DRW    = EX_MEM_Ctrl_LDIP_r     ? 2'b11 :
                    EX_MEM_Ctrl_STIP_r     ? 2'b01 :
                    EX_MEM_Ctrl_MemRead_r  ? 2'b10 :
                    EX_MEM_Ctrl_MemWrite_r ? 2'b00 :
                                             2'b00 ;
                                             
    assign DADDR  = EX_MEM_ALU_out_r[31:2];
    assign DWDATA = EX_MEM_Rs2_data_r;


    // =========================================================================
    // [8] MEM/WB 파이프라인 레지스터
    // =========================================================================
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
    // [9] WB 단계 (Write Back)
    // =========================================================================
    assign WB_data     = MEM_WB_Ctrl_MemToReg_r ? MEM_WB_Mem_data_r : MEM_WB_ALU_out_r;
    assign WB_Rd       = MEM_WB_Rd_r;
    assign WB_RegWrite = MEM_WB_Ctrl_RegWrite_r;

endmodule
