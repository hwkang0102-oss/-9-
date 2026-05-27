/*****************************************
    
    Team XX : 
        2023104125    장세은
        2021103504    이경태
     	2021104197	  강현우

    [INTEGRATED VERSION]
    - IF / ID / EX / MEM / WB 5-stage 파이프라인을 RISC_TOY 모듈 하나로 통합
    - 외부 보조 모듈 없이 동작 (REGFILE 만 외부 model.v 사용)
    - Forwarding (EX/MEM, MEM/WB → EX), Load-Use Stall, Branch Flush 포함

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
    // 미리 선언 : 뒤쪽 단계에서 만들어져 앞쪽으로 피드백되는 신호들
    // =========================================================================
    // WB 단계에서 RegFile 로 보내는 신호
    wire [31:0] WB_data;
    wire [4:0]  WB_Rd;
    wire        WB_RegWrite;

    // Branch / Hazard 신호 (EX 단계에서 결정)
    wire        Branch_Taken;
    wire [31:0] Branch_Target;

    // Load-Use Hazard 로 인한 Stall (ID 단계에서 결정)
    wire        Stall_Flag;
    // Branch taken 시 IF/ID, ID/EX 를 Flush
    wire        Flush_Flag;
    assign      Flush_Flag = Branch_Taken;


    // =========================================================================
    // [1] IF 단계 (Instruction Fetch)
    // =========================================================================
    reg [31:0] PC;

    // 다음 PC 선택 : branch 우선, 아니면 PC+4
    wire [31:0] next_PC = Branch_Taken ? Branch_Target : (PC + 32'd4);

    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            PC <= 32'd0;            // 매뉴얼 : 리셋 시 PC=0
        end else if (!Stall_Flag) begin
            PC <= next_PC;
        end
        // Stall 시에는 PC 유지
    end

    assign IREQ  = 1'b1;
    assign IADDR = PC[31:2];        // word address (30-bit)


    // --- IF/ID 파이프라인 레지스터 ---
    reg [31:0] IF_ID_PC;
    reg [31:0] IF_ID_Inst;

    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            IF_ID_PC   <= 32'd0;
            IF_ID_Inst <= 32'd0;
        end else if (Flush_Flag) begin
            // Branch taken → IF 에 잘못 들어온 명령어 무효화 (NOP)
            IF_ID_PC   <= 32'd0;
            IF_ID_Inst <= 32'd0;
        end else if (Stall_Flag) begin
            // Stall : IF/ID 유지 (같은 명령어를 한 번 더 보게 함)
            IF_ID_PC   <= IF_ID_PC;
            IF_ID_Inst <= IF_ID_Inst;
        end else begin
            IF_ID_PC   <= PC;
            IF_ID_Inst <= INSTR;
        end
    end


    // =========================================================================
    // [2] ID 단계 (Instruction Decode)
    // =========================================================================
    // 매뉴얼 규격에 따른 필드 분리
    wire [4:0]  opcode = IF_ID_Inst[31:27];
    wire [4:0]  ra     = IF_ID_Inst[26:22];
    wire [4:0]  rb     = IF_ID_Inst[21:17];
    wire [4:0]  rc     = IF_ID_Inst[16:12];
    wire [16:0] imm17  = IF_ID_Inst[16:0];
    wire [21:0] imm22  = IF_ID_Inst[21:0];
    wire [2:0]  br_cond = IF_ID_Inst[2:0];      // BR/BRL 의 cond 필드

    // 부호 확장
    wire [31:0] sign_ext_17 = {{15{imm17[16]}}, imm17};
    wire [31:0] sign_ext_22 = {{10{imm22[21]}}, imm22};

    // 명령어에 따라 17비트 / 22비트 Immediate 선택
    //   imm22 사용 : J(17), JL(18), LDR(20), STR(22)
    wire use_imm22 = (opcode == 5'd17) || (opcode == 5'd18) ||
                     (opcode == 5'd20) || (opcode == 5'd22);
    wire [31:0] selected_imm = use_imm22 ? sign_ext_22 : sign_ext_17;

    // --- 제어부 (Control Unit) ---
    reg        Ctrl_RegWrite, Ctrl_MemRead, Ctrl_MemWrite, Ctrl_MemToReg, Ctrl_ALUSrc;
    reg [3:0]  Ctrl_ALUOp;             // EX_stage 의 4-bit ALU op encoding 으로 변환
    reg [2:0]  Ctrl_BranchType;        // 000:none / 001:BR / 010:BRL / 011:J / 100:JL

    // ALU op encoding (EX 단계와 일치)
    //   0000:ADD 0001:SUB 0010:NEG 0011:NOT 0100:AND 0101:OR 0110:XOR
    //   0111:LSR 1000:ASR 1001:SHL 1010:ROR 1011:PASS_B 1101:LINK(PC+4)
    always @(*) begin
        // default
        Ctrl_RegWrite   = 1'b0;
        Ctrl_MemRead    = 1'b0;
        Ctrl_MemWrite   = 1'b0;
        Ctrl_MemToReg   = 1'b0;
        Ctrl_ALUSrc     = 1'b0;
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
            5'd6 : begin  // NEG : R[ra] = -R[rb] → ALU 입력에서 op_b = R[rb]
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

            default: ; // NOP
        endcase
    end

    // =========================================================================
    // [3] 레지스터 파일 (REGFILE) 연결
    // =========================================================================
    //  - read 는 비동기, write 는 동기
    //  - 대부분의 R-type 은 rb / rc 를 소스로 사용
    //  - Store(ST/STR) : 저장할 값이 ra 에 있으므로 RA1 에서 ra 를 읽어야 함
    //  - Branch(BR/BRL) : 분기 주소가 rb (RA0), 조건 비교 대상이 rc (RA1)
    //  - LD 의 경우 베이스 레지스터 rb 가 R[31] 이면 절대주소 (zero-ext) 라는 매뉴얼 규정이 있으나,
    //    여기서는 일반적인 R[rb]+imm 동작으로 처리 (R[31] 값이 적절히 0 처리되도록 사용자 책임)
    //
    //  → 통합 정책 :
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
    // [3.5] Load-Use Hazard Detection (Stall 발생)
    //   직전 명령어가 LD/LDR (ID/EX.MemRead=1) 이고, 그 결과 레지스터를
    //   현재 ID 단계 명령어가 소스로 쓰면 1 사이클 stall.
    // =========================================================================
    //  → 아래 ID/EX 신호 선언 후 detection 로직을 다시 본격적으로 구현하지만,
    //    여기서 한 번 사용하므로 wire 로 미리 선언만 해 둠.
    wire        ID_EX_MemRead_w;
    wire [4:0]  ID_EX_Rd_num_w;

    // 현재 ID 명령어가 어떤 레지스터를 읽는지 (위와 동일 정책)
    //  branch/store 까지 포함해 RA0 = rb, RA1 = ra(ST/STR) or rc
    //   RISC-TOY 는 R0 가 zero register 가 아닌 일반 레지스터이므로
    //   목적지 번호가 0이어도 stall 검사를 해야 함.
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

    // 위에서 미리 선언한 wire 들을 실제 reg 값으로 연결
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
        end else if (Flush_Flag || Stall_Flag) begin
            // Branch taken → 잘못 fetch 된 명령어 무력화 (NOP 주입)
            // Stall      → ID/EX 에 NOP (= bubble) 주입
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
        end else begin
            ID_EX_PC        <= IF_ID_PC;
            ID_EX_Rs1_data  <= Rs1_Data_out;
            ID_EX_Rs2_data  <= Rs2_Data_out;
            ID_EX_Imm       <= selected_imm;
            ID_EX_Rs1_num   <= Read_Addr_0;
            ID_EX_Rs2_num   <= Read_Addr_1;
            ID_EX_Rd_num    <= ra;                  // 목적지는 항상 ra
            ID_EX_ALUOp     <= Ctrl_ALUOp;
            ID_EX_BranchType<= Ctrl_BranchType;
            ID_EX_BR_Cond   <= br_cond;
            ID_EX_RegWrite  <= Ctrl_RegWrite;
            ID_EX_MemRead   <= Ctrl_MemRead;
            ID_EX_MemWrite  <= Ctrl_MemWrite;
            ID_EX_MemToReg  <= Ctrl_MemToReg;
            ID_EX_ALUSrc    <= Ctrl_ALUSrc;
        end
    end


    // =========================================================================
    // [5] EX 단계 (Execute) - Forwarding + ALU + Branch Unit
    // =========================================================================
    //  뒤에서 선언될 EX/MEM, MEM/WB 레지스터 값들을 미리 wire 로 선언
    wire [31:0] EX_MEM_ALU_out;
    wire [4:0]  EX_MEM_Rd;
    wire        EX_MEM_Ctrl_RegWrite;
    wire [4:0]  MEM_WB_Rd;
    wire        MEM_WB_Ctrl_RegWrite;
    //  MEM/WB → EX forwarding 경로는 WB_data 를 그대로 사용

    // ---- Forwarding Unit ----
    //   2'b10 : EX/MEM 의 ALU 결과 forward (가장 최신)
    //   2'b01 : MEM/WB 의 WB 결과 forward
    //   2'b00 : ID/EX 에서 들어온 원래 값 사용
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

    // ---- ALU 입력 선택 ----
    //  LINK 명령 (BRL/JL) : op_a = currentPC (= ID_EX_PC) → +4 결과를 RegFile 에 씀
    //  그 외 : op_a = rs1_fwd
    wire is_link = (ID_EX_ALUOp == 4'b1101);
    wire [31:0] alu_op_a = is_link ? ID_EX_PC : rs1_fwd;
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
    //  branch_type 인코딩 :
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
    // [6] EX/MEM 파이프라인 레지스터
    // =========================================================================
    reg [31:0] EX_MEM_ALU_out_r;
    reg [31:0] EX_MEM_Rs2_data_r;     // ST/STR 시 메모리에 쓸 값 (forwarding 반영)
    reg [4:0]  EX_MEM_Rd_r;
    reg        EX_MEM_Ctrl_RegWrite_r;
    reg        EX_MEM_Ctrl_MemRead_r;
    reg        EX_MEM_Ctrl_MemWrite_r;
    reg        EX_MEM_Ctrl_MemToReg_r;

    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            EX_MEM_ALU_out_r       <= 32'd0;
            EX_MEM_Rs2_data_r      <= 32'd0;
            EX_MEM_Rd_r            <= 5'd0;
            EX_MEM_Ctrl_RegWrite_r <= 1'b0;
            EX_MEM_Ctrl_MemRead_r  <= 1'b0;
            EX_MEM_Ctrl_MemWrite_r <= 1'b0;
            EX_MEM_Ctrl_MemToReg_r <= 1'b0;
        end else begin
            EX_MEM_ALU_out_r       <= alu_result;
            EX_MEM_Rs2_data_r      <= rs2_fwd;            // store data : forwarding 반영
            EX_MEM_Rd_r            <= ID_EX_Rd_num;
            EX_MEM_Ctrl_RegWrite_r <= ID_EX_RegWrite;
            EX_MEM_Ctrl_MemRead_r  <= ID_EX_MemRead;
            EX_MEM_Ctrl_MemWrite_r <= ID_EX_MemWrite;
            EX_MEM_Ctrl_MemToReg_r <= ID_EX_MemToReg;
        end
    end

    // EX 단계의 forwarding 검출용 wire 들을 reg 값에 연결
    assign EX_MEM_ALU_out       = EX_MEM_ALU_out_r;
    assign EX_MEM_Rd            = EX_MEM_Rd_r;
    assign EX_MEM_Ctrl_RegWrite = EX_MEM_Ctrl_RegWrite_r;


    // =========================================================================
    // [7] MEM 단계 (Memory Access)
    // =========================================================================
    //   DATA_MEM 인터페이스 :
    //      DREQ = MemRead | MemWrite
    //      DRW  = MemRead ? 2'b10 (Read)  : MemWrite ? 2'b00 (Write) : 00
    //      DADDR = ALU_out[31:2]  (word address)
    //      DWDATA = Rs2_data (store data)
    assign DREQ   = EX_MEM_Ctrl_MemRead_r | EX_MEM_Ctrl_MemWrite_r;
    assign DRW    = EX_MEM_Ctrl_MemRead_r  ? 2'b10 :
                    EX_MEM_Ctrl_MemWrite_r ? 2'b00 :
                                             2'b00 ;
    assign DADDR  = EX_MEM_ALU_out_r[31:2];
    assign DWDATA = EX_MEM_Rs2_data_r;

    // MEM 단계는 조합적으로 EX_MEM 레지스터의 출력을 그대로 통과시키는 역할
    //   → 별도 레지스터 없이 다음 MEM/WB 레지스터에서 latch


    // =========================================================================
    // [8] MEM/WB 파이프라인 레지스터
    // =========================================================================
    //   DRDATA 는 동기 read 라 다음 클럭에 valid → MEM/WB 레지스터가 같은 엣지에 latch
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
    // [9] WB 단계 (Write Back) - MUX
    // =========================================================================
    //   MemToReg = 0 → ALU_out  (R-type, ADDI, BRL/JL link 등)
    //   MemToReg = 1 → Mem_data (LD, LDR)
    assign WB_data     = MEM_WB_Ctrl_MemToReg_r ? MEM_WB_Mem_data_r : MEM_WB_ALU_out_r;
    assign WB_Rd       = MEM_WB_Rd_r;
    assign WB_RegWrite = MEM_WB_Ctrl_RegWrite_r;


endmodule

