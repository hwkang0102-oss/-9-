/*****************************************
    
    Team XX : 
        2023104125    장세은
        2021103504    이경태
     	2021104197	  강현우

    [INTEGRATED VERSION]
    - IF / ID / EX / MEM / WB 5-stage 파이프라인을 RISC_TOY 모듈 하나로 통합
    - 외부 보조 모듈 없이 동작 (REGFILE 만 외부 model.v 사용)
    - Forwarding (EX/MEM, MEM/WB → EX), Load-Use Stall, Branch Flush 포함

    [FIX 1] EX/MEM 파이프라인 레지스터에 Branch_Taken flush 추가
            → Branch 후 잘못된 명령어의 제어신호가 MEM/WB → WB까지 전파되어
              레지스터에 쓰레기값이 기록되던 버그 수정
    [FIX 2] LD 명령어에서 rb == R[31]일 때 베이스 주소를 0으로 강제 (절대주소)
            → 매뉴얼 규정 준수: LD rb=R[31] → zero-extend (절대주소 모드)
    [FIX 3] LD/LDR 동기 read latency 흡수 (R4=x 버그 수정)
            → DATA_RAM 은 동기 read 메모리(요청 클럭의 다음 사이클에 DRDATA valid).
              기존 설계는 LD 결과를 1클럭 일찍 MEM/WB 에 latch 하여 load 값이 x 로
              깨지고, 이것이 의존 명령(ADD)으로 연쇄 전파되었음.
            → LD 가 MEM 단계에 도착한 첫 클럭에만 1클럭 Mem_Stall 을 삽입하여
              DRDATA 가 valid 된 다음 클럭에 MEM/WB 가 정상 latch 하게 함.
            → 연속 stall(load-use + mem-stall) 에도 명령어가 증발하지 않도록
              IF 단계 구출 버퍼(INSTR_reg)를 stall 동안 보존하도록 수정.

    [FIX 4] 분기 후 두 번째 flush 누락 수정 (R5=99 오염 버그)
            → 분기/점프는 EX 단계에서 결정되고 INST_MEM 이 동기 read 이므로,
              분기 직후 delay-slot 이 2개 생긴다(IF/ID 로 새어드는 쓰레기 명령 2개).
              기존 설계는 ID/EX 입구에서 1개만 죽여 둘째 쓰레기가 살아남아 실행됨.
            → Branch_Taken 클럭 + 그 다음 1클럭, 총 2클럭 동안 IF/ID 입구를
              NOP 으로 막아(IFID_Flush) delay-slot 2개를 모두 무력화.
              (목적지 명령은 그 2클럭 뒤에 IF/ID 로 들어오므로 보존된다.)

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
    wire        Stall_Flag;       // load-use (ID 단계)
    wire        Mem_Stall;        // [FIX 3] LD 동기 read latency 흡수용 1클럭 stall (MEM 단계)
    wire        Pipe_Stall;       // 두 stall 의 OR : 앞단(PC/IF/ID/ID/EX) 정지에 사용
    assign      Pipe_Stall = Stall_Flag | Mem_Stall;
    // Branch taken 시 IF/ID, ID/EX 를 Flush
    wire        Flush_Flag;
    assign      Flush_Flag = Branch_Taken;

    // =========================================================================
    // [Flush 로직] 점프(J/Branch) 처리
    //   파형 분석 결과(타이트한 파이프라인):
    //     - J 가 EX 에서 결정되어 Branch_Taken=1 이 되는 클럭에는,
    //       "점프로 죽여야 할 명령(점프 바로 다음 명령)"이 이미 IF/ID 에 들어와 있다.
    //     - 그 다음 클럭에 IF/ID 로 들어오는 것은 "점프 목적지"이므로 살려야 한다.
    //   따라서 죽일 명령은 정확히 1개뿐이고, flush 는 Branch_Taken 1클럭이면 충분하다.
    //   (이전의 Flush_Delay 다단 확장은 목적지까지 죽이므로 제거)
    //
    //   ★ 핵심: Branch_Taken 클럭에 이미 IF/ID 안에 있는 명령은
    //     "앞으로 들어올 입력을 막는 IF/ID flush"로는 못 죽인다.
    //     → 그 명령이 ID/EX 로 넘어가는 것을 ID/EX 입구에서 막아야 한다(아래 [4] 참조).
    // =========================================================================
    wire        Extended_Flush = Branch_Taken;   // 1클럭 flush (ID/EX 입구용, 기존 유지)

    // =========================================================================
    // [FIX 4] 분기 후 IF/ID 2클럭 flush
    //   분기/점프는 EX 단계에서 결정된다(Branch_Taken=1). INST_MEM 이 동기 read 라
    //   분기 결정 클럭 기준으로 그 "다음 2번의 fetch" 가 분기 직후 쓰레기 명령
    //   (delay-slot 2개)이며 IF/ID 로 새어 들어온다.
    //   기존 설계는 ID/EX 입구에서 1개만 죽여(첫째 쓰레기), 둘째·셋째가 살아남았다.
    //   → Branch_Taken 직후 2클럭 동안 IF/ID 입구를 NOP 으로 막아 둘 다 죽인다.
    //     (목적지 명령은 그 2클럭이 지난 뒤 IF/ID 로 들어오므로 보존된다.)
    reg [1:0]   br_flush_cnt;
    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN)
            br_flush_cnt <= 2'd0;
        else if (Branch_Taken)
            br_flush_cnt <= 2'd1;            // 분기 결정 → 이후 1클럭 더 IF/ID flush 예약
                                             //  (분기 당일은 Branch_Taken 으로 직접 막으므로
                                             //   delay-slot 2개 = Branch_Taken(1) + cnt(1))
        else if (Pipe_Stall)
            br_flush_cnt <= br_flush_cnt;    // stall 중에는 카운터 동결(같은 단계 반복)
        else if (br_flush_cnt != 2'd0)
            br_flush_cnt <= br_flush_cnt - 2'd1;
    end
    wire        IFID_Flush = Branch_Taken | (br_flush_cnt != 2'd0);   // IF/ID 입구를 NOP 으로 막는 구간

   


    // =========================================================================
    // [1] IF 단계 (Instruction Fetch)
    // =========================================================================
    reg [31:0] PC;

    // =========================================================================
    // 🌟 [추가됨] Stall 발생 시 증발하는 명령어를 구출하는 버퍼 로직
    // =========================================================================
    reg [31:0]  INSTR_reg;
    reg         Was_Stalled;

    // 스톨이 풀린 직후(Was_Stalled=1)에는 메모리에서 막 나오는 값이 아니라, 녹화해둔 구출 명령어를 사용!
    // (ModelSim 호환: 선언과 assign 을 분리하고, always 블록보다 먼저 둔다)
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
            // [FIX 3] stall 이 연속될 때 구출 명령어가 덮어써지지 않도록 보존.
            //   - 현재 stall 중이 아니면: 메모리 출력을 평소처럼 녹화
            //   - 현재 stall 중이면: 직전에 구출해 둔 safe_INSTR 를 그대로 유지
            //     (PC 가 멈춰 있어 같은 명령을 가리키므로 메모리 출력을 덮어쓰면 안 됨)
            INSTR_reg   <= Pipe_Stall ? safe_INSTR : INSTR;
            Was_Stalled <= Pipe_Stall;  // 방금 전 클럭이 (어떤) 스톨이었는지 기억함
        end
    end
    
    //=========================================================================


    // 다음 PC 선택 : branch 우선, 아니면 PC+4
    wire [31:0] next_PC = Branch_Taken ? Branch_Target : (PC + 32'd4);

    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            PC <= 32'd0;            // 매뉴얼 : 리셋 시 PC=0
        end else if (!Pipe_Stall) begin
            PC <= next_PC;
        end
        // Stall 시에는 PC 유지
    end

    assign IREQ  = 1'b1;
    assign IADDR = PC[29:0];        // word address (30-bit)


    // --- IF/ID 파이프라인 레지스터 ---
    reg [31:0] IF_ID_PC;
    reg [31:0] IF_ID_Inst;
    reg        IF_ID_Valid;   // 🌟 [추가] 이 명령이 정상 명령(1)인지, 점프로 죽어야 할 명령(0)인지

    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            IF_ID_PC    <= 32'd0;
            IF_ID_Inst  <= 32'd0;
            IF_ID_Valid <= 1'b0;
        end else if (Pipe_Stall) begin
            // Load-Use Stall / Mem-Stall : IF/ID 유지 (같은 명령어를 한 번 더 보게 함)
            IF_ID_PC    <= IF_ID_PC;
            IF_ID_Inst  <= IF_ID_Inst;
            IF_ID_Valid <= IF_ID_Valid;
        end else if (IFID_Flush) begin
            // [FIX 4] 분기 직후 delay-slot(쓰레기) 명령을 IF/ID 입구에서 NOP 으로 죽인다.
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
    reg        Ctrl_IPMem;            // 🌟 [LDIP/STIP] 1이면 데이터 포트가 CPU 가 아니라 CUSTOM_IP 임
                                       //   (LDIP : 메모리→IPIN / STIP : IPOUT→메모리)
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
        Ctrl_IPMem      = 1'b0;        // 🌟 기본 : 일반 CPU 메모리 접근
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

            // ---- Custom IP Memory (ray-tracing accelerator I/O) ----
            //   주소 모드는 LD/ST 와 동일하게 R[rb] + signExt(imm17) 사용.
            //   다른 점은 데이터 경로뿐이다(메모리 ↔ CUSTOM_IP).
            //   ※ opcode 번호(23/24)는 가정값. inst.hex/사양서와 다르면 여기 두 숫자만 변경.
            5'd23: begin  // LDIP : M[R[rb]+signExt(imm17)] → IPIN  (DRW=2'b11)
                          //   CPU 레지스터에 쓰지 않음(RegWrite=0). 메모리 읽기를 IP 포트로 보냄.
                Ctrl_MemRead = 1; Ctrl_IPMem = 1;
                Ctrl_ALUSrc  = 1; Ctrl_ALUOp = 4'b0000;
            end
            5'd24: begin  // STIP : IPOUT → M[R[rb]+signExt(imm17)]  (DRW=2'b01)
                          //   쓰는 데이터는 CUSTOM_IP 의 IPOUT(=DATA_RAM.DI2). CPU 는 주소만 제공.
                Ctrl_MemWrite = 1; Ctrl_IPMem = 1;
                Ctrl_ALUSrc   = 1; Ctrl_ALUOp = 4'b0000;
            end

            default: ; // NOP
        endcase
    end

    // =========================================================================
    // 🌟 [방법2 핵심] IF_ID_Valid 로 제어신호 게이팅
    //   IF/ID 에 실린 명령이 무효(점프로 죽어야 할 명령)이면 모든 제어신호를 0으로.
    //   → 이 명령이 ID/EX 로 넘어가도 RegWrite/MemWrite 등이 0 이라 아무 효과 없음.
    //   → 메모리 지연으로 명령이 몇 클럭 늦게 도착하든, Valid 비트만 보고 죽이므로
    //     flush 타이밍(2클럭/3클럭)을 손으로 맞출 필요가 없다.
    // =========================================================================
    wire        g_RegWrite   = Ctrl_RegWrite   & IF_ID_Valid;
    wire        g_MemRead     = Ctrl_MemRead    & IF_ID_Valid;
    wire        g_MemWrite    = Ctrl_MemWrite   & IF_ID_Valid;
    wire        g_MemToReg    = Ctrl_MemToReg   & IF_ID_Valid;
    wire        g_IPMem       = Ctrl_IPMem      & IF_ID_Valid;   // 🌟 무효 명령은 IP 메모리 접근도 금지
    wire        g_ALUSrc      = Ctrl_ALUSrc;                      // 데이터패스용, 무효해도 무해
    wire [3:0]  g_ALUOp       = Ctrl_ALUOp;
    wire [2:0]  g_BranchType  = IF_ID_Valid ? Ctrl_BranchType : 3'b000; // 무효 명령은 분기도 금지
    // =========================================================================
    //  - read 는 비동기, write 는 동기
    //  - 대부분의 R-type 은 rb / rc 를 소스로 사용
    //  - Store(ST/STR) : 저장할 값이 ra 에 있으므로 RA1 에서 ra 를 읽어야 함
    //  - Branch(BR/BRL) : 분기 주소가 rb (RA0), 조건 비교 대상이 rc (RA1)
    //  - LD 의 경우 베이스 레지스터 rb 가 R[31] 이면 절대주소 (zero-ext) 라는 매뉴얼 규정이 있으며,
    //    EX 단계에서 is_ld_abs 신호로 alu_op_a 를 0으로 강제하여 처리함 [FIX 2]
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
    assign Stall_Flag = IF_ID_Valid && ID_EX_MemRead_w &&
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
    reg        ID_EX_IPMem;     // 🌟 [LDIP/STIP] 데이터 포트가 CUSTOM_IP 인지

    // 위에서 미리 선언한 wire 들을 실제 reg 값으로 연결
    //  [LDIP 보정] LDIP 는 RegWrite=0(레지스터에 안 씀)이라 load-use 해저드 대상이 아니다.
    //  → 해저드 검출용 MemRead 는 "레지스터에 쓰는 진짜 load(LD/LDR)"만 1 이 되도록 IPMem 을 제외.
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
            // Branch_Taken(점프 결정 클럭) → ID 단계의 잘못된 명령 즉시 무력화 (NOP)
            // Stall / Mem_Stall          → ID/EX 에 NOP (= bubble) 주입
            // ※ 점프 이후 "늦게" 올라오는 명령들은 IF_ID_Valid=0 게이팅으로 처리되므로
            //   여기서 3클럭 Extended_Flush 를 쓰지 않는다(정상 명령 오살 방지).
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
            ID_EX_Rd_num    <= ra;                  // 목적지는 항상 ra
            ID_EX_ALUOp     <= g_ALUOp;
            ID_EX_BranchType<= g_BranchType;        // 🌟 무효 명령은 분기 안 함
            ID_EX_BR_Cond   <= br_cond;
            ID_EX_RegWrite  <= g_RegWrite;          // 🌟 무효 명령은 레지스터 쓰기 안 함
            ID_EX_MemRead   <= g_MemRead;           // 🌟
            ID_EX_MemWrite  <= g_MemWrite;          // 🌟
            ID_EX_MemToReg  <= g_MemToReg;          // 🌟
            ID_EX_ALUSrc    <= g_ALUSrc;
            ID_EX_IPMem     <= g_IPMem;             // 🌟 LDIP/STIP 표시
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
    //  LINK 명령 (BRL/JL)  : op_a = currentPC (= ID_EX_PC) → +4 결과를 RegFile 에 씀
    //  LD rb==R[31] (절대주소) : op_a = 0  → 0 + imm17 = 절대주소 (매뉴얼 규정)
    //  그 외               : op_a = rs1_fwd
    wire is_link   = (ID_EX_ALUOp == 4'b1101);
    // [FIX 2] LD(opcode=19)이고 소스 레지스터 번호가 R[31]이면 절대주소 모드
    wire is_ld_abs = ID_EX_MemRead && (ID_EX_Rs1_num == 5'd31);
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
    reg        EX_MEM_IPMem_r;        // 🌟 [LDIP/STIP]

    // [FIX 3] LD/LDR 동기 read latency 흡수
    //   DATA_RAM 은 동기 read(요청 클럭의 다음 사이클에 DRDATA valid) + ATIME 지연.
    //   LD 가 MEM 단계에 막 도착한 클럭에는 DRDATA 가 아직 옛 값(x) 이므로,
    //   그 클럭에 MEM/WB 가 latch 하면 load 결과가 깨진다.
    //   → LD 가 MEM 에 도착한 "첫" 클럭에만 1클럭 stall 을 걸어,
    //     다음 클럭(=DRDATA valid)에 MEM/WB 가 정상 latch 하게 한다.
    reg        Mem_Stall_done;   // 이번 LD 에 대해 이미 mem-stall 을 1회 했는지
    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) Mem_Stall_done <= 1'b0;
        else       Mem_Stall_done <= Mem_Stall; // 방금 mem-stall 했으면 다음 클럭엔 통과
    end
    //  [LDIP 보정] LDIP 는 MEM/WB 에 쓰지 않으므로(레지스터 미기록) read latency 흡수
    //  stall 이 필요 없다. → IPMem 인 read 는 mem-stall 대상에서 제외.
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
        // 🌟 [방법2] 무효 명령은 ID/EX 입구에서 이미 RegWrite/MemRead/MemWrite=0 으로
        //   게이팅되었으므로, 여기서 Extended_Flush 로 다시 강제할 필요가 없다.
        //   (오히려 3클럭으로 길어진 Extended_Flush 가 EX 단계의 "정상" 명령을
        //    죽일 수 있으므로, 기존 강제 flush 분기를 제거하고 그대로 통과시킨다.)
        end else if (Mem_Stall) begin
            // [FIX 3] LD 가 MEM 에 막 도착한 클럭 : 1클럭 더 머물게 한다.
            //   - ALU_out/Rd/RegWrite/MemToReg 는 유지 (다음 클럭 MEM/WB 가 latch)
            //   - MemRead 는 0 으로 내려 다음 클럭에 Mem_Stall 자동 해제
            //     (read 요청은 첫 클럭에 이미 DATA_RAM 으로 발행됨)
            EX_MEM_ALU_out_r       <= EX_MEM_ALU_out_r;
            EX_MEM_Rs2_data_r      <= EX_MEM_Rs2_data_r;
            EX_MEM_Rd_r            <= EX_MEM_Rd_r;
            EX_MEM_Ctrl_RegWrite_r <= EX_MEM_Ctrl_RegWrite_r;
            EX_MEM_Ctrl_MemRead_r  <= 1'b0;
            EX_MEM_Ctrl_MemWrite_r <= 1'b0;
            EX_MEM_Ctrl_MemToReg_r <= EX_MEM_Ctrl_MemToReg_r;
            EX_MEM_IPMem_r         <= 1'b0;     // 요청은 이미 발행됨
        end else begin
            EX_MEM_ALU_out_r       <= alu_result;
            EX_MEM_Rs2_data_r      <= rs2_fwd;            // store data : forwarding 반영
            EX_MEM_Rd_r            <= ID_EX_Rd_num;
            EX_MEM_Ctrl_RegWrite_r <= ID_EX_RegWrite;
            EX_MEM_Ctrl_MemRead_r  <= ID_EX_MemRead;
            EX_MEM_Ctrl_MemWrite_r <= ID_EX_MemWrite;
            EX_MEM_Ctrl_MemToReg_r <= ID_EX_MemToReg;
            EX_MEM_IPMem_r         <= ID_EX_IPMem;   // 🌟 LDIP/STIP 표시 전달
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
    //      DRW  : 2'b10 LD(메모리→CPU) / 2'b11 LDIP(메모리→IP)
    //             2'b00 ST(CPU→메모리) / 2'b01 STIP(IP IPOUT→메모리)
    //      DADDR = ALU_out[31:2]  (word address)
    //      DWDATA = Rs2_data (store data, ST/STR 전용 — STIP 은 DI2=IPOUT 사용)
    assign DREQ   = EX_MEM_Ctrl_MemRead_r | EX_MEM_Ctrl_MemWrite_r;
    assign DRW    = (EX_MEM_Ctrl_MemRead_r  & ~EX_MEM_IPMem_r) ? 2'b10 :   // LD
                    (EX_MEM_Ctrl_MemRead_r  &  EX_MEM_IPMem_r) ? 2'b11 :   // LDIP
                    (EX_MEM_Ctrl_MemWrite_r & ~EX_MEM_IPMem_r) ? 2'b00 :   // ST
                    (EX_MEM_Ctrl_MemWrite_r &  EX_MEM_IPMem_r) ? 2'b01 :   // STIP
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
        end else if (Mem_Stall) begin
            // [FIX 3] mem-stall 첫 클럭 : DRDATA 아직 무효 → MEM/WB 에 버블(WB 금지).
            //   다음 클럭(stall 해제)에 valid DRDATA + 유지된 LD 제어신호가 정상 latch 됨.
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
