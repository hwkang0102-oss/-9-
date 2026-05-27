`timescale 1ns / 1ps

/*****************************************
    Testbench for RISC-TOY
    Team XX : 
        2024000000    Kim Mina
        2024000001    Lee Minho
        2023000002    Jake Jung
        [세은님 학번]   Jang Se-eun
*****************************************/

module tb_risc_toy();

    // --------------------------------------------------
    // 1. 신호 선언 (Signals)
    // --------------------------------------------------
    reg         CLK;
    reg         RSTN;
    
    // 명령어 메모리(IMEM) 인터페이스
    wire        IREQ;
    wire [29:0] IADDR;
    reg  [31:0] INSTR;
    
    // 데이터 메모리(DMEM) 인터페이스
    wire        DREQ;
    wire [1:0]  DRW;
    wire [29:0] DADDR;
    wire [31:0] DWDATA;
    reg  [31:0] DRDATA;
    
    // Custom IP 제어 신호
    wire [31:0] CONSIG;

    // --------------------------------------------------
    // 2. 가상 메모리 공간 생성 (Virtual Memory)
    // --------------------------------------------------
    reg [31:0] IMEM [0:1023]; // 4KB 명령어 메모리
    reg [31:0] DMEM [0:1023]; // 4KB 데이터 메모리

    // --------------------------------------------------
    // 3. RISC_TOY (우리가 만든 CPU) 인스턴스화
    // --------------------------------------------------
    RISC_TOY u_cpu (
        .CLK(CLK),
        .RSTN(RSTN),
        .IREQ(IREQ),
        .IADDR(IADDR),
        .INSTR(INSTR),
        .DREQ(DREQ),
        .DRW(DRW),
        .DADDR(DADDR),
        .DWDATA(DWDATA),
        .CONSIG(CONSIG),
        .DRDATA(DRDATA)
    );

    // --------------------------------------------------
    // 4. 클럭 및 리셋 생성, 시뮬레이션 제어
    // --------------------------------------------------
    integer i;
   
    initial begin

      
        // 클럭 초기화
        CLK = 1'b0;
        
        // 메모리를 0으로 싹 청소
        for (i = 0; i < 1024; i = i + 1) begin
            IMEM[i] = 32'd0;
            DMEM[i] = 32'd0;
        end
        
        // =========================================================
        // 🌟 명령어 하드코딩 구역 (inst.hex 대신 직접 입력)
        // =========================================================
        // 시나리오: R1 = 5, R2 = 10, R3 = R1 + R2
        
        // 1. MOVI r1, #5 (Opcode: 3, ra: 1, imm17: 5)
        IMEM[0] = 32'h18400005; 
        
        // 2. MOVI r2, #10 (Opcode: 3, ra: 2, imm17: 10)
        IMEM[1] = 32'h1880000A; 
        
        // 3. ADD r3, r1, r2 (Opcode: 4, ra: 3, rb: 1, rc: 2)
        IMEM[2] = 32'h20C22000; 
        
        // 4. NOP (파이프라인이 마저 비워지도록 더미 명령어 삽입)
        IMEM[3] = 32'h00000000; 
        IMEM[4] = 32'h00000000;
        IMEM[5] = 32'h00000000;
        IMEM[6] = 32'h00000000;
        
        // =========================================================
        
        // 시스템 리셋 (Active Low)
        RSTN = 1'b1;
        #10 RSTN = 1'b0; // 리셋 꾹 누름
        #10 RSTN = 1'b1; // 리셋 뗌 (이때부터 PC가 0부터 돌기 시작함)

        // 200 클럭(2000ns) 정도만 돌려보고 시뮬레이션 자동 종료
        #2000;
        $finish();
    end

    // 10ns 주기(100MHz) 클럭 생성
    always #5 CLK = ~CLK;

    // --------------------------------------------------
    // 5. 명령어 메모리 (IMEM) 동작 로직
    // --------------------------------------------------
    always @(posedge CLK) begin
        if (IREQ) begin
            // IADDR은 워드 주소(Word Address)이므로 배열 인덱스로 바로 사용
            INSTR <= IMEM[IADDR[9:0]]; 
        end
    end

    // --------------------------------------------------
    // 6. 데이터 메모리 (DMEM) 동작 로직
    // --------------------------------------------------
    always @(posedge CLK) begin
        if (DREQ) begin
            // 매뉴얼 기준: 2'b00은 Write, 2'b10은 Read [cite: 13, 14, 28, 29]
            if (DRW == 2'b00) begin
                DMEM[DADDR[9:0]] <= DWDATA; // Store
            end 
            else if (DRW == 2'b10) begin
                DRDATA <= DMEM[DADDR[9:0]]; // Load
            end
        end
    end

endmodule
