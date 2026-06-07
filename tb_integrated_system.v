`timescale 1ns / 1ps

// =========================================================================
// 통합 시스템 테스트벤치 (TOP 모듈)
// =========================================================================
module tb_integrated_system;

    // 1. 글로벌 클록 및 리셋 신호
    reg CLK;
    reg RSTN;

    // 2. RISC_TOY 인터페이스 와이어
    wire        IREQ;
    wire [29:0] IADDR;
    reg  [31:0] INSTR;
    wire        DREQ;
    wire [1:0]  DRW;
    wire [29:0] DADDR;
    wire [31:0] DWDATA;
    wire [31:0] CONSIG;
    wire [31:0] DRDATA;

    // 3. CUSTOM_IP 가속기 제어 와이어
    reg         tb_mode; 
    reg  [31:0] tb_CON;
    reg  [31:0] tb_IPIN;
    
    wire [31:0] final_CON  = tb_mode ? tb_CON  : CONSIG;
    wire [31:0] final_IPIN = tb_mode ? tb_IPIN : DWDATA;
    wire [31:0] IPOUT;

    // 4. 가상 메모리 시스템 (IMEM & DRAM)
    reg [31:0] IMEM [0:63];    
    reg [31:0] DRAM [0:1023];  
    reg [31:0] drdata_latched; 

    // Instruction Fetch 조합 회로
    always @(*) begin
        if (IADDR < 30'd64) begin
            INSTR = IMEM[IADDR];
        end else begin
            INSTR = 32'd0; 
        end
    end

    // Data RAM 동기식 읽기/쓰기 회로
    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            drdata_latched <= 32'd0;
        end else if (DREQ) begin
            if (DRW == 2'b10) begin 
                if (DADDR == 30'h3FFF_FFFF) begin
                    drdata_latched <= IPOUT;
                end else begin
                    drdata_latched <= DRAM[DADDR[9:0]];
                end
            end else if (DRW == 2'b00) begin 
                if (DADDR != 30'h3FFF_FFFF) begin
                    DRAM[DADDR[9:0]] <= DWDATA;
                end
            end
        end
    end
    
    assign DRDATA = drdata_latched;

    // 5. 모듈 인스턴스화 (CPU & Accelerator)
    RISC_TOY u_cpu (
        .CLK    (CLK),
        .RSTN   (RSTN),
        .IREQ   (IREQ),
        .IADDR  (IADDR),
        .INSTR  (INSTR),
        .DREQ   (DREQ),
        .DRW    (DRW),
        .DADDR  (DADDR),
        .DWDATA (DWDATA),
        .CONSIG (CONSIG),
        .DRDATA (DRDATA)
    );

    CUSTOM_IP u_accelerator (
        .CLK   (CLK),
        .RSTN  (RSTN),
        .CON   (final_CON),
        .IPIN  (final_IPIN),
        .IPOUT (IPOUT)
    );

    // 6. 클록 펄스 생성 (100MHz)
    always #5 CLK = ~CLK;

    // 7. 데이터 주입용 태스크 선언
    task send_sample(input [31:0] data, input reg toggle_bit);
        begin
            tb_IPIN = data;
            tb_CON[10] = toggle_bit; 
            @ (posedge CLK);
            #1;
        end
    endtask

    // 8. 시뮬레이션 메인 시나리오
    integer idx;
    initial begin
        // 초기화
        CLK     = 1'b0;
        RSTN    = 1'b0;
        tb_mode = 1'b1; 
        tb_CON  = 32'd0;
        tb_IPIN = 32'd0;

        for (idx = 0; idx < 64; idx = idx + 1) IMEM[idx] = 32'd0;
        
        IMEM[0] = {5'd3, 5'd1, 5'd0, 17'd10}; 
        IMEM[1] = {5'd0, 5'd2, 5'd1, 17'd5};
        IMEM[2] = {5'd3, 5'd31, 5'd0, 17'hA000}; 

        #20;
        RSTN = 1'b1; 
        $display("==================================================");
        $display("[SYSTEM] 리셋 해제! 시뮬레이션 시작");
        $display("==================================================");

        // [PHASE 1] 가속기 단독 테스트
        tb_CON = {16'h0000, 4'hA, 1'b1, 1'b0, 1'b0, 1'b0, 8'h55};
        @ (posedge CLK); #1;
        
        send_sample(32'h0002_0002, 1'b1);
        send_sample(32'h0002_0100, 1'b0);
        send_sample(32'h0100_0100, 1'b1);
        send_sample(32'h0001_0001, 1'b0);
        send_sample(32'h0001_0000, 1'b1);
        send_sample(32'h0000_0000, 1'b0);
        send_sample(32'h000A_000A, 1'b1);
        send_sample(32'h000A_0000, 1'b0); 

        @ (posedge CLK);
        @ (posedge CLK);
        #1;
        $display("[TC-RTBIA] 연산 결과 정답: 32'h%H", IPOUT);

        tb_CON[9] = ~tb_CON[9]; 
        @ (posedge CLK); #1;
        
        tb_CON[8] = 1'b1;
        @ (posedge CLK); #1;
        $display("[TC-RTBIA] 상태 레지스터 값: 32'h%H", IPOUT);

        // [PHASE 2] CPU 연동 테스트
        $display("\n[PHASE 2] CPU 파이프라인 연동 검증 시작");
        tb_mode = 1'b0; 
        
        repeat (15) begin
            @ (posedge CLK);
            #1;
            $display("PC: 32'h%H | INSTR: 32'h%H | DADDR: 32'h%H | DWDATA: 32'h%H", 
                     u_cpu.PC, INSTR, {DADDR, 2'b00}, DWDATA);
        end

        $display("==================================================");
        $display("[SYSTEM] 테스트 완료!");
        $display("==================================================");
        $stop; // $finish 대신 $stop을 쓰면 ModelSim이 꺼지지 않고 웨이브폼을 볼 수 있어!
    end

endmodule // <-- 테스트벤치 모듈 끝!


// =========================================================================
// 외부 REGFILE 모듈 (테스트벤치 모듈과 완전히 분리됨!)
// =========================================================================
module REGFILE #(parameter AW=5, ENTRY=32) (
    input wire CLK,
    input wire RSTN,
    input wire WEN,       
    input wire [AW-1:0] WA,
    input wire [31:0] DI,
    input wire [AW-1:0] RA0,
    input wire [AW-1:0] RA1,
    output wire [31:0] DOUT0,
    output wire [31:0] DOUT1,
    output wire [31:0] CONSIG   
);
    reg [31:0] registers [0:ENTRY-1];
    integer i;

    assign DOUT0  = registers[RA0];
    assign DOUT1  = registers[RA1];
    assign CONSIG = registers[5'd31]; 

    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            for (i = 0; i < ENTRY; i = i + 1) begin
                registers[i] <= 32'd0;
            end
        end else if (!WEN) begin
            registers[WA] <= DI;
        end
    end
endmodule // <-- REGFILE 모듈 끝!
