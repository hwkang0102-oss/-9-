`timescale 1ns / 1ps

/*******************************************************************************
 * Full System Integration Testbench (Module Name: tb_ALL)
 * - Target Architecture: RISC_TOY CPU + DATA_RAM + CUSTOM_IP Accelerator
 * - 사양: 교수님 포트 맵 규격 완벽 준수 및 Active-High CSN 제어망 교정 완결본
 *******************************************************************************/

module tb_all; // ★ 조장 신호망 경로인 tb_ALL 규격으로 모듈 이름 일치화!

    reg             CLK, RSTN;

    /// CLOCK Generator ///
    parameter   PERIOD = 10.0;
    parameter   HPERIOD = PERIOD/2.0;

    initial CLK <= 1'b0;
    always #(HPERIOD) CLK <= ~CLK;

    // 풀 하드웨어 시스템 내부 연결 버스 선로 선언
    wire              IREQ;
    wire    [29:0]    IADDR;
    wire    [31:0]    INSTR;
    wire              DREQ;
    wire    [1:0]     DRW;
    wire    [29:0]    DADDR;
    wire    [31:0]    DWDATA;
    wire    [31:0]    DRDATA;
    wire    [31:0]    CONSIG;
    wire    [31:0]    IPIN;
    wire    [31:0]    IPOUT;

    // [1] 마스터 코어: RISC_TOY 5단계 파이프라인 Host CPU 인스턴스화
    RISC_TOY    RISC_TOY    (
        .CLK        (CLK),
        .RSTN       (RSTN),
        .IREQ       (IREQ),
        .IADDR      (IADDR),
        .INSTR      (INSTR),
        .DREQ       (DREQ),
        .DRW        (DRW),
        .DADDR      (DADDR),
        .DWDATA     (DWDATA),
        .DRDATA     (DRDATA),
        .CONSIG     (CONSIG) // 레지스터 R[31] 버스가 가속기 CON 제어핀으로 직결됨
    );

    // [2] 명령어 ROM 메모리 블록 연결
    // (교수님 model.v 칩 스펙이 Active-High이므로 CSN을 1'b1로 묶어 상시 활성화합니다)
    INST_RAM    #(32, 10, 1024, 1, "inst.hex") INST_MEM    (
        .CLK        (CLK),
        .CSN        (1'b1), // ★ Active-High 조건 완벽 복구
        .A          (IADDR[11:2]),
        .WEN        (1'b1),
        .DI         (32'd0),
        .DOUT       (INSTR)
    );

    // [3] 시스템 데이터 허브 교차로: DATA_RAM 인터페이스 바인딩
    // CPU 스토어 데이터(DWDATA)가 가속기 입력(IPIN)으로 흐르고, 가속기 정답(IPOUT)이 CPU 로드 버스(DRDATA)로 흐릅니다.
    DATA_RAM    #(32, 10, 1024, 0, "mem.hex") DATA_MEM    (
        .CLK        (CLK),
        .CSN        (1'b1), // ★ Active-High 조건 완벽 복구
        .A          (DADDR[11:2]),
        .WEN        (DRW),
        .DI1        (DWDATA), // CPU -> RAM (Store)
        .DI2        (IPOUT),  // 가속기 -> RAM (Feedback 완료 데이터)
        .DOUT1      (DRDATA), // RAM -> CPU (Load 버스 수신)
        .DOUT2      (IPIN)    // RAM -> 가속기 (3D 공간 좌표 데이터 스트리밍)
    );
    
    // [4] 슬레이브 코어: CUSTOM_IP 레이 트레이싱 가속 연산 칩 인스턴스화
    CUSTOM_IP   CUSTOM_IP   (
        .CLK        (CLK),
        .RSTN       (RSTN),
        .IPIN       (IPIN),   // RAM 스트리밍 버스로부터 가속 좌표 장착[cite: 1]
        .CON        (CONSIG), // CPU R[31] 버스에 의해 가속 알고리즘 활성화 및 제어[cite: 1]
        .IPOUT      (IPOUT)   // 연산 완료 후 최종 패킷을 RAM으로 복사[cite: 1]
    );

    // [5] 구동 제어 시퀀스
    initial begin
        RSTN <= 1'b0;
        #(10*PERIOD);
        RSTN <= 1'b1; // 시스템 전체 락 동시 해제 및 구동 시작

        // 파이프라인 연산 및 분기 처리가 완벽히 종결될 때까지 대기
        #(150*PERIOD);
        $finish();
    end

endmodule
