`timescale 1ns / 1ps

module tb_CUSTOM_IP;

    // --- Inputs ---
    reg CLK;
    reg RSTN;
    reg [31:0] CON;
    reg [31:0] IPIN;

    // --- Outputs ---
    wire [31:0] IPOUT;

    // --- Unit Under Test (UUT) 인스턴스화 ---
    CUSTOM_IP uut (
        .CLK(CLK), 
        .RSTN(RSTN), 
        .CON(CON), 
        .IPIN(IPIN), 
        .IPOUT(IPOUT)
    );

    // --- 100MHz 시스템 클록 생성 ---
    always #5 CLK = ~CLK; 

    // --- 데이터 샘플 전송 태스크 (sample_toggle 자동화) ---
    reg sample_toggle_bit;
    task send_sample(input [31:0] data);
    begin
        IPIN = data;
        sample_toggle_bit = ~sample_toggle_bit; // 토글 상태 반전
        
        // CON[10] (sample_toggle) 비트 업데이트
        CON = {CON[31:11], sample_toggle_bit, CON[9:0]};
        
        // 1클록 대기 (데이터가 UUT 레지스터에 캡처될 시간 제공)
        #10; 
    end
    endtask

    // --- 시뮬레이션 시나리오 ---
    initial begin
        // 0. 포트 초기화
        CLK = 0;
        RSTN = 0;
        CON = 32'd0;
        IPIN = 32'd0;
        sample_toggle_bit = 0;

        // 글로벌 리셋 대기 후 활성화
        #100;
        RSTN = 1;
        #10;

        // ==========================================================
        // [Step 1] 가속기 초기화 (clear_config)
        // ==========================================================
        // Epsilon = 16'h0010 (상위 16비트)
        // Magic Key = 4'hA, clear_config(CON[11]) = 1
        $display("--- [Step 1] IP Initializing (clear_config) ---");
        CON = 32'h0010_A800; 
        #10;

        // clear_config 해제 (수신 대기 모드 진입)
        CON = 32'h0010_A000;
        #10;

        // ==========================================================
        // [Step 2] 3D 레이 트레이싱 데이터 패킷 8개 연속 전송
        // ==========================================================
        // 데이터 규격: 8.8 고정 소수점 ($1.0 = 256 = 16'\text{h}0100$)
        // - Ray Origin   = (0.0, 0.0, 0.0) -> 0x0000
        // - Ray Inv Dir  = (1.0, 1.0, 1.0) -> 0x0100
        // - Ray Dir      = (1.0, 1.0, 1.0) -> 0x0100
        // - Box Min      = (2.0, 2.0, 2.0) -> 0x0200
        // - Box Max      = (5.0, 5.0, 5.0) -> 0x0500
        
        $display("--- [Step 2] Sending 8 Packets via sample_toggle ---");
        send_sample(32'h0000_0000); // Pkt 0: orig_x, orig_y
        send_sample(32'h0000_0100); // Pkt 1: orig_z, inv_x
        send_sample(32'h0100_0100); // Pkt 2: inv_y, inv_z
        send_sample(32'h0100_0100); // Pkt 3: dir_x, dir_y
        send_sample(32'h0100_0200); // Pkt 4: dir_z, box_min_x
        send_sample(32'h0200_0200); // Pkt 5: box_min_y, box_min_z
        send_sample(32'h0500_0500); // Pkt 6: box_max_x, box_max_y
        send_sample(32'h0500_0000); // Pkt 7: box_max_z, padding

        // ==========================================================
        // [Step 3] 연산 종료 요청 (finish_toggle)
        // ==========================================================
        $display("--- [Step 3] Asserting finish_toggle & Waiting Pipeline ---");
        CON[9] = ~CON[9]; // CON[9] 토글로 연산 지시
        #50;              // 내부 3단계 파이프라인이 모두 비워질 때까지 대기

        // ==========================================================
        // [Step 4] 레이 트레이싱 Hit 결과 검증
        // ==========================================================
        // 예상 정답: 진입 거리(t_min) = 2.0 (0x0200), Hit 플래그 = 1 (최상위 비트)
        $display("--- [Step 4] Checking Intersection Result ---");
        $display("Expected: Hit=1, Distance=0200 | Actual IPOUT = %h", IPOUT);
        
        // ==========================================================
        // [Step 5] 필수 프로토콜 상태 워드 검증
        // ==========================================================
        $display("--- [Step 5] Checking Protocol Status Word ---");
        CON[8] = 1'b1; // output_select = 1 (상태 워드 MUX 스위칭)
        #10;
        
        // 예상 정답: 0x26(매직), 1(Done), 0(Resv), 08(8개 샘플), XXXX(암호화 해시값)
        $display("Expected: 2688XXXX | Actual IPOUT = %h", IPOUT);
        
        $display("--- Simulation Finished ---");
        $stop;
    end

endmodule
