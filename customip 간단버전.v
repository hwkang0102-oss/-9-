`timescale 1ns / 1ps

/*******************************************************************************
 * Core Architecture: 
 * TC-RTBIA (Temporal Coherence - Ray Tracing Bypassing & Intersection Accelerator)
 *
 * Structural Specification:
 * 1. System Control & Parameter Parsing
 * 2. Ray Data Parser (Sequential Demultiplexing)
 * 3. Temporal Coherence Cache (Error Comparator & Clock Gating)
 * 4. T-Intersection Pipeline (6-Parallel Multipliers & Min/Max Swapper)
 * 5. Reduction Tree & Output MUX (Tournament Tree & Data Forwarding)
 *******************************************************************************/

module CUSTOM_IP (
    input  wire        CLK,
    input  wire        RSTN,
    
    // RISC_TOY Interface
    input  wire [31:0] CON,     // CPU R[31]에서 인입: CON[0]=Start, CON[31:16]=Epsilon
    
    // Data Memory Interface
    input  wire [31:0] IPIN,    // 데이터 메모리에서 들어오는 32비트 패킷 버스
    output reg  [31:0] IPOUT    // 최종 출력 버스: [31]=Hit 플래그, [30:0]=최종 계산된 t 값
);

    // =========================================================================
    // [1] System Control: 제어 신호 및 파라미터 추출
    // =========================================================================
    wire        ip_start   = CON[0]; // 가속기 가동 스위치
    wire [15:0] epsilon_th = CON[31:16]; // 레이 트레이싱 오차 임계값인 입실론으로 사용

    // =========================================================================
    // [2] Ray Data Parser: 입출력 전처리 및 패킷 분해 (32-bit to 3D Space)
    // =========================================================================
    reg [2:0]  packet_cnt; // 0부터 7까지 세는 3비트 카운터 레지스터
    reg [15:0] ray_orig_x, ray_orig_y, ray_orig_z; // 공간 연산에 필요한 총 15개의 16비트 고정 소수점 변수들을 저장할 임시 저장 플립플롭 배열 선언 
    reg [15:0] ray_inv_x,  ray_inv_y,  ray_inv_z;
    reg [15:0] ray_dir_x,  ray_dir_y,  ray_dir_z;
    reg [15:0] box_min_x,  box_min_y,  box_min_z;
    reg [15:0] box_max_x,  box_max_y,  box_max_z;
    
    reg parsing_done; // 8주기의 데이터 수신이 완료되었음을 알리는 트리거

    always @(posedge CLK or negedge RSTN) begin // posedge CLK나 negedge RSTN에서만 로직이 실행되도록 하는 순차 논리 회로를 선언하는 구문
        if (!RSTN) begin // RSTN이 0이 되는 순간 실행 -> 모든 레지스터 상태를 0으로 만드는 초기화 작업을 시작
            packet_cnt   <= 3'd0;
            parsing_done <= 1'b0;
            
            // 물리 칩의 불정합(X-Propagation)을 완벽히 방어하기 위한 레지스터 초기화
            ray_orig_x   <= 16'd0;    ray_orig_y   <= 16'd0;    ray_orig_z   <= 16'd0;
            ray_inv_x    <= 16'd0;    ray_inv_y    <= 16'd0;    ray_inv_z    <= 16'd0;
            ray_dir_x    <= 16'd0;    ray_dir_y    <= 16'd0;    ray_dir_z    <= 16'd0;
            box_min_x    <= 16'h0000; box_min_y    <= 16'h0000; box_min_z    <= 16'h0000;
            box_max_x    <= 16'h0000; box_max_y    <= 16'h0000; box_max_z    <= 16'h0000;
        end else if (ip_start) begin // 리셋 상태가 아닌 CPU가 가속기 스위치를 킨 상태(ip_start=1)일때 작동
            packet_cnt   <= packet_cnt + 3'd1; // 클록이 한 번 뛸때마다 packet_cnt 레지스터 값을 1씩 증가시킴 
            parsing_done <= 1'b0; // 데이터 수집 단계이므로 수신 완료 신호인 parsing_done은 0으로 눌러놓음
            case (packet_cnt) // 32비트 버스로 유입되는 직렬 데이터를 16비트짜리 3D 공간 데이터 15개로 쪼개어 복원하는 역다중화기(Demultiplexer, DEMUX) 레지스터 제어문
                3'd0: begin ray_orig_x <= IPIN[31:16]; ray_orig_y <= IPIN[15:0];  end
                3'd1: begin ray_orig_z <= IPIN[31:16]; ray_inv_x  <= IPIN[15:0];  end
                3'd2: begin ray_inv_y  <= IPIN[31:16]; ray_inv_z  <= IPIN[15:0];  end
                3'd3: begin ray_dir_x  <= IPIN[31:16]; ray_dir_y  <= IPIN[15:0];  end
                3'd4: begin ray_dir_z  <= IPIN[31:16]; box_min_x  <= IPIN[15:0];  end
                3'd5: begin box_min_y  <= IPIN[31:16]; box_min_z  <= IPIN[15:0];  end
                3'd6: begin box_max_x  <= IPIN[31:16]; box_max_y  <= IPIN[15:0];  end
                3'd7: begin box_max_z  <= IPIN[31:16]; packet_cnt <= 3'd0; parsing_done <= 1'b1; end // 마지막 7번째 인입(3'd7)이 완료되는 순간 카운터를 다시 0으로 초기화하고, 뒷단 연산 엔진에게 준비를 위한 parsing_done <= 1'b1 신호를 준비함
            endcase
        end else begin // CPU가 가속기 가동 스위치를 내렸을 때(ip_start = 0) 처리하는 예외 안전장치
            packet_cnt   <= 3'd0;
            parsing_done <= 1'b0;
        end //가속기가 일을 안 할 때는 카운터와 파싱 플래그를 완전히 0으로 고정해서, 의도치 않은 클록 노이즈 때문에 데이터가 멋대로 변하는 오작동을 막음
    end

    // =========================================================================
    // [3] Temporal Coherence Cache: 방향 오차 캐싱 및 스마트 우회 제어
    // =========================================================================
    // 시간적 일관성(Temporal Coherence) 가속용 하드웨어 캐시 메모리' 공간을 선언
    reg [15:0] cache_dir_x, cache_dir_y, cache_dir_z; // 바로 직전에 계산했던 광선의 3차원 방향 벡터 값을 보관
    reg [31:0] cache_hit_data; // 그 광선이 상자에 부딪혔는지 안 부딪혔는지 계산해 놨던 최종 결과 정답을 담아두는 금고 역할
    
    wire [15:0] diff_x, diff_y, diff_z; //현재 들어온 광선 방향 벡터(Ray_{current})와 직전 광선 방향 벡터(Ray_{cached})의 하드웨어 절댓값 오차 수식 계산 회로망
    assign diff_x = (ray_dir_x > cache_dir_x) ? (ray_dir_x - cache_dir_x) : (cache_dir_x - ray_dir_x); // 삼항 연산자(? :)를 써서 "더 큰 값에서 작은 값을 빼라"는 크기 비교기(Comparator)와 감산기(Subtractor) 3세트
    assign diff_y = (ray_dir_y > cache_dir_y) ? (ray_dir_y - cache_dir_y) : (cache_dir_y - ray_dir_y);
    assign diff_z = (ray_dir_z > cache_dir_z) ? (ray_dir_z - cache_dir_z) : (cache_dir_z - ray_dir_z);

    // 3축 오차가 모두 설정된 임계값 미만일 때 일관성(Coherent) 상태로 판정 (오차 계산 결과가 허용 대역 이내인지 판별하여 Bypass(우회) 여부를 결정하는 지능형 판별기 게이트)
    wire is_coherent = (diff_x < epsilon_th) && (diff_y < epsilon_th) && (diff_z < epsilon_th) && (cache_dir_x != 16'd0); // X, Y, Z 세 축의 오차값(diff)이 모두 호스트 CPU가 설정해 준 임계값(epsilon_th)보다 작으면서, 캐시가 비어있지 않을 때(cache_dir_x != 0), is_coherent 와이어 선에 1을 보냄.
    // 데이터 로드가 끝났고, 일관성이 깨졌을 때만 무거운 메인 코어 엔진을 가동 (Clock Gating)
    wire enable_core_pipeline = parsing_done && !is_coherent; // 데이터 수신은 완벽히 끝났는데(parsing_done = 1), 오차가 임계값을 넘어서 일관성이 깨졌을 때만(is_coherent = 0 $\rightarrow$ !is_coherent = 1) 최종적으로 enable_core_pipeline = 1 신호를 킴
    // 만약 캐시가 적중해서 일관성이 확인되면 이 선이 0으로 죽어버림. 그러면 뒷단 연산 엔진들의 클록 전력을 완전히 차단해서 칩의 발열과 전력 소모를 제로로 얼려버림.

    // =========================================================================
    // [4] T-Intersection Pipeline: 6-Parallel 곱셈기 및 Min/Max Swapper
    // =========================================================================
    // 파이프라인 스테이지 1: 하드웨어 승산기를 이용한 t 연산 (조합 회로 무제한 병렬화)
    wire [31:0] raw_tx1 = (box_min_x - ray_orig_x) * ray_inv_x; // 6채널 완전 병렬 공간 교차 연산 엔진
    wire [31:0] raw_tx2 = (box_max_x - ray_orig_x) * ray_inv_x; // 레이 트레이싱의 근간 수식인 아래의 거리 연산 공식을 하드웨어화 (t = (Box - Ray_{orig}) \times Ray_{inv})
    wire [31:0] raw_ty1 = (box_min_y - ray_orig_y) * ray_inv_y; // X, Y, Z 세 개 축의 최소 경계면(box_min)과 최대 경계면(box_max)에 대해 6개의 하드웨어 곱셈기(Multiplier) 소자를 칩 안에 동시에 병렬로 배치
    wire [31:0] raw_ty2 = (box_max_y - ray_orig_y) * ray_inv_y; // 전기 신호가 인입되자마자 6개 수식의 정답 와이어(raw_t) 6줄이 단 1클록 만에 동시에 튀어나옴
    wire [31:0] raw_tz1 = (box_min_z - ray_orig_z) * ray_inv_z;
    wire [31:0] raw_tz2 = (box_max_z - ray_orig_z) * ray_inv_z;

    // 파이프라인 스테이지 2: Min/Max Swapper 레지스터 (고정 소수점 스케일 다운 >> 8 반영)
    reg [15:0] r_tmin_x, r_tmax_x; // 6채널 곱셈기에서 튀어 나온 생생한 원시 데이터들을 캡처해서 안전하게 한 박스 킵해둘 파이프라인 스테이지 2 물리 레지스터 공간 선언
    reg [15:0] r_tmin_y, r_tmax_y;
    reg [15:0] r_tmin_z, r_tmax_z;

    always @(posedge CLK or negedge RSTN) begin // 리셋 신호 유입 시 곱셈 데이터 6축 보관소 레지스터들을 깨끗하게 0으로 초기화
        if (!RSTN) begin
            r_tmin_x <= 16'd0; r_tmax_x <= 16'd0;
            r_tmin_y <= 16'd0; r_tmax_y <= 16'd0;
            r_tmin_z <= 16'd0; r_tmax_z <= 16'd0;
        end else if (enable_core_pipeline) begin //앞서 계산한 클록 게이팅 신호(enable_core_pipeline)가 1일 때만 이 레지스터들이 작동
            // 광선이 거꾸로 진입하는 음수 백터에 대비한 크기 자동 정렬 회로
            r_tmin_x <= (raw_tx1 > raw_tx2) ? raw_tx2[23:8] : raw_tx1[23:8]; //광선이 거꾸로 날아와서 최소/최대 거리 부호가 뒤집히는 극단적인 상황(Corner Case)에 대비하여, 내부 멀티플렉서 스위치를 통해 자동으로 더 작은 값을 tmin방에, 더 큰 값을 tmax방에 정렬시켜 저장
            r_tmax_x <= (raw_tx1 > raw_tx2) ? raw_tx1[23:8] : raw_tx2[23:8]; // [23:8]이라는 비트 슬라이싱 테크닉을 적용해, 32비트로 뻥튀기된 곱셈 결과 데이터 중 소수점 위치에 맞는 유효 데이터 대역 16비트만 칼 같이 뽑아내어 저장하는 고정 소수점 스케일 다운 가속 최적화를 동시에 수행
            r_tmin_y <= (raw_ty1 > raw_ty2) ? raw_ty2[23:8] : raw_ty1[23:8]; // [23:8]인 이유는 사용하는 고정 소수점(Fixed-point) 포맷이 소수부(Fractional bits)로 8비트를 사용한다고 가정했기 때문. 16bits x 16bits 곱셈을 하면 소수부가 16비트로 늘어나므로, 이를 다시 원래의 8비트 정밀도로 맞추기 위해 하위 8비트를 버리고 그 위부터 16비트([23:8])를 취하는 고정 소수점 스케일링 복원 연산
            r_tmax_y <= (raw_ty1 > raw_ty2) ? raw_ty1[23:8] : raw_ty2[23:8];
            r_tmin_z <= (raw_tz1 > raw_tz2) ? raw_tz2[23:8] : raw_tz1[23:8];
            r_tmax_z <= (raw_tz1 > raw_tz2) ? raw_tz1[23:8] : raw_tz2[23:8];
        end
    end

    // =========================================================================
    // [5] Reduction Tree & Final Output: 토너먼트 비교기 및 결과 포워딩
    // =========================================================================
    // 파이프라인 스테이지 2의 출력값을 가공하는 조합 논리 토너먼트 비교기 트리: 최솟값 중 최댓값(진입점), 최댓값 중 최솟값(이탈점) 추출
    wire [15:0] t_min_inter = (r_tmin_x > r_tmin_y) ? r_tmin_x : r_tmin_y; //3D 레이 트레이싱 수학 원리에 따라, 최종 진입점은 각 축의 최소 거리 결과 중 최대값(Max of Mins)을 구하고, 최종 이탈점은 각 축의 최대 거리 결과 중 최소값(Min of Maxes)을 구해야함.
    wire [15:0] t_min_final = (t_min_inter > r_tmin_z) ? t_min_inter : r_tmin_z; // 코드 그대로 x와 y를 먼저 대결시켜 승자를 뽑고(inter), 그 승자와 나머지 z축 데이터를 최종 대결시키는 2단계 하드웨어 토너먼트 트리로 설계

    wire [15:0] t_max_inter = (r_tmax_x < r_tmax_y) ? r_tmax_x : r_tmax_y;  
    wire [15:0] t_max_final = (t_max_inter < r_tmax_z) ? t_max_inter : r_tmax_z; 
     // t_{min\_final} = \max(t_{min,x}, t_{min,y}, t_{min,z}) , t_{max\_final} = \min(t_{max,x}, t_{max,y}, t_{max,z})

    // 최종 공간적 Hit 결론 판별 플래그 생성
    wire        pipeline_hit = (t_max_final >= t_min_final) && (t_max_final > 16'd0); // pipeline_hit: 상자 이탈 지점(t_{max\_final})이 진입 지점$t_{min\_final})보다 멀거나 같으면서 양수 대역에 존재할 때, 광선이 상자를 완벽히 관통했다는  플래그 신호 1을 생성
    wire [31:0] pipeline_computed_result = { pipeline_hit, {15{1'b0}}, t_min_final }; //pipeline_computed_result:  맨 왼쪽 31번 비트에는 방금 구한 Hit 플래그(1비트)를 싣고, 중간 15비트는 0으로 채운 뒤, 하위 16비트에는 최종 충돌 거리 정답인 t_min_final을 묶어 총 32비트 규격의 단일 데이터 패킷 패킹을 완성

    // 최종 출력 다중화기 제어 (Data Forwarding MUX)
    always @(posedge CLK or negedge RSTN) begin // 리셋 유입 시 가속기 출력 포트(IPOUT)와 내부 캐시 금고들을 깔끔하게 초기 조건인 0으로 초기화
        if (!RSTN) begin
            IPOUT           <= 32'd0;
            cache_dir_x     <= 16'd0; cache_dir_y <= 16'd0; cache_dir_z <= 16'd0;
            cache_hit_data  <= 32'd0;
        end else if (parsing_done) begin //스마트 바이패스(Smart Bypass) 데이터 포워딩 멀티플렉서 스위치
            if (is_coherent) begin
                // [Smart Bypass] 캐시가 적중하면 무거운 연산 결과를 씹고, 과거 정답을 1클록 즉시 출력 버스에 스킵 포워딩
                IPOUT <= cache_hit_data;
            end 
            //데이터 파싱이 끝났을 때(parsing_done = 1), 만약 캐시 오차 비교기가 "이전 광선이랑 똑같음"으로 is_coherent = 1 신호를 쏴주면, 뒤에서 곱셈 엔진 파이프라인이 연산해 온 결과는 스킵하고, 이건 보관 값인 cache_hit_data 을 단 1클록 만에 출력 버스 IPOUT으로 직접 전송(Data Forwarding)
            
            else begin // 오차가 커서 바이패스를 못 할 때는 파이프라인 엔진이 계산해 낸 pipeline_computed_result 결과를 IPOUT으로 보냄. 동시에 다음번에 날아올 광선이 현재와 비슷한 궤적인지 비교할 수 있도록, 현재 광선의 3축 방향 벡터(ray_dir)와 이번에 구한 정답 패킷을 캐시 금고 레지스터 세트에 갱신
                // [Normal Compute] 연산 결과 출력 및 다음 궤적 처리를 위해 아키텍처 캐시 데이터 백업 갱신
                IPOUT          <= pipeline_computed_result;
                cache_dir_x    <= ray_dir_x;
                cache_dir_y    <= ray_dir_y;
                cache_dir_z    <= ray_dir_z;
                cache_hit_data <= pipeline_computed_result;
            end
        end else if (!ip_start) begin //데이터 수집 중(parsing_done = 0)이거나 연산이 완전히 끝나서 CPU가 명령을 해제했을 때(ip_start = 0), 시스템 버스에 쓰레기 값이 흘러 다니지 않도록 IPOUT을 0으로 청소(Flush)하여 공유 버스를 안정화하는 하드웨어 매너 회로
            IPOUT <= 32'd0; // 가속 유닛 제어 해제 시 시스템 버스 플러시(Flush)
        end
    end

endmodule

이렇게 수정했어 어때
