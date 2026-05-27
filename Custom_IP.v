`timescale 1ns / 1ps

/*******************************************************************************
 * Core Architecture: 
 * TC-RTBIA (Temporal Coherence - Ray Tracing Bypassing & Intersection Accelerator)
 *
 * Structural Specification:
 * 1. System Control & Parameter Parsing (시스템 제어 및 파라미터 파싱 부)
 * 2. Ray Data Parser (Sequential Demultiplexing) (직렬 데이터 역다중화기 분해 부)
 * 3. Temporal Coherence Cache (Error Comparator & Clock Gating) (시간적 일관성 캐시 및 스마트 우회 부)
 * 4. T-Intersection Pipeline (6-Parallel Multipliers & Min/Max Swapper) (6채널 곱셈기 및 스왑 파이프라인 부)
 * 5. Reduction Tree & Output MUX (Tournament Tree & Data Forwarding) (토너먼트 트리 및 출력 제어 부)
 * *******************************************************************************/

module CUSTOM_IP (
    input  wire        CLK,     // 시스템 메인 클록 신호
    input  wire        RSTN,    // 시스템 액티브 로우(Active-Low) 리셋 신호
    
    // RISC_TOY Interface (호스트 CPU 통신용 제어 인터페이스)
    input  wire [31:0] CON,     // CPU R[31]에서 인입: CON[0]=Start, CON[31:16]=Epsilon
    
    // Data Memory Interface (데이터 메모리 직접 연결 인터페이스)
    input  wire [31:0] IPIN,    // 데이터 메모리에서 들어오는 32비트 패킷 버스
    output reg  [31:0] IPOUT,   // 최종 출력 버스: [31]=Hit 플래그, [30:0]=최종 계산된 t 값
    output reg         IP_VALID // 호스트 CPU에게 데이터 준비 완료를 알리는 핸드셰이크 펄스(Pulse) 플래그
);

    // =========================================================================
    // [1] System Control: 제어 신호 및 파라미터 추출
    // =========================================================================
    wire               ip_start   = CON[0]; // 가속기 가동 스위치
    // 부호 비교 연산 오류 방지를 위한 임계값 명시적 Signed 캐스팅
    wire signed [15:0] epsilon_th = $signed(CON[31:16]); 

    // =========================================================================
    // [2] Ray Data Parser: 입출력 전처리 및 패킷 분해 (32-bit to 3D Space)
    // =========================================================================
    reg [2:0]   packet_cnt; // 8주기 직렬 데이터 수신용 3비트 카운터 레지스터
    
    // 3D 공간 연산용 16비트 고정 소수점(Q8.8) 변수 임시 저장 플립플롭 배열 (음수 좌표 대응 signed)
    reg signed [15:0] ray_orig_x, ray_orig_y, ray_orig_z; 
    reg signed [15:0] ray_inv_x,  ray_inv_y,  ray_inv_z;
    reg signed [15:0] ray_dir_x,  ray_dir_y,  ray_dir_z;
    reg signed [15:0] box_min_x,  box_min_y,  box_min_z;
    reg signed [15:0] box_max_x,  box_max_y,  box_max_z;
    
    reg parsing_done; // 8주기의 데이터 수신 및 레지스터 래치(Latch) 완료 트리거

    always @(posedge CLK or negedge RSTN) begin 
        if (!RSTN) begin 
            packet_cnt   <= 3'd0;
            parsing_done <= 1'b0;
            
            // X-Propagation을 완벽히 방어하기 위한 전체 레지스터 초기화
            ray_orig_x   <= 16'd0; ray_orig_y   <= 16'd0; ray_orig_z   <= 16'd0;
            ray_inv_x    <= 16'd0; ray_inv_y    <= 16'd0; ray_inv_z    <= 16'd0;
            ray_dir_x    <= 16'd0; ray_dir_y    <= 16'd0; ray_dir_z    <= 16'd0;
            box_min_x    <= 16'd0; box_min_y    <= 16'd0; box_min_z    <= 16'd0;
            box_max_x    <= 16'd0; box_max_y    <= 16'd0; box_max_z    <= 16'd0;
        end else if (ip_start) begin 
            // 단일 사이클 펄스로 parsing_done을 생성하여 후위 파이프라인의 정확한 타이밍 매칭 보장
            if (packet_cnt == 3'd7) begin
                packet_cnt   <= 3'd0;
                parsing_done <= 1'b1; 
            end else begin
                packet_cnt   <= packet_cnt + 3'd1; 
                parsing_done <= 1'b0; 
            end

            // 32비트 버스로 유입되는 직렬 데이터를 16비트 단위 3D 공간 데이터로 복원하는 역다중화기(DEMUX)
            case (packet_cnt) 
                3'd0: begin ray_orig_x <= $signed(IPIN[31:16]); ray_orig_y <= $signed(IPIN[15:0]); end
                3'd1: begin ray_orig_z <= $signed(IPIN[31:16]); ray_inv_x  <= $signed(IPIN[15:0]); end
                3'd2: begin ray_inv_y  <= $signed(IPIN[31:16]); ray_inv_z  <= $signed(IPIN[15:0]); end
                3'd3: begin ray_dir_x  <= $signed(IPIN[31:16]); ray_dir_y  <= $signed(IPIN[15:0]); end
                3'd4: begin ray_dir_z  <= $signed(IPIN[31:16]); box_min_x  <= $signed(IPIN[15:0]); end
                3'd5: begin box_min_y  <= $signed(IPIN[31:16]); box_min_z  <= $signed(IPIN[15:0]); end
                3'd6: begin box_max_x  <= $signed(IPIN[31:16]); box_max_y  <= $signed(IPIN[15:0]); end
                3'd7: begin box_max_z  <= $signed(IPIN[31:16]); end 
                default: begin end // Full-case 유도를 통한 합성 시 불필요한 기생 래치(Latch) 생성 방지
            endcase
        end else begin 
            // 가속기 유휴(Idle) 상태 시 카운터 및 파싱 플래그 초기화하여 오작동 원천 차단
            packet_cnt   <= 3'd0;
            parsing_done <= 1'b0;
        end 
    end

    // =========================================================================
    // [3] Temporal Coherence Cache: 방향 오차 캐싱 및 스마트 우회 제어
    // =========================================================================
    reg signed [15:0] cache_dir_x, cache_dir_y, cache_dir_z; // 직전 광선의 3차원 방향 벡터 보관 레지스터
    reg [31:0]        cache_hit_data; // 시간적 일관성 확보 시 즉시 출력될 이전 연산 결론(Hit 플래그 및 t값)
    reg               cache_valid;    // 캐시 내 유효 데이터 존재 여부 플래그 (무한 미스 방지용)
    
    // 현재 광선 방향 벡터(Ray_current)와 직전 광선 방향 벡터(Ray_cached) 간의 하드웨어 절댓값 오차 도출 회로
    wire signed [15:0] diff_x, diff_y, diff_z; 
    
    // 삼항 연산자를 이용한 3축 병렬 크기 비교기(Comparator) 및 감산기(Subtractor) 
    assign diff_x = (ray_dir_x > cache_dir_x) ? (ray_dir_x - cache_dir_x) : (cache_dir_x - ray_dir_x); 
    assign diff_y = (ray_dir_y > cache_dir_y) ? (ray_dir_y - cache_dir_y) : (cache_dir_y - ray_dir_y);
    assign diff_z = (ray_dir_z > cache_dir_z) ? (ray_dir_z - cache_dir_z) : (cache_dir_z - ray_dir_z);

    // 3축 오차가 모두 설정된 임계값 미만이며 캐시가 유효할 때, 연산 우회(Bypass) 여부를 결정하는 지능형 판별기
    wire is_coherent = (diff_x < epsilon_th) && (diff_y < epsilon_th) && (diff_z < epsilon_th) && cache_valid; 
    
    // Clock Gating 트리거: 데이터 파싱이 끝났으나 일관성이 깨졌을 때만 무거운 메인 파이프라인 엔진 활성화
    wire enable_core_pipeline = parsing_done && !is_coherent; 

    // =========================================================================
    // [4] T-Intersection Pipeline: 6-Parallel 곱셈기 및 Min/Max Swapper
    // =========================================================================
    // 코어 파이프라인의 데이터 흐름을 정확히 추적하여 연산 안착 지점을 매칭하는 3-Stage Shift Valid 레지스터
    reg [2:0] pipe_valid;

    // 파이프라인 스테이지 1: 6채널 하드웨어 승산기를 이용한 t(거리) 연산 (단일 사이클 무제한 병렬화)
    reg signed [31:0] stg1_tx1, stg1_tx2;
    reg signed [31:0] stg1_ty1, stg1_ty2;
    reg signed [31:0] stg1_tz1, stg1_tz2;

    always @(posedge CLK or negedge RSTN) begin 
        if (!RSTN) begin
            stg1_tx1   <= 32'd0; stg1_tx2   <= 32'd0;
            stg1_ty1   <= 32'd0; stg1_ty2   <= 32'd0;
            stg1_tz1   <= 32'd0; stg1_tz2   <= 32'd0;
            pipe_valid <= 3'b0;
        end else begin
            // 데이터 흐름과 1:1 동기화되는 파이프라인 단계별 유효성 시프트 시그널
            pipe_valid[0] <= enable_core_pipeline;
            pipe_valid[1] <= pipe_valid[0];
            pipe_valid[2] <= pipe_valid[1]; 
            
            // 파이프라인 가동 신호 발생 시 단 1회 연산하여 동적 전력 소모 최소화 및 덮어쓰기 방지
            if (enable_core_pipeline) begin 
                stg1_tx1 <= (box_min_x - ray_orig_x) * ray_inv_x;
                stg1_tx2 <= (box_max_x - ray_orig_x) * ray_inv_x;
                stg1_ty1 <= (box_min_y - ray_orig_y) * ray_inv_y;
                stg1_ty2 <= (box_max_y - ray_orig_y) * ray_inv_y;
                stg1_tz1 <= (box_min_z - ray_orig_z) * ray_inv_z;
                stg1_tz2 <= (box_max_z - ray_orig_z) * ray_inv_z;
            end else begin
                // 게이팅 시 데이터 버스를 강제로 비워 기생 노이즈 전파 차단
                stg1_tx1 <= 32'd0; stg1_tx2 <= 32'd0;
                stg1_ty1 <= 32'd0; stg1_ty2 <= 32'd0;
                stg1_tz1 <= 32'd0; stg1_tz2 <= 32'd0;
            end
        end
    end

    // 파이프라인 스테이지 2: Min/Max Swapper 레지스터 (고정 소수점 Q16.16 -> Q8.8 스케일 다운 복원)
    reg signed [15:0] r_tmin_x, r_tmax_x;
    reg signed [15:0] r_tmin_y, r_tmax_y;
    reg signed [15:0] r_tmin_z, r_tmax_z;

    // 부호(Signed) 확장을 유지하며 소수부 스케일 다운을 수행하는 산술 우측 시프트(>>>) 디코딩 와이어
    wire signed [31:0] shifted_tx1 = stg1_tx1 >>> 8;
    wire signed [31:0] shifted_tx2 = stg1_tx2 >>> 8;
    wire signed [31:0] shifted_ty1 = stg1_ty1 >>> 8;
    wire signed [31:0] shifted_ty2 = stg1_ty2 >>> 8;
    wire signed [31:0] shifted_tz1 = stg1_tz1 >>> 8;
    wire signed [31:0] shifted_tz2 = stg1_tz2 >>> 8;

    // 스케일 다운된 32비트 연산 결과물에서 하위 유효 부호 대역 16비트를 정밀 추출
    wire signed [15:0] scaled_tx1 = shifted_tx1[15:0];
    wire signed [15:0] scaled_tx2 = shifted_tx2[15:0];
    wire signed [15:0] scaled_ty1 = shifted_ty1[15:0];
    wire signed [15:0] scaled_ty2 = shifted_ty2[15:0];
    wire signed [15:0] scaled_tz1 = shifted_tz1[15:0];
    wire signed [15:0] scaled_tz2 = shifted_tz2[15:0];

    always @(posedge CLK or negedge RSTN) begin 
        if (!RSTN) begin
            r_tmin_x <= 16'd0; r_tmax_x <= 16'd0;
            r_tmin_y <= 16'd0; r_tmax_y <= 16'd0;
            r_tmin_z <= 16'd0; r_tmax_z <= 16'd0;
        end else begin
            if (pipe_valid[0]) begin 
                // 광선 역진입 코너 케이스(Corner Case)에 대비한 하드웨어 크기 자동 정렬 멀티플렉서
                r_tmin_x <= (scaled_tx1 > scaled_tx2) ? scaled_tx2 : scaled_tx1; 
                r_tmax_x <= (scaled_tx1 > scaled_tx2) ? scaled_tx1 : scaled_tx2; 
                r_tmin_y <= (scaled_ty1 > scaled_ty2) ? scaled_ty2 : scaled_ty1; 
                r_tmax_y <= (scaled_ty1 > scaled_ty2) ? scaled_ty1 : scaled_ty2;
                r_tmin_z <= (scaled_tz1 > scaled_tz2) ? scaled_tz2 : scaled_tz1;
                r_tmax_z <= (scaled_tz1 > scaled_tz2) ? scaled_tz1 : scaled_tz2;
            end else begin
                r_tmin_x <= 16'd0; r_tmax_x <= 16'd0;
                r_tmin_y <= 16'd0; r_tmax_y <= 16'd0;
                r_tmin_z <= 16'd0; r_tmax_z <= 16'd0;
            end
        end
    end

    // =========================================================================
    // [5] Reduction Tree & Final Output: 토너먼트 비교기 및 결과 포워딩
    // =========================================================================
    // 스테이지 2 출력값을 기반으로 조합 논리를 통해 '최솟값 중 최댓값(진입점)' 및 '최댓값 중 최솟값(이탈점)' 추출
    
    // X축과 Y축의 최소 진입점 중 더 먼 지점을 1차 추출 (1st Stage Max)
    wire signed [15:0] t_min_inter = (r_tmin_x > r_tmin_y) ? r_tmin_x : r_tmin_y; 
    // 1차 추출 값과 Z축 진입점을 최종 비교하여 '최종 진입점' 추출 (Final Max of Mins)
    wire signed [15:0] t_min_final = (t_min_inter > r_tmin_z) ? t_min_inter : r_tmin_z; 

    // X축과 Y축의 최대 이탈점 중 더 짧은 지점을 1차 추출 (1st Stage Min)
    wire signed [15:0] t_max_inter = (r_tmax_x < r_tmax_y) ? r_tmax_x : r_tmax_y;  
    // 1차 추출 값과 Z축 이탈점을 최종 비교하여 '최종 이탈점' 추출 (Final Min of Maxes)
    wire signed [15:0] t_max_final = (t_max_inter < r_tmax_z) ? t_max_inter : r_tmax_z; 

    // 상자 이탈 지점이 진입 지점보다 멀거나 같으며 양수일 때, 1클럭 내에 공간적 Hit 결론 생성
    wire        pipeline_hit             = (t_max_final >= t_min_final) && (t_max_final >= 16'sd0); 
    
    // [31]: Hit 플래그, [30:16]: 패딩 비트, [15:0]: 충돌 거리 정답(t_min_final)으로 32비트 패킷 완성
    wire [31:0] pipeline_computed_result = { pipeline_hit, 15'd0, t_min_final }; 

    // =========================================================================
    // Output Multiplexer: 스마트 바이패스 및 정규 연산 결과 출력 제어
    // =========================================================================
    always @(posedge CLK or negedge RSTN) begin 
        if (!RSTN) begin
            IPOUT          <= 32'd0;
            IP_VALID       <= 1'b0;
            cache_dir_x    <= 16'd0; cache_dir_y <= 16'd0; cache_dir_z <= 16'd0;
            cache_hit_data <= 32'd0;
            cache_valid    <= 1'b0;  
        end else begin
            // Handshake 프로토콜 준수를 위해 기본적으로 유효 신호를 클리어(Clear)
            IP_VALID <= 1'b0; 

            if (parsing_done && is_coherent) begin
                // [Smart Bypass] 캐시 적중 시 연산을 생략하고 보관된 과거 정답을 1클럭 즉시 스킵 포워딩
                IPOUT    <= cache_hit_data;
                IP_VALID <= 1'b1;
            end 
            else if (pipe_valid[2]) begin 
                // [Normal Compute] 토너먼트 트리의 조합 논리가 완전히 안정화된 시점에 출력 레지스터 갱신
                IPOUT          <= pipeline_computed_result;
                IP_VALID       <= 1'b1;
                
                // 후속 레이 트레이싱 오차 판별을 위해 현재 궤적 데이터를 아키텍처 캐시에 백업
                cache_dir_x    <= ray_dir_x;
                cache_dir_y    <= ray_dir_y;
                cache_dir_z    <= ray_dir_z;
                cache_hit_data <= pipeline_computed_result;
                cache_valid    <= 1'b1; 
            end 
            else if (!ip_start) begin
                // 가속 유닛 제어 해제 시 시스템 버스 플러시(Flush)를 통해 잔여 쓰레기 값 오염 차단
                IPOUT          <= 32'd0;
                cache_dir_x    <= 16'd0;
                cache_dir_y    <= 16'd0;
                cache_dir_z    <= 16'd0;
                cache_hit_data <= 32'd0;
                cache_valid    <= 1'b0; 
            end 
        end
    end

endmodule
