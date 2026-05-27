`timescale 1ns / 1ps

/*******************************************************************************
 * Core Architecture: 
 * TC-RTBIA (Temporal Coherence - Ray Tracing Bypassing & Intersection Accelerator)
 *******************************************************************************/

module CUSTOM_IP (
    input  wire        CLK,      // 시스템 메인 클록 신호
    input  wire        RSTN,     // 시스템 액티브 로우(Active-Low) 리셋 신호
    
    // RISC_TOY Interface (호스트 CPU 통신용 제어 인터페이스)
    input  wire [31:0] CON,      // CPU R[31]에서 인입: CON[0]=Start, CON[31:16]=Epsilon
    
    // Data Memory Interface (데이터 메모리 직접 연결 인터페이스)
    input  wire [31:0] IPIN,     // 데이터 메모리에서 들어오는 32비트 패킷 버스
    output reg  [31:0] IPOUT,    // 최종 출력 버스: [31]=Hit 플래그, [30:0]=최종 계산된 t 값
    output reg         IP_VALID  // 호스트 CPU에게 데이터 준비 완료를 알리는 핸드셰이크 플래그
);

    // =========================================================================
    // [1] System Control: 제어 신호 및 파라미터 추출
    // =========================================================================
    // [보완] CPU 컨트롤 레지스터 매핑 및 임계값 부호 확장 캐스팅
    wire               ip_start   = CON[0]; 
    wire signed [15:0] epsilon_th = $signed(CON[31:16]); 

    // =========================================================================
    // [2] Ray Data Parser: 입출력 전처리 및 패킷 분해
    // =========================================================================
    reg [2:0]  packet_cnt; 
    
    // [보완] 광선(Ray) 및 바운딩 박스(AABB) 기하학적 데이터 레지스터 (Q8.8 포맷)
    reg signed [15:0] ray_orig_x, ray_orig_y, ray_orig_z; 
    reg signed [15:0] ray_inv_x,  ray_inv_y,  ray_inv_z;
    reg signed [15:0] ray_dir_x,  ray_dir_y,  ray_dir_z;
    reg signed [15:0] box_min_x,  box_min_y,  box_min_z;
    reg signed [15:0] box_max_x,  box_max_y,  box_max_z;
    
    reg parsing_done; 

    always @(posedge CLK or negedge RSTN) begin 
        if (!RSTN) begin 
            // [보완] 시스템 리셋 시 파서 및 모든 기하학적 레지스터 초기화
            packet_cnt   <= 3'd0;
            parsing_done <= 1'b0;
            
            ray_orig_x   <= 16'd0; ray_orig_y   <= 16'd0; ray_orig_z   <= 16'd0;
            ray_inv_x    <= 16'd0; ray_inv_y    <= 16'd0; ray_inv_z    <= 16'd0;
            ray_dir_x    <= 16'd0; ray_dir_y    <= 16'd0; ray_dir_z    <= 16'd0;
            box_min_x    <= 16'd0; box_min_y    <= 16'd0; box_min_z    <= 16'd0;
            box_max_x    <= 16'd0; box_max_y    <= 16'd0; box_max_z    <= 16'd0;
        end else if (ip_start) begin 
            // [보완] 8클록 주기로 16개의 패킷 워드를 순차적으로 분해하여 레지스터에 래치
            if (packet_cnt == 3'd7) begin
                packet_cnt   <= 3'd0;
                parsing_done <= 1'b1; 
            end else begin
                packet_cnt   <= packet_cnt + 3'd1; 
                parsing_done <= 1'b0; 
            end

            case (packet_cnt) 
                3'd0: begin ray_orig_x <= $signed(IPIN[31:16]); ray_orig_y <= $signed(IPIN[15:0]); end
                3'd1: begin ray_orig_z <= $signed(IPIN[31:16]); ray_inv_x  <= $signed(IPIN[15:0]); end
                3'd2: begin ray_inv_y  <= $signed(IPIN[31:16]); ray_inv_z  <= $signed(IPIN[15:0]); end
                3'd3: begin ray_dir_x  <= $signed(IPIN[31:16]); ray_dir_y  <= $signed(IPIN[15:0]); end
                3'd4: begin ray_dir_z  <= $signed(IPIN[31:16]); box_min_x  <= $signed(IPIN[15:0]); end
                3'd5: begin box_min_y  <= $signed(IPIN[31:16]); box_min_z  <= $signed(IPIN[15:0]); end
                3'd6: begin box_max_x  <= $signed(IPIN[31:16]); box_max_y  <= $signed(IPIN[15:0]); end
                3'd7: begin box_max_z  <= $signed(IPIN[31:16]); end 
                default: begin end 
            endcase
        end else begin 
            // [보완] ip_start가 비활성화되면 파서를 안전하게 초기 상태로 되돌림
            packet_cnt   <= 3'd0;
            parsing_done <= 1'b0;
        end 
    end

    // =========================================================================
    // [3] Temporal Coherence Cache
    // =========================================================================
    reg signed [15:0] cache_dir_x, cache_dir_y, cache_dir_z; 
    reg [31:0]        cache_hit_data; 
    reg               cache_valid;  
    
    // [오류 수정 A] 오버플로우 방지를 위한 17비트 부호 확장 및 안전한 절댓값 계산
    // [오류 수정 G] 합성 툴의 Implicit Unsigned Cast 경고를 원천 차단하기 위해 $signed 명시적 래핑 추가
    wire signed [16:0] ext_ray_dir_x = $signed({ray_dir_x[15], ray_dir_x});
    wire signed [16:0] ext_cache_dir_x = $signed({cache_dir_x[15], cache_dir_x});
    wire signed [16:0] diff_x_ext = ext_ray_dir_x - ext_cache_dir_x;
    wire signed [16:0] diff_x_abs = (diff_x_ext < 0) ? -diff_x_ext : diff_x_ext;

    wire signed [16:0] ext_ray_dir_y = $signed({ray_dir_y[15], ray_dir_y});
    wire signed [16:0] ext_cache_dir_y = $signed({cache_dir_y[15], cache_dir_y});
    wire signed [16:0] diff_y_ext = ext_ray_dir_y - ext_cache_dir_y;
    wire signed [16:0] diff_y_abs = (diff_y_ext < 0) ? -diff_y_ext : diff_y_ext;

    wire signed [16:0] ext_ray_dir_z = $signed({ray_dir_z[15], ray_dir_z});
    wire signed [16:0] ext_cache_dir_z = $signed({cache_dir_z[15], cache_dir_z});
    wire signed [16:0] diff_z_ext = ext_ray_dir_z - ext_cache_dir_z;
    wire signed [16:0] diff_z_abs = (diff_z_ext < 0) ? -diff_z_ext : diff_z_ext;

    // [오류 수정 G 적용] $signed 래핑으로 엄격한 부호 있는 정수 확장 보장
    wire signed [16:0] ext_epsilon = $signed({epsilon_th[15], epsilon_th});
    
    // [오류 수정 H] 임계값(Epsilon) 비교 논리를 < 에서 <= 로 수정하여 Epsilon이 0(완전 일치)일 때도 캐시가 정상 동작하도록 수정
    wire is_coherent = (diff_x_abs <= ext_epsilon) && (diff_y_abs <= ext_epsilon) && (diff_z_abs <= ext_epsilon) && cache_valid; 
    
    wire enable_core_pipeline = parsing_done && !is_coherent; 

    // =========================================================================
    // [4] T-Intersection Pipeline (Stage 1 & 2)
    // =========================================================================
    // [오류 수정 B] 실제 데이터 패스 깊이(2 Stages)에 맞춰 파이프라인 유효 신호 크기 축소 (3비트 -> 2비트)
    reg [1:0] pipe_valid;

    // [Stage 1] 하드웨어 곱셈기 (결과 포맷: Q16.16)
    reg signed [31:0] stg1_tx1, stg1_tx2;
    reg signed [31:0] stg1_ty1, stg1_ty2;
    reg signed [31:0] stg1_tz1, stg1_tz2;

    always @(posedge CLK or negedge RSTN) begin 
        if (!RSTN) begin
            stg1_tx1   <= 32'd0; stg1_tx2   <= 32'd0;
            stg1_ty1   <= 32'd0; stg1_ty2   <= 32'd0;
            stg1_tz1   <= 32'd0; stg1_tz2   <= 32'd0;
            pipe_valid <= 2'b0;
        end else begin
            pipe_valid[0] <= enable_core_pipeline;
            pipe_valid[1] <= pipe_valid[0]; 
            
            // [오류 수정 1] 단일 펄스 이후 강제 Flush 구문 제거 -> 값 안전하게 유지(Latch)
            if (enable_core_pipeline) begin 
                // [오류 수정 F] 방어적 코딩(Defensive Coding): 합성 툴의 암시적 부호 확장(Implicit Sign Extension) 오류를 원천 차단하기 위해 뺄셈 전 32비트로 명시적 부호 확장 적용
                stg1_tx1 <= ($signed({{16{box_min_x[15]}}, box_min_x}) - $signed({{16{ray_orig_x[15]}}, ray_orig_x})) * $signed({{16{ray_inv_x[15]}}, ray_inv_x});
                stg1_tx2 <= ($signed({{16{box_max_x[15]}}, box_max_x}) - $signed({{16{ray_orig_x[15]}}, ray_orig_x})) * $signed({{16{ray_inv_x[15]}}, ray_inv_x});
                stg1_ty1 <= ($signed({{16{box_min_y[15]}}, box_min_y}) - $signed({{16{ray_orig_y[15]}}, ray_orig_y})) * $signed({{16{ray_inv_y[15]}}, ray_inv_y});
                stg1_ty2 <= ($signed({{16{box_max_y[15]}}, box_max_y}) - $signed({{16{ray_orig_y[15]}}, ray_orig_y})) * $signed({{16{ray_inv_y[15]}}, ray_inv_y});
                stg1_tz1 <= ($signed({{16{box_min_z[15]}}, box_min_z}) - $signed({{16{ray_orig_z[15]}}, ray_orig_z})) * $signed({{16{ray_inv_z[15]}}, ray_inv_z});
                stg1_tz2 <= ($signed({{16{box_max_z[15]}}, box_max_z}) - $signed({{16{ray_orig_z[15]}}, ray_orig_z})) * $signed({{16{ray_inv_z[15]}}, ray_inv_z});
            end 
        end
    end

    // [Stage 2] Min/Max Swapper 레지스터
    reg signed [15:0] r_tmin_x, r_tmax_x;
    reg signed [15:0] r_tmin_y, r_tmax_y;
    reg signed [15:0] r_tmin_z, r_tmax_z;

    // [오류 수정 3] Q16.16 -> Q8.8 복원 시 비트 슬라이싱 [23:8] 적용 및 부호 캐스팅 유지 (정상 범주 내 동작 가정)
    wire signed [15:0] scaled_tx1 = $signed(stg1_tx1[23:8]);
    wire signed [15:0] scaled_tx2 = $signed(stg1_tx2[23:8]);
    wire signed [15:0] scaled_ty1 = $signed(stg1_ty1[23:8]);
    wire signed [15:0] scaled_ty2 = $signed(stg1_ty2[23:8]);
    wire signed [15:0] scaled_tz1 = $signed(stg1_tz1[23:8]);
    wire signed [15:0] scaled_tz2 = $signed(stg1_tz2[23:8]);

    always @(posedge CLK or negedge RSTN) begin 
        if (!RSTN) begin
            r_tmin_x <= 16'd0; r_tmax_x <= 16'd0;
            r_tmin_y <= 16'd0; r_tmax_y <= 16'd0;
            r_tmin_z <= 16'd0; r_tmax_z <= 16'd0;
        end else begin
            if (pipe_valid[0]) begin 
                r_tmin_x <= (scaled_tx1 > scaled_tx2) ? scaled_tx2 : scaled_tx1; 
                r_tmax_x <= (scaled_tx1 > scaled_tx2) ? scaled_tx1 : scaled_tx2; 
                r_tmin_y <= (scaled_ty1 > scaled_ty2) ? scaled_ty2 : scaled_ty1; 
                r_tmax_y <= (scaled_ty1 > scaled_ty2) ? scaled_ty1 : scaled_ty2;
                r_tmin_z <= (scaled_tz1 > scaled_tz2) ? scaled_tz2 : scaled_tz1;
                r_tmax_z <= (scaled_tz1 > scaled_tz2) ? scaled_tz1 : scaled_tz2;
            end 
        end
    end

    // =========================================================================
    // [5] Reduction Tree & Final Output (Combinational Logic)
    // =========================================================================
    // [보완] X, Y축 t_min 중 최댓값 도출 후 Z축과 비교 (광선 진입점인 최대 하한선 탐색)
    wire signed [15:0] t_min_inter = (r_tmin_x > r_tmin_y) ? r_tmin_x : r_tmin_y; 
    wire signed [15:0] t_min_final = (t_min_inter > r_tmin_z) ? t_min_inter : r_tmin_z; 

    // [보완] X, Y축 t_max 중 최솟값 도출 후 Z축과 비교 (광선 이탈점인 최소 상한선 탐색)
    wire signed [15:0] t_max_inter = (r_tmax_x < r_tmax_y) ? r_tmax_x : r_tmax_y;  
    wire signed [15:0] t_max_final = (t_max_inter < r_tmax_z) ? t_max_inter : r_tmax_z; 

    // [보완] 최종 교차(Hit) 판별: 상한선이 하한선보다 크거나 같고, 상한선이 0 이상(Box가 광선 진행 방향 앞)이어야 함
    wire        pipeline_hit             = (t_max_final >= t_min_final) && (t_max_final >= 16'sd0); 
    
    // [오류 수정 C] 광선이 Box 내부에서 시작될 경우 t_min_final은 음수가 됨. 이때는 t_max_final을 교차점으로 사용해야 함.
    wire signed [15:0] t_hit_final       = (t_min_final < 16'sd0) ? t_max_final : t_min_final;
    
    // [오류 수정 D] 남은 15비트 패딩 시 t_hit_final의 부호에 맞추어 부호 확장을 적용 (안정성 강화)
    wire [31:0] pipeline_computed_result = { pipeline_hit, {15{t_hit_final[15]}}, t_hit_final }; 

    // [보완] 최종 출력 다중화기 제어 및 캐시 업데이트
    always @(posedge CLK or negedge RSTN) begin 
        if (!RSTN) begin
            IPOUT          <= 32'd0;
            IP_VALID       <= 1'b0;
            cache_dir_x    <= 16'd0; cache_dir_y <= 16'd0; cache_dir_z <= 16'd0;
            cache_hit_data <= 32'd0;
            cache_valid    <= 1'b0;  
        end else begin
            // [보완] IP_VALID는 기본적으로 0으로 떨어지며, 유효한 사이클에서만 1클록 펄스를 생성함
            IP_VALID <= 1'b0; 

            if (parsing_done && is_coherent) begin
                // [Smart Bypass] 캐시 히트 시 파이프라인 연산 스킵
                IPOUT    <= cache_hit_data;
                IP_VALID <= 1'b1;
            end 
            // [오류 수정 E] 파이프라인 단계에 정확하게 맞추어 pipe_valid[1]에서 결과 래치
            else if (pipe_valid[1]) begin 
                // [Normal Compute] 파이프라인 최종 결과 출력 및 캐시 갱신
                IPOUT          <= pipeline_computed_result;
                IP_VALID       <= 1'b1;
                
                // [보완] 파서가 아직 덮어쓰기 전의 이전 광선 데이터를 안전하게 캐싱 (정확한 사이클 타이밍)
                cache_dir_x    <= ray_dir_x;
                cache_dir_y    <= ray_dir_y;
                cache_dir_z    <= ray_dir_z;
                cache_hit_data <= pipeline_computed_result;
                cache_valid    <= 1'b1; 
            end 
            else if (!ip_start) begin
                // [오류 수정 2] CPU 가동 정지 상태여도 캐시는 파괴하지 않고 유지! IPOUT만 클리어.
                IPOUT          <= 32'd0;
            end 
        end
    end

endmodule
