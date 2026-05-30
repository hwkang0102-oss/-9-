`timescale 1ns / 1ps

/*******************************************************************************
 * Core Architecture: 
 * TC-RTBIA (Temporal Coherence - Ray Tracing Bypassing & Intersection Accelerator)
 * * Protocol Wrapper Integration (RISC-TOY Compatible):
 * 1. Protocol Decoder: Magic Key (4'hA) 및 제어 신호(CON[11:8]) 에지 검출
 * 2. Status Output: Stream Signature 암호화 및 상태 워드 레지스터 
 * 3. Ray Data Parser: sample_toggle 동기화 방식의 순차 역다중화 분해기 및 대기열 로직
 * 4. Temporal Coherence Cache: 부호 확장 오차 비교기 및 스마트 우회 제어
 * 5. T-Intersection Pipeline: 부호 보존 6-병렬 곱셈기 및 정렬 스와퍼 파이프라인
 * 6. Reduction Tree & Output MUX: 다단 토너먼트 및 output_select 기반 포워딩
 *******************************************************************************/

module CUSTOM_IP (
    input  wire        CLK,      // 가속기 구동용 시스템 메인 클록
    input  wire        RSTN,     // 액티브 로우(Active-Low) 글로벌 비동기 리셋 신호
    
    // RISC_TOY Host Control Interface (사양서 명시 포트 5개로 원복)
    input  wire [31:0] CON,      // 제어 레지스터 (프로토콜 필수 제어 및 파라미터)
    input  wire [31:0] IPIN,     // 데이터 메모리에서 유입되는 32비트 데이터
    output reg  [31:0] IPOUT     // 결과 또는 프로토콜 상태 워드 출력 버스
);

    // =========================================================================
    // [1] Protocol Decoder & Control Wrapper: 필수 제어 규격 해독
    // =========================================================================
    // 사양서 규격에 따른 CON 비트맵 파싱
    wire        key_valid     = (CON[15:12] == 4'hA); // 보안 매직 키 검증기
    wire        clear_config  = CON[11];              // 프로토콜 상태 초기화 트리거
    wire        sample_toggle = CON[10];              // 데이터 수신용 토글 트리거
    wire        finish_toggle = CON[9];               // 연산 완료 요청 트리거
    wire        output_select = CON[8];               // 출력 MUX 제어 (1: 상태 워드, 0: 연산 결과)
    wire [7:0]  user_config   = CON[7:0];             // 유저 정의 설정 바이트
    
    // 비어있는 상위 16비트를 임계값 파라미터로 재활용하는 최적화
    wire signed [15:0] epsilon_th = $signed(CON[31:16]); 

    // 토글 에지 검출을 위한 추적(Tracking) 플립플롭
    reg prev_sample_toggle;
    reg prev_finish_toggle;
    reg [7:0] config_byte;

    // 현재 클록에서의 에지(Edge) 발생 여부 판별 로직
    wire sample_edge = key_valid && !clear_config && (sample_toggle != prev_sample_toggle);
    wire finish_edge = key_valid && !clear_config && (finish_toggle != prev_finish_toggle);

    // =========================================================================
    // [2] Mandatory Protocol Status: 필수 상태 워드 및 시그니처 연산기
    // =========================================================================
    reg [4:0]  accepted_sample_count; // 샘플 수신 카운터 (최대 32)
    reg [15:0] stream_signature;      // 데이터 무결성 검증용 16비트 서명
    reg        done;                  // 연산 완료 플래그
    reg        finish_pending;        // 파이프라인 연산 종료 대기열 플래그 (타이밍 버그 방어용)

    // 파이프라인 유효성 추적 선언 (아래 파이프라인 블록에서 제어됨)
    reg [1:0] pipe_valid;

    // 시그니처 갱신 수학 공식 하드웨어 구현
    wire [15:0] sig_rotated = {stream_signature[14:0], stream_signature[15]};
    wire [15:0] ipin_xor    = IPIN[15:0] ^ IPIN[31:16];
    wire [15:0] sig_next    = sig_rotated ^ ipin_xor ^ {8'h00, config_byte};

    // =========================================================================
    // [3] Ray Data Parser: sample_toggle 동기화 역다중화 분해기
    // =========================================================================
    reg signed [15:0] ray_orig_x, ray_orig_y, ray_orig_z; 
    reg signed [15:0] ray_inv_x,  ray_inv_y,  ray_inv_z;
    reg signed [15:0] ray_dir_x,  ray_dir_y,  ray_dir_z;
    reg signed [15:0] box_min_x,  box_min_y,  box_min_z;
    reg signed [15:0] box_max_x,  box_max_y,  box_max_z;
    reg parsing_done; 

    // 상태 워드 관리 및 데이터 파싱 통합 제어 블록
    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            prev_sample_toggle    <= 1'b0;
            prev_finish_toggle    <= 1'b0;
            config_byte           <= 8'd0;
            accepted_sample_count <= 5'd0;
            stream_signature      <= 16'd0;
            done                  <= 1'b0;
            finish_pending        <= 1'b0;
            parsing_done          <= 1'b0;
            
            // 데이터 레지스터 초기화
            ray_orig_x <= 16'd0; ray_orig_y <= 16'd0; ray_orig_z <= 16'd0;
            ray_inv_x  <= 16'd0; ray_inv_y  <= 16'd0; ray_inv_z  <= 16'd0;
            ray_dir_x  <= 16'd0; ray_dir_y  <= 16'd0; ray_dir_z  <= 16'd0;
            box_min_x  <= 16'd0; box_min_y  <= 16'd0; box_min_z  <= 16'd0;
            box_max_x  <= 16'd0; box_max_y  <= 16'd0; box_max_z  <= 16'd0;
        end else if (key_valid) begin
            if (clear_config) begin
                // 명령 해독: clear_config가 1이면 내부 추적 상태 및 프로토콜 변수 전면 포맷
                prev_sample_toggle    <= sample_toggle;
                prev_finish_toggle    <= finish_toggle;
                accepted_sample_count <= 5'd0;
                stream_signature      <= 16'd0;
                done                  <= 1'b0;
                finish_pending        <= 1'b0;
                config_byte           <= user_config;
                parsing_done          <= 1'b0;
            end else begin
                // 명령 해독: Finish 토글 에지 검출 시 대기열 등록 (파이프라인 비우기)
                if (finish_edge) begin
                    prev_finish_toggle <= finish_toggle;
                    finish_pending     <= 1'b1; 
                end
                
                // 파이프라인이 텅 비어서 데이터 연산이 최종 안정화(Stable) 되었을 때 비로소 done 활성화
                if (finish_pending && !pipe_valid[0] && !pipe_valid[1]) begin
                    done           <= 1'b1;
                    finish_pending <= 1'b0;
                end
                
                // 명령 해독: Sample 토글 에지 검출 시 데이터 샘플 1개 수용 및 시그니처 갱신
                parsing_done <= 1'b0; // 디폴트는 Low로 유지하여 1클록 펄스 생성 준비

                if (sample_edge) begin
                    prev_sample_toggle    <= sample_toggle;
                    stream_signature      <= sig_next; 
                    accepted_sample_count <= accepted_sample_count + 5'd1;

                    // 하위 3비트(0~7)를 파싱 인덱스로 사용하여 3D 공간 데이터 역다중화 캡처
                    case (accepted_sample_count[2:0])
                        3'd0: begin ray_orig_x <= $signed(IPIN[31:16]); ray_orig_y <= $signed(IPIN[15:0]); end
                        3'd1: begin ray_orig_z <= $signed(IPIN[31:16]); ray_inv_x  <= $signed(IPIN[15:0]); end
                        3'd2: begin ray_inv_y  <= $signed(IPIN[31:16]); ray_inv_z  <= $signed(IPIN[15:0]); end
                        3'd3: begin ray_dir_x  <= $signed(IPIN[31:16]); ray_dir_y  <= $signed(IPIN[15:0]); end
                        3'd4: begin ray_dir_z  <= $signed(IPIN[31:16]); box_min_x  <= $signed(IPIN[15:0]); end
                        3'd5: begin box_min_y  <= $signed(IPIN[31:16]); box_min_z  <= $signed(IPIN[15:0]); end
                        3'd6: begin box_max_x  <= $signed(IPIN[31:16]); box_max_y  <= $signed(IPIN[15:0]); end
                        3'd7: begin 
                            box_max_z <= $signed(IPIN[31:16]); 
                            parsing_done <= 1'b1; // 8번째(3'd7) 샘플이 수신되는 즉시 파이프라인 점화 펄스 트리거
                        end
                    endcase
                end
            end
        end
    end

    // =========================================================================
    // [4] Temporal Coherence Cache & Bypass Logic
    // =========================================================================
    reg signed [15:0] cache_dir_x, cache_dir_y, cache_dir_z;
    reg [31:0]        cache_hit_data;
    reg               cache_valid;

    // 부호 확장(Sign Extension)을 통한 17비트 오차 절댓값 산출기
    wire signed [16:0] diff_x_abs = ($signed({ray_dir_x[15], ray_dir_x}) - $signed({cache_dir_x[15], cache_dir_x}) < 0) ? 
                                   -($signed({ray_dir_x[15], ray_dir_x}) - $signed({cache_dir_x[15], cache_dir_x})) : 
                                    ($signed({ray_dir_x[15], ray_dir_x}) - $signed({cache_dir_x[15], cache_dir_x}));
                                    
    wire signed [16:0] diff_y_abs = ($signed({ray_dir_y[15], ray_dir_y}) - $signed({cache_dir_y[15], cache_dir_y}) < 0) ? 
                                   -($signed({ray_dir_y[15], ray_dir_y}) - $signed({cache_dir_y[15], cache_dir_y})) : 
                                    ($signed({ray_dir_y[15], ray_dir_y}) - $signed({cache_dir_y[15], cache_dir_y}));

    wire signed [16:0] diff_z_abs = ($signed({ray_dir_z[15], ray_dir_z}) - $signed({cache_dir_z[15], cache_dir_z}) < 0) ? 
                                   -($signed({ray_dir_z[15], ray_dir_z}) - $signed({cache_dir_z[15], cache_dir_z})) : 
                                    ($signed({ray_dir_z[15], ray_dir_z}) - $signed({cache_dir_z[15], cache_dir_z}));

    wire signed [16:0] ext_epsilon = $signed({epsilon_th[15], epsilon_th});
    
    // 오차 임계값 대조를 통한 스마트 우회 판별 플래그
    wire is_coherent = (diff_x_abs <= ext_epsilon) && (diff_y_abs <= ext_epsilon) && (diff_z_abs <= ext_epsilon) && cache_valid;
    wire enable_core_pipeline = parsing_done && !is_coherent;

    // =========================================================================
    // [5] T-Intersection Pipeline (6-Parallel Multipliers & Swapper)
    // =========================================================================
    reg signed [31:0] stg1_tx1, stg1_tx2, stg1_ty1, stg1_ty2, stg1_tz1, stg1_tz2;

    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            stg1_tx1 <= 32'd0; stg1_tx2 <= 32'd0;
            stg1_ty1 <= 32'd0; stg1_ty2 <= 32'd0;
            stg1_tz1 <= 32'd0; stg1_tz2 <= 32'd0;
            pipe_valid <= 2'b0;
        end else begin
            pipe_valid[0] <= enable_core_pipeline;
            pipe_valid[1] <= pipe_valid[0];
            
            if (enable_core_pipeline) begin
                stg1_tx1 <= ($signed({{16{box_min_x[15]}}, box_min_x}) - $signed({{16{ray_orig_x[15]}}, ray_orig_x})) * $signed({{16{ray_inv_x[15]}}, ray_inv_x});
                stg1_tx2 <= ($signed({{16{box_max_x[15]}}, box_max_x}) - $signed({{16{ray_orig_x[15]}}, ray_orig_x})) * $signed({{16{ray_inv_x[15]}}, ray_inv_x});
                stg1_ty1 <= ($signed({{16{box_min_y[15]}}, box_min_y}) - $signed({{16{ray_orig_y[15]}}, ray_orig_y})) * $signed({{16{ray_inv_y[15]}}, ray_inv_y});
                stg1_ty2 <= ($signed({{16{box_max_y[15]}}, box_max_y}) - $signed({{16{ray_orig_y[15]}}, ray_orig_y})) * $signed({{16{ray_inv_y[15]}}, ray_inv_y});
                stg1_tz1 <= ($signed({{16{box_min_z[15]}}, box_min_z}) - $signed({{16{ray_orig_z[15]}}, ray_orig_z})) * $signed({{16{ray_inv_z[15]}}, ray_inv_z});
                stg1_tz2 <= ($signed({{16{box_max_z[15]}}, box_max_z}) - $signed({{16{ray_orig_z[15]}}, ray_orig_z})) * $signed({{16{ray_inv_z[15]}}, ray_inv_z});
            end
        end
    end

    // 소수점 스케일링(>>> 8) 및 Min/Max Swapper
    reg signed [15:0] r_tmin_x, r_tmax_x, r_tmin_y, r_tmax_y, r_tmin_z, r_tmax_z;

    wire signed [15:0] scaled_tx1 = $signed(stg1_tx1 >>> 8);
    wire signed [15:0] scaled_tx2 = $signed(stg1_tx2 >>> 8);
    wire signed [15:0] scaled_ty1 = $signed(stg1_ty1 >>> 8);
    wire signed [15:0] scaled_ty2 = $signed(stg1_ty2 >>> 8);
    wire signed [15:0] scaled_tz1 = $signed(stg1_tz1 >>> 8);
    wire signed [15:0] scaled_tz2 = $signed(stg1_tz2 >>> 8);

    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            r_tmin_x <= 16'd0; r_tmax_x <= 16'd0;
            r_tmin_y <= 16'd0; r_tmax_y <= 16'd0;
            r_tmin_z <= 16'd0; r_tmax_z <= 16'd0;
        end else if (pipe_valid[0]) begin
            r_tmin_x <= (scaled_tx1 > scaled_tx2) ? scaled_tx2 : scaled_tx1;
            r_tmax_x <= (scaled_tx1 > scaled_tx2) ? scaled_tx1 : scaled_tx2;
            r_tmin_y <= (scaled_ty1 > scaled_ty2) ? scaled_ty2 : scaled_ty1;
            r_tmax_y <= (scaled_ty1 > scaled_ty2) ? scaled_ty1 : scaled_ty2;
            r_tmin_z <= (scaled_tz1 > scaled_tz2) ? scaled_tz2 : scaled_tz1;
            r_tmax_z <= (scaled_tz1 > scaled_tz2) ? scaled_tz1 : scaled_tz2;
        end
    end

    // =========================================================================
    // [6] Reduction Tree & Protocol Compliant Output MUX
    // =========================================================================
    // 진입점 및 이탈점 산출 토너먼트 로직
    wire signed [15:0] t_min_inter = (r_tmin_x > r_tmin_y) ? r_tmin_x : r_tmin_y;
    wire signed [15:0] t_min_final = (t_min_inter > r_tmin_z) ? t_min_inter : r_tmin_z;

    wire signed [15:0] t_max_inter = (r_tmax_x < r_tmax_y) ? r_tmax_x : r_tmax_y;  
    wire signed [15:0] t_max_final = (t_max_inter < r_tmax_z) ? t_max_inter : r_tmax_z;

    wire               pipeline_hit = (t_max_final >= t_min_final) && (t_max_final >= 16'sd0);
    wire signed [15:0] t_hit_final  = (t_min_final < 16'sd0) ? t_max_final : t_min_final;
    wire [31:0]        pipeline_computed_result = { pipeline_hit, {15{t_hit_final[15]}}, t_hit_final };

    // 가속기가 산출한 최종 정답을 안전하게 보관할 레지스터 (프로토콜 상태 워드와 분리 보관)
    reg [31:0] workload_result_reg;

    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            workload_result_reg <= 32'd0;
            cache_dir_x <= 16'd0; cache_dir_y <= 16'd0; cache_dir_z <= 16'd0;
            cache_hit_data <= 32'd0;
            cache_valid <= 1'b0;
        end else begin
            // [Smart Bypass]: 연산을 스킵하고 캐시된 정답을 바로 결과 레지스터에 로드
            if (parsing_done && is_coherent) begin
                workload_result_reg <= cache_hit_data;
            end 
            // [Normal Compute]: 파이프라인 연산이 끝나면 결과 레지스터 갱신 및 캐시 백업
            else if (pipe_valid[1]) begin
                workload_result_reg <= pipeline_computed_result;
                
                cache_dir_x    <= ray_dir_x;
                cache_dir_y    <= ray_dir_y;
                cache_dir_z    <= ray_dir_z;
                cache_hit_data <= pipeline_computed_result;
                cache_valid    <= 1'b1;
            end
        end
    end

    // 최종 출력(MUX)단: 사양서 기준 output_select 값에 따라 하드웨어 물리 포워딩 분기
    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            IPOUT <= 32'd0;
        end else if (key_valid) begin
            if (output_select) begin
                // CON[8] == 1 일 때: 강제 규격 상태 워드 (Mandatory Protocol Status Word) 패킹 출력
                // 구조: { Magic Code(0x26), Done 플래그, Reserved(00), Sample Count, Signature }
                IPOUT <= { 8'h26, done, 2'b00, accepted_sample_count, stream_signature };
            end else begin
                // CON[8] == 0 일 때: 우리가 설계한 레이 트레이싱 가속 최종 정답 출력
                IPOUT <= workload_result_reg;
            end
        end else begin
            // 매직 키가 없거나 스위치가 꺼진 상태에서는 버스 안정화를 위해 Flush
            IPOUT <= 32'd0;
        end
    end

endmodule
