`timescale 1ns / 1ps

/*******************************************************************************
 * Core Architecture: 
 * TC-RTBIA (Temporal Coherence - Ray Tracing Bypassing & Intersection Accelerator)
 *******************************************************************************/

module CUSTOM_IP (
    input  wire        CLK,     // 시스템 메인 클록 신호
    input  wire        RSTN,    // 시스템 액티브 로우(Active-Low) 리셋 신호
    
    // RISC_TOY Interface (호스트 CPU 통신용 제어 인터페이스)
    input  wire [31:0] CON,     // CPU R[31]에서 인입: CON[0]=Start, CON[31:16]=Epsilon
    
    // Data Memory Interface (데이터 메모리 직접 연결 인터페이스)
    input  wire [31:0] IPIN,    // 데이터 메모리에서 들어오는 32비트 패킷 버스
    output reg  [31:0] IPOUT,   // 최종 출력 버스: [31]=Hit 플래그, [30:0]=최종 계산된 t 값
    output reg         IP_VALID // 호스트 CPU에게 데이터 준비 완료를 알리는 핸드셰이크 플래그
);

    // =========================================================================
    // [1] System Control: 제어 신호 및 파라미터 추출
    // =========================================================================
    wire               ip_start   = CON[0]; 
    wire signed [15:0] epsilon_th = $signed(CON[31:16]); 

    // =========================================================================
    // [2] Ray Data Parser: 입출력 전처리 및 패킷 분해
    // =========================================================================
    reg [2:0]   packet_cnt; 
    
    reg signed [15:0] ray_orig_x, ray_orig_y, ray_orig_z; 
    reg signed [15:0] ray_inv_x,  ray_inv_y,  ray_inv_z;
    reg signed [15:0] ray_dir_x,  ray_dir_y,  ray_dir_z;
    reg signed [15:0] box_min_x,  box_min_y,  box_min_z;
    reg signed [15:0] box_max_x,  box_max_y,  box_max_z;
    
    reg parsing_done; 

    always @(posedge CLK or negedge RSTN) begin 
        if (!RSTN) begin 
            packet_cnt   <= 3'd0;
            parsing_done <= 1'b0;
            
            ray_orig_x   <= 16'd0; ray_orig_y   <= 16'd0; ray_orig_z   <= 16'd0;
            ray_inv_x    <= 16'd0; ray_inv_y    <= 16'd0; ray_inv_z    <= 16'd0;
            ray_dir_x    <= 16'd0; ray_dir_y    <= 16'd0; ray_dir_z    <= 16'd0;
            box_min_x    <= 16'd0; box_min_y    <= 16'd0; box_min_z    <= 16'd0;
            box_max_x    <= 16'd0; box_max_y    <= 16'd0; box_max_z    <= 16'd0;
        end else if (ip_start) begin 
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
    
    wire signed [15:0] diff_x, diff_y, diff_z; 
    
    assign diff_x = (ray_dir_x > cache_dir_x) ? (ray_dir_x - cache_dir_x) : (cache_dir_x - ray_dir_x); 
    assign diff_y = (ray_dir_y > cache_dir_y) ? (ray_dir_y - cache_dir_y) : (cache_dir_y - ray_dir_y);
    assign diff_z = (ray_dir_z > cache_dir_z) ? (ray_dir_z - cache_dir_z) : (cache_dir_z - ray_dir_z);

    wire is_coherent = (diff_x < epsilon_th) && (diff_y < epsilon_th) && (diff_z < epsilon_th) && cache_valid; 
    
    wire enable_core_pipeline = parsing_done && !is_coherent; 

    // =========================================================================
    // [4] T-Intersection Pipeline (Stage 1 & 2)
    // =========================================================================
    reg [2:0] pipe_valid;

    // [Stage 1] 하드웨어 곱셈기 (결과 포맷: Q16.16)
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
            pipe_valid[0] <= enable_core_pipeline;
            pipe_valid[1] <= pipe_valid[0];
            pipe_valid[2] <= pipe_valid[1]; 
            
            // [오류 수정 1] 단일 펄스 이후 강제 Flush 구문 제거 -> 값 안전하게 유지(Latch)
            if (enable_core_pipeline) begin 
                stg1_tx1 <= (box_min_x - ray_orig_x) * ray_inv_x;
                stg1_tx2 <= (box_max_x - ray_orig_x) * ray_inv_x;
                stg1_ty1 <= (box_min_y - ray_orig_y) * ray_inv_y;
                stg1_ty2 <= (box_max_y - ray_orig_y) * ray_inv_y;
                stg1_tz1 <= (box_min_z - ray_orig_z) * ray_inv_z;
                stg1_tz2 <= (box_max_z - ray_orig_z) * ray_inv_z;
            end 
        end
    end

    // [Stage 2] Min/Max Swapper 레지스터
    reg signed [15:0] r_tmin_x, r_tmax_x;
    reg signed [15:0] r_tmin_y, r_tmax_y;
    reg signed [15:0] r_tmin_z, r_tmax_z;

    // [오류 수정 3] Q16.16 -> Q8.8 복원 시 비트 슬라이싱 [23:8] 적용 및 부호 캐스팅
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
    // [5] Reduction Tree & Final Output
    // =========================================================================
    wire signed [15:0] t_min_inter = (r_tmin_x > r_tmin_y) ? r_tmin_x : r_tmin_y; 
    wire signed [15:0] t_min_final = (t_min_inter > r_tmin_z) ? t_min_inter : r_tmin_z; 

    wire signed [15:0] t_max_inter = (r_tmax_x < r_tmax_y) ? r_tmax_x : r_tmax_y;  
    wire signed [15:0] t_max_final = (t_max_inter < r_tmax_z) ? t_max_inter : r_tmax_z; 

    wire        pipeline_hit             = (t_max_final >= t_min_final) && (t_max_final >= 16'sd0); 
    wire [31:0] pipeline_computed_result = { pipeline_hit, 15'd0, t_min_final }; 

    // 출력 다중화기 제어 
    always @(posedge CLK or negedge RSTN) begin 
        if (!RSTN) begin
            IPOUT          <= 32'd0;
            IP_VALID       <= 1'b0;
            cache_dir_x    <= 16'd0; cache_dir_y <= 16'd0; cache_dir_z <= 16'd0;
            cache_hit_data <= 32'd0;
            cache_valid    <= 1'b0;  
        end else begin
            IP_VALID <= 1'b0; 

            if (parsing_done && is_coherent) begin
                // [Smart Bypass] 
                IPOUT    <= cache_hit_data;
                IP_VALID <= 1'b1;
            end 
            else if (pipe_valid[2]) begin 
                // [Normal Compute] 
                IPOUT          <= pipeline_computed_result;
                IP_VALID       <= 1'b1;
                
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
