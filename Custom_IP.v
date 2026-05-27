`timescale 1ns / 1ps

module CUSTOM_IP (
    input  wire        CLK,      
    input  wire        RSTN,     
    input  wire [31:0] CON,      
    input  wire [31:0] IPIN,     
    output reg  [31:0] IPOUT,    
    output reg         IP_VALID  
);

    // =========================================================================
    // [1] System Control
    // =========================================================================
    wire                ip_start   = CON[0]; 
    wire signed [15:0] epsilon_th = $signed(CON[31:16]); 

    // =========================================================================
    // [2] Ray Data Parser
    // =========================================================================
    reg [2:0]  packet_cnt; 
    reg signed [15:0] ray_orig_x, ray_orig_y, ray_orig_z; 
    reg signed [15:0] ray_inv_x,  ray_inv_y,  ray_inv_z;
    reg signed [15:0] ray_dir_x,  ray_dir_y,  ray_dir_z;
    reg signed [15:0] box_min_x,  box_min_y,  box_min_z;
    reg signed [15:0] box_max_x,  box_max_y,  box_max_z;
    reg parsing_done; 

    // 타이밍 버그 및 데이터 드라이버 문제 해결을 위한 통합 Sequential 블록
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
            // 데이터 수신과 카운터 업데이트 싱크 완료
            case (packet_cnt) 
                3'd0: begin ray_orig_x <= $signed(IPIN[31:16]); ray_orig_y <= $signed(IPIN[15:0]); end
                3'd1: begin ray_orig_z <= $signed(IPIN[31:16]); ray_inv_x  <= $signed(IPIN[15:0]); end
                3'd2: begin ray_inv_y  <= $signed(IPIN[31:16]); ray_inv_z  <= $signed(IPIN[15:0]); end
                3'd3: begin ray_dir_x  <= $signed(IPIN[31:16]); ray_dir_y  <= $signed(IPIN[15:0]); end
                3'd4: begin ray_dir_z  <= $signed(IPIN[31:16]); box_min_x  <= $signed(IPIN[15:0]); end
                3'd5: begin box_min_y  <= $signed(IPIN[31:16]); box_min_z  <= $signed(IPIN[15:0]); end
                3'd6: begin box_max_x  <= $signed(IPIN[31:16]); box_max_y  <= $signed(IPIN[15:0]); end
                3'd7: begin box_max_z  <= $signed(IPIN[31:16]); end 
            endcase

            if (packet_cnt == 3'd7) begin
                packet_cnt   <= 3'd0;
                parsing_done <= 1'b1; 
            end else begin
                packet_cnt   <= packet_cnt + 3'd1; 
                parsing_done <= 1'b0; 
            end
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

    wire signed [16:0] ext_epsilon = $signed({epsilon_th[15], epsilon_th});
    
    wire is_coherent = (diff_x_abs <= ext_epsilon) && (diff_y_abs <= ext_epsilon) && (diff_z_abs <= ext_epsilon) && cache_valid; 
    wire enable_core_pipeline = parsing_done && !is_coherent; 

    // =========================================================================
    // [4] T-Intersection Pipeline (Stage 1 & 2)
    // =========================================================================
    reg [1:0] pipe_valid;
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

    reg signed [15:0] r_tmin_x, r_tmax_x;
    reg signed [15:0] r_tmin_y, r_tmax_y;
    reg signed [15:0] r_tmin_z, r_tmax_z;

    // [부호 파괴 버그 수정] Arithmetic Right Shift (>>>) 연산자로 스케일링 복원
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

    // 안전 지대 판별 및 부호 예외 검증 완료
    wire        pipeline_hit             = (t_max_final >= t_min_final) && (t_max_final >= 16'sd0); 
    wire signed [15:0] t_hit_final       = (t_min_final < 16'sd0) ? t_max_final : t_min_final;
    wire [31:0] pipeline_computed_result = { pipeline_hit, {15{t_hit_final[15]}}, t_hit_final }; 

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
                IPOUT    <= cache_hit_data;
                IP_VALID <= 1'b1;
            end 
            else if (pipe_valid[1]) begin 
                IPOUT          <= pipeline_computed_result;
                IP_VALID       <= 1'b1;
                cache_dir_x    <= ray_dir_x;
                cache_dir_y    <= ray_dir_y;
                cache_dir_z    <= ray_dir_z;
                cache_hit_data <= pipeline_computed_result;
                cache_valid    <= 1'b1; 
            end 
            else if (!ip_start) begin
                IPOUT          <= 32'd0;
            end 
        end
    end

endmodule
