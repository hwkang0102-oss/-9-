`timescale 1ns / 1ps

/*******************************************************************************
 * Core Architecture: 
 * TC-RTBIA (Temporal Coherence - Ray Tracing Bypassing & Intersection Accelerator)
 *
 * Structural Specification:
 * 1. Protocol Decoder: 필수 매직 키(Magic Key) 및 제어 신호 토글 에지 검출망
 * 2. Status Output: 필수 상태 워드 패킹 및 16비트 시그니처 암호화 연산기
 * 3. Ray Data Parser: sample_toggle 동기화 기반 3D 공간 데이터 역다중화기(DEMUX)
 * 4. Temporal Coherence Cache: 부호 확장 오차 비교기 및 전력 차단(Clock Gating) 제어
 * 5. T-Intersection Pipeline: 부호 보존 6채널 병렬 곱셈기 및 Min/Max 자동 정렬 회로
 * 6. Reduction Tree & Output MUX: 2단계 하드웨어 토너먼트 트리 및 출력 데이터 포워딩
 *******************************************************************************/

module CUSTOM_IP (
    input  wire        CLK,      // 가속기 내부의 심장 박동을 관장하는 시스템 메인 클록 신호
    input  wire        RSTN,     // 액티브 로우(Active-Low)로 동작하며 칩 전체를 초기화하는 글로벌 리셋 스위치
    
    // RISC_TOY Host Control Interface
    input  wire [31:0] CON,      // CPU의 R[31] 레지스터에서 인입되는 32비트 제어 버스 (매직 키, 토글, 파라미터 포함)
    
    // Data Memory Interface
    input  wire [31:0] IPIN,     // 데이터 메모리에서 스트리밍으로 유입되는 32비트 원시 데이터 패킷 버스
    output reg  [31:0] IPOUT     // 연산 정답 또는 프로토콜 상태 워드를 최종적으로 메모리에 쏘아주는 출력 버스
);

    // =========================================================================
    // [1] Protocol Decoder & Control Wrapper: 필수 제어 규격 해독망
    // =========================================================================
    // 사양서 규격에 맞춰 32비트 CON 버스에 실려온 신호들을 각자 역할에 맞게 스위치 선으로 가닥가닥 쪼개어 연결합니다.
    wire        key_valid     = (CON[15:12] == 4'hA); // 상위 4비트가 'A'인지 확인하는 보안 매직 키 자물쇠 회로입니다.
    wire        clear_config  = CON[11];              // 프로토콜 상태와 내부 캐시를 싹 비우는 초기화 트리거 스위치입니다.
    wire        sample_toggle = CON[10];              // CPU가 새 데이터를 IPIN에 올렸으니 가져가라고 흔드는 깃발(토글) 신호입니다.
    wire        finish_toggle = CON[9];               // 연산을 모두 마쳤으니 결과를 내놓으라는 마무리 요청 스위치입니다.
    wire        output_select = CON[8];               // 1이면 규격 상태 워드를, 0이면 가속 정답을 내보내게 하는 MUX 분기 제어선입니다.
    wire [7:0]  user_config   = CON[7:0];             // 유저가 임의로 설정하는 8비트 데이터 대역입니다.
    
    // 비어있는 상위 16비트를 레이 트레이싱 오차 임계값(입실론) 파라미터로 알뜰하게 재활용하는 최적화 공간입니다.
    wire signed [15:0] epsilon_th = $signed(CON[31:16]); 

    // 토글 에지(Toggle Edge) 검출을 위해 직전 클록의 스위치 상태를 기억해둘 1비트 추적 플립플롭 공간입니다.
    reg prev_sample_toggle;
    reg prev_finish_toggle;
    reg [7:0] config_byte;

    // 단순히 신호가 1로 떠있는 상태(Level)가 아니라, 스위치가 0->1 또는 1->0으로 딱 '바뀌는 찰나(Edge)'를 잡아내는 논리 게이트입니다.
    wire sample_edge = key_valid && !clear_config && (sample_toggle != prev_sample_toggle);
    wire finish_edge = key_valid && !clear_config && (finish_toggle != prev_finish_toggle);

    // =========================================================================
    // [2] Mandatory Protocol Status: 필수 상태 워드 및 무결성 서명 연산기
    // =========================================================================
    reg [4:0]  accepted_sample_count; // CPU로부터 샘플을 몇 개나 받아먹었는지 세는 5비트 카운터입니다.
    reg [15:0] stream_signature;      // 데이터가 오염 없이 잘 들어왔는지 증명하는 16비트 암호화 금고입니다.
    reg        done;                  // 가속기가 모든 계산을 끝냈음을 CPU에게 알리는 작업 완료 깃발입니다.
    reg        finish_pending;        // 파이프라인 안에 아직 계산 중인 데이터가 빠져나올 때까지 셔터를 내리지 않고 기다리게 하는 타이밍 방어용 대기열 플래그입니다.

    // 파이프라인의 연산 유효성을 추적하는 선언부 (아래 5번 파이프라인에서 시프트 됨)
    reg [1:0] pipe_valid;

    // 교수님 사양서의 시그니처 갱신 수학 공식을 단일 클록 조합 논리 회로로 풀어낸 하드웨어 수식망입니다.
    wire [15:0] sig_rotated = {stream_signature[14:0], stream_signature[15]}; // 비트를 한 칸씩 왼쪽으로 둥글게 밀어 넘기는 순환 시프트(Rotated) 결합망
    wire [15:0] ipin_xor    = IPIN[15:0] ^ IPIN[31:16];                       // 32비트 IPIN 데이터를 반으로 접어서 배타적 논리합(XOR) 처리
    wire [15:0] sig_next    = sig_rotated ^ ipin_xor ^ {8'h00, config_byte};  // 이전 3개 덩어리를 모두 XOR로 합쳐서 다음 클록에 저장할 암호화 정답을 도출

    // =========================================================================
    // [3] Ray Data Parser: 토글 동기화 3D 공간 데이터 역다중화 분해기
    // =========================================================================
    // 공간 연산에 필요한 15개의 16비트 부호형 공간 좌표 변수들을 저장할 물리 플립플롭 배열을 선언합니다.
    reg signed [15:0] ray_orig_x, ray_orig_y, ray_orig_z; 
    reg signed [15:0] ray_inv_x,  ray_inv_y,  ray_inv_z;
    reg signed [15:0] ray_dir_x,  ray_dir_y,  ray_dir_z;
    reg signed [15:0] box_min_x,  box_min_y,  box_min_z;
    reg signed [15:0] box_max_x,  box_max_y,  box_max_z;
    
    reg parsing_done; // 8조각의 데이터 수집이 완전히 끝났으니 메인 코어 엔진에 시동을 걸라는 트리거입니다.

    always @(posedge CLK or negedge RSTN) begin // 클록이 뛸 때마다 칩의 물리적 불정합을 통제하는 거대한 순차 논리 동기화 블록입니다.
        if (!RSTN) begin
            // 리셋 스위치가 눌리면 칩 안의 모든 쓰레기 데이터를 초기화하여 X-Propagation 오류를 원천 차단합니다.
            prev_sample_toggle    <= 1'b0;
            prev_finish_toggle    <= 1'b0;
            config_byte           <= 8'd0;
            accepted_sample_count <= 5'd0;
            stream_signature      <= 16'd0;
            done                  <= 1'b0;
            finish_pending        <= 1'b0;
            parsing_done          <= 1'b0;
            
            ray_orig_x <= 16'd0; ray_orig_y <= 16'd0; ray_orig_z <= 16'd0;
            ray_inv_x  <= 16'd0; ray_inv_y  <= 16'd0; ray_inv_z  <= 16'd0;
            ray_dir_x  <= 16'd0; ray_dir_y  <= 16'd0; ray_dir_z  <= 16'd0;
            box_min_x  <= 16'd0; box_min_y  <= 16'd0; box_min_z  <= 16'd0;
            box_max_x  <= 16'd0; box_max_y  <= 16'd0; box_max_z  <= 16'd0;
        end else if (key_valid) begin // 호스트 CPU가 보낸 매직 키 자물쇠가 풀려야만 가속기가 귀를 엽니다.
            if (clear_config) begin
                // CPU가 초기화 명령(clear_config)을 내리면, 수집했던 샘플 수와 암호화 시그니처 등을 0으로 깨끗이 포맷합니다.
                prev_sample_toggle    <= sample_toggle;
                prev_finish_toggle    <= finish_toggle;
                accepted_sample_count <= 5'd0;
                stream_signature      <= 16'd0;
                done                  <= 1'b0;
                finish_pending        <= 1'b0;
                config_byte           <= user_config;
                parsing_done          <= 1'b0;
            end else begin
                // [안전 종료 대기열 로직]: CPU가 작업 끝! 이라고 토글을 치면 곧바로 done=1을 주지 않고 대기열(finish_pending)에 먼저 올립니다.
                if (finish_edge) begin
                    prev_finish_toggle <= finish_toggle;
                    finish_pending     <= 1'b1; 
                end
                
                // 파이프라인 파도타기가 다 끝나서 내부 곱셈기(pipe_valid)가 텅 비었을 때(안정화 상태), 비로소 CPU에게 작업 완료 깃발(done=1)을 흔듭니다. 타이밍 버그 철벽 방어선입니다.
                if (finish_pending && !pipe_valid[0] && !pipe_valid[1]) begin
                    done           <= 1'b1;
                    finish_pending <= 1'b0;
                end
                
                parsing_done <= 1'b0; // 데이터 수집 중일 때는 연산 엔진이 멋대로 출발하지 못하게 파싱 완료 플래그를 0으로 강제 고정합니다.

                // CPU가 sample_toggle 핀의 전압을 딱 흔드는 순간(Edge)에만 데이터를 삼키고 시그니처를 갱신합니다.
                if (sample_edge) begin
                    prev_sample_toggle    <= sample_toggle;
                    stream_signature      <= sig_next; 
                    accepted_sample_count <= accepted_sample_count + 5'd1;

                    // 카운터의 맨 뒷자리 3비트(0~7)를 번지수로 써서, 직렬 IPIN 데이터를 16비트 3D 공간 데이터 15개로 착착 분배해 담는 역다중화기(DEMUX)입니다.
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
                            parsing_done <= 1'b1; // 대망의 마지막 8번째 퍼즐이 맞춰지는 즉시, 연산 엔진에게 출발 신호(parsing_done=1)를 1클록 동안 튕겨줍니다.
                        end
                    endcase
                end
            end
        end
    end

    // =========================================================================
    // [4] Temporal Coherence Cache: 방향 오차 캐싱 및 스마트 우회 제어 (클록 게이팅)
    // =========================================================================
    // 이전에 연산에 성공했던 광선의 방향 벡터와 그 결과 정답을 킵해둘 하드웨어 물리 캐시(금고) 공간입니다.
    reg signed [15:0] cache_dir_x, cache_dir_y, cache_dir_z;
    reg [31:0]        cache_hit_data;
    reg               cache_valid;

    // 16비트끼리 빼다가 오버플로우로 부호 비트가 깨지는 참사를 막기 위해, 상위 부호를 17비트로 비트 연장({[15], ...}) 시킨 뒤 절댓값을 뽑아내는 고속 하드웨어 연산기입니다.
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
    
    // X, Y, Z 세 축의 오차가 모두 임계값 미만이어서 '이전 광선이랑 거의 똑같음(Coherent)' 판정이 나면 이 선에 1이 들어옵니다.
    wire is_coherent = (diff_x_abs <= ext_epsilon) && (diff_y_abs <= ext_epsilon) && (diff_z_abs <= ext_epsilon) && cache_valid;
    
    // 데이터 수신이 다 끝났는데, 궤적이 달라서 캐시 우회에 실패(Miss)했을 때만 무거운 메인 코어 엔진 파이프라인의 전력을 켭니다. (발열과 전력을 막는 Clock Gating 효과)
    wire enable_core_pipeline = parsing_done && !is_coherent;

    // =========================================================================
    // [5] T-Intersection Pipeline: 6채널 병렬 곱셈기 및 스케일 다운 자동 정렬 회로
    // =========================================================================
    // 파이프라인 스테이지 1: 하드웨어 승산기를 이용한 t 연산 (조합 회로 무제한 병렬화)
    reg signed [31:0] stg1_tx1, stg1_tx2, stg1_ty1, stg1_ty2, stg1_tz1, stg1_tz2;

    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            stg1_tx1 <= 32'd0; stg1_tx2 <= 32'd0;
            stg1_ty1 <= 32'd0; stg1_ty2 <= 32'd0;
            stg1_tz1 <= 32'd0; stg1_tz2 <= 32'd0;
            pipe_valid <= 2'b0;
        end else begin
            pipe_valid[0] <= enable_core_pipeline; // 1단계 파이프라인 데이터 생존 신고
            pipe_valid[1] <= pipe_valid[0];        // 2단계 파이프라인 데이터 생존 신고
            
            if (enable_core_pipeline) begin
                // 레이 트레이싱 교차 연산의 근간 공식을 하드웨어화합니다. 
                // X, Y, Z 세 개 축에 대해 물리적인 하드웨어 곱셈기(Multiplier) 소자 6개를 동원해, 단 1클록 타이밍에 6개의 정답을 동시에 폭격하듯 산출해냅니다.
                stg1_tx1 <= ($signed({{16{box_min_x[15]}}, box_min_x}) - $signed({{16{ray_orig_x[15]}}, ray_orig_x})) * $signed({{16{ray_inv_x[15]}}, ray_inv_x});
                stg1_tx2 <= ($signed({{16{box_max_x[15]}}, box_max_x}) - $signed({{16{ray_orig_x[15]}}, ray_orig_x})) * $signed({{16{ray_inv_x[15]}}, ray_inv_x});
                stg1_ty1 <= ($signed({{16{box_min_y[15]}}, box_min_y}) - $signed({{16{ray_orig_y[15]}}, ray_orig_y})) * $signed({{16{ray_inv_y[15]}}, ray_inv_y});
                stg1_ty2 <= ($signed({{16{box_max_y[15]}}, box_max_y}) - $signed({{16{ray_orig_y[15]}}, ray_orig_y})) * $signed({{16{ray_inv_y[15]}}, ray_inv_y});
                stg1_tz1 <= ($signed({{16{box_min_z[15]}}, box_min_z}) - $signed({{16{ray_orig_z[15]}}, ray_orig_z})) * $signed({{16{ray_inv_z[15]}}, ray_inv_z});
                stg1_tz2 <= ($signed({{16{box_max_z[15]}}, box_max_z}) - $signed({{16{ray_orig_z[15]}}, ray_orig_z})) * $signed({{16{ray_inv_z[15]}}, ray_inv_z});
            end
        end
    end

    // 파이프라인 스테이지 2: Min/Max 자동 정렬 및 산술 시프트 레지스터
    reg signed [15:0] r_tmin_x, r_tmax_x, r_tmin_y, r_tmax_y, r_tmin_z, r_tmax_z;

    // 단순 비트 슬라이싱([23:8]) 방식은 음수일 경우 부호 비트 1이 잘려나가는 치명적인 하드웨어 결함이 있습니다.
    // 이를 산술 우측 시프트(>>>) 연산자로 처리하여, 최상위 부호를 안전하게 보존하면서 고정 소수점 소수부 스케일 다운을 완벽히 복원합니다.
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
            // 광선이 거꾸로 진입하는 음수 벡터나 코너 케이스에 대비한 크기 자동 정렬 회로입니다.
            // 멀티플렉서 스위치(삼항 연산자)를 달아서, 더 작은 거리는 무조건 tmin 방에, 큰 거리는 tmax 방에 제자리 정렬시켜 캡처합니다.
            r_tmin_x <= (scaled_tx1 > scaled_tx2) ? scaled_tx2 : scaled_tx1;
            r_tmax_x <= (scaled_tx1 > scaled_tx2) ? scaled_tx1 : scaled_tx2;
            r_tmin_y <= (scaled_ty1 > scaled_ty2) ? scaled_ty2 : scaled_ty1;
            r_tmax_y <= (scaled_ty1 > scaled_ty2) ? scaled_ty1 : scaled_ty2;
            r_tmin_z <= (scaled_tz1 > scaled_tz2) ? scaled_tz2 : scaled_tz1;
            r_tmax_z <= (scaled_tz1 > scaled_tz2) ? scaled_tz1 : scaled_tz2;
        end
    end

    // =========================================================================
    // [6] Reduction Tree & Protocol Compliant Output MUX: 최종 트리 및 출력 분기
    // =========================================================================
    // 앞단에서 정렬해온 출력값을 2단계 다단 토너먼트 비교기(Combinational Logic)로 꽉 짜냅니다.
    // 3D 큐브 진입점은 각 축의 최소 거리 결과 중 최댓값(Max of Mins)을 구해야 합니다. (X, Y 결승 후 Z와 최종 결승)
    wire signed [15:0] t_min_inter = (r_tmin_x > r_tmin_y) ? r_tmin_x : r_tmin_y;
    wire signed [15:0] t_min_final = (t_min_inter > r_tmin_z) ? t_min_inter : r_tmin_z;

    // 3D 큐브 이탈점은 각 축의 최대 거리 결과 중 최솟값(Min of Maxes)을 구해야 합니다.
    wire signed [15:0] t_max_inter = (r_tmax_x < r_tmax_y) ? r_tmax_x : r_tmax_y;  
    wire signed [15:0] t_max_final = (t_max_inter < r_tmax_z) ? t_max_inter : r_tmax_z;

    // 이탈 지점이 진입 지점보다 멀거나 같으면서 양수 대역(물체 앞)에 존재할 때, 상자 관통(Hit) 플래그 1을 띄웁니다.
    wire               pipeline_hit = (t_max_final >= t_min_final) && (t_max_final >= 16'sd0);
    wire signed [15:0] t_hit_final  = (t_min_final < 16'sd0) ? t_max_final : t_min_final; // 광선이 상자 내부에 있을 때의 기하학적 예외 스위칭 처리입니다.
    
    // 왼쪽 끝(31번)에는 Hit 플래그를, 가운데는 부호 비트로 채우고, 오른쪽 끝에는 거리를 담아 32비트 패킷으로 완성합니다.
    wire [31:0]        pipeline_computed_result = { pipeline_hit, {15{t_hit_final[15]}}, t_hit_final };

    // 가속기가 곱셈기를 굴려 힘들게 산출해 낸 최종 레이 트레이싱 연산 정답을 보관해둘 전용 레지스터 방입니다.
    reg [31:0] workload_result_reg;

    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            workload_result_reg <= 32'd0;
            cache_dir_x <= 16'd0; cache_dir_y <= 16'd0; cache_dir_z <= 16'd0;
            cache_hit_data <= 32'd0;
            cache_valid <= 1'b0;
        end else begin
            // [Smart Bypass 모드]: 궤적이 똑같아 캐시가 적중하면 연산 엔진 결과를 무시하고, 금고에 보관된 과거 정답을 결과 레지스터에 즉시 로드합니다.
            if (parsing_done && is_coherent) begin
                workload_result_reg <= cache_hit_data;
            end 
            // [Normal Compute 모드]: 오차가 커서 엔진이 직접 연산했다면, 그 따끈한 정답을 레지스터에 넣고 다음 비교를 위해 3축 방향 캐시도 갱신 백업합니다.
            else if (pipe_valid[1]) begin
                workload_result_reg <= pipeline_computed_result;
                
                cache_dir_x    <= ray_dir_x;
                cache_dir_y    <= ray_dir_y;
                cache_dir_z    <= ray_dir_z;
                cache_hit_data <= pipeline_computed_result;
                cache_valid    <= 1'b1; // 캐시에 유효 데이터가 담겼음을 도장 찍습니다.
            end
        end
    end

    // 최종 출력 멀티플렉서(MUX) 스위치단: 호스트 버스로 무슨 데이터를 내보낼지 물리적 선로를 결정합니다.
    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            IPOUT <= 32'd0;
        end else if (key_valid) begin
            if (output_select) begin
                // CPU가 output_select(CON[8])를 1로 세팅하면, 가속 결과 대신 교수님 필수 규격인 프로토콜 상태 워드를 조립해서 내보냅니다.
                // { 프로토콜 매직 번호 0x26, 작업 끝 깃발, 빈칸 00, 받은 샘플 개수, 16비트 보안 시그니처 } 포맷입니다.
                IPOUT <= { 8'h26, done, 2'b00, accepted_sample_count, stream_signature };
            end else begin
                // output_select가 0일 때는, 우리가 레이 트레이싱 가속기로 직접 연산해 낸 3D 교차 거리 최종 정답을 데이터 메모리로 쏴줍니다.
                IPOUT <= workload_result_reg;
            end
        end else begin
            // 매직 키가 틀리거나 가속기 스위치가 꺼지면, 시스템 버스에 쓰레기 신호가 떠다니지 않도록 출력을 0으로 밀어 청소(Flush)해 주는 매너 회로입니다.
            IPOUT <= 32'd0;
        end
    end

endmodule
