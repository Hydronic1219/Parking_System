`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/29/2025 09:04:52 AM
// Design Name: 
// Module Name: inside_module
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
module parking_spot_ctrl #(
    // ── 프레임/타임아웃 ────────────────────────────────────────
    parameter integer GAP_US          = 60000, // 프레임 간격(권장 ≥60ms, 반향 안정)
    parameter integer TIMEOUT_US      = 30000, // 상승/하강 대기 타임아웃(보수적 종료)
    // ── 판정(히스테리시스) ─────────────────────────────────────
    parameter integer TH_ON_CM        = 5,     // 5cm 미만이면 "점유 후보"
    parameter integer TH_OFF_CM       = 7,     // 7cm 초과면 "비점유 후보"
    // ── 프레임 디바운스 ───────────────────────────────────────
    parameter integer DB_ON_FRAMES    = 2,     // 연속 히트 N프레임 → 점유 확정
    parameter integer DB_OFF_FRAMES   = 2,     // 연속 미스 N프레임 → 비점유 확정
    // ── RGB 극성 ───────────────────────────────────────────────
    //  공통애노드(Active-Low)=1, 공통캐소드(Active-High)=0
    parameter integer ACTIVE_LOW_RGB  = 1
)(
    input  wire clk,        // 시스템 클럭(예: 100MHz)
    input  wire reset_p,    // 동기 리셋(High)
    input  wire echo,       // HC-SR04 ECHO (5V→3.3V 레벨변환 필수)
    output reg  trig,       // HC-SR04 TRIG

    // RGB 출력 (보드 극성은 파라미터로 처리)
    output reg  rgb_r,
    output reg  rgb_g,
    output reg  rgb_b,

    // 상태/펄스
    output reg  occ_level,    // 현재 점유상태(1=점유)
    output reg  park_pulse,   // 0→1 전이 시 1클럭 High
    output reg  leave_pulse,  // 1→0 전이 시 1클럭 High

    // 디버그 LED 버스
    output wire [15:0] led
);

    // ───────────────────────────────────────────────────────────
    // 1us 틱 생성기 (외부 모듈 clock_div_100 사용: pedge_div_100 = 1us 펄스)
    // ───────────────────────────────────────────────────────────
    wire tick_1us;
    wire pedge_div_100;
    clock_div_100 u_clkdiv100(
        .clk           (clk),
        .reset_p       (reset_p),
        .clk_div_100   (),              // 미사용
        .nedge_div_100 (),              // 미사용
        .pedge_div_100 (pedge_div_100)  // 1us 양에지 펄스
    );
    assign tick_1us = pedge_div_100;

    // ───────────────────────────────────────────────────────────
    // ECHO 동기화(2단) + 에지 검출 (메타 안정성 향상)
    // ───────────────────────────────────────────────────────────
    reg echo_ff1, echo_ff2;
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            echo_ff1 <= 1'b0;
            echo_ff2 <= 1'b0;
        end else begin
            echo_ff1 <= echo;
            echo_ff2 <= echo_ff1;
        end
    end
    wire echo_sync = echo_ff2;

    wire echo_pedge, echo_nedge;
    edge_detector_p u_echo_ed (
        .clk     (clk),
        .reset_p (reset_p),
        .cp      (echo_sync),
        .p_edge  (echo_pedge),
        .n_edge  (echo_nedge)
    );

    // ───────────────────────────────────────────────────────────
    // µs 카운터: tick_1us & count_en일 때만 +1 (단일 드라이버)
    // ───────────────────────────────────────────────────────────
    reg [21:0] count_usec;
    reg        count_en, count_clr;
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            count_usec <= 22'd0;
        end else if (count_clr) begin
            count_usec <= 22'd0;
        end else if (tick_1us && count_en) begin
            count_usec <= count_usec + 1'b1;
        end
    end

    // ───────────────────────────────────────────────────────────
    // FSM 상태 인코딩 
    // ───────────────────────────────────────────────────────────
    localparam [6:0]
      S_IDLE       = 7'b000_0001, // 프레임 간격 확보
      S_TRIG       = 7'b000_0010, // 10us TRIG High
      S_WAIT_HIGH  = 7'b000_0100, // ECHO 상승 엣지 대기
      S_MEASURE    = 7'b000_1000, // ECHO High 폭 측정
      S_LATCH      = 7'b001_0000, // 비교 준비(레지스터 단계)
      S_DECIDE     = 7'b010_0000, // 히스테리시스 + 디바운스
      S_DONE       = 7'b100_0000; // 최종 래치/펄스/복귀

    reg [6:0] state, next_state;
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) state <= S_IDLE;
        else         state <= next_state;
    end

    // ───────────────────────────────────────────────────────────
    // 측정/판정 보조 신호
    // ───────────────────────────────────────────────────────────
    reg [21:0] echo_time_us;   // 측정된 ECHO High 폭(µs)
    reg        frame_timeout;  // 해당 프레임에서 타임아웃 발생 여부
    reg        meas_under_on;  // echo_time_us < TH_ON_US ?
    reg        meas_over_off;  // echo_time_us > TH_OFF_US ?

    //  TRIG 가시화 카운터 (현장 LED용)
    reg [15:0] trig_vis_cnt;
    reg        trig_vis;

    // 임계시간(µs) : 합성 시 상수 폴딩됨(곱셈 없음)
    localparam integer TH_ON_US  = TH_ON_CM  * 58; // 5cm → 290us
    localparam integer TH_OFF_US = TH_OFF_CM * 58; // 7cm → 406us

    // 프레임 디바운스 카운터(포화)
    reg [7:0] on_cnt, off_cnt;
    reg       occ_next; // 디바운싱 후 다음 상태 후보

    // ───────────────────────────────────────────────────────────
    // 메인 FSM
    // ───────────────────────────────────────────────────────────
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            next_state    <= S_IDLE;

            trig          <= 1'b0;

            // 리셋 직후 비점유로 초기화 → RGB 조합 블록이 G를 켜게 됨
            occ_level     <= 1'b0;
            park_pulse    <= 1'b0;
            leave_pulse   <= 1'b0;

            count_en      <= 1'b0;
            count_clr     <= 1'b1;

            echo_time_us  <= 22'd0;
            frame_timeout <= 1'b0;
            meas_under_on <= 1'b0;
            meas_over_off <= 1'b0;

            trig_vis      <= 1'b0;
            trig_vis_cnt  <= 16'd0;

            on_cnt        <= 8'd0;
            off_cnt       <= 8'd0;
            occ_next      <= 1'b0;

        end else begin
            // 매 사이클 기본값(펄스류는 1클럭만 하이)
            count_clr   <= 1'b0;
            park_pulse  <= 1'b0;
            leave_pulse <= 1'b0;

            // TRIG 표시
            if (trig_vis_cnt != 0) begin
                trig_vis_cnt <= trig_vis_cnt - 1'b1;
                if (trig_vis_cnt == 16'd1) trig_vis <= 1'b0;
            end

            case (state)
                // ── S_IDLE: 프레임 간격 확보 ──────────────────
                S_IDLE: begin
                    count_en <= 1'b1;                   // µs 누적
                    if (count_usec < GAP_US) begin
                        next_state <= S_IDLE;
                    end else begin
                        count_en   <= 1'b0;
                        count_clr  <= 1'b1;             // 다음 구간 준비
                        next_state <= S_TRIG;
                    end
                end

                // ── S_TRIG: 10us TRIG High ───────────────────
                S_TRIG: begin
                    trig         <= 1'b1;
                    trig_vis     <= 1'b1;
                    trig_vis_cnt <= 16'd50000;          // 약 0.5ms 표시

                    count_en <= 1'b1;
                    if (count_usec < 22'd10) begin
                        next_state <= S_TRIG;
                    end else begin
                        trig          <= 1'b0;          // 10us 완료
                        count_en      <= 1'b0;
                        count_clr     <= 1'b1;
                        frame_timeout <= 1'b0;          // 프레임 시작 시 클리어
                        next_state    <= S_WAIT_HIGH;
                    end
                end

                // ── S_WAIT_HIGH: ECHO 상승 대기 ───────────────
                S_WAIT_HIGH: begin
                    count_en <= 1'b1;                   // 대기시간 측정
                    if (echo_pedge) begin               // 상승 엣지 감지
                        count_en   <= 1'b0;
                        count_clr  <= 1'b1;
                        next_state <= S_MEASURE;
                    end else if (count_usec >= TIMEOUT_US) begin
                        count_en      <= 1'b0;
                        count_clr     <= 1'b1;
                        frame_timeout <= 1'b1;          // 상승 못봄=미검출 취급
                        next_state    <= S_LATCH;
                    end
                end

                // ── S_MEASURE: ECHO High 폭 측정 ──────────────
                S_MEASURE: begin
                    count_en <= 1'b1;                   // 0부터 적산
                    if (echo_nedge) begin               // 하강 엣지 시 폭 래치
                        count_en     <= 1'b0;
                        echo_time_us <= count_usec;
                        next_state   <= S_LATCH;
                    end else if (count_usec >= TIMEOUT_US) begin
                        count_en      <= 1'b0;
                        count_clr     <= 1'b1;
                        frame_timeout <= 1'b1;          // 하강 못봄=미검출
                        next_state    <= S_LATCH;
                    end
                end

                // ── S_LATCH: 비교 준비(레지스터 단계) ──────────
                S_LATCH: begin
                    if (frame_timeout) begin
                        meas_under_on <= 1'b0;
                        meas_over_off <= 1'b1;
                    end else begin
                        // 히스테리시스 비교(임계는 상수 폴딩됨)
                        meas_under_on <= (echo_time_us < TH_ON_US);
                        meas_over_off <= (echo_time_us > TH_OFF_US);
                    end
                    next_state <= S_DECIDE;
                end

                // ── S_DECIDE: 히스테리시스 + 프레임 디바운스 ─
                S_DECIDE: begin
                    // 히트 프레임: on_cnt 누적, 일정 횟수 도달 시 점유
                    if (meas_under_on) begin
                        on_cnt  <= (on_cnt  == 8'hFF) ? on_cnt  : (on_cnt  + 8'd1);
                        off_cnt <= 8'd0;
                        if (on_cnt >= DB_ON_FRAMES-1) occ_next <= 1'b1;
                    end
                    // 미스 프레임: off_cnt 누적, 일정 횟수 도달 시 비점유
                    else if (meas_over_off) begin
                        off_cnt <= (off_cnt == 8'hFF) ? off_cnt : (off_cnt + 8'd1);
                        on_cnt  <= 8'd0;
                        if (off_cnt >= DB_OFF_FRAMES-1) occ_next <= 1'b0;
                    end
                    // 그 외(ON/OFF 사이 구간): 상태 유지
                    next_state <= S_DONE;
                end

                // ── S_DONE: 최종 래치/펄스/복귀 ────────────────
                S_DONE: begin
                    // 전이 검출 → 펄스 1클럭
                    if (occ_level == 1'b0 && occ_next == 1'b1) begin
                        park_pulse <= 1'b1;  // 주차 발생(0→1)
                    end else if (occ_level == 1'b1 && occ_next == 1'b0) begin
                        leave_pulse <= 1'b1; // 출차 발생(1→0)
                    end
                    // 상태 반영
                    occ_level <= occ_next;

                    // 다음 프레임 준비
                    count_clr  <= 1'b1;
                    next_state <= S_IDLE;
                end

                default: next_state <= S_IDLE;
            endcase
        end
    end

    // ───────────────────────────────────────────────────────────
    // RGB 항상 매핑(항상조합): 상태만 보고 즉시 표시 결정
    //   - occ_level=1(점유)   → R=ON, G=OFF
    //   - occ_level=0(비점유) → G=ON, R=OFF
    //   - B는 사용하지 않음(OFF)
    //   - ACTIVE_LOW_RGB=1 이면 출력 반전(0=켜짐), 0이면 그대로(1=켜짐)
    // ───────────────────────────────────────────────────────────
    reg r_on, g_on, b_on;  // 조합용 내부 플래그
    always @* begin
        // 논리 ON/OFF (극성과 무관)
        r_on = (occ_level == 1'b0);
        g_on = (occ_level == 1'b1);
        b_on = 1'b0;

        // 극성 적용
        if (ACTIVE_LOW_RGB != 0) begin
            rgb_r = ~r_on;
            rgb_g = ~g_on;
            rgb_b = ~b_on;
        end else begin
            rgb_r =  r_on;
            rgb_g =  g_on;
            rgb_b =  b_on;
        end
    end

    // ───────────────────────────────────────────────────────────
    // 디버그 LED 맵
    //  [15]=ECHO 현재값, [14]=TRIG 표시, [13:8]=state, [7]=occ_level,
    //  [6]=park_pulse, [5]=leave_pulse, [4:0]=echo_time_us[4:0]
    // ───────────────────────────────────────────────────────────
    assign led = { echo_sync, trig_vis, state[5:0],
                   occ_level, park_pulse, leave_pulse, echo_time_us[4:0] };

endmodule


//3) 솔루션 (전체 모듈 교체본)
module hc_sr04_cntr #(
    parameter integer GAP_US         = 60000, // 트리거 간 최소 간격
    parameter integer TIMEOUT_US     = 30000, // 에코 대기/측정 타임아웃
    // ── NEW: 임계 및 디바운스 옵션 ─────────────────────────────
    parameter integer THRESH_CM      = 5,     // 5cm 미만이면 1
    parameter integer DB_ON_COUNT    = 2,     // 1로 전이 시 연속 히트 필요 횟수
    parameter integer DB_OFF_COUNT   = 2      // 0으로 전이 시 연속 미스 필요 횟수
)(
    input  wire        clk, reset_p,
    input  wire        echo,          // (주의) 5V → 3.3V 레벨변환 후 입력
    output reg         trig,
    output reg [15:0]  distance,      // LSB=occ(1/0), 상위비트는 0
    output wire [15:0] led            // 디버그 표시
);
    // 상태 인코딩 (기존 유지)
    localparam [6:0]
      S_IDLE       = 7'b000_0001,
      S_TRIG       = 7'b000_0010,
      S_WAIT_HIGH  = 7'b000_0100,
      S_MEASURE    = 7'b000_1000,
      S_LATCH      = 7'b001_0000, // echo_time_us 래치
      S_DIVIDE     = 7'b010_0000, // (의미변경) 디바운스/결정 준비 단계
      S_DONE       = 7'b100_0000;

    // 1us tick (posedge only)
    wire clk_usec_pedge;
    clock_div_100 u_clkdiv100(
        .clk           (clk),
        .reset_p       (reset_p),
        .clk_div_100   (),
        .nedge_div_100 (),
        .pedge_div_100 (clk_usec_pedge)
    );
    wire tick_1us = clk_usec_pedge;

    // ECHO 동기화/에지 검출
    reg echo_ff1, echo_ff2;
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            echo_ff1 <= 1'b0;
            echo_ff2 <= 1'b0;
        end else begin
            echo_ff1 <= echo;
            echo_ff2 <= echo_ff1;
        end
    end
    wire echo_sync = echo_ff2;

    wire echo_pedge, echo_nedge;
    edge_detector_p u_echo_ed (
        .clk     (clk),
        .reset_p (reset_p),
        .cp      (echo_sync),
        .p_edge  (echo_pedge),
        .n_edge  (echo_nedge)
    );

    // µs 카운터
    reg [21:0] count_usec;
    reg        count_en;
    reg        count_clr;
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            count_usec <= 22'd0;
        end else if (count_clr) begin
            count_usec <= 22'd0;
        end else if (tick_1us && count_en) begin
            count_usec <= count_usec + 1'b1;
        end
    end

    // 상태/레지스터
    reg [6:0] state, next_state;
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) state <= S_IDLE;
        else         state <= next_state;
    end

    // 계측값 저장(µs)
    reg [21:0] echo_time_us;

    // TRIG 가시화
    reg [15:0] trig_vis_cnt;
    reg        trig_vis;

    // ── NEW: 이진 점유 판정 ────────────────────────────────────
    localparam integer THRESH_US = THRESH_CM * 58; // 합성시 상수로 폴딩됨
    reg occ_state;                 // 최종 출력(LSB)
    reg [7:0] on_cnt, off_cnt;     // 디바운스 카운터
    reg       meas_hit;            // 현재 프레임 임계 통과 여부(조합/레지스터)

    // 메인 FSM
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            next_state   <= S_IDLE;
            trig         <= 1'b0;
            distance     <= 16'd0;

            count_en     <= 1'b0;
            count_clr    <= 1'b1;

            echo_time_us <= 22'd0;

            trig_vis     <= 1'b0;
            trig_vis_cnt <= 16'd0;

            occ_state    <= 1'b0;
            on_cnt       <= 8'd0;
            off_cnt      <= 8'd0;
            meas_hit     <= 1'b0;

        end else begin
            // 기본값
            count_clr <= 1'b0;

            // TRIG 표시 유지
            if (trig_vis_cnt != 0) begin
                trig_vis_cnt <= trig_vis_cnt - 1'b1;
                if (trig_vis_cnt == 16'd1) trig_vis <= 1'b0;
            end

            case (state)
                // 1) 측정 간격 확보
                S_IDLE: begin
                    count_en <= 1'b1;
                    if (count_usec < GAP_US) begin
                        next_state <= S_IDLE;
                    end else begin
                        count_en  <= 1'b0;
                        count_clr <= 1'b1;
                        next_state <= S_TRIG;
                    end
                end

                // 2) TRIG 10us High
                S_TRIG: begin
                    trig         <= 1'b1;
                    trig_vis     <= 1'b1;
                    trig_vis_cnt <= 16'd50000; // 표시용

                    count_en <= 1'b1;
                    if (count_usec < 22'd10) begin
                        next_state <= S_TRIG;
                    end else begin
                        trig      <= 1'b0;
                        count_en  <= 1'b0;
                        count_clr <= 1'b1;
                        next_state <= S_WAIT_HIGH;
                    end
                end

                // 3) ECHO 상승 대기 (타임아웃 보호)
                S_WAIT_HIGH: begin
                    count_en <= 1'b1;
                    if (echo_pedge) begin
                        count_en  <= 1'b0;
                        count_clr <= 1'b1;
                        next_state <= S_MEASURE;
                    end else if (count_usec >= TIMEOUT_US) begin
                        count_en  <= 1'b0;
                        count_clr <= 1'b1;
                        // 측정 실패 프레임: 미스 처리
                        meas_hit  <= 1'b0;
                        next_state <= S_LATCH;
                    end
                end

                // 4) ECHO High 폭 측정
                S_MEASURE: begin
                    count_en <= 1'b1;
                    if (echo_nedge) begin
                        count_en     <= 1'b0;
                        echo_time_us <= count_usec; // µs 폭
                        next_state   <= S_LATCH;
                    end else if (count_usec >= TIMEOUT_US) begin
                        count_en  <= 1'b0;
                        count_clr <= 1'b1;
                        // 타임아웃 → 히트 아님
                        meas_hit  <= 1'b0;
                        next_state <= S_LATCH;
                    end
                end

                // 4.5) 래치 후 임계 비교(조합 단순화 지점)
                S_LATCH: begin
                    // echo_time_us와 임계시간 비교
                    // (주의) S_MEASURE에서 타임아웃으로 온 경우 echo_time_us는 이전값일 수 있어
                    // 그 경우 meas_hit를 0으로 이미 설정함.
                    if (count_en == 1'b0 && (state==S_LATCH) && (echo_time_us != 22'd0)) begin
                        meas_hit <= (echo_time_us < THRESH_US);
                    end
                    next_state <= S_DIVIDE; // 의미상 '결정 준비' 단계
                end

                // 5) 디바운스/상태 결정 준비
                S_DIVIDE: begin
                    // 디바운스: 히트면 on_cnt++, 미스면 off_cnt++
                    if (meas_hit) begin
                        on_cnt  <= (on_cnt  == 8'hFF) ? on_cnt  : on_cnt  + 1'b1;
                        off_cnt <= 8'd0;
                        if (on_cnt >= DB_ON_COUNT-1) begin
                            occ_state <= 1'b1;
                        end
                    end else begin
                        off_cnt <= (off_cnt == 8'hFF) ? off_cnt : off_cnt + 1'b1;
                        on_cnt  <= 8'd0;
                        if (off_cnt >= DB_OFF_COUNT-1) begin
                            occ_state <= 1'b0;
                        end
                    end
                    next_state <= S_DONE;
                end

                // 6) 결과 래치(LSB=점유)
                S_DONE: begin
                    distance  <= {15'd0, occ_state}; // 상위 15비트 0, LSB=occ
                    count_clr <= 1'b1;               // 다음 주기 준비
                    next_state<= S_IDLE;
                end

                default: next_state <= S_IDLE;
            endcase
        end
    end

    // LED 디버그: [15]=ECHO, [14]=TRIG 표시, [13:8]=state 상위6비트, [7]=occ_state, [6:0]=echo_time 하위7비트
    assign led = { echo_sync, trig_vis, state[5:0], occ_state, echo_time_us[6:0] };

endmodule

module parking_top_1spot #(
    // 초음파/히스테리시스 파라미터 - 현장에 맞게 조정 가능
    parameter integer GAP_US        = 60000, // 프레임 간격(≥60ms 권장)
    parameter integer TIMEOUT_US    = 30000, // ECHO 타임아웃
    parameter integer TH_ON_CM      = 5,     // 점유 판정 ON 임계(미만)
    parameter integer TH_OFF_CM     = 7,     // 점유 판정 OFF 임계(초과)
    parameter integer DB_ON_FRAMES  = 2,     // 0->1 전환에 필요한 연속 히트 프레임 수
    parameter integer DB_OFF_FRAMES = 2      // 1->0 전환에 필요한 연속 미스 프레임 수
)(
    input  wire clk,              // 시스템 클럭(예: 100MHz)
    input  wire reset_p,          // 동기 리셋(High)
    // 초음파 센서 #0
    input  wire echo0, echo1, echo2, echo3,            // ECHO(레벨변환 후)
    output wire trig0,           // TRIG
    
    // RGB(공통캐소드 가정, Active-High) : 점유=빨강, 비점유=초록
    output wire rgb0_r, rgb1_r, rgb2_r, rgb3_r,
    output wire rgb0_g, rgb1_g, rgb2_g, rgb3_g,
    // LCD 팀 연동: 가용면수 카운트용 펄스
//    output wire [3:0]lcd_dec_pulse,    // 주차 발생(0->1 전이) 시 1클럭 펄스
//    output wire lcd_inc_pulse,    // 출차 발생(1->0 전이) 시 1클럭 펄스
    // (옵션) 상태 모니터링용
    output wire [3:0] occ0_level,       // 현재 점유 상태(1=점유)
    output wire [15:13] led0,       // 디버그
    output wire [12:10] led1,
    output wire [9:7] led2,
    output wire [6:0] led3
);
    wire [3:0]lcd_dec_pulse;
    wire [3:0]lcd_inc_pulse;
    // 내부 RGB_B는 사용하지 않음(0 고정)
    wire rgb0_b_unused;

    // 컨트롤러 인스턴스 (1면)
    parking_spot_ctrl #(
        .GAP_US        (GAP_US),
        .TIMEOUT_US    (TIMEOUT_US),
        .TH_ON_CM      (TH_ON_CM),
        .TH_OFF_CM     (TH_OFF_CM),
        .DB_ON_FRAMES  (DB_ON_FRAMES),
        .DB_OFF_FRAMES (DB_OFF_FRAMES)
    ) U_SPOT0 (
        .clk          (clk),
        .reset_p      (reset_p),
        .echo         (echo0),
        .trig         (trig0),
        .rgb_r        (rgb0_r),
        .rgb_g        (rgb0_g),
        .occ_level    (occ0_level[0]),
        .park_pulse   (lcd_dec_pulse[0]),  // 주차 → 가용면수 -1 (LCD 팀에 '1' 보내기)
        .leave_pulse  (lcd_inc_pulse[0]),  // 출차 → 가용면수 +1
        .led          (led0)
    );
    
        parking_spot_ctrl #(
        .GAP_US        (GAP_US),
        .TIMEOUT_US    (TIMEOUT_US),
        .TH_ON_CM      (TH_ON_CM),
        .TH_OFF_CM     (TH_OFF_CM),
        .DB_ON_FRAMES  (DB_ON_FRAMES),
        .DB_OFF_FRAMES (DB_OFF_FRAMES)
    ) U_SPOT1 (
        .clk          (clk),
        .reset_p      (reset_p),
        .echo         (echo1),
        .trig         (trig1),
        .rgb_r        (rgb1_r),
        .rgb_g        (rgb1_g),
        .occ_level    (occ0_level[1]),
        .park_pulse   (lcd_dec_pulse[1]),  // 주차 → 가용면수 -1 (LCD 팀에 '1' 보내기)
        .leave_pulse  (lcd_inc_pulse[1]),  // 출차 → 가용면수 +1
        .led          (led1)
    );
    
           parking_spot_ctrl #(
        .GAP_US        (GAP_US),
        .TIMEOUT_US    (TIMEOUT_US),
        .TH_ON_CM      (TH_ON_CM),
        .TH_OFF_CM     (TH_OFF_CM),
        .DB_ON_FRAMES  (DB_ON_FRAMES),
        .DB_OFF_FRAMES (DB_OFF_FRAMES)
    ) U_SPOT2 (
        .clk          (clk),
        .reset_p      (reset_p),
        .echo         (echo2),
        .trig         (trig2),
        .rgb_r        (rgb2_r),
        .rgb_g        (rgb2_g),
        .occ_level    (occ0_level[2]),
        .park_pulse   (lcd_dec_pulse[2]),  // 주차 → 가용면수 -1 (LCD 팀에 '1' 보내기)
        .leave_pulse  (lcd_inc_pulse[2]),  // 출차 → 가용면수 +1
        .led          (led2)
    );
    
          parking_spot_ctrl #(
        .GAP_US        (GAP_US),
        .TIMEOUT_US    (TIMEOUT_US),
        .TH_ON_CM      (TH_ON_CM),
        .TH_OFF_CM     (TH_OFF_CM),
        .DB_ON_FRAMES  (DB_ON_FRAMES),
        .DB_OFF_FRAMES (DB_OFF_FRAMES)
    ) U_SPOT3 (
        .clk          (clk),
        .reset_p      (reset_p),
        .echo         (echo3),
        .trig         (trig3),
        .rgb_r        (rgb3_r),
        .rgb_g        (rgb3_g),
        .occ_level    (occ0_level[3]),
        .park_pulse   (lcd_dec_pulse[3]),  // 주차 → 가용면수 -1 (LCD 팀에 '1' 보내기)
        .leave_pulse  (lcd_inc_pulse[3]),  // 출차 → 가용면수 +1
        .led          (led3)
    );
    // 파랑선 미사용: 보드가 당김저항 등 외부회로가 없으면 확실히 0으로 고정
    // (합성기 최적화 위해 직접 0 할당 X, 위에서 unused 와이어로 흡수됨)

endmodule
