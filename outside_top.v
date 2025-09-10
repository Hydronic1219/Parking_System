`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/26/2025 03:25:21 PM
// Design Name: 
// Module Name: outside_top
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



module keypad_top (
    input clk, reset_p,
    input [3:0] row,
    output [3:0] column,
    output [7:0] seg_7,
    output [3:0] com,
    output [15:0] led
);
    
    wire key_valid;
    wire [3:0] key_value;
    assign led[0] = key_valid;

    keypad_cntr key_pad(
    clk, reset_p,
    row,
    column,
    key_value,
    key_valid
    );


    fnd_cntr fnd(
        .clk(clk),
        .reset_p(reset_p),
        .fnd_value(key_value),
        .hex_bcd(1),
        .seg_7(seg_7),
        .com(com)
    );

endmodule

module servo_motor_pwm_top (
    input clk, reset_p,
    input  motor_open,
    output motor,
    output [15:0] led
    );
    
    integer step;
    reg inc_flag;
    wire count_pedge;
    integer cnt;

    reg     busy;         
    reg     hold;         
    integer hold_cnt;

    localparam integer HOLD_TICKS = 120;
    
    always @(posedge clk, posedge reset_p) begin
    if (reset_p) cnt = 0;
    else         cnt = cnt + 1;
    end
    
    wire motor_open_pedge;
    edge_detector_p open_ed(
        .clk(clk), .reset_p(reset_p), .cp(motor_open), .p_edge(motor_open_pedge)
    );

    edge_detector_p cnt_ed(
        .clk(clk), .reset_p(reset_p), .cp(cnt[21]), .p_edge(count_pedge)
    );
   
   always @(posedge clk, posedge reset_p) begin
        if (reset_p) begin
            // 초기 상태
            step     = 8;      // 기존 값 유지(필요하면 27로 바꿔도 됨)
            inc_flag = 1;      // 열림 방향으로 준비
            busy     = 0;
            hold     = 0;
            hold_cnt = 0;
        end
        else begin
            if (motor_open_pedge && !busy) begin
                busy     = 1;
                inc_flag = 1;  
                hold_cnt = 0;
            end
            if (count_pedge && busy) begin
                if (hold) begin
                    if (hold_cnt >= HOLD_TICKS) begin
                        hold     = 0;
                        hold_cnt = 0;
                    end else begin
                        hold_cnt = hold_cnt + 1;
                    end
                end
                else begin
                    if (inc_flag) begin
                        if (step >= 110) begin
                            hold     = 1;
                            hold_cnt = 0;
                            inc_flag = 0;
                        end else begin
                            step = step + 1;
                        end
                    end else begin
                        if (step <= 27) begin
                            busy     = 0;
                            hold     = 0;
                            hold_cnt = 0;
                        end else begin
                            step = step - 1;
                        end
                    end
                end
            end
        end
    end


    pwm_Nstep #(.pwm_freq(50), .duty_step_N(1440)) servomotor(
        .clk(clk), .reset_p(reset_p),
        .duty(step),
        .pwm(motor)
    );

endmodule



module parking_system(
    input        clk, reset_p,
    input  [3:0] row,
    output [3:0] column,
    output [3:0] key_value,
   output reg [15:0] lcd_number,     // LCD로 보낼 4자리(각 4비트)
   output reg [15:0] parking_time,   // LCD로 보낼 경과시간(틱 단위)
    output reg         data_valid,     
    output reg         not_found,       
    output motor,
    output key_valid,
    output buz_sig,
    output [15:0] led
);
    parameter integer CLK_FREQ_HZ = 100_000_000;

    reg [31:0] sec_div;
    wire       sec_tick; 

    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            sec_div <= 32'd0;
        end else if (sec_div == CLK_FREQ_HZ-1) begin
            sec_div <= 32'd0;
        end else begin
            sec_div <= sec_div + 32'd1;
        end
    end

    assign sec_tick = (sec_div == CLK_FREQ_HZ-1);

    reg [5:0]  sec_cnt;        
    reg        min_tick;
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            sec_cnt  <= 6'd0;
            min_tick <= 1'b0;
        end else if (sec_tick) begin
            if (sec_cnt == 6'd59) begin
                sec_cnt  <= 6'd0;
                min_tick <= 1'b1;     
            end else begin
                sec_cnt  <= sec_cnt + 6'd1;
                min_tick <= 1'b0;
            end
        end else begin
            min_tick <= 1'b0;
        end
    end

    reg         buzzer_en;
    reg         servo_open;
    // reg         data_valid;   
    // reg         not_found;
    // reg [15:0] lcd_number;    
    // reg [15:0] parking_time;  
//    wire [3:0] key_value;
    // wire       key_valid;

    keypad_cntr u_keypad (
        .clk(clk),
        .reset_p(reset_p),
        .row(row),
        .column(column),
        .key_value(key_value),
        .key_valid(key_valid)
    );

    reg key_valid_q;
    wire key_valid_rise = key_valid & ~key_valid_q;
    always @(posedge clk or posedge reset_p) begin
    if (reset_p) key_valid_q <= 1'b0;
    else         key_valid_q <= key_valid;
    end
    
    reg         wr_en, rd_en;
    reg [9:0]   wr_addr, rd_addr;
    reg [31:0]  i_data;
    wire [31:0] o_data;

    memory u_mem (
        .clk(clk),
        .rd_en(rd_en),
        .wr_en(wr_en),
        .wr_addr(wr_addr),
        .rd_addr(rd_addr),
        .i_data(i_data),
        .o_data(o_data)
    );

    servo_motor_pwm_top servo_motor(
    .clk(clk), .reset_p(reset_p),
    .motor_open(servo_open),
    .motor(motor)
    );
    
    buzzer buzz(
    .clk(clk), .reset_p(reset_p),
    .buzzer_en(buzzer_en),
    .buz_sig(buz_sig)
    );


    reg [15:0] time_counter16;
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) time_counter16 <= 16'd0;
        else if (min_tick) time_counter16 <= time_counter16 + 16'd1; 
    end


    reg [15:0] plate_buf;   
    
    reg [1:0]  digit_cnt;   
    reg [9:0]  wr_index;    
    reg [10:0]  rec_count;   
    reg [9:0]  match_idx;     // ★ 매치된 레코드 주소 저장
    reg        clear_done;
    reg [9:0]  srch_idx;
    reg        hit;
    reg [15:0] rd_time_tmp;

    localparam [3:0] KEY_A = 4'd10;
    localparam [3:0] KEY_B = 4'd11;
    localparam [3:0] KEY_C = 4'd12;

    localparam [15:0] SERVO_PULSE_TICKS = 16'd2000; 
    reg [15:0] servo_cnt;
    reg        servo_busy;

    localparam S_WAIT_MODE     = 13'b0_0000_0000_0001;  
    localparam S_ENTER_IN      = 13'b0_0000_0000_0010;  
    localparam S_WRITE_SETUP   = 13'b0_0000_0000_0100;  
    localparam S_WRITE_COMMIT  = 13'b0_0000_0000_1000;  
    localparam S_ENTER_OUT     = 13'b0_0000_0001_0000;    
    localparam S_SR_INIT       = 13'b0_0000_0010_0000;  
    localparam S_RD_SETUP      = 13'b0_0000_0100_0000;  
    localparam S_RD_LATCH      = 13'b0_0000_1000_0000;  
    localparam S_CALC          = 13'b0_0001_0000_0000;  
    localparam S_DISPLAY       = 13'b0_0010_0000_0000;  
    localparam S_WAIT_PAY      = 13'b0_0100_0000_0000;  
    localparam S_SERVO_PULSE   = 13'b0_1000_0000_0000;  
    localparam S_RD_WAIT       = 13'b1_0000_0000_0000; 
    reg [12:0] state, next_state;
    
    assign led[0]  = key_valid;                         
    assign led[1]  = key_valid_rise;                   
    assign led[2]  = key_valid_rise && (key_value==4'hA); 
    assign led[15:3]  = state;
//    assign led = plate_buf;
    // assign led = lcd_number;
    
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) state <= S_WAIT_MODE;
        else         state <= next_state;
    end

    wire is_digit = (key_value <= 4'd9); 

    always @(*) begin
        next_state = state;
        case (state)
            S_WAIT_MODE: begin
                if (key_valid_rise && key_value == KEY_A) next_state = S_ENTER_IN;
                else if (key_valid_rise && key_value == KEY_B) next_state = S_ENTER_OUT;
            end
            S_ENTER_IN: begin
                if (digit_cnt == 2'd3 && key_valid_rise && is_digit) next_state = S_WRITE_SETUP;
            end
            S_WRITE_SETUP  : next_state = S_WRITE_COMMIT;
            S_WRITE_COMMIT : next_state = S_WAIT_MODE;
            S_ENTER_OUT: begin
                if (digit_cnt == 2'd3 && key_valid_rise && is_digit) next_state = S_SR_INIT;
            end
            S_SR_INIT  : next_state = (rec_count == 0) ? S_DISPLAY : S_RD_SETUP;
            S_RD_SETUP : next_state = S_RD_WAIT;
            S_RD_WAIT  : next_state = S_RD_LATCH;
            S_RD_LATCH : next_state = hit ? S_CALC
                                          : ((srch_idx + 10'd1 < rec_count) ? S_RD_SETUP : S_DISPLAY);
            S_CALC     : next_state = S_DISPLAY;
            S_DISPLAY  : begin 
                if (hit) next_state = S_WAIT_PAY;   
                else next_state = S_ENTER_OUT;
                end

            S_WAIT_PAY: begin
                if (key_valid_rise && key_value == KEY_C) next_state = S_SERVO_PULSE;

            end
            S_SERVO_PULSE: next_state = S_WAIT_MODE;
        endcase
    end


    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            wr_en        <= 1'b0;
            rd_en        <= 1'b0;
            wr_addr      <= 10'd0;
            rd_addr      <= 10'd0;
            i_data       <= 32'd0;

            plate_buf    <= 16'd0;
            digit_cnt    <= 2'd0;

            wr_index     <= 10'd0;
            rec_count    <= 10'd0;

            srch_idx     <= 10'd0;
            hit          <= 1'b0;
            rd_time_tmp  <= 16'd0;
            
            match_idx   <= 10'd0;     
            clear_done  <= 1'b0;
            
            lcd_number   <= 16'd0;
            parking_time <= 16'd0;
            data_valid   <= 1'b0;
            not_found    <= 1'b0;
            buzzer_en    <= 1'b0;
            servo_open   <= 1'b0;
            servo_cnt    <= 16'd0;
            servo_busy   <= 1'b0;
        end else begin
            wr_en      <= 1'b0;
            rd_en      <= 1'b0;
            data_valid <= 1'b0;
            servo_open <= 1'b0;
            buzzer_en  <= 1'b0;
            case (state)
                S_WAIT_MODE: begin
                    digit_cnt  <= 2'd0;
                    not_found  <= 1'b0;
                    servo_open <= 1'b0;
                    buzzer_en    <= 1'b0;
                end
                S_ENTER_IN: begin
                    if (key_valid_rise && is_digit) begin
                        plate_buf <= {plate_buf[11:0], key_value};
                        digit_cnt <= digit_cnt + 2'd1;
                    end
                end
                S_WRITE_SETUP: begin
                    wr_addr <= wr_index;
                    i_data  <= {plate_buf, time_counter16};
                end
                S_WRITE_COMMIT: begin
                    servo_open <= 1'b1;
                    buzzer_en  <= 1'b1;
                    wr_en <= 1'b1; 
                    if (wr_index != 10'd1023) wr_index <= wr_index + 10'd1;
                    if (rec_count != 11'd1024) rec_count <= rec_count + 10'd1;
                    lcd_number <= plate_buf;
                    digit_cnt  <= 2'd0;
                end
                S_ENTER_OUT: begin
                    if (key_valid_rise && is_digit) begin
                        plate_buf <= {plate_buf[11:0], key_value};
                        digit_cnt <= digit_cnt + 2'd1;
                    end
                end
                S_SR_INIT: begin
                    srch_idx   <= 10'd0;
                    hit        <= 1'b0;
                    not_found  <= (rec_count == 0);
                end
                S_RD_SETUP: begin
                    rd_addr <= srch_idx;
                    rd_en   <= 1'b1;
                    hit     <= 1'b0;
                end
                S_RD_WAIT: begin
                    
                end
                S_RD_LATCH: begin
                    if (o_data[31:16] == plate_buf) begin
                        hit         <= 1'b1;
                        rd_time_tmp <= o_data[15:0];
                        not_found   <= 1'b0;
                        match_idx   <= srch_idx;
                    end else begin
                        hit       <= 1'b0;
                        srch_idx  <= srch_idx + 10'd1;
                    end
                end
                S_CALC: begin
                    parking_time <= time_counter16 - rd_time_tmp;
                end
                S_DISPLAY: begin
                    lcd_number <= plate_buf; 
                    data_valid <= 1'b1;  
                    if (~hit) begin
                        not_found  <= 1'b1;       
                        digit_cnt  <= 2'd0;        
                        plate_buf  <= 16'd0;       
                    end else begin
                        not_found  <= 1'b0;     
                    end
                end
                S_WAIT_PAY: begin
                    clear_done <= 1'b0;
                end
                S_SERVO_PULSE: begin
                        wr_addr    <= match_idx;
                        i_data     <= 32'd0;
                        wr_en      <= 1'b1;          
                        servo_open <= 1'b1;
                        buzzer_en    <= 1'b1;
                end

            endcase
        end
    end
endmodule




module IP_cha (
    input clk, reset_p,
    input echo,
    input  [3:0] row,
    output [3:0] column,
    input  echo0, echo1, echo2, echo3, 
   output rgb0_r, rgb1_r, rgb2_r, rgb3_r,
   output rgb0_g, rgb1_g, rgb2_g, rgb3_g,
    output reg mode_req_out,   
    output scl, sda,
    output buz_sig,
    output motor,
    output trig,
    output [15:0] parking_time_o,  
    output [15:0] lcd_number_o,    
    output        data_valid_o,    
    output        not_found_o,  
    output        key_valid_o,   
    output [3:0]  key_value_o,
    output [15:0] led
);
    
    integer cnt_sysclk;
    reg count_clk_e;
    always @(negedge clk, posedge reset_p) begin
        if (reset_p) begin
            cnt_sysclk = 0;
        end
        else if (count_clk_e) begin
            cnt_sysclk = cnt_sysclk + 1;
        end
        else cnt_sysclk = 0;
    end

    reg [7:0] send_buffer;
    reg send, rs;
    wire busy;
    
    wire [3:0] key_value;
    wire [15:0] lcd_number;
    wire is_digit = (key_value <= 4'd9); 
    wire        key_valid;
    wire        data_valid;
    wire        not_found;
    wire [15:0] parking_time;
    
    parking_system park(
    .clk(clk), .reset_p(reset_p),
    .row(row),
    .column(column),
    .key_value(key_value),
    .lcd_number(lcd_number), 
    .key_valid(key_valid),
    .data_valid(data_valid),
    .parking_time(parking_time),
    .not_found(not_found),
    .motor(motor),
    .buz_sig(buz_sig)
    );

    I2C_lcd_send_byte lcd_byte(
    clk, reset_p,
    7'h27,
    send_buffer,
    send, rs,
    scl, sda,
    busy,
    led
    );
    // wire rgb0_r, rgb1_r, rgb2_r, rgb3_r;
    // wire rgb0_g, rgb1_g, rgb2_g, rgb3_g;
    parking_top_1spot inside_sonic(
    .clk(clk),
    .reset_p(reset_p),
    .echo0(echo0), 
    .echo1(echo1), 
    .echo2(echo2), 
    .echo3(echo3), 
    .trig0(trig0),
    .rgb0_r(rgb0_r), 
    .rgb1_r(rgb1_r), 
    .rgb2_r(rgb2_r), 
    .rgb3_r(rgb3_r),
    .rgb0_g(rgb0_g), 
    .rgb1_g(rgb1_g), 
    .rgb2_g(rgb2_g), 
    .rgb3_g(rgb3_g)
    );

 

   
    wire key_valid_pedge;
    edge_detector_p key_valid_ed(
        .clk(clk), .reset_p(reset_p), .cp(key_valid), .p_edge(key_valid_pedge)
    );

    always @(posedge clk or posedge reset_p) begin
        if (reset_p) mode_req_out <= 1'b0;
        else begin
            mode_req_out <= 1'b0; // 기본 0
            if (key_valid_pedge && key_value==4'd10) mode_req_out <= 1'b1; // 출차 전환
        end
    end
    localparam IDLE                 = 8'b0000_0001;
    localparam INIT                 = 8'b0000_0010;
    localparam SEND_WELCOME         = 8'b0000_0100;
    localparam ENTER_CAR_NUM        = 8'b0000_1000;
    localparam SEND_CAR_NUM         = 8'b0001_0000;
    localparam EMPTY_NUM            = 8'b0010_0000;
    localparam WAIT_WELCOME         = 8'b0100_0000;
    localparam SET_CURSOR_LINE2     = 8'b1000_0000;

    reg [7:0] state, next_state;
    assign led[7:0] = state;
    assign led[8] = busy;
    always @(negedge clk, posedge reset_p) begin
        if(reset_p) begin
            state = IDLE;
        end
        else begin
            state = next_state;
        end
    end
    assign parking_time_o = parking_time;
    assign lcd_number_o   = lcd_number;
    assign data_valid_o   = data_valid;
    assign not_found_o    = not_found;
    assign key_valid_o    = key_valid;
    assign key_value_o    = key_value;
    wire  rgb_r;
    wire rgb_r_pedge;
    edge_detector_p rgb_r_ed(
        .clk(clk), .reset_p(reset_p), .cp(rgb_r), .p_edge(rgb_r_pedge)
    );

    parking_spot_ctrl ultrasonic(
    .clk(clk),
    .reset_p(reset_p),
    .echo(echo),
    .trig(trig),
    .rgb_r(rgb_r)
    );

    wire [1:0] floor1_free = {1'b0, rgb0_g} + {1'b0, rgb1_g}; 
    wire [1:0] floor2_free = {1'b0, rgb2_g} + {1'b0, rgb3_g}; 
    wire [7:0] floor1_ascii = "0" + floor1_free; 
    wire [7:0] floor2_ascii = "0" + floor2_free;
    wire p0, p1, p2, p3;
    edge_detector_p ed0(.clk(clk), .reset_p(reset_p), .cp(rgb0_r), .p_edge(p0));
    edge_detector_p ed1(.clk(clk), .reset_p(reset_p), .cp(rgb1_r), .p_edge(p1));
    edge_detector_p ed2(.clk(clk), .reset_p(reset_p), .cp(rgb2_r), .p_edge(p2));
    edge_detector_p ed3(.clk(clk), .reset_p(reset_p), .cp(rgb3_r), .p_edge(p3));

    wire entry_pedge = p0 | p1 | p2 | p3;
    reg [31:0] tmr;
    reg [5:0] cnt_st;
    reg  [15:0] plate_lat;
    reg init_flag;
    reg [10:0] cnt_data, cnt_data_1, cnt_data_2, cnt_data_3;
    reg force_status;
    reg [1:0] prev_floor1, prev_floor2;
    reg       need_refresh;
    reg [31:0] refresh_holdoff;
    localparam integer REFRESH_HOLDOFF_TICKS = 50_000_000/5;

    always @(posedge clk, posedge reset_p) begin
        if(reset_p)begin
           next_state = IDLE;
           init_flag = 0;
           force_status = 1'b0;
           count_clk_e = 0;
           send = 0;
           send_buffer = 0;
           rs = 0; 
           cnt_data = 0;
           cnt_data_1 = 0;
           cnt_data_2 = 0;
           cnt_data_3 = 0;
           prev_floor1      = 2'd3;   // 존재하지 않는 값으로 시작 → 첫 프레임 강제 갱신
            prev_floor2      = 2'd3;
            need_refresh     = 1'b1;   // 첫 화면 그리기
            refresh_holdoff  = 0;
        end
        else begin
             if ((floor1_free != prev_floor1) || (floor2_free != prev_floor2)) begin
                prev_floor1  <= floor1_free;
                prev_floor2  <= floor2_free;
                need_refresh <= 1'b1;
            end
            if (refresh_holdoff != 0) refresh_holdoff <= refresh_holdoff - 1;
            case (state)
                IDLE                : begin
                    if (init_flag) begin
                    next_state = EMPTY_NUM;
                        if(rgb_r_pedge)next_state = SEND_WELCOME;
                    end
                    else begin
                        if(cnt_sysclk < 32'd8_000_000)begin
                            count_clk_e = 1;
                        end
                        else begin
                            next_state = INIT;
                            count_clk_e = 0;
                        end
                    end
                end
                INIT                : begin
                    if(busy)begin
                        send = 0;
                        if(cnt_data >= 6)begin
                            cnt_data = 0;
                            next_state = IDLE;
                            init_flag = 1;
                        end
                    end
                    else if(!send)begin
                        case (cnt_data)
                            0:send_buffer = 8'h33;
                            1:send_buffer = 8'h32;
                            2:send_buffer = 8'h28; 
                            3:send_buffer = 8'h0C; 
                            4:send_buffer = 8'h01; 
                            5:send_buffer = 8'h06;  
                        endcase
                        send = 1;
                        cnt_data = cnt_data + 1;
                    end
                end
                SEND_WELCOME: begin
                if (busy) begin
                    send = 0;       
                end else if (!send) begin
                    if (cnt_data_1 == 0) begin
                        rs          = 0;
                        send_buffer = 8'h01;
                        send        = 1;
                        cnt_data_1  = 1;
                    end else if (cnt_data_1 == 1) begin
                        rs          = 0;
                        send_buffer = 8'h80; 
                        send        = 1;
                        cnt_data_1  = 2; 
                    end else begin
                        rs = 1;   
                        case (cnt_data_1)
                            2:  send_buffer = "W";
                            3:  send_buffer = "E";
                            4:  send_buffer = "L";
                            5:  send_buffer = "C";
                            6:  send_buffer = "O";
                            7:  send_buffer = "M";
                            8:  send_buffer = "E";
                            default: begin
                                cnt_data_1 = 0;
                                tmr = 0;
                                next_state = WAIT_WELCOME;
                            end
                        endcase

                        if (cnt_data_1 >= 2 && cnt_data_1 <= 8) begin
                            send       = 1;
                            cnt_data_1 = cnt_data_1 + 1;
                            if (cnt_data_1 == 9) begin
                                cnt_data_1 = 0;
                                tmr = 0;
                                next_state = WAIT_WELCOME;
                            end
                        end
                    end
                end
            end 

            WAIT_WELCOME: begin
                send = 0;                     // ★ 추가: 다음 상태 진입 전 항상 send LOW
                if (tmr < 299_999_999) begin
                    tmr = tmr + 1;
                end else begin
                    tmr = 0;                  // (권장) 타이머 리셋
                    cnt_data_2 = 0;           // (권장) 다음 상태용 카운터 초기화
                    next_state = ENTER_CAR_NUM;
                end
            end

            ENTER_CAR_NUM: begin
                if (busy) begin
                    send = 0;
                end else if (!send) begin
                    if (cnt_data_2 == 0) begin
                        rs          = 0;
                        send_buffer = 8'h01; 
                        send        = 1;
                        cnt_data_2  = 1;
                    end else if (cnt_data_2 == 1) begin
                        rs          = 0;
                        send_buffer = 8'h0F; 
                        send        = 1;
                        cnt_data_2  = 2;
                    end else if (cnt_data_2 == 2) begin
                        rs          = 0;
                        send_buffer = 8'h80; 
                        send        = 1;
                        cnt_data_2  = 3;
                    end else begin
                        rs = 1;
                        case (cnt_data_2)
                            3:  send_buffer = "E";
                            4:  send_buffer = "N";
                            5:  send_buffer = "T";
                            6:  send_buffer = "E";
                            7:  send_buffer = "R";
                            8:  send_buffer = " ";
                            9:  send_buffer = "C";
                            10:  send_buffer = "A";
                            11:  send_buffer = "R";
                            12: send_buffer = " ";
                            13: send_buffer = "N";
                            14: send_buffer = "U";
                            15: send_buffer = "M";
                            16: send_buffer = ":";
                            default: begin
                                cnt_data_2 = 0;
                                next_state = SEND_CAR_NUM;
                            end
                        endcase
        
                        if (cnt_data_2 >= 3 && cnt_data_2 <= 16) begin
                        send       = 1;
                        cnt_data_2 = cnt_data_2 + 1;
                        if (cnt_data_2 == 17) begin
                            cnt_data_2 = 0;
                            next_state = SET_CURSOR_LINE2;
                                end
                            end
                        end
                    end
                end     
                SET_CURSOR_LINE2: begin
                if (busy) begin
                    send = 0;
                end else if (!send) begin
                    rs          = 0;
                    send_buffer = 8'hC0; // 2행 시작 주소
                    send        = 1;
                    cnt_data_3   = 0;
                    next_state  = SEND_CAR_NUM;
                    end
                end
                SEND_CAR_NUM  : begin
                if (busy) begin
                    send = 0;
                end else if (key_valid_pedge && !send) begin
                    if (is_digit && (cnt_data_3 < 4)) begin
                        rs          = 1;
                        send_buffer = "0" + key_value;
                        send        = 1;
                        cnt_data_3  = cnt_data_3 + 1;
                    end
                    // ★ 4자리 완료되면 강제 갱신 플래그 세움
                    if (cnt_data_3 == 4) begin
                        send            = 0;      // 잔상 제거
                        cnt_st          = 0;      // STATUS 출력 시퀀스 처음부터
                        force_status    = 1'b1;   // 강제 갱신 요청
                        refresh_holdoff = 0;      // 쿨다운 무시
                        next_state      = EMPTY_NUM;
                    end else begin
                        next_state = SEND_CAR_NUM;
                    end
                end
                end
                EMPTY_NUM            : begin
                    if (rgb_r_pedge) begin
                        if (busy) send = 0;
                        cnt_st     = 0;
                        next_state = SEND_WELCOME;
                        force_status    = 1'b0; 
                    end
                    // ② 값이 바뀐 경우 + 쿨다운 끝난 경우에만 한 프레임 그림
                    else if ( force_status || (need_refresh && (refresh_holdoff == 0)) ) begin
                        if (busy) begin
                        send = 0;
                        end else if (!send) begin
                        case (cnt_st)
                            0:  begin rs=0; send_buffer=8'h01; send=1; cnt_st=1; end
                            1:  begin rs=0; send_buffer=8'h80; send=1; cnt_st=2; end
                            2:  begin rs=1; send_buffer="1"; send=1; cnt_st=3; end
                            3:  begin rs=1; send_buffer=" "; send=1; cnt_st=4; end
                            4:  begin rs=1; send_buffer="F"; send=1; cnt_st=5; end
                            5:  begin rs=1; send_buffer="L"; send=1; cnt_st=6; end
                            6:  begin rs=1; send_buffer="O"; send=1; cnt_st=7; end
                            7:  begin rs=1; send_buffer="O"; send=1; cnt_st=8; end
                            8:  begin rs=1; send_buffer="R"; send=1; cnt_st=9; end
                            9:  begin rs=1; send_buffer=" "; send=1; cnt_st=10; end
                            10: begin rs=1; send_buffer=":"; send=1; cnt_st=11; end
                            11: begin rs=1; send_buffer=" "; send=1; cnt_st=12; end
                            12: begin rs=1; send_buffer=floor1_ascii; send=1; cnt_st=13; end
                            13: begin rs=0; send_buffer=8'hC0; send=1; cnt_st=14; end
                            14: begin rs=1; send_buffer="2"; send=1; cnt_st=15; end
                            15: begin rs=1; send_buffer=" "; send=1; cnt_st=16; end
                            16: begin rs=1; send_buffer="F"; send=1; cnt_st=17; end
                            17: begin rs=1; send_buffer="L"; send=1; cnt_st=18; end
                            18: begin rs=1; send_buffer="O"; send=1; cnt_st=19; end
                            19: begin rs=1; send_buffer="O"; send=1; cnt_st=20; end
                            20: begin rs=1; send_buffer="R"; send=1; cnt_st=21; end
                            21: begin rs=1; send_buffer=" "; send=1; cnt_st=22; end
                            22: begin rs=1; send_buffer=":"; send=1; cnt_st=23; end
                            23: begin rs=1; send_buffer=" "; send=1; cnt_st=24; end
                            24: begin
                            rs=1; send_buffer=floor2_ascii; send=1;

                            // ★ 프레임 완료: 다음을 위해 정리
                            cnt_st          = 0;
                            need_refresh    = 1'b0;                      // 이번에 그렸으니 더는 안 그림
                            refresh_holdoff = REFRESH_HOLDOFF_TICKS;     // (옵션) 200ms 쿨다운
                            next_state      = EMPTY_NUM;   
                            force_status    = 1'b0;              // 기본 상태 유지
                            end
                        endcase
                        end
                    end
                    // ③ 변화도 없고, 웰컴도 아니면 아무것도 안 함(깜빡임 X)
                    else begin
                        if (busy) send = 0;
                        next_state = EMPTY_NUM;
                    end
                end
            endcase
        end
    end
endmodule








/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


module CHUL_cha (
    input clk, reset_p,
    input echo,
    input  [3:0] row,
    output [3:0] column,
    input [15:0] parking_time_i,    
    input [15:0] lcd_number_i,      
    input        data_valid_i,      
    input        not_found_i,     
    input        key_valid_i,       
    input [3:0]  key_value_i,      
    output scl, sda,
    output trig,
    output buz_sig,
    output motor,
    output reg mode_req_in,
    output [15:0] led
);

    integer cnt_sysclk;
    reg count_clk_e;
    always @(negedge clk, posedge reset_p) begin
        if (reset_p) begin
            cnt_sysclk = 0;
        end
        else if (count_clk_e) begin
            cnt_sysclk = cnt_sysclk + 1;
        end
        else cnt_sysclk = 0;
    end


    reg [7:0] send_buffer;
    reg send, rs;
    wire busy;
    
    wire [3:0] key_value;
    wire [15:0] lcd_number;
    wire is_digit = (key_value_i <= 4'd9); 
    wire [15:0] parking_time;
    wire data_valid, not_found;
    wire plate_found = data_valid_i && ~not_found_i;
//    wire [3:0] row;
//    wire [3:0] column;
    
    I2C_lcd_send_byte lcd_byte(
    clk, reset_p,
    7'h27,
    send_buffer,
    send, rs,
    scl, sda,
    busy,
    led
    );
    
    wire key_valid_pedge;
    edge_detector_p key_valid_ed(
        .clk(clk), .reset_p(reset_p), .cp(key_valid_i), .p_edge(key_valid_pedge)
    );

    localparam IDLE          = 10'b00_0000_0001;
    localparam INIT          = 10'b00_0000_0010;
    localparam WAIT_PLS      = 10'b00_0000_0100; 
    localparam STOP_MSG      = 10'b00_0000_1000;
    localparam STOP_WAIT     = 10'b00_0001_0000;
    localparam ENTER_CAR_NUM = 10'b00_0010_0000;
    localparam SET_LINE2     = 10'b00_0100_0000;
    localparam SEND_CAR_NUM  = 10'b00_1000_0000;
    localparam PAY           = 10'b01_0000_0000;
    localparam WAIT_RESULT   = 10'b10_0000_0000;

    reg [9:0] state, next_state;
    // assign led[9:0] = state;
    // assign led[10] = busy;
    always @(negedge clk, posedge reset_p) begin
        if(reset_p) begin
            state = IDLE;
        end
        else begin
            state = next_state;
        end
    end

    wire  rgb_r;
    wire rgb_r_pedge;
    edge_detector_p rgb_r_ed(
        .clk(clk), .reset_p(reset_p), .cp(rgb_r), .p_edge(rgb_r_pedge)
    );

    always @(posedge clk or posedge reset_p) begin
        if (reset_p) mode_req_in <= 1'b0;
        else begin
            mode_req_in <= 1'b0;
            if (key_valid_pedge && key_value_i==4'd11) mode_req_in <= 1'b1; // 출차 전환
        end
    end

    parking_spot_ctrl ultrasonic(
    .clk(clk),
    .reset_p(reset_p),
    .echo(echo),
    .trig(trig),
    .rgb_r(rgb_r)
    );

    reg init_flag;
    reg [10:0] cnt_data, cnt_init, cnt_waitpls, cnt_stop, cnt_enter, cnt_send;
    reg [10:0] cnt_bill;
    reg [31:0] stop_start_secs;

    reg [3:0] plate_d0, plate_d1, plate_d2, plate_d3;
    
    wire [15:0] pay_min   = parking_time_i;
    wire [3:0]  min_tens  = (pay_min / 10) % 10;
    wire [3:0]  min_ones  =  pay_min % 10;
    wire [7:0]  ascii_tens = "0" + min_tens;
    wire [7:0]  ascii_ones = "0" + min_ones;
    
    wire data_valid_pedge;
    edge_detector_p dv_ed(.clk(clk), .reset_p(reset_p), .cp(data_valid_i), .p_edge(data_valid_pedge));
    
    reg [15:0] pay_min_latch;
    always @(posedge clk or posedge reset_p) begin
      if (reset_p) pay_min_latch <= 16'd0;
      else if (data_valid_pedge && !not_found_i) pay_min_latch <= parking_time_i;
    end
    
    assign  led[7:0] = ascii_tens;
    assign  led[15:8] = ascii_ones;

    always @(posedge clk, posedge reset_p) begin
        if(reset_p)begin
           next_state      <= IDLE;
            init_flag       <= 1'b0;
            count_clk_e     <= 1'b0;
            send            <= 1'b0;
            send_buffer     <= 8'h00;
            rs              <= 1'b0;
            cnt_init        <= 0;
            cnt_waitpls     <= 0;
            cnt_stop        <= 0;
            cnt_enter       <= 0;
            cnt_send        <= 0;
            cnt_bill        <= 0;
            stop_start_secs <= 0;
            plate_d0 <= 0; 
            plate_d1 <= 0; 
            plate_d2 <= 0; 
            plate_d3 <= 0;
        end
        else begin
            case (state)
                IDLE                : begin
                    if (init_flag) begin
                    next_state = WAIT_PLS;
                    end
                    else begin
                        if(cnt_sysclk < 32'd8_000_000)begin
                            count_clk_e = 1;
                        end
                        else begin
                            next_state = INIT;
                            count_clk_e = 0;
                        end
                    end
                end
                INIT                : begin
                    if(busy)begin
                        send = 0;
                        if(cnt_data >= 6)begin
                            cnt_data = 0;
                            next_state = IDLE;
                            init_flag = 1;
                        end
                    end
                    else if(!send)begin
                        case (cnt_data)
                            0:send_buffer = 8'h33;
                            1:send_buffer = 8'h32;
                            2:send_buffer = 8'h28; 
                            3:send_buffer = 8'h0C; 
                            4:send_buffer = 8'h01; 
                            5:send_buffer = 8'h06;  
                        endcase
                        send = 1;
                        cnt_data = cnt_data + 1;
                    end
                end
                WAIT_PLS: begin
                    if (rgb_r_pedge) begin
                        cnt_stop   <= 0;
                        next_state <= STOP_MSG;
                    end else if (!busy && !send) begin
                        case (cnt_waitpls)
                            0:  begin rs<=0; send_buffer<=8'h01; send<=1; cnt_waitpls<=1; end
                            1:  begin rs<=0; send_buffer<=8'h80; send<=1; cnt_waitpls<=2; end
                            2:  begin rs<=1; send_buffer<="W";  send<=1; cnt_waitpls<=3; end
                            3:  begin rs<=1; send_buffer<="A";  send<=1; cnt_waitpls<=4; end
                            4:  begin rs<=1; send_buffer<="I";  send<=1; cnt_waitpls<=5; end
                            5:  begin rs<=1; send_buffer<="T";  send<=1; cnt_waitpls<=6; end
                            6:  begin rs<=1; send_buffer<=" ";  send<=1; cnt_waitpls<=7; end
                            7:  begin rs<=1; send_buffer<="P";  send<=1; cnt_waitpls<=8; end
                            8:  begin rs<=1; send_buffer<="L";  send<=1; cnt_waitpls<=9; end
                            9:  begin rs<=1; send_buffer<="E";  send<=1; cnt_waitpls<=10; end
                            10: begin rs<=1; send_buffer<="A";  send<=1; cnt_waitpls<=11; end
                            11: begin rs<=1; send_buffer<="S";  send<=1; cnt_waitpls<=12; end
                            12: begin rs<=1; send_buffer<="E";  send<=1; cnt_waitpls<=13; end
                            13: begin cnt_waitpls<=13; end
                        endcase
                        next_state <= WAIT_PLS;
                    end else begin
                        if (busy) send <= 0;
                        next_state <= WAIT_PLS;
                    end
                end
                STOP_MSG: begin
                    if (busy) begin
                        send <= 0;
                        next_state <= STOP_MSG;
                    end else if (!send) begin
                        case (cnt_stop)
                            0:  begin rs<=0; send_buffer<=8'h01; send<=1; cnt_stop<=1; end
                            1:  begin rs<=0; send_buffer<=8'h80; send<=1; cnt_stop<=2; end
                            2:  begin rs<=1; send_buffer<="S"; send<=1; cnt_stop<=3; end
                            3:  begin rs<=1; send_buffer<="T"; send<=1; cnt_stop<=4; end
                            4:  begin rs<=1; send_buffer<="O"; send<=1; cnt_stop<=5; end
                            5:  begin rs<=1; send_buffer<="P"; send<=1; cnt_stop<=6; end
                            6:  begin rs<=1; send_buffer<="!"; send<=1; cnt_stop<=7; end
                            7:  begin rs<=1; send_buffer<="!"; send<=1; cnt_stop<=8; end
                            8:  begin rs<=0; send_buffer<=8'hC0; send<=1; cnt_stop<=9; end
                            9:  begin rs<=1; send_buffer<="W"; send<=1; cnt_stop<=10; end
                            10: begin rs<=1; send_buffer<="a"; send<=1; cnt_stop<=11; end
                            11: begin rs<=1; send_buffer<="i"; send<=1; cnt_stop<=12; end
                            12: begin rs<=1; send_buffer<="t"; send<=1; cnt_stop<=13; end
                            13: begin rs<=1; send_buffer<=" "; send<=1; cnt_stop<=14; end
                            14: begin rs<=1; send_buffer<="a"; send<=1; cnt_stop<=15; end
                            15: begin rs<=1; send_buffer<=" "; send<=1; cnt_stop<=16; end
                            16: begin rs<=1; send_buffer<="s"; send<=1; cnt_stop<=17; end
                            17: begin rs<=1; send_buffer<="e"; send<=1; cnt_stop<=18; end
                            18: begin rs<=1; send_buffer<="c"; send<=1; cnt_stop<=19; end
                            19: begin rs<=1; send_buffer<="o"; send<=1; cnt_stop<=20; end
                            20: begin rs<=1; send_buffer<="n"; send<=1; cnt_stop<=21; end
                            21: begin rs<=1; send_buffer<="d"; send<=1; cnt_stop<=22; end
                            22: begin cnt_stop<=0; next_state<=STOP_WAIT; end
                        endcase
                end
                end
                STOP_WAIT: begin
                    if (cnt_sysclk < 32'd100_000_000) begin 
                        count_clk_e <= 1;
                        next_state  <= STOP_WAIT;
                    end else begin
                        count_clk_e <= 1'b0;
                        cnt_enter   <= 0;
                        next_state  <= ENTER_CAR_NUM;
                    end
                end

                ENTER_CAR_NUM: begin
                    if (busy) begin
                        send <= 0;
                        next_state <= ENTER_CAR_NUM;
                    end else if (!send) begin
                        if (cnt_enter == 0) begin
                            rs<=0; send_buffer<=8'h01; send<=1; cnt_enter<=1;
                        end else if (cnt_enter == 1) begin
                            rs<=0; send_buffer<=8'h80; send<=1; cnt_enter<=2;
                        end else begin
                            rs<=1;
                            case (cnt_enter)
                                2:  send_buffer<="E";
                                3:  send_buffer<="N";
                                4:  send_buffer<="T";
                                5:  send_buffer<="E";
                                6:  send_buffer<="R";
                                7:  send_buffer<=" ";
                                8:  send_buffer<="C";
                                9:  send_buffer<="A";
                                10: send_buffer<="R";
                                11: send_buffer<=" ";
                                12: send_buffer<="N";
                                13: send_buffer<="U";
                                14: send_buffer<="M";
                                15: send_buffer<=":";
                                default: begin cnt_enter<=0; next_state<=SET_LINE2; end
                            endcase
                            if (cnt_enter>=2 && cnt_enter<=15) begin
                                send<=1; cnt_enter<=cnt_enter+1;
                            end
                        end
                    end
                end

                SET_LINE2: begin
                    if (busy) begin
                        send <= 0;
                        next_state <= SET_LINE2;
                    end else if (!send) begin
                        rs<=0; send_buffer<=8'hC0; send<=1;
                        cnt_send<=0;
                        plate_d0<=0; plate_d1<=0; plate_d2<=0; plate_d3<=0;
                        next_state<=SEND_CAR_NUM;
                    end
                end

               SEND_CAR_NUM: begin
                    if (busy) begin
                        send <= 0;
                        next_state <= SEND_CAR_NUM;
                    end
                    else if (key_valid_pedge && !send) begin
                        if (is_digit && (cnt_send < 4)) begin
                            rs <= 1;
                            send_buffer <= "0" + key_value_i;
                            send <= 1;

                            case (cnt_send)
                                0: plate_d0 <= key_value_i;
                                1: plate_d1 <= key_value_i;
                                2: plate_d2 <= key_value_i;
                                3: plate_d3 <= key_value_i;
                            endcase

                            if (cnt_send == 3) begin
                                cnt_send   <= 4;    
                                next_state <= WAIT_RESULT; 
                            end else begin
                                cnt_send   <= cnt_send + 1;
                                next_state <= SEND_CAR_NUM;
                            end
                        end else begin
                            next_state <= SEND_CAR_NUM;
                        end
                    end
                    else begin
                        next_state <= SEND_CAR_NUM;
                    end
                end
                PAY: begin
                if (busy) begin
                    send <= 0;
                    next_state <= PAY;
                end else if (!send) begin
                    case (cnt_bill)
                        0:  begin rs<=0; send_buffer<=8'h01; send<=1; cnt_bill<=1; end
                        1:  begin rs<=0; send_buffer<=8'h80; send<=1; cnt_bill<=2; end
                        2:  begin rs<=1; send_buffer<="T"; send<=1; cnt_bill<=3; end
                        3:  begin rs<=1; send_buffer<="I"; send<=1; cnt_bill<=4; end
                        4:  begin rs<=1; send_buffer<="M"; send<=1; cnt_bill<=5; end
                        5:  begin rs<=1; send_buffer<="E"; send<=1; cnt_bill<=6; end
                        6:  begin rs<=1; send_buffer<=":"; send<=1; cnt_bill<=7; end
                        7:  begin rs<=1; send_buffer<=" "; send<=1; cnt_bill<=8; end
                        8:  begin rs<=1; send_buffer<=ascii_tens; send<=1; cnt_bill<=9; end
                        9:  begin rs<=1; send_buffer<=ascii_ones; send<=1; cnt_bill<=10; end
                        10: begin rs<=1; send_buffer<="m"; send<=1; cnt_bill<=11; end
                        11: begin rs<=0; send_buffer<=8'hC0; send<=1; cnt_bill<=12; end
                        12: begin rs<=1; send_buffer<="P"; send<=1; cnt_bill<=13; end
                        13: begin rs<=1; send_buffer<="a"; send<=1; cnt_bill<=14; end
                        14: begin rs<=1; send_buffer<="y"; send<=1; cnt_bill<=15; end
                        15: begin rs<=1; send_buffer<=" "; send<=1; cnt_bill<=16; end
                        16: begin rs<=1; send_buffer<="p"; send<=1; cnt_bill<=17; end
                        17: begin rs<=1; send_buffer<="l"; send<=1; cnt_bill<=18; end
                        18: begin rs<=1; send_buffer<="e"; send<=1; cnt_bill<=19; end
                        19: begin rs<=1; send_buffer<="a"; send<=1; cnt_bill<=20; end
                        20: begin rs<=1; send_buffer<="s"; send<=1; cnt_bill<=21; end
                        21: begin rs<=1; send_buffer<="e"; send<=1; cnt_bill<=22; end
                        22: begin cnt_bill<=22; end
                    endcase
                    next_state <= PAY;
                end
                if (key_valid_pedge && key_value_i==4'd12) begin
                    cnt_waitpls <= 0;
                    cnt_send    <= 0;
                    next_state  <= WAIT_PLS;
                end
                end
            

                WAIT_RESULT: begin
                if (data_valid_pedge) begin
                    if (!not_found_i) begin
                        // 차 번호 찾음 → PAY로
                        cnt_bill   <= 0;
                        next_state <= PAY;
                    end else begin
                        if (busy) begin
                            send <= 0;
                            next_state <= WAIT_RESULT;
                        end else if (!send) begin
                            rs <= 0; send_buffer <= 8'h01; send <= 1;
                            cnt_send <= 0;
                            next_state <= ENTER_CAR_NUM;
                        end
                    end
                end else begin
                    // 아직 검색 중 → 그대로 대기
                    next_state <= WAIT_RESULT;
                end
            end
            default: next_state <= IDLE;
    
            endcase
        end
    end
    
endmodule





module IP_CHUL_cha (
    input clk, reset_p,
    input echo_i,echo_o,echo0,echo1,echo2,echo3,
    input  [3:0] row,
    output [3:0] column,
    output scl_i, sda_i,
    output scl_o, sda_o,
    output trig_i,
    output rgb0_r, rgb1_r, rgb2_r, rgb3_r,
    output rgb0_g, rgb1_g, rgb2_g, rgb3_g,
    output buz_sig,
    output motor,
    output [15:0] led
);
    wire [15:0] parking_time_bus;
    wire [15:0] lcd_number_bus;
    wire        data_valid_bus, not_found_bus;
    wire        key_valid_bus;
    wire [3:0]  key_value_bus;
    wire [3:0] column_ip;
//    wire [3:0] column_chul;

    wire req_out_from_ip;
    wire req_in_from_chul; 
    
    IP_cha Input_LCD(
    .clk(clk), .reset_p(reset_p),
    .echo(echo_i),
    .row(row),
    .column(column_ip),
    .echo0(echo0), 
    .echo1(echo1), 
    .echo2(echo2), 
    .echo3(echo3),
    .parking_time_o(parking_time_bus),
    .lcd_number_o(lcd_number_bus),
    .data_valid_o(data_valid_bus),
    .not_found_o(not_found_bus),
    .key_valid_o(key_valid_bus),
    .key_value_o(key_value_bus), 
    .rgb0_r(rgb0_r), 
    .rgb1_r(rgb1_r), 
    .rgb2_r(rgb2_r), 
    .rgb3_r(rgb3_r),
    .rgb0_g(rgb0_g), 
    .rgb1_g(rgb1_g), 
    .rgb2_g(rgb2_g), 
    .rgb3_g(rgb3_g),
    .scl(scl_i), .sda(sda_i),
    .buz_sig(buz_sig),
    .motor(motor),
    .mode_req_out(req_out_from_ip),
    .trig(trig_i)
    );

CHUL_cha Output_LCD(
    .clk(clk), .reset_p(reset_p),
    .echo(echo_o),
   .row(row),
//   .column(column_chul),
    .scl(scl_o), .sda(sda_o),
    .parking_time_i(parking_time_bus),
    .lcd_number_i(lcd_number_bus),
    .data_valid_i(data_valid_bus),
    .not_found_i(not_found_bus),
    .key_valid_i(key_valid_bus),
    .key_value_i(key_value_bus),
    .trig(trig_o),
    .mode_req_in(req_in_from_chul)
    );


    assign column = column_ip;

endmodule