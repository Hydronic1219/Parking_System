`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/26/2025 02:26:30 PM
// Design Name: 
// Module Name: outside_module
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

module edge_detector_n(
    input clk,
    input reset_p,
    input cp,

    output p_edge,
    output n_edge

    );

    reg ff_cur, ff_old;

    always @(negedge clk or posedge reset_p) begin
        if(reset_p) begin
            ff_cur <= 0;
            ff_old <= 0;
        end else begin
            ff_old <= ff_cur;
            ff_cur <= cp;
        end
    end

    assign p_edge = ({ff_cur, ff_old} == 2'b10) ? 1 : 0;
    assign n_edge = ({ff_cur, ff_old} == 2'b01) ? 1 : 0;
endmodule

    // cur = 1, old = 0 => p = 1
    // cur = 1, old = 1 => p = 0
    // cur = 0, old = 1 => n = 1



module edge_detector_p(
    input clk,
    input reset_p,
    input cp,

    output p_edge,
    output n_edge

    );

    reg ff_cur, ff_old;

    always @(posedge clk or posedge reset_p) begin
        if(reset_p) begin
            ff_cur <= 0;
            ff_old <= 0;
        end else begin
            ff_old <= ff_cur;
            ff_cur <= cp;
        end
    end

    assign p_edge = ({ff_cur, ff_old} == 2'b10) ? 1'b1 : 1'b0;
    assign n_edge = ({ff_cur, ff_old} == 2'b01) ? 1'b1 : 1'b0;
endmodule



module seg_decoder (
    input [3:0] digit_in,
    output reg [7:0] seg_out
);

    always @(*) begin
        case (digit_in)
            4'd0: seg_out = 8'b1100_0000;   // 0 (dp 꺼짐)
            4'd1: seg_out = 8'b1111_1001; // 1
            4'd2: seg_out = 8'b1010_0100; // 2
            4'd3: seg_out = 8'b1011_0000; // 3
            4'd4: seg_out = 8'b1001_1001; // 4
            4'd5: seg_out = 8'b1001_0010; // 5
            4'd6: seg_out = 8'b1000_0010; // 6
            4'd7: seg_out = 8'b1111_1000; // 7
            4'd8: seg_out = 8'b1000_0000; // 8
            4'd9: seg_out = 8'b1001_0000; // 9
            4'd10: seg_out = 8'b1000_1000;  //a
            4'd11: seg_out = 8'b1000_0011;  //b
            4'd12: seg_out = 8'b1100_0110;  //c
            4'd13: seg_out = 8'b1010_0001;  //d
            4'd14: seg_out = 8'b1000_0110;  //e
            4'd15: seg_out = 8'b1000_1110;  //f
            default: seg_out = 8'b1111_1111;
        endcase
    end
endmodule

module anode_selector (
    input [1:0] scan_count,
    output reg [3:0] an_out
);
    always @(*) begin
        case (scan_count)
            2'd0: an_out = 4'b1110; // an[0]
            2'd1: an_out = 4'b1101; // an[1]
            2'd2: an_out = 4'b1011; // an[2]
            2'd3: an_out = 4'b0111; // an[3]
            default: an_out = 4'b1111;
        endcase
    end
endmodule





module bin_to_dec(
    input [11:0] bin,           // 12비트 이진 입력
    output reg [15:0] bcd       // 4자리의 BCD를 출력 (4비트 x 4자리)
    );

    integer i;      // 반복문

    always @(bin) begin

        bcd = 0;    // initial value

        for(i = 0; i < 12; i = i + 1) begin
            // 1) 알고리즘 각 단위비트 자리별로 5이상이면 +3 해줌
            if(bcd[3:0] >= 5) bcd[3:0] = bcd[3:0] + 3;          // 1의 자리수
            if(bcd[7:4] >= 5) bcd[7:4] = bcd[7:4] + 3;          // 10의 자리수
            if(bcd[11:8] >= 5) bcd[11:8] = bcd[11:8] + 3;       // 100의 자리수
            if(bcd[15:12] >= 5) bcd[15:12] = bcd[15:12] +3;     // 1000의 자리수

            // 2) 1비트 left shift + 새로운 비트 붙임
            bcd = {bcd[14:0], bin[11 - i]};
        end

    end
endmodule


module fnd_cntr(
    input clk,
    input reset_p,
    input [15:0] fnd_value,     // 표시할 값 (HEX or BCD)
    input hex_bcd,              // 1이면 HEX, 0이면 BCD
    output [7:0] seg_7,
    output [3:0] com
    );

    wire [15:0] bcd_value;

    bin_to_dec bcd(
        .bin(fnd_value[11:0]),  // 하위 12비트만 변환
        .bcd(bcd_value)         // 변환된 BCD 값 
        );

    // 클럭 분주 -> 2비트 scan_count 생성 (4자리 스캔용) 
    reg [16:0] clk_div;
    always @(posedge clk) clk_div = clk_div + 1;

    // 자리 선택 : 2비트 scan_count로 4자리 중 하나 선택 
    anode_selector ring_com(
        .scan_count(clk_div[16:15]),
        .an_out(com)
    );

    reg [3:0] digit_value;  // 현재 선택된 자리의 한 자리 값
    wire [15:0] out_value;  // HEX or BCD 변환 값 선택 
    assign out_value = hex_bcd ? fnd_value : bcd_value;

    always @(posedge clk or posedge reset_p) begin
        if(reset_p) begin
            digit_value = 0;
        end
        else begin
            case(com)
                4'b1110 : digit_value = out_value[3:0];
                4'b1101 : digit_value = out_value[7:4];
                4'b1011 : digit_value = out_value[11:8];
                4'b0111 : digit_value = out_value[15:12];
            endcase
        end
    end

    seg_decoder dec(
        .digit_in(digit_value),
        .seg_out(seg_7)
    );

endmodule




module keypad_cntr (
    input clk, reset_p,
    input [3:0] row,
    output reg [3:0] column,
    output reg [3:0] key_value,
    output reg key_valid
    );

    localparam SCAN_0      = 5'b0_0001;
    localparam SCAN_1      = 5'b0_0010;
    localparam SCAN_2      = 5'b0_0100;
    localparam SCAN_3      = 5'b0_1000;
    localparam KEY_PROCESS = 5'b1_0000;

    reg [19:0] clk_10ms;
    always @(posedge clk) begin
        clk_10ms = clk_10ms + 1;
    end

    wire clk_10ms_nedge, clk_10ms_pedge;
    edge_detector_p clk_10ms_ed(
        .clk(clk), .reset_p(reset_p), .cp(clk_10ms[19]), .p_edge(clk_10ms_pedge), .n_edge(clk_10ms_nedge)
    );

    reg [4:0] state, next_state;
    always @(posedge clk, posedge reset_p) begin
        if(reset_p) begin
            state = SCAN_0;
        end 
        else if(clk_10ms_pedge)state = next_state;
    end


    always @(*) begin                           // 항상 실행 되서 조합회로 생성
        case (state)
            SCAN_0      : begin
                if(row == 0)next_state = SCAN_1;
                else next_state = KEY_PROCESS;
            end
            SCAN_1      : begin
                if(row == 0)next_state = SCAN_2;
                else next_state = KEY_PROCESS;
            end
            SCAN_2      : begin
                if(row == 0)next_state = SCAN_3;
                else next_state = KEY_PROCESS;
            end
            SCAN_3      : begin
                if(row == 0)next_state = SCAN_0;
                else next_state = KEY_PROCESS;
            end
            KEY_PROCESS : begin
                if(row == 0)next_state = SCAN_0;
                else next_state = KEY_PROCESS;
            end
        endcase
    end

    always @(posedge clk, posedge reset_p) begin
        if(reset_p) begin
           column = 4'b0001;
           key_value = 0;
           key_valid = 0; 
        end
        else if(clk_10ms_nedge) begin
            case (state)
                SCAN_0      : begin
                    column = 4'b0001;
                    key_valid = 0;
                end
                SCAN_1      : begin
                    column = 4'b0010;
                    key_valid = 0;
                end
                SCAN_2      : begin
                    column = 4'b0100;
                    key_valid = 0;
                end
                SCAN_3      : begin
                    column = 4'b1000;
                    key_valid = 0;
                end
                KEY_PROCESS : begin
                    key_valid = 1;
                    case ({column, row})                
                        8'b0001_0001 : key_value = 4'h0;
                        8'b0001_0010 : key_value = 4'h1;
                        8'b0001_0100 : key_value = 4'h2;
                        8'b0001_1000 : key_value = 4'h3;
                        8'b0010_0001 : key_value = 4'h4;
                        8'b0010_0010 : key_value = 4'h5;
                        8'b0010_0100 : key_value = 4'h6;
                        8'b0010_1000 : key_value = 4'h7;
                        8'b0100_0001 : key_value = 4'h8; 
                        8'b0100_0010 : key_value = 4'h9;
                        8'b0100_0100 : key_value = 4'hA;
                        8'b0100_1000 : key_value = 4'hB;
                        8'b1000_0001 : key_value = 4'hC;                
                        8'b1000_0010 : key_value = 4'hD;
                        8'b1000_0100 : key_value = 4'hE;
                        8'b1000_1000 : key_value = 4'hF;
                        


                    endcase                           
                end
            endcase
        end
    end

endmodule


module memory(
    input clk, 
    input rd_en, wr_en,
    input [9:0] wr_addr,
    input [9:0] rd_addr,
    input [31:0] i_data,
    output reg [31:0] o_data);

    reg [31:0] ram [0:1023];
    always @(posedge clk)begin
        if(wr_en)ram[wr_addr] <= i_data;
    end
    always @(posedge clk)begin
        if(rd_en)o_data <= ram[rd_addr];
    end

endmodule



module pwm_Nstep (
    input clk, reset_p,
    input [31:0] duty,
    output reg pwm
);
    
    parameter sys_clk_freq = 100_000_000;
    parameter pwm_freq = 10_000;                                // led 10000hz짜리  
    parameter duty_step_N = 200;                                  // N분주
    parameter temp = sys_clk_freq / pwm_freq / duty_step_N / 2;   // 2를 나누는 것은 1,0이 한주기 이므로 2로 나눠줌

    integer cnt;
    reg pwm_freqXN;
    always @(posedge clk, posedge reset_p) begin
        if(reset_p)begin
            cnt = 0;
            pwm_freqXN = 0;
        end
        else begin
            if(cnt >= temp - 1)begin
                cnt = 0;
                pwm_freqXN = ~pwm_freqXN;
            end
            else cnt = cnt + 1;
        end
    end

    wire pwm_freqXN_nedge;
    edge_detector_p send_ed(
        .clk(clk), .reset_p(reset_p), .cp(pwm_freqXN),
        .n_edge(pwm_freqXN_nedge));

    integer cnt_duty;
    always @(posedge clk, posedge reset_p) begin
        if(reset_p) begin
            cnt_duty = 0;
            pwm = 0;
        end
        else if(pwm_freqXN_nedge) begin
            if(cnt_duty >= duty_step_N) cnt_duty = 0; 
            cnt_duty = cnt_duty + 1;
            if(cnt_duty < duty) pwm = 1;
            else pwm = 0;
        end
    end


endmodule




module buzzer (
    input  clk, reset_p,
    input  buzzer_en,
    output reg buz_sig
);
    
    localparam IDLE = 2'b01;
    localparam BEEP = 2'b10;
    
    reg [1:0] state, next_state;
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) state <= IDLE;
        else         state <= next_state;
    end

    wire buzzer_en_pedge;
    edge_detector_p buzzer_en_ed(
        .clk(clk), .reset_p(reset_p), .cp(buzzer_en), .p_edge(buzzer_en_pedge)
    );


    reg [31:0] cnt;
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            cnt <= 32'd0;
        end else if (state == BEEP) begin
            if (cnt >= 32'd499_999_999) cnt <= 32'd0;   
            else                         cnt <= cnt + 1;
        end else begin
            cnt <= 32'd0;                                
        end
    end

    // 10ms 토글 카운터
    reg [19:0] cnt10ms;
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            next_state <= IDLE;
            cnt10ms    <= 20'd0;
            buz_sig    <= 1'b0;
        end else begin
            next_state <= next_state;

            case (state)
                IDLE: begin
                    cnt10ms <= 20'd0;
                    buz_sig <= 1'b0;
                    if (buzzer_en_pedge) begin
                        next_state <= BEEP;
                    end
                end

                BEEP: begin
                    if (cnt >= 32'd499_999_999) begin
                        next_state <= IDLE;
                        cnt10ms    <= 20'd0;
                        buz_sig    <= 1'b0;
                    end else begin
                        if (cnt10ms >= 20'd999_999) begin
                            cnt10ms <= 20'd0;
                            buz_sig <= ~buz_sig;
                        end else begin
                            cnt10ms <= cnt10ms + 1;
                        end
                    end
                end
            endcase
        end
    end

endmodule





module I2C_master_write_only (  //100Khz로 I2C통신
    input clk, reset_p,
    input [6:0] addr,
    input [7:0] data,
    input rd_wr, comm_start,
    output reg scl, sda, 
    output [15:0] led
    );

    localparam IDLE = 7'b000_0001;
    localparam COMM_START = 7'b000_0010;
    localparam SEND_ADDR = 7'b000_0100;
    localparam RD_ACK = 7'b000_1000;
    localparam SEND_DATA = 7'b001_0000;
    localparam SCL_STOP = 7'b010_0000;
    localparam COMM_STOP = 7'b100_0000;

    wire clk_usec_nedge;
    clock_div_100 us_clk(
    .clk(clk), .reset_p(reset_p),
    .nedge_div_100(clk_usec_nedge)
    );

    wire comm_start_nedge, comm_start_pedge;
    edge_detector_p comm_start_ed(
        .clk(clk), .reset_p(reset_p), .cp(comm_start), .p_edge(comm_start_pedge), .n_edge(comm_start_nedge)
    );

    wire scl_nedge, scl_pedge;
    edge_detector_p scl_ed(
        .clk(clk), .reset_p(reset_p), .cp(scl), .p_edge(scl_pedge), .n_edge(scl_nedge)
    );

    reg [6:0] state, next_state;
    always @(negedge clk, posedge reset_p) begin
        if (reset_p) begin
            state = IDLE;
            end else begin 
            state = next_state;
        end
    end

    reg [2:0] count_usec5;                          // 10us의 주기를 가지는 clk 생성 atmega128에서 100khz였기 때문에 비슷한 속도로 맞춰주기 위해 
    reg scl_e;
    always @(posedge clk, posedge reset_p) begin
        if(reset_p) begin
            count_usec5 = 0;
            scl = 0;
        end
        else if(scl_e)begin
            if(clk_usec_nedge)begin
                if(count_usec5 >= 4)begin
                    count_usec5 = 0;
                    scl = ~scl;
                end
                else count_usec5 = count_usec5 + 1;
            end
        end
        else if(!scl_e)begin
            count_usec5 = 0;
            scl = 1;
        end
    end

    wire [7:0] addr_rd_wr;                          // 주소를 보낼때 read인지 write인지 정보도 같이 보내야 하기 때문에 8비트짜리 정보로 만들어준다.
    assign addr_rd_wr = {addr, rd_wr};
    reg [2:0] cnt_bit;
    reg stop_flag;

    always @(posedge clk, posedge reset_p) begin
        if(reset_p) begin
            next_state = IDLE;
            cnt_bit = 0;
            scl_e = 0;
            sda = 1;
            cnt_bit = 7;
            stop_flag = 0;
        end
        else begin
            case (state)
                IDLE       : begin
                    scl_e = 0;
                    sda = 1;
                    if(comm_start_pedge)next_state = COMM_START;
                end     
                COMM_START : begin
                    sda = 0;
                    scl_e = 1;
                    next_state = SEND_ADDR;
                end 
                SEND_ADDR  : begin
                    if(scl_nedge)sda = addr_rd_wr[cnt_bit];
                    if(scl_pedge)begin
                        if(cnt_bit == 0)begin
                            cnt_bit = 7;
                            next_state = RD_ACK;
                        end
                        else cnt_bit = cnt_bit - 1;
                    end
                end 
                RD_ACK     : begin
                    if(scl_nedge) sda = 'bz;    // 출력 연결을 끊겠다.
                    else if(scl_pedge)begin
                        if(stop_flag) begin         // send data가 끝난후 stop으로 보냄
                            stop_flag = 0;
                            next_state = SCL_STOP;
                        end
                        else begin                  // 아직 data를 안보냈기 때문에 send_data로 보냄
                            stop_flag = 1; 
                            next_state = SEND_DATA;
                        end
                    end
                end     
                SEND_DATA  : begin
                    if(scl_nedge)sda = data[cnt_bit];
                    if(scl_pedge)begin
                        if(cnt_bit == 0)begin
                            cnt_bit = 7;
                            next_state = RD_ACK;
                        end
                        else cnt_bit = cnt_bit - 1;
                    end
                end 
                SCL_STOP   : begin
                    if(scl_nedge)sda = 0;
                    if(scl_pedge)next_state = COMM_STOP;
                end 
                COMM_STOP  : begin
                    if(count_usec5 >3) begin
                        scl_e = 0;
                        sda = 1;
                        next_state = IDLE;
                    end
                end
                default: next_state = IDLE;
            endcase
        end
    end


endmodule



module I2C_lcd_send_byte(
    input clk, reset_p,
    input [6:0] addr, 
    input [7:0] send_buffer,
    input send, rs,
    output scl, sda,
    output reg busy,
    output [15:0] led);

    localparam IDLE                     = 6'b00_0001;
    localparam SEND_HIGH_NIBBLE_DISABLE = 6'b00_0010;
    localparam SEND_HIGH_NIBBLE_ENABLE  = 6'b00_0100;
    localparam SEND_LOW_NIBBLE_DISABLE  = 6'b00_1000;
    localparam SEND_LOW_NIBBLE_ENABLE   = 6'b01_0000;
    localparam SEND_DISABLE             = 6'b10_0000;
    
    wire clk_usec_nedge;
    clock_div_100 us_clk(.clk(clk), .reset_p(reset_p),
        .nedge_div_100(clk_usec_nedge));
    
    reg [7:0] data;
    reg comm_start;
    
    wire send_pedge;
    edge_detector_p send_ed(
        .clk(clk), .reset_p(reset_p), .cp(send),
        .p_edge(send_pedge));
        
    reg [21:0] count_usec;
    reg count_usec_e;
    always @(negedge clk, posedge reset_p)begin
        if(reset_p)count_usec = 0;
        else if(clk_usec_nedge && count_usec_e)count_usec = count_usec + 1;
        else if(!count_usec_e)count_usec = 0;
    end    
    
    I2C_master_write_only master(clk, reset_p, addr, data, 1'b0, comm_start, scl, sda);
    
    reg [5:0] state, next_state;
    always @(negedge clk, posedge reset_p)begin
        if(reset_p)begin
            state = IDLE;
        end
        else begin
            state = next_state;
        end
    end
    
    always @(posedge clk, posedge reset_p)begin
        if(reset_p)begin
            next_state = IDLE;
            comm_start = 0;
            count_usec_e = 0;
            data = 0;
            busy = 0;
        end
        else begin
            case(state)
                IDLE                    :begin
                    if(send_pedge)begin
                        next_state = SEND_HIGH_NIBBLE_DISABLE;
                        busy = 1;
                    end
                end
                SEND_HIGH_NIBBLE_DISABLE:begin
                    if(count_usec <= 22'd200)begin
                                //d7 d6 d5 d4       BL en rw rs    
                        data = {send_buffer[7:4], 3'b100, rs};
                        comm_start = 1;
                        count_usec_e = 1; 
                    end
                    else begin
                        next_state = SEND_HIGH_NIBBLE_ENABLE;
                        count_usec_e = 0;
                        comm_start = 0;
                    end
                end
                SEND_HIGH_NIBBLE_ENABLE :begin
                    if(count_usec <= 22'd200)begin
                                //d7 d6 d5 d4       BL en rw rs    
                        data = {send_buffer[7:4], 3'b110, rs};
                        comm_start = 1;
                        count_usec_e = 1; 
                    end
                    else begin
                        next_state = SEND_LOW_NIBBLE_DISABLE;
                        count_usec_e = 0;
                        comm_start = 0;
                    end
                end
                SEND_LOW_NIBBLE_DISABLE :begin
                    if(count_usec <= 22'd200)begin
                                //d7 d6 d5 d4       BL en rw rs    
                        data = {send_buffer[3:0], 3'b100, rs};
                        comm_start = 1;
                        count_usec_e = 1; 
                    end
                    else begin
                        next_state = SEND_LOW_NIBBLE_ENABLE;
                        count_usec_e = 0;
                        comm_start = 0;
                    end
                end
                SEND_LOW_NIBBLE_ENABLE  :begin
                    if(count_usec <= 22'd200)begin
                                //d7 d6 d5 d4       BL en rw rs    
                        data = {send_buffer[3:0], 3'b110, rs};
                        comm_start = 1;
                        count_usec_e = 1; 
                    end
                    else begin
                        next_state = SEND_DISABLE;
                        count_usec_e = 0;
                        comm_start = 0;
                    end
                end
                SEND_DISABLE            :begin 
                    if(count_usec <= 22'd200)begin
                                //d7 d6 d5 d4       BL en rw rs    
                        data = {send_buffer[7:4], 3'b100, rs};
                        comm_start = 1;
                        count_usec_e = 1; 
                    end
                    else begin
                        next_state = IDLE;
                        count_usec_e = 0;
                        comm_start = 0;
                        busy = 0;
                    end
                end
            endcase
        end
    end
endmodule






module parking_db_shared(
    input  wire       clk,
    input  wire       reset_p,
    input  wire [15:0] time_now,

    input  wire        in_write_req,   
    input  wire [15:0] plate_in,     
    input  wire        out_search_req, 
    input  wire [15:0] plate_query,
    output reg  [15:0] lcd_number_out,   
    output reg  [15:0] parking_time_out, 
    output reg         data_valid_mem,     
    output reg         not_found_mem        
);

    reg        wr_en, rd_en;
    reg [9:0]  wr_addr, rd_addr;
    reg [31:0] i_data;
    wire [31:0] o_data;

    memory u_mem(
        .clk(clk),
        .rd_en(rd_en),
        .wr_en(wr_en),
        .wr_addr(wr_addr),
        .rd_addr(rd_addr),
        .i_data(i_data),
        .o_data(o_data)
    );


    reg [9:0]  wr_index;  
    reg [10:0] rec_count;
    reg [9:0]  srch_idx;
    reg        hit;
    reg [15:0] rd_time_tmp;


    localparam IDLE     = 3'd0;
    localparam W_SETUP  = 3'd1;
    localparam W_COMMIT = 3'd2;
    localparam S_INIT   = 3'd3;
    localparam RD_SETUP = 3'd4;
    localparam RD_WAIT  = 3'd5;
    localparam RD_LATCH = 3'd6;
    localparam S_DONE   = 3'd7;

    reg [2:0] state, next_state;


    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            state            <= IDLE;
            wr_en            <= 1'b0;
            rd_en            <= 1'b0;
            wr_addr          <= 10'd0;
            rd_addr          <= 10'd0;
            i_data           <= 32'd0;

            wr_index         <= 10'd0;
            rec_count        <= 11'd0;

            srch_idx         <= 10'd0;
            hit              <= 1'b0;
            rd_time_tmp      <= 16'd0;

            lcd_number_out   <= 16'd0;
            parking_time_out <= 16'd0;
            data_valid_mem       <= 1'b0;
            not_found_mem        <= 1'b0;
        end else begin
            state      <= next_state;

            wr_en      <= 1'b0;
            rd_en      <= 1'b0;
            data_valid_mem <= 1'b0;

            case (state)
                IDLE: begin

                end


                W_SETUP: begin
                    wr_addr <= wr_index;
                    i_data  <= {plate_in, time_now}; 
                end

                W_COMMIT: begin
                    wr_en  <= 1'b1;
                    if (wr_index  != 10'd1023) wr_index  <= wr_index + 10'd1;
                    if (rec_count != 11'd1024) rec_count <= rec_count + 11'd1;
                end


                S_INIT: begin
                    srch_idx       <= 10'd0;
                    hit            <= 1'b0;
                    not_found_mem      <= (rec_count == 0);
                    lcd_number_out <= plate_query;
                end

                RD_SETUP: begin
                    rd_addr <= srch_idx;
                    rd_en   <= 1'b1;
                    hit     <= 1'b0;
                end

                RD_WAIT: begin
                
                end

                RD_LATCH: begin
                    if (o_data[31:16] == plate_query) begin
                        hit         <= 1'b1;
                        rd_time_tmp <= o_data[15:0];
                        not_found_mem   <= 1'b0;
                    end else begin
                        hit      <= 1'b0;
                        srch_idx <= srch_idx + 10'd1;
                    end
                end

                S_DONE: begin
                    data_valid_mem <= 1'b1; 
                    if (hit) begin
                        parking_time_out <= time_now - rd_time_tmp;
                        not_found_mem        <= 1'b0;
                    end else begin
                        parking_time_out <= 16'd0;
                        not_found_mem        <= 1'b1;
                    end
                end
            endcase
        end
    end


    always @(*) begin
        next_state = state;
        case (state)
            IDLE: begin
                if (in_write_req)       next_state = W_SETUP;
                else if (out_search_req)next_state = S_INIT;
            end

            W_SETUP  : next_state = W_COMMIT;
            W_COMMIT : next_state = IDLE;

            S_INIT   : next_state = (rec_count == 0) ? S_DONE : RD_SETUP;
            RD_SETUP : next_state = RD_WAIT;
            RD_WAIT  : next_state = RD_LATCH;
            RD_LATCH : next_state = (hit) ? S_DONE
                                          : ((srch_idx + 10'd1 < rec_count) ? RD_SETUP : S_DONE);
            S_DONE   : next_state = IDLE;

            default  : next_state = IDLE;
        endcase
    end
endmodule

