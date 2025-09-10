`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/28/2025 04:00:59 PM
// Design Name: 
// Module Name: clock_lib
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


module clock_div_100(
    input clk,reset_p,
    output reg clk_div_100,
    output pedge_div_100, nedge_div_100
    );

    reg [5:0] cnt_sysclk;

    always @(posedge clk, posedge reset_p) begin
        if(reset_p)begin
            cnt_sysclk = 0;
            clk_div_100 = 0;
        end
        else begin
            if(cnt_sysclk >= 49)begin
                cnt_sysclk = 0;
                clk_div_100 = ~clk_div_100;
            end
            else cnt_sysclk = cnt_sysclk + 1;
        end
    end

    edge_detector_p btn_ed(
        .clk(clk), .reset_p(reset_p), .cp(clk_div_100), .p_edge(pedge_div_100), .n_edge(nedge_div_100)
    );

endmodule

module time_counter16 #(
    parameter integer CLK_FREQ_HZ = 100_000_000 
)(
    input  wire       clk,
    input  wire       reset_p,
    output reg [15:0] time_now 
);
    reg [31:0] sec_div;
    wire sec_tick = (sec_div == CLK_FREQ_HZ-1);

    reg [5:0] sec_cnt;
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            sec_div  <= 32'd0;
            sec_cnt  <= 6'd0;
            time_now <= 16'd0;
        end else begin
            if (sec_div == CLK_FREQ_HZ-1) sec_div <= 32'd0;
            else                          sec_div <= sec_div + 32'd1;

            if (sec_tick) begin
                if (sec_cnt == 6'd59) begin
                    sec_cnt  <= 6'd0;
                    time_now <= time_now + 16'd1; 
                end else begin
                    sec_cnt <= sec_cnt + 6'd1;
                end
            end
        end
    end
endmodule