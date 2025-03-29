module TwoCyclePulse (
    input wire clk,
    input wire rst,
    input wire trigger,   // goes high for one cycle to start the 2-cycle pulse
    output reg enOut
);
    reg [1:0] counter;
    reg active;

    always @(posedge clk) begin
        if (~rst) begin
            counter <= 0;
            active <= 0;
            enOut <= 0;
        end else begin
            if (trigger && !active) begin
                active <= 1;
                counter <= 2;   // hold for two cycles
                enOut <= 1;
            end else if (active) begin
                if (counter > 1) begin
                    counter <= counter - 1;
                end else begin
                    enOut <= 0;
                    active <= 0;
                    counter <= 0;
                end
            end
        end
    end
endmodule