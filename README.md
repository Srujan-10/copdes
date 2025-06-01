```
module uart_apb_if (
    input  wire        pclk,
    input  wire        presetn,
    input  wire        psel,
    input  wire        penable,
    input  wire        pwrite,
    input  wire [7:0]  paddr,
    input  wire [31:0] pwdata,
    output reg  [31:0] prdata,
    output wire        pready,

    output reg  [7:0]  tx_data,
    output reg         tx_valid,
    input  wire        tx_ready,
    input  wire [7:0]  rx_data,
    input  wire        rx_valid,
    output reg         rx_ready,
    output reg  [15:0] baud_div,
    output reg         tx_irq,
    output reg         rx_irq
);

    localparam ADDR_TXDATA = 8'h00;
    localparam ADDR_RXDATA = 8'h04;
    localparam ADDR_STATUS = 8'h08;
    localparam ADDR_BAUD   = 8'h0C;

    assign pready = 1'b1;

    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            tx_valid <= 0;
            rx_ready <= 0;
            baud_div <= 16'd104; // 9600 baud default
            tx_irq   <= 0;
            rx_irq   <= 0;
        end else begin
            tx_valid <= 0;
            rx_ready <= 0;

            if (psel && penable) begin
                if (pwrite) begin
                    case (paddr)
                        ADDR_TXDATA: begin
                            tx_data <= pwdata[7:0];
                            tx_valid <= 1;
                        end
                        ADDR_BAUD: begin
                            baud_div <= pwdata[15:0];
                        end
                    endcase
                end else begin
                    case (paddr)
                        ADDR_RXDATA: begin
                            prdata <= {24'd0, rx_data};
                            rx_ready <= 1;
                        end
                        ADDR_STATUS: begin
                            prdata <= {30'd0, tx_ready, rx_valid};
                        end
                        ADDR_BAUD: begin
                            prdata <= {16'd0, baud_div};
                        end
                        default: prdata <= 32'd0;
                    endcase
                end
            end

            tx_irq <= ~tx_ready;
            rx_irq <= rx_valid;
        end
    end
endmodule

```
```
module uart_baud_gen (
    input  wire       clk,
    input  wire       resetn,
    input  wire [15:0] baud_div,
    output reg        tick
);

    reg [15:0] counter;

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            counter <= 0;
            tick <= 0;
        end else begin
            if (counter == baud_div) begin
                tick <= 1;
                counter <= 0;
            end else begin
                tick <= 0;
                counter <= counter + 1;
            end
        end
    end
endmodule

```
```
module uart_fifo #(
    parameter DATA_WIDTH = 8,
    parameter DEPTH = 16
)(
    input  wire                  clk,
    input  wire                  resetn,
    input  wire                  wr_en,
    input  wire                  rd_en,
    input  wire [DATA_WIDTH-1:0] din,
    output reg  [DATA_WIDTH-1:0] dout,
    output wire                  full,
    output wire                  empty,
    output wire [3:0]            count
);

    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];
    reg [3:0] w_ptr, r_ptr;
    reg [4:0] fifo_cnt;

    assign full  = (fifo_cnt == DEPTH);
    assign empty = (fifo_cnt == 0);
    assign count = fifo_cnt[3:0];

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            w_ptr <= 0;
            r_ptr <= 0;
            fifo_cnt <= 0;
        end else begin
            if (wr_en && !full) begin
                mem[w_ptr] <= din;
                w_ptr <= w_ptr + 1;
                fifo_cnt <= fifo_cnt + 1;
            end
            if (rd_en && !empty) begin
                dout <= mem[r_ptr];
                r_ptr <= r_ptr + 1;
                fifo_cnt <= fifo_cnt - 1;
            end
        end
    end
endmodule

```
```
module uart_rx (
    input  wire clk,
    input  wire resetn,
    input  wire tick,
    input  wire rx,
    output wire [7:0] data_out,
    output wire       valid,
    input  wire       ready
);

    reg [3:0] bit_cnt;
    reg [9:0] shift_reg;
    reg [2:0] state;
    reg       fifo_wr_en;

    wire fifo_full, fifo_empty;

    localparam IDLE = 0, START = 1, DATA = 2, STOP = 3;

    uart_fifo rx_fifo (
        .clk(clk),
        .resetn(resetn),
        .wr_en(fifo_wr_en),
        .rd_en(ready & ~fifo_empty),
        .din(shift_reg[8:1]),
        .dout(data_out),
        .full(fifo_full),
        .empty(fifo_empty),
        .count()
    );

    assign valid = ~fifo_empty;

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            state <= IDLE;
            fifo_wr_en <= 0;
        end else if (tick) begin
            fifo_wr_en <= 0;
            case (state)
                IDLE: if (!rx) state <= START;
                START: begin
                    bit_cnt <= 0;
                    shift_reg <= 0;
                    state <= DATA;
                end
                DATA: begin
                    shift_reg <= {rx, shift_reg[9:1]};
                    bit_cnt <= bit_cnt + 1;
                    if (bit_cnt == 7) state <= STOP;
                end
                STOP: begin
                    if (!fifo_full) fifo_wr_en <= 1;
                    state <= IDLE;
                end
            endcase
        end
    end
endmodule

```
