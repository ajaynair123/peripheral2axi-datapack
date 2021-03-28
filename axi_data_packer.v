module axi_data_packer (
input                          clk,
input                          rst_n,
input                          b_empty,      //burst_fifo empty status
input                          data_empty,   //data_fifo empty status
input [31:0]                   burst_addr,	//we need burst address
input [11:0]				   burst_info,	//from burst fifo
input [35:0]			       wdata_pckt,	//from data fifo
input                          rdy_from_slv,	//from slave


output reg                         axi_wvalid,	//to slave
output reg [3:0]                   axi_wstrb,		//to slave
output reg [31:0]                  axi_wdata,		//to slave
output reg                         axi_wlast,		//to slave
output reg 						 data_fifo_pop	//pop to data fifo
);

parameter IDLE = 'd0;
parameter DATA_POP = 'd1;
parameter CNT_UPDATE = 'd2;
parameter DATA_ADJUST = 'd3;
parameter CHECK_BYTE_CNTR = 'd4;
parameter VALID_RDY = 'd5;
parameter LATCH = 'd6;

reg [2:0] crnt_st, nxt_st;
reg [3:0] crnt_byte_en, crnt_byte_en_reg;
reg [31:0] crnt_wdata, crnt_wdata_reg;
reg [7:0] wbyte_cntr, wbyte_cntr_reg;
reg [1:0] wstrb_cntr, wstrb_cntr_reg;		//For narrow transfers
reg [5:0] wbyte_en_dec;
reg [31:0] data_store, data_store_reg; 	//Packed to-send out data
reg [31:0] data_latch, data_latch_reg;	//Latch remaining popped data
reg [7:0] prev_byte_cntr, prev_byte_cntr_reg;
reg [7:0] beat_cntr, beat_cntr_reg;		
reg axi_wvalid_reg;
reg [2:0] first_beat_size;				//First beat unaligned check
reg data_fifo_pop_reg, data_fifo_pop_reg_d;
reg latch_valid, latch_valid_reg;

wire [3:0] burst_size;
wire [7:0] burst_len;

wire [3:0] actual_burst_size;
wire [6:0] burst_size_bits;
wire [7:0] crnt_rem_bits;
wire [7:0] burst_rem_bits;
wire [7:0] wbyte_cntr_bits;
wire [7:0] prev_byte_cntr_bits;
wire first_beat;
wire second_beat; 
wire last_beat;

assign actual_burst_size = 1<<(burst_info[3:0]);
assign burst_size = first_beat ? first_beat_size : actual_burst_size;
assign burst_len  = burst_info[11:4];

assign burst_size_bits = burst_size << 3;
assign crnt_rem_bits = (prev_byte_cntr_reg - wbyte_cntr_reg) << 3;
assign burst_rem_bits = (burst_size - prev_byte_cntr_reg) << 3;
assign wbyte_cntr_bits = wbyte_cntr_reg << 3;
assign prev_byte_cntr_bits = prev_byte_cntr_reg << 3;
assign first_beat = (beat_cntr_reg==1);
assign second_beat = (beat_cntr_reg==2);
assign last_beat = (beat_cntr_reg == (burst_len+1));

integer i;

always @ (posedge clk or negedge rst_n) 
begin
	if(~rst_n)
	begin
	crnt_st <= 3'b0;
	crnt_wdata_reg <= 32'd0;
	crnt_byte_en_reg <= 4'd0;
	wbyte_cntr_reg <= 8'd0;
	data_store_reg <= 32'd0;
	data_latch_reg <= 32'd0;
	wstrb_cntr_reg <= 2'd0;
	beat_cntr_reg <= 8'd0;
	axi_wvalid_reg <= 1'b0;
	prev_byte_cntr_reg <= 8'd0;
	data_fifo_pop_reg <= 1'b0;
	latch_valid_reg <= 1'b0;
	end
    else
	begin
    crnt_st <= nxt_st;
	crnt_wdata_reg <= crnt_wdata;
	crnt_byte_en_reg <= crnt_byte_en;	
	wbyte_cntr_reg <= wbyte_cntr;
	data_store_reg <= data_store;
	data_latch_reg <= data_latch;
	wstrb_cntr_reg <= wstrb_cntr;
	beat_cntr_reg <= beat_cntr;
	axi_wvalid_reg <= axi_wvalid;
	prev_byte_cntr_reg <= prev_byte_cntr; 
	data_fifo_pop_reg <= data_fifo_pop;
	latch_valid_reg <= latch_valid;
	end
end

always @ *
begin
	case(crnt_byte_en)
	4'h1,4'h2,4'h4,4'h8 : wbyte_en_dec = 6'd8;
	4'h3,4'h6,4'hC : wbyte_en_dec = 6'd16;
	4'h7,4'hE : wbyte_en_dec = 6'd24;
	4'hF : wbyte_en_dec = 6'd32;
	default : wbyte_en_dec = 6'd32;
	endcase
	
	case(actual_burst_size)	//Unaligned,narrow data and strobe
	4'h1:
		case(wstrb_cntr_reg)
		2'b00: begin axi_wstrb = 4'h1; axi_wdata = {24'd0,data_store_reg[7:0]}; end
		2'b01: begin axi_wstrb = 4'h2; axi_wdata = {16'd0,data_store_reg[7:0],8'd0}; end
		2'b10: begin axi_wstrb = 4'h4; axi_wdata = {8'd0,data_store_reg[7:0],16'd0}; end
		2'b11: begin axi_wstrb = 4'h8; axi_wdata = {data_store_reg[7:0],24'd0}; end
		endcase
	4'h2:
		case(wstrb_cntr_reg)
		2'b00: begin axi_wstrb = 4'h3; axi_wdata = {16'd0,data_store_reg[15:0]}; end
		2'b10: begin axi_wstrb = 4'hC; axi_wdata = {data_store_reg[15:0],16'd0}; end
		2'b11: begin axi_wstrb = 4'h1; axi_wdata = {data_store_reg[7:0],24'd0}; end
		endcase
	4'h4:
		case(wstrb_cntr_reg)
		2'b00: begin axi_wstrb = 4'hF; axi_wdata = data_store_reg; end
		2'b01: begin axi_wstrb = 4'hE; axi_wdata = {data_store_reg[23:0],8'd0}; end
		2'b10: begin axi_wstrb = 4'hC; axi_wdata = {data_store_reg[15:0],16'd0}; end
		2'b11: begin axi_wstrb = 4'h8; axi_wdata = {data_store_reg[7:0],24'd0}; end
		endcase
	endcase
	
	case(burst_addr[1:0])		//Unaligned address
	2'b00: first_beat_size = actual_burst_size[0] ? 3'd1 : actual_burst_size[1] ? 3'd2 : 3'd4;
	2'b01: first_beat_size = actual_burst_size[0] ? 3'd1 : actual_burst_size[1] ? 3'd2 : 3'd3;
	2'b10: first_beat_size = actual_burst_size[0] ? 3'd1 : actual_burst_size[1] ? 3'd2 : 3'd2;
	2'b11: first_beat_size = actual_burst_size[0] ? 3'd1 : actual_burst_size[1] ? 3'd1 : 3'd1;
	endcase
end

always @ *
begin
//#1;
	nxt_st = crnt_st;
    crnt_wdata = crnt_wdata_reg;
    crnt_byte_en = crnt_byte_en_reg;
    wbyte_cntr = wbyte_cntr_reg;
    data_store = data_store_reg;
    data_latch = data_latch_reg;
    wstrb_cntr = wstrb_cntr_reg;
    beat_cntr = beat_cntr_reg;
    axi_wvalid = axi_wvalid_reg;
    prev_byte_cntr = prev_byte_cntr_reg;
	data_fifo_pop = data_fifo_pop_reg;
	latch_valid = latch_valid_reg;
	axi_wlast = 1'b0;
	case(crnt_st)
		IDLE:
		begin 
			if({b_empty} == 1'b0) 
			begin //give pop from cmd_intf w/ required delay to burst_fifo
			wstrb_cntr = burst_addr[1:0];
			nxt_st =  DATA_POP;
		    end
		    else  
			begin
		    	nxt_st = IDLE;
            end
		end
		
		DATA_POP:
		begin
			if ( data_empty == 1'b1)
			begin
        	axi_wvalid = 1'b0;
        	nxt_st = DATA_POP;
			end
			else 
			begin	
         	data_fifo_pop = (data_fifo_pop_reg) ? 1'b0 : 1'b1;
			nxt_st = (data_fifo_pop_reg) ? CNT_UPDATE : DATA_POP;
			end
		end

		CNT_UPDATE:
		begin
			crnt_wdata    =  wdata_pckt[35 : 4];	//from data fifo
            crnt_byte_en  =  wdata_pckt[3 : 0];
			beat_cntr = (!(|beat_cntr_reg)) ? 8'd1 : beat_cntr_reg;
			wbyte_cntr = wbyte_cntr_reg + (wbyte_en_dec>>3);
			prev_byte_cntr = wbyte_cntr_reg;
			nxt_st = DATA_ADJUST;
		end
		
		DATA_ADJUST:	//to make valid data from LSByte position
		begin
			crnt_wdata = crnt_byte_en_reg[0] ? crnt_wdata_reg : crnt_wdata_reg >> 8;
			crnt_byte_en = crnt_byte_en_reg[0] ? crnt_byte_en_reg : crnt_byte_en_reg >> 1;
			nxt_st = crnt_byte_en_reg[0] ? CHECK_BYTE_CNTR : crnt_st;
		end
		
		CHECK_BYTE_CNTR:
		begin
			if (wbyte_cntr_reg < burst_size) 
			begin
				if ((prev_byte_cntr_reg < burst_size) & !latch_valid_reg)		//Maintain previous store data, store incoming data at MSBytes
				begin
				//data_store = {crnt_wdata_reg [wbyte_en_dec-1:0] , data_store_reg };
				for(i=0;i<wbyte_en_dec;i=i+1)
				data_store[i+prev_byte_cntr_bits] = crnt_wdata_reg[i];
				data_latch = data_latch_reg;
				nxt_st = DATA_POP;
				end
				else															//Maintain store and latch data
				begin
				data_store = data_store_reg;
				data_latch = data_latch_reg;
				latch_valid = 1'b0;
				nxt_st = DATA_POP;
				end
			end
			else
			begin
				if (prev_byte_cntr_reg <= burst_size)							//Split incoming data into store and latch
				begin
				//data_store = {crnt_wdata_reg [burst_rem_bits-1:0] , data_store_reg };
				for(i=0;i<burst_rem_bits;i=i+1)
				data_store[i+prev_byte_cntr_bits] = crnt_wdata_reg[i];
				
				//data_latch = crnt_wdata_reg [wbyte_en_dec-1:burst_rem_bits];
				for(i=burst_rem_bits;i<wbyte_en_dec;i=i+1)
				data_latch[i-burst_rem_bits] = crnt_wdata_reg[i];
				nxt_st = VALID_RDY;
				end
				else															//Might have to latch
				begin
				data_store = data_store_reg;
				data_latch = data_latch_reg;
				latch_valid = 1'b0;
				nxt_st = VALID_RDY;
				end
			end
		end
		
		VALID_RDY:
		begin
			wstrb_cntr = wstrb_cntr_reg;
			axi_wvalid = 1'b1;
			axi_wlast = last_beat ? 1'b1 : 1'b0;
     		if( rdy_from_slv == 1'b0) 
			begin 
     		nxt_st = VALID_RDY;
     		end
     		else if( axi_wvalid_reg && rdy_from_slv == 1'b1 ) 
			begin
				if(last_beat)
				begin
				nxt_st = IDLE;
				axi_wvalid = 1'b0;
				axi_wlast = 1'b0;
				beat_cntr = 8'd0;
				wbyte_cntr = 8'd0;
				wstrb_cntr = 2'd0;
				end
				else
				begin
				nxt_st = LATCH;
				axi_wvalid = 1'b0;
				wbyte_cntr = wbyte_cntr_reg - burst_size;
				prev_byte_cntr = wbyte_cntr_reg;
				wstrb_cntr = wstrb_cntr_reg + burst_size;
				beat_cntr = beat_cntr_reg + 1;
				end
     		end
			else
			begin
			nxt_st = VALID_RDY;
			end
		end
		
		LATCH:
		begin
			latch_valid = 1'b1;
			if(wbyte_cntr_reg <= burst_size)
			begin
			//data_store = {data_store_reg [31-wbyte_cntr_bits:0] , data_latch_reg [wbyte_cntr_bits-1:0]};
			for(i=0;i<wbyte_cntr_bits;i=i+1)
			data_store[i] = data_latch_reg[i];
			for(i=wbyte_cntr_bits;i<32;i=i+1)
			data_store[i] = data_store_reg[i-wbyte_cntr_bits];
			
			data_latch = data_latch_reg >> wbyte_cntr_bits;		//Remove placed info
			nxt_st  = CHECK_BYTE_CNTR;
			end
			else
			begin
			//data_store = {data_store_reg [31-burst_size_bits:0] , data_latch_reg [burst_size_bits-1:0]};
			for(i=0;i<burst_size_bits;i=i+1)
			data_store[i] = data_latch_reg[i];
			for(i=burst_size_bits;i<32;i=i+1)
			data_store[i] = data_store_reg[i-burst_size_bits];
			
			data_latch = data_latch_reg >> burst_size_bits;		//Remove placed info and keep next info ready
			nxt_st  = CHECK_BYTE_CNTR;
			end
		end
	
	endcase
end
endmodule
