
module sd_card_sec_read_write
#(
	parameter  SPI_LOW_SPEED_DIV = 248,         // spi clk speed = clk speed /((SPI_LOW_SPEED_DIV + 2) * 2 )
	parameter  SPI_BOOT_SPEED_DIV = 123,        // 冷上电首次初始化用 400kHz（SD 规范），软复位仍走 LOW
	parameter  SPI_HIGH_SPEED_DIV = 0           // spi clk speed = clk speed /((SPI_HIGH_SPEED_DIV + 2) * 2 )
)
(
	input            clk,
	input            rst,
	output reg       sd_init_done,
	input            sd_sec_read,
	input[31:0]      sd_sec_read_addr,
	output[7:0]      sd_sec_read_data,
	output           sd_sec_read_data_valid,
	output           sd_sec_read_end,
	input            sd_sec_write,
	input[31:0]      sd_sec_write_addr,
	input[7:0]       sd_sec_write_data,
	output           sd_sec_write_data_req,
	output           sd_sec_write_end,

	output reg[15:0] spi_clk_div,
	output reg       cmd_req,
	input            cmd_req_ack,
	input            cmd_req_error,
	output reg[47:0] cmd,
	output reg[7:0]  cmd_r1,
	output reg[15:0] cmd_data_len,
	output reg       block_read_req,
	input            block_read_valid,
	input[7:0]       block_read_data,
	input            block_read_req_ack,
	output reg       block_write_req,
	output[7:0]      block_write_data,
	input            block_write_data_rd,
	input            block_write_req_ack,

	// 调试直播（数码管显示，定位卡死用）：
	//   dbg_state[4:0]  控制器状态机 state（0=IDLE 1=CMD0 2=CMD8 3=CMD55 4=CMD41
	//                   5=CMD17 6=READ 7=CMD24 8=WRITE E=ERR F=WR_END 10=RD_END
	//                   11=WAIT_RW 12=CMD16 13=INIT_WAIT 14=WD_ABORT）
	//   dbg_retry[5:0]  读失败计数 rd_retry（0..30）
	output wire [4:0] dbg_state,
	output wire [5:0] dbg_retry
);
reg[7:0] read_data;
reg[31:0] timer;

assign dbg_state = state;
assign dbg_retry = rd_retry;
// 读命令连续失败计数：超过阈值软复位（回 S_IDLE 重新初始化），
// 避免 SD 卡偶发无响应时无限重发导致整机卡死。
// 2026-10-26 调参：0.5s/15次 → 0.1s/30次——失败后快速重试让瞬态错误原地
// 恢复（手动切图首扇区读失败不再滑向软复位），总窗口仍 ~3 秒
reg [5:0] rd_retry;
// 初始化重试间隔计数器（CMD16 失败后等待，避免紧挨着重发轰炸卡）
reg [15:0] retry_wait;
// 读命令失败后的等待计数器（0.1 秒后重试，等卡忙完）
reg [25:0] rd_wait;

localparam S_IDLE               = 0;
localparam S_CMD0               = 1;
localparam S_CMD8               = 2;
localparam S_CMD55              = 3;
localparam S_CMD41              = 4;
localparam S_CMD17              = 5;
localparam S_READ               = 6;
localparam S_CMD24              = 7;
localparam S_WRITE              = 8;
localparam S_ERR                = 14;
localparam S_WRITE_END          = 15;
localparam S_READ_END           = 16;
localparam S_WAIT_READ_WRITE    = 17;
localparam S_CMD16              = 18;
localparam S_INIT_WAIT          = 19;   // CMD16 失败重试等待(原始语义,勿占用)

reg[4:0]                       state;
reg[31:0]                      sec_addr;
reg                            first_init;      // 冷上电首次初始化标志（走规范 400kHz）

// 初始化看门狗（2026-10-26）：初始化链（CMD0..CMD16）无总时限，命令层按字节
// 超时（0xFFFF 字节，400kHz 下单次 1.3 秒），卡无应答时会无限慢速重试——
// 开机表现为数码管全 0 卡死。超过 8 秒未完成初始化强制回 S_IDLE 重来：
// 此时 first_init 已清零，自动改用 1.9MHz（本板久经验证），最坏情况
// 从"永久卡死"变成"慢 8 秒后自愈"。

// 宽计数器"为零"标志打一拍寄存（2026-10-26 状态机时序加固）：
// 状态机下一态逻辑曾直接吃 26/16 位比较器输出，输入锥过深；寄存后 case 只吃 1 位标志。
reg rd_wait_z;        // rd_wait == 0
reg retry_wait_z;     // retry_wait == 0
assign sd_sec_read_data_valid = (state == S_READ) && block_read_valid;
assign sd_sec_read_data = block_read_data;
assign sd_sec_read_end = (state == S_READ_END);
assign sd_sec_write_data_req = (state == S_WRITE) && block_write_data_rd;
assign block_write_data = sd_sec_write_data;
assign sd_sec_write_end = (state == S_WRITE_END);

always@(posedge clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		state <= S_IDLE;
		cmd_req <= 1'b0;
		cmd_data_len <= 16'd0;
		cmd_r1 <= 8'd0;
		cmd <= 48'd0;
		spi_clk_div <= SPI_BOOT_SPEED_DIV[15:0];   // 冷上电：400kHz 规范速度
		first_init <= 1'b1;
		block_write_req <= 1'b0;
		block_read_req <= 1'b0;
			sec_addr <= 32'd0;
			sd_init_done <= 1'b0;
			rd_retry <= 6'd0;
			retry_wait <= 16'd0;
			rd_wait <= 26'd0;
	end
	else
		case(state)
			S_IDLE:
			begin
				state <= S_CMD0;
				sd_init_done <= 1'b0;
				// 首次（冷上电）用规范 400kHz；此后软复位都是热卡，用 1.9MHz 快恢复
				spi_clk_div <= first_init ? SPI_BOOT_SPEED_DIV[15:0] : SPI_LOW_SPEED_DIV[15:0];
				first_init <= 1'b0;
				rd_retry <= 6'd0;
			retry_wait <= 16'd0;
			rd_wait <= 26'd0;
			end
			S_CMD0:
			begin
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state <= S_CMD8;
					cmd_req <= 1'b0;
				end
				else
				begin
					cmd_req <= 1'b1;
					cmd_data_len <= 16'd0;
					cmd_r1 <= 8'h01;
					cmd <= {8'd0,8'h00,8'h00,8'h00,8'h00,8'h95};
				end
			end
			S_CMD8:
			begin
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state <= S_CMD55;
					cmd_req <= 1'b0;
				end
				else
				begin
					cmd_req <= 1'b1;
					cmd_data_len <= 16'd4;
					cmd_r1 <= 8'h01;
					cmd <= {8'd8,8'h00,8'h00,8'h01,8'haa,8'h87};
				end
			end
			S_CMD55:
			begin
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state <= S_CMD41;
					cmd_req <= 1'b0;
				end
				else
				begin
					cmd_req <= 1'b1;
					cmd_data_len <= 16'd0;
					cmd_r1 <= 8'h01;
					cmd <= {8'd55,8'h00,8'h00,8'h00,8'h00,8'hff};
				end
			end
			S_CMD41:
			begin
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state <= S_CMD16;
					cmd_req <= 1'b0;
					sd_init_done <= 1'b1;
					spi_clk_div <= SPI_HIGH_SPEED_DIV[15:0];
				end
				else if(cmd_req_ack)
				begin
					state <= S_CMD55;
				end
				else
				begin
					cmd_req <= 1'b1;
					cmd_data_len <= 16'd0;
					cmd_r1 <= 8'h00;
					cmd <= {8'd41,8'h40,8'h00,8'h00,8'h00,8'hff};
				end
			end
			S_CMD16:
			begin
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state <= S_WAIT_READ_WRITE;
					cmd_req <= 1'b0;
					sd_init_done <= 1'b1;
					spi_clk_div <= SPI_HIGH_SPEED_DIV[15:0];
				end
				else if(cmd_req_ack)
				begin
					// 失败：等 0.5ms 再重新握手，避免紧挨着轰炸未就绪的卡
					retry_wait <= 16'd50000;
					state      <= S_INIT_WAIT;
					cmd_req    <= 1'b0;
				end
				else
				begin
					cmd_req <= 1'b1;
					cmd_data_len <= 16'd0;
					cmd_r1 <= 8'h00;
					cmd <= {8'd16,32'd512,8'hff};
				end
			end
			S_INIT_WAIT:
			begin
				if(!retry_wait_z)
					retry_wait <= retry_wait - 16'd1;
				else
					state <= S_CMD55;
			end			
			S_WAIT_READ_WRITE:
			begin
				if(sd_sec_write ==  1'b1)
				begin
					state <= S_CMD24;
					sec_addr <= sd_sec_write_addr;
				end
				else if(sd_sec_read == 1'b1)
				begin
					state <= S_CMD17;
					sec_addr <= sd_sec_read_addr;
				end

				// 2026-10-26 修复:原硬编码 16'd0=25MHz 覆盖了 CMD41/CMD16 设置的
				// 12.5MHz,数据读取一直以历史易错速度运行(大图长读突发成簇失败根因)
				spi_clk_div <= SPI_HIGH_SPEED_DIV[15:0];
			end
			S_CMD24:
			begin
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state <= S_WRITE;
					cmd_req <= 1'b0;
				end
				else
				begin
					cmd_req <= 1'b1;
					cmd_data_len <= 16'd0;
					cmd_r1 <= 8'h00;
					cmd <= {8'd24,sec_addr,8'hff};

				end
			end
			S_WRITE:
			begin
				if(block_write_req_ack == 1'b1)
				begin
					block_write_req <= 1'b0;
					state <= S_WRITE_END;
				end
				else
					block_write_req <= 1'b1;
			end
			S_CMD17:
			begin
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state <= S_READ;
					cmd_req <= 1'b0;
					rd_retry <= 6'd0;
					rd_wait  <= 26'd0;
				end
				else if(cmd_req_ack)
				begin
					// 读命令失败：等 0.5 秒再重试（卡忙时耐心等它恢复），
					// 连续 30 次失败（约 3 秒）才软复位
					if(rd_retry >= 6'd30)
					begin
						state <= S_IDLE;
						cmd_req <= 1'b0;
					end
					else
					begin
						rd_retry <= rd_retry + 6'd1;
						cmd_req  <= 1'b0;               // 停止轰炸
						rd_wait  <= 26'd10_000_000;     // 0.1 秒等待
					end
				end
				else if(!rd_wait_z)
				begin
					rd_wait <= rd_wait - 26'd1;         // 等待期
					cmd_req <= 1'b0;
				end
				else
				begin
					cmd_req <= 1'b1;
					cmd_data_len <= 16'd0;
					cmd_r1 <= 8'h00;
					cmd <= {8'd17,sec_addr,8'hff};
				end
			end
			S_READ:
			begin
				if(block_read_req_ack)
				begin
					state <= S_READ_END;
					block_read_req <= 1'b0;
				end
				else
				begin
					block_read_req <= 1'b1;
				end
			end
			S_WRITE_END:
			begin
				state <= S_WAIT_READ_WRITE;
			end
			S_READ_END:
			begin
				state <= S_WAIT_READ_WRITE;
			end
			default:
				state <= S_IDLE;
		endcase
end

// 宽计数器"为零"标志更新（独立 always，2026-10-26 状态机时序加固；
// 初始化看门狗已移除，此处仅维护两个 1 位标志）
always @(posedge clk or posedge rst) begin
	if (rst) begin
		rd_wait_z     <= 1'b1;
		retry_wait_z  <= 1'b1;
	end
	else begin
		rd_wait_z    <= (rd_wait    == 26'd0);
		retry_wait_z <= (retry_wait == 16'd0);
	end
end
endmodule