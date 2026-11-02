library verilog;
use verilog.vl_types.all;
entity spectrum_core is
    port(
        I_clk           : in     vl_logic;
        I_rst           : in     vl_logic;
        I_sample_valid  : in     vl_logic;
        I_sample_left   : in     vl_logic_vector(23 downto 0);
        O_levels        : out    vl_logic_vector(191 downto 0)
    );
end spectrum_core;
