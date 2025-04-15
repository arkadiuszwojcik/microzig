///
/// based on: https://github.com/embassy-rs/embassy/blob/main/cyw43-pio/src/lib.rs
///

const std = @import("std");
const microzig = @import("microzig");
const SPI_Device = microzig.hal.drivers.SPI_Device;
const GPIO_Device = microzig.hal.drivers.GPIO_Device;
const wifi = microzig.drivers.wireless.cyw43;

const rp2xxx = microzig.hal;
const time = rp2xxx.time;
const gpio = rp2xxx.gpio;

const uart = rp2xxx.uart.instance.num(0);
const baud_rate = 115200;
const uart_tx_pin = gpio.num(0);

// These may change depending on which GPIO pins you have your SPI device routed to.
const CS_PIN = 17;
const SCK_PIN = 18;
// NOTE: rp2xxx doesn't label pins as MOSI/MISO. Instead a pin is always for
// either receiving or transmitting SPI data, no matter whether the chip is in
// master or slave mode.
const TX_PIN = 19;

pub fn sleep(time_ms: u32) void {
    microzig.hal.time.sleep_ms(time_ms);
}

pub const microzig_options = microzig.Options{
    .log_level = .debug,
    .logFn = rp2xxx.uart.logFn,
};

// Communicate with another RP2040 over spi
pub fn main() !void {
    // Set pin functions for CS, SCK, RX
    //const csn = rp2xxx.gpio.num(29);
    //const tx = rp2xxx.gpio.num(24);
    //const rx = rp2xxx.gpio.num(24);
    //const sck = rp2xxx.gpio.num(25);
    //inline for (&.{ csn, tx, rx, sck }) |pin| {
    //    pin.set_function(.spi);
    //}

    uart_tx_pin.set_function(.uart);

    uart.apply(.{
        .baud_rate = baud_rate,
        .clock_config = rp2xxx.clock_config,
    });

    rp2xxx.uart.init_logger(uart);

    // STEP1: PWR and CS PINs initalization
    // Inicjalizacja pinow: https://github.com/embassy-rs/embassy/blob/main/examples/rp235x/src/bin/blinky_wifi.rs
    // Uwaga1: Pod innym linkiem inicjalizuja pull pina na .up: https://github.com/raspberrypi/pico-sdk/blob/ee68c78d0afae2b69c03ae1a72bf5cc267a2d94c/src/rp2_common/pico_cyw43_driver/cyw43_bus_pio_spi.c#L343
    // Uwaga2: Tutaj ustawiaja jeszcze pozostale piny ale to stary przyklad: https://github.com/embassy-rs/cyw43/blob/e33b99e9ec9902d6f93582530fd9cfe38953ce69/examples/rpi-pico-w/src/main.rs
    const pwr_pin = rp2xxx.gpio.num(23);
    pwr_pin.set_function(.sio);        // ustawienie funkcji ustawia IE na true (kod w gpio)
    pwr_pin.put(0);
    pwr_pin.set_direction(.out);

    const cs_pin = rp2xxx.gpio.num(25);
    cs_pin.set_function(.sio);         // ustawienie funkcji ustawia IE na true (kod w gpio)
    cs_pin.put(1);
    cs_pin.set_direction(.out);

    var spi_cyw43: Cyw43PioSpi = .{};
    spi_cyw43.init();

    // From: https://github.com/raspberrypi/pico-sdk/blob/ee68c78d0afae2b69c03ae1a72bf5cc267a2d94c/src/rp2_common/pico_cyw43_driver/cyw43_bus_pio_spi.c#L343 line 344
    //var power_pin: GPIO_Device = GPIO_Device.init(pwr_pin);
    //var wifi_bus = wifi.Bus.init(power_pin.digital_io(), spi_cyw43.spi_bus_cyw43(), sleep);

    var i: u32 = 0;
    while (true) : (i += 1) {
        std.log.info("what {}", .{i});
        time.sleep_ms(500);

        time.sleep_ms(500);

        time.sleep_ms(8000);

        //try wifi_bus.init_bus();

        pwr_pin.put(0);
        microzig.hal.time.sleep_ms(50);
        pwr_pin.put(1);
        microzig.hal.time.sleep_ms(250);
        microzig.hal.time.sleep_ms(250);

        const cmd = CYW43Cmd { .cmd = .read, .incr = .incremental, .func = .bus, .addr = 0x14, .len = 4 };
        const cmd_swapped = swap16(@bitCast(cmd));

        var buff = [1]u32{0};
        const op_stat = spi_cyw43.cmd_read2(cmd_swapped, &buff);
        std.log.info("cmd_read status: 0x{X} and buf: 0x{X}", .{op_stat, buff[0]});
    }
}

fn swap16(x: u32) u32 {
    return x << 16 | x >> 16;
}

const Cyw43PioSpi = struct {
    const Self = @This();
    pio: rp2xxx.pio.Pio = rp2xxx.pio.num(0),
    sm: rp2xxx.pio.StateMachine = .sm0,
    io_pin: rp2xxx.gpio.Pin = rp2xxx.gpio.num(24),
    clk_pin: rp2xxx.gpio.Pin = rp2xxx.gpio.num(29),
    cs_pin: rp2xxx.gpio.Pin = rp2xxx.gpio.num(25),

    const cw49spi_program = blk: {
        @setEvalBranchQuota(5000);
        break :blk rp2xxx.pio.assemble(
            \\.program cw49spi
            \\.side_set 1
            \\
            \\.wrap_target
            \\
            \\; write out x-1 bits
            \\lp:
            \\out pins, 1    side 0
            \\jmp x-- lp     side 1
            \\
            \\; switch directions
            \\set pindirs, 0 side 0
            \\nop            side 0
            \\
            \\; read in y-1 bits
            \\lp2:
            \\in pins, 1     side 1
            \\jmp y-- lp2    side 0
            \\
            \\; wait for event and irq host
            \\wait 1 pin 0   side 0
            \\irq 0          side 0
            \\
            \\.wrap
        , .{}).get_program_by_name("cw49spi");
    };

    pub fn init(this: *Self) void {
        // make pio pin
        //pin.pad_ctrl().write(|w| w.set_od(false));  tego brakuje
        //pin.pad_ctrl().write(|w| w.set_ie(true));   tego brakuje

        // TODO: are we missing direction set here?
        this.io_pin.set_function(.pio0);
        this.io_pin.set_output_disabled(false);
        this.io_pin.set_pull(.disabled);
        this.io_pin.set_schmitt_trigger(.enabled);

        //pin_io.set_input_sync_bypass(true);
        const mask = @as(u32, 1) << @truncate(@intFromEnum(this.io_pin));
        var val = this.pio.get_regs().INPUT_SYNC_BYPASS.raw;
        val |= mask;
        this.pio.get_regs().INPUT_SYNC_BYPASS.write_raw(val);

        this.io_pin.set_drive_strength(.@"12mA");
        this.io_pin.set_slew_rate(.fast);

        // make pio pin
        //pin.pad_ctrl().write(|w| w.set_od(false));  tego brakuje ale chyba nasz set_function to robi
        //pin.pad_ctrl().write(|w| w.set_ie(true));   tego brakuje

        // TODO: are we missing direction set here?
        this.clk_pin.set_function(.pio0);
        this.clk_pin.set_output_disabled(false);
        this.clk_pin.set_drive_strength(.@"12mA");
        this.clk_pin.set_slew_rate(.fast);

        this.pio.sm_load_and_start_program(this.sm, cw49spi_program, .{
            .clkdiv = .{ .int = 2 },
            .pin_mappings = .{
                .out = .{ .base = @truncate(@intFromEnum(this.io_pin)), .count = 1 },           // HACK!! but why??? with truncate
                .set = .{ .base = @truncate(@intFromEnum(this.io_pin)), .count = 1 },           // HACK!! but why??? with truncate
                .side_set = .{ .base = @truncate(@intFromEnum(this.clk_pin)), .count = 1 },     // HACK!! but why??? with truncate
                .in_base = @truncate(@intFromEnum(this.io_pin)),                                // HACK!! but why??? with truncate
            },
            .shift = .{
                .out_shiftdir = .left,
                .in_shiftdir= .left,
                .autopull = true,
                .autopush = true,
            },
        }) catch unreachable;

        this.pio.sm_set_pindir(this.sm, @truncate(@intFromEnum(this.clk_pin)), 1, .out);
        this.pio.sm_set_pindir(this.sm, @truncate(@intFromEnum(this.io_pin)), 1, .out);

        this.pio.sm_set_pin(this.sm, @truncate(@intFromEnum(this.clk_pin)), 1, 0);
        this.pio.sm_set_pin(this.sm, @truncate(@intFromEnum(this.io_pin)), 1, 0);
    }

    pub fn cmd_read(selfopaque: *anyopaque, cmd: u32, buffer: []u32) u32 {
        const this: *Self = @ptrCast(@alignCast(selfopaque));
        this.cs_pin.put(0);
        this.pio.sm_set_enabled(this.sm, false);

        const write_bits = 31;
        const read_bits = buffer.len * 32 + 32 - 1;

        this.pio.sm_exec_set_y(this.sm, read_bits);
        this.pio.sm_exec_set_x(this.sm, write_bits);
        this.pio.sm_exec_set_pindir(this.sm, 0b1);
        this.pio.sm_exec_jmp(this.sm, cw49spi_program.wrap_target.?);

        this.pio.sm_set_enabled(this.sm, true);

        const dma = rp2xxx.dma.channel(2);

        const cmd_data = std.mem.asBytes(&cmd);
        dma.trigger_transfer(@intFromPtr(this.pio.sm_get_tx_fifo(this.sm)), @intFromPtr(cmd_data.ptr), 1, .{ .data_size = .size_32, .enable = true, .read_increment = true, .write_increment = false, .dreq = @enumFromInt(@intFromEnum(this.pio) * @as(u6, 8) + @intFromEnum(this.sm)) });
        while (dma.is_busy()) {}
        dma.trigger_transfer(@intFromPtr(this.pio.sm_get_rx_fifo(this.sm)), @intFromPtr(buffer.ptr), buffer.len, .{ .data_size = .size_32, .enable = true, .read_increment = false, .write_increment = true, .dreq = @enumFromInt(@intFromEnum(this.pio) * @as(u6, 8) + @intFromEnum(this.sm) + 4) });
        while (dma.is_busy()) {}
        var status: u32 = 0;
        const status_data = std.mem.asBytes(&status);
        dma.trigger_transfer(@intFromPtr(this.pio.sm_get_rx_fifo(this.sm)), @intFromPtr(status_data.ptr), 1, .{ .data_size = .size_32, .enable = true, .read_increment = false, .write_increment = true, .dreq = @enumFromInt(@intFromEnum(this.pio) * @as(u6, 8) + @intFromEnum(this.sm) + 4) });
        while (dma.is_busy()) {}

        this.cs_pin.put(1);
        return status;
    }

    pub fn cmd_read2(this: *Self, cmd: u32, buffer: []u32) u32 {
        //const this: *Self = @ptrCast(@alignCast(selfopaque));
        this.cs_pin.put(0);
        this.pio.sm_set_enabled(this.sm, false);

        const write_bits = 31;
        const read_bits = buffer.len * 32 + 32 - 1;

        this.pio.sm_exec_set_y(this.sm, read_bits);
        this.pio.sm_exec_set_x(this.sm, write_bits);
        this.pio.sm_exec_set_pindir(this.sm, 0b1);
        this.pio.sm_exec_jmp(this.sm, cw49spi_program.wrap_target.?);

        this.pio.sm_set_enabled(this.sm, true);

        const dma = rp2xxx.dma.channel(2);

        const cmd_data = std.mem.asBytes(&cmd);
        dma.trigger_transfer(@intFromPtr(this.pio.sm_get_tx_fifo(this.sm)), @intFromPtr(cmd_data.ptr), 1, .{ .data_size = .size_32, .enable = true, .read_increment = true, .write_increment = false, .dreq = @enumFromInt(@intFromEnum(this.pio) * @as(u6, 8) + @intFromEnum(this.sm)) });
        while (dma.is_busy()) {}
        dma.trigger_transfer(@intFromPtr(this.pio.sm_get_rx_fifo(this.sm)), @intFromPtr(buffer.ptr), buffer.len, .{ .data_size = .size_32, .enable = true, .read_increment = false, .write_increment = true, .dreq = @enumFromInt(@intFromEnum(this.pio) * @as(u6, 8) + @intFromEnum(this.sm) + 4) });
        while (dma.is_busy()) {}
        var status: u32 = 0;
        const status_data = std.mem.asBytes(&status);
        dma.trigger_transfer(@intFromPtr(this.pio.sm_get_rx_fifo(this.sm)), @intFromPtr(status_data.ptr), 1, .{ .data_size = .size_32, .enable = true, .read_increment = false, .write_increment = true, .dreq = @enumFromInt(@intFromEnum(this.pio) * @as(u6, 8) + @intFromEnum(this.sm) + 4) });
        while (dma.is_busy()) {}

        this.cs_pin.put(1);
        return status;
    }

    fn cmd_write(_: *anyopaque, _: []const u32) u32 {
        return 0;
    }

    pub fn spi_bus_cyw43(spi: *Self) wifi.SpiBusCyw43 {
        return .{
            .ptr = spi,
            .fn_cmd_write = cmd_write,
            .fn_cmd_read = cmd_read,
        };
    }
};

const SpiBusCyw43 = struct {
    const Self = @This();
    spi_device: *SPI_Device,

    fn init(spi: *SPI_Device) SpiBusCyw43 {
        return .{ .spi_device = spi };
    }

    fn cmd_write(dev: *anyopaque, buffer: []const u32) u32 {
        const spi_dev: *SPI_Device = @ptrCast(@alignCast(dev));
        _ = spi_dev.write(std.mem.sliceAsBytes(buffer)) catch {};
        //spi_dev.write(buffer);
        return 0;
    }

    fn cmd_read(dev: *anyopaque, cmd: u32, buffer: []u32) u32 {
        const spi_dev: *SPI_Device = @ptrCast(@alignCast(dev));
        const cmd_bytes: [4]u8 = std.mem.toBytes(cmd);
        std.log.info("spi read op...", .{});
        _ = spi_dev.write(&cmd_bytes) catch {
            std.log.info("spi write error", .{});
        };
        _ = spi_dev.read(std.mem.sliceAsBytes(buffer)) catch {
            std.log.info("spi read error", .{});
        };
        //spi_dev.read(buffer);
        return 0;
    }

    pub fn spi_bus_cyw43(spi: *Self) wifi.SpiBusCyw43 {
        return .{
            .ptr = spi,
            .fn_cmd_write = cmd_write,
            .fn_cmd_read = cmd_read,
        };
    }
};

const WifiDevice = struct {};



/// Command type
pub const CmdType = enum(u1) {
    read = 0,    // Read operation
    write = 1,   // Write operation
};

/// Increment mode
pub const IncrMode = enum(u1) {
    fixed = 0,       // Fixed address (no increment)
    incremental = 1, // Incremental burst
};

/// Function type
pub const FuncType = enum(u2) {
    bus = 0,
    backplane = 1,
    wlan = 2,
    bt = 3
};

/// CYW43 Command Word (32-bit)
pub const CYW43Cmd = packed struct(u32) {
    len: u11, 
    addr: u17,
    func: FuncType = .bus,
    incr: IncrMode = .fixed,
    cmd: CmdType,
};