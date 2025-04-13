const std = @import("std");
const microzig = @import("microzig");
const SPI_Device =  microzig.hal.drivers.SPI_Device;
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
    const csn = rp2xxx.gpio.num(29);
    const tx = rp2xxx.gpio.num(24);
    const rx = rp2xxx.gpio.num(24);
    const sck = rp2xxx.gpio.num(25);
    inline for (&.{ csn, tx, rx, sck }) |pin| {
        pin.set_function(.spi);
    }

    uart_tx_pin.set_function(.uart);

    uart.apply(.{
        .baud_rate = baud_rate,
        .clock_config = rp2xxx.clock_config,
    });

    rp2xxx.uart.init_logger(uart);

    const spi1 = rp2xxx.spi.instance.SPI1;
    try spi1.apply(.{ .clock_config = rp2xxx.clock_config });

    var spi_device: SPI_Device = SPI_Device.init(spi1, rp2xxx.gpio.num(25), .{ .active_level = .low });
    try spi_device.connect();

    var spi_cyw43: SpiBusCyw43 = SpiBusCyw43.init(&spi_device);
    var power_pin: GPIO_Device = GPIO_Device.init(rp2xxx.gpio.num(23));
    var wifi_bus = wifi.Bus.init(power_pin.digital_io(), spi_cyw43.spi_bus_cyw43(), sleep);

    var i: u32 = 0;
    while (true) : (i += 1) {
        std.log.info("what {}", .{i});
        time.sleep_ms(500);

        time.sleep_ms(500);

        time.sleep_ms(8000);
        try wifi_bus.init_bus();
    }
}

const Cyw43PioSpi = struct {
    const Self = @This();
    const pio: rp2xxx.pio.Pio = rp2xxx.pio.num(0);
    sm: rp2xxx.pio.StateMachine,
    io_pin: rp2xxx.gpio.num(0),
    clk_pin: rp2xxx.gpio.num(0),

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

    fn init(this: *Self) void {
        this.io_pin.set_function(.pio0);
        this.io_pin.set_pull(.disable);
        this.io_pin.set_schmitt_trigger(.enabled);
        //pin_io.set_input_sync_bypass(true);
        //pin_io.set_drive_strength(Drive::_12mA);
        this.io_pin.set_slew_rate(.fast);

        this.clk_pin.set_function(.pio0);
        //pin_io.set_drive_strength(Drive::_12mA);
        this.clk_pin.set_slew_rate(.fast);
    }

    fn cmd_read(selfopaque: *anyopaque, cmd: u32, buffer: []u32) u32 {
        const this: *Self = @ptrCast(@alignCast(selfopaque));
        this.pio.sm_set_enabled(this.sm, false);

        const write_bits = 31;
        const read_bits = buffer.len * 32 + 32 - 1;

        this.sm.sm_exec_set_y(read_bits);
        this.sm.sm_exec_set_x(write_bits);
        this.sm.sm_exec_set_pindir(0b1);
        this.sm.sm_exec_jmp(cw49spi_program.wrap_target.?);
        
        this.pio.sm_set_enabled(this.sm, true);

        const dma = rp2xxx.dma.channel(1);
        dma.claim();
        defer dma.unclaim();

        const cmd_data = std.mem.sliceAsBytes(cmd);
        dma.trigger_transfer(this.sm.sm_get_tx_fifo(), cmd_data.ptr, 1, .{ .transfer_size_bytes = 4, .enable = true, .read_increment = true, .write_increment = false, .dreq = @enumFromInt(@intFromEnum(this.pio) * 8 + @intFromEnum(this.sm)) });
        dma.trigger_transfer(this.sm.sm_get_rx_fifo(), buffer.ptr, buffer.len, .{ .transfer_size_bytes = 4, .enable = true, .read_increment = false, .write_increment = true, .dreq = @enumFromInt(@intFromEnum(this.pio) * 8 + @intFromEnum(this.sm) + 4) });
        var status: u32 = 0;
        var status_data = std.mem.sliceAsBytes(status);
        dma.trigger_transfer(this.sm.sm_get_rx_fifo(), status_data.ptr, buffer.len, .{ .transfer_size_bytes = 4, .enable = true, .read_increment = false, .write_increment = true, .dreq = @enumFromInt(@intFromEnum(this.pio) * 8 + @intFromEnum(this.sm) + 4) });

        return status;
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

const WifiDevice = struct {
};