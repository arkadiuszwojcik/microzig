const std = @import("std");
const mdf = @import("../../framework.zig");

const assert = std.debug.assert;

const DigitalIO = mdf.base.Digital_IO;

pub const delayus_callback = fn (delay: u32) void;

pub const SpiBusCyw43 = struct {
    ptr: *anyopaque,
    fn_cmd_write: *const fn (ptr: *anyopaque, buffer: []const u32) u32,
    fn_cmd_read: *const fn (ptr: *anyopaque, cmd: u32, buffer: []u32) u32,

    pub fn cmd_write(self: *@This(), buffer: []const u32) u32 {
        return self.fn_cmd_write(self.ptr, buffer);
    }

    pub fn cmd_read(self: *@This(), cmd: u32, buffer: []u32) u32 {
        return self.fn_cmd_read(self.ptr, cmd, buffer);
    }
};

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
    cmd: CmdType,
    incr: IncrMode = .fixed,
    func: FuncType = .bus,
    addr: u17,
    len: u11, 
};

const TEST_PATTERN: u32 = 0x12345678;
const FEEDBEAD: u32 = 0xFEEDBEAD;

const REG_BUS_TEST_RO: u17 = 0x14;
const REG_BUS_TEST_RW: u17 = 0x18;

pub const Bus = struct {
    const Self = @This();
    pwr: DigitalIO,
    spi_bus: SpiBusCyw43,
    internal_delay_ms: *const delayus_callback,
    status: u32 = 0,

    pub fn init(pwr: DigitalIO, spi_bus: SpiBusCyw43, delay_callback: *const delayus_callback) Self {
        return Self{ .pwr = pwr, .spi_bus = spi_bus, .internal_delay_ms = delay_callback };
    }

    pub fn init_bus(bus: *Self) !void {
        try bus.pwr.write(.low);
        bus.internal_delay_ms(20);
        try bus.pwr.write(.high);
        bus.internal_delay_ms(250);

        bus.internal_delay_ms(150);

        const tes = bus.read32_swapped(.bus, REG_BUS_TEST_RO);
        std.log.info("before while {}", .{tes});
        while (true) {
            const r_val = bus.read32_swapped(.bus, REG_BUS_TEST_RO);
            if (r_val == FEEDBEAD)
                break;
            std.log.info("bla bla 0x{X}", .{r_val});
            bus.internal_delay_ms(550);
        }
        std.log.info("after while", .{});

        bus.write32_swapped(.bus, REG_BUS_TEST_RW, TEST_PATTERN);
        //const val = bus.read32_swapped(.bus, REG_BUS_TEST_RW);
        _ = bus.read32_swapped(.bus, REG_BUS_TEST_RW);
        //assert(val == TEST_PATTERN);
    }

    fn read32_swapped(bus: *Self, func: FuncType, addr: u17) u32 {
        const cmd = CYW43Cmd { .cmd = .read, .incr = .incremental, .func = func, .addr = addr, .len = 4 };
        const cmd_swapped = swap16(@bitCast(cmd));

        var buff = [1]u32{0};
        _ = bus.spi_bus.cmd_read(cmd_swapped, &buff);

        const status = swap16(buff[0]);
        // TODO: Rust code don't use swap for bus.status
        bus.status = status;

        return status;
    }

    fn write32_swapped(bus: *Self, func: FuncType, addr: u17, val: u32) void {
        const cmd = CYW43Cmd { .cmd = .write, .incr = .incremental, .func = func, .addr = addr, .len = 4 };
        const buff = [2]u32{ swap16(@bitCast(cmd)), swap16(val) };
        // TODO: Rust code don't use swap for bus.status
        bus.status = swap16(bus.spi_bus.cmd_write(&buff));
    }

    fn swap16(x: u32) u32 {
        return x << 16 | x >> 16;
    }
};