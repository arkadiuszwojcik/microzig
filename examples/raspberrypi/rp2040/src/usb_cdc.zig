const std = @import("std");
const microzig = @import("microzig");

const rp2040 = microzig.hal;
const time = rp2040.time;
const gpio = rp2040.gpio;
const clocks = rp2040.clocks;
const usb = rp2040.usb;

const uart0 = rp2040.uart.num(0);

pub const microzig_options = .{
    .log_level = .debug,
    .logFn = rp2040.uart.log,
};

var usb_config = [_]usb.ConfigEntry {
    .{ .configuration = &.{ .descriptor_type = usb.DescType.Config, .total_length = 9 + 66, .num_interfaces = 2, .configuration_value = 1, .configuration_s = 0, .attributes = 0xc0, .max_power = 0x32, }  },
    .{ .interface_association = &.{ .length = @sizeOf(@This()), .descriptor_type = .InterfaceAssociation, .first_interface = 0, .interface_count = 2, .function_class = 2, .function_subclass = 2, .function_protocol = 0, .function = 0 } },
    .{ .interface = &.{ .descriptor_type = .Interface, .interface_number = 0, .alternate_setting = 0, .num_endpoints = 1, .interface_class = 2, .interface_subclass = 2, .interface_protocol = 0, .interface_s = 3 } },
    .{ .cdc = &.{ .cdc_header = &.{ .descriptor_type = .CsInterface, .descriptor_subtype = .Header, .bcd_cdc = 0x0120 } } },
    .{ .cdc = &.{ .cdc_call_management = &.{ .descriptor_type = .CsInterface, .descriptor_subtype = .CallManagement, .capabilities = 0, .data_interface = 0 + 1 } } },
    .{ .cdc = &.{ .cdc_acm = &.{ .descriptor_type = .CsInterface, .descriptor_subtype = .ACM, .capabilities = 6 } } },
    .{ .cdc = &.{ .cdc_union = &.{ .descriptor_type = .CsInterface, .descriptor_subtype = .Union, .master_interface = 0, .slave_interface_0 = 0 + 1 } } },
    .{ .endpoint = &.{ .descriptor_type = .Endpoint, .endpoint_address = 0x81, .attributes = 3, .max_packet_size = 8, .interval = 16 } },
    .{ .interface = &.{ .descriptor_type = .Interface, .interface_number = 0 + 1, .alternate_setting = 0, .num_endpoints = 2, .interface_class = 10, .interface_subclass = 0, .interface_protocol = 0, .interface_s = 0 } },
    .{ .endpoint = &.{ .descriptor_type = .Endpoint, .endpoint_address = 0x02, .attributes = 2, .max_packet_size = 64, .interval = 0 } },
    .{ .endpoint = &.{ .descriptor_type = .Endpoint, .endpoint_address = 0x82, .attributes = 2, .max_packet_size = 64, .interval = 0 } },
};

const part0: usb.ConfigurationDescriptor = .{ .descriptor_type = usb.DescType.Config, .total_length = 9 + 66, .num_interfaces = 2, .configuration_value = 1, .configuration_s = 0, .attributes = 0xc0, .max_power = 0x32, };
const part1: usb.InterfaceAssociationDescriptor = .{ .length = @sizeOf(@This()), .descriptor_type = .InterfaceAssociation, .first_interface = 0, .interface_count = 2, .function_class = 2, .function_subclass = 2, .function_protocol = 0, .function = 0 };
const part2: usb.InterfaceDescriptor = .{ .descriptor_type = .Interface, .interface_number = 0, .alternate_setting = 0, .num_endpoints = 1, .interface_class = 2, .interface_subclass = 2, .interface_protocol = 0, .interface_s = 3 };
const part3: microzig.core.usb.cdc.CdcHeader = .{ .descriptor_type = .CsInterface, .descriptor_subtype = .Header, .bcd_cdc = 0x0120 };
const part4: microzig.core.usb.cdc.CdcCallManagement = .{ .descriptor_type = .CsInterface, .descriptor_subtype = .CallManagement, .capabilities = 0, .data_interface = 0 + 1 };
const part5: microzig.core.usb.cdc.CdcAcm = .{ .descriptor_type = .CsInterface, .descriptor_subtype = .ACM, .capabilities = 6, };
const part6: microzig.core.usb.cdc.CdcUnion = .{ .descriptor_type = .CsInterface, .descriptor_subtype = .Union, .master_interface = 0, .slave_interface_0 = 0 + 1 };
const part7: microzig.core.usb.EndpointDescriptor = .{ .descriptor_type = .Endpoint, .endpoint_address = 0x81, .attributes = 3, .max_packet_size = 8, .interval = 16 };
const part8: microzig.core.usb.InterfaceDescriptor = .{ .descriptor_type = .Interface, .interface_number = 0 + 1, .alternate_setting = 0, .num_endpoints = 2, .interface_class = 10, .interface_subclass = 0, .interface_protocol = 0, .interface_s = 0 };
const part9: microzig.core.usb.descriptors.EndpointDescriptor = .{ .descriptor_type = .Endpoint, .endpoint_address = 0x02, .attributes = 2, .max_packet_size = 64, .interval = 0 };
const part10: microzig.core.usb.descriptors.EndpointDescriptor = .{ .descriptor_type = .Endpoint, .endpoint_address = 0x82, .attributes = 2, .max_packet_size = 64, .interval = 0, };

// The endpoints EP0_IN and EP0_OUT are already defined but you can
// add your own endpoints to...
pub var EP1_OUT_CFG: usb.EndpointConfiguration = .{
    .descriptor = &part9,
    .endpoint_control_index = 2,
    .buffer_control_index = 3,
    .data_buffer_index = 2,
    .next_pid_1 = false,
    // The callback will be executed if we got an interrupt on EP1_OUT
    .callback = ep1_out_callback,
};

pub var EP1_IN_CFG: usb.EndpointConfiguration = .{
    .descriptor = &part10,
    .endpoint_control_index = 1,
    .buffer_control_index = 2,
    .data_buffer_index = 3,
    .next_pid_1 = false,
    // The callback will be executed if we got an interrupt on EP1_IN
    .callback = ep1_in_callback,
};

// First we define two callbacks that will be used by the endpoints we define next...
fn ep1_in_callback(_: *usb.DeviceConfiguration, data: []const u8) void {
    _ = data;
    // The host has collected the data we repeated onto
    // EP1! Set up to receive more data on EP1.
    
    //usb.Usb.callbacks.usb_start_rx(
    //    dc.endpoints[0], // EP1_OUT_CFG,
    //    64,
    //);
}

fn ep1_out_callback(_: *usb.DeviceConfiguration, data: []const u8) void {
    _ = data;
    // We've gotten data from the host on our custom
    // EP1! Set up EP1 to repeat it.

    //usb.Usb.callbacks.usb_start_tx(
    //    dc.endpoints[1], // EP1_IN_CFG,
    //    data,
    //);
}

// This is our device configuration
pub var DEVICE_CONFIGURATION: usb.DeviceConfiguration = .{
    .device_descriptor = &.{
        .descriptor_type = usb.DescType.Device,
        .bcd_usb = 0x0200,
        .device_class = 0,
        .device_subclass = 0,
        .device_protocol = 0,
        .max_packet_size0 = 64,
        .vendor = 0,
        .product = 1,
        .bcd_device = 0x0100,
        .manufacturer_s = 1,
        .product_s = 2,
        .serial_s = 0,
        .num_configurations = 1,
    },
    // Not used
    .interface_descriptor = &.{
        .descriptor_type = usb.DescType.Interface,
        .interface_number = 0,
        .alternate_setting = 0,
        // We have two endpoints (EP0 IN/OUT don't count)
        .num_endpoints = 2,
        .interface_class = 3,
        .interface_subclass = 0,
        .interface_protocol = 0,
        .interface_s = 0,
    },
    .config_descriptor = &.{
        .descriptor_type = usb.DescType.Config,
        // This is calculated via the sizes of underlying descriptors contained in this configuration.
        // ConfigurationDescriptor(9) + InterfaceDescriptor(9) * 1 + EndpointDescriptor(8) * 2
        .total_length = 9 + 66,
        .num_interfaces = 2,
        .configuration_value = 1,
        .configuration_s = 0,
        .attributes = 0xc0,
        .max_power = 0x32,
    },
    .new_config = &usb_config,

    .lang_descriptor = "\x04\x03\x09\x04", // length || string descriptor (0x03) || Engl (0x0409)
    .descriptor_strings = &.{
        // ugly unicode :|
        "R\x00a\x00s\x00p\x00b\x00e\x00r\x00r\x00y\x00 \x00P\x00i\x00",
        "P\x00i\x00c\x00o\x00 \x00T\x00e\x00s\x00t\x00 \x00D\x00e\x00v\x00i\x00c\x00e\x00",
        "B\x00o\x00a\x00r\x00d\x00 \x00C\x00D\x00C\x00"
    },
    .endpoints = .{
        &usb.EP0_OUT_CFG,
        &usb.EP0_IN_CFG,
        &EP1_OUT_CFG,
        &EP1_IN_CFG,
    },
};

pub fn panic(message: []const u8, _: ?*std.builtin.StackTrace, _: ?usize) noreturn {
    std.log.err("panic: {s}", .{message});
    @breakpoint();
    while (true) {}
}

pub fn main() !void {

    // UART for log
    uart0.apply(.{
        .baud_rate = 115200,
        .tx_pin = gpio.num(0),
        .rx_pin = gpio.num(1),
        .clock_config = rp2040.clock_config,
    });

    rp2040.uart.init_logger(uart0);

    // First we initialize the USB clock
    rp2040.usb.Usb.init_clk();
    // Then initialize the USB device using the configuration defined above
    rp2040.usb.Usb.init_device(&DEVICE_CONFIGURATION) catch unreachable;

    var old: u64 = time.get_time_since_boot().to_us();
    var new: u64 = 0;
    var i: u32 = 0;
    while (true) {

        new = time.get_time_since_boot().to_us();
        if (new - old > 6 * 1000000) {
            old = new;
            i = i + 1;
            std.log.info("loop {}", .{i});
        }
        
        rp2040.usb.Usb.task(
            false, // debug output over UART [Y/n]
        ) catch unreachable;
    }
}