// USB CDC
//
// This is just enough implementation to allow us to act as a USB serial device.
// https://cscott.net/usb_dev/data/devclass/usbcdc11.pdf

const std = @import("std");
const buffers = @import("../buffers.zig");

pub const descriptors = @import("descriptors.zig");

const BufferReader = buffers.BufferReader;
const BufferWriter = buffers.BufferWriter;
const DescType = descriptors.DescType;

pub const DescSubType = enum(u8) {
    Header = 0x00,
    CallManagement = 0x01,
    ACM = 0x02,
    Union = 0x06,

    pub fn from_u16(v: u16) ?@This() {
        return std.meta.intToEnum(@This(), v) catch null;
    }
};

pub const CdcHeader = extern struct {
    // Type of this descriptor, must be `ClassSpecific`.
    descriptor_type: DescType,
    // Subtype of this descriptor, must be `Header`.
    descriptor_subtype: DescSubType,
    // USB Class Definitions for Communication Devices Specification release
    // number in binary-coded decimal. Typically 0x01_10.
    bcd_cdc: u16,

    pub fn serialize(self: *const @This()) [5]u8 {
        var out: [5]u8 = undefined;
        out[0] = out.len; // length
        out[1] = @intFromEnum(self.descriptor_type);
        out[2] = @intFromEnum(self.descriptor_subtype);
        out[3] = @intCast(self.bcd_cdc & 0xff);
        out[4] = @intCast((self.bcd_cdc >> 8) & 0xff);
        return out;
    }

    pub fn serialize_buff(self: *const @This(), buff: *BufferWriter) BufferWriter.Error!void {
        const length = 5;
        try buff.bound_check(length);
        buff.write_int_unsafe(u8, length);
        buff.write_int_unsafe(u8, @intFromEnum(self.descriptor_type));
        buff.write_int_unsafe(u8, @intFromEnum(self.descriptor_subtype));
        buff.write_int_unsafe(u16, self.bcd_cdc);
    }
};

pub const CdcCallManagement = extern struct {
    // Type of this descriptor, must be `ClassSpecific`.
    descriptor_type: DescType,
    // Subtype of this descriptor, must be `CallManagement`.
    descriptor_subtype: DescSubType,
    // Capabilities. Should be 0x00 for use as a serial device.
    capabilities: u8,
    // Data interface number.
    data_interface: u8,

    pub fn serialize(self: *const @This()) [5]u8 {
        var out: [5]u8 = undefined;
        out[0] = out.len; // length
        out[1] = @intFromEnum(self.descriptor_type);
        out[2] = @intFromEnum(self.descriptor_subtype);
        out[3] = self.capabilities;
        out[4] = self.data_interface;
        return out;
    }

    pub fn serialize_buff(self: *const @This(), buff: *BufferWriter) BufferWriter.Error!void {
        const length = 5;
        try buff.bound_check(length);
        buff.write_int_unsafe(u8, length);
        buff.write_int_unsafe(u8, @intFromEnum(self.descriptor_type));
        buff.write_int_unsafe(u8, @intFromEnum(self.descriptor_subtype));
        buff.write_int_unsafe(u8, self.capabilities);
        buff.write_int_unsafe(u8, self.data_interface);
    }
};

pub const CdcAcm = extern struct {
    // Type of this descriptor, must be `ClassSpecific`.
    descriptor_type: DescType,
    // Subtype of this descriptor, must be `ACM`.
    descriptor_subtype: DescSubType,
    // Capabilities. Should be 0x02 for use as a serial device.
    capabilities: u8,

    pub fn serialize(self: *const @This()) [4]u8 {
        var out: [4]u8 = undefined;
        out[0] = out.len;
        out[1] = @intFromEnum(self.descriptor_type);
        out[2] = @intFromEnum(self.descriptor_subtype);
        out[3] = self.capabilities;
        return out;
    }

    pub fn serialize_buff(self: *const @This(), buff: *BufferWriter) BufferWriter.Error!void {
        const length = 4;
        try buff.bound_check(length);
        buff.write_int_unsafe(u8, length);
        buff.write_int_unsafe(u8, @intFromEnum(self.descriptor_type));
        buff.write_int_unsafe(u8, @intFromEnum(self.descriptor_subtype));
        buff.write_int_unsafe(u8, self.capabilities);
    }
};

pub const CdcUnion = extern struct {
    // Type of this descriptor, must be `ClassSpecific`.
    descriptor_type: DescType,
    // Subtype of this descriptor, must be `Union`.
    descriptor_subtype: DescSubType,
    // The interface number of the communication or data class interface
    // designated as the master or controlling interface for the union.
    master_interface: u8,
    // The interface number of the first slave or associated interface in the
    // union.
    slave_interface_0: u8,

    pub fn serialize(self: *const @This()) [5]u8 {
        var out: [5]u8 = undefined;
        out[0] = out.len;
        out[1] = @intFromEnum(self.descriptor_type);
        out[2] = @intFromEnum(self.descriptor_subtype);
        out[3] = self.master_interface;
        out[4] = self.slave_interface_0;
        return out;
    }

    pub fn serialize_buff(self: *const @This(), buff: *BufferWriter) BufferWriter.Error!void {
        const length = 5;
        try buff.bound_check(length);
        buff.write_int_unsafe(u8, length);
        buff.write_int_unsafe(u8, @intFromEnum(self.descriptor_type));
        buff.write_int_unsafe(u8, @intFromEnum(self.descriptor_subtype));
        buff.write_int_unsafe(u8, self.master_interface);
        buff.write_int_unsafe(u8, self.slave_interface_0);
    }
};