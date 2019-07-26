"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

import cStringIO as StringIO
import struct

class laser_t(object):
    __slots__ = ["utime", "nranges", "ranges", "nintensities", "intensities", "rad0", "radstep"]

    def __init__(self):
        self.utime = 0
        self.nranges = 0
        self.ranges = []
        self.nintensities = 0
        self.intensities = []
        self.rad0 = 0.0
        self.radstep = 0.0

    def encode(self):
        buf = StringIO.StringIO()
        buf.write(laser_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qi", self.utime, self.nranges))
        buf.write(struct.pack('>%df' % self.nranges, *self.ranges[:self.nranges]))
        buf.write(struct.pack(">i", self.nintensities))
        buf.write(struct.pack('>%df' % self.nintensities, *self.intensities[:self.nintensities]))
        buf.write(struct.pack(">ff", self.rad0, self.radstep))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = StringIO.StringIO(data)
        if buf.read(8) != laser_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return laser_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = laser_t()
        self.utime, self.nranges = struct.unpack(">qi", buf.read(12))
        self.ranges = struct.unpack('>%df' % self.nranges, buf.read(self.nranges * 4))
        self.nintensities = struct.unpack(">i", buf.read(4))[0]
        self.intensities = struct.unpack('>%df' % self.nintensities, buf.read(self.nintensities * 4))
        self.rad0, self.radstep = struct.unpack(">ff", buf.read(8))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if laser_t in parents: return 0
        tmphash = (0xf1e8ba118c05af46) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if laser_t._packed_fingerprint is None:
            laser_t._packed_fingerprint = struct.pack(">Q", laser_t._get_hash_recursive([]))
        return laser_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

