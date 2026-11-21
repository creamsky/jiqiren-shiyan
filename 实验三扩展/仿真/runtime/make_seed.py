"""Build a minimal ISO9660 NoCloud seed using the Python standard library."""
import argparse
import struct
from pathlib import Path

SECTOR = 2048


def both16(value):
    return struct.pack('<H', value) + struct.pack('>H', value)


def both32(value):
    return struct.pack('<I', value) + struct.pack('>I', value)


def record(name, sector, size, directory=False):
    identifier = name if isinstance(name, bytes) else name.encode('ascii')
    length = 33 + len(identifier) + (len(identifier) % 2 == 0)
    return (bytes((length, 0)) + both32(sector) + both32(size)
            + bytes((126, 11, 19, 0, 0, 0, 0, 2 if directory else 0, 0, 0))
            + both16(1) + bytes((len(identifier),)) + identifier
            + (b'\0' if len(identifier) % 2 == 0 else b''))


def create(source, destination):
    files = [(name, (source / name).read_bytes()) for name in ('user-data', 'meta-data')]
    next_sector = 21
    entries = []
    for name, data in files:
        entries.append((name, data, next_sector))
        next_sector += (len(data) + SECTOR - 1) // SECTOR
    data = bytearray(next_sector * SECTOR)
    pvd = bytearray(SECTOR)
    pvd[:7] = b'\x01CD001\x01'
    pvd[8:40] = b'SCOUT LAB'.ljust(32)
    pvd[40:72] = b'cidata'.ljust(32)
    pvd[80:88] = both32(next_sector)
    pvd[120:124] = both16(1)
    pvd[124:128] = both16(1)
    pvd[128:132] = both16(SECTOR)
    pvd[132:140] = both32(10)
    pvd[140:144] = struct.pack('<I', 19)
    pvd[148:152] = struct.pack('>I', 20)
    pvd[156:190] = record(b'\0', 18, SECTOR, True)
    for start in (190, 318, 446, 574):
        pvd[start:start + 128] = b' ' * 128
    for start in (702, 739, 776):
        pvd[start:start + 37] = b' ' * 37
    for start in (813, 830, 847, 864):
        pvd[start:start + 17] = b'2026111900000000\0'
    pvd[881] = 1
    data[16 * SECTOR:17 * SECTOR] = pvd
    data[17 * SECTOR:17 * SECTOR + 7] = b'\xffCD001\x01'
    directory = record(b'\0', 18, SECTOR, True) + record(b'\1', 18, SECTOR, True)
    for name, content, sector in entries:
        directory += record(name.upper() + ';1', sector, len(content))
        data[sector * SECTOR:sector * SECTOR + len(content)] = content
    data[18 * SECTOR:18 * SECTOR + len(directory)] = directory
    data[19 * SECTOR:19 * SECTOR + 10] = b'\1\0' + struct.pack('<IH', 18, 1) + b'\0\0'
    data[20 * SECTOR:20 * SECTOR + 10] = b'\1\0' + struct.pack('>IH', 18, 1) + b'\0\0'
    destination.write_bytes(data)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('source', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    create(args.source, args.destination)
