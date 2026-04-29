#!/usr/bin/env python3
"""
Step 1: Detect & Identify the BNO055 IMU on I2C bus.

Run this on the Jetson via SSH:
    python3 tests/step1_detect_imu.py

Expected output (if wiring is correct):
    ✅ BNO055 detected at 0x29 on bus 1
    CHIP_ID  = 0xA0  (expected 0xA0)  ✅
    ACC_ID   = 0xFB  (expected 0xFB)  ✅
    MAG_ID   = 0x32  (expected 0x32)  ✅
    GYR_ID   = 0x0F  (expected 0x0F)  ✅
    SW_REV   = <version>
    BL_REV   = <version>

What this script does:
    1. Opens I2C bus 1
    2. Reads the CHIP_ID register (0x00) — should be 0xA0 for BNO055
    3. Reads sub-sensor IDs (accelerometer, magnetometer, gyroscope)
    4. Reads firmware/bootloader versions
    5. Reports pass/fail for each check

Analogy:
    This is like calling a phone number and checking "Hello, is this BNO055?"
    The WHO_AM_I register is the sensor's name tag — if it answers 0xA0,
    we know we're talking to the right chip.
"""

import sys
import time

# ── BNO055 Register Map (only what we need for detection) ────────────
CHIP_ID_REG       = 0x00   # Should read 0xA0
ACC_ID_REG        = 0x01   # Should read 0xFB
MAG_ID_REG        = 0x02   # Should read 0x32
GYR_ID_REG        = 0x03   # Should read 0x0F
SW_REV_LSB_REG    = 0x04   # Software revision (low byte)
SW_REV_MSB_REG    = 0x05   # Software revision (high byte)
BL_REV_REG        = 0x06   # Bootloader revision
SYS_STATUS_REG    = 0x39   # System status
SYS_ERROR_REG     = 0x3A   # System error
OPR_MODE_REG      = 0x3D   # Operating mode

# Expected ID values
EXPECTED_CHIP_ID  = 0xA0
EXPECTED_ACC_ID   = 0xFB
EXPECTED_MAG_ID   = 0x32
EXPECTED_GYR_ID   = 0x0F

# I2C configuration
I2C_BUS = 1
I2C_ADDR = 0x29   # Your BNO055 has COM3=HIGH, so address is 0x29


def check(label, actual, expected):
    """Print a pass/fail check for a register value."""
    match = actual == expected
    icon = "✅" if match else "❌"
    print(f"  {label:8s} = 0x{actual:02X}  (expected 0x{expected:02X})  {icon}")
    return match


def main():
    # ── Try to import smbus2 (preferred) or smbus ────────────────────
    try:
        from smbus2 import SMBus
        bus_lib = "smbus2"
    except ImportError:
        try:
            from smbus import SMBus
            bus_lib = "smbus"
        except ImportError:
            print("❌ Neither smbus2 nor smbus is installed!")
            print()
            print("   Fix: run one of these on the Jetson:")
            print("     pip3 install smbus2")
            print("     sudo apt install python3-smbus")
            sys.exit(1)

    print(f"Using {bus_lib} library")
    print(f"Scanning I2C bus {I2C_BUS} at address 0x{I2C_ADDR:02X}...")
    print()

    # ── Open I2C bus ─────────────────────────────────────────────────
    try:
        bus = SMBus(I2C_BUS)
    except FileNotFoundError:
        print(f"❌ I2C bus {I2C_BUS} not found!")
        print(f"   /dev/i2c-{I2C_BUS} does not exist.")
        print(f"   Check your Jetson I2C configuration.")
        sys.exit(1)
    except PermissionError:
        print(f"❌ Permission denied on I2C bus {I2C_BUS}!")
        print(f"   Run with: sudo python3 {sys.argv[0]}")
        print(f"   Or add yourself to the i2c group: sudo usermod -aG i2c $USER")
        sys.exit(1)

    # ── Read chip identification registers ───────────────────────────
    try:
        chip_id = bus.read_byte_data(I2C_ADDR, CHIP_ID_REG)
    except OSError as e:
        print(f"❌ Failed to read from address 0x{I2C_ADDR:02X}: {e}")
        print()
        print("   Possible causes:")
        print("   1. IMU is not wired to bus 1 (pins 3 SDA, 5 SCL)")
        print("   2. Wrong I2C address — try 0x28 instead of 0x29")
        print("   3. IMU not powered (check VIN/3.3V connection)")
        print("   4. Wires are loose or swapped (SDA↔SCL)")
        print()
        print("   Debug: run 'sudo i2cdetect -y -r 1' and check what shows up")
        bus.close()
        sys.exit(1)

    # ── Verify CHIP_ID ───────────────────────────────────────────────
    if chip_id == EXPECTED_CHIP_ID:
        print(f"✅ BNO055 detected at 0x{I2C_ADDR:02X} on bus {I2C_BUS}!")
    else:
        print(f"⚠️  Chip at 0x{I2C_ADDR:02X} responded with CHIP_ID=0x{chip_id:02X}")
        print(f"   (BNO055 should be 0xA0)")
        print(f"   This might be a different sensor.")

    print()

    # ── Read all sub-sensor IDs ──────────────────────────────────────
    acc_id = bus.read_byte_data(I2C_ADDR, ACC_ID_REG)
    mag_id = bus.read_byte_data(I2C_ADDR, MAG_ID_REG)
    gyr_id = bus.read_byte_data(I2C_ADDR, GYR_ID_REG)

    print("Sub-sensor identification:")
    all_pass = True
    all_pass &= check("CHIP_ID", chip_id, EXPECTED_CHIP_ID)
    all_pass &= check("ACC_ID",  acc_id,  EXPECTED_ACC_ID)
    all_pass &= check("MAG_ID",  mag_id,  EXPECTED_MAG_ID)
    all_pass &= check("GYR_ID",  gyr_id,  EXPECTED_GYR_ID)

    print()

    # ── Read firmware info ───────────────────────────────────────────
    sw_lsb = bus.read_byte_data(I2C_ADDR, SW_REV_LSB_REG)
    sw_msb = bus.read_byte_data(I2C_ADDR, SW_REV_MSB_REG)
    bl_rev = bus.read_byte_data(I2C_ADDR, BL_REV_REG)
    sw_rev = (sw_msb << 8) | sw_lsb

    print(f"Firmware info:")
    print(f"  SW_REV   = 0x{sw_rev:04X} (v{sw_msb}.{sw_lsb})")
    print(f"  BL_REV   = 0x{bl_rev:02X}")

    print()

    # ── Read current operating mode ──────────────────────────────────
    opr_mode = bus.read_byte_data(I2C_ADDR, OPR_MODE_REG) & 0x0F
    mode_names = {
        0x00: "CONFIGMODE",
        0x01: "ACCONLY",
        0x02: "MAGONLY",
        0x03: "GYROONLY",
        0x04: "ACCMAG",
        0x05: "ACCGYRO",
        0x06: "MAGGYRO",
        0x07: "AMG (all raw, no fusion)",
        0x08: "IMU (accel+gyro fusion, no mag)",
        0x09: "COMPASS",
        0x0A: "M4G",
        0x0B: "NDOF_FMC_OFF",
        0x0C: "NDOF (full 9-axis fusion)",
    }
    mode_str = mode_names.get(opr_mode, f"UNKNOWN (0x{opr_mode:02X})")
    print(f"Operating mode:")
    print(f"  OPR_MODE = 0x{opr_mode:02X} → {mode_str}")

    # ── Read system status ───────────────────────────────────────────
    sys_status = bus.read_byte_data(I2C_ADDR, SYS_STATUS_REG)
    sys_error  = bus.read_byte_data(I2C_ADDR, SYS_ERROR_REG)

    status_names = {
        0: "Idle",
        1: "System Error",
        2: "Initializing Peripherals",
        3: "System Initialization",
        4: "Executing Self-Test",
        5: "Running (fusion active)",
        6: "Running (no fusion)",
    }
    error_names = {
        0: "No Error",
        1: "Peripheral Initialization Error",
        2: "System Initialization Error",
        3: "Self Test Failed",
        4: "Register Map Value Out of Range",
        5: "Register Map Address Out of Range",
        6: "Register Map Write Error",
        7: "Low Power Mode Not Available",
        8: "Accelerometer Power Mode Not Available",
        9: "Fusion Algorithm Configuration Error",
        10: "Sensor Configuration Error",
    }

    print()
    print(f"System status:")
    print(f"  SYS_STATUS = {sys_status} → {status_names.get(sys_status, 'Unknown')}")
    print(f"  SYS_ERROR  = {sys_error} → {error_names.get(sys_error, 'Unknown')}")

    bus.close()

    # ── Final verdict ────────────────────────────────────────────────
    print()
    print("=" * 50)
    if all_pass and sys_error == 0:
        print("🎉 Step 1 PASSED — BNO055 is alive and well!")
        print("   Ready for Step 2: Reading raw sensor data.")
    elif all_pass:
        print("⚠️  Step 1 PARTIAL — BNO055 identified but has a system error.")
        print("   Check wiring and power supply.")
    else:
        print("❌ Step 1 FAILED — unexpected sensor IDs.")
        print("   Double-check your wiring and I2C address.")
    print("=" * 50)


if __name__ == "__main__":
    main()
