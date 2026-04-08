# EECE 5554 — Sensor Emulator

A pseudo-serial-port emulator for the GPS, RTK, and VectorNav IMU sensors used in **EECE 5554: Robotics Sensing and Navigation**. It replays recorded data files over a Linux pseudoterminal (`/dev/pts/N`), so you can develop and test your ROS driver nodes without the physical hardware.

## Quick Start

```bash
# Clone
git clone https://github.com/ShawnLiNU/EECE5554-Robotics-Sensing-And-Navigation.git
cd sensor_emulator

# Install dependency
pip3 install pyserial

# Run (GPS example — loops by default)
python3 serial_emulator.py -f GPS_Chicago.txt -d gps
```

The emulator prints a pseudo-device path (e.g., `/dev/pts/2` on Linux, `/dev/ttys003` on macOS). In a second terminal, read the output with:

```bash
cat /dev/pts/2
```

Or point your ROS driver node at that same path. (`minicom -D /dev/pts/2` also works but is unnecessary for a pseudoterminal — baud rate and flow control don't apply since data moves through kernel memory, not a physical wire.)

## Usage

```
python3 serial_emulator.py -f <FILE> -d <DEVICE> [-l yes|no] [-V <VN_CMD>]
```

| Flag | Required | Description |
|------|----------|-------------|
| `-f`, `--file` | Yes | Path to the recorded data file |
| `-d`, `--device_type` | Yes | `gps`, `rtk`, or `imu` |
| `-l`, `--loop` | No | `yes` (default) to loop, `no` to stop at EOF |
| `-V`, `--VN_reg` | No | VectorNav write-register command (IMU only) |

### Default Sample Rates

| Device | Rate | Behavior |
|--------|------|----------|
| `gps`  | 1 Hz | NMEA sentences grouped by UTC timestamp; one group per second |
| `rtk`  | 5 Hz | Same grouping, five groups per second |
| `imu`  | 200 Hz | One `$VNYMR` line every 5 ms |

### Examples

```bash
# GPS — play once, then quit
python3 serial_emulator.py -f GPS_Chicago.txt -d gps -l no

# GPS — loop forever (Ctrl-C to stop)
python3 serial_emulator.py -f GPS_ISEC.txt -d gps

# RTK at 5 Hz
python3 serial_emulator.py -f rtk_data.txt -d rtk

# IMU at default 200 Hz
python3 serial_emulator.py -f imu_data.txt -d imu

# IMU — set custom rate to 40 Hz via VectorNav register write
python3 serial_emulator.py -f imu_data.txt -d imu -V "$VNWRG,07,40*XX"
```

## How It Works

1. The emulator opens a Linux **pseudoterminal** pair (master + slave).
2. It reads the data file and writes each line to the master side with `\r\n` line endings.
3. Your driver (or minicom) reads from the slave side (`/dev/pts/N`), just like a real serial port.

**GPS / RTK mode:** NMEA sentences are grouped by their UTC timestamp. All sentences sharing the same timestamp (e.g., `$GPGGA`, `$GPGSA`, `$GPRMC`, `$GPGSV`) are written together as one burst, then the emulator waits for the next tick. This matches how a real receiver behaves — it outputs an entire navigation solution at once, not one sentence at a time.

**IMU mode:** Each `$VNYMR` line is written individually at the configured rate.

## Included Data Files

| File | Sensor | Location | Notes |
|------|--------|----------|-------|
| `GPS_Chicago.txt` | GPS | Chicago, IL | ~587 epochs, moving + stationary |
| `GPS_ISEC.txt` | GPS | ISEC, Northeastern | ~22 epochs, stationary |
| `gps_data.txt` | GPS | Europe (simulated) | 76 epochs, moving pattern |
| `imu_data.txt` | IMU | — | 163 samples of `$VNYMR` data |

## Changing the VectorNav Sample Rate

The `-V` flag lets you practice sending a write-register command to the emulated VectorNav. The command format is:

```
$VNWRG,07,<rate>*XX
```

where `<rate>` is the desired output rate in Hz. For example, `$VNWRG,07,40*XX` sets the rate to 40 Hz. The checksum (`XX`) is not validated by the emulator, so any two characters work. If the command is malformed, the emulator will print an error and exit.

## Troubleshooting

**"File not found"** — Make sure you are running the command from the directory containing the data files, or provide a full path with `-f`.

**No output from `cat`** — Verify the device path matches exactly. If you restart the emulator, the number may change.

**Data looks garbled** — The baud rate does not matter for pseudoterminals (data is transferred in-memory), but if your driver expects a specific baud, configure it to match what the real sensor uses (e.g., 4800 for GPS, 115200 for VectorNav).

**Terminal escape characters in data files** — Some data files were originally captured with `minicom` and contain embedded ANSI escape sequences. The emulator strips these automatically before writing to the port.


## Credits

The emulator has been tested with Python 3.10+ by Dr. Xian Li, written by Jagapreet Singh Nir, and updated by Dr. Kris Dorsey & Dr. Xian Li.