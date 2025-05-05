import argparse
import time

import matplotlib.pyplot as plt
import numpy as np
import serial


def read_and_plot_data(port, baudrate=115200, range=8, timeout=1):
    """
    Read accelerometer data from serial port and plot it
    """
    print(f"Opening serial port {port} at {baudrate} baud...")

    try:
        # Open serial connection
        ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2)  # Allow time for connection to establish

        # Send command to read data
        print("Sending READ_DATA command...")
        ser.write(b"READ_DATA\n")

        # Initialize variables for data collection
        accel_x = []
        accel_y = []
        accel_z = []
        indices = []
        collecting = False
        time_scale_us = 1000  # Default time scale in microseconds

        print("Waiting for data...")

        # Read data until "End of data" is received
        while True:
            line = ser.readline().decode("utf-8").strip()

            if not line:
                continue

            print(f"Received: {line}")

            # Check if we've started receiving the data
            if "Reading fall data from EEPROM" in line:
                collecting = True
                continue

            # Check for time scale information
            if "Time scale between data:" in line:
                try:
                    time_scale_str = line.split(":")[1].strip()
                    time_scale_us = int(time_scale_str.split()[0])
                    print(f"Time scale between samples: {time_scale_us} microseconds")
                except (IndexError, ValueError) as e:
                    print(f"Error parsing time scale: {e}")
                continue

            # Check if we've reached the end of data
            if "End of data" in line:
                break

            # Skip other metadata lines
            if collecting and "Number of records" in line:
                continue

            # Parse data lines
            if collecting and line[0].isdigit():
                try:
                    # Split the line to get index and values
                    parts = line.split(",", 1)

                    if len(parts) == 2:
                        idx = int(parts[0])
                        vals = parts[1].split(",")

                        if len(vals) == 3:
                            x = int(vals[0])
                            y = int(vals[1])
                            z = int(vals[2])

                            indices.append(idx)
                            accel_x.append(x)
                            accel_y.append(y)
                            accel_z.append(z)
                except ValueError as e:
                    print(f"Error parsing line: {line}, Error: {e}")

        ser.close()

        # Convert lists to numpy arrays
        indices = np.array(indices)
        accel_x = np.array(accel_x)
        accel_y = np.array(accel_y)
        accel_z = np.array(accel_z)

        # Create time array in milliseconds
        time_ms = indices * (time_scale_us / 1000)

        print(f"Collected {len(indices)} data points")

        # Scale data as in the example
        # Sensitivity values in LSB/g for each range
        sensitivity = {2: 16384.0, 4: 8192.0, 8: 4096.0, 16: 2048.0}  # ±2g  # ±4g  # ±8g  # ±16g
        scale = sensitivity.get(range, 4096.0)
        ax = accel_x / scale
        ay = accel_y / scale
        az = accel_z / scale
        a_total = np.sqrt(ax * ax + ay * ay + az * az)

        # Create figure and subplots
        plt.figure(figsize=(12, 10))

        # Plot total acceleration
        plt.subplot(2, 1, 1)
        plt.plot(time_ms, a_total, "k-", label="Total")
        plt.xlabel("Time (ms)")
        plt.ylabel("Acceleration [g]")
        plt.title("Accelerometer Data - Total")
        plt.legend()
        plt.grid(True)

        # Plot X, Y, Z accelerations
        plt.subplot(2, 1, 2)
        plt.plot(time_ms, ax, "r-", label="X-Axis")
        plt.plot(time_ms, ay, "g-", label="Y-Axis")
        plt.plot(time_ms, az, "b-", label="Z-Axis")
        plt.xlabel("Time (ms)")
        plt.ylabel("Acceleration [g]")
        plt.title("Accelerometer Data - X, Y, Z Components")
        plt.legend()
        plt.grid(True)

        plt.tight_layout()
        plt.savefig("accelerometer_data.png", dpi=1200)
        plt.show()

        print("Plot saved as 'accelerometer_data.png'")

    except serial.SerialException as e:
        print(f"Error with serial port: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Serial Accelerometer Data Visualizer")
    parser.add_argument("--port", type=str, help="Serial port (e.g., COM3 or /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--range", type=int, default=8, help="MPU range (default: 8) [2, 4, 8, 18] g")

    args = parser.parse_args()

    if args.port:
        read_and_plot_data(args.port, args.baud, args.range)
    else:
        print("Error: Please specify a serial port (--port)")
        parser.print_help()
