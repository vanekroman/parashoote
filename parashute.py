# Make sure to install pyserial with: pip install pyserial
import argparse
import csv
import os
import time
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np
import serial  # This is the correct import for pyserial


def read_acceleration_data(port, baudrate=9600, timeout=5):
    """
    Connect to Arduino via serial, send READ_DATA command, and parse the received data
    """
    print(f"Connecting to Arduino on {port} at {baudrate} baud...")

    try:
        # Open serial connection
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Wait for connection to establish

        # Clear any pending data
        ser.reset_input_buffer()

        # Send READ_DATA command
        print("Sending READ_DATA command...")
        ser.write(b"READ_DATA\n")

        # Wait for response
        time.sleep(1)

        # Read data until timeout or until we stop receiving data
        print("Reading response...")
        start_time = time.time()
        lines = []
        data_started = False
        data_ended = False

        while (time.time() - start_time) < timeout and not data_ended:
            if ser.in_waiting > 0:
                line = ser.readline().decode("utf-8").strip()
                print(f"Received: {line}")

                if line == "Reading fall data from EEPROM:":
                    data_started = True
                elif line == "End of data" and data_started:
                    data_ended = True
                elif data_started and "Index,AccX,AccY,AccZ" not in line and line:
                    # Check if line matches expected format (digit,digit,digit,digit)
                    if line.replace("-", "").replace(",", "").isdigit() or all(part.strip().replace("-", "").isdigit() for part in line.split(",")):
                        lines.append(line)
            else:
                time.sleep(0.1)

        # Close serial connection
        ser.close()

        if not lines:
            print("No valid data received. Make sure the Arduino is sending data in the expected format.")
            return None

        # Parse data
        data = []
        for line in lines:
            # Split by comma and convert to integers
            parts = line.split(",")
            if len(parts) >= 4:
                try:
                    row = [int(parts[0]), int(parts[1]), int(parts[2]), int(parts[3])]
                    data.append(row)
                except ValueError:
                    print(f"Skipping invalid line: {line}")

        return data

    except Exception as e:
        print(f"Error opening serial port: {e}")
        return None


def visualize_data(data, save_path=None):
    """
    Create visualization of accelerometer data
    """
    if not data or len(data) == 0:
        print("No data to visualize")
        return

    # Extract time and acceleration values
    indices = [row[0] for row in data]
    accel_x = [row[1] for row in data]
    accel_y = [row[2] for row in data]
    accel_z = [row[3] for row in data]

    # Calculate the magnitude of acceleration
    accel_magnitude = [np.sqrt(x**2 + y**2 + z**2) for x, y, z in zip(accel_x, accel_y, accel_z)]

    # Calculate the total acceleration
    ax = np.array(accel_x) / 16384.0
    ay = np.array(accel_y) / 16384.0
    az = np.array(accel_z) / 16384.0

    a_total = np.sqrt(ax * ax + ay * ay + az * az) / 3

    # Create figure and subplots
    plt.figure(figsize=(12, 10))

    # Plot total acceleration
    plt.subplot(3, 1, 1)
    plt.plot(indices, a_total, "k-")
    plt.xlabel("Sample Index")
    plt.ylabel("Acceleration [g]")
    plt.title("Accelerometer Data - Total")
    plt.legend()
    plt.grid(True)

    # Plot X, Y, Z accelerations
    plt.subplot(3, 1, 2)
    plt.plot(indices, accel_x, "r-", label="X-Axis")
    plt.plot(indices, accel_y, "g-", label="Y-Axis")
    plt.plot(indices, accel_z, "b-", label="Z-Axis")
    plt.xlabel("Sample Index")
    plt.ylabel("Acceleration (raw units)")
    plt.title("Accelerometer Data - X, Y, Z Components")
    plt.legend()
    plt.grid(True)

    # Plot magnitude
    plt.subplot(3, 1, 3)
    plt.plot(indices, accel_magnitude, "k-", linewidth=2)
    plt.xlabel("Sample Index")
    plt.ylabel("Acceleration Magnitude")
    plt.title("Accelerometer Data - Magnitude")
    plt.grid(True)

    # Add annotation to highlight fall event
    max_index = accel_magnitude.index(max(accel_magnitude))
    plt.annotate("Peak Acceleration", xy=(max_index, accel_magnitude[max_index]), xytext=(max_index, accel_magnitude[max_index] * 0.8), arrowprops=dict(facecolor="black", shrink=0.05), horizontalalignment="center")

    # Adjust layout
    plt.tight_layout()

    # Save figure if path provided
    if save_path:
        plt.savefig(save_path)
        print(f"Graph saved to {save_path}")

    # Show statistics
    print(f"Maximum acceleration magnitude: {max(accel_magnitude):.2f} at sample {accel_magnitude.index(max(accel_magnitude))}")
    print(f"Average acceleration magnitude: {sum(accel_magnitude)/len(accel_magnitude):.2f}")

    # Show plot
    plt.show()

    return accel_magnitude


def save_to_csv(data, filename):
    """Save acceleration data to CSV file"""
    with open(filename, "w", newline="") as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(["Index", "AccX", "AccY", "AccZ"])
        csvwriter.writerows(data)
    print(f"Data saved to {filename}")


def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Read and visualize accelerometer data from Arduino")
    parser.add_argument("port", help="Serial port to connect to (e.g., COM3 or /dev/ttyUSB0)")
    parser.add_argument("--baudrate", type=int, default=9600, help="Baud rate (default: 9600)")
    parser.add_argument("--timeout", type=int, default=10, help="Timeout in seconds (default: 10)")
    parser.add_argument("--no-graph", action="store_true", help="Do not display graph")
    parser.add_argument("--save-csv", action="store_true", help="Save data to CSV file")
    parser.add_argument("--save-graph", action="store_true", help="Save graph to file")
    parser.add_argument("--output-dir", default="./output", help="Directory to save output files (default: ./output)")

    args = parser.parse_args()

    # Create output directory if it doesn't exist
    if (args.save_csv or args.save_graph) and not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)

    # Generate timestamp for file names
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    # Read data from Arduino
    data = read_acceleration_data(args.port, args.baudrate, args.timeout)

    if data:
        print(f"Received {len(data)} data points")

        # Save to CSV if requested
        if args.save_csv:
            csv_path = os.path.join(args.output_dir, f"accel_data_{timestamp}.csv")
            save_to_csv(data, csv_path)

        # Show graph if not disabled
        if not args.no_graph:
            graph_path = None
            if args.save_graph:
                graph_path = os.path.join(args.output_dir, f"accel_graph_{timestamp}.png")

            visualize_data(data, graph_path)
    else:
        print("Failed to get valid data from Arduino")
        return 1

    return 0


if __name__ == "__main__":
    exit(main())
