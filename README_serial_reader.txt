# Serial Reader ROS Node

## Overview

`serial_reader.py` is a ROS node that reads serial data from an Arduino (or similar device) on `/dev/ttyACM0`, parses circuit status and RPM information, publishes motor shaft RPM to a ROS topic, and triggers Prophesee event camera recording via `rosbag` when specific RPM thresholds and circuit conditions are met.

This script:

- Initializes a ROS node and a publisher for motor RPM.
- Opens a serial connection at 57600 baud.
- Continuously reads incoming serial lines.
- Parses three types of messages:
  - **Circuit Status**: Extracts the integer status from lines containing "Circuit is".
  - **Polarizer RPM**: Reads "Circular Polarizing Filter RPM" and starts a `rosbag record` session when conditions are met, stopping after ~1 second.
  - **Motor Shaft RPM**: Extracts motor RPM and publishes it to the `motor_rpm` topic.
- Launches the Prophesee ROS driver on startup.


## Features

- **Serial Communication**: Robust parsing of numeric data from textual serial lines.
- **Conditional Recording**: Automatically starts and stops `rosbag` based on circuit status and RPM range.
- **ROS Integration**: Publishes motor RPM to `/motor_rpm`, facilitating downstream processing.


## Prerequisites

- **ROS Noetic** (or compatible ROS distribution)
- Python 3.x
- Packages:
  - `rospy`
  - `serial` (`pyserial`)
  - `std_msgs`
- Prophesee ROS wrapper installed in your workspace:
  ```bash
  cd ~/catkin_ws2/src
  git clone https://github.com/prophesee/prophesee_ros_wrapper.git
  cd ..
  catkin_make
  ```
- Ensure your serial device is available at `/dev/ttyACM0`. Adjust if needed.


## Installation

1. **Clone your workspace** (if not already):
   ```bash
   cd ~/catkin_ws2/src
   git clone <your-repo-url>
   cd ../
   catkin_make
   source devel/setup.bash
   ```

2. **Place `serial_reader.py`** inside a ROS package (e.g., `scripts/` folder) and make it executable:
   ```bash
   chmod +x scripts/serial_reader.py
   ```


## Usage

1. **Launch ROS Master**:
   ```bash
   roscore
   ```
2. **Run the Serial Reader Node**:
   ```bash
   rosrun <your_package> serial_reader.py
   ```
3. **Monitor Topics**:
   ```bash
   rostopic echo /motor_rpm
   ```

The node will automatically launch the Prophesee driver and record a single ~1-second `rosbag` when the polarizer RPM is between **120** and **121** and the circuit status is `0` (OPEN).


## Configuration

You can adjust key parameters directly in the script:

| Variable         | Description                                      | Default | Location in Code     |
|------------------|--------------------------------------------------|---------|----------------------|
| `PORT`           | Serial port device                               | `/dev/ttyACM0` | Line with `serial.Serial(...)` |
| `BAUD_RATE`      | Serial communication speed (bps)                 | `57600` | Same line as above   |
| `THRESHOLD_RPM`  | RPM threshold to trigger recording               | `120`   | `THRESHOLD_RPM`      |
| `LOWER_RPM`      | Lower bound of RPM window for recording          | `120`   | `LOWER_RPM`          |
| `UPPER_RPM`      | Upper bound of RPM window for recording          | `121`   | `UPPER_RPM`          |
| `RECORD_DURATION`| Duration (seconds) to record rosbag (via timer)  | `0.8`   | `threading.Timer(0.8...)` |


## How It Works

1. **Initialization**:
   - ROS node `serial_reader` starts.
   - Serial port opens and flushes.
   - Prophesee ROS driver is launched.

2. **Reading Loop**:
   - On each incoming line from serial:
     - If it contains "Circuit is", update `circuit_status`.
     - If it contains "Circular Polarizing Filter RPM:", parse RPM and, if conditions met (`circuit_status == 0` and RPM in range), start a `rosbag record -a` process and schedule its stop.
     - If it contains "Motor Shaft RPM", publish the extracted RPM to `/motor_rpm`.

3. **Recording Control**:
   - `rosbag_process` is started once per condition.
   - A timer stops recording after the configured duration.


## Troubleshooting

- **No Serial Data**:
  - Verify cable connection and device path (`ls /dev/tty*`).
  - Check baud rate matches your Arduino sketch.

- **Recording Not Triggering**:
  - Confirm printed RPM values in terminal logs.
  - Adjust `THRESHOLD_RPM`, `LOWER_RPM`, or `UPPER_RPM` as needed.

- **Permissions Error**:
  - Add your user to the `dialout` group:
    ```bash
    sudo usermod -aG dialout $USER
    ```


## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.


## Author

Shahid Shabeer Malik – Graduate Research Assistant, SLUAIR Lab, Saint Louis University

Contact: [shahid.malik@slu.edu](mailto:shahid.malik@slu.edu)

---

*Generated on May 12, 2025.*
