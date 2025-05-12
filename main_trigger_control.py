# #!/usr/bin/env python

# import rospy
# import serial
# import subprocess  # Allows running another ROS package
# from std_msgs.msg import Int16

# # Initialize ROS node
# rospy.init_node("serial_reader")

# # Publishers
# rpm_pub = rospy.Publisher("motor_rpm", Int16, queue_size=10)

# # Open Serial Port
# ser = serial.Serial('/dev/ttyACM0', 57600, timeout=1)
# ser.flush()

# # Threshold RPM to start the new ROS package
# THRESHOLD_RPM = 101
# package_started = False  # Ensure the package runs only once
# circuit_status = None  # Initialize circuit status variable
# subprocess.Popen([
#                             "roslaunch",
#                             "/home/lab_user/catkin_ws2/src/prophesee_ros_wrapper/prophesee_ros_driver/launch/prophesee_publisher.launch"
#                         ])
# while not rospy.is_shutdown():
    
#     if ser.in_waiting > 0:
#         line = ser.readline().decode('utf-8').strip()
        
#         try:
#             # **Extract Circuit Status Properly**
#             if "Circuit is" in line:
#                 status_str = line.split("Circuit is")[-1].strip()  # Extract the number part
#                 status_str = ''.join(c for c in status_str if c.isdigit())  # Keep only numbers

#                 if status_str:  # Ensure it's not empty
#                     circuit_status = int(status_str)  # Convert to integer
#                     rospy.loginfo(f"Circuit Status: {circuit_status}")
#                 else:
#                     rospy.logwarn(f"Failed to extract circuit status from: {line}")

#             # **Extract Circular Polarizing Filter RPM**
#             # elif "Circular Polarizing Filter RPM" in line:
#             #     try:
#             #         filter_rpm = float(line.split(":")[-1].strip())
#             #         rospy.loginfo(f"Circular Polarizing Filter RPM: {filter_rpm}")
#             elif "Revolutions per minute (RPM):" in line:
#                 try:
#                     filter_rpm = float(line.split(":")[-1].strip())  # Extract RPM
#                     rospy.loginfo(f"RPM from Circuit Timing: {filter_rpm}")

#                     # **Start Prophesee ROS package only if RPM is above threshold and circuit is OPEN (0)**
#                     if circuit_status == 0 and filter_rpm >= THRESHOLD_RPM and not package_started:
#                         rospy.loginfo("Threshold reached and Circuit is OPEN! Starting Prophesee ROS package...")

#                         # Run the launch file
#                         rosbag_process = subprocess.Popen([
#                             "rosbag", "record", "-a"
#                         ])

#                         package_started = True  # Ensure it starts only once

#                 except ValueError:
#                     rospy.logwarn(f"Invalid RPM data received: {line}")

#             # **Extract Motor Shaft RPM**
#             elif "Motor Shaft RPM" in line:
#                 try:
#                     motor_rpm = int(float(line.split(":")[-1].strip()))
#                     rpm_pub.publish(motor_rpm)  # Publish Motor RPM to ROS topic
#                 except ValueError:
#                     rospy.logwarn(f"Invalid Motor RPM received: {line}")

#         except Exception as e:
#             rospy.logwarn(f"Unexpected error processing data: {e}")














############################NEW UPDATED WITH RECORDING OF ONLY ONE ROTATION##################################################
#!/usr/bin/env python

# import rospy
# import serial
# import subprocess
# import signal
# from std_msgs.msg import Int16

# # Initialize ROS node
# rospy.init_node("serial_reader")

# # Publishers
# rpm_pub = rospy.Publisher("motor_rpm", Int16, queue_size=10)

# # Open Serial Port
# ser = serial.Serial('/dev/ttyACM0', 57600, timeout=1)
# ser.flush()

# # Threshold RPM to start the new ROS package
# THRESHOLD_RPM = 101
# package_started = False  # Ensure rosbag starts only once per condition
# rosbag_process = None  # Store rosbag process
# circuit_status = None  # Initialize circuit status variable

# # Start Prophesee ROS package at the beginning
# subprocess.Popen([
#     "roslaunch",
#     "/home/lab_user/catkin_ws2/src/prophesee_ros_wrapper/prophesee_ros_driver/launch/prophesee_publisher.launch"
# ])

# while not rospy.is_shutdown():
#     if ser.in_waiting > 0:
#         line = ser.readline().decode('utf-8').strip()
        
#         try:
#             # **Extract Circuit Status Properly**
#             if "Circuit is" in line:
#                 status_str = line.split("Circuit is")[-1].strip()
#                 status_str = ''.join(c for c in status_str if c.isdigit())  # Extract numeric part

#                 if status_str:
#                     new_circuit_status = int(status_str)
#                     rospy.loginfo(f"Circuit Status: {new_circuit_status}")

#                     # **Stop rosbag recording when circuit closes again**
#                     if package_started and new_circuit_status == 0 and rosbag_process is not None:
#                         rospy.loginfo("Circuit closed again! Stopping rosbag recording.")
#                         rosbag_process.send_signal(signal.SIGINT)  # Stop rosbag
#                         rosbag_process.wait()  # Ensure it fully stops
#                         rosbag_process = None  # Reset process variable
#                         package_started = False  # Reset so it can start again when needed

#                     circuit_status = new_circuit_status  # Update the status variable

#                 else:
#                     rospy.logwarn(f"Failed to extract circuit status from: {line}")

#             # **Extract Circular Polarizing Filter RPM**
#             elif "Revolutions per minute (RPM):" in line:
#                 try:
#                     filter_rpm = float(line.split(":")[-1].strip())  # Extract RPM
#                     rospy.loginfo(f"RPM from Circuit Timing: {filter_rpm}")

#                     # **Start rosbag if circuit is OPEN and RPM threshold is reached**
#                     if circuit_status == 0 and filter_rpm >= THRESHOLD_RPM and not package_started:
#                         rospy.loginfo("Threshold reached and Circuit is OPEN! Starting rosbag recording...")
#                         rosbag_process = subprocess.Popen(["rosbag", "record", "-a"])
#                         package_started = True  # Ensure it starts only once until it closes again

#                 except ValueError:
#                     rospy.logwarn(f"Invalid RPM data received: {line}")

#             # **Extract Motor Shaft RPM**
#             elif "Motor Shaft RPM" in line:
#                 try:
#                     motor_rpm = int(float(line.split(":")[-1].strip()))
#                     rpm_pub.publish(motor_rpm)  # Publish Motor RPM to ROS topic
#                 except ValueError:
#                     rospy.logwarn(f"Invalid Motor RPM received: {line}")

#         except Exception as e:
#             rospy.logwarn(f"Unexpected error processing data: {e}")



#################################STOP after 1 Second################################################################################################
import rospy
import serial
import subprocess
import signal
import threading
from std_msgs.msg import Int16

# Initialize ROS node
rospy.init_node("serial_reader")

# Publishers
rpm_pub = rospy.Publisher("motor_rpm", Int16, queue_size=10)

# Open Serial Port
ser = serial.Serial('/dev/ttyACM0', 57600, timeout=1)
ser.flush()

# Threshold RPM to start the new ROS package
THRESHOLD_RPM = 120
package_started = False
rosbag_process = None
circuit_status = None
LOWER_RPM = 120
UPPER_RPM = 121
# Start Prophesee ROS package at the beginning
subprocess.Popen([
    "roslaunch",
    "/home/lab_user/catkin_ws2/src/prophesee_ros_wrapper/prophesee_ros_driver/launch/prophesee_publisher.launch"
])

# Function to stop rosbag after 1 second
def stop_rosbag():
    global rosbag_process, package_started
    if rosbag_process:
        rospy.loginfo("1 second elapsed. Stopping rosbag recording.")
        rosbag_process.send_signal(signal.SIGINT)
        rosbag_process.wait()
        rosbag_process = None
        package_started = True

while not rospy.is_shutdown():
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').strip()
        
        try:
            # Extract Circuit Status
            if "Circuit is" in line:
                status_str = line.split("Circuit is")[-1].strip()
                status_str = ''.join(c for c in status_str if c.isdigit())

                if status_str:
                    circuit_status = int(status_str)
                    rospy.loginfo(f"Circuit Status: {circuit_status}")
                else:
                    rospy.logwarn(f"Failed to extract circuit status from: {line}")

            # Extract RPM
            elif "Circular Polarizing Filter RPM:" in line:
                try:
                    filter_rpm = float(line.split(":")[-1].strip())
                    filter_rpm= int(filter_rpm)
                    rospy.loginfo(f"RPM from Circuit Timing: {filter_rpm}")
                    

                    # Start rosbag if circuit is OPEN and RPM threshold is reached
                    if circuit_status == 0 and  LOWER_RPM <= filter_rpm <= UPPER_RPM and not package_started:
                        rospy.loginfo("Threshold reached and Circuit is OPEN! Starting rosbag recording...")
                        rosbag_process = subprocess.Popen(["rosbag", "record", "-a"])
                        package_started = True

                        # Set timer to stop rosbag after 1 second
                        threading.Timer(0.8, stop_rosbag).start()
                        # threading.Timer(2, stop_rosbag).start()

                except ValueError:
                    rospy.logwarn(f"Invalid RPM data received: {line}")

            # Extract Motor Shaft RPM
            elif "Motor Shaft RPM" in line:
                try:
                    motor_rpm = int(float(line.split(":")[-1].strip()))
                    rpm_pub.publish(motor_rpm)
                except ValueError:
                    rospy.logwarn(f"Invalid Motor RPM received: {line}")

        except Exception as e:
            rospy.logwarn(f"Unexpected error processing data: {e}")
