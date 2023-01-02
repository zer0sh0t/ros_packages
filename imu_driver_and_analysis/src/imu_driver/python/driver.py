import sys
import time
import rospy
import serial
import datetime
import numpy as np
from imu_driver.msg import imu_msg

def euler_to_quat(yaw, pitch, roll):
    yaw = yaw * (np.pi / 180)
    pitch = pitch * (np.pi / 180)
    roll = roll * (np.pi / 180)

    x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return (x, y, z, w)

def imu_driver_pub(port_id):
    imu_data_pub = rospy.Publisher('/imu', imu_msg, queue_size=10)
    rospy.init_node('driver', anonymous=True)

    port = serial.Serial(port_id, 115200, timeout=1.0)
    port.write(b"$VNWRG,07,40*XX")
    port.write(b"$VNWRG,06,14*XX")
    
    imu_msg_obj = imu_msg()
    imu_msg_obj.Header.frame_id = "IMU1_Frame"
    imu_msg_obj.Header.seq = 0

    while not rospy.is_shutdown():
        line = port.readline()

        if line == "":
            rospy.logwarn("no data from the puck")

        else:
            if line.startswith(b"$VNYMR") or line.startswith(b"\r$VNYMR"):
                ll = line.split(b',')
                print(ll)

                if len(ll) == 13:
                    yaw, pitch, roll, mag_x, mag_y, mag_z, accel_x, accel_y, accel_z, gyro_x, gyro_y = [float(ll[i]) for i in range(1, len(ll) - 1)]
                    gyro_z = float(ll[-1].decode("utf-8").split('*')[0])
                    x, y, z, w = euler_to_quat(yaw, pitch, roll)

                    imu_msg_obj.Header.stamp = rospy.Time.now()
                    imu_msg_obj.Header.seq += 1

                    imu_msg_obj.IMU.orientation.x = x
                    imu_msg_obj.IMU.orientation.y = y
                    imu_msg_obj.IMU.orientation.z = z
                    imu_msg_obj.IMU.orientation.w = w

                    imu_msg_obj.IMU.angular_velocity.x = gyro_x
                    imu_msg_obj.IMU.angular_velocity.y = gyro_y
                    imu_msg_obj.IMU.angular_velocity.z = gyro_z

                    imu_msg_obj.IMU.linear_acceleration.x = accel_x
                    imu_msg_obj.IMU.linear_acceleration.y = accel_y
                    imu_msg_obj.IMU.linear_acceleration.z = accel_z

                    imu_msg_obj.MagField.magnetic_field.x = mag_x
                    imu_msg_obj.MagField.magnetic_field.y = mag_y
                    imu_msg_obj.MagField.magnetic_field.z = mag_z

                    imu_msg_obj.raw_data = line.decode("utf-8")
                    print(imu_msg_obj)
                    print()

                    imu_data_pub.publish(imu_msg_obj)
                else:
                    print('13 elements not found!')

if __name__ == "__main__":
    try:
        imu_driver_pub(sys.argv[1])
    except:
        pass
