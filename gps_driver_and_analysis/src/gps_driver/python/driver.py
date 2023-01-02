import sys
import time
import utm
import rospy
import serial
import datetime
from gps_driver.msg import gps_msg

def gps_driver_pub(port_id):
    gps_data_pub = rospy.Publisher('/gps', gps_msg, queue_size=10)
    rospy.init_node('driver', anonymous=True)

    port = serial.Serial(port_id, 4800, timeout=1.0)
    
    gps_msg_obj = gps_msg()
    gps_msg_obj.Header.frame_id = "GPS1_Frame"
    gps_msg_obj.Header.seq = 0

    while not rospy.is_shutdown():
        line = port.readline()

        if line == "":
            rospy.logwarn("no data from the puck")

        else:
            if line.startswith(b"$GPGGA") or line.startswith(b"\r$GPGGA"):
                ll = line.split(b',')
                print(ll)

                if ll.count(b'') == 1 and len(ll) == 15:
                    time_stat, lat, lat_dir, lon, lon_dir, alt = ll[1], ll[2], ll[3], ll[4], ll[5], float(ll[9])
                    lat = float(lat[0:2]) + (float(lat[2:]) / 60)
                    lon = float(lon[0:3]) + (float(lon[3:]) / 60)
                    
                    lat_dir = lat_dir.decode("utf-8")
                    lon_dir = lon_dir.decode("utf-8")

                    if lat_dir == "S":
                        lat = -1 * lat

                    if lon_dir == "W":
                        lon = -1 * lon

                    today = datetime.date.today()
                    yr = int(today.year)
                    mn = int(today.month)
                    dt = int(today.day)
                    hr = int(time_stat[0:2])
                    mi = int(time_stat[2:4])
                    se = int(time_stat[4:6])
                    ds = int(time_stat[7:9]) * (10 ** 6) * 0.864 / 100 # 1 decimal second = 0.864 secs
                    ds = int(ds)

                    epoch_time = datetime.datetime(yr, mn, dt, hr, mn, se, ds).timestamp()
                    gps_msg_obj.Header.stamp = rospy.Time(epoch_time)
                    gps_msg_obj.Header.seq += 1

                    utm_data = utm.from_latlon(lat, lon)
                    gps_msg_obj.Latitude = lat
                    gps_msg_obj.Longitude = lon
                    gps_msg_obj.Altitude = alt
                    gps_msg_obj.UTM_easting = utm_data[0]
                    gps_msg_obj.UTM_northing = utm_data[1]
                    gps_msg_obj.Zone = utm_data[2]
                    gps_msg_obj.Letter = utm_data[3]

                    print(gps_msg_obj)
                    print()

                    gps_data_pub.publish(gps_msg_obj)
                else:
                    print('15 elements not found!')

if __name__ == "__main__":
    try:
        gps_driver_pub(sys.argv[1])
    except rospy.ROSInterruptException:
        pass
