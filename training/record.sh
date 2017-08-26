rosbag record -O ~/data/V79-$(date +"%Y-%m-%d_%H:%M:%S").bag \
/camera/image/compressed \
/car_velocity \
/car_rpm \
/bus_comm \
/car_command
