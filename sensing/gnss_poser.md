可能缺少 autoware_sensing_msgs::msg::GnssInsOrientationStamped


# 目的

## gnss_poser 是一个节点，用于订阅 GNSS 传感消息并计算带有协方差的车辆姿态。

该节点订阅 NavSatFix 消息，以发布 base_link 的姿态。NavSatFix 中的数据表示天线的位置。因此，它使用从 base_link 到天线位置的 tf 进行坐标转换。天线位置的 frame_id 是指 NavSatFix 的 header.frame_id。

如果无法获得从 base_link 到天线的转换，它将输出天线位置的姿态，而不进行坐标转换。


# 设置了gnss相关的frame_id 

设置了 base_frame 的 frame id 为 base_link 	，是车辆的基础坐标系，通常定义为车辆的中心或车辆的某个参考点。所有其他传感器和设备都以这个坐标系为基准进行安装和校准。

设置了 gnss_base_frame 的 frame id 为	gnss_base_link，表示GNSS天线的基准位置

设置了 map_frame 	的	frame id 为 	map 	，是全局地图的坐标系，用于表示车辆在地图上的位置。

use_gnss_ins_orientation 	设置为	true， 系统将使用GNSS/INS提供的方向信息来计算车辆姿态
