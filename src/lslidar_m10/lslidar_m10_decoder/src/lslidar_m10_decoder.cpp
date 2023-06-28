/*
 * This file is part of lslidar_m10 driver.
 *
 * The driver is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the driver.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <lslidar_m10_decoder/lslidar_m10_decoder.h>

using namespace std;

namespace lslidar_m10_decoder {
LslidarM10Decoder::LslidarM10Decoder(
        ros::NodeHandle& n, ros::NodeHandle& pn):
    nh(n),
    pnh(pn),
    last_degree(0.0),
	scan_size(16),
	idx(0){
	return;
	}

bool LslidarM10Decoder::loadParameters() {
    pnh.param<int>("versions", versions, 2);
    pnh.param<string>("child_frame_id", child_frame_id, "laser_link");
	pnh.param<string>("scan_topic", scan_topic_, "scan");

	pnh.param<double>("/m10/min_distance",min_distance,0);
	pnh.param<double>("/m10/max_distance",max_distance,30);
	pnh.param("/m10/truncated_mode", truncated_mode_, 0);
	pnh.param<std::vector<int>>("/m10/disable_min", disable_angle_min_range, disable_angle_range_default);
	pnh.param<std::vector<int>>("/m10/disable_max", disable_angle_max_range, disable_angle_range_default);
	
	angle_able_min = 0;
	angle_able_max = 360;
	count_num = 0 ;
	scan_points_.resize(2000);

	if (versions == 1)
	{
		lidar_frequency = 10;
		package_points = 42;
		data_bits_start = 6;
		degree_bits_start = 2;
		rpm_bits_start = 4;
		point_num = 1008;
	}
	else if (versions == 2)
	{
		lidar_frequency = 12;
		package_points = 70;
		data_bits_start = 8;
		degree_bits_start = 4;
		rpm_bits_start = 6;
		point_num = 1680;
	}
	else if (versions == 3)
	{
		lidar_frequency = 12;
		package_points = 70;
		data_bits_start = 8;
		degree_bits_start = 4;
		rpm_bits_start = 6;
		point_num = 1680;
	}

    return true;

}

bool LslidarM10Decoder::createRosIO() {
	difop_sub_ = nh.subscribe("lslidar_packet_difop", 10, &LslidarM10Decoder::processDifop, (LslidarM10Decoder*)this);
    packet_sub = nh.subscribe<lslidar_m10_msgs::LslidarM10Packet>("lslidar_packet", 200, &LslidarM10Decoder::packetCallback, this, ros::TransportHints().tcpNoDelay(true));
    scan_pub = nh.advertise<sensor_msgs::LaserScan>(scan_topic_, 200);
	device_pub = nh.advertise<lslidar_m10_msgs::LslidarM10Difop>("difop_information", 200);

    return true;
}

bool LslidarM10Decoder::initialize() {
    if (!loadParameters()) {
        ROS_ERROR("Cannot load all required parameters...");
        return false;
    }

    if (!createRosIO()) {
        ROS_ERROR("Cannot create ROS I/O...");
        return false;
    }

    return true;
}

void LslidarM10Decoder::processDifop(const lslidar_m10_msgs::LslidarM10Packet::ConstPtr& difop_msg)
{
	  const uint8_t* data = &difop_msg->data[0];
	  
	  Difop_data = lslidar_m10_msgs::LslidarM10DifopPtr(
							new lslidar_m10_msgs::LslidarM10Difop());
							
	  if (data[0] != 0xa5 || data[1] != 0xff || data[2] != 0x00 || data[3] != 0x5a)
	  {
		return;
	  }

	 Difop_data->rpm = (data[20]<<8) | data[21];
	 
	 device_pub.publish(Difop_data);
}

void LslidarM10Decoder::publishScan()
{
    sensor_msgs::LaserScan::Ptr scan(new sensor_msgs::LaserScan);
	
    scan->header.frame_id = child_frame_id;
    scan->header.stamp = ros::Time::now();  // timestamp will obtained from sweep data stamp
	if(count_num <= 1)	return;
	count_num = point_num;
	int scan_num = count_num+1;
    scan->time_increment = 1/lidar_frequency/(double)(count_num - 1);
    scan->scan_time= 1/lidar_frequency;

	scan->angle_min = - M_PI;
    scan->angle_max =  M_PI;

    scan->angle_increment = 2 * M_PI / count_num;
    scan->range_min = min_distance;
    scan->range_max = max_distance;
    scan->ranges.reserve(scan_num);
    scan->ranges.assign(scan_num, std::numeric_limits<float>::infinity());
    scan->intensities.reserve(scan_num);
    scan->intensities.assign(scan_num, std::numeric_limits<float>::infinity());

	int start_num = floor(angle_able_min * count_num / 360);
	int end_num = floor(angle_able_max * count_num / 360);
	for(uint16_t i = 0; i < count_num; i++)
	{
		int point_idx = round((360 - scan_points_[i].degree) * count_num / 360);
		
		if(point_idx<(end_num-count_num))
			point_idx += count_num;
		point_idx =  point_idx - start_num;
		
		if(point_idx < 0 || point_idx >= scan_num) 
			continue;
		if (scan_points_[i].range == 0.0) {
			scan->ranges[point_idx] = std::numeric_limits<float>::infinity();
		}
		else{
			scan->ranges[point_idx] = scan_points_[i].range;
		}
		if(truncated_mode_==1)
		{
			for (int j = 0; j < disable_angle_max_range.size(); ++j) 
			{
				if ((scan_points_[i].degree >= disable_angle_min_range[j] ) && (scan_points_[i].degree <= disable_angle_max_range[j] ))
				{
	   			scan->ranges[point_idx] = std::numeric_limits<float>::infinity();
				}
			}
		}
		scan->intensities[point_idx] = scan_points_[i].intensity;
	}
	count_num = 0;

    scan_pub.publish(scan);
}

void LslidarM10Decoder::packetCallback(
        const lslidar_m10_msgs::LslidarM10PacketConstPtr& msg) {

	 Difop_data = lslidar_m10_msgs::LslidarM10DifopPtr(new lslidar_m10_msgs::LslidarM10Difop());
	if (msg->data[0] != 0xA5 || msg->data[1] != 0x5A) return;

	TwoBytes raw_degree;
	raw_degree.bytes[0] = msg->data[degree_bits_start + 1];
	raw_degree.bytes[1] = msg->data[degree_bits_start];
	degree = raw_degree.distance/100.f;
	TwoBytes raw_rpm;
	raw_rpm.bytes[0] = msg->data[rpm_bits_start + 1];
	raw_rpm.bytes[1] = msg->data[rpm_bits_start];
	if(raw_rpm.bytes[1] != 0x00 || raw_rpm.bytes[0] != 0x00)
	{
		Difop_data->rpm = 2500000/raw_rpm.distance;
		device_pub.publish(Difop_data);
	}
	
	TwoBytes raw_range;
	int invalidValue = 0;

	for (size_t num = 0; num < ( package_points*2); num+=2)
	{
		int z = msg->data[num + data_bits_start + 1];
		int s = msg->data[num + data_bits_start];
		
		int dist_temp = s & 0x7F;
		int inten_temp = s & 0x80;
		if ((s * 256 + z) != 65535)
		{
			scan_points_[idx].range = double(dist_temp * 256 + (z)) / 1000.f;
			if (inten_temp)	scan_points_[idx].intensity = 255;
			else 			scan_points_[idx].intensity = 0;
			idx++;
			count_num++;
		}
		else
		{
			invalidValue++;
		}
	}

	invalidValue = package_points - invalidValue;

	for (size_t i = 0; i < invalidValue; i++)
	{
		if ((degree + (15.0 / invalidValue * i)) > 360.0)
			scan_points_[idx-invalidValue+i].degree = degree + (15.0 / invalidValue * i) - 360.0;
		else
			scan_points_[idx-invalidValue+i].degree = degree + (15.0 / invalidValue * i);
	}

	if (degree < last_degree || idx >=point_num) 	
	{

		idx = 0;
		for(int k=0;k<scan_points_.size();k++)
		{	
			if(scan_points_[k].range < min_distance || scan_points_[k].range > max_distance)
				scan_points_[k].range = 0;
		}

		publishScan();

		for(int k=0; k<scan_points_.size(); k++)
		{
			scan_points_[k].range = 0;
			scan_points_[k].degree = 0;
		}
	}
	last_degree = degree;

    return;
}

} 

