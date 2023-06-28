/*******************************************************
@company: Copyright (C) 2021, Leishen Intelligent System
@product: LSM10
@filename: lsm10.cpp
@brief:
@version:       date:       author:     comments:
@v1.0           21-2-4      yao          new
*******************************************************/
#include "lsm10_v2/lsm10.h"
#include <stdio.h>
#include <signal.h> 
#include<lsm10_v2/difop.h>

namespace ls{
LSM10 * LSM10::instance()
{
  static LSM10 obj;
  return &obj;
}

LSM10::LSM10()
{
  int code = 0;
  initParam();
  pub_ = n_.advertise<sensor_msgs::LaserScan>(scan_topic_, 3);
  device_pub = n_.advertise<lsm10_v2::difop>("difop_information", 100);
  difop_switch = n_.subscribe<std_msgs::Int8>("lslidar_order", 1 ,&LSM10::order,this);

  serial_ = LSIOSR::instance(serial_port_, baud_rate_);
  code = serial_->init();
  if(code != 0)
  {
	  printf("open_port %s ERROR !\n", serial_port_.c_str());
	  ros::shutdown();
	  exit(0);
  }
  printf("open_port %s  OK !\n", serial_port_.c_str());

  recv_thread_ = new boost::thread(boost::bind(&LSM10::recvThread, this));
  pubscan_thread_ = new boost::thread(boost::bind(&LSM10::pubScanThread, this));
  open();
}

LSM10::~LSM10()
{
  printf("start LSM10::~LSM10()\n");

  is_shutdown_ = true;

  pubscan_thread_->interrupt();
  pubscan_thread_->join();
  pubscan_thread_ = NULL;
  delete pubscan_thread_;

  recv_thread_->interrupt();
  recv_thread_->join();

  recv_thread_ = NULL;
  delete recv_thread_;

  serial_->close();
  serial_ = NULL;
  delete serial_;
  printf("end LSM10::~LSM10()\n");
}

void LSM10::initParam()
{
  std::string scan_topic = "/scan";
  std::string frame_id = "laser_link";
  std::string port = "/dev/ttyUSB0";
  ros::NodeHandle nh("~");
  nh.param("versions", versions, 2);
  nh.param("scan_topic", scan_topic_, scan_topic);
  nh.param("frame_id", frame_id_, frame_id);
  nh.param("serial_port", serial_port_, port);
  nh.param<double>("/m10/min_distance",min_distance,0);
  nh.param<double>("/m10/max_distance",max_distance,30);
  nh.param("/m10/truncated_mode", truncated_mode_, 0);
  nh.param<std::vector<int>>("/m10/disable_min", disable_angle_min_range, disable_angle_range_default);
  nh.param<std::vector<int>>("/m10/disable_max", disable_angle_max_range, disable_angle_range_default);
  angle_able_min = 0.0;
  angle_able_max = 360.0;
  is_shutdown_ = false;
  count_num = 0;
  data_len_ = 180;

  scan_points_.resize(2000);
  
  is_start = true;
  
  if (versions == 1){
	PACKET_SIZE = 92;
	package_points = 42;
	data_bits_start = 6;
	degree_bits_start = 2;
	rpm_bits_start = 4;
	baud_rate_= 460800;
	points_size_ = 1008;
	printf("agreement is 1.1.0 \n");
  }
  else if (versions == 2){
	PACKET_SIZE = 160;
	package_points = 70;
	data_bits_start = 8;
	degree_bits_start = 4;
	rpm_bits_start = 6;
	baud_rate_=500000;
  	points_size_ = 1680;
	printf("agreement is 1.3.2 \n");
  }
  else if (versions == 3){
	PACKET_SIZE = 162;
	package_points = 70;
	data_bits_start = 8;
	degree_bits_start = 4;
	rpm_bits_start = 6;
	baud_rate_=500000;
  	points_size_ = 1680;
	printf("agreement is 1.4.0 \n");
  }

}

int LSM10::getScan(std::vector<ScanPoint> &points, ros::Time &scan_time, float &scan_duration)
{
  boost::unique_lock<boost::mutex> lock(mutex_);
  points.assign(scan_points_bak_.begin(), scan_points_bak_.end());
  scan_time = pre_time_;
  scan_duration = (time_ - pre_time_).toSec();
}

void LSM10::open()
{ 
    for(int k = 0 ; k <10 ; k++)
    {   
		int rtn;
		if(versions == 1){
			char data[188]= {0x00};
			data[0] = 0xA5;
			data[1] = 0x5A;
			data[184] = 0x01;
			data[185] = 0x01;
			data[186] = 0xFA;
			data[187] = 0xFB;
			rtn = serial_->send((const char*)data, 188);
		}
		else if (versions == 2){
			char data[188]= {0x00};
            data[0] = 0xA5;
            data[1] = 0x5A;
            data[2] = 0x01;
            data[184] = 0x01;
            data[185] = 0x01;
            data[186] = 0xFA;
            data[187] = 0xFB;
			rtn = serial_->send((const char*)data, 188);
		}
		else if (versions == 3){
			char data[11]= {0x00};
			data[0] = 0xA5;
			data[1] = 0x5A;
			data[3] = 0x0B;
			data[4] = 0x55;
			data[5] = 0x12;
			data[6] = 0x02;
			data[7] = 0x01;
			data[8] = 0x74;
			data[9] = 0xFA;
			data[10] = 0xFB;
			rtn = serial_->send((const char*)data, 11);
		}
		if (rtn < 0)
			printf("start scan error !\n");
        else 
        {
            usleep(3000000);
        	is_start = true;
            return ;
        }   
    }
  
    return ;
}
void LSM10::order(const std_msgs::Int8 msg) {
    int i = msg.data;
    for(int k = 0 ; k <10 ; k++)
    {   
		int rtn;
		if(versions == 1){
			char data[188]= {0x00};
			data[0] = 0xA5;
			data[1] = 0x5A;
			if (i == 0)						//雷达停转
				data[184] = 0x01;
			else if (i == 1){				//雷达启转
				data[184] = 0x01;
				data[185] = 0x01;
			}
			if (i == 2){						//雷达点云不滤波
				data[181] = 0x0A;
				data[184] = 0x06;
				if(is_start) data[185] = 0x01;
			}
			else if (i == 3){				//雷达点云正常滤波
				data[181] = 0x0B;
				data[184] = 0x06;
				if(is_start) data[185] = 0x01;
			}
			else if (i == 4){				//雷达近距离滤波
				data[181] = 0x0C;
				data[184] = 0x06;
				if(is_start) data[185] = 0x01;
			}        
			data[186] = 0xFA;
			data[187] = 0xFB;
			rtn= serial_->send((const char*)data, 188);
		}
		if(versions == 2){
			char data[188]= {0x00};
            data[0] = 0xA5;
            data[1] = 0x5A;
            data[2] = 0x01;
            data[184] = 0x01;
            if(i == 1)      data[185] = 0x01;
            else            data[185] = 0x00;
            data[186] = 0xFA;
            data[187] = 0xFB;
			rtn= serial_->send((const char*)data, 188);
		}
		else if (versions == 3){
			char data[11]= {0x00};
			data[0] = 0xA5;
			data[1] = 0x5A;
			data[3] = 0x0B;
			data[4] = 0x55;
			data[5] = 0x12;
			data[6] = 0x02;
			if(i == 1)      
			{
				data[7] = 0x01;
				data[8] = 0x74;
			}
			else               
			{
				data[7] = 0x00;
				data[8] = 0x73;
			}
			data[9] = 0xFA;
			data[10] = 0xFB;
			rtn= serial_->send((const char*)data, 11);
		}
		if (rtn < 0)
			printf("start scan error !\n");
        else{
            if(i == 1)  usleep(3000000);
            if(i == 0)  is_start = false;
            if(i == 1)  is_start = true;
            return ;
        }   
    }
    return ;
}

int LSM10::getVersion(std::string &version)
{
  version = "lsm10_v1_0";
  return 0;
}


void LSM10::recvThread()
{
  unsigned char * packet_bytes = new unsigned char[data_len_];
  int idx = 0;
  int link_time = 0;
  double degree;
  double last_degree = 0.0;
  
  boost::posix_time::ptime t1,t2;
  t1 = boost::posix_time::microsec_clock::universal_time();
  
  while(!is_shutdown_&&ros::ok()){

	int count = serial_->read(packet_bytes, PACKET_SIZE);
	
	if(count <= 0) 
		link_time++;
	else
		link_time = 0;
	
	if(link_time > 10000)
	{
		serial_->close();
		int ret = serial_->init();
		if(ret < 0)
		{
			ROS_WARN("serial open fail");
			usleep(200000);
		}
		link_time = 0;
	}
  
	
	for (int i = 0; i < count; i++)
	{
		int k = packet_bytes[i];
		int y = packet_bytes[i + 1];

		int k_1 = packet_bytes[i + 2];
		int y_1 = packet_bytes[i + 3];
		
		if (k == 0xA5 && y == 0x5A)					 //应答距离
		{
			if(i != 0)
			{
				memcpy(packet_bytes, packet_bytes + i, PACKET_SIZE - i);
				serial_->read(packet_bytes + PACKET_SIZE - i, i);
			}
			
			int s = packet_bytes[i + degree_bits_start];
			int z = packet_bytes[i + degree_bits_start + 1];
			
			boost::unique_lock<boost::mutex> lock(mutex_);

			//if ((s * 256 + z) / 100.f > 360)
			//	degree = 0;
			//else
				degree = (s * 256 + z) / 100.f;
				degree = (degree > 360) ? degree-360 : degree;
			//转速
			lsm10_v2::difopPtr Difop_data = lsm10_v2::difopPtr(
						new lsm10_v2::difop());
			s = packet_bytes[i + rpm_bits_start];
			z = packet_bytes[i + rpm_bits_start + 1];
			

			if(s != 0 || z != 0)
			{
				Difop_data->MotorSpeed = float(2500000.0 / (s * 256 + z));
				//printf("s= %d z = %d MotorSpeed = %d\n",s,z,Difop_data->MotorSpeed);
				device_pub.publish(Difop_data);
			}	

			int invalidValue = 0;
			for (size_t num = 0; num < 2*package_points; num+=2)
			{
				int s = packet_bytes[i + num + data_bits_start];
				int z = packet_bytes[i + num + data_bits_start + 1];
				
				int dist_temp = s & 0x7F;
				int inten_temp = s & 0x80;

				if ((s * 256 + z) != 0xFFFF)
				{	
					scan_points_[idx].range = double(dist_temp * 256 + (z)) / 1000.f;
					if (inten_temp)	scan_points_[idx].intensity = 255;
					else 	scan_points_[idx].intensity = 0;
					idx++;
					count_num++;
				}
				else
					invalidValue++;
			}

			invalidValue = package_points - invalidValue;

			for (size_t i = 0; i < invalidValue; i++)
			{
				if ((degree + (15.0 / invalidValue * i)) > 360)
					scan_points_[idx-invalidValue+i].degree = degree + (15.0 / invalidValue * i) - 360;
				else
					scan_points_[idx-invalidValue+i].degree = degree + (15.0 / invalidValue * i);
			}
			
			lock.unlock();
			
			//if (degree > 359.5) 
			if (degree < last_degree || idx>=points_size_) 	
			{
				idx = 0;
				// printf ("degree = %f\n",degree);
				// printf ("last_degree = %f\n",last_degree);
				boost::unique_lock<boost::mutex> lock(mutex_);
				scan_points_bak_.resize(scan_points_.size());
				scan_points_bak_.assign(scan_points_.begin(), scan_points_.end());
				for(int k=0; k<scan_points_.size(); k++)
				{
					scan_points_[k].range = 0;
					scan_points_[k].degree = 0;
				}
				pre_time_ = time_;
				lock.unlock();
				pubscan_cond_.notify_one();
				time_ = ros::Time::now();
			}
			last_degree = degree;
			
		}	
	}
  }
  if (packet_bytes)
  {
    packet_bytes = NULL;
    delete packet_bytes;
  }
}

void LSM10::pubScanThread()
{
  bool wait_for_wake = true;
  boost::unique_lock<boost::mutex> lock(pubscan_mutex_);

  while (ros::ok() && !is_shutdown_)
  {
    while (wait_for_wake)
    {
      pubscan_cond_.wait(lock);
      wait_for_wake = false;
    }
	if(count_num <= 1 )
		continue;
    std::vector<ScanPoint> points;
    ros::Time start_time;
    count_num=points_size_;
    float scan_time;
    this->getScan(points, start_time, scan_time);
	int scan_num = ceil((angle_able_max-angle_able_min)/360*count_num)+1;
	//printf("scan_num = %d\n",scan_num);
	//if (count_num > points_size_)	printf("count_num = %d\n",count_num);
    sensor_msgs::LaserScan msg;
    msg.header.frame_id = frame_id_;
    msg.header.stamp = start_time;
    msg.angle_min = -M_PI;
    msg.angle_max = M_PI;
	msg.angle_increment = 2 * M_PI / count_num;
    msg.range_min = min_distance;
    msg.range_max = max_distance;
    msg.ranges.resize(scan_num);
    msg.intensities.resize(scan_num);
    msg.scan_time = scan_time;
    msg.time_increment = scan_time / (double)(count_num - 1);
	
	for(int k=0; k<scan_num; k++){
		msg.ranges[k] = std::numeric_limits<float>::infinity();
        msg.intensities[k] = 0;
	}

	int start_num = floor(angle_able_min * count_num / 360);
	int end_num = floor(angle_able_max * count_num / 360);

	for (int i = 0; i < count_num; i++) {
		int point_idx = round((360-points[i].degree) * count_num / 360);
		if(point_idx<(end_num-count_num))
			point_idx += count_num;
		point_idx =  point_idx - start_num;
		if(point_idx < 0 || point_idx >= scan_num) 
			continue;


		if (points[i].range == 0.0) {
		msg.ranges[point_idx] = std::numeric_limits<float>::infinity();
		msg.intensities[point_idx] = 0;
		}
		else if(points[i].range<min_distance||points[i].range>max_distance){
			msg.ranges[point_idx] = std::numeric_limits<float>::infinity();
		msg.intensities[point_idx] = 0;
		}
		else {
			double dist = points[i].range;
			msg.ranges[point_idx] = (float) dist;
		}
		if(truncated_mode_==1)
		{
			for (int j = 0; j < disable_angle_max_range.size(); ++j) {
		    if ((points[i].degree <= (360-disable_angle_min_range[j]) ) && (points[i].degree >= (360-disable_angle_max_range[j]))) {
		        msg.ranges[point_idx] = std::numeric_limits<float>::infinity();
		        msg.intensities[point_idx] = 0;
		      }
		    }
		}


	msg.intensities[point_idx] = points[i].intensity;
    }

	count_num = 0;
    if(is_start) pub_.publish(msg);
    wait_for_wake = true;
  }
}

}

void handleSig(int signo)
{
  printf("handleSig\n");
  ros::shutdown();
  exit(0);
}

int main(int argv, char **argc)
{
  signal(SIGINT, handleSig);
  signal(SIGTERM, handleSig);
  ros::init(argv, argc, "lsm10");
 
  ls::LSM10* lsm10 = ls::LSM10::instance();

  ros::spin();
  return 0;
}
