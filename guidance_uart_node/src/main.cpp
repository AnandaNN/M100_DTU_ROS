#include <stdio.h>
#include <string.h>

#include <iostream>
#include <chrono>
#include <thread>

#include "serial.h"
#include "crc32.h"
#include "protocal_uart_sdk.h"
#include "DJI_guidance.h"

#include <ros/ros.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float32MultiArray.h>

#define UART 1
#define CAMERA_PAIR_NUM 5

#define UART_PORT 0

int main(int argc, char** argv)
{
	if ( connect_serial( UART_PORT ) < 0 )
	{
        printf( "connect serial error\n" );
        return 0;
	}

	ros::init(argc, argv, "guidance_uart_node");
  	ros::NodeHandle nh;

  	ros::Publisher voVelocityPub = nh.advertise<geometry_msgs::Vector3>("/guidance/vo_velocity", 0);
  	ros::Publisher accelerationPub = nh.advertise<geometry_msgs::Vector3>("/guidance/acceleration", 0);
  	ros::Publisher quaternionPub = nh.advertise<geometry_msgs::Quaternion>("/guidance/quaternion", 0);
  	ros::Publisher ultrasonicPub = nh.advertise<std_msgs::Float32MultiArray>("/guidance/ultrasonic", 0);

  	geometry_msgs::Vector3 voVelocityMsg;
  	geometry_msgs::Vector3 accelerationMsg;
  	geometry_msgs::Quaternion quaterionMsg;
  	std_msgs::Float32MultiArray ultrasonicMsg;
  	
  	// ultrasonicMsg.layout.dim[0].size = 5;
  	// ultrasonicMsg.layout.dim[0].label = "ultrasonic";
  	// ultrasonicMsg.layout.dim[0].stride = 1;
  	// ultrasonicMsg.layout.data_offset = 0;
  	ultrasonicMsg.data = {0,0,0,0,0};

  	std::cout << "ultrasonic set" << std::endl;

  	// for(int i = 0; i < 5; i++) ultrasonicMsg.data.pushback

	// std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	// std::chrono::steady_clock::time_point end;
	// for ( int i = 0; i < 100; ++i )
	while(ros::ok())
	{
		
		
		unsigned char data[200] = {0};
		// int max_size = (int)sizeof(data);
		int max_size = 100;
		int timeout = 1;

		// begin = std::chrono::steady_clock::now();
		int n = read_serial( data, max_size, timeout);
		// end = std::chrono::steady_clock::now();
		// std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
		
		if( n <= 0 )
		{
			continue;
		}

		// std::cout << "n = " << n << std::endl;

		// continue;

		// std::cout << "Data size = " << sizeof(data) << std::endl;

		push( data, sizeof(data) );

		bool got_data = 0;

		for ( ; ; )
		{
			
			unsigned int len = 0;
			int has_packet = pop( data, len );
			

			if ( has_packet )
			{
				// printf("HAS PACKET Len = %d\n", (int)len);
				if ( len )
				{
					unsigned char cmd_id = data[1];
					if ( e_imu == cmd_id )
					{
						imu imu_data;
						memcpy( &imu_data, data + 2, sizeof(imu_data) );
						// printf( "imu:%f %f %f,%f %f %f %f\n", imu_data.acc_x, imu_data.acc_y, imu_data.acc_z, 
							     // imu_data.q[0], imu_data.q[1], imu_data.q[2], imu_data.q[3] );
						// printf( "frame index:%d,stamp:%d\n", imu_data.frame_index, imu_data.time_stamp );
						// printf( "\n" );

						accelerationMsg.x = imu_data.acc_x;
						accelerationMsg.y = imu_data.acc_y;
						accelerationMsg.z = imu_data.acc_z;

						accelerationPub.publish(accelerationMsg);

						quaterionMsg.x = imu_data.q[0];
						quaterionMsg.y = imu_data.q[1];
						quaterionMsg.z = imu_data.q[2];
						quaterionMsg.w = imu_data.q[3];

						quaternionPub.publish(quaterionMsg);

						got_data = true;
					}
					if ( e_ultrasonic == cmd_id )
					{
						ultrasonic_data ultrasonic;
						memcpy( &ultrasonic, data + 2, sizeof(ultrasonic) );
						// for ( int d = 0; d < CAMERA_PAIR_NUM; ++d )
						// {
							// printf( "distance:%f,reliability:%d\n", ultrasonic.ultrasonic[d] * 0.001f, (int)ultrasonic.reliability[d] );
						// }
						// printf( "frame index:%d,stamp:%d\n", ultrasonic.frame_index, ultrasonic.time_stamp );
						// printf( "\n" );
						ultrasonicMsg.data[0] = 0.001*ultrasonic.ultrasonic[0];
						ultrasonicMsg.data[1] = 0.001*ultrasonic.ultrasonic[1];
						ultrasonicMsg.data[2] = 0.001*ultrasonic.ultrasonic[2];
						ultrasonicMsg.data[3] = 0.001*ultrasonic.ultrasonic[3];
						ultrasonicMsg.data[4] = 0.001*ultrasonic.ultrasonic[4];

						ultrasonicPub.publish(ultrasonicMsg);

						got_data = true;
					}
					if ( e_velocity == cmd_id )
					{
						velocity vo;
						soc2pc_vo_can_output output;
						memcpy( &output, data + 2, sizeof(vo) );
						vo.vx = output.m_vo_output.vx;
						vo.vy = output.m_vo_output.vy;
						vo.vz = output.m_vo_output.vz;
						voVelocityMsg.x = (float) vo.vx;
						voVelocityMsg.y = (float) vo.vy;
						voVelocityMsg.z = (float) vo.vz;
						voVelocityPub.publish(voVelocityMsg);
						// printf( "vx:%f vy:%f vz:%f\n", 0.001f * vo.vx, 0.001f * vo.vy, 0.001f * vo.vz );
						// printf( "frame index:%d,stamp:%d\n", vo.frame_index, vo.time_stamp );
						// printf( "\n" );
						got_data = true;
					}
					if ( e_obstacle_distance == cmd_id )
					{
						obstacle_distance oa;
						memcpy( &oa, data + 2, sizeof(oa) );
						// printf( "obstacle distance:" );
						// for ( int direction = 0; direction < CAMERA_PAIR_NUM; ++direction )
						// {
							// printf( " %f ", 0.01f * oa.distance[direction] );
						// }
						// printf( "\n" );
						// printf( "frame index:%d,stamp:%d\n", oa.frame_index, oa.time_stamp );
						// printf( "\n" );
						got_data = true;
					}
				}
				else
				{
					printf( "err\n" );
				}
			}
			else
			{
				break;
			}
		}
	}



	disconnect_serial();

	return 0;
}
