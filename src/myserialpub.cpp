/*
 * myserialpub.cpp
 *
 *  Created on: 2016年7月18日
 *      Author: ubuntu
 */

#include "ros/ros.h"
//#include "myserial/myserial.h"
#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

using namespace std;
using namespace boost::asio;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serialpub"); // Initiate new ROS node named "talker"

    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<myserial::myserial>("chatter", 1000);
    ros::Rate loop_rate(1000);

    io_service iosev;  //节点文件
    serial_port sp(iosev, "/dev/ttyUSB0");
     // 设置参数
     sp.set_option(serial_port::baud_rate(115200));
     sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
     sp.set_option(serial_port::parity(serial_port::parity::none));
     sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
     sp.set_option(serial_port::character_size(8));

    int count = 0;
    while (ros::ok()) // Keep spinning loop until user presses Ctrl+C
    {
        myserial::myserial msg;
       // 向串口写数据
        // write(sp, buffer("Hello world", 12));
         // 向串口读数据
        char buf[10];
        read(sp, buffer(buf));
        msg.a1=buf[0];
        msg.a2 = buf[1];
        ROS_INFO(" %.6f, %.6f", msg.a1, msg.a2);

        chatter_pub.publish(msg);
        ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
        loop_rate.sleep(); // Sleep for the rest of the cycle, to enforce the loop rate
        count++;
    }
    iosev.run();
    return 0;
}


