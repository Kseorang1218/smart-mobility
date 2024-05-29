#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include <time.h>
#include <stdlib.h>

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "tl_publisher");
    ros::NodeHandle n;
    ros::Publisher tl_L_pub = n.advertise<std_msgs::String>("Left_color", 1000);
    ros::Publisher tl_R_pub = n.advertise<std_msgs::String>("Right_color", 1000);
    ros::Publisher tl_S_pub = n.advertise<std_msgs::String>("Single_color",1000);
    ros::Publisher count_pub = n.advertise<std_msgs::Int64>("time_count", 1000);
    ros::Rate loop_rate(1);
    std_msgs::String tl_L_color;
    std_msgs::String tl_R_color;
    std_msgs::String tl_S_color;
    std_msgs::Int64 count;
    int c = 20;
    srand(time(NULL));
    int c2 = rand() %10;

    while(ros::ok())
    {
        if(c <= 0) {
            c = 20;
        }
        if(c > 13) {
            tl_L_color.data="G";
            tl_R_color.data="R";
            count.data= c-10;
        } else if(c > 10) {
            tl_L_color.data="Y";
            tl_R_color.data="R";
            count.data= c-10;
        } else if(c > 3) {
            tl_L_color.data="R";
            tl_R_color.data="G";
            count.data= c;
        } else if(c > 0) {
            tl_L_color.data="R";
            tl_R_color.data="Y";
            count.data= c;
        }

        if( c2 <= 0){
            c2 = 20;
        }
        if(c2 > 13) {
            tl_S_color.data="G";
        } else if(c2 > 10) {
            tl_S_color.data="Y";
        } else if(c2 > 0) {
            tl_S_color.data="R";
        }


        ROS_INFO("Single : %s, Left : %s, Right : %s, time : %d", tl_S_color.data.c_str(), tl_L_color.data.c_str(), tl_R_color.data.c_str(), int(count.data));
        tl_L_pub.publish(tl_L_color);
        tl_R_pub.publish(tl_R_color);
        tl_S_pub.publish(tl_S_color);
        count_pub.publish(count);
        loop_rate.sleep();
        c--;
        c2--;
    }
ros::spinOnce();
return 0;
}