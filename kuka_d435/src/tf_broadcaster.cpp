#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_msgs/TFMessage.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include <iostream>
#include <sstream>
#include <vector>
#include <math.h>

std::vector<tf2_msgs::TFMessage::ConstPtr> transforms;

void number_callback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
    geometry_msgs::Quaternion quat;
    geometry_msgs::Vector3 trans;
    float rot_mat[4][4];
    float identity[4][4];
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            if(i==j)
            {
                identity[i][j]=1;
            }
            else
            {
                identity[i][j]=0;
            }
        }
    }

    /*for(int i=0;i<6;i++)
    {
        quat=msg->transforms[i].transform.rotation;
        trans=msg->transforms[i].transform.translation;
        char qt[]="xyz"
        for(int j=0;j<4;j++)
        {
            for(int k=0;k<4;k++)
            {
                if(j==k && j!=3)
                {
                    rot_mat[j][k]=2*(pow(quat.w,2)+pow(quat.[j],2))-1;
                }
                else
                {
                    rot_mat[j][k]=1;
                }
                if(k>j && k<3)
                {
                    rot_mat[j][k]=2*(quat[j]*quat[k]-quat.w*quat[3-k-j]);
                }
            }
        }
        for(int i=0;i<4;i++)
        {
            for(int j=0;j<4;j++)
            {
                identity[i][j]*=rot_mat[i][j];
            }
        }
    }
    ROS_INFO_STREAM(identity);*/
}

int main(int arg,char** argvc)
{
    ros::init(arg,argvc,"my_tf_broadcaster");
    ros::NodeHandle nh;
    ros::Subscriber number_subscriber = nh.subscribe("/tf",10,number_callback);
    ros::spin();
    return 0;
    
}