#include "ros/ros.h"
#include <iostream>
#include "tf/transform_listener.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include <sstream>

 class echoListener
 {
 public:
 
   tf::TransformListener tf;
 
   //constructor with name
   echoListener()
   {
 
   }
 
   ~echoListener()
   {
 
   }
 
 private:
 
 };

int main(int arg,char** argv)
{
    ros::init(arg,argv,"listener");
    ros::NodeHandle nh;
    ros::Rate rate(10);
    echoListener echoListener;
    ros::Publisher number_publisher = nh.advertise<geometry_msgs::Transform>("/tranform_vector",10);
    geometry_msgs::Transform tranform_val;
    geometry_msgs::Vector3 vec;
    geometry_msgs::Quaternion quat;
    std::string source_frameid = "world";
    std::string target_frameid = "link_6";

    echoListener.tf.waitForTransform(source_frameid, target_frameid, ros::Time(), ros::Duration(1.0));

    while(nh.ok())
     {
       try
       {
         tf::StampedTransform echo_transform;
         echoListener.tf.lookupTransform(source_frameid, target_frameid, ros::Time(), echo_transform);
         double yaw, pitch, roll;
         echo_transform.getBasis().getRPY(roll, pitch, yaw);
         tf::Quaternion q = echo_transform.getRotation();
         tf::Vector3 v = echo_transform.getOrigin();
         vec.x=v.getX();
         vec.y=v.getY();
         vec.z=v.getZ();
         quat.x=q.getX();
         quat.y=q.getY();
         quat.z=q.getZ();
         quat.w=q.getW();
         tranform_val.translation=vec;
         tranform_val.rotation=quat;
         number_publisher.publish(tranform_val);
       }
       catch(tf::TransformException& ex)
       {
         std::cout << "Failure at "<< ros::Time::now() << std::endl;
         std::cout << "Exception thrown:" << ex.what()<< std::endl;
         std::cout << "The current list of frames is:" <<std::endl;
         std::cout << echoListener.tf.allFramesAsString()<<std::endl;
         
       }
       rate.sleep();
     }
 
   return 0;
}