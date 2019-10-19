#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>   
#include "iostream"
#include "stdio.h"
#include <termios.h>   
using namespace std;
int main(int argc, char **argv)
{
  ros::init(argc,argv,"tp");

  ros::NodeHandle n;
  //
  ros::Publisher pub_start = n.advertise<sensor_msgs::PointCloud2>("start_points", 1);
  ros::Publisher pub_end = n.advertise<sensor_msgs::PointCloud2>("end_points", 1);
  pcl::PointCloud<pcl::PointXYZ>::Ptr start_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr end_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  std::string start_path;
  std::string end_path;
  ros::param::get("~start_path",start_path);
  ros::param::get("~end_path",end_path);
  if(pcl::io::loadPCDFile<pcl::PointXYZ>(start_path,*start_cloud)==-1)
  {
    ROS_INFO("loadPCDstart error!");
    return -1;
  }

  if(pcl::io::loadPCDFile<pcl::PointXYZ>(end_path,*end_cloud)==-1)
  {
    ROS_INFO("loadPCDend error!");
    return -1;
  }

  ////转成彩色点云
  // uint8_t r(255), g(15), b(15);
  // uint32_t rgb = (static_cast<uint32_t> (r) << 16 | static_cast<uint32_t> (g) << 8 | static_cast<uint32_t> (b));
  // float rgb_float = *reinterpret_cast<float*> (&rgb); 
  // for(int i=0;i< end_cloud ->points.size();i++)
  // {
  //   pcl::PointXYZRGB points;
  //   points.x = end_cloud ->points[i].x;
  //   points.y = end_cloud ->points[i].y;
  //   points.z = end_cloud ->points[i].z;
  //   points.rgb = rgb_float;
  //   end_cloud_color -> points.push_back(points); 
  // }

  // end_cloud_color -> width = (int)end_cloud_color -> points.size();
  // end_cloud_color -> height = 1;

  //过滤
  // pcl::PassThrough<pcl::PointXYZ> passx;
  // passx.setInputCloud (pass_cloud);            //设置输入点云
  // passx.setFilterFieldName ("x");         //设置过滤时所需要点云类型的Z字段
  // passx.setFilterLimits (-10, 10);
  // passx.setFilterLimitsNegative (false);   //设置保留范围内还是过滤掉范围内
  // passx.filter (*pass_cloud);

  // pcl::PassThrough<pcl::PointXYZ> passy;
  // passy.setInputCloud (pass_cloud);            //设置输入点云
  // passy.setFilterFieldName ("y");         //设置过滤时所需要点云类型的Z字段
  // passy.setFilterLimits (-10, 10);
  // passy.setFilterLimitsNegative (false);   //设置保留范围内还是过滤掉范围内
  // passy.filter (*pass_cloud);

  // pcl::PassThrough<pcl::PointXYZ> passz;
  // passz.setInputCloud (pass_cloud);            //设置输入点云
  // passz.setFilterFieldName ("z");         //设置过滤时所需要点云类型的Z字段
  // passz.setFilterLimits (-2, 1.0);
  // passz.setFilterLimitsNegative (false);   //设置保留范围内还是过滤掉范围内
  // passz.filter (*pass_cloud);
 // Convert to ROS data type 
  sensor_msgs::PointCloud2 output_start;
  pcl::toROSMsg(*start_cloud,output_start);
  output_start.header.frame_id = std::string("rslidar"); 
  
  

  sensor_msgs::PointCloud2 output_end;
   pcl::toROSMsg(*end_cloud,output_end);
   output_end.header.frame_id = std::string("rslidar"); 

  sleep(1);

  pub_start.publish (output_start);
  pub_end.publish (output_end);
  
  // Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

  //   // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
  //   // Here we defined a 45° (PI/4) rotation around the Z axis and a translation on the X axis.
  //   float theta = M_PI/4; // The angle of rotation in radians
  //   transform_1 (0,0) = cos (theta);
  //   transform_1 (0,1) = -sin(theta);
  //   transform_1 (1,0) = sin (theta);
  //   transform_1 (1,1) = cos (theta);
  //   transform_1 (0,3) = 50;
  // pcl::transformPointCloud (*pass_cloud,*pass_cloud, transform_1);
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);
  newt = oldt; newt.c_lflag &= ~(ICANON); 
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);
  float t_x=0.0,t_y=0.0,t_z=0.0,t_roll=0,t_pitch=0,t_yaw=0;
  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
  float theta = M_PI/180; // The angle of rotation in radians
  int put = 1;
 
  float dw = 0.05;
  while(put!=10)
  {
     //ROS_INFO("%d",put);
     put = getchar();
     
     switch(put)
     {
       case 43: //+
       dw+=0.05;
       ROS_INFO("---------%f----------",dw);
       break;

       case 45: //-
       dw=0.05;
       ROS_INFO("---------%f----------",dw);
       break;

       case 49://1
       t_roll-=dw;
       break;

       case 51://3
       t_roll+=dw;
       break;

       case 52://4
       t_pitch-=dw;
       break;

       case 54://6
       t_pitch+=dw;
       break;

       case 55://7
       t_yaw-=dw;
       break;

       case 57://9
       t_yaw+=dw;
       break;

       case 97: //a  
       t_y+=dw;
       break;

       case 100://d
       t_y-=dw;
       break;

       case 101://e
       t_z+=dw;
       break;

       case 113://q
       t_z-=dw;
       break;

       case 115: //s　
       t_x-=dw;
       break;

       case 119: //w 
       t_x+=dw;
       break;
     }
     float d_yaw = theta * t_yaw;
     float d_pitch = theta * t_pitch;
     float d_roll = theta * t_roll;
     //x
     transform_1 (0,0) = cos(d_yaw) * cos(d_pitch);
     transform_1 (0,1) = (cos(d_yaw) * sin(d_pitch) * sin(d_roll)) - (sin(d_yaw)*cos(d_roll));
     transform_1 (0,2) = (cos(d_yaw) * sin(d_pitch) * cos(d_roll)) + (sin(d_yaw) * sin(d_roll));
     //y
     transform_1 (1,0) = sin(d_yaw)*cos(d_pitch);
     transform_1 (1,1) = (sin(d_yaw) * sin(d_pitch) * sin(d_roll)) + (cos(d_yaw)*cos(d_roll));
     transform_1 (1,2) = (sin(d_yaw) * sin(d_pitch) * cos(d_roll)) - (cos(d_yaw) * sin(d_roll));
     //z
     transform_1 (2,0) = -sin(d_pitch);
     transform_1 (2,1) = cos(d_pitch) * sin(d_roll);
     transform_1 (2,2) = cos(d_pitch) * cos(d_roll);
     

     transform_1 (0,3) = t_x;//ｘ轴坐标偏移
     transform_1 (1,3) = t_y;//y轴坐标偏移
     transform_1 (2,3) = t_z;//z轴坐标偏移
     pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZ>);
     pcl::transformPointCloud (*end_cloud, *trans_cloud,transform_1);
     
     pcl::toROSMsg(*trans_cloud,output_end);
     output_end.header.frame_id = std::string("rslidar");
     pub_end.publish (output_end);
     ROS_INFO("x:%f y:%f z:%f roll:%f pitch:%f yaw:%f",t_x,t_y,t_z,t_roll,t_pitch,t_yaw);
  }
  
    
  

  

 

  return 0;
}
