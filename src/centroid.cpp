#include <ros/ros.h>
#include <pcl_usage/Centroid.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::Publisher cmdpub_;
ros::Publisher centroidpub_;


void cloudcb(const PointCloud::ConstPtr& cloud)
  {    
    
    //X,Y,Z of the centroid
    float x = 0.0;
    float y = 0.0;
    float z = 1e6;
    
    //min max x, y, z
    float min_y_ = 0.1;
    float max_y_ = 0.5;
    float min_x_ = -0.2;
    float max_x_ = 0.2;
    float max_z_ = 1; 
    float goal_z_ = 0.6;
    float z_scale_ = 1.0;
    float x_scale_ = 5.0;
    
    //Number of points observed
    unsigned int n = 0;
    //Iterate through all the points in the region and find the average of the position
    BOOST_FOREACH (const pcl::PointXYZ& pt, cloud->points)
    {
      //First, ensure that the point's position is valid. This must be done in a seperate
      //if because we do not want to perform comparison on a nan value.
      if (!std::isnan(x) && !std::isnan(y) && !std::isnan(z))
      {
        //printf("VALID- x:%f y:%f z:%f\n", pt.x, -pt.y, pt.z);
        //Test to ensure the point is within the aceptable box.
        if (-pt.y > min_y_ && -pt.y < max_y_ && pt.x < max_x_ && pt.x > min_x_ && pt.z < max_z_)
        {
          //Add the point to the totals
          x += pt.x;
          y += pt.y;
          z = std::min(z, pt.z);
          n++;
        }
      }
    }

    //If there are points, find the centroid and calculate the command goal.
    //If there are no points, simply publish a stop goal.
    if (n>4000)
    {
      x /= n;
      y /= n;
      if(z > max_z_){
        printf("No valid points detected, stopping the robot\n");
        
        pcl_usage::Centroid centroid;
        centroid.has_centroid = false;
        centroidpub_.publish(centroid);
        
        return;
      }

      printf("Centroid at %f %f %f with %d points\n", x, y, z, n);

     
      pcl_usage::Centroid centroid;
      centroid.has_centroid = true;
      centroid.x = x;
      centroid.y = y;
      centroid.z = z;
      centroidpub_.publish(centroid);
      
    }
    else
    {
      printf("No points detected, stopping the robot (%d)\n",n);
      pcl_usage::Centroid centroid;
      centroid.has_centroid = false;
      centroidpub_.publish(centroid);
      
    }
  }

int main(int argc, char **argv)
{
   printf("INIT\n");
  ros::init(argc, argv, "follower_data");
  ros::NodeHandle n;
  
  centroidpub_ = n.advertise<pcl_usage::Centroid> ("/follower_data/centroid", 1);
  ros::Subscriber sub = n.subscribe("/camera/depth/points", 1000, cloudcb);
  ros::spin();
}
