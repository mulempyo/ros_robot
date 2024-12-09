#include <ros/ros.h>
#include <gb_visual_detection_3d_msgs/BoundingBoxes3d.h>
#include <cmath>

void boundingBoxCallback(const gb_visual_detection_3d_msgs::BoundingBoxes3d::ConstPtr& msg)
{        
      for(const auto& box : msg->bounding_boxes){
        // 특정 클래스만 처리
        if (box.Class == "person")
        {
            // 중심 좌표
            float x = (box.xmax + box.xmin)/2;
            float y = (box.ymax + box.ymin)/2;
            float z = (box.zmax + box.zmin)/2;

            // 카메라와 물체 간 거리 계산
            float distance = std::sqrt(x * x + y * y + z * z);
            ROS_WARN("x: %f,y: %f,z: %f", x,y,z);
            ROS_WARN("Detected %s at distance: %f meters", box.Class.c_str(), distance);
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bounding_boxes_3d_distance");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/darknet_ros_3d/bounding_boxes", 10, boundingBoxCallback);

    ros::spin();
    return 0;
}
