#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
 
int main(int argc, char** argv)
{
    if (argc != 2)
    {
        ROS_ERROR("please specify the camera index");
    }

    ros::init(argc, argv, "video_pub");
    ros::NodeHandle nh;
    const int cam_idx = std::stoi(argv[1]);
    cv::VideoCapture cap(cam_idx);
    if (!cap.isOpened())
    {
        ROS_ERROR_STREAM("failed to open camera with index " << cam_idx << "!");
        ros::shutdown();
    }
     
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_frame = it.advertise("webcam_feed", 1);
    cv::Mat frame;
    sensor_msgs::ImagePtr msg;
    ros::Rate loop_rate(10);
 
    while (nh.ok())
    {
        cap >> frame; 
        if (frame.empty())
        {
            ROS_ERROR_STREAM("failed to capture image!");
            ros::shutdown();
        }

        ROS_INFO("publishing frame");
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        pub_frame.publish(msg);
        cv::waitKey(1);
        ros::spinOnce();
        loop_rate.sleep();
    }  
 
    cap.release();
    return 0;
}
