#include "ros/ros.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// ros_msgs
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

static const std::string OPENCV_WINDOW = "Image window";

#define WHITE_MIN_THRESH 253
#define ROI_SQUARE_SIDE 6
#define WHITE_PIXEL_VALUE 255

class ImageProcessor{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
  // Define a global client that can request services
    ros::ServiceClient client;
    float linear_cmd = 0;
    float angular_cmd = 0;

    public:
        ImageProcessor()
        : it_(nh_)
        {
            image_sub_ = it_.subscribe("/camera/rgb/image_raw", 5,
            &ImageProcessor::imageCb, this);

            // Define a client service capable of requesting services from command_robot
            client = nh_.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
        
            cv::namedWindow(OPENCV_WINDOW);
        }

        ~ImageProcessor()
        {
            cv::destroyWindow(OPENCV_WINDOW);
        }

        void imageCb(const sensor_msgs::ImageConstPtr& msg)
        {
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
            cv::Mat src, gray;
            src = cv_ptr->image;
            cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
            cv::GaussianBlur(gray, gray, cv::Size(9, 9), 0);
            std::vector<cv::Point> white_spots;
            white_spots.empty();

            for(int i = 0; i < gray.rows; i ++){
                for(int j = 0; j < gray.cols; j ++){
                    if( (int)gray.at<uchar>(i,j)  > WHITE_MIN_THRESH){
                        white_spots.push_back( cv::Point(i, j) );
                    }
                }
            }

            cv::Point brightest_point = cv::Point(0,0);
            for(auto point : white_spots){
                brightest_point.x += point.y;
                brightest_point.y += point.x;
            }
            if( ! (int)white_spots.size() ){
                brightest_point.x = 0;
                brightest_point.y = 0;

                linear_cmd = 0;
                angular_cmd = 0;

                ROS_WARN("no white spots detected");
                ROS_INFO("STOPPING");
            }else{
                brightest_point.x = brightest_point.x / (int)white_spots.size();
                brightest_point.y = brightest_point.y / (int)white_spots.size();
                ROS_WARN("Brightest Point: %d, %d", brightest_point.x, brightest_point.y);
                cv::circle( src, brightest_point, 30, cv::Scalar(0,255,0),CV_FILLED, 8,0);

                if( brightest_point.x < (gray.rows / 3) ){
                    linear_cmd = 0;
                    angular_cmd = 0.25;
                    ROS_INFO("LEFT");
                }else if( (brightest_point.x > (gray.rows - (gray.rows / 3)) ) ){
                    linear_cmd = 0;
                    angular_cmd = -0.25;
                    ROS_INFO("RIGHT");
                }else if( ( brightest_point.x > (gray.rows / 3) ) && (brightest_point.x < (gray.rows - (gray.rows / 3)) )){
                    linear_cmd = 0.10;
                    angular_cmd = 0.0;
                    ROS_INFO("FORWARD");
                }
                

            }
            cv::imshow(OPENCV_WINDOW, src);
            cv::waitKey(3);

            drive_robot( linear_cmd, angular_cmd);
    }

    void drive_robot(float lin_x, float ang_z)
    {
        ball_chaser::DriveToTarget srv;
        srv.request.linear_x = lin_x;
        srv.request.angular_z = ang_z;
        if (!client.call(srv))
            ROS_ERROR("Failed to call service safe_move");
    }
};

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");

    ImageProcessor image_processor;
    
    // Handle ROS communication events
    ros::spin();

    return 0;
}