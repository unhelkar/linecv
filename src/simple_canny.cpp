#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class SimpleCanny
 {
   ros::NodeHandle nh_;
   image_transport::ImageTransport it_;
   image_transport::Subscriber image_sub_;
   image_transport::Publisher image_pub_;
   // Additional class parameters
   ros::NodeHandle n;
   ros::Publisher pub ;
   std_msgs::String msg;

public:
   SimpleCanny()
     : it_(nh_)
   {
     image_pub_ = it_.advertise("/linecv/canny_image", 1);
     image_sub_ = it_.subscribe("/px4flow/camera_image", 1, &SimpleCanny::ImageCb, this);
     // Additional window note
     // cv::namedWindow(WINDOW);
   }

   ~SimpleCanny()
   {
     cv::destroyWindow(WINDOW);
   }

   void ImageCb(const sensor_msgs::ImageConstPtr& msg)
   {
     cv_bridge::CvImagePtr cv_ptr;
     try
     {
       cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
     }
     catch (cv_bridge::Exception& e)
     {
       ROS_ERROR("cv_bridge exception: %s", e.what());
       return;
     }

     // Initialize place holders for new iamges: publishedImage, grayScale, Canny, ...
//. /*
      // sensor_msgs::CvBridge bridge;
      // IplImage* img = bridge.imgMsgToCv(msg,"bgr8");  //image being converted from ros to opencv using cvbridge
      IplImage* img = cv_ptr->image;
      IplImage* out1 = cv::cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 3 );   //make sure to feed the image(img) data to the parameters necessary for canny edge output 
      IplImage* gray_out = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1 ); 
      IplImage* canny_out = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1 );
      IplImage* gray_out1=cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 3 );
      IplImage* img1 = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 3 ); 
//. */


     // Carry out Canny conversions
//. /*
      cvCvtColor(img, gray_out, CV_BGR2GRAY);
      cvSmooth(gray_out, gray_out, CV_GAUSSIAN, 9, 9); 
      cvCanny( gray_out, canny_out, 50, 125, 3 );
      cvCvtColor(canny_out ,gray_out1, CV_GRAY2BGR);
//. */

/*     if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
       cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
*/
     cv::imshow(WINDOW, cv_ptr->image);
     cv::waitKey(2);

     image_pub_.publish(cv_ptr->toImageMsg());
   }
 };

 int main(int argc, char** argv)
 {
   ros::init(argc, argv, "simple_canny");
   SimpleCanny ic;
   ros::spin();
   return 0;
 }
