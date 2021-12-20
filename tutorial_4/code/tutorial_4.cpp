/*

This code implements task 10 from tutorial_4

Group D: Selin Kesler, Victor Kowalski and Miriam Senne

*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aruco/aruco.h>

using namespace std;
using namespace cv;
using namespace aruco;
static const std::string winName = "3D marker position";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

  public:

  ImageConverter() : it_(nh_)
  {
    image_sub_ = it_.subscribe("/nao_robot/camera/top/camera/image_raw", 1, &ImageConverter::imageCb, this);
    namedWindow(winName);
  }

  ~ImageConverter()
  {
    destroyWindow(winName);
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

    // create a variable in the class
    CameraParameters TheCameraParameters;

    // load the parameter matrix in the constructor
    Mat dist(1,5,CV_32FC1);
    dist.at<float>(0,0)=-0.066494;
    dist.at<float>(0,1)=0.095481;
    dist.at<float>(0,2)=-0.000279;
    dist.at<float>(0,3)=0.002292;
    dist.at<float>(0,4)=0.000000;
    Mat cameraP(3,3,CV_32FC1);

    cameraP.at<float>(0,0)=551.543059;
    cameraP.at<float>(0,1)=0.000000;
    cameraP.at<float>(0,2)=327.382898;
    cameraP.at<float>(1,0)=0.000000;
    cameraP.at<float>(1,1)=553.736023;
    cameraP.at<float>(1,2)=225.026380;
    cameraP.at<float>(2,0)=0.000000;
    cameraP.at<float>(2,1)=0.000000;
    cameraP.at<float>(2,2)=1.000000;

    TheCameraParameters.setParams(cameraP,dist,Size(640,480));
    TheCameraParameters.resize( Size(640,480));


    // Detection of the 3D position of the marker
    aruco::MarkerDetector Detector;
    std::vector< Marker >  detectedMarkers;
    Detector.detect(cv_ptr->image, detectedMarkers, TheCameraParameters, 5, true);
    for (unsigned i = 0; i< detectedMarkers.size(); i++ )
    {
      ROS_INFO_STREAM("\n3D position:\n" << detectedMarkers[i].Tvec);
      detectedMarkers[i].draw(cv_ptr->image, Scalar(0,0,255), 2);
    }
    
    imshow(winName, cv_ptr->image);
    waitKey(3);

  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tutorial_4");
  ImageConverter imConv;
  ros::spin();
  return 0;
}