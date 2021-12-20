#include <ros/ros.h>
#include <string.h>
#include <cv.h>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include "sensor_msgs/JointState.h"
#include <cv_bridge/cv_bridge.h>
#include <aruco/aruco.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>


#include "tutorial_6/MoveJoints.h"

using namespace std;
using namespace cv;
using namespace aruco;
using namespace tf;
using namespace ros;
using namespace tutorial_6;

class ArucoControl 
{
public:
    // ros handler
    NodeHandle NH;

    // Image transport and subscriber:
    image_transport::ImageTransport IT;
    image_transport::Subscriber image_sub_;
    Mat image;
    Mat flipped;

    // Aruco camera parameters
    CameraParameters cameraParameters;

    // Aruco marker parameters
    float aruco_x, aruco_y, aruco_z;
    float markerSize;
    

    ArucoControl() : IT(NH)
    {
        image_sub_ = IT.subscribe("/nao_robot/camera/bottom/camera/image_raw", 500, &ArucoControl::imageCallBack, this);
    }

    ~ArucoControl()
    {
    }

    // Image callback function is executed when a new image is received:
    void imageCallBack(const sensor_msgs::ImageConstPtr& msg) 
    {
        cv_bridge::CvImageConstPtr cv_ptr;
	    try 
	    {
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	        image = cv_ptr->image.clone();
        }
	    catch (cv_bridge::Exception& except) 
	    {
	        ROS_ERROR("cv_bridge exception: %s", except.what());
            return;
        }

        float datad[5] = {-0.0481869853715082,  0.0201858398559121,
                   0.0030362056699177, -0.00172241952442813, 0};
        float datac[9] = {278.236008818534, 0,                156.194471689706,
                  0,                279.380102992049, 126.007123836447,
                  0,                0,                1};

        Mat cameraP(3,3,CV_32FC1, datac);
        Mat dist(5,1,CV_32FC1, datad);


	    cameraParameters.setParams(cameraP,dist,cv::Size(320,240));
        aruco::MarkerDetector arucoDetector;
        vector<Marker> detectedMarkers;
        Marker detectedMarker;
        arucoDetector.detect(image, detectedMarkers);

        if (detectedMarkers.size() > 0) 
	    {
          detectedMarker = detectedMarkers[0];
	      markerSize = 0.064;
          detectedMarker.calculateExtrinsics(markerSize, cameraParameters, true);
          detectedMarker.draw(image, cv::Scalar(0,0,255), 2);
          aruco_x = detectedMarker.Tvec.at<float>(0);
          aruco_y = detectedMarker.Tvec.at<float>(1);
          aruco_z = detectedMarker.Tvec.at<float>(2);
          static TransformBroadcaster br;
          Transform transform;
          transform.setOrigin( Vector3(aruco_x, aruco_y, aruco_z) );
        
        }
	    else
	    {
        // set it to zeros if nothing is seen
            aruco_x = 0.0;
            aruco_y = 0.0;
            aruco_z = 0.0;
	    }

      flip(image, flipped, 1);
      imshow("marker", flipped);
      waitKey(3);

      ServiceClient movejointaruco_client = NH.serviceClient <MoveJoints>("MoveJoint_aruco_stuff");
      MoveJoints srv;
      srv.request.name="RArm";
      srv.request.desired.linear.x = aruco_x;
      srv.request.desired.linear.y = aruco_y;
      srv.request.desired.linear.z = aruco_z;

      movejointaruco_client.call(srv);

    }

};

int main(int argc, char** argv)
{
    init(argc, argv, "tutorial_6");

    ArucoControl arucoControl;

    spin();
    return 0;

}
