////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "ros/ros.h"
#include "cvision/ObjectPose.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/Image.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat)
#include <image_transport/image_transport.h>

#include <fstream>
#include <sstream>
#include <iostream> // for standard I/O
#include <string>   // for strings
#include <cstring>


using namespace std;
using namespace cv;

cv_bridge::CvImagePtr cv_img_ptr_ros;
cv_bridge::CvImagePtr cv_img_gray_ptr_ros;

void imageCallback(const sensor_msgs::ImageConstPtr& input)
{
//Convert ROS image message to CvImage suitable for OpenCV
    try
    {
        cv_img_ptr_ros = cv_bridge::toCvCopy(input, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv)
{

    bool bStream = false;
    int stream_rate = 1;
    int frame_rate = 30;

    int obj_shape = 2; //1-circle, 2-rotated rect
    int min_obj_sz = 5;
    int obj_sz = 0;
    int morph_sz = 5;
    int merge_thres = 100;

    int frame_counter = 0;
    int frame_count_max = -1; //infinite

    //Threshold values
    int iLowHue = 0;
    int iHighHue = 179;
    int iLowLum = 0;
    int iHighLum = 255;
    int iLowSat = 0;
    int iHighSat = 255;

    int iLowBlue = 0;
    int iHighBlue = 255;
    int iLowGreen = 0;
    int iHighGreen = 255;
    int iLowRed = 0;
    int iHighRed = 255;

    int iLowL = 0;
    int iHighL = 255; //standard convention 0 TO 100
    int iLowA = 0;
    int iHighA = 255; //standard convention -127 TO 127
    int iLowB = 0;
    int iHighB = 255; //standard convention -127 TO 127

    std::string pkgpath = "/home/odroid/ros_ws/src/ch-1-3/cvision";
    std::string srcpath = "/home/odroid/ros_ws/src/ch-1-3/cvision/src";
    std::string img_tp = "/cv_camera/image_raw";

    //ROS Init
    ros::init(argc, argv, "box_detector");

    //Node handle
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    cvision::ObjectPose msg;

    //Get the namespace
    std::string ns = ros::this_node::getNamespace();

    // get src path as a ros parameter. Should be loaded by cvision/configs/configs.yaml
    if (n.getParam(ns + "/pkg_path", pkgpath))
    {
        ROS_INFO("Got pkg path: %s", pkgpath.c_str());
        srcpath = pkgpath + "/src";
    }
    else
    {
        ROS_INFO("Failed to get param 'pkgpath'. Default to hardcoded paths.");
    }

    //get the topic name of image feed
    if (n.getParam(ns + "/image_feed", img_tp))
    {
        ROS_INFO("Got param: %s", img_tp.c_str());
    }
    else
    {
        ROS_INFO("Failed to get param 'image_feed'. Default to hardcoded image feed.");
    }

    // get gray image stream rate
    if (n.getParam(ns + "/bStream", bStream) && n.getParam(ns + "/stream_rate", stream_rate) )
    {
        ROS_INFO("Got streaming info.");
    }
    else
    {
        ROS_INFO("Failed to get streaming info.");
    }

    // Subscribe to image ROS topic
    ros::Subscriber image_sub = n.subscribe(img_tp,1,imageCallback);
    // Publish object pose
    ros::Publisher object_pub = n.advertise<cvision::ObjectPose>("boxPose",1000);
    // Publish video stream
    image_transport::Publisher img_pub = it.advertise("boxImg", 10);

    //Set loop rate for ros
    ros::Rate loop_rate(frame_rate);

    std::vector<int> thresBox_low(9);
    std::vector<int> thresBox_high(9);
    if (n.getParam(ns + "/Box/low", thresBox_low) && n.getParam(ns + "/Box/high", thresBox_high))
    {
        ROS_INFO("Got Box Thresholds.");
        iLowBlue = thresBox_low[0];
        iLowGreen = thresBox_low[1];
        iLowRed = thresBox_low[2];
        iLowL = thresBox_low[3];
        iLowA = thresBox_low[4];
        iLowB = thresBox_low[5];
        iLowHue = thresBox_low[6];
        iLowLum = thresBox_low[7];
        iLowSat = thresBox_low[8];
        iHighBlue = thresBox_high[0];
        iHighGreen = thresBox_high[1];
        iHighRed = thresBox_high[2];
        iHighL = thresBox_high[3];
        iHighA = thresBox_high[4];
        iHighB = thresBox_high[5];
        iHighHue = thresBox_high[6];
        iHighLum = thresBox_high[7];
        iHighSat = thresBox_high[8];
    }
    else
    {
        ROS_INFO("Failed to get thresholds.");
    }

    if (n.getParam(ns + "/Box/obj_shape", obj_shape) && n.getParam(ns + "/Box/min_obj_sz", min_obj_sz) && n.getParam(ns + "/Box/morph_sz", morph_sz) && n.getParam(ns + "/Box/merge_thres", merge_thres))
    {
        ROS_INFO("Got Object Parameters.");
    }
    else
    {
        ROS_INFO("Failed to get object parameters.");
    }

    Mat imgGray;
    Mat imgBGR;
    Mat imgHLS;
    Mat imgLAB;
    Mat imgThresSum;
    Mat imgThresAll;
    Mat imgThresAllGray;
    Mat imgThres3[3];
    Mat imgBGRThres;
    Mat imgHLSThres;
    Mat imgLABThres;
    Mat imgContours;

    while (frame_counter != frame_count_max && ros::ok())
    {
        //cout << "Frame: " << frame_counter << endl;

        if (cv_img_ptr_ros)
        {
            imgBGR = cv_img_ptr_ros->image;
        }
        else
        {
            ROS_INFO("No image.");
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }

        frame_counter++;

        //Determine size of video input
        int irows_imgBGR = imgBGR.rows;
        int icols_imgBGR = imgBGR.cols;

        cvtColor(imgBGR, imgHLS, COLOR_BGR2HLS); //Convert the captured frame from BGR to HLS
        cvtColor(imgBGR, imgLAB, COLOR_BGR2Lab); //Convert the captured frame from BGR to LAB

        //Thresholding
        inRange(imgBGR, Scalar(iLowBlue, iLowGreen, iLowRed), Scalar(iHighBlue, iHighGreen, iHighRed), imgBGRThres); //Threshold the BGR image
        inRange(imgHLS, Scalar(iLowHue, iLowLum, iLowSat), Scalar(iHighHue, iHighLum, iHighSat), imgHLSThres); //Threshold the HLS image
        inRange(imgLAB, Scalar(iLowL, iLowA, iLowB), Scalar(iHighL, iHighA, iHighB), imgLABThres); //Threshold the LAB image

        //Merge results from color spaces
        imgThres3[0] = imgBGRThres;
        imgThres3[1] = imgLABThres;
        imgThres3[2] = imgHLSThres;
        merge(imgThres3,3,imgThresAll);

        //Threshold combined results
        //gray = 0.114b + 0.587g + 0.299r
        //e.g merge_thres = 100 - LAB or HLS + BGR, 160 - LAB + HLS or LAB + BGR, 200 - LAB + HLS
        cvtColor(imgThresAll, imgThresAllGray, COLOR_BGR2GRAY);
        //GaussianBlur(imgThresAllGray, imgThresAllGray, Size(0, 0),morph_sz);
        //blur(imgThresAllGray, imgThresAllGray, Size(morph_width, morph_height));
        threshold(imgThresAllGray,imgThresSum,merge_thres,255,THRESH_BINARY);

        morph_sz=max(morph_sz,1);
        //Width and height for morph
        int morph_width = morph_sz;
        int morph_height = morph_sz;

        //morphological opening (removes small objects from the foreground)
        erode(imgThresSum, imgThresSum, getStructuringElement(MORPH_ELLIPSE, Size(morph_width, morph_height)));
        dilate(imgThresSum, imgThresSum, getStructuringElement(MORPH_ELLIPSE, Size(morph_width, morph_height)));

        //morphological closing (removes small holes from the foreground)
        dilate(imgThresSum, imgThresSum, getStructuringElement(MORPH_ELLIPSE, Size(morph_width, morph_height)));
        erode(imgThresSum, imgThresSum, getStructuringElement(MORPH_ELLIPSE, Size(morph_width, morph_height)));

        imgContours = imgThresSum.clone();

        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;

        findContours(imgContours, contours, hierarchy,
                     CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

        int cont_sz = contours.size();
        vector<Moments> mu(cont_sz);
        vector<vector<Point> > contours_poly(cont_sz);
        vector<RotatedRect> minRect(cont_sz);
        vector<Point2f> mc(cont_sz);
        vector<Point2f> mc_dist(cont_sz);
        vector<Point2f> cc(cont_sz);
        vector<float> cr(cont_sz);
        vector<int> minRectArea(cont_sz);
        vector<int> minCircleArea(cont_sz);
        int max_idx_c= 0;
        int max_idx_r= 0;

        if (cont_sz>0)
        {

            //center of frame
            Point2f frameCenter(icols_imgBGR / 2, irows_imgBGR / 2);

            /// Approximate contours to polygons and fit rotated rectange or circle
            for (int i = 0; i < cont_sz; i++)
            {
                approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
                // Get the moments
                mu[i] = moments(contours_poly[i], false);
                //Assuming object is circle, calculate diameter
                //m00 is object area in pixels
                obj_sz = 2*sqrt(mu[i].m00/CV_PI);
//                cout << "Approximate Object Diameter: " << obj_sz << endl;

                if (obj_sz>min_obj_sz)
                {
                    // Get the mass centers:
                    mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
                    mc_dist[i] = frameCenter - mc[i];

                    //Circle
                    if (obj_shape == 1)
                    {
                        minEnclosingCircle( (Mat)contours_poly[i], cc[i], cr[i] );
                        minCircleArea[i]=cr[i]*cr[i]*CV_PI;
                        if (minCircleArea[max_idx_c]<minCircleArea[i])
                        {
                            max_idx_c = i;
                        }
                    }

                    //Rotated rect
                    else if (obj_shape == 2)
                    {
                        minRect[i] = minAreaRect( Mat(contours_poly[i]) );
                        minRectArea[i]=minRect[i].size.width*minRect[i].size.height;
                        if (minRectArea[max_idx_r]<minRectArea[i])
                        {
                            max_idx_r = i;
                        }
                    }

                }
            }
        }


        Scalar color = Scalar(0, 255, 255);

        if ( (bStream && (((frame_counter*stream_rate)%frame_rate)<stream_rate) ) )
        {

            //Circle
            if (obj_shape == 1)
            {
                //min circle
                circle(imgBGR, cc[max_idx_c], (int)cr[max_idx_c], color, 2, 8, 0 );
                //Center
                //circle(imgBGR, mc[max_idx_c], 5, color, -1, 8, 0);
                circle(imgBGR, cc[max_idx_c], 5, color, -1, 8, 0);
            }

            //Rotated rect
            else if (obj_shape == 2)
            {
                //rotated rectangle
                Point2f rect_points[4];
                minRect[max_idx_r].points( rect_points );
                for( int j = 0; j < 4; j++ )
                    line(imgBGR, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
                //Center
                //circle(imgBGR, mc[max_idx_r], 5, color, -1, 8, 0);
                circle(imgBGR, minRect[max_idx_r].center, 5, color, -1, 8, 0);
            }

            //Publish gray image to ROS
            cvtColor(imgBGR, imgGray, CV_BGR2GRAY);
            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", imgGray).toImageMsg();
            img_pub.publish(img_msg);

        }

        //ROS Topics
        //pose (setpoint - 2D float [m], heading - float [deg])
        //valid - bool
        //radius - float [m]
        int x_SP = 0;
        int y_SP = 0;
        float r_SP = 0;
        float t_SP = 0;

        //Circle
        if (obj_shape == 1)
        {
            //x_SP = mc[max_idx_c].x;
            //y_SP = mc[max_idx_c].y;
            x_SP = cc[max_idx_c].x;
            y_SP = cc[max_idx_c].y;
            r_SP = cr[max_idx_c];
        }
        //Rotated rect
        else if (obj_shape == 2)
        {
            //x_SP = mc[max_idx_r].x;
            //y_SP = mc[max_idx_r].y;
            x_SP = minRect[max_idx_r].center.x;
            y_SP = minRect[max_idx_r].center.y;
            t_SP = minRect[max_idx_r].angle;
        }

        if (cont_sz>0)
        {
            msg.pose.x = x_SP;
            msg.pose.y = y_SP;
            msg.pose.theta = t_SP;
            msg.radius = r_SP;
            msg.valid = true;
        }
        else
        {
            msg.valid = false;
        }

        //ROS Publisher
        object_pub.publish(msg);

        if (cv_img_ptr_ros)
        {
            //Clear pointer
            cv_img_ptr_ros.reset();
        }

        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}

