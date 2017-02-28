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

    bool bBlue = true;
    bool bGreen = true;
    bool bRed = true;
    bool bOrange = true;
    bool bYellow = true;
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

    std::string colorAsString = "None";
    std::string pkgpath = "/home/odroid/ros_ws/src/ch-1-3/cvision";
    std::string srcpath = "/home/odroid/ros_ws/src/ch-1-3/cvision/src";
    std::string img_tp = "/cv_camera/image_raw";

    //ROS Init
    ros::init(argc, argv, "obj_detector");

    //Node handle
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    cvision::ObjectPose msg_pos;
    cvision::ObjectPose msg_dist;

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
    // Publish object position
    ros::Publisher obj_pos_pub = n.advertise<cvision::ObjectPose>("objPosition",1000);
    // Publish object distance [m]
    ros::Publisher obj_dist_pub = n.advertise<cvision::ObjectPose>("objDistance",1000);
    // Publish video stream
    image_transport::Publisher img_pub = it.advertise("objImg", 10);
    image_transport::Publisher dbg_pub = it.advertise("objDbg", 10);

    //Set loop rate for ros
    ros::Rate loop_rate(frame_rate);

    std::vector<int> thres_low(9);
    std::vector<int> thres_high(9);

    Mat imgGray;
    Mat imgBGR;
    Mat imgHLS;
    Mat imgLAB;
    Mat imgThresSumBlue;
    Mat imgThresSumGreen;
    Mat imgThresSumRed;
    Mat imgThresSumOrange;
    Mat imgThresSumYellow;
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
        Mat imgThresSum=Mat::zeros(Size(icols_imgBGR,irows_imgBGR), CV_8UC1 );

        cvtColor(imgBGR, imgHLS, COLOR_BGR2HLS); //Convert the captured frame from BGR to HLS
        cvtColor(imgBGR, imgLAB, COLOR_BGR2Lab); //Convert the captured frame from BGR to LAB

        if (n.getParam(ns + "/FindBlue", bBlue) && n.getParam(ns + "/FindGreen", bGreen) && n.getParam(ns + "/FindRed", bRed) && n.getParam(ns + "/FindOrange", bOrange) && n.getParam(ns + "/FindYellow", bYellow))
        {
            //ROS_INFO("Got Allowed Colors.");
        }
        else
        {
            ROS_INFO("Failed to get allowed colors.");
        }

        if (bYellow)
        {
            colorAsString = "Yellow";
            if (n.getParam(ns + "/"+colorAsString+"/low", thres_low) && n.getParam(ns + "/"+colorAsString+"/high", thres_high) && n.getParam(ns + "/"+colorAsString+"/obj_shape", obj_shape) && n.getParam(ns + "/"+colorAsString+"/min_obj_sz", min_obj_sz) && n.getParam(ns + "/"+colorAsString+"/morph_sz", morph_sz) && n.getParam(ns + "/"+colorAsString+"/merge_thres", merge_thres))
            {
                //Thresholding
                inRange(imgBGR, Scalar(thres_low[0], thres_low[1], thres_low[2]), Scalar(thres_high[0], thres_high[1], thres_high[2]), imgBGRThres); //Threshold the BGR image
                inRange(imgHLS, Scalar(thres_low[6], thres_low[7], thres_low[8]), Scalar(thres_high[6], thres_high[7], thres_high[8]), imgHLSThres); //Threshold the HLS image
                inRange(imgLAB, Scalar(thres_low[3], thres_low[4], thres_low[5]), Scalar(thres_high[3], thres_high[4], thres_high[5]), imgLABThres); //Threshold the LAB image

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
                threshold(imgThresAllGray,imgThresSumYellow,merge_thres,255,THRESH_BINARY);

                morph_sz=max(morph_sz,1);
                //Width and height for morph
                int morph_width = morph_sz;
                int morph_height = morph_sz;

                //morphological opening (removes small objects from the foreground)
                erode(imgThresSumYellow, imgThresSumYellow, getStructuringElement(MORPH_ELLIPSE, Size(morph_width, morph_height)));
                dilate(imgThresSumYellow, imgThresSumYellow, getStructuringElement(MORPH_ELLIPSE, Size(morph_width, morph_height)));

                //morphological closing (removes small holes from the foreground)
                dilate(imgThresSumYellow, imgThresSumYellow, getStructuringElement(MORPH_ELLIPSE, Size(morph_width, morph_height)));
                erode(imgThresSumYellow, imgThresSumYellow, getStructuringElement(MORPH_ELLIPSE, Size(morph_width, morph_height)));

                bitwise_or(imgThresSumYellow,imgThresSum,imgThresSum);

            }
            else
            {
                ROS_INFO("Failed to get yellow thresholds and object parameters.");
            }
        }

        if (bBlue)
        {
            colorAsString = "Blue";
            if (n.getParam(ns + "/"+colorAsString+"/low", thres_low) && n.getParam(ns + "/"+colorAsString+"/high", thres_high) && n.getParam(ns + "/"+colorAsString+"/obj_shape", obj_shape) && n.getParam(ns + "/"+colorAsString+"/min_obj_sz", min_obj_sz) && n.getParam(ns + "/"+colorAsString+"/morph_sz", morph_sz) && n.getParam(ns + "/"+colorAsString+"/merge_thres", merge_thres))
            {
                //Thresholding
                inRange(imgBGR, Scalar(thres_low[0], thres_low[1], thres_low[2]), Scalar(thres_high[0], thres_high[1], thres_high[2]), imgBGRThres); //Threshold the BGR image
                inRange(imgHLS, Scalar(thres_low[6], thres_low[7], thres_low[8]), Scalar(thres_high[6], thres_high[7], thres_high[8]), imgHLSThres); //Threshold the HLS image
                inRange(imgLAB, Scalar(thres_low[3], thres_low[4], thres_low[5]), Scalar(thres_high[3], thres_high[4], thres_high[5]), imgLABThres); //Threshold the LAB image

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
                threshold(imgThresAllGray,imgThresSumBlue,merge_thres,255,THRESH_BINARY);

                morph_sz=max(morph_sz,1);
                //Width and height for morph
                int morph_width = morph_sz;
                int morph_height = morph_sz;

                //morphological opening (removes small objects from the foreground)
                erode(imgThresSumBlue, imgThresSumBlue, getStructuringElement(MORPH_ELLIPSE, Size(morph_width, morph_height)));
                dilate(imgThresSumBlue, imgThresSumBlue, getStructuringElement(MORPH_ELLIPSE, Size(morph_width, morph_height)));

                //morphological closing (removes small holes from the foreground)
                dilate(imgThresSumBlue, imgThresSumBlue, getStructuringElement(MORPH_ELLIPSE, Size(morph_width, morph_height)));
                erode(imgThresSumBlue, imgThresSumBlue, getStructuringElement(MORPH_ELLIPSE, Size(morph_width, morph_height)));

                bitwise_or(imgThresSumBlue,imgThresSum,imgThresSum);

            }
            else
            {
                ROS_INFO("Failed to get blue thresholds and object parameters.");
            }
        }

        if (bGreen)
        {
            colorAsString = "Green";
            if (n.getParam(ns + "/"+colorAsString+"/low", thres_low) && n.getParam(ns + "/"+colorAsString+"/high", thres_high) && n.getParam(ns + "/"+colorAsString+"/obj_shape", obj_shape) && n.getParam(ns + "/"+colorAsString+"/min_obj_sz", min_obj_sz) && n.getParam(ns + "/"+colorAsString+"/morph_sz", morph_sz) && n.getParam(ns + "/"+colorAsString+"/merge_thres", merge_thres))
            {
                //Thresholding
                inRange(imgBGR, Scalar(thres_low[0], thres_low[1], thres_low[2]), Scalar(thres_high[0], thres_high[1], thres_high[2]), imgBGRThres); //Threshold the BGR image
                inRange(imgHLS, Scalar(thres_low[6], thres_low[7], thres_low[8]), Scalar(thres_high[6], thres_high[7], thres_high[8]), imgHLSThres); //Threshold the HLS image
                inRange(imgLAB, Scalar(thres_low[3], thres_low[4], thres_low[5]), Scalar(thres_high[3], thres_high[4], thres_high[5]), imgLABThres); //Threshold the LAB image

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
                threshold(imgThresAllGray,imgThresSumGreen,merge_thres,255,THRESH_BINARY);

                morph_sz=max(morph_sz,1);
                //Width and height for morph
                int morph_width = morph_sz;
                int morph_height = morph_sz;

                //morphological opening (removes small objects from the foreground)
                erode(imgThresSumGreen, imgThresSumGreen, getStructuringElement(MORPH_ELLIPSE, Size(morph_width, morph_height)));
                dilate(imgThresSumGreen, imgThresSumGreen, getStructuringElement(MORPH_ELLIPSE, Size(morph_width, morph_height)));

                //morphological closing (removes small holes from the foreground)
                dilate(imgThresSumGreen, imgThresSumGreen, getStructuringElement(MORPH_ELLIPSE, Size(morph_width, morph_height)));
                erode(imgThresSumGreen, imgThresSumGreen, getStructuringElement(MORPH_ELLIPSE, Size(morph_width, morph_height)));

                bitwise_or(imgThresSumGreen,imgThresSum,imgThresSum);

            }
            else
            {
                ROS_INFO("Failed to get green thresholds and object parameters.");
            }
        }

        if (bRed)
        {
            colorAsString = "Red";
            if (n.getParam(ns + "/"+colorAsString+"/low", thres_low) && n.getParam(ns + "/"+colorAsString+"/high", thres_high) && n.getParam(ns + "/"+colorAsString+"/obj_shape", obj_shape) && n.getParam(ns + "/"+colorAsString+"/min_obj_sz", min_obj_sz) && n.getParam(ns + "/"+colorAsString+"/morph_sz", morph_sz) && n.getParam(ns + "/"+colorAsString+"/merge_thres", merge_thres))
            {
                //Thresholding
                inRange(imgBGR, Scalar(thres_low[0], thres_low[1], thres_low[2]), Scalar(thres_high[0], thres_high[1], thres_high[2]), imgBGRThres); //Threshold the BGR image
                inRange(imgHLS, Scalar(thres_low[6], thres_low[7], thres_low[8]), Scalar(thres_high[6], thres_high[7], thres_high[8]), imgHLSThres); //Threshold the HLS image
                inRange(imgLAB, Scalar(thres_low[3], thres_low[4], thres_low[5]), Scalar(thres_high[3], thres_high[4], thres_high[5]), imgLABThres); //Threshold the LAB image

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
                threshold(imgThresAllGray,imgThresSumRed,merge_thres,255,THRESH_BINARY);

                morph_sz=max(morph_sz,1);
                //Width and height for morph
                int morph_width = morph_sz;
                int morph_height = morph_sz;

                //morphological opening (removes small objects from the foreground)
                erode(imgThresSumRed, imgThresSumRed, getStructuringElement(MORPH_ELLIPSE, Size(morph_width, morph_height)));
                dilate(imgThresSumRed, imgThresSumRed, getStructuringElement(MORPH_ELLIPSE, Size(morph_width, morph_height)));

                //morphological closing (removes small holes from the foreground)
                dilate(imgThresSumRed, imgThresSumRed, getStructuringElement(MORPH_ELLIPSE, Size(morph_width, morph_height)));
                erode(imgThresSumRed, imgThresSumRed, getStructuringElement(MORPH_ELLIPSE, Size(morph_width, morph_height)));

                bitwise_or(imgThresSumRed,imgThresSum,imgThresSum);

            }
            else
            {
                ROS_INFO("Failed to get red thresholds and object parameters.");
            }
        }

        if (bOrange)
        {
            colorAsString = "Orange";
            if (n.getParam(ns + "/"+colorAsString+"/low", thres_low) && n.getParam(ns + "/"+colorAsString+"/high", thres_high) && n.getParam(ns + "/"+colorAsString+"/obj_shape", obj_shape) && n.getParam(ns + "/"+colorAsString+"/min_obj_sz", min_obj_sz) && n.getParam(ns + "/"+colorAsString+"/morph_sz", morph_sz) && n.getParam(ns + "/"+colorAsString+"/merge_thres", merge_thres))
            {
                //Thresholding
                inRange(imgBGR, Scalar(thres_low[0], thres_low[1], thres_low[2]), Scalar(thres_high[0], thres_high[1], thres_high[2]), imgBGRThres); //Threshold the BGR image
                inRange(imgHLS, Scalar(thres_low[6], thres_low[7], thres_low[8]), Scalar(thres_high[6], thres_high[7], thres_high[8]), imgHLSThres); //Threshold the HLS image
                inRange(imgLAB, Scalar(thres_low[3], thres_low[4], thres_low[5]), Scalar(thres_high[3], thres_high[4], thres_high[5]), imgLABThres); //Threshold the LAB image

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
                threshold(imgThresAllGray,imgThresSumOrange,merge_thres,255,THRESH_BINARY);

                morph_sz=max(morph_sz,1);
                //Width and height for morph
                int morph_width = morph_sz;
                int morph_height = morph_sz;

                //morphological opening (removes small objects from the foreground)
                erode(imgThresSumOrange, imgThresSumOrange, getStructuringElement(MORPH_ELLIPSE, Size(morph_width, morph_height)));
                dilate(imgThresSumOrange, imgThresSumOrange, getStructuringElement(MORPH_ELLIPSE, Size(morph_width, morph_height)));

                //morphological closing (removes small holes from the foreground)
                dilate(imgThresSumOrange, imgThresSumOrange, getStructuringElement(MORPH_ELLIPSE, Size(morph_width, morph_height)));
                erode(imgThresSumOrange, imgThresSumOrange, getStructuringElement(MORPH_ELLIPSE, Size(morph_width, morph_height)));

                bitwise_or(imgThresSumOrange,imgThresSum,imgThresSum);

            }
            else
            {
                ROS_INFO("Failed to get orange thresholds and object parameters.");
            }
        }

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
        //center of frame
        Point2f frame_center(icols_imgBGR / 2, irows_imgBGR / 2);

        if (cont_sz>0)
        {

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
                    mc_dist[i] = frame_center - mc[i];

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
            if (cont_sz>0)
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
            }

            //Publish gray image to ROS
            cvtColor(imgBGR, imgGray, CV_BGR2GRAY);
            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", imgGray).toImageMsg();
            sensor_msgs::ImagePtr dbg_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", imgThresSum).toImageMsg();
            img_pub.publish(img_msg);
            dbg_pub.publish(dbg_msg);

        }

        //ROS Topics
        //pose (setpoint - 2D float [m], heading - float [deg])
        //valid - bool
        //radius - float [m]
        Point2f obj_location;
        float x_dist = 0;
        float y_dist = 0;

        if (cont_sz>0)
        {
            //Circle
            if (obj_shape == 1)
            {
                //obj_location = mc[max_idx_c];
                obj_location = cc[max_idx_c];

                msg_pos.pose.x = obj_location.x;
                msg_pos.pose.y = obj_location.y;
                msg_pos.pose.theta = 0;
                msg_pos.radius = cr[max_idx_c];

                x_dist = obj_location.x - frame_center.x;
                y_dist = frame_center.y - obj_location.y;

                //convert to meters using emperical data fit and flip for NED
                msg_dist.pose.x = y_dist/100*(0.0018037*sqrt(pow(x_dist,2) + pow(y_dist,2)) + 0.3124266);
                msg_dist.pose.y = x_dist/100*(0.0018037*sqrt(pow(x_dist,2) + pow(y_dist,2)) + 0.3124266);
                msg_dist.pose.theta = 0;
                msg_dist.radius = cr[max_idx_c];

            }
            //Rotated rect
            else if (obj_shape == 2)
            {
                //obj_location = mc[max_idx_r];
                obj_location = minRect[max_idx_r].center;

                msg_pos.pose.x = obj_location.x;
                msg_pos.pose.y = obj_location.y;
                msg_pos.pose.theta = minRect[max_idx_r].angle;
                msg_pos.radius = 0;

                x_dist = obj_location.x - frame_center.x;
                y_dist = frame_center.y - obj_location.y;

                //convert to meters using emperical data fit and flip for NED
                msg_dist.pose.x = y_dist/100*(0.0018037*sqrt(pow(x_dist,2) + pow(y_dist,2)) + 0.3124266);
                msg_dist.pose.y = x_dist/100*(0.0018037*sqrt(pow(x_dist,2) + pow(y_dist,2)) + 0.3124266);
                msg_dist.pose.theta = minRect[max_idx_r].angle;
                msg_dist.radius = 0;
            }
            msg_pos.valid = true;
            msg_dist.valid = true;

        }
        else
        {
            msg_pos.valid = false;
            msg_dist.valid = false;
        }

        //ROS Publisher
        obj_pos_pub.publish(msg_pos);
        obj_dist_pub.publish(msg_dist);

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

