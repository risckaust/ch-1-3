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

struct MouseParams
{
    Mat img_LAB;
    Mat img_HLS;
    Mat img_BGR;
    Point3_<uchar> pt_LAB;
    Point3_<uchar> pt_HLS;
    Point3_<uchar> pt_BGR;
    bool bMouseClicked;
};

void mouseHandler( int event, int x, int y, int flags, void* param)
{

    if( event != CV_EVENT_LBUTTONDOWN )
        return;

// Mount back the parameters
    MouseParams* mp = (MouseParams*)param;
    Mat img_LAB = mp->img_LAB;
    Mat img_HLS = mp->img_HLS;
    Mat img_BGR = mp->img_BGR;

    Point3_<uchar>* point_LAB = img_LAB.ptr<Point3_<uchar> >(y,x);
    Point3_<uchar>* point_HLS = img_HLS.ptr<Point3_<uchar> >(y,x);
    Point3_<uchar>* point_BGR = img_BGR.ptr<Point3_<uchar> >(y,x);
//Point3_<uchar>* point = img->ptr<Point3_<uchar> >(y,x);
    mp->pt_LAB = *point_LAB;
    mp->pt_HLS = *point_HLS;
    mp->pt_BGR = *point_BGR;
    mp->bMouseClicked = true;

    int vL=point_LAB->x; //hue
    int vA=point_LAB->y; //saturation
    int vB=point_LAB->z; //value

    int vHue=point_HLS->x; //hue
    int vSat=point_HLS->y; //saturation
    int vLum=point_HLS->z; //value

    int vBlue=point_BGR->x; //blue
    int vGreen=point_BGR->y; //green
    int vRed=point_BGR->z; //red

//cout << "L:" << vL << " A:" << vA << " B:" << vB << endl;
//cout << "H:" << vHue << " S:" << vSat << " L:" << vLum << endl;
//cout << "B:" << vBlue << " G:" << vGreen << " R:" << vRed << endl;

}

cv_bridge::CvImagePtr cv_img_ptr_ros;

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
    MouseParams mp;
    mp.bMouseClicked = false;
    //pause and resume code
    bool bPause = false;
    bool bESC = false;

    //Modes for convenience
    bool bDebug = false;
    bool bCtrl = false;
    bool bOutputVideo = false;
    bool bCamera = false;
    bool bVideo = false;
    bool bViz = false;
    bool bCompetition = false;
    bool bLoad = false;
    bool bSend = false;
    bool bWrite = false;
    bool bStream = false;

    //Object shapes
    int obj_shape = 1; //0-upright bouding box, 1-circle, 2-rotated rect
    int red_shape = 1; //circle
    int green_shape = 1; //circle
    int blue_shape = 1; //circle
    int yellow_shape = 2; //rotated rect
    int box_shape = 2; //rotated rect

    //Fit shapes
    bool bFitBoundingBox = false;
    bool bFitRotatedRect = false;
    bool bFitCircle = true;
    bool bUpdateThres = false;

    //Set min object size
    int min_obj_sz = 5;
    int obj_sz = 0;
    int thres_tol_LAB = 10;
    int thres_tol_HLS = 5;
    int thres_tol_BGR = 30;
    int morph_sz = 5;
    int merge_thres = 100;

    int ex = CV_FOURCC('D', 'I', 'V', 'X');     //Codec Type- Int form
    int frame_counter = 0;
    int frame_count_max = -1; //infinite
    int frame_rate = 30;
    int stream_rate = 1;

    int color=0; //0-default, 1-red, 2-green, 3-blue, 4-yellow, 5-box
    std::string colorAsString = "None";

    std::string pkgpath = "/home/odroid/ros_ws/src/ch-1-3/cvision";
    std::string srcpath = "/home/odroid/ros_ws/src/ch-1-3/cvision/src";
    std::string img_tp = "/cv_camera/image_raw";

    //ROS Init
    ros::init(argc, argv, "threshold_tuning");

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
    ros::Publisher object_pub = n.advertise<cvision::ObjectPose>("objPose",1000);
    // Publish video stream
    image_transport::Publisher img_pub = it.advertise("objImg", 10);

    //Set loop rate for ros
    ros::Rate loop_rate(frame_rate);

    /* list variables to be published in ROS parameters server */
    std::vector<int> thres_low(9);
    std::vector<int> thres_high(9);

    string configFile = srcpath + "/config.txt";
    ifstream f_config(configFile.c_str());
    if (!f_config)
    {
        cout << "error: could not load config file," << endl;
    }

    string txt_line, name, tmp;
    while (getline(f_config, txt_line))
    {
        istringstream iss(txt_line);
        iss >> name >> tmp;

        // skip invalid lines and comments
        if (iss.fail() || tmp != "=" || name[0] == '#') continue;

        if (name == "bDebug") iss >> bDebug;
        else if (name == "bCtrl") iss >> bCtrl;
        else if (name == "bOutputVideo") iss >> bOutputVideo;
        else if (name == "bCamera") iss >> bCamera;
        else if (name == "bVideo") iss >> bVideo;
        else if (name == "bViz") iss >> bViz;
        else if (name == "bCompetition") iss >> bCompetition;
        else if (name == "obj_shape") iss >> obj_shape;
        else if (name == "min_obj_sz") iss >> min_obj_sz;
        else if (name == "thres_tol_LAB") iss >> thres_tol_LAB;
        else if (name == "thres_tol_HLS") iss >> thres_tol_HLS;
        else if (name == "thres_tol_BGR") iss >> thres_tol_BGR;
        else if (name == "merge_thres") iss >> merge_thres;
        else if (name == "morph_sz") iss >> morph_sz;
        else if (name == "frame_rate") iss >> frame_rate;
    }

    int thres_tol_LAB_old = thres_tol_LAB;
    int thres_tol_HLS_old = thres_tol_HLS;
    int thres_tol_BGR_old = thres_tol_BGR;

    RNG rng(12345);

    Size imgSz;
    VideoCapture cap;
    VideoWriter outputVideo;

    if (bCamera && !bCompetition)
    {
        // open the default camera, use something different from 0 otherwise;
        cap.open(0);

        if (!cap.isOpened())  // if not success, exit program
        {
            cout << "Cannot open webcam" << endl;
            return -1;
        }
        imgSz = Size((int)cap.get(CV_CAP_PROP_FRAME_WIDTH),    // Acquire input size
                     (int)cap.get(CV_CAP_PROP_FRAME_HEIGHT));
    }
    else if (bVideo && !bCompetition)
    {
        string inputFile = srcpath + "/InputVideo.avi";
        cap.open(inputFile);

        if (!cap.isOpened())  // if not success, exit program
        {
            cout << "Cannot open video" << endl;
            return -1;
        }

        imgSz = Size((int)cap.get(CV_CAP_PROP_FRAME_WIDTH),    // Acquire input size
                     (int)cap.get(CV_CAP_PROP_FRAME_HEIGHT));

        frame_counter = 0;
        frame_count_max = cap.get(CV_CAP_PROP_FRAME_COUNT); //get the frame count
        cout << "Number of frames: " << frame_count_max << endl;

        ex = static_cast<int>(cap.get(CV_CAP_PROP_FOURCC));     // Get Codec Type- Int form

        // Transform from int to char via Bitwise operators
        //char EXT[] = { (char)(ex & 0XFF), (char)((ex & 0XFF00) >> 8), (char)((ex & 0XFF0000) >> 16), (char)((ex & 0XFF000000) >> 24), 0 };

        frame_rate = cap.get(CV_CAP_PROP_FPS);
    }

    if (bOutputVideo && !bCompetition)
    {
        const string NAME = srcpath + "/ProcessedVideo.avi";   // Form the new name with container

        // Open the output
        outputVideo.open(NAME, ex, frame_rate, imgSz, true);

        if (!outputVideo.isOpened())
        {
            cout << "Could not open the output video for write" << endl;
            return -1;
        }
    }

    //Set threshold values
    int iLowHue = 0;
    int iHighHue = 179;
    int iLowSat = 0;
    int iHighSat = 255;
    int iLowLum = 0;
    int iHighLum = 255;

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


    string colorThres = srcpath + "/ThresholdValues.txt";
    ifstream f_colorThres(colorThres.c_str());
    if (!f_colorThres)
    {
        cout << "error: could not load color threshold file," << endl;
    }

    string txt_line1, name1, tmp1;
    while (getline(f_colorThres, txt_line1))
    {
        istringstream iss(txt_line1);
        iss >> name1 >> tmp1;

        // skip invalid lines and comments
        if (iss.fail() || tmp1 != "=" || name1[0] == '#') continue;

        if (name1 == "iLowHue") iss >> iLowHue;
        else if (name1 == "iHighHue") iss >> iHighHue;
        else if (name1 == "iLowSat") iss >> iLowSat;
        else if (name1 == "iHighSat") iss >> iHighSat;
        else if (name1 == "iLowLum") iss >> iLowLum;
        else if (name1 == "iHighLum") iss >> iHighLum;
        else if (name1 == "iLowBlue") iss >> iLowBlue;
        else if (name1 == "iHighBlue") iss >> iHighBlue;
        else if (name1 == "iLowGreen") iss >> iLowGreen;
        else if (name1 == "iHighGreen") iss >> iHighGreen;
        else if (name1 == "iLowRed") iss >> iLowRed;
        else if (name1 == "iHighRed") iss >> iHighRed;
        else if (name1 == "iLowL") iss >> iLowL;
        else if (name1 == "iHighL") iss >> iHighL;
        else if (name1 == "iLowA") iss >> iLowA;
        else if (name1 == "iHighA") iss >> iHighA;
        else if (name1 == "iLowB") iss >> iLowB;
        else if (name1 == "iHighB") iss >> iHighB;
    }

//Initialize some values
    int vHue=floor((iLowHue+iHighHue)/2); //hue
    int vSat=floor((iLowSat+iHighSat)/2); //saturation
    int vLum=floor((iLowLum+iHighLum)/2); //luminosity

    int vBlue=floor((iLowBlue+iHighBlue)/2); //blue
    int vGreen=floor((iLowGreen+iHighGreen)/2); //green
    int vRed=floor((iLowRed+iHighRed)/2); //red

    int vL=floor((iLowL+iHighL)/2); //L
    int vA=floor((iLowA+iHighA)/2); //A
    int vB=floor((iLowB+iHighB)/2); //B

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

    cout << "Ready to loop..." << endl;
    while (frame_counter != frame_count_max && !bESC  && ros::ok())
    {
        //cout << "Frame: " << frame_counter << endl;

        if ( (bCamera || bVideo) && !bCompetition)
        {
            bool bSuccess = cap.read(imgBGR); // read a new frame from video

            if (!bSuccess) //if not success, break loop
            {
                cout << "Cannot read a frame from video stream" << endl;
                break;
            }

        }

        else if (cv_img_ptr_ros)
        {
            imgBGR = cv_img_ptr_ros->image;
        }
        else
        {
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }

        frame_counter++;

        //Determine size of video input
        int irows_imgBGR = imgBGR.rows;
        int icols_imgBGR = imgBGR.cols;

        imgSz = Size(icols_imgBGR,irows_imgBGR);
        cvtColor(imgBGR, imgHLS, COLOR_BGR2HLS); //Convert the captured frame from BGR to HLS
        cvtColor(imgBGR, imgLAB, COLOR_BGR2Lab); //Convert the captured frame from BGR to LAB

        ///////Thresholding
        inRange(imgBGR, Scalar(iLowBlue, iLowGreen, iLowRed), Scalar(iHighBlue, iHighGreen, iHighRed), imgBGRThres); //Threshold the image
        inRange(imgHLS, Scalar(iLowHue, iLowLum, iLowSat), Scalar(iHighHue, iHighLum, iHighSat), imgHLSThres); //Threshold the image
        inRange(imgLAB, Scalar(iLowL, iLowA, iLowB), Scalar(iHighL, iHighA, iHighB), imgLABThres); //Threshold the image


        imgThres3[0] = imgBGRThres;
        imgThres3[1] = imgLABThres;
        imgThres3[2] = imgHLSThres;
        merge(imgThres3,3,imgThresAll);

        //gray = 0.114b + 0.587g + 0.299r
        //100, 160, 200
        cvtColor(imgThresAll, imgThresAllGray, COLOR_BGR2GRAY);
        threshold(imgThresAllGray,imgThresSum,merge_thres,255,THRESH_BINARY);


        morph_sz=max(morph_sz,1);
        //Width and height for morph
        int morph_width = morph_sz;
        int morph_height = morph_sz;

        //GaussianBlur(imgHLSThres, imgHLSThres, Size(0, 0),morph_sz);
        //blur(imgHLSThres, imgHLSThres, Size(morph_width, morph_height));

        //morphological opening (removes small objects from the foreground)
        erode(imgThresSum, imgThresSum, getStructuringElement(MORPH_ELLIPSE, Size(morph_width, morph_height)));
        dilate(imgThresSum, imgThresSum, getStructuringElement(MORPH_ELLIPSE, Size(morph_width, morph_height)));

        //morphological closing (removes small holes from the foreground)
        dilate(imgThresSum, imgThresSum, getStructuringElement(MORPH_ELLIPSE, Size(morph_width, morph_height)));
        erode(imgThresSum, imgThresSum, getStructuringElement(MORPH_ELLIPSE, Size(morph_width, morph_height)));

        imgContours = imgThresSum.clone();

        //Create a black image with the size as the camera output
        Mat drawing = Mat::zeros(imgSz, CV_8UC3 );
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        //cvFindContours()
        findContours(imgContours, contours, hierarchy,
                     CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        int cont_sz = contours.size();

        int x_SP = 0;
        int y_SP = 0;
        float r_SP = 0;
        float t_SP = 0;

        if (cont_sz>0)
        {
            vector<Moments> mu(cont_sz);
            vector<vector<Point> > contours_poly(cont_sz);
            vector<Rect> boundRect(cont_sz);
            vector<RotatedRect> minRect(cont_sz);
            vector<Point2f> mc(cont_sz);
            vector<Point2f> mc_dist(cont_sz);
            vector<Point2f> cc(cont_sz);
            vector<float> cr(cont_sz);
            vector<int> minBoundArea(cont_sz);
            vector<int> minRectArea(cont_sz);
            vector<int> minCircleArea(cont_sz);
            int max_idx_b= 0;
            int max_idx_c= 0;
            int max_idx_r= 0;

            //center of frame
            Point2f frameCenter(icols_imgBGR / 2, irows_imgBGR / 2);

            /// Approximate contours to polygons + get bounding rects and circles
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

//                    approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
                    if (bFitBoundingBox || bDebug)
                    {
                        boundRect[i] = boundingRect( Mat(contours_poly[i]) );
                        minBoundArea[i]=boundRect[i].width*boundRect[i].height;
                        if (minBoundArea[max_idx_b]<minBoundArea[i])
                        {
                            max_idx_b = i;
                        }
                    }
                    if (bFitRotatedRect || bDebug)
                    {
                        minRect[i] = minAreaRect( Mat(contours_poly[i]) );
                        minRectArea[i]=minRect[i].size.width*minRect[i].size.height;
                        if (minRectArea[max_idx_r]<minRectArea[i])
                        {
                            max_idx_r = i;
                        }
                    }
                    if (bFitCircle || bDebug)
                    {
                        minEnclosingCircle( (Mat)contours_poly[i], cc[i], cr[i] );
                        minCircleArea[i]=cr[i]*cr[i]*CV_PI;
                        if (minCircleArea[max_idx_c]<minCircleArea[i])
                        {
                            max_idx_c = i;
                        }
                    }
                    //cout << "Bounding Box: " << boundRect[i] << endl;
                    //cout << "Smallest Rect: " << minRect[i] << endl;
                    //cout << "Smallest Circle: " << cc[i] << ", " << cr[i] << endl;
                }
            }

            //Circle
            if (bFitCircle)
            {
                //x_SP = mc[max_idx_c].x;
                //y_SP = mc[max_idx_c].y;
                x_SP = cc[max_idx_c].x;
                y_SP = cc[max_idx_c].y;
                r_SP = cr[max_idx_c];
            }
            //Rotated rect
            else if (bFitRotatedRect)
            {
                //x_SP = mc[max_idx_r].x;
                //y_SP = mc[max_idx_r].y;
                x_SP = minRect[max_idx_r].center.x;
                y_SP = minRect[max_idx_r].center.y;
                t_SP = minRect[max_idx_r].angle;
            }

            //cout << "x: " << x_SP << " y: " << y_SP << " r: " << r_SP << " t: " << t_SP << endl;

            //Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            Scalar color = Scalar(0, 255, 255);
            Scalar color1 = Scalar(255, 0, 0);
            Scalar color2 = Scalar(0, 255, 0);
            Scalar color3 = Scalar(0, 0, 255);

            if ((bViz && !bCompetition) || bStream)
            {
                if (bFitBoundingBox)
                {
                    //bounding box
                    rectangle(imgBGR, boundRect[max_idx_b].tl(), boundRect[max_idx_b].br(), color1, 2, 8, 0 );
                }
                if (bFitRotatedRect)
                {
                    //rotated rectangle
                    Point2f rect_points[4];
                    minRect[max_idx_r].points( rect_points );
                    for( int j = 0; j < 4; j++ )
                        line(imgBGR, rect_points[j], rect_points[(j+1)%4], color2, 1, 8 );
                    //Center
                    //circle(imgBGR, mc[max_idx_r], 5, color, -1, 8, 0);
                    circle(imgBGR, minRect[max_idx_r].center, 5, color, -1, 8, 0);
                }
                if (bFitCircle)
                {
                    //min circle
                    circle(imgBGR, cc[max_idx_c], (int)cr[max_idx_c], color3, 2, 8, 0 );
                    //Center
                    //circle(imgBGR, mc[max_idx_c], 5, color, -1, 8, 0);
                    circle(imgBGR, cc[max_idx_c], 5, color, -1, 8, 0);
                }
                //putText(imgBGR, "Object Detected", mc[i] + Point2f(50, 50), 1, 2, Scalar(150, 0, 0), 2);
            }

            if (bDebug && !bCompetition)
            {
                for (int i = 0; i< contours.size(); i++)
                {
                    if (obj_sz>min_obj_sz) //Minimum size for object, otherwise it is considered noise
                    {

                        /// Draw polygonal contour + bonding rects + circles
                        //poly contours
                        //drawContours( drawing, contours_poly, i, color, 1, 8, hierarchy, 0, Point() );
                        drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
                        //bounding box
                        rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color1, 2, 8, 0 );
                        //rotated rectangle
                        Point2f rect_points[4];
                        minRect[i].points( rect_points );
                        for( int j = 0; j < 4; j++ )
                            line( drawing, rect_points[j], rect_points[(j+1)%4], color2, 1, 8 );
                        //min circle
                        circle( drawing, cc[i], (int)cr[i], color3, 2, 8, 0 );
                    }

                }
            }
        }
        else
        {
            //No contours found
        }


        if (!bCompetition)
        {

            //Adjust thresholds with values selected by mouse
            if(mp.bMouseClicked)
            {
                mp.bMouseClicked = false;
                vHue=mp.pt_HLS.x; //hue
                vSat=mp.pt_HLS.y; //saturation
                vLum=mp.pt_HLS.z; //value

                vBlue=mp.pt_BGR.x; //blue
                vGreen=mp.pt_BGR.y; //green
                vRed=mp.pt_BGR.z; //red

                vL=mp.pt_LAB.x; //L
                vA=mp.pt_LAB.y; //A
                vB=mp.pt_LAB.z; //B

                bUpdateThres = true;

                cout << "L:" << vL << " A:" << vA << " B:" << vB << endl;
                cout << "H:" << vHue << " S:" << vSat << " L:" << vLum << endl;
                cout << "B:" << vBlue << " G:" << vGreen << " R:" << vRed << endl;
            }

            if (bDebug == true)
            {
                imshow( "PolyContours", drawing );
                drawing = Mat::zeros(imgSz, CV_8UC3 );
                //imshow("Contour Image", imgContours); //show the thresholded image
                //imshow("ThresholdedImageLAB", imgLABThres); //show the thresholded image
                //imshow("ThresholdedImageHLS", imgHLSThres); //show the thresholded image
                //imshow("ThresholdedImageBGR", imgBGRThres); //show the thresholded image
                imshow("ThresholdedImageAll", imgThresAll); //show the thresholded image
            }
            else
            {
                //if not in debug mode, destroy the window
                cv::destroyWindow("PolyContours");
                //cv::destroyWindow("Contour Image");
                //cv::destroyWindow("ThresholdedImageLAB");
                //cv::destroyWindow("ThresholdedImageHLS");
                //cv::destroyWindow("ThresholdedImageBGR");
                cv::destroyWindow("ThresholdedImageAll");

            }

            if (bViz == true)
            {
                /// Show in a window
                imshow("VideoFeed", imgBGR); //show the original image
                imshow("ThresholdedImage", imgThresSum); //show the thresholded image
                createTrackbar("ToleranceBGR", "ThresholdedImage", &thres_tol_BGR, 100);
                createTrackbar("ToleranceLAB", "ThresholdedImage", &thres_tol_LAB, 50);
                createTrackbar("ToleranceHLS", "ThresholdedImage", &thres_tol_HLS, 25);
                createTrackbar("Object Sz", "ThresholdedImage", &min_obj_sz, 100);
                createTrackbar("Morph Sz", "ThresholdedImage", &morph_sz, 10);

                mp.img_LAB = imgLAB;
                mp.img_HLS = imgHLS;
                mp.img_BGR = imgBGR;
                cv::setMouseCallback("VideoFeed", mouseHandler, (void*)&mp);
            }
            else
            {
                //if not in viz mode, destroy the windows
                cv::destroyWindow("VideoFeed");
                cv::destroyWindow("ThresholdedImage");
            }


            if (bCtrl == true)
            {
                namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
                //Create trackbars in "Control" window
                createTrackbar("LowHue", "Control", &iLowHue, 179); //Hue (0 - 179)
                createTrackbar("HighHue", "Control", &iHighHue, 179);
                createTrackbar("LowSat", "Control", &iLowSat, 255); //Saturation (0 - 255)
                createTrackbar("HighSat", "Control", &iHighSat, 255);
                createTrackbar("LowLum", "Control", &iLowLum, 255);//Value (0 - 255)
                createTrackbar("HighLum", "Control", &iHighLum, 255);

                createTrackbar("LowBlue", "Control", &iLowBlue, 255); //Blue (0 - 179)
                createTrackbar("HighBlue", "Control", &iHighBlue, 255);
                createTrackbar("LowGreen", "Control", &iLowGreen, 255); //Green (0 - 255)
                createTrackbar("HighGreen", "Control", &iHighGreen, 255);
                createTrackbar("LowRed", "Control", &iLowRed, 255);//Red (0 - 255)
                createTrackbar("HighRed", "Control", &iHighRed, 255);

                createTrackbar("LowL", "Control", &iLowL, 255); //L (0 - 179)
                createTrackbar("HighL", "Control", &iHighL, 255);
                createTrackbar("LowA", "Control", &iLowA, 255); //A (0 - 255)
                createTrackbar("HighA", "Control", &iHighA, 255);
                createTrackbar("LowB", "Control", &iLowB, 255);//B (0 - 255)
                createTrackbar("HighB", "Control", &iHighB, 255);
            }
            else
            {
                //if not in ctrl mode, destroy the window
                cv::destroyWindow("Control");
            }

            //If threshold tolerance was changed update values.
            if ((thres_tol_LAB_old - thres_tol_LAB != 0) || bUpdateThres)
            {
                iLowL = max(vL-thres_tol_LAB*10,0);
                iHighL = min(vL+thres_tol_LAB*10,255);
                iLowA = max(vA-thres_tol_LAB,0);
                iHighA = min(vA+thres_tol_LAB,255);
                iLowB = max(vB-thres_tol_LAB,0);
                iHighB = min(vB+thres_tol_LAB,255);
                thres_tol_LAB_old = thres_tol_LAB;
            }

            //If threshold tolerance was changed update values.
            if ((thres_tol_HLS_old - thres_tol_HLS != 0) || bUpdateThres)
            {
                iLowHue = max(vHue-thres_tol_HLS,0);
                iHighHue = min(vHue+thres_tol_HLS,179);
                iLowSat = max(vSat-thres_tol_HLS*10,0);
                iHighSat = min(vSat+thres_tol_HLS*10,255);
                iLowLum = max(vLum-thres_tol_HLS*10,0);
                iHighLum = min(vLum+thres_tol_HLS*10,255);
                thres_tol_HLS_old = thres_tol_HLS;
            }

            //If threshold tolerance was changed update values.
            if ((thres_tol_BGR_old - thres_tol_BGR != 0) || bUpdateThres)
            {
                if (vBlue < 85)
                {
                    iLowBlue = 0;
                }
                else
                {
                    iLowBlue = vBlue-thres_tol_BGR;
                }
                if (vBlue > 170)
                {
                    iHighBlue = 255;
                }
                else
                {
                    iHighBlue = vBlue+thres_tol_BGR;
                }
                if (vGreen < 85)
                {
                    iLowGreen = 0;
                }
                else
                {
                    iLowGreen = vGreen-thres_tol_BGR;
                }
                if (vGreen > 170)
                {
                    iHighGreen = 255;
                }
                else
                {
                    iHighGreen = vGreen+thres_tol_BGR;
                }
                if (vRed < 85)
                {
                    iLowRed = 0;
                }
                else
                {
                    iLowRed = vRed-thres_tol_BGR;
                }
                if (vRed > 170)
                {
                    iHighRed = 255;
                }
                else
                {
                    iHighRed = vRed+thres_tol_BGR;
                }

                thres_tol_BGR_old = thres_tol_BGR;
            }

            bUpdateThres = false;

            if (bOutputVideo)
            {
                outputVideo << imgBGR;
            }


            int key = (waitKey(30) & 0xFF);
            //Check for key presses
            switch (key)
            {
            //cout << "Reached switch statement..." << endl;
            case 27: //'esc' key has been pressed, exit program.
                bESC = 1;
                break;

            case 114: //'r' has been pressed.
                color = 1;
                cout << "Color: Red" << endl;
                break;

            case 103: //'g' has been pressed.
                color = 2;
                cout << "Color: Green" << endl;
                break;

            case 98: //'b' has been pressed.
                color = 3;
                cout << "Color: Blue" << endl;
                break;

            case 121: //'y' has been pressed.
                color = 4;
                cout << "Color: Yellow" << endl;
                break;

            case 120: //'x' has been pressed.
                color = 5;
                cout << "Color: Box" << endl;
                break;

            case 108: //'l' has been pressed.
                bLoad = true;
                cout << "Loading triggered." << endl;
                break;

            case 115: //'s' has been pressed.
                bSend = true;
                cout << "Sending triggered." << endl;
                break;

            case 119: //'w' has been pressed. Toggle visualization
                bWrite = true;
                cout << "Writing triggered." << endl;
                break;

            case 100: //'d' has been pressed. Toggle debug
                bDebug = !bDebug;
                if (bDebug == false) cout << "Debug disabled." << endl;
                else cout << "Debug enabled." << endl;
                break;

            case 99: //'c' has been pressed. Toggle control
                bCtrl = !bCtrl;
                if (bCtrl == false) cout << "Control disabled." << endl;
                else cout << "Control enabled." << endl;
                break;

            case 118: //'v' has been pressed. Toggle visualization
                bViz = !bViz;
                if (bViz == false) cout << "Visualization disabled." << endl;
                else cout << "Visualization enabled." << endl;
                break;

            case 112: //'p' has been pressed. this will pause/resume the code.
                bPause = !bPause;
                if (bPause == true)
                {
                    cout << "Code paused, press 'p' again to resume" << endl;
                    while (bPause == true)
                    {
                        key = (waitKey(30) & 0xFF);
                        //stay in this loop until
                        switch (key)
                        {
                        //a switch statement inside a switch statement? Mind blown.
                        case 112:
                            //change pause back to false
                            bPause = false;
                            cout << "Code Resumed" << endl;
                            break;
                        }
                    }
                }
            }


            switch (color)
            {
            case 1: //Red color selected.
                obj_shape = red_shape;
                colorAsString = "Red";
                break;
            case 2: //Green color selected.
                obj_shape = green_shape;
                colorAsString = "Green";
                break;
            case 3: //Blue color selected.
                obj_shape = blue_shape;
                colorAsString = "Blue";
                break;
            case 4: //Yellow color selected.
                obj_shape = yellow_shape;
                colorAsString = "Yellow";
                break;
            case 5: // box
                obj_shape = box_shape;
                colorAsString = "Box";
                break;
            }


            switch (obj_shape)
            {
            case 0:
                bFitBoundingBox = true;
                bFitCircle = false;
                bFitRotatedRect = false;
                break;
            case 1:
                bFitBoundingBox = false;
                bFitCircle = true;
                bFitRotatedRect = false;
                break;
            case 2:
                bFitBoundingBox = false;
                bFitCircle = false;
                bFitRotatedRect = true;
                break;
            }

            if (bLoad)
            {
                if (n.getParam(ns + "/" + colorAsString + "/low", thres_low) && n.getParam(ns + "/" + colorAsString + "/high", thres_high))
                {
                    ROS_INFO("Got Thresholds.");
                    iLowBlue = thres_low[0];
                    iLowGreen = thres_low[1];
                    iLowRed = thres_low[2];
                    iLowL = thres_low[3];
                    iLowA = thres_low[4];
                    iLowB = thres_low[5];
                    iLowHue = thres_low[6];
                    iLowLum = thres_low[7];
                    iLowSat = thres_low[8];
                    iHighBlue = thres_high[0];
                    iHighGreen = thres_high[1];
                    iHighRed = thres_high[2];
                    iHighL = thres_high[3];
                    iHighA = thres_high[4];
                    iHighB = thres_high[5];
                    iHighHue = thres_high[6];
                    iHighLum = thres_high[7];
                    iHighSat = thres_high[8];
                }
                else
                {
                    ROS_INFO("Failed to get thresholds.");
                }

                if (n.getParam(ns + "/" + colorAsString + "/obj_shape", obj_shape) && n.getParam(ns + "/" + colorAsString + "/min_obj_sz", min_obj_sz) && n.getParam(ns + "/" + colorAsString + "/morph_sz", morph_sz) && n.getParam(ns + "/" + colorAsString + "/merge_thres", merge_thres))
                {
                    ROS_INFO("Got Object Parameters.");
                }
                else
                {
                    ROS_INFO("Failed to get object parameters.");
                }

                bLoad = false;
                cout << "Loading " << colorAsString << " parameters complete." << endl;
            }

            if (bSend)
            {
                thres_low[0] = iLowBlue;
                thres_low[1] = iLowGreen;
                thres_low[2] = iLowRed;
                thres_low[3] = iLowL;
                thres_low[4] = iLowA;
                thres_low[5] = iLowB;
                thres_low[6] = iLowHue;
                thres_low[7] = iLowLum;
                thres_low[8] = iLowSat;
                thres_high[0] = iHighBlue;
                thres_high[1] = iHighGreen;
                thres_high[2] = iHighRed;
                thres_high[3] = iHighL;
                thres_high[4] = iHighA;
                thres_high[5] = iHighB;
                thres_high[6] = iHighHue;
                thres_high[7] = iHighLum;
                thres_high[8] = iHighSat;

                // send low values
                n.setParam("/Quad1/" + colorAsString + "/low", thres_low);
                n.setParam("/Quad2/" + colorAsString + "/low", thres_low);
                n.setParam("/Quad3/" + colorAsString + "/low", thres_low);
                // send high values
                n.setParam("/Quad1/" + colorAsString + "/high", thres_high);
                n.setParam("/Quad2/" + colorAsString + "/high", thres_high);
                n.setParam("/Quad3/" + colorAsString + "/high", thres_high);
                // send obj_shape
                n.setParam("/Quad1/" + colorAsString + "/obj_shape", obj_shape);
                n.setParam("/Quad2/" + colorAsString + "/obj_shape", obj_shape);
                n.setParam("/Quad3/" + colorAsString + "/obj_shape", obj_shape);
                // send min_obj_sz
                n.setParam("/Quad1/" + colorAsString + "/min_obj_sz", min_obj_sz);
                n.setParam("/Quad2/" + colorAsString + "/min_obj_sz", min_obj_sz);
                n.setParam("/Quad3/" + colorAsString + "/min_obj_sz", min_obj_sz);
                // send min_obj_sz
                n.setParam("/Quad1/" + colorAsString + "/morph_sz", morph_sz);
                n.setParam("/Quad2/" + colorAsString + "/morph_sz", morph_sz);
                n.setParam("/Quad3/" + colorAsString + "/morph_sz", morph_sz);
                // send merge_thres
                n.setParam("/Quad1/" + colorAsString + "/merge_thres", merge_thres);
                n.setParam("/Quad2/" + colorAsString + "/merge_thres", merge_thres);
                n.setParam("/Quad3/" + colorAsString + "/merge_thres", merge_thres);

                bSend = false;
                cout << "Sending " << colorAsString << " parameters complete." << endl;
            }

            if (bWrite)
            {

                string configYaml = pkgpath + "/configs/color_thresholds.yaml";
                ifstream streamYaml (configYaml.c_str());
                if (!streamYaml)
                {
                    cout << "error: could not load yaml color_thresholds file," << endl;
                }

                int idx = 0;
                std::vector<std::string> text_file(10);
                string txt_line;
                while (getline(streamYaml, txt_line))
                {
                    text_file[idx] = txt_line;
                    idx++;
                }

                ofstream fileYaml(configYaml.c_str());
                //Save values to file
                if (fileYaml.is_open())
                {
                    for (int i=0; i<10; i++)
                    {

                        if (i==color)
                        {
                            fileYaml << colorAsString << ": {"
                                     <<"low: [" << iLowBlue << ", " << iLowGreen << ", " << iLowRed << ", " << iLowL << ", " << iLowA << ", " << iLowB << ", " << iLowHue << ", " << iLowLum << ", " << iLowSat << "],"
                                     << "high: [" << iHighBlue << ", " << iHighGreen << ", " << iHighRed << ", " << iHighL << ", " << iHighA << ", " << iHighB << ", " << iHighHue << ", " << iHighLum << ", " << iHighSat << "],"
                                     << "obj_shape: " << obj_shape << ","
                                     << "min_obj_sz: " << min_obj_sz << ","
                                     << "morph_sz: " << morph_sz << ","
                                     << "merge_thres: " << merge_thres << "} \n";
                        }
                        else
                        {
                            fileYaml << text_file[i] << "\n";
                        }
                    }
                    fileYaml.close();
                    cout << "Writing " << colorAsString << " parameters complete." << endl;
                }
                else cout << "Unable to open file";

                bWrite = false;
            }

            //end if(!bCompetition)
        }


        if ( (bStream && (((frame_counter*stream_rate)%frame_rate)<stream_rate) ) )
        {
            //Publish gray image to ROS
            cvtColor(imgBGR, imgGray, CV_BGR2GRAY);
            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", imgGray).toImageMsg();
            img_pub.publish(img_msg);
        }

        //ROS Topics
        //pose (setpoint - 2D float [m], heading - float [deg])
        //valid - bool
        //radius - float [m]
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

    string newThres = srcpath + "/ThresholdValuesNew.txt";
    switch (color)
    {
    case 1: //Red color selected.
        newThres = srcpath + "/ThresholdValuesRed.txt";
        break;

    case 2: //Green color selected.
        newThres = srcpath + "/ThresholdValuesGreen.txt";
        break;

    case 3: //Blue color selected.
        newThres = srcpath + "/ThresholdValuesBlue.txt";
        break;

    case 4: //Yellow color selected.
        newThres = srcpath + "/ThresholdValuesYellow.txt";
        break;

    case 5: //Box color selected.
        newThres = srcpath + "/ThresholdValuesBox.txt";
        break;
    }

    ofstream myfile(newThres.c_str());
    //Save values to file
    if (myfile.is_open())
    {
        myfile << "iLowHue = " << iLowHue << "\n";
        myfile << "iHighHue = " << iHighHue << "\n";
        myfile << "iLowSat = " << iLowSat << "\n";
        myfile << "iHighSat = " << iHighSat << "\n";
        myfile << "iLowLum = " << iLowLum << "\n";
        myfile << "iHighLum = " << iHighLum << "\n";
        myfile << "iLowBlue = " << iLowBlue << "\n";
        myfile << "iHighBlue = " << iHighBlue << "\n";
        myfile << "iLowGreen = " << iLowGreen << "\n";
        myfile << "iHighGreen = " << iHighGreen << "\n";
        myfile << "iLowRed = " << iLowRed << "\n";
        myfile << "iHighRed = " << iHighRed << "\n";
        myfile << "iLowL = " << iLowL << "\n";
        myfile << "iHighL = " << iHighL << "\n";
        myfile << "iLowA = " << iLowA << "\n";
        myfile << "iHighA = " << iHighA << "\n";
        myfile << "iLowB = " << iLowB << "\n";
        myfile << "iHighB = " << iHighB << "\n";
        myfile.close();
        cout << "Finished writing" << endl;
    }
    else cout << "Unable to open file";
    return 0;
}
