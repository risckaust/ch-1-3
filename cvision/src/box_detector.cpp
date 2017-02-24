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
    //pause and resume code
    bool bPause = false;
    bool bESC = false;

    //Modes for convenience
    bool bDebug = false;
    bool bCtrl = false;
    bool bOutputVideo = false;
    bool bCamera = false;
    bool bVideo = false;
    bool bROS = false;
    bool bViz = false;
    bool bCompetition = false;
    //Fit shapes
    bool bFitBoundingBox = true;
    bool bFitRotatedRect = true;
    bool bFitCircle = true;
    //Set min object size
    int min_obj_sz = 5;
    int thres_tol = 50;
    int morph_sz = 5;
    int frameRate = 30;

    int ex = CV_FOURCC('D', 'I', 'V', 'X');     //Codec Type- Int form
    int frame_counter = 0;
    int frame_count_max = -1; //infinite
    int color=0; //0-default, 1-red, 2-green, 3-blue, 4-yellow


    std::string pkgpath = "/home/odroid/ros_ws/src/ch-1-3/cvision";
    std::string srcpath = "/home/odroid/ros_ws/src/ch-1-3/cvision/src";

    //ROS Init
    ros::init(argc, argv, "box_detector");

    //Node handle
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);


    std::string ns = ros::this_node::getNamespace();

     /* get src path as a ros parameter. Should be loaded by cvision/configs/configs.yaml*/
    if (n.getParam(ns + "/pkg_path", pkgpath))
    {
      ROS_INFO("Got pkg path: %s", pkgpath.c_str());
      srcpath = pkgpath + "/src";
    }
    else
    {
      ROS_INFO("Failed to get param 'pkgpath'. Default to hardcoded paths.");
    }

    /* get the topic name of image feed */
    std::string img_tp;
    if (n.getParam(ns + "/image_feed", img_tp))
    {
      ROS_INFO("Got param: %s", img_tp.c_str());
    }
    else
    {
      ROS_INFO("Failed to get param 'image_feed'. Default to 'Quad1/cvision/frame' ");
      img_tp = "/Quad1/cvision/frame";
    }

    cvision::ObjectPose msg;

    ros::Publisher object_pub = n.advertise<cvision::ObjectPose>("boxPose",1000);

    image_transport::Publisher img_pub = it.advertise("boxImg", 10);
    //ros::Publisher img_pub = n.advertise<sensor_msgs::Image>("boxImg",10);

    bool bStream = false;
    int stream_rate = 1;
    // get gray image stream rate

    if (n.getParam(ns + "/bStream", bStream) && n.getParam(ns + "/stream_rate", stream_rate) )
    {
      ROS_INFO("Got streaming info.");
    }
    else
    {
      ROS_INFO("Failed to get streaming info.");
    }

    /* Subscribe to image ROS topic*/
    ros::Subscriber image_sub = n.subscribe(img_tp,1,imageCallback);

    string configFile = srcpath + "/box_config.txt";
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
        else if (name == "bFitBoundingBox") iss >> bFitBoundingBox;
        else if (name == "bFitRotatedRect") iss >> bFitRotatedRect;
        else if (name == "bFitCircle") iss >> bFitCircle;
        else if (name == "min_obj_sz") iss >> min_obj_sz;
        else if (name == "thres_tol") iss >> thres_tol;
        else if (name == "morph_sz") iss >> morph_sz;
        else if (name == "frameRate") iss >> frameRate;
    }

    ros::Rate loop_rate(frameRate);

    RNG rng(12345);

    Size imgSz;
    VideoCapture cap;
    VideoWriter outputVideo;

    if (bCamera)
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
    else if (bVideo)
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

        frameRate = cap.get(CV_CAP_PROP_FPS);
    }

    if (bOutputVideo && !bCompetition)
    {
        const string NAME = srcpath + "/ProcessedVideo.avi";   // Form the new name with container

        // Open the output
        outputVideo.open(NAME, ex, frameRate, imgSz, true);

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


    /* get the topic name of image feed */
    std::vector<int> threshBox_low(9);
    std::vector<int> threshBox_high(9);
    if (n.getParam(ns+"Box/low", threshBox_low) && n.getParam(ns+"Box/high", threshBox_high))
    {
      ROS_INFO("Got Box Thresholds.");
	iLowBlue = threshBox_low[0];
	iLowGreen = threshBox_low[1];
	iLowRed = threshBox_low[2];
	iLowL = threshBox_low[3];
	iLowA = threshBox_low[4];
	iLowB = threshBox_low[5];
	iLowHue = threshBox_low[6];
	iLowLum = threshBox_low[7];
	iLowSat = threshBox_low[8];	    
	iHighBlue = threshBox_high[0];
	iHighGreen = threshBox_high[1];
	iHighRed = threshBox_high[2];
	iHighL = threshBox_high[3];
	iHighA = threshBox_high[4];
	iHighB = threshBox_high[5];
	iHighHue = threshBox_high[6];
	iHighLum = threshBox_high[7];
	iHighSat = threshBox_high[8];
    }
    else
    {
      ROS_INFO("Failed to get thresholds.");
    }

    cout << "Ready to loop..." << endl;
    Mat imgBGR;
    Mat imgHSV;
    Mat imgGray;
    Mat imgThresholded;
    Mat imgContours;

    while (frame_counter != frame_count_max && !bESC  && ros::ok())
    {
        //cout << "Frame: " << frame_counter << endl;


            if ( (bCamera || bVideo))
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
            cvtColor(imgBGR, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

            ///////Thresholding
            inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

            //Width and height for morph
            int morph_width = morph_sz;
            int morph_height = morph_sz;

            //morphological opening (removes small objects from the foreground)
            erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(morph_width, morph_height)));
            dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(morph_width, morph_height)));

            //morphological closing (removes small holes from the foreground)
            dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(morph_width, morph_height)));
            erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(morph_width, morph_height)));

            imgContours = imgThresholded;
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
                vector<int> minRectArea(cont_sz);
                vector<int> minCircleArea(cont_sz);
                int max_idx_c= 0;
                int max_idx_r= 0;

                //center of frame
                Point2f frameCenter(icols_imgBGR / 2, irows_imgBGR / 2);

                /// Approximate contours to polygons + get bounding rects and circles
                for (int i = 0; i < cont_sz; i++)
                {
                    // Get the moments
                    mu[i] = moments(contours[i], false);

                    if (mu[i].m00>min_obj_sz)
                    {
                        // Get the mass centers:
                        mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
                        mc_dist[i] = frameCenter - mc[i];

                        approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
                        if (bFitBoundingBox || bDebug)
                        {
                            boundRect[i] = boundingRect( Mat(contours_poly[i]) );
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


                if (bFitCircle || bDebug)
                {
                    x_SP = cc[max_idx_c].x;
                    y_SP = cc[max_idx_c].y;
                    r_SP = cr[max_idx_c];
                }
                if (bFitRotatedRect || bDebug)
                {
                    x_SP = mc[max_idx_r].x;
                    y_SP = mc[max_idx_r].y;
                    t_SP = minRect[max_idx_r].angle;
                }

                cout << "x: " << x_SP << " y: " << y_SP << " r: " << r_SP << " t: " << t_SP << endl;

                //Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
                Scalar color = Scalar(0, 255, 255);
                Scalar color1 = Scalar(255, 0, 0);
                Scalar color2 = Scalar(0, 255, 0);
                Scalar color3 = Scalar(0, 0, 255);


                //if ( (bViz && !bCompetition) || (bStream && (((frame_counter*stream_rate)%frameRate)<stream_rate) ) )  //replace condition with stream param, then publish imgBGR
                if ( (bStream && (((frame_counter*stream_rate)%frameRate)<stream_rate) ) )
                {
//                    if (bFitBoundingBox)
//                    {
//                        //bounding box
//                        rectangle(imgBGR, boundRect[i].tl(), boundRect[i].br(), color1, 2, 8, 0 );
//                    }
                    if (bFitRotatedRect)
                    {
                        //rotated rectangle
                        Point2f rect_points[4];
                        minRect[max_idx_r].points( rect_points );
                        for( int j = 0; j < 4; j++ )
                            line(imgBGR, rect_points[j], rect_points[(j+1)%4], color2, 1, 8 );
                        //Center
                        circle(imgBGR, mc[max_idx_r], 5, color, -1, 8, 0);
                    }
                    if (bFitCircle)
                    {
                        //min circle
                        circle(imgBGR, cc[max_idx_c], (int)cr[max_idx_c], color3, 2, 8, 0 );
                        //Center
                        circle(imgBGR, cc[max_idx_c], 5, color, -1, 8, 0);
                    }
                    //putText(imgBGR, "Object Detected", mc[i] + Point2f(50, 50), 1, 2, Scalar(150, 0, 0), 2);

                    //Publish gray image to ROS
                    cvtColor(imgBGR, imgGray, CV_BGR2GRAY);
                    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", imgGray).toImageMsg();
                    img_pub.publish(img_msg);

                }

                if (bDebug && !bCompetition)
                {
                    for (int i = 0; i< contours.size(); i++)
                    {
                        if (mu[i].m00 > min_obj_sz) //Minimum size for object, otherwise it is considered noise
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


if (!bCompetition) {

            if (bDebug == true)
            {
                namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
                imshow( "Contours", drawing );
                drawing = Mat::zeros(imgSz, CV_8UC3 );

                imshow("Thresholded Image", imgThresholded); //show the thresholded image
            }
            else
            {
                //if not in debug mode, destroy the window
                cv::destroyWindow("Thresholded Image");
                cv::destroyWindow("Contours");
            }

            if (bViz == true)
            {
                /// Show in a window
                namedWindow( "VideoFeed", CV_WINDOW_AUTOSIZE );
                imshow("VideoFeed", imgBGR); //show the original image
            }
            else
            {
                //if not in viz mode, destroy the windows
                cv::destroyWindow("VideoFeed");
            }

            if (bCtrl == true)
            {
                namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

                //Create trackbars in "Control" window
                createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
                createTrackbar("HighH", "Control", &iHighH, 179);

                createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
                createTrackbar("HighS", "Control", &iHighS, 255);

                createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
                createTrackbar("HighV", "Control", &iHighV, 255);

                createTrackbar("Tolerance", "Control", &thres_tol, 100);

            }
            else
            {
                //if not in ctrl mode, destroy the window
                cv::destroyWindow("Control");
            }

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

            if (cv_img_ptr_ros){
            //Clear pointer
            cv_img_ptr_ros.reset();
            }

            ros::spinOnce();
            loop_rate.sleep();
    }

    return 0;
}
