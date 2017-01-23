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
    Mat img;
    Point3_<uchar> pt;
    bool bMouseClicked;
};

void mouseHandler( int event, int x, int y, int flags, void* param)
{

    if( event != CV_EVENT_LBUTTONDOWN )
        return;

// Mount back the parameters
    MouseParams* mp = (MouseParams*)param;
    Mat img = mp->img;

    Point3_<uchar>* point = img.ptr<Point3_<uchar> >(y,x);
//Point3_<uchar>* point = img->ptr<Point3_<uchar> >(y,x);
    mp->pt = * point;
    mp->bMouseClicked = true;

//int H=p->x; //hue
//int S=p->y; //saturation
//int V=p->z; //value
//cout << "H:" << H << " S:" << S << " V:" << V << endl;

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
    int frameRate =30;
    //ROS Init
    ros::init(argc, argv, "detector");

    //Node handle
    ros::NodeHandle n;


    cvision::ObjectPose msg;

    ros::Publisher object_pub = n.advertise<cvision::ObjectPose>("blueObj",1000);

    ros::Subscriber image_sub = n.subscribe("/cv_camera/image_raw",10,imageCallback);

    //ImageConverter imgC;

    ros::Rate loop_rate(frameRate);


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
    //Fit shapes
    bool bFitBoundingBox = true;
    bool bFitRotatedRect = true;
    bool bFitCircle = true;
    //Set min object size
    int min_obj_sz = 5;
    int thres_tol = 50;
    int morph_sz = 5;

    int ex = CV_FOURCC('D', 'I', 'V', 'X');     //Codec Type- Int form
    double frame_counter = 0;
    double frame_count_max = -1; //infinite

    string srcpath = "/home/odroid/ros_ws/src/ch-1-3/cvision/src";

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
        else if (name == "bFitBoundingBox") iss >> bFitBoundingBox;
        else if (name == "bFitRotatedRect") iss >> bFitRotatedRect;
        else if (name == "bFitCircle") iss >> bFitCircle;
        else if (name == "min_obj_sz") iss >> min_obj_sz;
        else if (name == "thres_tol") iss >> thres_tol;
        else if (name == "morph_sz") iss >> morph_sz;
    }

    RNG rng(12345);

    string newThres = srcpath + "/ThresholdValuesNew.txt";
    ofstream myfile(newThres.c_str());

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
    int iLowH = 0;
    int iHighH = 179;

    int iLowS = 0;
    int iHighS = 255;

    int iLowV = 0;
    int iHighV = 255;

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

        if (name1 == "iLowH") iss >> iLowH;
        else if (name1 == "iHighH") iss >> iHighH;
        else if (name1 == "iLowS") iss >> iLowS;
        else if (name1 == "iHighS") iss >> iHighS;
        else if (name1 == "iLowV") iss >> iLowV;
        else if (name1 == "iHighV") iss >> iHighV;
    }

    int iLastX = -1;
    int iLastY = -1;

    //Capture a temporary image from the camera
    //Mat imgTmp;
    //cap.read(imgTmp)
    //S=imgTmp.size();

    while (frame_counter != frame_count_max && !bESC  && ros::ok())
    {
        Mat imgOriginal;
        Mat imgHSV;
        Mat imgThresholded;
        Mat imgContours;

        if (cv_img_ptr_ros)
        {

            if ( (bCamera || bVideo) && !bCompetition)
            {
                bool bSuccess = cap.read(imgOriginal); // read a new frame from video

                if (!bSuccess) //if not success, break loop
                {
                    cout << "Cannot read a frame from video stream" << endl;
                    break;
                }

            }

            else
            {
                imgOriginal = cv_img_ptr_ros->image;
            }

            frame_counter++;

            //Determine size of video input
            int irows_imgOriginal = imgOriginal.rows;
            int icols_imgOriginal = imgOriginal.cols;

            imgSz = Size(icols_imgOriginal,irows_imgOriginal);

            //Create a black image with the size as the camera output
            Mat drawing = Mat::zeros(imgSz, CV_8UC3 );

            cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV


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
            vector<vector<Point> > contours;
            vector<Vec4i> hierarchy;
            findContours(imgContours, contours, hierarchy,
                         CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

            vector<Moments> mu(contours.size());
            vector<vector<Point> > contours_poly( contours.size() );
            vector<Rect> boundRect( contours.size() );
            vector<RotatedRect> minRect( contours.size() );
            vector<Point2f> mc(contours.size());
            vector<Point2f> mc_dist(contours.size());
            vector<Point2f> cc(contours.size());
            vector<float> cr(contours.size());

            /// Get the moments

            /// Approximate contours to polygons + get bounding rects and circles
            for (int i = 0; i < contours.size(); i++)
            {
                //mu contains moments
                mu[i] = moments(contours[i], false);

                approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
                if (bFitBoundingBox || bDebug) {
                boundRect[i] = boundingRect( Mat(contours_poly[i]) );
                }
                if (bFitRotatedRect || bDebug) {
                minRect[i] = minAreaRect( Mat(contours_poly[i]) );
                }
                if (bFitCircle || bDebug) {
                minEnclosingCircle( (Mat)contours_poly[i], cc[i], cr[i] );
                }

                //cout << "Bounding Box: " << boundRect[i] << endl;
                //cout << "Smallest Rect: " << minRect[i] << endl;
                //cout << "Smallest Circle: " << cc[i] << ", " << cr[i] << endl;

            }

            ///  Get the mass centers:
            //center of frame
            Point2f frameCenter(icols_imgOriginal / 2, irows_imgOriginal / 2);

            for (int i = 0; i < contours.size(); i++)
            {
                //mc contains centers (posX, posY)
                mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
                mc_dist[i] = frameCenter - mc[i];
            }

            //Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            Scalar color = Scalar(0, 255, 255);
            Scalar color1 = Scalar(255, 0, 0);
            Scalar color2 = Scalar(0, 255, 0);
            Scalar color3 = Scalar(0, 0, 255);

            for (int i = 0; i< contours.size(); i++)
            {
                if (mu[i].m00 > min_obj_sz) //Minimum size for object, otherwise it is considered noise
                {

                    if (bViz)
                    {
                        if (bFitBoundingBox) {
                        //bounding box
                        rectangle(imgOriginal, boundRect[i].tl(), boundRect[i].br(), color1, 2, 8, 0 );
                        }
                        if (bFitRotatedRect) {
                        //rotated rectangle
                        Point2f rect_points[4];
                        minRect[i].points( rect_points );
                        for( int j = 0; j < 4; j++ )
                            line(imgOriginal, rect_points[j], rect_points[(j+1)%4], color2, 1, 8 );
                        }
                        if (bFitCircle) {
                        //min circle
                        circle(imgOriginal, cc[i], (int)cr[i], color3, 2, 8, 0 );
                        }
                        //Center
                        circle(imgOriginal, mc[i], 5, color, -1, 8, 0);
                        //putText(imgOriginal, "Object Detected", mc[i] + Point2f(50, 50), 1, 2, Scalar(150, 0, 0), 2);
                    }
                    if (bDebug)
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
                imshow("VideoFeed", imgOriginal); //show the original image
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

                mp.img = imgHSV;
                cv::setMouseCallback("VideoFeed", mouseHandler, (void*)&mp);

            }
            else
            {
                //if not in ctrl mode, destroy the window
                cv::destroyWindow("Control");
            }

            if (bOutputVideo)
            {
                outputVideo << imgOriginal;
            }

            //Adjust thresholds with values selected by mouse
            if(mp.bMouseClicked)
            {
                mp.bMouseClicked = false;
                int H=mp.pt.x; //hue
                int S=mp.pt.y; //saturation
                int V=mp.pt.z; //value
                iLowH = max(H-thres_tol,0);
                iHighH = min(H+thres_tol,179);
                iLowS = max(S-thres_tol,0);
                iHighS = min(S+thres_tol,255);
                iLowV = max(V-thres_tol,0);
                iHighV = min(V+thres_tol,255);
                cout << "H:" << H << " S:" << S << " V:" << V << endl;
            }

            //Check for key presses
            switch (waitKey(30))
            {

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
                        //stay in this loop until
                        switch (waitKey())
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

            //ROS Topics
            //pose (setpoint - 2D float [m], heading - float [deg])
            //valid - bool
            //radius - float [m]
            msg.pose.x = mc[0].x;
            msg.pose.y = mc[0].y;
            msg.pose.theta = 0;
            msg.radius = cr[0];
            msg.valid = true;

            //ROS Publisher
            object_pub.publish(msg);

            //Clear pointer
            cv_img_ptr_ros.reset();

        }
        else
        {
            // nothing can be done here; we have to spin and wait for images to arrive
        }

        ros::spinOnce();
    }

    //Save values to file
    if (myfile.is_open())
    {
        myfile << "iLowH = " << iLowH << "\n";
        myfile << "iHighH = " << iHighH << "\n";
        myfile << "iLowS = " << iLowS << "\n";
        myfile << "iHighS = " << iHighS << "\n";
        myfile << "iLowV = " << iLowV << "\n";
        myfile << "iHighV = " << iHighV << "\n";
        myfile.close();
    }
    else cout << "Unable to open file";
    cout << "Finished writing" << endl;
    return 0;
}
