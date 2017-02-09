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

//int H=point->x; //hue
//int S=point->y; //saturation
//int V=point->z; //value
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
    bool bSend = false;
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

    string srcpath = "/home/odroid/ros_ws/src/ch-1-3/cvision/src";
    //string srcpath = "/home/risc/ros_ws/src/ch-1-3/cvision/src";

    //ROS Init
    ros::init(argc, argv, "detector");

    //Node handle
    ros::NodeHandle n;

    cvision::ObjectPose msg;

    ros::Publisher object_pub = n.advertise<cvision::ObjectPose>("blueObj",1000);

    ros::Subscriber image_sub = n.subscribe("/cv_camera/image_raw",10,imageCallback);

    //ImageConverter imgC;

    ros::Rate loop_rate(frameRate);

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
        else if (name == "frameRate") iss >> frameRate;
        else if (name == "srcpath") iss >> srcpath;
    }

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

    cout << "Ready to loop..." << endl;
    while (frame_counter != frame_count_max && !bESC  && ros::ok())
    {
        //cout << "Frame: " << frame_counter << endl;
        Mat imgBGR;
        Mat imgHSV;
        Mat imgThresholded;
        Mat imgContours;


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


            frame_counter++;

            //Determine size of video input
            int irows_imgBGR = imgBGR.rows;
            int icols_imgBGR = imgBGR.cols;

            imgSz = Size(icols_imgBGR,irows_imgBGR);
            cvtColor(imgBGR, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

            ///////Thresholding
            //inRange(imgBGR, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
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
                            if (minRectArea[max_idx_c]<minRectArea[i])
                            {
                                max_idx_c = i;
                            }
                        }
                        if (bFitCircle || bDebug)
                        {
                            minEnclosingCircle( (Mat)contours_poly[i], cc[i], cr[i] );
                        }
                        if (minRectArea[max_idx_r]<minRectArea[i])
                        {
                            max_idx_r = i;
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

                //cout << "x: " << x_SP << " y: " << y_SP << " r: " << r_SP << " t: " << t_SP << endl;

                //Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
                Scalar color = Scalar(0, 255, 255);
                Scalar color1 = Scalar(255, 0, 0);
                Scalar color2 = Scalar(0, 255, 0);
                Scalar color3 = Scalar(0, 0, 255);

                if (bViz && !bCompetition)
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
                //cout << "H:" << H << " S:" << S << " V:" << V << endl;
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
                    
            case 115: //'s' has been pressed.
                bSend = 1;
                cout << "Sending triggered." << endl;
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
                if (bSend) {
                //Send ROS message here
                bSend = false;
                cout << "Sending red thresholds complete." << endl;
                }
                break;

            case 2: //Green color selected.
                if (bSend) {
                //Send ROS message here
                bSend = false;
                cout << "Sending green thresholds complete." << endl;
                }
                break;

            case 3: //Blue color selected.
                if (bSend) {
                //Send ROS message here
                bSend = false;
                cout << "Sending blue thresholds complete." << endl;
                }
                break;

            case 4: //Yellow color selected.
                if (bSend) {
                //Send ROS message here
                bSend = false;
                cout << "Sending yellow thresholds complete." << endl;
                }
                break;
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
    }

    ofstream myfile(newThres.c_str());
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
        cout << "Finished writing" << endl;
    }
    else cout << "Unable to open file";
    return 0;
}
