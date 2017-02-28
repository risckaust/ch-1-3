#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "autopilots/StateMachine.h"
#include "serial/serial.h"

#include <string>

/* Header bytes for feedback/command messages */
#define MSG_HEADER 7
#define MSG_END 9
#define MSG_LENGTH 28 // bytes
#define MAX_BUFFER_SIZE 256

using namespace std;

/* Listens to user command on '/gripper_command' topic. Once available, sends it to gripper's microcontroller */
class MyCallbacks
{
public:
     MyCallbacks() { }
    ~MyCallbacks() { }

    autopilots::StateMachine _sm_msg;
    sensor_msgs::NavSatFix _gps_msg;

    // callback
    //gps
    void gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {
        _gps_msg.latitude = msg.latitude;
        _gps_msg.longitude = msg.longitude;
    }
    //state
    void sm_cb(const autopilots::StateMachine::ConstPtr& msg)
    {
        _sm_msg.state = msg.state;
    }

};
/*--------------------------------------------------------*/

/** MAIN:
* This node interacts with the magnetic gripper controller, Aurduino.
* 1) opens a serial port to talk to aurduino
* 2) listens to gripper status, and updates/publishes the corresponding topic
* 3) listens to the commands topics (by user),  and sends commands to aurduino
*/
int main(int argc, char **argv)
{

    /**
      make sure to get port/baud from command line arguments
    */
    if (argc != 3)
    {
        ROS_ERROR("Please provide port and baud. Exiting");
                return 1;
    }
    // port and baud
    string port = argv[1];
    int baud = atoi(argv[2]);

    /**
    * You must call one of the versions of ros::init() before using any other
    * part of the ROS system.
    */
    ros::init(argc, argv, "telem_serial");

    /**
    * NodeHandle is the main access point to communications with the ROS system.
    * The first NodeHandle constructed will fully initialize this node, and the last
    * NodeHandle destructed will close down the node.
    */
    ros::NodeHandle n("~");
    //Get the namespace
    std::string ns = ros::this_node::getNamespace();


    ros::Rate loop_rate(20);	/* Hz */

    // my quad number
    uint8_t quadN=1;

    /*------------ Serial setup---------------------- */
    uint8_t input_buffer[MAX_BUFFER_SIZE];		/* read buffer */
    uint8_t output_buffer[MSG_LENGTH];		/* write buffer */
    size_t bytes_read;		/* number of bytes read from serail port returned by read() method*/
    size_t bytes_available;		/* bytes available, returned by available() method*/
    /* create serial handle */
    serial::Serial serial_handle(port, (unsigned long)baud, serial::Timeout::simpleTimeout(1000));
    ROS_INFO("Serial port is opened");

    // other quads numbers
    uint8_t quadA_N = 2;
    uint8_t quadB_N = 3;
    // other quads name spaces
    string quadA_ns = "/Quad2";
    string quadB_ns = "/Quad3";

    // resolve other quads numbers
    if (quadN == 1)
    {
        quadA_N = 2;
        quadB_N = 3;
        quadA_ns = "/Quad2";
        quadB_ns = "/Quad3";
    }
    else if(quadN == 2){
        quadA_N = 1;
        quadB_N = 3;
        quadA_ns = "/Quad1";
        quadB_ns = "/Quad3";

    }
    else if(quadN == 3){
        quadA_N = 1;
        quadB_N = 2;
        quadA_ns = "/Quad1";
        quadB_ns = "/Quad2";

    }
    else{
        ROS_ERROR("Quad ID is not 1, 2, or 3");
        return 1;
    }

    // create other quads msg to be published
    sensor_msgs::NavSatFix quadA_gps_msg;
    autopilots::StateMachine quadA_sm_msg;

    sensor_msgs::NavSatFix quadB_gps_msg;
    autopilots::StateMachine quadB_sm_msg;

    /* other quads msg publisher */
    ros::Publisher quadA_gps_pub = n.advertise<sensor_msgs::NavSatFix>(quadA_ns+"/mavros/global_position/global", 10);
    ros::Publisher quadB_gps_pub = n.advertise<sensor_msgs::NavSatFix>(quadB_ns+"/mavros/global_position/global", 10);

    ros::Publisher quadA_sm_pub = n.advertise<autopilots::StateMachine>(quadA_ns+"/state_machine/state", 10);
    ros::Publisher quadB_sm_pub = n.advertise<autopilots::StateMachine>(quadB_ns+"/state_machine/state", 10);

    /* Subscribers to this quads gps and state*/
    MyCallbacks my_cb;			/* holds the callbacks for this quad's gps and state*/
    ros::Subscriber my_gps_sub = n.subscribe(ns+"/mavros/global_position/global", 10, &MyCallbacks::gps_cb, &my_cb);
    ros::Subscriber my_sm_sub = n.subscribe(ns+"/state_machine/state", 10, &MyCallbacks::sm_cb, &my_cb);

    /* counter */
    // transmission caounter
    uint32_t tx_counter = 0;
    // reception counters
    uint32_t qA_gps_c = 0;
    uint32_t qA_sm_c = 0;
    uint32_t qB_gps_c = 0;
    uint32_t qB_sm_c = 0;

    /* flags to update the msgs, to keep the same rate they were published at */
    bool update_gps_a = false;
    bool update_sm_a = false;
    bool update_gps_b = false;
    bool update_sm_b = false;

    /* variable to hold received quad number*/
    uint8_t qn = 0;


    /* Main loop */
    while (ros::ok())
    {

        if (serial_handle.isOpen())
        {

            /* Reset flags */
            update_gps_a = false;
            update_gps_b = false;
            update_sm_a = false;
            update_sm_b = false;


            /* -----------------send my gps and state------------------- */
            // make sure we have new data to send
            uint32_t seq1 = my_cb._gps_msg.header.seq;
            uint32_t seq2 = my_cb._sm_msg.header.seq;
            if (seq1 > tx_counter || seq2 > tx_counter)
            {
                tx_counter = (seq1<=seq2)?seq2:seq1;
                //update output_buffer
                output_buffer[0]= (uint8_t)MSG_HEADER; // header
                output_buffer[1] = (uint8_t)quadN; // my quad number
                output_buffer[MSG_LENGTH-1] = (uint8_t)MSG_END; // end char

                // prepare gps sequence
                uint8_t intBytes[4];
                memcpy( intBytes , &seq1 , sizeof( uint32_t ) );
                for (int k=0; k<4; k++)
                    output_buffer[2+k]=intBytes[k];

                //prepare lat/lon
                uint8_t dBytes[8];
                memcpy( dBytes , &(my_cb._gps_msg.latitude) , sizeof( double ) );
                for (int k=0; k<8; k++)
                    output_buffer[6+k]=intBytes[k];

                // lon
                memcpy( dBytes , &(my_cb._gps_msg.longitude) , sizeof( double ) );
                for (int k=0; k<8; k++)
                    output_buffer[14+k]=intBytes[k];

                // state sequence
                memcpy( intBytes , &seq2 , sizeof( uint32_t ) );
                for (int k=0; k<4; k++)
                    output_buffer[22+k]=intBytes[k];

                // state
                string mystate = my_cb._sm_msg.state;
                uint8_t mysID=9;
                switch (mystate)
                {
                case "Start":
                    mysID = 0;
                    break;
                case "Takeoff":
                    mysID = 1;
                    break;
                case "ObjectSearch":
                    mysID = 2;
                    break;
                case "Picking":
                    mysID = 3;
                    break;
                case "GoToDrop":
                    mysID = 4;
                    break;
                case "WaitToDrop":
                    mysID = 5;
                    break;
                case "Drop":
                    mysID = 6;
                    break;
                case "Hover":
                    mysID = 7;
                    break;
                default:
                    mysID = 9;
                }

                output_buffer[26] = mysID;

                // send buffer
                serial_handle.write(output_buffer, MSG_LENGTH);
            }


            /* ----------Receive dats---------- */
            // Reset input_buffer
            memset(input_buffer, 0, sizeof(input_buffer));

            bytes_read=0;
            /* read serial buffer, if data is available */
            bytes_available = serial_handle.available();
            ROS_INFO("D: %d\n", bytes_available);
            if ( bytes_available > 0)
            {
                bytes_read = serial_handle.read(input_buffer, bytes_available);
            }

            /* parse data */
            if (bytes_read > 0 )
            {
                /* loop over the whole buffer */
                for (int i=0; i < bytes_read; i++)
                {
                    /* 20 bytes/msg are expected which represent: */
                    /* header+quadN+seq+Lat+Lon+seq+stateId+end = 1+1+4+8+84++1+1 = 28*/
                    if ((uint8_t)buffer[i] == MSG_HEADER && (i+MSG_LENGTH-1) < bytes_read) /* make sure we always can read 20 bytes. Otherwise, wrong msg */
                    {
                        // make sure that we can read the end character
                        if ( (uint8_t)input_buffer[i+MSG_LENGTH-1] == MSG_END ){
                            /* ----- parse msg------ */
                            // get quad number
                            qn = (uint8_t)input_buffer[i+1];

                            // get gps sequence number
                            uint8_t intbytes[4];
                            for (int k=0; k<4; k++)
                                intbytes[k] = input_buffer[i+2+k];
                            uint32_t gps_seq;
                            memcpy(&gps_seq, intbytes, sizeof(uint32_t));

                            // get state sequence
                            for (int k=0; k<4; k++)
                                intbytes[k] = input_buffer[i+22+k];
                            uint32_t sm_seq;
                            memcpy(&sm_seq, intbytes, sizeof(uint32_t));

                            // get lat
                            uint8_t double_bytes[8];
                            for (int k =0; k<8; k++)
                                double_bytes[k]=input_buffer[i+6+k];
                            double lat;
                            memcpy(&lat, double_bytes, sizeof(double));
                            // get lon
                            for (int k =0; k<8; k++)
                                double_bytes[k]=input_buffer[i+14+k];
                            double lon;
                            memcpy(&lon, double_bytes, sizeof(double));

                            // parse the state
                            uint8_t sID = input_buffer[i+26];
                            string state = "Unknown";
                            switch (sID)
                            {
                            case 0:
                                state = "Start";
                                break;
                            case 1:
                                state = "Takeoff";
                                break;
                            case 2:
                                state = "ObjectSearch";
                                break;
                            case 3:
                                state = "Picking";
                                break;
                            case 4:
                                state = "GoToDrop";
                                break;
                            case 5:
                                state = "WaitToDrop";
                                break;
                            case 6:
                                state = "Drop";
                                break;
                            case 7:
                                state = "Hover";
                                break;
                            default:
                                state = "Unknown";
                                break;
                            }

                            // update other quads messages (to be published)
                            if (qn == quadA_N)
                            {
                                // check if we should update msgs based on sequence numbers
                                if (gps_seq > qA_gps_c)
                                {
                                    qA_gps_c = gps_seq;
                                    update_gps_a = true;
                                    // update gps msg
                                    quadA_gps_msg.header.stamp = ros::Time::now();
                                    quadA_gps_msg.latitude = lat;
                                    quadA_gps_msg.longitude = lon;
                                }
                                if (sm_seq > qA_sm_c)
                                {
                                    qA_sm_c = sm_seq;
                                    update_sm_a = true;
                                    // update state
                                    quadA_sm_msg.header.stamp = ros::Time::now();
                                    quadA_sm_msg.state = state;
                                }
                            }
                            else if (qn == quadB_N)
                            {
                                // check if we should update msgs based on sequence numbers
                                if (gps_seq > qB_gps_c)
                                {
                                    qB_gps_c = gps_seq;
                                    update_gps_b = true;
                                    // update gps msg
                                    quadB_gps_msg.header.stamp = ros::Time::now();
                                    quadB_gps_msg.latitude = lat;
                                    quadB_gps_msg.longitude = lon;
                                }
                                if (sm_seq > qB_sm_c)
                                {
                                    qB_sm_c = sm_seq;
                                    update_sm_b = true;
                                    // update state
                                    quadB_sm_msg.header.stamp = ros::Time::now();
                                    quadB_sm_msg.state = state;
                                }
                            }
                            else
                            {
                                // error
                                ROS_ERROR("Received Quad: %d,  either not supported or being used.", qn);
                            }

                        } // end of msg end checking
                    } // end of msg header checking

                    // update i, to go to next possible message
                    i = i + MSG_LENGTH -1;
                } /* finished reading the buffer */
            } /* finished parsing data */
        }// serial isOpen -> checked
        else
        {
            ROS_ERROR("Serial port is not open. Shutting down the node.");
            break;
        }


        /* -----------------Publish msgs---------------*/

        //QuadA
        if(update_gps_a)
        {
            update_gps_a = false;
            quadA_gps_pub.publish(quadA_gps_msg);
        }
        if(update_sm_a)
        {
            update_sm_a = false;
            quadA_sm_pub.publish(quadA_sm_msg);
        }
        //QuadB
        if(update_gps_b)
        {
            update_gps_b = false;
            quadB_gps_pub.publish(quadB_gps_msg);
        }
        if(update_sm_b)
        {
            update_sm_b = false;
            quadB_sm_pub.publish(quadB_sm_msg);
        }



        ros::spinOnce();

        loop_rate.sleep();
    } // END of ROS loop

    /* close port on exit */
    ROS_INFO("Closing serial port\n");
    serial_handle.close();


    return 0;
}
