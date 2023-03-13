//*****************************************************************************
//**********************  WORKSPACE_LIMITS_AVOIDANCE.CPP **********************
//**********************      Author: Livio Bisogni      **********************
//*****************************************************************************

/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
                                  INSTRUCTIONS

    Please read the attached `README.md` file.
_____________________________________________________________________________*/


/*-----------------------------------------------------------------------------
-------------------------------------------------------------------------------
--------------------------------- HEADER FILES --------------------------------
-------------------------------------------------------------------------------
-----------------------------------------------------------------------------*/

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include <termios.h>


/*-----------------------------------------------------------------------------
-------------------------------------------------------------------------------
------------------------------- GLOBAL CONSTANTS ------------------------------
-------------------------------------------------------------------------------
-----------------------------------------------------------------------------*/

const double FORWARD_SPEED      = 1;  // Forward speed of the turtle
const double PI                 = 3.14159265359;
const double WALL_CLEARANCE     = 0.3;   // Minimum distance to any wall
const double REVERSING_DISTANCE = 0.35;  // distance covered in reversed by the
                                         // turtle when a wall is detected
const int KEY_ESC = 27;                  // ASCII code equivalence
// Walls coordinates:
const double LEFT_WALL   = 0.0;   // left wall
const double BOTTOM_WALL = 0.0;   // bottom wall
const double RIGHT_WALL  = 11.0;  // right wall
const double TOP_WALL    = 11.0;  // top wall

const int ANG_MIN = 30;   // abs. val. of the min possible random angle [deg]
const int ANG_MAX = 150;  // abs. val. of the max possible random angle [deg]
const int ANGULAR_SPEED = 160;  // angular speed [deg]


/*-----------------------------------------------------------------------------
-------------------------------------------------------------------------------
------------------------------- GLOBAL VARIABLES ------------------------------
-------------------------------------------------------------------------------
-----------------------------------------------------------------------------*/

bool           stop = false;  // flag to stop the turtle when it's set to 'true'
ros::Publisher velocity_pub;  // velocity publisher


/*-----------------------------------------------------------------------------
-------------------------------------------------------------------------------
----------------------- CALLBACK & FUNCTION DEFINITIONS -----------------------
-------------------------------------------------------------------------------
-----------------------------------------------------------------------------*/

/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    DETECTWALLS_CALLBACK:       Read current pose and set the 'stop' flag
                                to 'true' when a wall is too close (i.e., within
                                WALL_CLEARANCE)
_____________________________________________________________________________*/

void detectWalls_Callback(const turtlesim::PoseConstPtr &msg)
{
    if (msg->x <= (LEFT_WALL + WALL_CLEARANCE) ||
        msg->x >= (RIGHT_WALL - WALL_CLEARANCE) ||
        msg->y <= (BOTTOM_WALL + WALL_CLEARANCE) ||
        msg->y >= (TOP_WALL - WALL_CLEARANCE)) {
        stop = true;
    }
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    GOAHEAD:        Move the turtle for a given 'distance' at a certain 'speed'
                    either forwards (if 'isForward==true') or backwards (if
                    'isForward==false')
_____________________________________________________________________________*/

void goAhead(double speed, double distance, bool isForward)
{
    // Publish to topic '/turtle1/cmd_vel' of type 'geometry_msgs/Twist'
    // Create a Twist message
    geometry_msgs::Twist vel_msg;

    // Fill in the fields of the Twist message with appropriate values
    //
    // Linear velocity
    if (isForward)
        vel_msg.linear.x = abs(speed);
    else
        vel_msg.linear.x = -abs(speed);
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    //
    // Angular velocity
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;

    // t0: initial time
    double t0 = ros::Time::now().toSec();
    // t1: final time (aka elapsed time)
    double t1 = 0;
    // Initialize current distance
    double current_distance = 0;
    // Set the number of messages (100) sent per second
    ros::Rate loop_rate(100);

    // Check for velocity as long as the distance covered matches the desired
    // one
    do {
        velocity_pub.publish(vel_msg);
        // Get elapsed time
        t1 = ros::Time::now().toSec();
        // Compute distance covered
        current_distance = speed * (t1 - t0);
        ros::spinOnce();
        loop_rate.sleep();
    } while (current_distance < distance);
    vel_msg.linear.x = 0;
    velocity_pub.publish(vel_msg);
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    ROTATE:     Rotate the turtle by a given 'angle' at a certain
                'angular_speed' either clockwise (if 'clockwise==true') or
                counterclockwise (if 'clockwise==false')
                Note that the idea is similar to the one used in the 'goAhead'
                function
_____________________________________________________________________________*/

void rotate(double angular_speed, double angle, bool clockwise)
{
    // Create a Twist message
    geometry_msgs::Twist vel_msg;
    // Fill in the fields of the Twist message with appropriate values
    //
    // Linear velocity
    vel_msg.linear.x = 0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    //
    // Angular velocity
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    if (clockwise)
        vel_msg.angular.z = -abs(angular_speed);
    else
        vel_msg.angular.z = abs(angular_speed);

    // t0: initial time
    double t0 = ros::Time::now().toSec();
    // t1: final time (aka elapsed time)
    double t1 = 0;
    // Initialize current angle
    double current_angle = 0;
    // Set the number of messages sent per second
    ros::Rate loop_rate(10);

    // Check for velocity as long as the current angle matches the desired one
    do {
        velocity_pub.publish(vel_msg);
        // Get elapsed time
        t1 = ros::Time::now().toSec();
        // Compute current angle
        current_angle = angular_speed * (t1 - t0);
        ros::spinOnce();
        loop_rate.sleep();
    } while (current_angle < angle);

    vel_msg.angular.z = 0;
    velocity_pub.publish(vel_msg);
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    DEGREES2RADIANS:        Convert angle_in_degrees from degrees to radians
_____________________________________________________________________________*/

double degrees2radians(double angle_in_degrees)
{
    return angle_in_degrees * PI / 180.0;
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    RAND_BOOL:      Generate a pseudo-random boolean variable (either 'true' or
                    'false')
_____________________________________________________________________________*/

bool rand_bool()
{
    if (rand() % 2 == 0)
        return true;
    else
        return false;
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    GETCH:      Get pressed key character; non-blocking function.
                Code adapted from:

                https://answers.ros.org/question/63491/keyboard-key-pressed/
_____________________________________________________________________________*/

char getch()
{
    fd_set         set;
    struct timeval timeout;
    int            rv;
    char           buff     = 0;
    int            len      = 1;
    int            filedesc = 0;
    FD_ZERO(&set);
    FD_SET(filedesc, &set);

    timeout.tv_sec  = 0;
    timeout.tv_usec = 1000;

    rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

    struct termios old = {0};
    if (tcgetattr(filedesc, &old) < 0)
        ROS_ERROR("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN]  = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(filedesc, TCSANOW, &old) < 0)
        ROS_ERROR("tcsetattr ICANON");

    if (rv == -1)
        ROS_ERROR("select");
    else if (rv == 0) {
        // ROS_INFO("no_key_pressed"); // DEBUG
    } else
        read(filedesc, &buff, len);

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
        ROS_ERROR("tcsetattr ~ICANON");
    return (buff);
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    RANDANGLE:      Generate a pseudo-random angle within:
                    [-ANG_MAX, -ANG_MIN] U [+ANG_MIN, +ANG_MAX]
_____________________________________________________________________________*/

int randAngle()
{
    int angle;  // Rotation angle

    // Pseudo-randomly select an angle from +ANG_MIN to +ANG_MAX degrees
    angle = (rand() % (ANG_MAX - ANG_MIN)) + (ANG_MIN + 1);

    return angle;
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    RANDDIRECTION:      Generate a pseudo-random angle direction (either
                        clockwise ('true') or counterclockwise ('false'))
_____________________________________________________________________________*/

bool randDirection(int angle)
{
    bool direction;  // angle direction

    direction = rand_bool();  // Pseudo-randomly select the direction of the
                              // angle (either clockwise or counterclockwise),
                              // so that the turtle rotates within [-ANG_MAX,
                              // -ANG_MIN] U [+ANG_MIN, +ANG_MAX]
    if (direction == true)    // clockwise
        printf("The turtle is rotating by -%i degrees\n", angle);
    else  // counterclockwise
        printf("The turtle is rotating by %i degrees\n", angle);

    return direction;
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    MAIN:       Da main
_____________________________________________________________________________*/

int main(int argc, char **argv)
{
    int  c         = 0;  // key pressed (ASCII code)
    int  angle     = 0;
    bool direction = false;

    // Seed the random number generator with the current time.
    // It should be called once (and only once) prior to calling any other
    // random number function
    srand(time(NULL));

    // Node creation
    ros::init(argc, argv, "workspace_collision_avoidance");
    ros::NodeHandle node;

    // Subscriber for current pose
    ros::Subscriber CurrPose_sub =
        node.subscribe("turtle1/pose", 10, detectWalls_Callback);

    // Publisher for velocity
    velocity_pub = node.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);

    // Twist initialization
    geometry_msgs::Twist cmdVel;
    cmdVel.linear.x = FORWARD_SPEED;

    // Node frequency
    ros::Rate rate(10);  // Frequency to run loops to (10 Hz)

    while (ros::ok() && c != KEY_ESC) {
        ros::spinOnce();
        
        if (stop) {
            printf("\nCollision avoided!\n");

            // Stop the turtle
            cmdVel.linear.x = 0;
            velocity_pub.publish(cmdVel);

            printf("The turtle is moving backward\n");
            // Move the turtle in reverse for REVERSING_DISTANCE (when it stops,
            // it is REVERSING_DISTANCE + WALL_CLEARANCE away from the
            // previously detected wall)
            goAhead(FORWARD_SPEED, REVERSING_DISTANCE, 0);

            // Pseudo-randomly rotate the turtle
            angle     = randAngle();
            direction = randDirection(angle);
            rotate(degrees2radians(ANGULAR_SPEED), degrees2radians(angle),
                   direction);

            // Reset the 'stop' flag
            stop = false;
        } else {  // (once the flag becomes 'false')
            printf("The turtle is moving forward ...\n");
            // Turtle resumes to move forward
            cmdVel.linear.x = FORWARD_SPEED;
            velocity_pub.publish(cmdVel);
        }

        c = getch();  // read character (non-blocking)

        rate.sleep();
    }

    return 0;
}
