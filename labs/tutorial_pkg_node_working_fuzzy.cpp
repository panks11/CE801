#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <fstream>
#include <time.h>
#include <iomanip>

using namespace std::chrono_literals;
using namespace std;


ofstream laserFile; // Declare a file object for recording your laser data.Laser Range data recorded
ofstream odomTrajFile; // Declare a file object for recording robot Trajectory
ofstream odomVelFile;   // Declare a file object for recording robot Velocity
ofstream laserMapFile; // Declare a file object for recording x,y locations of Map objects,boundary and objects


struct EulerAngles{double roll, pitch, yaw;}; // yaw is what you want, i.e. Th
struct Quaternion{double w, x, y, z;};

struct PID_para{double kp, ki, kd, ei_pre, ed_pre, Max_output;};

double PID_control(PID_para pid, double setPoint, double measuredData)
{
    //double err = measuredData - setPoint;
    double err = setPoint - measuredData;
    double ei = pid.ei_pre + err;
    double ed = err - pid.ed_pre;
    pid.ei_pre = ei;
    pid.ed_pre = ed;
    double output = pid.kp*err + pid.ki*ei + pid.kd*ed;
    if (output > pid.Max_output )
        {
        output = pid.Max_output;
        }
    else if(output < -pid.Max_output)
        {
            output = -pid.Max_output;
        }
    return output;
}


EulerAngles ToEulerAngles(Quaternion q)
{ 
    // for calculating Th
    EulerAngles angles;
    
    double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z); // roll (x-axis rotation)
    double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    angles.roll = atan2(sinr_cosp, cosr_cosp);
    double sinp = +2.0 * (q.w * q.y - q.z * q.x);// pitch (y-axis rotation)

    if (fabs(sinp) >= 1)
        angles.pitch = copysign(M_PI/2, sinp); //use 90 degrees if out of range
    else
        angles.pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    angles.yaw = atan2(siny_cosp, cosy_cosp);
    return angles;
}


class Stopper : public rclcpp::Node
{
    public:
        /* velocity control variables*/
        constexpr const static double FORWARD_SPEED_LOW = 0.1;
        constexpr const static double FORWARD_SPEED_MIDDLE = 0.3;
        constexpr const static double FORWARD_SPEED_HIGH = 0.5;
        constexpr const static double FORWARD_SPEED_STOP = 0;
        constexpr const static double TURN_LEFT_SPEED_LOW = 0.3;
        constexpr const static double TURN_LEFT_SPEED_MIDDLE = 0.6;
        constexpr const static double TURN_LEFT_SPEED_HIGH = 1.0;
        constexpr const static double TURN_RIGHT_SPEED_LOW = -0.3;
        constexpr const static double TURN_RIGHT_SPEED_MIDDLE = -0.6;
        constexpr const static double TURN_RIGHT_SPEED_HIGH = -1.0;
        /* class constructor */
        Stopper():Node("Stopper"), count_(0)
            {
                publisher_=this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
                odomSub_=this->create_subscription<nav_msgs::msg::Odometry>("odom",10,std::bind(&Stopper::odomCallback, this, std::placeholders::_1));
                laserScan_=this->create_subscription<sensor_msgs::msg::LaserScan>("scan",10,std::bind(&Stopper::scanCallback, this, std::placeholders::_1));
            };
            /* moving function */
        void startMoving();
        void moveStop();
        void moveForward(double forwardSpeed);
        void moveRight(double turn_right_speed);
        void moveForwardRight(double forwardSpeed, double turn_right_speed);
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odomMsg);
        void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
        void PID_wallFollowing(double forwardSpeed, double laserData);
        void PID_pass1stGap(double moveSpeed, double robotHeading);
        void PID_pass2ndGap(double moveSpeed, double robotHeading);
        void Fuzzy_wallFollowing(double laserData1, double laserData2);
        void Fuzzy_to1stGap(double laserData1, double laserData2);
        void Fuzzy_to2ndGap(double laserData1, double laserData2);
        void Fuzzy_ReachStop(double laserData1, double laserData2);

        

        double frontRange, mleftRange, leftRange, rightRange, mrightRange;
        double PositionX=0.3, PositionY=0.3, homeX=0.3, homeY=0.3;
        double odom_landmark1=1.20, odom_landmark1a=0.38, odom_landmark2=0.8;
        double odom_landmark3=1.20, odom_landmark4=1.80, odom_landmark5=2.25;
        double robVelocity;

        Quaternion robotQuat;
        EulerAngles robotAngles;
        double robotHeadAngle;
        double leftAngle = M_PI/2, mleftAngle = M_PI/4, frontAngle=0;
        double mrightAngle = -M_PI/4, rightAngle = -M_PI/2;
        void transformMapPoint(ofstream& fp, double laserRange, double laserTh, double robotTh, double robotX, double robotY);

        int laser_index = 0; // index the laser scan data
        int numberOfCycle=0;
        // int stage=0;
        
        double laser_landmark1 = 1.3, laser_landmark2 = 1.4, laser_landmark3 = 1.6, laser_landmark4 = 0.3  ;
        int stage = 1;

        constexpr const static double TURN_SPEED_ZERO = 0;
        

    private:
        // Publisher to the robot's velocity command topic
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        size_t count_;
        //Subscriber to robot’s odometry topic
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserScan_;
};

void Stopper::moveStop()
{
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = FORWARD_SPEED_STOP;
    publisher_->publish(msg);
}

void Stopper::moveForward(double forwardSpeed)
{
    //The default constructor to set all commands to 0
    auto msg=geometry_msgs::msg::Twist();
    //Drive forward at a given speed along the x-axis.
    msg.linear.x = forwardSpeed;
    publisher_->publish(msg);
}
void Stopper::moveRight(double turn_right_speed)
    {
    auto msg = geometry_msgs::msg::Twist();
    msg.angular.z = turn_right_speed;
    publisher_->publish(msg);
    }

void Stopper::moveForwardRight(double forwardSpeed, double turn_right_speed)
{
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = forwardSpeed;
    msg.angular.z = turn_right_speed;
    publisher_->publish(msg);
}

void Stopper::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odomMsg)
{
    PositionX = odomMsg->pose.pose.position.x + homeX;
    PositionY = odomMsg->pose.pose.position.y + homeY;
    RCLCPP_INFO(this->get_logger(),"RobotPostion: %.2f , %.2f",PositionX, PositionY );
    RCLCPP_INFO(this->get_logger(), "Robot stage: %d ", stage );
    // if (PositionY < odom_landmark1 && PositionX < odom_landmark1a)
    // {
    //     stage = 1;
    //     moveForward(FORWARD_SPEED_MIDDLE);
    // }
    // else if (PositionX < odom_landmark2)
    // {
    //     stage =2;
    //     moveForwardRight(FORWARD_SPEED_MIDDLE,TURN_RIGHT_SPEED_MIDDLE);
    // }
    // else if (PositionX < odom_landmark3)
    // {
    //     stage = 3;
    //     moveForward(FORWARD_SPEED_HIGH);
    // }
    // else if (PositionX < odom_landmark4)
    // {
    //     stage = 4;
    //     moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_MIDDLE);
    // }
    // else if (PositionX < odom_landmark5)
    // {
    //     stage = 5;
    //     moveForward(FORWARD_SPEED_MIDDLE);
    // }
    // else
    // {
    //     stage = 6;
    //     moveStop();
    // }
    odomTrajFile<< PositionX <<" "<< PositionY<<endl;
    robVelocity = odomMsg->twist.twist.angular.z;
    odomVelFile << numberOfCycle++ << " " << robVelocity << endl;
    robotQuat.x = odomMsg->pose.pose.orientation.x;
    robotQuat.y = odomMsg->pose.pose.orientation.y;
    robotQuat.z = odomMsg->pose.pose.orientation.z;
    robotQuat.w = odomMsg->pose.pose.orientation.w;
    robotAngles = ToEulerAngles(robotQuat);
    robotHeadAngle = robotAngles.yaw;
}

void Stopper::PID_wallFollowing(double forwardSpeed, double laserData)
{
    PID_para controller;
    double landmark1_toWall = 0.3;
    controller.kp = 0.8, controller.ki = 0.01;
    controller.kd = 0.001, controller.Max_output = 0.6;
    double PID_output = PID_control(controller, landmark1_toWall, laserData);
    cout << "Stage1:Wall Following : Right Speed is :" << PID_output << " Left Distance From Wall : "<< laserData << endl;
    moveForwardRight(forwardSpeed, PID_output);
}

void Stopper::PID_pass1stGap(double moveSpeed, double robotHeading)
{
    PID_para controller;
    double robotHeadingGap1 = -0.7; // the robot heading should be 0 degree
    controller.kp = 0.8, controller.ki = 0.06;
    controller.kd = 0.006, controller.ei_pre = 0;
    controller.ed_pre = 0, controller.Max_output = 0.6;
    double PID_output = PID_control(controller, robotHeadingGap1, robotHeading);
    cout << "Stage2:Pass Gap 1 : Right Speed is :" << PID_output << " Heading Angle is : "<< robotHeading << endl;
    moveForwardRight(moveSpeed, PID_output);
}

void Stopper::PID_pass2ndGap(double moveSpeed, double robotHeading)
{
    PID_para controller;
    double robotHeadingGap1 = -1.3; // the robot heading should be 0 degree
    controller.kp = 0.5, controller.ki = 0.06;
    controller.kd = 0.006, controller.ei_pre = 0;
    controller.ed_pre = 0, controller.Max_output = 0.6;
    double PID_output = PID_control(controller, robotHeadingGap1, robotHeading);
    cout << "Stage2:Pass Gap 2 : Right Speed is :" << PID_output << " Heading Angle is : "<< robotHeading << endl;
    moveForwardRight(moveSpeed, PID_output);
}

void Stopper::Fuzzy_wallFollowing(double laserData1, double laserData2)
{
    int fuzzySensor1, fuzzySensor2;
    // sensor data fuzzification
    if (laserData1 < 0.3) fuzzySensor1 = 1; // The robot is near to the wall
    else if (laserData1 < 0.5) fuzzySensor1 = 2; // The robot is on the right distance
    else fuzzySensor1 = 3; // The robot is far from the wall;

    if (laserData2 < 0.4) fuzzySensor2 = 1; // The robot is near to the wall
    else if (laserData2 < 0.6) fuzzySensor2 = 2; // The robot at the right distance;
    else fuzzySensor2 = 3; // The robot is far from the wall;
    // Fuzzy rule base and control output

    if (fuzzySensor1 == 1 && fuzzySensor2 == 1)
        {moveForwardRight(FORWARD_SPEED_LOW, TURN_RIGHT_SPEED_LOW);
        cout << "Stage1:Wall Follow :" << "fuzzySensor1: " <<fuzzySensor1<<" ,fuzzySensor2 :"<<fuzzySensor2<< endl;}
    else if (fuzzySensor1 == 1 && fuzzySensor2 == 2)
        {moveForwardRight(FORWARD_SPEED_LOW, TURN_RIGHT_SPEED_LOW);
        cout << "Stage1:Wall Follow :" << "fuzzySensor1: " <<fuzzySensor1<<" ,fuzzySensor2 :"<<fuzzySensor2<< endl;}
    else if (fuzzySensor1 == 1 && fuzzySensor2 == 3)
        {moveForwardRight(FORWARD_SPEED_LOW, TURN_LEFT_SPEED_LOW);
        cout << "Stage1:Wall Follow :" << "fuzzySensor1: " <<fuzzySensor1<<" ,fuzzySensor2 :"<<fuzzySensor2<< endl;}
    else if (fuzzySensor1 == 2 && fuzzySensor2 == 1)
        {moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_LOW);
        cout << "Stage1:Wall Follow :" << "fuzzySensor1: " <<fuzzySensor1<<" ,fuzzySensor2 :"<<fuzzySensor2<< endl;}
    else if (fuzzySensor1 == 2 && fuzzySensor2 == 2)
        {moveForwardRight(FORWARD_SPEED_HIGH, TURN_SPEED_ZERO);
        cout << "Stage1:Wall Follow :" << "fuzzySensor1: " <<fuzzySensor1<<" ,fuzzySensor2 :"<<fuzzySensor2<< endl;}
    else if (fuzzySensor1 == 2 && fuzzySensor2 == 3)
        {moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_LEFT_SPEED_LOW);
        cout << "Stage1:Wall Follow :" << "fuzzySensor1: " <<fuzzySensor1<<" ,fuzzySensor2 :"<<fuzzySensor2<< endl;}
    else if (fuzzySensor1 == 3 && fuzzySensor2 == 1)
        {moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_MIDDLE);
        cout << "Stage1:Wall Follow :" << "fuzzySensor1: " <<fuzzySensor1<<" ,fuzzySensor2 :"<<fuzzySensor2<< endl;}
    else if (fuzzySensor1 == 3 && fuzzySensor2 == 2)
        {moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_MIDDLE);
        cout << "Stage1:Wall Follow :" << "fuzzySensor1: " <<fuzzySensor1<<" ,fuzzySensor2 :"<<fuzzySensor2<< endl;}
    else if (fuzzySensor1 == 3 && fuzzySensor2 == 3)
        {moveForwardRight(FORWARD_SPEED_HIGH, TURN_LEFT_SPEED_LOW);
        cout << "Stage1:Wall Follow :" << "fuzzySensor1: " <<fuzzySensor1<<" ,fuzzySensor2 :"<<fuzzySensor2<< endl;}
    else RCLCPP_INFO(this->get_logger(),"Following the left wall");
    
}

void Stopper::Fuzzy_to1stGap(double laserData1, double laserData2)
{
    int fuzzySensor1, fuzzySensor2;
    // sensor data fuzzification
    if (laserData1 < 0.4) fuzzySensor1 = 1;
    else if (laserData1 < 0.6) fuzzySensor1 = 2;
    else fuzzySensor1 = 3;

    if (laserData2 < 0.4) fuzzySensor2 = 1;
    else if (laserData2 < 0.8) fuzzySensor2 = 2;
    else fuzzySensor2 = 3;

    // Fuzzy rule base and control output
    if (fuzzySensor1 == 1 && fuzzySensor2 == 1)
        {moveForwardRight(FORWARD_SPEED_LOW, TURN_RIGHT_SPEED_LOW);
        cout << "Stage2: Gap1 :" << "fuzzySensor1: " <<fuzzySensor1<<" ,fuzzySensor2 :"<<fuzzySensor2<< endl;}
    else if (fuzzySensor1 == 1 && fuzzySensor2 == 2)
        {moveForwardRight(FORWARD_SPEED_LOW, TURN_RIGHT_SPEED_LOW);
        cout << "Stage2: Gap1 :" << "fuzzySensor1: " <<fuzzySensor1<<" ,fuzzySensor2 :"<<fuzzySensor2<< endl;}
    else if (fuzzySensor1 == 1 && fuzzySensor2 == 3)
        {moveForwardRight(FORWARD_SPEED_LOW, TURN_LEFT_SPEED_LOW);
        cout << "Stage2: Gap1 :" << "fuzzySensor1: " <<fuzzySensor1<<" ,fuzzySensor2 :"<<fuzzySensor2<< endl;}
    else if (fuzzySensor1 == 2 && fuzzySensor2 == 1)
        {moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_MIDDLE);
        cout << "Stage2: Gap1 :" << "fuzzySensor1: " <<fuzzySensor1<<" ,fuzzySensor2 :"<<fuzzySensor2<< endl;}
    else if (fuzzySensor1 == 2 && fuzzySensor2 == 2)
        {moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_MIDDLE);
        cout << "Stage2: Gap1 :" << "fuzzySensor1: " <<fuzzySensor1<<" ,fuzzySensor2 :"<<fuzzySensor2<< endl;}
    else if (fuzzySensor1 == 2 && fuzzySensor2 == 3)
        {moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_MIDDLE);
        cout << "Stage2: Gap1 :" << "fuzzySensor1: " <<fuzzySensor1<<" ,fuzzySensor2 :"<<fuzzySensor2<< endl;}
    else if (fuzzySensor1 == 3 && fuzzySensor2 == 1)
        {moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_MIDDLE);
        cout << "Stage2: Gap1 :" << "fuzzySensor1: " <<fuzzySensor1<<" ,fuzzySensor2 :"<<fuzzySensor2<< endl;}
    else if (fuzzySensor1 == 3 && fuzzySensor2 == 2)
        {moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_MIDDLE);
        cout << "Stage2: Gap1 :" << "fuzzySensor1: " <<fuzzySensor1<<" ,fuzzySensor2 :"<<fuzzySensor2<< endl;}
    else if (fuzzySensor1 == 3 && fuzzySensor2 == 3)
        {moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_MIDDLE);
        cout << "Stage2: Gap1 :" << "fuzzySensor1: " <<fuzzySensor1<<" ,fuzzySensor2 :"<<fuzzySensor2<< endl;}
    else RCLCPP_INFO(this->get_logger(), "Going through the 1st gap");
}

void Stopper::Fuzzy_to2ndGap(double laserData1, double laserData2)
{
    int mleftSensor, mrightSensor;
    // sensor data fuzzification
    if (laserData1 < 0.5) mleftSensor = 1;
    else if (laserData1 < 1) mleftSensor = 2;
    else mleftSensor = 3;

    if (laserData2 < 0.5) mrightSensor = 1;
    else if (laserData2 < 1) mrightSensor = 2;
    else mrightSensor = 3;

    // Fuzzy rule base and control output
    if (mleftSensor == 1 && mrightSensor == 1)
        {moveForwardRight(FORWARD_SPEED_LOW, TURN_SPEED_ZERO);
        cout << "Stage3: Gap2 :" << "mleftSensor: " <<mleftSensor<<" ,mrightSensor :"<<mrightSensor<< endl;}
    else if (mleftSensor == 1 && mrightSensor == 2)
        {moveForwardRight(FORWARD_SPEED_LOW, TURN_RIGHT_SPEED_LOW);
        cout << "Stage3: Gap2 :" << "mleftSensor: " <<mleftSensor<<" ,mrightSensor :"<<mrightSensor<< endl;}
    else if (mleftSensor == 1 && mrightSensor == 3)
        {moveForwardRight(FORWARD_SPEED_LOW, TURN_RIGHT_SPEED_MIDDLE);
        cout << "Stage3: Gap2 :" << "mleftSensor: " <<mleftSensor<<" ,mrightSensor :"<<mrightSensor<< endl;}
    else if (mleftSensor == 2 && mrightSensor == 1)
        {moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_LEFT_SPEED_LOW);
        cout << "Stage3: Gap2 :" << "mleftSensor: " <<mleftSensor<<" ,mrightSensor :"<<mrightSensor<< endl;}
    else if (mleftSensor == 2 && mrightSensor == 2)
        {moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_LOW);
        cout << "Stage3: Gap2 :" << "mleftSensor: " <<mleftSensor<<" ,mrightSensor :"<<mrightSensor<< endl;}
    else if (mleftSensor == 2 && mrightSensor == 3)
        {moveForwardRight(FORWARD_SPEED_LOW, TURN_RIGHT_SPEED_MIDDLE);
        cout << "Stage3: Gap2 :" << "mleftSensor: " <<mleftSensor<<" ,mrightSensor :"<<mrightSensor<< endl;}
    else if (mleftSensor == 3 && mrightSensor == 1)
        {moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_LEFT_SPEED_MIDDLE);
        cout << "Stage3: Gap2 :" << "mleftSensor: " <<mleftSensor<<" ,mrightSensor :"<<mrightSensor<< endl;}
    else if (mleftSensor == 3 && mrightSensor == 2)
        {moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_MIDDLE);
        cout << "Stage3: Gap2 :" << "mleftSensor: " <<mleftSensor<<" ,mrightSensor :"<<mrightSensor<< endl;}
    else if (mleftSensor == 3 && mrightSensor == 3)
        {moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_SPEED_ZERO);
        cout << "Stage3: Gap2 :" << "mleftSensor: " <<mleftSensor<<" ,mrightSensor :"<<mrightSensor<< endl;}
    else RCLCPP_INFO(this->get_logger(), "Going through the 1st gap");
}

void Stopper::Fuzzy_ReachStop(double laserData1, double laserData2)
{
    int leftSensor, frontSensor;
    // sensor data fuzzification
    if (laserData1 < 0.5) leftSensor = 1;
    else leftSensor = 2;

    if (laserData2 < 0.3) frontSensor = 1;
    else frontSensor = 2;

    // Fuzzy rule base and control output
    if (leftSensor == 1 && frontSensor == 1)
        {moveForwardRight(TURN_SPEED_ZERO, TURN_SPEED_ZERO);
        cout << "Stage3: Gap2 :" << "leftSensor: " <<leftSensor<<" ,frontSensor :"<<frontSensor<< endl;}
    else if (leftSensor == 1 && frontSensor == 2)
        {moveForwardRight(FORWARD_SPEED_LOW, TURN_SPEED_ZERO);
        cout << "Stage3: Gap2 :" << "leftSensor: " <<leftSensor<<" ,frontSensor :"<<frontSensor<< endl;}
    else if (leftSensor == 2 && frontSensor == 1)
        {moveForwardRight(FORWARD_SPEED_LOW, TURN_SPEED_ZERO);
        cout << "Stage3: Gap2 :" << "leftSensor: " <<leftSensor<<" ,frontSensor :"<<frontSensor<< endl;}
    else if (leftSensor == 2 && frontSensor == 2)
        {moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_SPEED_ZERO);
        cout << "Stage3: Gap2 :" << "leftSensor: " <<leftSensor<<" ,frontSensor :"<<frontSensor<< endl;}
    else RCLCPP_INFO(this->get_logger(), "Going through the 1st gap");
}

void Stopper::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    leftRange = scan->ranges[300]; // get a range reading at the left angle
    mleftRange = scan->ranges[250]; // get a range reading at the front-left angle
    frontRange = scan->ranges[200]; // get a range reading at the front angle
    mrightRange = scan->ranges[150]; // get a range reading at the front-right angle
    rightRange = scan->ranges[100]; // get the range reading at the right angle
    laserFile << leftRange << ","<< mleftRange << "," << frontRange<<"," << mrightRange << "," << rightRange <<"," <<laser_index++<< endl;
    cout << "l="<<leftRange << ", ml="<< mleftRange << ", f=" << frontRange<<",mr=" << mrightRange << ",r=" << rightRange <<"," <<laser_index++<< endl;
    transformMapPoint(laserMapFile,frontRange,frontAngle,robotHeadAngle,PositionX, PositionY);
    transformMapPoint(laserMapFile, mleftRange, mleftAngle, robotHeadAngle,PositionX, PositionY);
    transformMapPoint(laserMapFile, leftRange, leftAngle, robotHeadAngle,PositionX, PositionY);
    transformMapPoint(laserMapFile, rightRange, rightAngle, robotHeadAngle,PositionX, PositionY);
    transformMapPoint(laserMapFile, mrightRange, mrightAngle, robotHeadAngle,PositionX, PositionY);

    switch(stage)
    {
        case 1:
            if (PositionY < odom_landmark1)
            {
                Fuzzy_wallFollowing(leftRange, mleftRange);
            }
            else stage = 2;
                break;

        case 2:
            if(PositionX < 1.2)
            {
                Fuzzy_to1stGap(leftRange, mleftRange);
            }
            else stage = 3;
                break;

        case 3:
            if (PositionX < 1.93)
                Fuzzy_to2ndGap(mleftRange, mrightRange);
            else stage = 4;
                break;

        case 4:
            if (PositionX < 2.28)
                Fuzzy_ReachStop(mleftRange,frontRange);
            else stage = 5;
                break;
        case 5:
            moveStop();
            break;
    }
}

void Stopper::transformMapPoint(ofstream& fp, double laserRange, double laserTh,double robotTh, double robotX, double robotY)
{
    double transX, transY;
    transX = laserRange * cos(robotTh + laserTh) + robotX;
    transY = laserRange * sin(robotTh + laserTh) + robotY;
    if (transX < 0) transX = 0;
    if (transY < 0) transY = 0;
    fp << transX << ", " << transY << endl;
}

void Stopper::startMoving()
{   
    laserMapFile.open("/home/pk/Documents/ros_workspace/src/tutorial_pkg/laserMapData.csv",ios::trunc);
    laserFile.open("/home/pk/Documents/ros_workspace/src/tutorial_pkg/laserData.csv",ios::trunc);
    odomTrajFile.open("/home/pk/Documents/ros_workspace/src/tutorial_pkg/odomTrajData.csv",ios::trunc); 
    odomVelFile.open("/home/pk/Documents/ros_workspace/src/tutorial_pkg/odomVelData.csv",ios::trunc); 
    RCLCPP_INFO(this->get_logger(), "Start moving");
    rclcpp::WallRate loop_rate(10);
    while (rclcpp::ok())
    {
        auto node = std::make_shared<Stopper>();
        rclcpp::spin(node); // update
        loop_rate.sleep(); // wait delta time
    }
    odomTrajFile.close();
    odomVelFile.close();
    laserFile.close();
    laserMapFile.close();

}




int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    Stopper stopper;
    stopper.startMoving();
    return 0;
}