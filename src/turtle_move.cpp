// ROSに関する基本的なAPIをインクルード
#include <ros/ros.h>
// Twist.msgから生成されたメッセージを定義しているヘッダをインクルード
#include <geometry_msgs/Twist.h>
// Pose.msgから生成されたメッセージを定義しているヘッダをインクルード
#include <turtlesim/Pose.h>


//パブリッシャーの定義    (メッセージ送信)
ros::Publisher velocity_publisher;
//サブスクライバーの定義  (メッセージ受信)
ros::Subscriber pose_subscriber;

//サブスクライバーの定義  (メッセージ受信)
geometry_msgs::Twist vel_msg;
//サブスクライバーの定義  (メッセージ受信)
turtlesim::Pose turtlesim_pose;

const double PI = 3.14159265359;

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
void move(double speed, double distance, bool isForward);
void rotate(double angular_speed, double relative_angle, bool clockwise);
void writeCircle(double radius, double turn_angle, bool clockwise);

int main(int argc, char **argv)
{
     //Initiate new ROS node named "turtle_move"
     ros::init(argc, argv, "turtle_move");
     ros::NodeHandle n;

     double speed, angular_speed;
     double distance, angle;
     bool isForward, clockwise;

     turtlesim::Pose goal_pose;
     velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
     pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);
     ros::Rate start(1);
     start.sleep();

     ROS_INFO("\n\n************** Start the program **************\n");
     
     rotate(2, PI-PI/4, 1);
     for(int i = 0; i < 4; i++){
          rotate(2, PI/4, 0);
          move(2, 2, 1);
          writeCircle(1, PI, 0);
          rotate(2, PI/2, 1);
          writeCircle(1, PI, 0);
          move(2, 2, 1);
          rotate(2, PI-PI/4 - 0.007, 0);
     }
     rotate(2, PI/4, 0);
     move(2, 4, 1);


     ROS_INFO("\n************** Finished the program **************\n");
     ros::spin();
     return 0;
}



void poseCallback(const turtlesim::Pose::ConstPtr & pose_message){
     turtlesim_pose.x = pose_message -> x;
     turtlesim_pose.y = pose_message -> y;
     turtlesim_pose.theta = pose_message -> theta;
}


/********************************************************************/
/*               前身と後進を提供する関数                           */
/*               引数: 移動速度, 進む距離, 1:前身 0:後進            */
/********************************************************************/
void move(double speed, double distance, bool isForward)
{
     double t0; 
     double t1;
     double current_distance = 0;

     //set a rondom linear velocity in the x-axis
     vel_msg.linear.x =0;
     vel_msg.linear.y =0;
     vel_msg.linear.z =0;
     //set a rondom angular velocity inthe y-axis
     vel_msg.angular.x =0;
     vel_msg.angular.y =0;
     vel_msg.angular.z =0;
     t0 = ros::Time::now().toSec();
     ros::Rate loop_rate(1000);

     if(isForward)
          vel_msg.linear.x = abs(speed);
     else
          vel_msg.linear.x = -abs(speed);

     do{
          velocity_publisher.publish(vel_msg);
          t1 = ros::Time::now().toSec();
          current_distance = speed * (t1-t0);
          loop_rate.sleep();
     }while(current_distance < distance);

     vel_msg.linear.x = 0;
     velocity_publisher.publish(vel_msg);
}


/***********************************************************************/
/*               目標角度へ超信地旋回させる関数                        */
/*           引数: 角速度, 回転角度[rad], 1:時計回り 0:半時計回り      */
/***********************************************************************/
void rotate(double angular_speed, double relative_angle, bool clockwise)
{
     double t0;
     double t1;
     double current_angle = 0.0;

     //set a rondom linear velocity in the x-axis
     vel_msg.linear.x = 0;
     vel_msg.linear.y = 0;
     vel_msg.linear.z = 0;
     //set a rondom angular velocity inthe y-axis
     vel_msg.angular.x = 0;
     vel_msg.angular.y = 0;
     vel_msg.angular.z = 0;
     t0 = ros::Time::now().toSec();
     ros::Rate loop_rate(1000);

     if(clockwise)
          vel_msg.angular.z = -abs(angular_speed);
     else
          vel_msg.angular.z = abs(angular_speed);

     do{
          velocity_publisher.publish(vel_msg);
          t1 = ros::Time::now().toSec();
          current_angle = angular_speed * (t1 - t0);
          loop_rate.sleep();
     }while(current_angle < relative_angle);

     vel_msg.angular.z = 0;
     velocity_publisher.publish(vel_msg);
}


/***********************************************************************/
/*                    円を描きながら旋回する関数                       */
/*        引数: 描く円の半径, 回転角度[rad], 1:時計回り 0:半時計回り   */
/***********************************************************************/
void writeCircle(double radius, double turn_angle, bool clockwise)
{
     double t0;
     double t1;
     double current_angle = 0.0;

     //set a rondom linear velocity in the x-axis
     vel_msg.linear.x = 0;
     vel_msg.linear.y = 0;
     vel_msg.linear.z = 0;
     //set a rondom angular velocity inthe y-axis
     vel_msg.angular.x = 0;
     vel_msg.angular.y = 0;
     vel_msg.angular.z = 0;
     t0 = ros::Time::now().toSec();
     ros::Rate loop_rate(1000);

     if(clockwise){
          vel_msg.angular.z = abs(turn_angle);
          vel_msg.linear.x = abs(radius * PI);
     }
     else {
          vel_msg.angular.z = abs(turn_angle);
          vel_msg.linear.x = abs(radius * PI);
     }

     do{
          velocity_publisher.publish(vel_msg);
          t1 = ros::Time::now().toSec();
          current_angle = turn_angle * (t1 - t0);
          loop_rate.sleep();
     }while(current_angle < turn_angle);

     vel_msg.angular.z = 0;
     vel_msg.linear.x = 0;
     velocity_publisher.publish(vel_msg);
}

