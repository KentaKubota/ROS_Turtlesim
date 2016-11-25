// ROSに関する基本的なAPIをインクルード
#include <ros/ros.h>
// geometry_msgsのTwist.msgから生成されたメッセージを定義しているヘッダをインクルード
#include <geometry_msgs/Twist.h>


// パブリッシャーの定義    (メッセージ送信)
ros::Publisher velocity_publisher;

// vel_msgをgeometry_msgsのTwist型メッセージとして定義
geometry_msgs::Twist vel_msg;


// 関数のプロトタイプ宣言
void writeCircle();

int main(int argc, char **argv)
{
    // ノード名をicart_moveとしてROSの初期化をする
    ros::init(argc, argv, "icart_move");
    // 名前nでノードのネームスペースを生成
    ros::NodeHandle n;

    // velocity_publisherは"/turtle1/cmd_vel"というトピック名でTwist型のメッセージを送信するものと定義. 
    // キューのサイズは10.　10個のメッセージを保持する
    velocity_publisher = n.advertise<geometry_msgs::Twist>("/icart_mini/cmd_vel", 10);


    // スクリーンに文字を出力
    ROS_INFO("\n************** Start the turtle moving **************\n");

    while(ros::ok()){
        writeCircle();
    }

    ROS_INFO("\n************** Finished the turtle moving **************\n");
    return 0;
}

void writeCircle()
{
    vel_msg.angular.z = 1;
    vel_msg.linear.x = 1;
    velocity_publisher.publish(vel_msg);
}

