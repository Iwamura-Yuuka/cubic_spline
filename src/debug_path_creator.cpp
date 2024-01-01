#include "cubic_spline/debug_path_creator.h"

DebugPathCreator::DebugPathCreator():private_nh_("~")
{
    // param
    private_nh_.param("hz", hz_, {10});
    private_nh_.param("path_frame", path_frame_, {"map"});
    private_nh_.getParam("x", x_);  // ここはyamlファイルで必ず指定する
    private_nh_.getParam("y", y_);  // ここはyamlファイルで必ず指定する

    // publisher
    pub_debug_path_ = nh_.advertise<nav_msgs::Path>("/debug_path", 1);
}

// 軌道を生成
void DebugPathCreator::create_path()
{
    debug_path_.header.frame_id = path_frame_;

    const int size = x_.size();

    for(int i=0; i<size; i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = path_frame_;

        // 座標を代入
        pose.pose.position.x = x_[i];
        pose.pose.position.y = y_[i];

        debug_path_.poses.push_back(pose);        
    }

}

//メイン文で実行する関数
void DebugPathCreator::process()
{
    ros::Rate loop_rate(hz_);
    create_path();
    
    while(ros::ok())
    {
        debug_path_.header.stamp = ros::Time::now();
        pub_debug_path_.publish(debug_path_);
        loop_rate.sleep();
    }
}