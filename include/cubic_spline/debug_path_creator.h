#ifndef DEBUG_PATH_CREATOR_H
#define DEBUG_PATH_CREATOR_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

class DebugPathCreator
{
public:
    DebugPathCreator();
    void process();

private:
    // 引数なし関数
    void create_path();

    // yamlファイルで設定可能な変数
    int hz_;                  // ループ周波数 [Hz]
    std::string path_frame_;  // 生成するpathのframe_id
    std::vector<double> x_;   // pathのx座標
    std::vector<double> y_;   // pathのy座標

    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Publisher
    ros::Publisher pub_debug_path_;

    // 各種オブジェクト
    nav_msgs::Path debug_path_;  // スプライン補間前の経路
};

#endif // DEBUG_PATH_CREATOR_H