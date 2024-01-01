#ifndef CUBIC_SPLINE_H
#define CUBIC_SPLINE_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

class CubicSpline
{
public:
    CubicSpline();
    void process();

private:
    //コールバック関数
    void debug_path_callback(const nav_msgs::Path::ConstPtr& msg);

    // 引数あり関数
    double calc_y(const double x, const double path_x, const int index);

    // 引数なし関数
    void init_param();
    void calc_param();
    void optimize_path();

    // yamlファイルで設定可能な変数
    int hz_;                  // ループ周波数 [Hz]
    std::string path_frame_;  // 生成するpathのframe_id
    double cource_length_;    // スプライン補間をする軌道の長さ [m]
    double resolution_;       // 軌道の刻み幅 [m]
    // std::vector<double> x_;   // pathのx座標
    // std::vector<double> y_;   // pathのy座標

    // その他の変数
    // スプライン補間用のパラメーター
    std::vector<double> a_;
    std::vector<double> b_;
    std::vector<double> c_;
    std::vector<double> d_;
    std::vector<double> w_;
   

    //msgの受け取り判定用
    bool flag_debug_path_ = false;

    // スプライン補間を実行したかの確認用
    bool flag_spline_ = false;

    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    //Subscriber
    ros::Subscriber sub_debug_path_;

    // Publisher
    ros::Publisher pub_optimal_path_;

    // 各種オブジェクト
    nav_msgs::Path debug_path_;  // global path
    nav_msgs::Path optimal_path_;  // global path
};

#endif // CUBIC_SPLINE_H