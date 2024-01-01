#include "cubic_spline/cubic_spline.h"

CubicSpline::CubicSpline():private_nh_("~")
{
    // param
    private_nh_.param("hz", hz_, {10});
    private_nh_.param("path_frame", path_frame_, {"map"});
    private_nh_.param("cource_length", cource_length_, {3.2});
    private_nh_.param("resolution", resolution_, {0.05});
    // private_nh_.getParam("y", y_);  // ここはyamlファイルで必ず指定する

    // private_nh_.param("init_x", init_x_, {0.0});
    // private_nh_.param("init_y", init_y_, {0.0});
    // private_nh_.param("init_theta", init_theta_, {0.0});
    // private_nh_.param("cource_length", cource_length_, {20.0});
    // private_nh_.param("resolution", resolution_, {0.05});

    //Subscriber
    sub_debug_path_ = nh_.subscribe("/debug_path", 1, &CubicSpline::debug_path_callback, this, ros::TransportHints().reliable().tcpNoDelay());

    // publisher
    pub_optimal_path_ = nh_.advertise<nav_msgs::Path>("/optimal_path", 1);
}

// debug_pathのコールバック関数
void CubicSpline::debug_path_callback(const nav_msgs::Path::ConstPtr& msg)
{
    debug_path_ = *msg;
    flag_debug_path_ = true;
}

// スプライン補間用のパラメーターを初期化
void CubicSpline::init_param()
{
    // clear()を使って，配列の中身をすべて削除
    a_.clear();
    b_.clear();
    c_.clear();
    d_.clear();
    w_.clear();
}

// スプライン補間用のパラメーターを計算
// コードは下記URLを参照
// URL : https://myenigma.hatenablog.com/entry/2016/10/12/073335#C%E3%82%B5%E3%83%B3%E3%83%97%E3%83%AB%E3%82%B3%E3%83%BC%E3%83%89
void CubicSpline::calc_param()
{
    int ndata = debug_path_.poses.size();

    for(const auto& pose : debug_path_.poses)
    {
        a_.push_back(pose.pose.position.y);
    }

    for(int i=0; i<=ndata; i++)
    {
        if(i == 0)
        {
            c_.push_back(0.0);
        }
        else if(i == ndata)
        {
            c_.push_back(0.0);
        }
        else
        {
            double c = 3.0 * (a_[i-1] - 2.0*a_[i] + a_[i+1]);
            c_.push_back(c);
        }
    }

    for(int i=0; i<=ndata; i++)
    {
        if(i == 0)
        {
            w_.push_back(0.0);
        }
        else
        {
            double tmp = 4.0 - w_[i-1];

            c_[i] = (c_[i] - c_[i-1]) / tmp;

            double w = 1.0 / tmp;
            w_.push_back(w);
        }
    }

    for(int i=(ndata-1); i>0; i--)
    {
        c_[i] -= c_[i+1] * w_[i];
    }

    for(int i=0; i<=ndata; i++)
    {
        if(i == ndata)
        {
            d_.push_back(0.0);
            b_.push_back(0.0);
        }
        else
        {
            double d = (c_[i+1] - c_[i]) / 3.0;
            d_.push_back(d);

            double b = a_[i+1] - a_[i] - c_[i] - d_[i];
            b_.push_back(b);
        }
    }
}

// スプライン補間後のy座標を計算
double CubicSpline::calc_y(const double x, const double path_x, const int index)
{
    double dx = x - path_x;

    double y = a_[index] + (b_[index] + (c_[index] + d_[index]*dx) *dx) *dx;
    return y;
}

// スプライン補間を実行
void CubicSpline::optimize_path()
{
    nav_msgs::Path path;
    path.header.frame_id = path_frame_;

    // パラメーターを初期化
    init_param();

    // パラメーターを計算
    calc_param();

    int debug_path_index = 0;
    int ndata = debug_path_.poses.size();

    for(double x=0.0; x<=cource_length_; x+=resolution_)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = path_frame_;

        // 基準となるx座標を探索
        if(debug_path_index < ndata-1)  // セグフォしないかの確認用
        {
            if(x >= debug_path_.poses[debug_path_index+1].pose.position.x)
                debug_path_index ++;
        }

        double path_x = debug_path_.poses[debug_path_index].pose.position.x;

        // y座標を計算
        double y = calc_y(x, path_x, debug_path_index);
        ROS_INFO_STREAM("x : " << x);
        ROS_INFO_STREAM("y : " << y);

        // 座標を代入
        pose.pose.position.x = x;
        pose.pose.position.y = y;

        path.poses.push_back(pose);
    }

    optimal_path_ = path;
}

//メイン文で実行する関数
void CubicSpline::process()
{
    ros::Rate loop_rate(hz_);
    
    while(ros::ok())
    {
        if(flag_spline_ == false)
        {
            if(flag_debug_path_ == true)
            {
                optimize_path();
                flag_spline_ = true;
                ROS_INFO_STREAM("Cubic spline!");
            }
        }
        else if(flag_spline_ == true)
        {
            optimal_path_.header.stamp = ros::Time::now();
            pub_optimal_path_.publish(optimal_path_);
            ROS_INFO_STREAM("Pub path!");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}