#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "estimator.h"
#include "parameters.h"
#include "utility/visualization.h"


Estimator estimator;  // 整个VINS的核心

std::condition_variable con;
double current_time = -1;
queue<sensor_msgs::ImuConstPtr> imu_buf;              // 存储IMU数据
queue<sensor_msgs::PointCloudConstPtr> feature_buf;   // 存储收到的特征点云数据
queue<sensor_msgs::PointCloudConstPtr> relo_buf;      // 存储回环检测的数据
int sum_of_wait = 0;

std::mutex m_buf;
std::mutex m_state;
std::mutex i_buf;
std::mutex m_estimator;

double latest_time;
 // 下面这种形式的初始化会将所有元素初始化为0
Eigen::Vector3d tmp_P;        // 位移        加速度积分得到
Eigen::Quaterniond tmp_Q;     // b2n的姿态   角速度积分得到
Eigen::Vector3d tmp_V;        // 速度        加速度积分得到
Eigen::Vector3d tmp_Ba;       
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;        // 上一帧的加速度
Eigen::Vector3d gyr_0;        // 上一帧的角速度
bool init_feature = 0;    // feature初始化标志，未初始化时为0
bool init_imu = 1;        // imu初始化的标识，未初始化时为1
double last_imu_t = 0;

// 使用获取到的IMU数据做位姿预测： 计算世界系下的PVQ（使用中值积分）  
void predict(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    if(init_imu)  // 当还没初始化时
    {
        latest_time = t;
        init_imu = 0;
        return;
    }

    double dt = t - latest_time;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};  // 加速度   暂时还不知道IMU的坐标系是怎么样的

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};     // 角速度

    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g; // 上一帧加速度扣掉bias后转到世界系再减去重力加速度

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg; // 角速度是前后两帧均值后再扣掉bias的
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g; // 当前帧的

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);  // 前后两帧的世界系加速度平滑

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;   // p = p0 + v * t + 0.5 * a * t ^2;
    tmp_V = tmp_V + dt * un_acc;                           // v = v0 + a * t

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

// 用估计器的状态量更新temp的，并且将tmp_imu里面的IMU全做predict
void update()
{
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = estimator.Ps[WINDOW_SIZE];
    tmp_Q = estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;

    queue<sensor_msgs::ImuConstPtr> tmp_imu_buf = imu_buf;
    for (sensor_msgs::ImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front());

}

// 对齐IMU和图像点云数据并返回
std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>>
getMeasurements()
{
    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;

    while(true)
    {
        if(imu_buf.empty() || feature_buf.empty())
            return measurements;

        if(!(imu_buf.back()->header.stamp.toSec() > feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            //ROS_WARN("wait for imu, only should happen at the beginning");
            sum_of_wait++;
            return measurements;
        }

        if (!(imu_buf.front()->header.stamp.toSec() < feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            ROS_WARN("throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
        }
        sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front();
        feature_buf.pop();

        std::vector<sensor_msgs::ImuConstPtr> IMUs;
        while (imu_buf.front()->header.stamp.toSec() < img_msg->header.stamp.toSec() + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty())
            ROS_WARN("no imu between two image");
        measurements.emplace_back(IMUs, img_msg); // 因为IMU频率比图像的高很多，所以图像特征点和多帧IMU数据组合成一组数据
    }
    return measurements;
}

// imu的回调函数
void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    if (imu_msg->header.stamp.toSec() <= last_imu_t)
    {
        ROS_WARN("imu message in disorder!");
        return;
    }

    last_imu_t = imu_msg->header.stamp.toSec();

    m_buf.lock();
    imu_buf.push(imu_msg);   // 存储的是IMU的ros数据
    m_buf.unlock();
    con.notify_one();        // notify_one()：因为只唤醒等待队列中的第一个线程；不存在锁争用，所以能够立即获得锁。其余的线程不会被唤醒，需要等待再次调用notify_one()或者notify_all()。

    last_imu_t = imu_msg->header.stamp.toSec();

    {
        std::lock_guard<std::mutex> lg(m_state);
        predict(imu_msg);    // IMU进行PVQ预测
        std_msgs::Header header = imu_msg->header;
        header.frame_id = "world";
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);             // 频率大概是200hz
    }
}

// 特征点云回调函数，接收的是 feature_tracker_node发布的特征点云数据
void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    if(!init_feature)
    {
        //skip the first detected feature, which doesn't contain optical flow speed
        init_feature = 1;  // 跳过第一帧的特征点云，因为里面不包含光流速度
        return;
    }

    m_buf.lock();
    feature_buf.push(feature_msg);
    m_buf.unlock();
    con.notify_one();
}

// restart回调函数，收到restart消息时清空feature_buf和imu_buf，估计器重置，时间重置
void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if(restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        m_buf.lock();
        while(!feature_buf.empty())
            feature_buf.pop();

        while(!imu_buf.empty())
            imu_buf.pop();

        m_buf.unlock();
        m_estimator.lock();
        estimator.clearState();    // 估计器状态清零
        estimator.setParameter();
        m_estimator.unlock();
        current_time = -1;
        last_imu_t = 0;
    }
    return;
}

// 回环检测信号的回调函数
void relocalization_callback(const sensor_msgs::PointCloudConstPtr &points_msg)
{
    //printf("relocalization callback! \n");
    m_buf.lock();
    relo_buf.push(points_msg);
    m_buf.unlock();
}

// thread: visual-inertial odometry
void process()
{
    while(true)
    {
        std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;
        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&]{return (measurements = getMeasurements()).size() != 0;}); // con.wait(mutex, true/false); 当为false时候让线程等待
        lk.unlock();
        m_estimator.lock();
        for(auto &measurement : measurements)
        {
            auto img_msg = measurement.second;
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            for(auto &imu_msg : measurement.first)
            {
                double t = imu_msg->header.stamp.toSec();
                double img_t = img_msg->header.stamp.toSec() + estimator.td;
                if (t <= img_t) // 当imu的时间小于图像时间戳
                { 
                    if (current_time < 0)
                        current_time = t;

                    double dt = t - current_time; // imu帧间的时间间隔
                    ROS_ASSERT(dt >= 0);
                    current_time = t;
                    dx = imu_msg->linear_acceleration.x;
                    dy = imu_msg->linear_acceleration.y;
                    dz = imu_msg->linear_acceleration.z;
                    rx = imu_msg->angular_velocity.x;
                    ry = imu_msg->angular_velocity.y;
                    rz = imu_msg->angular_velocity.z;
                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz)); // 主要做预积分计算
                    //printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);

                }
                else  // imu时间戳大于图像时间戳（上一帧时间戳小于图像时间戳，也就是图像时间戳处于两帧IMU之间）
                {
                    double dt_1 = img_t - current_time;  // current_time是上一帧的IMU的时间
                    double dt_2 = t - img_t;             // t是当前帧IMU的时间
                    current_time = img_t;
                    ROS_ASSERT(dt_1 >= 0);
                    ROS_ASSERT(dt_2 >= 0);
                    ROS_ASSERT(dt_1 + dt_2 > 0);

                    /* 对根据图像的时间戳对加速度和角速度做线性插值 */ 
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));  // 把根据图像插值出来的IMU数据做预积分
                    //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                }
            }
            /*  开始处理回环检测信号   set relocalization frame  */ 
            sensor_msgs::PointCloudConstPtr relo_msg = NULL;  // 回环的数据也是点云
            while(!relo_buf.empty())
            {
                relo_msg = relo_buf.front();
                relo_buf.pop();
            }

            if(relo_msg != NULL)  // 如果有回环，取出回环的帧号、匹配点和姿态
            {
                vector<Vector3d> match_points;  // 匹配的点
                double frame_stamp = relo_msg->header.stamp.toSec();
                for(unsigned int i = 0; i < relo_msg->points.size(); i++)
                {
                    Vector3d u_v_id;
                    u_v_id.x() = relo_msg->points[i].x;
                    u_v_id.y() = relo_msg->points[i].y;
                    u_v_id.z() = relo_msg->points[i].z;
                    match_points.push_back(u_v_id);
                }

                
                Vector3d relo_t(relo_msg->channels[0].values[0], relo_msg->channels[0].values[1], relo_msg->channels[0].values[2]); // 位置
                Quaterniond relo_q(relo_msg->channels[0].values[3], relo_msg->channels[0].values[4], relo_msg->channels[0].values[5], relo_msg->channels[0].values[6]);
                Matrix3d relo_r = relo_q.toRotationMatrix();  // 姿态
                int frame_index;                              
                frame_index = relo_msg->channels[0].values[7];// 帧号
                estimator.setReloFrame(frame_stamp, frame_index, match_points, relo_t, relo_r); // 暂时没看明白这个函数的具体作用
            }

            ROS_DEBUG("processing vision data with stamp %f \n", img_msg->header.stamp.toSec());

            TicToc t_s;
            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image; // key对应特征ID，value对应不同相机对同一特征点的xyz、uz和速度（pair中的int就是相机ID）
            for (unsigned int i = 0; i < img_msg->points.size(); i++)
            {
                int v = img_msg->channels[0].values[i] + 0.5;
                int feature_id = v / NUM_OF_CAM;   // 特征点ID
                int camera_id = v % NUM_OF_CAM;    // 相机ID
                double x = img_msg->points[i].x;              // 3维空间位置坐标
                double y = img_msg->points[i].y;
                double z = img_msg->points[i].z;
                double p_u = img_msg->channels[1].values[i];       // 像素坐标
                double p_v = img_msg->channels[2].values[i];
                double velocity_x = img_msg->channels[3].values[i];    // 像素速度
                double velocity_y = img_msg->channels[4].values[i];
                ROS_ASSERT(z == 1);
                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                image[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
            }

            estimator.processImage(image, img_msg->header);  // TODO: 

            double whole_t = t_s.toc();
            printStatistics(estimator, whole_t);
            std_msgs::Header header = img_msg->header;
            header.frame_id = "world";

            /* 发布各种ROS信息 */
            pubOdometry(estimator, header);
            pubKeyPoses(estimator, header);
            pubCameraPose(estimator, header);
            pubPointCloud(estimator, header);
            pubTF(estimator, header);
            pubKeyframe(estimator);
            if (relo_msg != NULL)
                pubRelocalization(estimator);
            //ROS_ERROR("end: %f, at %f", img_msg->header.stamp.toSec(), ros::Time::now().toSec());
        }

        m_estimator.unlock();
        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();
        m_state.unlock();
        m_buf.unlock();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);
    estimator.setParameter();
#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif
    ROS_WARN("waiting for image and imu...");

    registerPub(n);

    ros::Subscriber sub_imu         = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_image       = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    ros::Subscriber sub_restart     = n.subscribe("/feature_tracker/restart", 2000, restart_callback);
    ros::Subscriber sub_relo_points = n.subscribe("/pose_graph/match_points", 2000, relocalization_callback);

    std::thread measurement_process{process};  // VIO主线程
    ros::spin();

    return 0;
}
