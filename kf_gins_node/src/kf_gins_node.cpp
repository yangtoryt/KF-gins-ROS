// src/kf_gins_node.cpp (robust version for NaN hunts)
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "kf-gins/gi_engine.h"
#include "kf-gins/insmech.h"
#include "kf-gins/kf_gins_types.h"

#include <Eigen/Geometry>
#include <vector>
#include <string>
#include <cmath>
#include <limits>

using std::placeholders::_1;

static inline bool finite3(const Eigen::Vector3d &v) {
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

class KfGinsNode : public rclcpp::Node {
public:
  KfGinsNode()
  : Node("kf_gins_ros2_node"), last_imu_time_(-1.0) {
    // ------------ 参数声明（与你的 YAML 对齐） ------------
    this->declare_parameter<std::string>("imupath", "");
    this->declare_parameter<std::string>("gnsspath", "");
    this->declare_parameter<std::string>("outputpath", "");
    this->declare_parameter<int>("imudatalen", 7);
    this->declare_parameter<int>("imudatarate", 200);
    this->declare_parameter<double>("starttime", 0.0);
    this->declare_parameter<double>("endtime", -1.0);

    this->declare_parameter<std::vector<double>>("initpos", {0.0, 0.0, 0.0}); // deg,deg,h(m)
    this->declare_parameter<std::vector<double>>("initvel", {0.0, 0.0, 0.0}); // m/s
    this->declare_parameter<std::vector<double>>("initatt", {0.0, 0.0, 0.0}); // deg

    this->declare_parameter<std::vector<double>>("initgyrbias", {0.0, 0.0, 0.0}); // deg/h
    this->declare_parameter<std::vector<double>>("initaccbias", {0.0, 0.0, 0.0}); // 1e-5 g
    this->declare_parameter<std::vector<double>>("initgyrscale", {0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("initaccscale", {0.0, 0.0, 0.0});

    this->declare_parameter<std::vector<double>>("initposstd", {1.0, 1.0, 1.0}); // m
    this->declare_parameter<std::vector<double>>("initvelstd", {0.1, 0.1, 0.1}); // m/s
    this->declare_parameter<std::vector<double>>("initattstd", {0.1, 0.1, 0.1}); // deg

    this->declare_parameter<std::vector<double>>("imunoise.arw",  {0.003, 0.003, 0.003});
    this->declare_parameter<std::vector<double>>("imunoise.vrw",  {0.03, 0.03, 0.03});
    this->declare_parameter<std::vector<double>>("imunoise.gbstd",{0.027, 0.027, 0.027});
    this->declare_parameter<std::vector<double>>("imunoise.abstd",{15.0, 15.0, 15.0});
    this->declare_parameter<std::vector<double>>("imunoise.gsstd",{300.0, 300.0, 300.0});
    this->declare_parameter<std::vector<double>>("imunoise.asstd",{300.0, 300.0, 300.0});
    this->declare_parameter<double>("imunoise.corrtime", 4.0);

    this->declare_parameter<std::vector<double>>("antlever", {0.0, 0.0, 0.0});

    // 新增：IMU 话题是否为增量、IMU 单位
    this->declare_parameter<bool>("imu_is_increment", false); // 默认按 ROS 标准：速率/加速度
    this->declare_parameter<std::string>("imu_units", "si");  // "si" 或 "deg_g"

    // ------------ 读参数填 options ------------
    GINSOptions options;
    std::vector<double> v;

    // 初始化标准差默认值，然后被 YAML 覆盖
    options.initstate_std.pos   = Eigen::Vector3d(1.0, 1.0, 1.0);
    options.initstate_std.vel   = Eigen::Vector3d(0.1, 0.1, 0.1);
    options.initstate_std.euler = Eigen::Vector3d(0.1 * D2R, 0.1 * D2R, 0.1 * D2R);
    options.initstate_std.imuerror.gyrbias  = Eigen::Vector3d(1e-6, 1e-6, 1e-6);
    options.initstate_std.imuerror.accbias  = Eigen::Vector3d(1e-4, 1e-4, 1e-4);
    options.initstate_std.imuerror.gyrscale = Eigen::Vector3d(1e-6, 1e-6, 1e-6);
    options.initstate_std.imuerror.accscale = Eigen::Vector3d(1e-6, 1e-6, 1e-6);

    if (this->get_parameter("initposstd", v) && v.size() >= 3)
      options.initstate_std.pos = Eigen::Vector3d(v[0], v[1], v[2]);
    if (this->get_parameter("initvelstd", v) && v.size() >= 3)
      options.initstate_std.vel = Eigen::Vector3d(v[0], v[1], v[2]);
    if (this->get_parameter("initattstd", v) && v.size() >= 3)
      options.initstate_std.euler = Eigen::Vector3d(v[0]*D2R, v[1]*D2R, v[2]*D2R);

    int imudatarate = 200;
    this->get_parameter("imudatarate", imudatarate);
    imudatarate_ = imudatarate;

    if (this->get_parameter("initpos", v) && v.size() >= 3) {
      options.initstate.pos[0] = v[0] * D2R;
      options.initstate.pos[1] = v[1] * D2R;
      options.initstate.pos[2] = v[2] ;
    }
    if (this->get_parameter("initvel", v) && v.size() >= 3) {
      options.initstate.vel = Eigen::Vector3d(v[0], v[1], v[2]);
    }
    if (this->get_parameter("initatt", v) && v.size() >= 3) {
      options.initstate.euler = Eigen::Vector3d(v[0]*D2R, v[1]*D2R, v[2]*D2R);
    }

    if (this->get_parameter("initgyrbias", v) && v.size() >= 3) {
      options.initstate.imuerror.gyrbias = Eigen::Vector3d(
        v[0]*D2R/3600.0, v[1]*D2R/3600.0, v[2]*D2R/3600.0);
    }
    if (this->get_parameter("initaccbias", v) && v.size() >= 3) {
      options.initstate.imuerror.accbias = Eigen::Vector3d(
        v[0]*1e-5, v[1]*1e-5, v[2]*1e-5);
    }
    if (this->get_parameter("initgyrscale", v) && v.size() >= 3) {
      options.initstate.imuerror.gyrscale = Eigen::Vector3d(v[0], v[1], v[2]);
    }
    if (this->get_parameter("initaccscale", v) && v.size() >= 3) {
      options.initstate.imuerror.accscale = Eigen::Vector3d(v[0], v[1], v[2]);
    }

    if (this->get_parameter("antlever", v) && v.size() >= 3) {
      options.antlever = Eigen::Vector3d(v[0], v[1], v[2]);
    }

    if (this->get_parameter("imunoise.arw", v) && v.size() >= 3)
      options.imunoise.gyr_arw = Eigen::Vector3d(v[0], v[1], v[2]);
    if (this->get_parameter("imunoise.vrw", v) && v.size() >= 3)
      options.imunoise.acc_vrw = Eigen::Vector3d(v[0], v[1], v[2]);
    if (this->get_parameter("imunoise.gbstd", v) && v.size() >= 3)
      options.imunoise.gyrbias_std = Eigen::Vector3d(v[0], v[1], v[2]);
    if (this->get_parameter("imunoise.abstd", v) && v.size() >= 3)
      options.imunoise.accbias_std = Eigen::Vector3d(v[0], v[1], v[2]);
    if (this->get_parameter("imunoise.gsstd", v) && v.size() >= 3)
      options.imunoise.gyrscale_std = Eigen::Vector3d(v[0], v[1], v[2]);
    if (this->get_parameter("imunoise.asstd", v) && v.size() >= 3)
      options.imunoise.accscale_std = Eigen::Vector3d(v[0], v[1], v[2]);
    double corr = 4.0;
    this->get_parameter("imunoise.corrtime", corr);
    options.imunoise.corr_time = corr;

    this->get_parameter("imu_is_increment", imu_is_increment_);
    this->get_parameter("imu_units", imu_units_);

    // Print final resolved imu flags:
    RCLCPP_INFO(this->get_logger(), "Final imu_is_increment = %s, imu_units = %s",
                imu_is_increment_ ? "true" : "false", imu_units_.c_str());


    // --- Protective sanitize of options before handing to GIEngine ---
    auto sanitize_vec3 = [](Eigen::Vector3d &v, const Eigen::Vector3d &fallback){
        for (int i=0;i<3;++i) if (!std::isfinite(v[i])) v[i] = fallback[i];
    };
    Eigen::Vector3d zero3 = Eigen::Vector3d::Zero();
    sanitize_vec3(options.initstate.pos, zero3);
    sanitize_vec3(options.initstate.vel, zero3);
    sanitize_vec3(options.initstate.euler, zero3);
    sanitize_vec3(options.initstate.imuerror.gyrbias, zero3);
    sanitize_vec3(options.initstate.imuerror.accbias, zero3);
    sanitize_vec3(options.initstate.imuerror.gyrscale, zero3);
    sanitize_vec3(options.initstate.imuerror.accscale, zero3);

    // Print them (debug)
    RCLCPP_INFO(this->get_logger(), "Sanitized initstate pos=%g %g %g", options.initstate.pos[0], options.initstate.pos[1], options.initstate.pos[2]);
    RCLCPP_INFO(this->get_logger(), "Sanitized initstate vel=%g %g %g", options.initstate.vel[0], options.initstate.vel[1], options.initstate.vel[2]);
    RCLCPP_INFO(this->get_logger(), "Sanitized initstate euler=%g %g %g (rad)", options.initstate.euler[0], options.initstate.euler[1], options.initstate.euler[2]);
    RCLCPP_INFO(this->get_logger(), "Sanitized imuerror gyrbias=%g %g %g", options.initstate.imuerror.gyrbias[0], options.initstate.imuerror.gyrbias[1], options.initstate.imuerror.gyrbias[2]);


    // 打印配置（若库里有这个函数）
    options.print_options();

    // ------------ 创建引擎 ------------
    engine_ = std::make_shared<GIEngine>(options);

    // ------------ 订阅与发布 ------------
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", rclcpp::SensorDataQoS(),
      std::bind(&KfGinsNode::imuCallback, this, _1));

    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/fix", 10, std::bind(&KfGinsNode::gpsCallback, this, _1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/kf_gins/odom", 10);

    RCLCPP_INFO(this->get_logger(), "KF-GINS ROS2 Node initialized (imu_is_increment=%s, imu_units=%s)",
                imu_is_increment_ ? "true" : "false", imu_units_.c_str());
  }

private:
  static bool navstateHasInvalid(const NavState &s) {
    for (int i = 0; i < 3; ++i) {
      if (!std::isfinite(s.pos[i]) || !std::isfinite(s.vel[i]) || !std::isfinite(s.euler[i])) return true;
      if (!std::isfinite(s.imuerror.gyrbias[i]) || !std::isfinite(s.imuerror.accbias[i])) return true;
      if (!std::isfinite(s.imuerror.gyrscale[i]) || !std::isfinite(s.imuerror.accscale[i])) return true;
    }
    return false;
  }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    IMU imu_data;

    // 时间戳（秒）
    const double cur_time = double(msg->header.stamp.sec) + double(msg->header.stamp.nanosec) * 1e-9;
    imu_data.time = cur_time;

    // 采样周期
    double dt = 0.0;
    if (last_imu_time_ < 0.0) {
      dt = 1.0 / std::max(1, imudatarate_);
    } else {
      dt = cur_time - last_imu_time_;
      if (!(dt > 0.0) || !std::isfinite(dt)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "IMU dt invalid: %.9f, fallback to %g", dt, 1.0/std::max(1, imudatarate_));
        dt = 1.0 / std::max(1, imudatarate_);
      }
      if (dt < 1e-6) dt = 1e-6;
      if (dt > 0.2) dt = 0.2; // 
    }
    imu_data.dt = dt;
    last_imu_time_ = cur_time;

    // 读取原始（ROS 标准：rad/s, m/s²）
    double wx = msg->angular_velocity.x;
    double wy = msg->angular_velocity.y;
    double wz = msg->angular_velocity.z;
    double ax = msg->linear_acceleration.x;
    double ay = msg->linear_acceleration.y;
    double az = msg->linear_acceleration.z;

    // 快速检查 ROS 消息字段
    if (!std::isfinite(wx) || !std::isfinite(wy) || !std::isfinite(wz) ||
        !std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(az)) {
      RCLCPP_WARN(this->get_logger(), "IMU ROS message contains non-finite values, dropping sample. stamp=%.9f", cur_time);
      return;
    }

    // 若单位为 "deg_g"，做转换；否则（默认 "si"）不转换
    if (imu_units_ == "deg_g") {
      const double DEG2RAD = M_PI / 180.0;
      const double G2MS2   = 9.81;
      wx *= DEG2RAD; wy *= DEG2RAD; wz *= DEG2RAD;
      ax *= G2MS2;   ay *= G2MS2;   az *= G2MS2;
    }

    // 填充 dtheta / dvel
    if (imu_is_increment_) {
      imu_data.dtheta << wx, wy, wz;
      imu_data.dvel   << ax, ay, az;
    } else {
      imu_data.dtheta << wx * dt, wy * dt, wz * dt;
      imu_data.dvel   << ax * dt, ay * dt, az * dt;
    }

    // 严格检查计算后的增量
    if (!finite3(imu_data.dtheta) || !finite3(imu_data.dvel)) {
      RCLCPP_ERROR(this->get_logger(), "Computed dtheta/dvel contain non-finite values, drop sample. stamp=%.9f dt=%g", cur_time, dt);
      RCLCPP_DEBUG(this->get_logger(), "raw wx,wy,wz = %g %g %g; ax,ay,az = %g %g %g", wx, wy, wz, ax, ay, az);
      return;
    }

    // 诊断日志（在 debug 模式下打印）
    RCLCPP_DEBUG(this->get_logger(), "IMU sample stamp=%.6f dt=%.6g dtheta=[%.8g, %.8g, %.8g] dvel=[%.8g, %.8g, %.8g]",
                 imu_data.time, imu_data.dt,
                 imu_data.dtheta.x(), imu_data.dtheta.y(), imu_data.dtheta.z(),
                 imu_data.dvel.x(), imu_data.dvel.y(), imu_data.dvel.z());

    imu_data.odovel = 0.0;

    // 将 safe 的数据传入 GIEngine
    if (!engine_) {
      RCLCPP_ERROR(this->get_logger(), "engine_ is null, cannot process IMU");
      return;
    }

    // 再次保护：查阅 GIEngine 内部 imuerror 是否已是 finite
    // 如果 engine_->debugDump 可用，则可以要求其打印（不用调用在正常路径）
    engine_->addImuData(imu_data, true); // 传 true 以让引擎在内部补偿偏差（若实现了）
    engine_->newImuProcess();

    // 发布或调试（在 publishState() 中已有检查）
    publishState();
  }


  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    GNSS gnss_data;
    gnss_data.time = double(msg->header.stamp.sec) + double(msg->header.stamp.nanosec) * 1e-9;
    gnss_data.blh  << msg->latitude, msg->longitude, msg->altitude;
    gnss_data.std  << 1.0, 1.0, 3.0; 
    gnss_data.isvalid = std::isfinite(gnss_data.blh.x()) && std::isfinite(gnss_data.blh.y()) && std::isfinite(gnss_data.blh.z());

    engine_->addGnssData(gnss_data);
    publishState();
  }

  void publishState() {
    NavState state = engine_->getNavState();

    if (navstateHasInvalid(state)) {
      RCLCPP_ERROR(this->get_logger(), "NavState contains NaN/Inf! Skipping publish and dumping debug info.");
      RCLCPP_ERROR(this->get_logger(), "last_imu_time=%.6f", last_imu_time_);
      if (engine_) engine_->debugDump();
      return;
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = this->get_clock()->now();
    odom.header.frame_id = "map";

    odom.pose.pose.position.x = state.pos(0);
    odom.pose.pose.position.y = state.pos(1);
    odom.pose.pose.position.z = state.pos(2);

    // euler (rad) -> quaternion
    Eigen::AngleAxisd rx(state.euler(0), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd ry(state.euler(1), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rz(state.euler(2), Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = rz * ry * rx;

    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom_pub_->publish(odom);
  }

private:
  std::shared_ptr<GIEngine> engine_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  double last_imu_time_;
  bool   imu_is_increment_{false};
  int    imudatarate_{200};
  std::string imu_units_{"si"}; // "si" 或 "deg_g"
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KfGinsNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
