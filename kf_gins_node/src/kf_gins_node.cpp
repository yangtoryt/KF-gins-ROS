// src/kf_gins_node.cpp  (统一：GIEngine 的 state.pos 按 BLH(rad) 解释)

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "kf-gins/gi_engine.h"
#include "kf-gins/insmech.h"
#include "kf-gins/kf_gins_types.h"

#include <Eigen/Geometry>
#include <vector>
#include <string>
#include <cmath>
#include <limits>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using std::placeholders::_1;

static inline bool finite3(const Eigen::Vector3d &v) {
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

// BLH (rad, m) -> ECEF (m)
static void blhToEcef(double lat, double lon, double h, Eigen::Vector3d &ecef) {
  const double a = 6378137.0;
  const double f = 1.0 / 298.257223563;
  const double e2 = f * (2.0 - f);
  const double sLat = std::sin(lat), cLat = std::cos(lat);
  const double sLon = std::sin(lon), cLon = std::cos(lon);
  const double N = a / std::sqrt(1.0 - e2 * sLat * sLat);
  ecef.x() = (N + h) * cLat * cLon;
  ecef.y() = (N + h) * cLat * sLon;
  ecef.z() = (N * (1.0 - e2) + h) * sLat;
}

// ECEF->ENU rotation at (lat0,lon0) (rad)
static Eigen::Matrix3d ecefToEnuRot(double lat0, double lon0) {
  const double sLat = std::sin(lat0), cLat = std::cos(lat0);
  const double sLon = std::sin(lon0), cLon = std::cos(lon0);
  Eigen::Matrix3d R;
  R << -sLon,         cLon,        0,
       -cLon*sLat, -sLon*sLat,  cLat,
        cLon*cLat,  sLon*cLat,  sLat;
  return R;
}

class KfGinsNode : public rclcpp::Node {
public:
  KfGinsNode()
  : Node("kf_gins_ros2_node"), last_imu_time_(-1.0) {

    // ---------------- Params ----------------
    this->declare_parameter<std::string>("imu_topic", "/imu/data");
    this->declare_parameter<std::string>("gps_topic", "/gps/fix_cov");  // 默认用带协方差的话题

    this->declare_parameter<int>("imudatarate", 200);
    this->declare_parameter<bool>("imu_linear_no_gravity", true);       // 你的 IMU 是去重力数据

    this->declare_parameter<std::vector<double>>("initpos", {0.0, 0.0, 0.0}); // [deg,deg,m]
    this->declare_parameter<std::vector<double>>("initvel", {0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("initatt", {0.0, 0.0, 0.0}); // [deg]

    this->declare_parameter<std::vector<double>>("initposstd",{1.0,1.0,1.0});
    this->declare_parameter<std::vector<double>>("initvelstd",{0.1,0.1,0.1});
    this->declare_parameter<std::vector<double>>("initattstd",{0.1,0.1,0.1});

    this->declare_parameter<std::vector<double>>("initgyrbias", {0.0,0.0,0.0}); // [deg/h]
    this->declare_parameter<std::vector<double>>("initaccbias", {0.0,0.0,0.0}); // [mGal]
    this->declare_parameter<std::vector<double>>("initgyrscale",{0.0,0.0,0.0});
    this->declare_parameter<std::vector<double>>("initaccscale",{0.0,0.0,0.0});

    this->declare_parameter<std::vector<double>>("imunoise.arw",  {0.003,0.003,0.003});
    this->declare_parameter<std::vector<double>>("imunoise.vrw",  {0.03,0.03,0.03});
    this->declare_parameter<std::vector<double>>("imunoise.gbstd",{0.027,0.027,0.027});
    this->declare_parameter<std::vector<double>>("imunoise.abstd",{0.05,0.05,0.05});
    this->declare_parameter<std::vector<double>>("imunoise.gsstd",{500,500,500});
    this->declare_parameter<std::vector<double>>("imunoise.asstd",{500,500,500});
    this->declare_parameter<double>("imunoise.corrtime", 4.0);

    this->declare_parameter<std::vector<double>>("antlever", {0.0,0.0,0.0});

    this->declare_parameter<bool>("imu_is_increment", true);     // 你的播放器给的是“增量”，但你现在实际用 false（见yaml）
    this->declare_parameter<std::string>("imu_units", "si");     // "si" or "deg_g"

    // 关键：把 GIEngine 的 state.pos 解释为 BLH(rad)
    this->declare_parameter<std::string>("state_pos_mode", "blh_rad"); // enu|ecef|blh_rad|blh_deg|auto
    this->declare_parameter<std::string>("z_strategy", "init_alt");    // zero|init_alt|raw_clamped
    this->declare_parameter<bool>("publish_tf", true);

    this->declare_parameter<double>("orientation_slerp_alpha", 0.2);
    this->declare_parameter<double>("clamp_xy_max_m", 5e5);
    this->declare_parameter<double>("clamp_z_band_m", 2000.0);
    this->declare_parameter<double>("smooth_z_step_m", 50.0);
    this->declare_parameter<int>("path_max_len", 50000);

    // --------- Build GI options ---------
    GINSOptions options;
    std::vector<double> v;

    options.initstate_std.pos   = Eigen::Vector3d(1,1,1);
    options.initstate_std.vel   = Eigen::Vector3d(0.1,0.1,0.1);
    options.initstate_std.euler = Eigen::Vector3d(0.1*D2R,0.1*D2R,0.1*D2R);
    options.initstate_std.imuerror.gyrbias  = Eigen::Vector3d(1e-6,1e-6,1e-6);
    options.initstate_std.imuerror.accbias  = Eigen::Vector3d(1e-4,1e-4,1e-4);
    options.initstate_std.imuerror.gyrscale = Eigen::Vector3d(1e-6,1e-6,1e-6);
    options.initstate_std.imuerror.accscale = Eigen::Vector3d(1e-6,1e-6,1e-6);

    if (this->get_parameter("initposstd", v) && v.size()>=3) options.initstate_std.pos = {v[0],v[1],v[2]};
    if (this->get_parameter("initvelstd", v) && v.size()>=3) options.initstate_std.vel = {v[0],v[1],v[2]};
    if (this->get_parameter("initattstd", v) && v.size()>=3) options.initstate_std.euler = {v[0]*D2R,v[1]*D2R,v[2]*D2R};

    int imudatarate = 200; this->get_parameter("imudatarate", imudatarate); imudatarate_ = imudatarate;

    // 初始位置：把度转弧度，传给引擎用弧度
    if (this->get_parameter("initpos", v) && v.size()>=3) {
      init_blh_rad_[0] = v[0]*D2R;
      init_blh_rad_[1] = v[1]*D2R;
      init_blh_rad_[2] = v[2];
      options.initstate.pos = init_blh_rad_;    // ★ 弧度
    }
    if (this->get_parameter("initvel", v) && v.size()>=3) options.initstate.vel = {v[0],v[1],v[2]};
    if (this->get_parameter("initatt", v) && v.size()>=3) options.initstate.euler = {v[0]*D2R,v[1]*D2R,v[2]*D2R};

    if (this->get_parameter("initgyrbias", v) && v.size()>=3)
      options.initstate.imuerror.gyrbias = {v[0]*D2R/3600.0, v[1]*D2R/3600.0, v[2]*D2R/3600.0};
    if (this->get_parameter("initaccbias", v) && v.size()>=3)
      options.initstate.imuerror.accbias = {v[0]*1e-5, v[1]*1e-5, v[2]*1e-5};
    if (this->get_parameter("initgyrscale", v) && v.size()>=3)
      options.initstate.imuerror.gyrscale = {v[0],v[1],v[2]};
    if (this->get_parameter("initaccscale", v) && v.size()>=3)
      options.initstate.imuerror.accscale = {v[0],v[1],v[2]};
    if (this->get_parameter("antlever", v) && v.size()>=3) options.antlever = {v[0],v[1],v[2]};

    const double DEG_SQH_2_RAD_SQS = M_PI/180.0/std::sqrt(3600.0);
    const double MPS_SQH_2_MPS_SQS = 1.0/std::sqrt(3600.0);
    const double DEG_H_2_RAD_S     = M_PI/180.0/3600.0;
    const double MGAL_2_MPS2       = 1e-5;

    if (this->get_parameter("imunoise.arw", v) && v.size()>=3)
      options.imunoise.gyr_arw = {v[0]*DEG_SQH_2_RAD_SQS, v[1]*DEG_SQH_2_RAD_SQS, v[2]*DEG_SQH_2_RAD_SQS};
    if (this->get_parameter("imunoise.vrw", v) && v.size()>=3)
      options.imunoise.acc_vrw = {v[0]*MPS_SQH_2_MPS_SQS, v[1]*MPS_SQH_2_MPS_SQS, v[2]*MPS_SQH_2_MPS_SQS};
    if (this->get_parameter("imunoise.gbstd", v) && v.size()>=3)
      options.imunoise.gyrbias_std = {v[0]*DEG_H_2_RAD_S, v[1]*DEG_H_2_RAD_S, v[2]*DEG_H_2_RAD_S};
    if (this->get_parameter("imunoise.abstd", v) && v.size()>=3)
      options.imunoise.accbias_std = {v[0]*MGAL_2_MPS2, v[1]*MGAL_2_MPS2, v[2]*MGAL_2_MPS2};
    if (this->get_parameter("imunoise.gsstd", v) && v.size()>=3) options.imunoise.gyrscale_std = {v[0],v[1],v[2]};
    if (this->get_parameter("imunoise.asstd", v) && v.size()>=3) options.imunoise.accscale_std = {v[0],v[1],v[2]};
    double corr=4.0; this->get_parameter("imunoise.corrtime", corr); options.imunoise.corr_time = corr;

    // switches
    this->get_parameter("imu_is_increment", imu_is_increment_);
    this->get_parameter("imu_units", imu_units_);
    this->get_parameter("state_pos_mode", state_pos_mode_);
    this->get_parameter("z_strategy", z_strategy_);
    this->get_parameter("publish_tf", publish_tf_);
    this->get_parameter("orientation_slerp_alpha", orientation_slerp_alpha_);
    this->get_parameter("clamp_xy_max_m", clamp_xy_max_m_);
    this->get_parameter("clamp_z_band_m", clamp_z_band_m_);
    this->get_parameter("smooth_z_step_m", smooth_z_step_m_);
    this->get_parameter("path_max_len", path_max_len_);
    this->get_parameter("imu_linear_no_gravity", imu_linear_no_gravity_);

    RCLCPP_INFO(this->get_logger(),
      "Final imu_is_increment=%s, imu_units=%s | state_pos_mode=%s, z_strategy=%s, publish_tf=%s",
      imu_is_increment_?"true":"false", imu_units_.c_str(),
      state_pos_mode_.c_str(), z_strategy_.c_str(), publish_tf_?"true":"false");

    options.print_options();

    engine_ = std::make_shared<GIEngine>(options);

    // IO
    auto imu_topic = this->get_parameter("imu_topic").as_string();
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      std::bind(&KfGinsNode::imuCallback, this, _1));

    auto gps_topic = this->get_parameter("gps_topic").as_string();
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      gps_topic, 10, std::bind(&KfGinsNode::gpsCallback, this, _1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/kf_gins/odom", 10);

    rclcpp::QoS path_qos(rclcpp::KeepLast(1000));
    path_qos.transient_local().reliable();
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/kf_gins/path", path_qos);
    path_msg_.header.frame_id = "map";
    path_msg_.poses.clear();

    if (publish_tf_) tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    RCLCPP_INFO(this->get_logger(), "KF-GINS ROS2 Node initialized.");
  }

private:
  // NavState 合法性
  static bool navstateHasInvalid(const NavState &s) {
    for (int i = 0; i < 3; ++i) {
      if (!std::isfinite(s.pos[i]) || !std::isfinite(s.vel[i]) || !std::isfinite(s.euler[i])) return true;
      if (!std::isfinite(s.imuerror.gyrbias[i]) || !std::isfinite(s.imuerror.accbias[i])) return true;
      if (!std::isfinite(s.imuerror.gyrscale[i]) || !std::isfinite(s.imuerror.accscale[i])) return true;
    }
    return false;
  }

  // 将 engine 的 state.pos（依照 state_pos_mode_）统一转为 ENU(m) 相对 init_blh_rad_。
  bool toENU(const Eigen::Vector3d &state_pos, Eigen::Vector3d &enu_out) {
    if (!finite3(state_pos)) return false;

    const auto &p = state_pos;
    Eigen::Vector3d ecef_ref; blhToEcef(init_blh_rad_[0], init_blh_rad_[1], init_blh_rad_[2], ecef_ref);
    Eigen::Matrix3d R = ecefToEnuRot(init_blh_rad_[0], init_blh_rad_[1]);

    if (state_pos_mode_ == "enu") {
      enu_out = p;
      return finite3(enu_out);
    } else if (state_pos_mode_ == "ecef") {
      enu_out = R * (p - ecef_ref);
      return finite3(enu_out);
    } else if (state_pos_mode_ == "blh_rad") {
      Eigen::Vector3d ecef; blhToEcef(p[0], p[1], p[2], ecef);
      enu_out = R * (ecef - ecef_ref);
      return finite3(enu_out);
    } else if (state_pos_mode_ == "blh_deg") {
      Eigen::Vector3d ecef; blhToEcef(p[0]*D2R, p[1]*D2R, p[2], ecef);
      enu_out = R * (ecef - ecef_ref);
      return finite3(enu_out);
    } else if (state_pos_mode_ == "auto") {
      double norm = p.norm();
      if (norm > 3e6 && norm < 1.2e7) enu_out = R * (p - ecef_ref); // ECEF 量级
      else enu_out = p;                                              // 当 ENU
      return finite3(enu_out);
    }
    return false;
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    IMU imu_data;
    const double t = double(msg->header.stamp.sec) + double(msg->header.stamp.nanosec)*1e-9;
    imu_data.time = t;

    double dt;
    if (last_imu_time_ < 0.0) dt = 1.0 / std::max(1, imudatarate_);
    else {
      dt = t - last_imu_time_;
      if (!(dt > 0.0) || !std::isfinite(dt)) dt = 1.0 / std::max(1, imudatarate_);
      if (dt < 1e-6) dt = 1e-6;
      if (dt > 0.2)  dt = 0.2;
    }
    imu_data.dt = dt; last_imu_time_ = t;

    double wx = msg->angular_velocity.x;
    double wy = msg->angular_velocity.y;
    double wz = msg->angular_velocity.z;
    double ax = msg->linear_acceleration.x;
    double ay = msg->linear_acceleration.y;
    double az = msg->linear_acceleration.z;

    if (!std::isfinite(wx) || !std::isfinite(wy) || !std::isfinite(wz) ||
        !std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(az)) return;

    if (imu_units_ == "deg_g") {
      const double DEG2RAD = M_PI/180.0;
      const double G2MS2   = 9.81;
      wx*=DEG2RAD; wy*=DEG2RAD; wz*=DEG2RAD;
      ax*=G2MS2;   ay*=G2MS2;   az*=G2MS2;
    }

    // 去重力的 IMU：补成“比力”f = a_b - C_nb*g
    if (imu_linear_no_gravity_) {
      const NavState s_est = engine_->getNavState();   // euler 在 nav-frame
      Eigen::AngleAxisd rx(s_est.euler(0), Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd ry(s_est.euler(1), Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd rz(s_est.euler(2), Eigen::Vector3d::UnitZ());
      Eigen::Matrix3d C_bn = (rz * ry * rx).toRotationMatrix();
      Eigen::Matrix3d C_nb = C_bn.transpose();
      const Eigen::Vector3d g_n(0.0, 0.0, -9.80665);   // ENU: -Z 向下
      const Eigen::Vector3d g_b = C_nb * g_n;
      ax = ax - g_b.x();
      ay = ay - g_b.y();
      az = az - g_b.z();
    }

    if (imu_is_increment_) {
      imu_data.dtheta << wx, wy, wz;           // 已是增量
      imu_data.dvel   << ax, ay, az;
    } else {
      imu_data.dtheta << wx*dt, wy*dt, wz*dt;  // 由“率/加速度”转增量
      imu_data.dvel   << ax*dt, ay*dt, az*dt;
    }
    if (!finite3(imu_data.dtheta) || !finite3(imu_data.dvel)) return;

    imu_data.odovel = 0.0;
    engine_->addImuData(imu_data, true);
    engine_->newImuProcess();

    publishState();
  }

  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    GNSS g;
    g.time = double(msg->header.stamp.sec) + double(msg->header.stamp.nanosec)*1e-9;

    // GIEngine 侧按 BLH(rad) 处理
    g.blh  << msg->latitude * D2R,
              msg->longitude * D2R,
              msg->altitude;

    g.std  << 3.0, 3.0, 5.0;  // 保守 std，避免一上来跳变过大
    g.isvalid = std::isfinite(g.blh[0]) && std::isfinite(g.blh[1]) && std::isfinite(g.blh[2]);

    engine_->addGnssData(g);
    publishState();
  }

  void publishState() {
    const NavState s = engine_->getNavState();
    if (navstateHasInvalid(s)) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "NavState contains NaN/Inf, skip publish.");
      return;
    }

    const rclcpp::Time now = this->get_clock()->now();
    const uint64_t ns = now.nanoseconds();
    const int32_t sec = static_cast<int32_t>(ns / 1000000000ULL);
    const uint32_t nsec = static_cast<uint32_t>(ns % 1000000000ULL);

    // euler->quat with smoothing
    Eigen::AngleAxisd rx(s.euler(0), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd ry(s.euler(1), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rz(s.euler(2), Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = rz*ry*rx; q.normalize();

    if (have_last_quat_) {
      if (q.dot(last_quat_) < 0.0) q.coeffs() *= -1.0;
      q = last_quat_.slerp(orientation_slerp_alpha_, q).normalized();
    }
    last_quat_ = q; have_last_quat_ = true;

    // pos -> ENU(m)
    Eigen::Vector3d enu;
    if (!toENU(s.pos, enu)) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "toENU failed, skip.");
      return;
    }

    // XY clamp
    if (!finite3(enu) || std::abs(enu.x()) > clamp_xy_max_m_ || std::abs(enu.y()) > clamp_xy_max_m_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "ENU XY out of range (%.3f,%.3f), clamp to last or origin.", enu.x(), enu.y());
      if (have_last_valid_) enu = last_valid_enu_;
      else enu = Eigen::Vector3d(0,0,init_blh_rad_.z());
    }

    // Z strategy
    const double init_alt = init_blh_rad_.z();
    if (z_strategy_ == "zero")      enu.z() = 0.0;
    else if (z_strategy_ == "init_alt") enu.z() = init_alt;
    else { // raw_clamped
      if (have_last_valid_) {
        const double dz = enu.z() - last_valid_enu_.z();
        if (std::abs(dz) > smooth_z_step_m_) {
          enu.z() = last_valid_enu_.z() + std::copysign(smooth_z_step_m_, dz);
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Z smoothing applied.");
        }
      }
      const double dz0 = enu.z() - init_alt;
      if (!std::isfinite(enu.z()) || std::abs(dz0) > clamp_z_band_m_) {
        enu.z() = init_alt + std::copysign(clamp_z_band_m_, dz0);
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Z hard-clamped.");
      }
    }

    last_valid_enu_ = enu; have_last_valid_ = true;

    // Publish Odom
    nav_msgs::msg::Odometry odom;
    odom.header.stamp.sec = sec; odom.header.stamp.nanosec = nsec;
    odom.header.frame_id = "map";
    odom.child_frame_id  = "base_link";
    odom.pose.pose.position.x = enu.x();
    odom.pose.pose.position.y = enu.y();
    odom.pose.pose.position.z = enu.z();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom_pub_->publish(odom);

    // optional TF
    if (publish_tf_ && tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp.sec = sec; t.header.stamp.nanosec = nsec;
      t.header.frame_id = "map"; t.child_frame_id = "base_link";
      t.transform.translation.x = enu.x();
      t.transform.translation.y = enu.y();
      t.transform.translation.z = enu.z();
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();
      tf_broadcaster_->sendTransform(t);
    }

    // path
    if (path_pub_) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = "map";
      ps.header.stamp.sec = sec; ps.header.stamp.nanosec = nsec;
      ps.pose = odom.pose.pose;

      bool push = path_msg_.poses.empty();
      if (!push) {
        const auto &last = path_msg_.poses.back().pose.position;
        const double dx = ps.pose.position.x - last.x;
        const double dy = ps.pose.position.y - last.y;
        const double dxy = std::sqrt(dx*dx + dy*dy);
        if (dxy >= 0.02) push = true;
      }
      if (push) {
        if (!path_msg_.poses.empty()) {
          auto &lastp = path_msg_.poses.back().pose.position;
          const double dz = ps.pose.position.z - lastp.z;
          const double MAX_PATH_Z_JUMP = 100.0;
          if (std::abs(dz) > MAX_PATH_Z_JUMP) {
            ps.pose.position.z = lastp.z + std::copysign(MAX_PATH_Z_JUMP, dz);
          }
        }
        path_msg_.poses.push_back(ps);
        if ((int)path_msg_.poses.size() > path_max_len_) {
          const size_t cut = path_msg_.poses.size() - path_max_len_;
          path_msg_.poses.erase(path_msg_.poses.begin(), path_msg_.poses.begin()+cut);
        }
        path_msg_.header.stamp.sec = sec; path_msg_.header.stamp.nanosec = nsec;
        path_pub_->publish(path_msg_);
      }
    }
  }

private:
  std::shared_ptr<GIEngine> engine_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  nav_msgs::msg::Path path_msg_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  double last_imu_time_;
  int    imudatarate_{200};

  bool   imu_is_increment_{true};
  std::string imu_units_{"si"};
  std::string state_pos_mode_{"blh_rad"};
  std::string z_strategy_{"init_alt"};
  bool   publish_tf_{true};
  bool   imu_linear_no_gravity_{true};

  double orientation_slerp_alpha_{0.2};
  double clamp_xy_max_m_{5e5};
  double clamp_z_band_m_{2000.0};
  double smooth_z_step_m_{50.0};
  int    path_max_len_{50000};

  Eigen::Vector3d init_blh_rad_{0,0,0}; // [lat,lon,h] rad,m
  Eigen::Vector3d last_valid_enu_{0,0,0};
  bool have_last_valid_{false};

  Eigen::Quaterniond last_quat_{1,0,0,0};
  bool have_last_quat_{false};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KfGinsNode>());
  rclcpp::shutdown();
  return 0;
}
