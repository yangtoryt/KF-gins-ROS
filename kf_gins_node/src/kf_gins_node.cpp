// src/kf_gins_node.cpp (robust version + microdeg/1e7-deg detection + TF switch)
//
// 关键改动：
// 1) 判别 state.pos 时新增 llh_microdeg(度×1e6) & llh_deg1e7(度×1e7) → 统一转 ENU(m)
// 2) 增加 publish_tf 参数（默认 true），方便调试时关闭 TF
// 3) 更清晰的日志：detected=llh_microdeg/llh_deg1e7 等

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

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/parameter_events_filter.hpp"

// TF
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using std::placeholders::_1;

static inline bool finite3(const Eigen::Vector3d &v) {
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

// BLH(rad) -> ECEF(m)
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

// ECEF->ENU 的旋转（rad 入）
static Eigen::Matrix3d ecefToEnuRot(double lat0, double lon0) {
  const double sLat = std::sin(lat0), cLat = std::cos(lat0);
  const double sLon = std::sin(lon0), cLon = std::cos(lon0);
  Eigen::Matrix3d R;
  R << -sLon,         cLon,         0,
       -cLon*sLat, -sLon*sLat,  cLat,
        cLon*cLat,  sLon*cLat,  sLat;
  return R;
}

class KfGinsNode : public rclcpp::Node {
public:
  KfGinsNode()
  : Node("kf_gins_ros2_node"), last_imu_time_(-1.0), have_last_quat_(false) {
    // ------------ 参数声明（ YAML 对齐） ------------
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

    this->declare_parameter<std::string>("imu_topic", "/imu/data");
    this->declare_parameter<std::string>("gps_topic", "/gps/fix");

    this->declare_parameter<std::vector<double>>("antlever", {0.0, 0.0, 0.0});

    // 新增：IMU 话题是否为增量、IMU 单位
    this->declare_parameter<bool>("imu_is_increment", true); // 默认按 ROS 标准：速率/加速度
    this->declare_parameter<std::string>("imu_units", "si");  // "si" 或 "deg_g"

    // 可调：四元数平滑强度
    this->declare_parameter<double>("orientation_slerp_alpha", 0.2);

    // 新增：是否发布 TF（调试时可关）
    this->declare_parameter<bool>("publish_tf", true);

    std::string pkg_path = ament_index_cpp::get_package_share_directory("kf_gins_node");
    std::string imu_path, gps_path;
    this->get_parameter("imupath", imu_path);
    this->get_parameter("gnsspath", gps_path);
    std::string full_imu_path = pkg_path + "/" + imu_path;
    std::string full_gps_path = pkg_path + "/" + gps_path;

    // ------------ 读参数填 options ------------
    GINSOptions options;
    std::vector<double> v;

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
      // store initpos in radians internally (lat, lon in rad)
      options.initstate.pos[0] = v[0] * D2R;
      options.initstate.pos[1] = v[1] * D2R;
      options.initstate.pos[2] = v[2];
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to load initpos");
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
    const double DEG_PER_SQRT_HOUR_TO_RAD_PER_SQRT_SEC = M_PI / 180.0 / std::sqrt(3600.0);
    if (this->get_parameter("imunoise.arw", v) && v.size() >= 3)
      options.imunoise.gyr_arw = Eigen::Vector3d(
        v[0] * DEG_PER_SQRT_HOUR_TO_RAD_PER_SQRT_SEC,
        v[1] * DEG_PER_SQRT_HOUR_TO_RAD_PER_SQRT_SEC,
        v[2] * DEG_PER_SQRT_HOUR_TO_RAD_PER_SQRT_SEC
      );
    const double MPS_PER_SQRT_HOUR_TO_MPS_PER_SQRT_SEC = 1.0 / std::sqrt(3600.0);
    if (this->get_parameter("imunoise.vrw", v) && v.size() >= 3)
      options.imunoise.acc_vrw = Eigen::Vector3d(
        v[0] * MPS_PER_SQRT_HOUR_TO_MPS_PER_SQRT_SEC,
        v[1] * MPS_PER_SQRT_HOUR_TO_MPS_PER_SQRT_SEC,
        v[2] * MPS_PER_SQRT_HOUR_TO_MPS_PER_SQRT_SEC
      );
    const double DEG_PER_HOUR_TO_RAD_PER_SEC = M_PI / 180.0 / 3600.0;
    if (this->get_parameter("imunoise.gbstd", v) && v.size() >= 3)
      options.imunoise.gyrbias_std = Eigen::Vector3d(
        v[0] * DEG_PER_HOUR_TO_RAD_PER_SEC,
        v[1] * DEG_PER_HOUR_TO_RAD_PER_SEC,
        v[2] * DEG_PER_HOUR_TO_RAD_PER_SEC
      );
    const double MILLIGAL_TO_MPS2 = 1e-5;
    if (this->get_parameter("imunoise.abstd", v) && v.size() >= 3)
      options.imunoise.accbias_std = Eigen::Vector3d(
        v[0] * MILLIGAL_TO_MPS2,
        v[1] * MILLIGAL_TO_MPS2,
        v[2] * MILLIGAL_TO_MPS2
      );

    if (this->get_parameter("imunoise.gsstd", v) && v.size() >= 3)
      options.imunoise.gyrscale_std = Eigen::Vector3d(v[0], v[1], v[2]);
    if (this->get_parameter("imunoise.asstd", v) && v.size() >= 3)
      options.imunoise.accscale_std = Eigen::Vector3d(v[0], v[1], v[2]);

    double corr = 4.0;
    this->get_parameter("imunoise.corrtime", corr);
    options.imunoise.corr_time = corr;

    if (imudatarate <= 0 || imudatarate > 2000) {
      RCLCPP_ERROR(this->get_logger(), "imudatarate must be 1~2000");
      rclcpp::shutdown();
    }

    if (this->get_parameter("initpos", v) && v.size() >= 2) {
      if (v[0] < -90 || v[0] > 90 || v[1] < -180 || v[1] > 180) {
        RCLCPP_ERROR(this->get_logger(), "initpos (lat/lon) out of range");
        rclcpp::shutdown();
      }
    }

    auto check_positive = [this](const std::vector<double> &v, const std::string &name) {
      for (double val : v) {
        if (val <= 0) {
          RCLCPP_ERROR(this->get_logger(), "%s must be positive", name.c_str());
          rclcpp::shutdown();
        }
      }
    };
    if (this->get_parameter("imunoise.arw", v)) check_positive(v, "imunoise.arw");

    this->get_parameter("imu_is_increment", imu_is_increment_);
    this->get_parameter("imu_units", imu_units_);
    this->get_parameter("orientation_slerp_alpha", orientation_slerp_alpha_);
    this->get_parameter("publish_tf", publish_tf_);

    RCLCPP_INFO(this->get_logger(), "Loaded imu_is_increment from config: %s",
                imu_is_increment_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Final imu_is_increment = %s, imu_units = %s, slerp_alpha=%.3f",
                imu_is_increment_ ? "true" : "false", imu_units_.c_str(), orientation_slerp_alpha_);
    RCLCPP_INFO(this->get_logger(), "publish_tf = %s", publish_tf_ ? "true" : "false");

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

    RCLCPP_INFO(this->get_logger(), "Sanitized initstate pos=%g %g %g", options.initstate.pos[0], options.initstate.pos[1], options.initstate.pos[2]);
    RCLCPP_INFO(this->get_logger(), "Sanitized initstate vel=%g %g %g", options.initstate.vel[0], options.initstate.vel[1], options.initstate.vel[2]);
    RCLCPP_INFO(this->get_logger(), "Sanitized initstate euler=%g %g %g (rad)", options.initstate.euler[0], options.initstate.euler[1], options.initstate.euler[2]);
    RCLCPP_INFO(this->get_logger(), "Sanitized imuerror gyrbias=%g %g %g", options.initstate.imuerror.gyrbias[0], options.initstate.imuerror.gyrbias[1], options.initstate.imuerror.gyrbias[2]);

    // Print options
    options.print_options();

    // ------------ 创建引擎 ------------
    engine_ = std::make_shared<GIEngine>(options);

    // ------------ 订阅与发布 ------------
    std::string imu_topic = this->get_parameter("imu_topic").as_string();
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      std::bind(&KfGinsNode::imuCallback, this, _1)
    );

    std::string gps_topic = this->get_parameter("gps_topic").as_string();
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      gps_topic, 10, std::bind(&KfGinsNode::gpsCallback, this, _1)
    );

    auto param_callback = [this](const std::vector<rclcpp::Parameter> &params) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      result.reason = "";
      return result;
    };
    param_sub_ = this->add_on_set_parameters_callback(param_callback);

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/kf_gins/odom", 10);

    // Path
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/kf_gins/path", 10);
    path_msg_.header.frame_id = "map";
    path_msg_.poses.clear();

    // store initial BLH (radians)
    init_blh_rad_ = options.initstate.pos;
    have_last_valid_ = false;

    // TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

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
    const double cur_time = double(msg->header.stamp.sec) + double(msg->header.stamp.nanosec) * 1e-9;
    imu_data.time = cur_time;

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
      if (dt > 0.2) dt = 0.2;
    }
    imu_data.dt = dt;
    last_imu_time_ = cur_time;

    double wx = msg->angular_velocity.x;
    double wy = msg->angular_velocity.y;
    double wz = msg->angular_velocity.z;
    double ax = msg->linear_acceleration.x;
    double ay = msg->linear_acceleration.y;
    double az = msg->linear_acceleration.z;

    if (!std::isfinite(wx) || !std::isfinite(wy) || !std::isfinite(wz) ||
        !std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(az)) {
      RCLCPP_WARN(this->get_logger(), "IMU ROS message contains non-finite values, dropping sample. stamp=%.9f", cur_time);
      return;
    }

    if (imu_units_ == "deg_g") {
      const double DEG2RAD = M_PI / 180.0;
      const double G2MS2   = 9.81;
      wx *= DEG2RAD; wy *= DEG2RAD; wz *= DEG2RAD;
      ax *= G2MS2;   ay *= G2MS2;   az *= G2MS2;
    }

    if (imu_is_increment_) {
      imu_data.dtheta << wx, wy, wz;
      imu_data.dvel   << ax, ay, az;
    } else {
      imu_data.dtheta << wx * dt, wy * dt, wz * dt;
      imu_data.dvel   << ax * dt, ay * dt, az * dt;
    }

    if (!finite3(imu_data.dtheta) || !finite3(imu_data.dvel)) {
      RCLCPP_ERROR(this->get_logger(), "Computed dtheta/dvel contain non-finite values, drop sample. stamp=%.9f dt=%g", cur_time, dt);
      RCLCPP_DEBUG(this->get_logger(), "raw wx,wy,wz = %g %g %g; ax,ay,az = %g %g %g", wx, wy, wz, ax, ay, az);
      return;
    }

    RCLCPP_DEBUG(this->get_logger(), "IMU sample stamp=%.6f dt=%.6g dtheta=[%.8g, %.8g, %.8g] dvel=[%.8g, %.8g, %.8g]",
                 imu_data.time, imu_data.dt,
                 imu_data.dtheta.x(), imu_data.dtheta.y(), imu_data.dtheta.z(),
                 imu_data.dvel.x(), imu_data.dvel.y(), imu_data.dvel.z());

    imu_data.odovel = 0.0;

    if (!engine_) {
      RCLCPP_ERROR(this->get_logger(), "engine_ is null, cannot process IMU");
      return;
    }

    engine_->addImuData(imu_data, true);
    engine_->newImuProcess();

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
      RCLCPP_ERROR(this->get_logger(), "NavState contains NaN/Inf! Skipping publish.");
      RCLCPP_ERROR(this->get_logger(), "last_imu_time=%.6f", last_imu_time_);
      return;
    }

    // build timestamp once
    rclcpp::Time now_rcl = this->get_clock()->now();
    uint64_t now_ns = now_rcl.nanoseconds();
    int32_t sec = static_cast<int32_t>(now_ns / 1000000000ULL);
    uint32_t nanosec = static_cast<uint32_t>(now_ns % 1000000000ULL);

    // === 统一把 state.pos 转 ENU(m) ===
    Eigen::Vector3d pub_pos = state.pos; // default assume ENU(m)
    bool converted = false;
    std::string detected = "unknown";

    RCLCPP_DEBUG(this->get_logger(), "Raw state.pos = [%.12g, %.12g, %.12g]", state.pos[0], state.pos[1], state.pos[2]);

    if (std::isfinite(state.pos[0]) && std::isfinite(state.pos[1]) && std::isfinite(state.pos[2])) {
      const double ax = std::abs(state.pos[0]);
      const double ay = std::abs(state.pos[1]);
      const double az = std::abs(state.pos[2]);

      // 1) ENU(m) 范围
      if (ax < 1e5 && ay < 1e5 && az < 2e4) {
        converted = true; detected = "enu"; pub_pos = state.pos;
      } else {
        // 2) BLH(rad)
        if (ax <= 1.2 && ay <= 1.2 && az < 1e5) {
          Eigen::Vector3d ecef, ecef_ref;
          blhToEcef(state.pos[0], state.pos[1], state.pos[2], ecef);
          blhToEcef(init_blh_rad_[0], init_blh_rad_[1], init_blh_rad_[2], ecef_ref);
          Eigen::Matrix3d R = ecefToEnuRot(init_blh_rad_[0], init_blh_rad_[1]);
          pub_pos = R * (ecef - ecef_ref);
          converted = true; detected = "blh_rad";
        }
        // 3) BLH(deg)
        else if (ax <= 180.0 && ay <= 180.0 && az < 1e5) {
          const double lat_rad = state.pos[0] * M_PI / 180.0;
          const double lon_rad = state.pos[1] * M_PI / 180.0;
          Eigen::Vector3d ecef, ecef_ref;
          blhToEcef(lat_rad, lon_rad, state.pos[2], ecef);
          blhToEcef(init_blh_rad_[0], init_blh_rad_[1], init_blh_rad_[2], ecef_ref);
          Eigen::Matrix3d R = ecefToEnuRot(init_blh_rad_[0], init_blh_rad_[1]);
          pub_pos = R * (ecef - ecef_ref);
          converted = true; detected = "blh_deg";
        }
        // 4) BLH(度×1e6) micro-deg
        else if (ax <= 90.0e6 && ay <= 180.0e6 && az < 1e8) {
          const double lat_deg = state.pos[0] / 1e6;
          const double lon_deg = state.pos[1] / 1e6;
          const double h_m     = state.pos[2]; // 高度通常已是米；若是毫米/厘米可按需再加判断
          const double lat_rad = lat_deg * M_PI / 180.0;
          const double lon_rad = lon_deg * M_PI / 180.0;
          Eigen::Vector3d ecef, ecef_ref;
          blhToEcef(lat_rad, lon_rad, h_m, ecef);
          blhToEcef(init_blh_rad_[0], init_blh_rad_[1], init_blh_rad_[2], ecef_ref);
          Eigen::Matrix3d R = ecefToEnuRot(init_blh_rad_[0], init_blh_rad_[1]);
          pub_pos = R * (ecef - ecef_ref);
          converted = true; detected = "blh_microdeg(1e6)";
        }
        // 5) BLH(度×1e7)
        else if (ax <= 90.0e7 && ay <= 180.0e7 && az < 1e9) {
          const double lat_deg = state.pos[0] / 1e7;
          const double lon_deg = state.pos[1] / 1e7;
          const double h_m     = state.pos[2];
          const double lat_rad = lat_deg * M_PI / 180.0;
          const double lon_rad = lon_deg * M_PI / 180.0;
          Eigen::Vector3d ecef, ecef_ref;
          blhToEcef(lat_rad, lon_rad, h_m, ecef);
          blhToEcef(init_blh_rad_[0], init_blh_rad_[1], init_blh_rad_[2], ecef_ref);
          Eigen::Matrix3d R = ecefToEnuRot(init_blh_rad_[0], init_blh_rad_[1]);
          pub_pos = R * (ecef - ecef_ref);
          converted = true; detected = "blh_deg1e7";
        }
        // 6) ECEF(m)
        else {
          const double norm = state.pos.norm();
          if ((norm > 3.0e6 && norm < 1.2e7) ||
              ax > 1e6 || ay > 1e6 || az > 1e6) {
            Eigen::Vector3d ecef_ref;
            blhToEcef(init_blh_rad_[0], init_blh_rad_[1], init_blh_rad_[2], ecef_ref);
            Eigen::Matrix3d R = ecefToEnuRot(init_blh_rad_[0], init_blh_rad_[1]);
            pub_pos = R * (state.pos - ecef_ref);
            converted = true; detected = "ecef";
          } else {
            detected = "unrecognized";
          }
        }
      }
    }

    // SAFETY/GATING：限制异常 ENU 值
    bool used_fallback = false;
    const double MAX_ACCEPT = 5e5; // 500 km guard
    const double MAX_ALT_DELTA = 2000.0; // +/- 2000 m
    const double SMOOTH_Z_DELTA = 50.0;
    const double MAX_MATCH_Z_DIFF = 100.0;

    const double init_alt = init_blh_rad_.z();
    bool publish_raw = true;

    RCLCPP_DEBUG(this->get_logger(), "Detected pos format=%s converted=%d pub_pos(before clamp)=[%.6f, %.6f, %.6f] init_alt=%.3f",
                 detected.c_str(), converted?1:0, pub_pos.x(), pub_pos.y(), pub_pos.z(), init_alt);

    if (!converted) {
      publish_raw = false;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "publishState: not converted (detected=%s) -> fallback.", detected.c_str());
    } else {
      if (!finite3(pub_pos) ||
          std::abs(pub_pos.x()) > MAX_ACCEPT || std::abs(pub_pos.y()) > MAX_ACCEPT ||
          std::abs(pub_pos.z() - init_alt) > MAX_ALT_DELTA) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "publishState: converted pub_pos out of bounds: [%.3f,%.3f,%.3f], init_alt=%.3f (detected=%s)",
          pub_pos.x(), pub_pos.y(), pub_pos.z(), init_alt, detected.c_str());
        publish_raw = true; // 仍然发布，但 z 会被强力钳制
      }
    }

    // 选参考 z
    double reference_z = init_alt;
    if (!path_msg_.poses.empty()) {
      reference_z = path_msg_.poses.back().pose.position.z;
    } else if (have_last_valid_) {
      reference_z = last_valid_enu_.z();
    }

    if (!publish_raw) {
      if (have_last_valid_) {
        pub_pos = last_valid_enu_;
        used_fallback = true;
      } else {
        pub_pos = Eigen::Vector3d(0.0, 0.0, init_alt);
        used_fallback = true;
      }
    } else {
      if (have_last_valid_) {
        const double last_z = last_valid_enu_.z();
        const double dz = pub_pos.z() - last_z;
        if (std::abs(dz) > SMOOTH_Z_DELTA) {
          pub_pos.z() = last_z + (dz > 0.0 ? SMOOTH_Z_DELTA : -SMOOTH_Z_DELTA);
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Smoothing large z jump.");
        }
      }
      if (!path_msg_.poses.empty()) {
        const double dzp = pub_pos.z() - reference_z;
        if (std::abs(dzp) > MAX_MATCH_Z_DIFF) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Enforcing path-based z clamp.");
          pub_pos.z() = reference_z;
        }
      }
      if (!std::isfinite(pub_pos.z()) || std::abs(pub_pos.z()) > 1e4 || std::abs(pub_pos.z() - init_alt) > MAX_ALT_DELTA) {
        pub_pos.z() = reference_z; // 优先跟随 path，否则 init_alt
      }
      last_valid_enu_ = pub_pos;
      have_last_valid_ = true;
    }

    // ============ 姿态 ============
    Eigen::AngleAxisd rx(state.euler(0), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd ry(state.euler(1), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rz(state.euler(2), Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = rz * ry * rx;
    if (q.norm() <= 0.0) q = Eigen::Quaterniond::Identity();
    else q.normalize();

    // 额外异常检查
    bool suppress_tf_and_path = false;
    if (!finite3(pub_pos) ||
        std::abs(pub_pos.x()) > MAX_ACCEPT || std::abs(pub_pos.y()) > MAX_ACCEPT) {
      if (have_last_valid_) pub_pos = last_valid_enu_;
      else pub_pos = Eigen::Vector3d(0.0, 0.0, init_alt);
      used_fallback = true;
      suppress_tf_and_path = true;
      RCLCPP_ERROR(this->get_logger(), "publishState: ENU XY out-of-bounds, suppress TF/Path for this tick.");
    }

    // ============ 发布 Odom ============
    nav_msgs::msg::Odometry odom;
    odom.header.stamp.sec = sec;
    odom.header.stamp.nanosec = nanosec;
    odom.header.frame_id = "map";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = pub_pos.x();
    odom.pose.pose.position.y = pub_pos.y();
    odom.pose.pose.position.z = pub_pos.z();

    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    if (odom_pub_) odom_pub_->publish(odom);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Published odom pos=%.3f %.3f %.3f quat=%.6f %.6f %.6f %.6f (detected=%s converted=%d fallback=%d suppress_tf_path=%d)",
      pub_pos.x(), pub_pos.y(), pub_pos.z(), q.x(), q.y(), q.z(), q.w(),
      detected.c_str(), converted?1:0, used_fallback?1:0, suppress_tf_and_path?1:0);

    // ============ 发布 TF（可关闭） ============
    if (publish_tf_ && !suppress_tf_and_path && tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp.sec = sec; t.header.stamp.nanosec = nanosec;
      t.header.frame_id = "map"; t.child_frame_id = "base_link";
      t.transform.translation.x = pub_pos.x();
      t.transform.translation.y = pub_pos.y();
      t.transform.translation.z = pub_pos.z();
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();
      tf_broadcaster_->sendTransform(t);
    } else if (!publish_tf_) {
      RCLCPP_DEBUG(this->get_logger(), "publish_tf=false, TF not sent.");
    } else if (suppress_tf_and_path) {
      RCLCPP_DEBUG(this->get_logger(), "TF suppressed for abnormal state.");
    }

    // ============ Path ============
    if (path_pub_ && !used_fallback && !suppress_tf_and_path) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = "map";
      ps.header.stamp.sec = sec; ps.header.stamp.nanosec = nanosec;
      ps.pose = odom.pose.pose;

      const double min_dist_xy = 0.02; // meters
      const double min_dt   = 0.02; // seconds
      bool push = false;
      if (path_msg_.poses.empty()) push = true;
      else {
        auto &last = path_msg_.poses.back();
        const double dx = ps.pose.position.x - last.pose.position.x;
        const double dy = ps.pose.position.y - last.pose.position.y;
        const double dist_xy = std::sqrt(dx*dx + dy*dy);
        const uint64_t prev_ns = uint64_t(last.header.stamp.sec) * 1000000000ULL + uint64_t(last.header.stamp.nanosec);
        const uint64_t cur_ns  = uint64_t(ps.header.stamp.sec) * 1000000000ULL + uint64_t(ps.header.stamp.nanosec);
        const double dt = (cur_ns > prev_ns) ? double(cur_ns - prev_ns) * 1e-9 : 0.0;
        if (dist_xy >= min_dist_xy || dt >= min_dt) push = true;
      }
      if (push) {
        if (!path_msg_.poses.empty()) {
          const double last_z = path_msg_.poses.back().pose.position.z;
          const double MAX_PATH_Z_JUMP = 100.0;
          const double dz = ps.pose.position.z - last_z;
          if (std::abs(dz) > MAX_PATH_Z_JUMP) {
            ps.pose.position.z = last_z + (dz > 0.0 ? MAX_PATH_Z_JUMP : -MAX_PATH_Z_JUMP);
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
              "Clamping path point z jump.");
          }
        }
        path_msg_.poses.push_back(ps);
        const size_t maxlen = 50000;
        if (path_msg_.poses.size() > maxlen) {
          path_msg_.poses.erase(path_msg_.poses.begin(), path_msg_.poses.begin() + (path_msg_.poses.size() - maxlen));
        }
        path_msg_.header.stamp.sec = sec; path_msg_.header.stamp.nanosec = nanosec;
        path_pub_->publish(path_msg_);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "Published path size=%zu last=(%.3f,%.3f,%.3f)",
          path_msg_.poses.size(),
          path_msg_.poses.back().pose.position.x,
          path_msg_.poses.back().pose.position.y,
          path_msg_.poses.back().pose.position.z);
      }
    } else {
      if (suppress_tf_and_path) {
        RCLCPP_DEBUG(this->get_logger(), "Skipping path append due to suppressed publishing.");
      } else if (used_fallback) {
        RCLCPP_DEBUG(this->get_logger(), "Skipping path append because fallback was used.");
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
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_sub_;

  double last_imu_time_;
  bool   imu_is_increment_{false};
  int    imudatarate_{200};
  std::string imu_units_{"si"};

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  bool publish_tf_{true};

  // conversion helpers / fallback
  Eigen::Vector3d init_blh_rad_{0,0,0}; // 初始经纬高（rad,rad,m）
  Eigen::Vector3d last_valid_enu_{0,0,0};
  bool have_last_valid_{false};

  // orientation smoothing / sign alignment
  Eigen::Quaterniond last_published_quat_;
  bool have_last_quat_;
  double orientation_slerp_alpha_{0.2};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KfGinsNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
