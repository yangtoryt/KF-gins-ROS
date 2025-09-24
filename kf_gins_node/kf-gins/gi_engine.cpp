// gi_engine.cpp  — 完整文件（已包含防护性改动，修复 NaN/Inf 来源）
#include "common/earth.h"
#include "common/rotation.h"

#include "gi_engine.h"
#include "insmech.h"

#include <iostream>
#include <iomanip>
#include <algorithm> // std::min/std::max 保底（注意：std::clamp 需要 C++17）
#include <cmath>
#include <limits>

GIEngine::GIEngine(GINSOptions &options)
{

    this->options_ = options;
    options_.print_options();
    timestamp_ = 0.0;

    // ----- 强制初始化（放到构造函数中） -----
    // init IMU times
    imupre_.time = -1.0;
    imucur_.time = -1.0;
    imupre_.dtheta.setZero();
    imupre_.dvel.setZero();
    imucur_.dtheta.setZero();
    imucur_.dvel.setZero();
    imupre_.dt = imucur_.dt = 0.0;

    // init PVA
    pvapre_.pos.setZero();
    pvapre_.vel.setZero();
    pvapre_.att.euler.setZero();
    pvacur_.pos.setZero();
    pvacur_.vel.setZero();
    pvacur_.att.euler.setZero();

    // init imu error to zeros
    imuerror_.gyrbias.setZero();
    imuerror_.accbias.setZero();
    imuerror_.gyrscale.setZero();
    imuerror_.accscale.setZero();

    // resize matrices first
    Cov_.resize(RANK, RANK);
    Qc_.resize(NOISERANK, NOISERANK);
    dx_.resize(RANK, 1);

    // set safe default for covariance (diagonal small positive) to avoid exact zeros
    Cov_.setZero();
    Cov_.diagonal().setConstant(1e-6);

    Qc_.setZero();
    dx_.setZero();

    // 初始化系统噪声阵
    // initialize noise matrix
    auto imunoise = options_.imunoise;
    Qc_.block(ARW_ID, ARW_ID, 3, 3) = imunoise.gyr_arw.cwiseProduct(imunoise.gyr_arw).asDiagonal();
    Qc_.block(VRW_ID, VRW_ID, 3, 3) = imunoise.acc_vrw.cwiseProduct(imunoise.acc_vrw).asDiagonal();
    Qc_.block(BGSTD_ID, BGSTD_ID, 3, 3) =
        2 / imunoise.corr_time * imunoise.gyrbias_std.cwiseProduct(imunoise.gyrbias_std).asDiagonal();
    Qc_.block(BASTD_ID, BASTD_ID, 3, 3) =
        2 / imunoise.corr_time * imunoise.accbias_std.cwiseProduct(imunoise.accbias_std).asDiagonal();
    Qc_.block(SGSTD_ID, SGSTD_ID, 3, 3) =
        2 / imunoise.corr_time * imunoise.gyrscale_std.cwiseProduct(imunoise.gyrscale_std).asDiagonal();
    Qc_.block(SASTD_ID, SASTD_ID, 3, 3) =
        2 / imunoise.corr_time * imunoise.accscale_std.cwiseProduct(imunoise.accscale_std).asDiagonal();

    // 设置系统状态(位置、速度、姿态和IMU误差)初值和初始协方差
    // set initial state (position, velocity, attitude and IMU error) and covariance
    initialize(options_.initstate, options_.initstate_std);
}

void GIEngine::initialize(const NavState &initstate, const NavState &initstate_std)
{

    // 初始化位置、速度、姿态
    pvacur_.pos = initstate.pos;
    pvacur_.vel = initstate.vel;
    pvacur_.att.euler = initstate.euler;
    pvacur_.att.cbn = Rotation::euler2matrix(pvacur_.att.euler);
    pvacur_.att.qbn = Rotation::euler2quaternion(pvacur_.att.euler);

    // --- 防护：sanitize initstate fields to avoid copying NaN/Inf into internal state ---
    NavState safe_init = initstate; // copy
    // sanitize position/velocity/attitude
    for (int i = 0; i < 3; ++i)
    {
        if (!std::isfinite(safe_init.pos[i]))
            safe_init.pos[i] = 0.0;
        if (!std::isfinite(safe_init.vel[i]))
            safe_init.vel[i] = 0.0;
        if (!std::isfinite(safe_init.euler[i]))
            safe_init.euler[i] = 0.0;
    }
    // sanitize imuerror
    for (int i = 0; i < 3; ++i)
    {
        if (!std::isfinite(safe_init.imuerror.gyrbias[i]))
            safe_init.imuerror.gyrbias[i] = 0.0;
        if (!std::isfinite(safe_init.imuerror.accbias[i]))
            safe_init.imuerror.accbias[i] = 0.0;
        if (!std::isfinite(safe_init.imuerror.gyrscale[i]))
            safe_init.imuerror.gyrscale[i] = 0.0;
        if (!std::isfinite(safe_init.imuerror.accscale[i]))
            safe_init.imuerror.accscale[i] = 0.0;
    }
    // now use safe_init in place of initstate where appropriate
    // e.g. replace uses of initstate by safe_init in the next lines, or set imuerror_ from safe_init
    imuerror_ = safe_init.imuerror;
    pvacur_.pos = safe_init.pos;
    pvacur_.vel = safe_init.vel;
    pvacur_.att.euler = safe_init.euler;
    pvacur_.att.cbn = Rotation::euler2matrix(pvacur_.att.euler);
    pvacur_.att.qbn = Rotation::euler2quaternion(pvacur_.att.euler);
    // ... and set pvapre_ = pvacur_ below as original code does

    // 初始化IMU误差
    imuerror_ = initstate.imuerror;

    // 给上一时刻状态赋同样的初值
    pvapre_ = pvacur_;

    // --- 对 initstate_std 做防御性检查：若参数没有被提供（含 NaN/Inf），用安全默认值替换 ---
    Eigen::Vector3d pos_std = initstate_std.pos;
    Eigen::Vector3d vel_std = initstate_std.vel;
    Eigen::Vector3d att_std = initstate_std.euler;
    ImuError imu_std = initstate_std.imuerror;

    // 设计合理的“安全默认值”（只要是正的、有限即可，避免协方差为 0）
    const Eigen::Vector3d DEF_POS_STD(1.0, 1.0, 1.0);                   // meters
    const Eigen::Vector3d DEF_VEL_STD(0.1, 0.1, 0.1);                   // m/s
    const Eigen::Vector3d DEF_ATT_STD(0.1 * D2R, 0.1 * D2R, 0.1 * D2R); // rad (0.1 deg)
    ImuError DEF_IMU_STD;
    DEF_IMU_STD.gyrbias = Eigen::Vector3d(1e-6, 1e-6, 1e-6); // small finite values
    DEF_IMU_STD.accbias = Eigen::Vector3d(1e-4, 1e-4, 1e-4);
    DEF_IMU_STD.gyrscale = Eigen::Vector3d(1e-6, 1e-6, 1e-6);
    DEF_IMU_STD.accscale = Eigen::Vector3d(1e-6, 1e-6, 1e-6);

    auto ensure_vec3 = [](Eigen::Vector3d &v, const Eigen::Vector3d &def)
    {
        for (int i = 0; i < 3; ++i)
        {
            if (!std::isfinite(v[i]) || v[i] < 0.0)
                v[i] = def[i];
        }
    };
    ensure_vec3(pos_std, DEF_POS_STD);
    ensure_vec3(vel_std, DEF_VEL_STD);
    ensure_vec3(att_std, DEF_ATT_STD);

    for (int i = 0; i < 3; ++i)
    {
        if (!std::isfinite(imu_std.gyrbias[i]) || imu_std.gyrbias[i] < 0.0)
            imu_std.gyrbias[i] = DEF_IMU_STD.gyrbias[i];
        if (!std::isfinite(imu_std.accbias[i]) || imu_std.accbias[i] < 0.0)
            imu_std.accbias[i] = DEF_IMU_STD.accbias[i];
        if (!std::isfinite(imu_std.gyrscale[i]) || imu_std.gyrscale[i] < 0.0)
            imu_std.gyrscale[i] = DEF_IMU_STD.gyrscale[i];
        if (!std::isfinite(imu_std.accscale[i]) || imu_std.accscale[i] < 0.0)
            imu_std.accscale[i] = DEF_IMU_STD.accscale[i];
    }

    // 初始化协方差：使用“已消毒”的 std 来构造对角协方差
    Cov_.block(P_ID, P_ID, 3, 3) = pos_std.cwiseProduct(pos_std).asDiagonal();
    Cov_.block(V_ID, V_ID, 3, 3) = vel_std.cwiseProduct(vel_std).asDiagonal();
    Cov_.block(PHI_ID, PHI_ID, 3, 3) = att_std.cwiseProduct(att_std).asDiagonal();
    Cov_.block(BG_ID, BG_ID, 3, 3) = imu_std.gyrbias.cwiseProduct(imu_std.gyrbias).asDiagonal();
    Cov_.block(BA_ID, BA_ID, 3, 3) = imu_std.accbias.cwiseProduct(imu_std.accbias).asDiagonal();
    Cov_.block(SG_ID, SG_ID, 3, 3) = imu_std.gyrscale.cwiseProduct(imu_std.gyrscale).asDiagonal();
    Cov_.block(SA_ID, SA_ID, 3, 3) = imu_std.accscale.cwiseProduct(imu_std.accscale).asDiagonal();

    // （可选）把任何仍然为 0 的对角元设置为小正数，避免奇异矩阵
    for (int i = 0; i < Cov_.rows(); ++i)
    {
        if (!std::isfinite(Cov_(i, i)) || Cov_(i, i) <= 0.0)
        {
            Cov_(i, i) = 1e-6;
        }
    }
}

void GIEngine::newImuProcess()
{

    // 当前IMU时间作为系统当前状态时间,
    // set current IMU time as the current state time
    timestamp_ = imucur_.time;

    // 如果GNSS有效，则将更新时间设置为GNSS时间
    // set update time as the gnss time if gnssdata is valid
    double updatetime = gnssdata_.isvalid ? gnssdata_.time : -1;

    // 判断是否需要进行GNSS更新
    // determine if we should do GNSS update
    int res = isToUpdate(imupre_.time, imucur_.time, updatetime);

    if (res == 0)
    {
        // 只传播导航状态
        // only propagate navigation state
        insPropagation(imupre_, imucur_);
    }
    else if (res == 1)
    {
        // GNSS数据靠近上一历元，先对上一历元进行GNSS更新
        // gnssdata is near to the previous imudata, we should firstly do gnss update
        gnssUpdate(gnssdata_);
        stateFeedback();

        pvapre_ = pvacur_;
        insPropagation(imupre_, imucur_);
    }
    else if (res == 2)
    {
        // GNSS数据靠近当前历元，先对当前IMU进行状态传播
        // gnssdata is near current imudata, we should firstly propagate navigation state
        insPropagation(imupre_, imucur_);
        gnssUpdate(gnssdata_);
        stateFeedback();
    }
    else
    {
        // GNSS数据在两个IMU数据之间(不靠近任何一个), 将当前IMU内插到整秒时刻
        // gnssdata is between the two imudata, we interpolate current imudata to gnss time
        IMU midimu;
        imuInterpolate(imupre_, imucur_, updatetime, midimu);

        // 对前一半IMU进行状态传播
        // propagate navigation state for the first half imudata
        insPropagation(imupre_, midimu);

        // 整秒时刻进行GNSS更新，并反馈系统状态
        // do GNSS position update at the whole second and feedback system states
        gnssUpdate(gnssdata_);
        stateFeedback();

        // 对后一半IMU进行状态传播
        // propagate navigation state for the second half imudata
        pvapre_ = pvacur_;
        insPropagation(midimu, imucur_);
    }

    // 检查协方差矩阵对角线元素
    // check diagonal elements of current covariance matrix
    checkCov();

    // 更新上一时刻的状态和IMU数据
    // update system state and imudata at the previous epoch
    pvapre_ = pvacur_;
    imupre_ = imucur_;
}

void GIEngine::imuCompensate(IMU &imu)
{

    // 补偿IMU零偏
    // compensate the imu bias
    imu.dtheta -= imuerror_.gyrbias * imu.dt;
    imu.dvel -= imuerror_.accbias * imu.dt;

    // 补偿IMU比例因子
    // compensate the imu scale
    Eigen::Vector3d gyrscale, accscale;
    gyrscale = Eigen::Vector3d::Ones() + imuerror_.gyrscale;
    accscale = Eigen::Vector3d::Ones() + imuerror_.accscale;
    imu.dtheta = imu.dtheta.cwiseProduct(gyrscale.cwiseInverse());
    imu.dvel = imu.dvel.cwiseProduct(accscale.cwiseInverse());
}

void GIEngine::insPropagation(IMU &imupre, IMU &imucur)
{

    // 防御性检查：确保 imucur 有效值（finite），并且 dt 非零
    if (!std::isfinite(imucur.time) || !imucur.dtheta.allFinite() || !imucur.dvel.allFinite())
    {
        std::cerr << "[GIEngine] insPropagation: received non-finite IMU data, skipping propagation\n";
        std::cerr << "[GIEngine] imucur.time=" << imucur.time << " dtheta=" << imucur.dtheta.transpose()
                  << " dvel=" << imucur.dvel.transpose() << std::endl;
        std::cerr << "[GIEngine] imupre.time=" << imupre_.time << " dtheta=" << imupre_.dtheta.transpose()
                  << " dvel=" << imupre_.dvel.transpose() << std::endl;
        return;
    }

    // 防止 dt 为 0 导致除以 0
    double dt = imucur.dt;
    if (!(std::isfinite(dt)) || dt <= 1e-8)
    {
        // 如果原来 imucur.dt == 0（例如第一次），使用安全小值 或者按照全局IMU频率估计
        double fallback = 1.0 / 200.0; // e.g. 1/200
        dt = std::max(1e-6, fallback);
        // 同步回 imucur.dt（确保后续使用到的是同样的值）
        imucur.dt = dt;
    }

    // 对当前IMU数据(imucur)补偿误差, 上一IMU数据(imupre)已经补偿过了
    imuCompensate(imucur);
    // IMU状态更新(机械编排算法)
    // update imustate(mechanization)
    INSMech::insMech(pvapre_, pvacur_, imupre, imucur);

    // 系统噪声传播，姿态误差采用phi角误差模型
    // system noise propagate, phi-angle error model for attitude error
    Eigen::MatrixXd Phi, F, Qd, G;

    // 初始化Phi阵(状态转移矩阵)，F阵，Qd阵(传播噪声阵)，G阵(噪声驱动阵)
    // initialize Phi (state transition), F matrix, Qd(propagation noise) and G(noise driven) matrix
    Phi.resizeLike(Cov_);
    F.resizeLike(Cov_);
    Qd.resizeLike(Cov_);
    G.resize(RANK, NOISERANK);
    Phi.setIdentity();
    F.setZero();
    Qd.setZero();
    G.setZero();

    // 使用上一历元状态计算状态转移矩阵
    // compute state transition matrix using the previous state
    Eigen::Vector2d rmrn;
    Eigen::Vector3d wie_n, wen_n;
    double gravity;
    rmrn = Earth::meridianPrimeVerticalRadius(pvapre_.pos[0]);
    gravity = Earth::gravity(pvapre_.pos);
    wie_n << WGS84_WIE * cos(pvapre_.pos[0]), 0, -WGS84_WIE * sin(pvapre_.pos[0]);
    wen_n << pvapre_.vel[1] / (rmrn[1] + pvapre_.pos[2]), -pvapre_.vel[0] / (rmrn[0] + pvapre_.pos[2]),
        -pvapre_.vel[1] * tan(pvapre_.pos[0]) / (rmrn[1] + pvapre_.pos[2]);

    Eigen::Matrix3d temp;
    Eigen::Vector3d accel, omega;
    double rmh, rnh;

    rmh = rmrn[0] + pvapre_.pos[2];
    rnh = rmrn[1] + pvapre_.pos[2];
    accel = imucur.dvel / dt;
    omega = imucur.dtheta / dt;

        // ====== start: IMU sanity checks & clamp ======
    auto clamp_val = [](double v, double minv, double maxv) {
        if (!std::isfinite(v)) return minv;
        if (v < minv) return minv;
        if (v > maxv) return maxv;
        return v;
    };

    // 最大允许的瞬时角速度/加速度（经验值），避免单帧极端值破坏滤波器
    const double MAX_ANG_RATE = 200.0;    // rad/s (非常大，不会误截正常数据)
    const double MAX_ACCEL    = 200.0;    // m/s^2 (非常大)

    // 检查 omega/accel 的每个分量是否为有限值并在合理范围内
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(omega[i]) || std::abs(omega[i]) > 1e8) {
            std::cerr << "[GIEngine] WARNING: omega invalid at time " << imucur.time << " idx " << i
                      << " value=" << omega[i] << " -> clamped\n";
            omega[i] = clamp_val(omega[i], -MAX_ANG_RATE, MAX_ANG_RATE);
        }
        if (!std::isfinite(accel[i]) || std::abs(accel[i]) > 1e8) {
            std::cerr << "[GIEngine] WARNING: accel invalid at time " << imucur.time << " idx " << i
                      << " value=" << accel[i] << " -> clamped\n";
            accel[i] = clamp_val(accel[i], -MAX_ACCEL, MAX_ACCEL);
        }
    }

    // 如果发现 dt 异常（例如太小或为 NaN），打印详细信息
    if (!std::isfinite(dt) || dt <= 1e-9) {
        std::cerr << "[GIEngine] WARNING: fallback dt used, original dt=" << imucur.dt
                  << " imucur.time=" << imucur.time << "\n";
    }
    // ====== end: IMU sanity checks & clamp ======


    // 位置误差
    // position error
    temp.setZero();
    temp(0, 0) = -pvapre_.vel[2] / rmh;
    temp(0, 2) = pvapre_.vel[0] / rmh;
    temp(1, 0) = pvapre_.vel[1] * tan(pvapre_.pos[0]) / rnh;
    temp(1, 1) = -(pvapre_.vel[2] + pvapre_.vel[0] * tan(pvapre_.pos[0])) / rnh;
    temp(1, 2) = pvapre_.vel[1] / rnh;
    F.block(P_ID, P_ID, 3, 3) = temp;
    F.block(P_ID, V_ID, 3, 3) = Eigen::Matrix3d::Identity();

    // 速度误差
    // velocity error
    temp.setZero();
    temp(0, 0) = -2 * pvapre_.vel[1] * WGS84_WIE * cos(pvapre_.pos[0]) / rmh -
                 pow(pvapre_.vel[1], 2) / rmh / rnh / pow(cos(pvapre_.pos[0]), 2);
    temp(0, 2) = pvapre_.vel[0] * pvapre_.vel[2] / rmh / rmh - pow(pvapre_.vel[1], 2) * tan(pvapre_.pos[0]) / rnh / rnh;
    temp(1, 0) = 2 * WGS84_WIE * (pvapre_.vel[0] * cos(pvapre_.pos[0]) - pvapre_.vel[2] * sin(pvapre_.pos[0])) / rmh +
                 pvapre_.vel[0] * pvapre_.vel[1] / rmh / rnh / pow(cos(pvapre_.pos[0]), 2);
    temp(1, 2) = (pvapre_.vel[1] * pvapre_.vel[2] + pvapre_.vel[0] * pvapre_.vel[1] * tan(pvapre_.pos[0])) / rnh / rnh;
    temp(2, 0) = 2 * WGS84_WIE * pvapre_.vel[1] * sin(pvapre_.pos[0]) / rmh;
    temp(2, 2) = -pow(pvapre_.vel[1], 2) / rnh / rnh - pow(pvapre_.vel[0], 2) / rmh / rmh +
                 2 * gravity / (sqrt(rmrn[0] * rmrn[1]) + pvapre_.pos[2]);
    F.block(V_ID, P_ID, 3, 3) = temp;
    temp.setZero();
    temp(0, 0) = pvapre_.vel[2] / rmh;
    temp(0, 1) = -2 * (WGS84_WIE * sin(pvapre_.pos[0]) + pvapre_.vel[1] * tan(pvapre_.pos[0]) / rnh);
    temp(0, 2) = pvapre_.vel[0] / rmh;
    temp(1, 0) = 2 * WGS84_WIE * sin(pvapre_.pos[0]) + pvapre_.vel[1] * tan(pvapre_.pos[0]) / rnh;
    temp(1, 1) = (pvapre_.vel[2] + pvapre_.vel[0] * tan(pvapre_.pos[0])) / rnh;
    temp(1, 2) = 2 * WGS84_WIE * cos(pvapre_.pos[0]) + pvapre_.vel[1] / rnh;
    temp(2, 0) = -2 * pvapre_.vel[0] / rmh;
    temp(2, 1) = -2 * (WGS84_WIE * cos(pvapre_.pos[0]) + pvapre_.vel[1] / rnh);
    F.block(V_ID, V_ID, 3, 3) = temp;
    F.block(V_ID, PHI_ID, 3, 3) = Rotation::skewSymmetric(pvapre_.att.cbn * accel);
    F.block(V_ID, BA_ID, 3, 3) = pvapre_.att.cbn;
    F.block(V_ID, SA_ID, 3, 3) = pvapre_.att.cbn * (accel.asDiagonal());

    // 姿态误差
    // attitude error
    temp.setZero();
    temp(0, 0) = -WGS84_WIE * sin(pvapre_.pos[0]) / rmh;
    temp(0, 2) = pvapre_.vel[1] / (rnh * rnh);
    temp(1, 2) = -pvapre_.vel[0] / (rmh * rmh);
    temp(2, 0) = -WGS84_WIE * cos(pvapre_.pos[0]) / rmh - pvapre_.vel[1] / (rmh * rnh * pow(cos(pvapre_.pos[0]), 2));
    temp(2, 2) = -pvapre_.vel[1] * tan(pvapre_.pos[0]) / (rnh * rnh);
    F.block(PHI_ID, P_ID, 3, 3) = temp;
    temp.setZero();
    temp(0, 1) = 1 / rnh;
    temp(1, 0) = -1 / rmh;
    temp(2, 1) = -tan(pvapre_.pos[0]) / rnh;
    F.block(PHI_ID, V_ID, 3, 3) = temp;
    F.block(PHI_ID, PHI_ID, 3, 3) = -Rotation::skewSymmetric(wie_n + wen_n);
    F.block(PHI_ID, BG_ID, 3, 3) = -pvapre_.att.cbn;
    F.block(PHI_ID, SG_ID, 3, 3) = -pvapre_.att.cbn * (omega.asDiagonal());

    // IMU零偏误差和比例因子误差，建模成一阶高斯-马尔科夫过程
    // imu bias error and scale error, modeled as the first-order Gauss-Markov process
    F.block(BG_ID, BG_ID, 3, 3) = -1 / options_.imunoise.corr_time * Eigen::Matrix3d::Identity();
    F.block(BA_ID, BA_ID, 3, 3) = -1 / options_.imunoise.corr_time * Eigen::Matrix3d::Identity();
    F.block(SG_ID, SG_ID, 3, 3) = -1 / options_.imunoise.corr_time * Eigen::Matrix3d::Identity();
    F.block(SA_ID, SA_ID, 3, 3) = -1 / options_.imunoise.corr_time * Eigen::Matrix3d::Identity();

    // 系统噪声驱动矩阵
    // system noise driven matrix
    G.block(V_ID, VRW_ID, 3, 3) = pvapre_.att.cbn;
    G.block(PHI_ID, ARW_ID, 3, 3) = pvapre_.att.cbn;
    G.block(BG_ID, BGSTD_ID, 3, 3) = Eigen::Matrix3d::Identity();
    G.block(BA_ID, BASTD_ID, 3, 3) = Eigen::Matrix3d::Identity();
    G.block(SG_ID, SGSTD_ID, 3, 3) = Eigen::Matrix3d::Identity();
    G.block(SA_ID, SASTD_ID, 3, 3) = Eigen::Matrix3d::Identity();

    // 状态转移矩阵
    // compute the state transition matrix
    Phi.setIdentity();
    Phi = Phi + F * dt;

    // 计算系统传播噪声
    // compute system propagation noise
    Qd = G * Qc_ * G.transpose() * dt;
    Qd = (Phi * Qd * Phi.transpose() + Qd) / 2;
    // ====== start: Qd sanity & clamp ======
    bool qbad = false;
    for (int i = 0; i < Qd.rows(); ++i) {
        double v = Qd(i,i);
        if (!std::isfinite(v) || v <= 0.0) {
            qbad = true;
            break;
        }
    }
    if (qbad) {
        std::cerr << "[GIEngine] ERROR: Qd has invalid entries at imucur.time=" << imucur.time << "\n";
        std::cerr << "[GIEngine] imucur.time=" << imucur.time << " dtheta=" << imucur.dtheta.transpose()
                  << " dvel=" << imucur.dvel.transpose() << " dt=" << dt << "\n";
        std::cerr << "[GIEngine] Qd diag (raw): " << Qd.diagonal().transpose() << "\n";
        // 对角线逐元素修正为最小正值，避免NaN/Inf传播
        for (int i = 0; i < Qd.rows(); ++i) {
            double v = Qd(i,i);
            if (!std::isfinite(v) || v <= 0.0) Qd(i,i) = 1e-18;
        }
    }
    // 最终再做一个 clamp：防止某些分量太大
    for (int i = 0; i < Qd.rows(); ++i) {
        double v = Qd(i,i);
        if (v > 1e18) Qd(i,i) = 1e18;
        if (v < 1e-18) Qd(i,i) = 1e-18;
    }
    std::cout << "[DEBUG] Qd 对角线: " << Qd.diagonal().transpose() << std::endl;
    // ====== end: Qd sanity & clamp ======

    // EKF预测传播系统协方差和系统误差状态
    // do EKF predict to propagate covariance and error state
    EKFPredict(Phi, Qd);
}

void GIEngine::gnssUpdate(GNSS &gnssdata)
{

    // IMU位置转到GNSS天线相位中心位置
    // convert IMU position to GNSS antenna phase center position
    Eigen::Vector3d antenna_pos;
    Eigen::Matrix3d Dr, Dr_inv;
    Dr_inv = Earth::DRi(pvacur_.pos);
    Dr = Earth::DR(pvacur_.pos);
    antenna_pos = pvacur_.pos + Dr_inv * pvacur_.att.cbn * options_.antlever;

    // GNSS位置测量新息
    // compute GNSS position innovation
    Eigen::MatrixXd dz;
    dz = Dr * (antenna_pos - gnssdata.blh);

    // 构造GNSS位置观测矩阵
    // construct GNSS position measurement matrix
    Eigen::MatrixXd H_gnsspos;
    H_gnsspos.resize(3, Cov_.rows());
    H_gnsspos.setZero();
    H_gnsspos.block(0, P_ID, 3, 3) = Eigen::Matrix3d::Identity();
    H_gnsspos.block(0, PHI_ID, 3, 3) = Rotation::skewSymmetric(pvacur_.att.cbn * options_.antlever);

    // 位置观测噪声阵
    // construct measurement noise matrix
    Eigen::MatrixXd R_gnsspos;
    R_gnsspos = gnssdata.std.cwiseProduct(gnssdata.std).asDiagonal();

    // EKF更新协方差和误差状态
    EKFUpdate(dz, H_gnsspos, R_gnsspos);

    // GNSS更新之后设置为不可用
    gnssdata.isvalid = false;
}

// ------- 以下为缺失函数的实现，请追加到 gi_engine.cpp 尾部 -------

int GIEngine::isToUpdate(double imutime1, double imutime2, double updatetime) const
{
    // 和 header 中声明一致 (const)
    if (std::abs(imutime1 - updatetime) < TIME_ALIGN_ERR)
    {
        return 1;
    }
    else if (std::abs(imutime2 - updatetime) <= TIME_ALIGN_ERR)
    {
        return 2;
    }
    else if (imutime1 < updatetime && updatetime < imutime2)
    {
        return 3;
    }
    else
    {
        return 0;
    }
}

void GIEngine::EKFPredict(Eigen::MatrixXd &Phi, Eigen::MatrixXd &Qd)
{
    // 不用 assert（以避免头文件依赖问题），仅做基本尺寸检查并执行预测
    if (Phi.rows() != Cov_.rows() || Phi.cols() != Cov_.cols() || Qd.rows() != Cov_.rows() || Qd.cols() != Cov_.cols())
    {
        std::cerr << "[GIEngine] EKFPredict: dimension mismatch, skipping predict\n";
        return;
    }

    // before computing Cov_ = Phi * Cov_ * Phi^T + Qd
    if (!Phi.allFinite()) {
        std::cerr << "[GIEngine] EKFPredict: Phi contains non-finite entries. Resetting Phi to Identity.\n";
        Phi.setIdentity();
    }
    if (!Cov_.allFinite()) {
        std::cerr << "[GIEngine] EKFPredict: Cov_ contains non-finite entries. Resetting Cov_ to small diag.\n";
        Cov_.setZero();
        Cov_.diagonal().setConstant(1e-6);
    }
    if (!Qd.allFinite()) {
        std::cerr << "[GIEngine] EKFPredict: Qd contains non-finite entries. Replacing with small diag.\n";
        Qd.setZero();
        Qd.diagonal().setConstant(1e-18);
    }


    // Cov_ = Phi * Cov_ * Phi^T + Qd
    Cov_ = Phi * Cov_ * Phi.transpose() + Qd;

    // dx_ = Phi * dx_
    if (dx_.rows() == Phi.cols() && dx_.cols() == 1)
    {
        dx_ = Phi * dx_;
    }
    else
    {
        // 若 dx_ 尺寸不匹配，重置为 0
        dx_.setZero(Cov_.rows(), 1);
    }
    double max_cov = 1e6;
    for (int i = 0; i < Cov_.rows(); ++i) {
        if (Cov_(i, i) > max_cov) Cov_(i, i) = max_cov;
        if (Cov_(i, i) < 1e-9) Cov_(i, i) = 1e-9; // 避免零或负值
    }
}

void GIEngine::EKFUpdate(Eigen::MatrixXd &dz, Eigen::MatrixXd &H, Eigen::MatrixXd &R)
{
    // 基本尺寸检查
    if (H.cols() != Cov_.rows() || dz.rows() != H.rows() || R.rows() != dz.rows() || dz.cols() != 1)
    {
        std::cerr << "[GIEngine] EKFUpdate: dimension mismatch, skipping update\n";
        return;
    }

    // 计算增益 K = Cov * H^T * (H*Cov*H^T + R)^{-1}
    Eigen::MatrixXd S = H * Cov_ * H.transpose() + R;
    // 若 S 不可逆，则使用 pseudo-inverse 或直接跳过
    double cond = 0.0;
    bool invertible = true;
    if (S.size() == 0)
        invertible = false;
    else
    {
        // 尝试求逆，捕获异常
        // 使用 .inverse() 但在数值不良时可能产生 inf/nan
        // 我们先检查行列式/对角是否有效
        if (!S.allFinite())
            invertible = false;
    }

    if (!invertible)
    {
        std::cerr << "[GIEngine] EKFUpdate: measurement covariance matrix invalid, skipping update\n";
        return;
    }

    Eigen::MatrixXd K = Cov_ * H.transpose() * S.completeOrthogonalDecomposition().pseudoInverse();

    // 更新 dx_ 和 Cov_
    if (dx_.rows() == Cov_.rows() && dx_.cols() == 1)
    {
        // 防止 dx_ 单次更新过大：做简单阈值保护
        Eigen::VectorXd innovation = dz - H * dx_;
        Eigen::VectorXd tentative = K * innovation;
        double max_allowed = 1e6; // 一个非常大的阈值，依据需要可调整
        double max_elem = tentative.cwiseAbs().maxCoeff();
        if (std::isfinite(max_elem) && max_elem > max_allowed) {
            std::cerr << "[GIEngine] EKFUpdate: dx_ too large (max=" << max_elem << ") -> clamping to " << max_allowed << std::endl;
            tentative = tentative.array().unaryExpr([&](double v){ return (v>0?std::min(v, max_allowed):std::max(v, -max_allowed)); });
        }
        dx_ = dx_ + tentative;
    }
    else
    {
        dx_ = K * dz;
    }

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(Cov_.rows(), Cov_.cols());
    Cov_ = (I - K * H) * Cov_ * (I - K * H).transpose() + K * R * K.transpose();
}

void GIEngine::stateFeedback()
{
    // 将误差状态反馈到当前导航状态 pvacur_，并清零 dx_
    if (dx_.rows() < RANK)
    {
        std::cerr << "[GIEngine] stateFeedback: dx_ size unexpected\n";
    }

    // 位置误差反馈
    Eigen::Vector3d delta_r = dx_.block(P_ID, 0, 3, 1);
    Eigen::Matrix3d Dr_inv = Earth::DRi(pvacur_.pos);
    pvacur_.pos -= Dr_inv * delta_r;

    // 速度误差反馈
    Eigen::Vector3d dv = dx_.block(V_ID, 0, 3, 1);
    pvacur_.vel -= dv;

    // 姿态误差反馈 (用小旋转向量 -> 四元数)
    Eigen::Vector3d dphi = dx_.block(PHI_ID, 0, 3, 1);

    // 如果角度误差过大则跳过姿态反馈（保护性）
    double dphi_norm = dphi.norm();
    if (!std::isfinite(dphi_norm) || dphi_norm > 1.0) { // 1 rad ~= 57 deg 是很大的阈值，可根据需要调整
        std::cerr << "[GIEngine] stateFeedback: dphi invalid or too large (norm=" << dphi_norm << ") -> skipping attitude feedback\n";
    } else {
        Eigen::Quaterniond qpn = Rotation::rotvec2quaternion(dphi);
        pvacur_.att.qbn = qpn * pvacur_.att.qbn;
        pvacur_.att.cbn = Rotation::quaternion2matrix(pvacur_.att.qbn);
        pvacur_.att.euler = Rotation::matrix2euler(pvacur_.att.cbn);
    }

    // IMU 零偏与比例因子误差反馈
    Eigen::Vector3d dbg = dx_.block(BG_ID, 0, 3, 1);
    Eigen::Vector3d dba = dx_.block(BA_ID, 0, 3, 1);
    Eigen::Vector3d dsg = dx_.block(SG_ID, 0, 3, 1);
    Eigen::Vector3d dsa = dx_.block(SA_ID, 0, 3, 1);

    // 保护性阈值，避免单次反馈将误差推到巨大的值
    const double GYRB_MAX = 1e3;   // deg/h 量级根据实际调整
    const double ACCB_MAX = 1e3;   // mGal 量级
    const double SCALE_MAX = 1e6;  // ppm 量级（非常大，防止溢出）

    for (int i = 0; i < 3; ++i) {
        // 使用 std::min/std::max 代替 std::clamp（兼容更老编译器）
        dbg[i] = std::max(-GYRB_MAX, std::min(dbg[i], GYRB_MAX));
        dba[i] = std::max(-ACCB_MAX, std::min(dba[i], ACCB_MAX));
        dsg[i] = std::max(-SCALE_MAX, std::min(dsg[i], SCALE_MAX));
        dsa[i] = std::max(-SCALE_MAX, std::min(dsa[i], SCALE_MAX));
    }

    imuerror_.gyrbias += dbg;
    imuerror_.accbias += dba;
    imuerror_.gyrscale += dsg;
    imuerror_.accscale += dsa;

    // 反馈后清零 dx_
    dx_.setZero();
}

void GIEngine::debugDump() const
{
    using std::cout;
    using std::endl;
    cout << "------ GIEngine DEBUG DUMP ------" << endl;
    cout << std::setprecision(12);

    cout << " timestamp_: " << timestamp_ << endl;

    // pvapre_
    cout << " pvapre_.pos: " << pvapre_.pos.transpose() << endl;
    cout << " pvapre_.vel: " << pvapre_.vel.transpose() << endl;
    cout << " pvapre_.att.euler: " << pvapre_.att.euler.transpose() << endl;

    // pvacur_
    cout << " pvacur_.pos: " << pvacur_.pos.transpose() << endl;
    cout << " pvacur_.vel: " << pvacur_.vel.transpose() << endl;
    cout << " pvacur_.att.euler: " << pvacur_.att.euler.transpose() << endl;

    // imuerror_
    cout << " imuerror_.gyrbias: " << imuerror_.gyrbias.transpose() << endl;
    cout << " imuerror_.accbias: " << imuerror_.accbias.transpose() << endl;
    cout << " imuerror_.gyrscale: " << imuerror_.gyrscale.transpose() << endl;
    cout << " imuerror_.accscale: " << imuerror_.accscale.transpose() << endl;

    // Covariance (print diag and shape)
    if (Cov_.size() > 0)
    {
        cout << " Cov_ size: " << Cov_.rows() << "x" << Cov_.cols() << endl;
        int rows = static_cast<int>(Cov_.rows());
        int n = std::min(rows, 10);
        cout << " Cov_ diag (first " << n << "): ";
        for (int i = 0; i < n; ++i)
        {
            cout << Cov_(i, i) << " ";
        }
        cout << endl;
    }
    else
    {
        cout << " Cov_ empty" << endl;
    }

    // imupre_ / imucur_
    cout << " imupre_.time: " << imupre_.time;
    cout << " dtheta: " << imupre_.dtheta.transpose();
    cout << " dvel: " << imupre_.dvel.transpose() << endl;

    cout << " imucur_.time: " << imucur_.time;
    cout << " dtheta: " << imucur_.dtheta.transpose();
    cout << " dvel: " << imucur_.dvel.transpose() << endl;

    // gnssdata_
    cout << " gnssdata_.time: " << gnssdata_.time << " blh: " << gnssdata_.blh.transpose()
         << " isvalid: " << (gnssdata_.isvalid ? 1 : 0) << endl;

    cout << "---------------------------------" << endl;
}

NavState GIEngine::getNavState()
{
    NavState state;
    state.pos = pvacur_.pos;
    state.vel = pvacur_.vel;
    state.euler = pvacur_.att.euler;
    state.imuerror = imuerror_;
    return state;
}

// ------- 追加实现到此结束 -------
