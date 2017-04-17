#pragma once

#include <Eigen/Dense>
#include <vector>

#include "landmark.h"

struct Sensor
{
    double range;
    double bearing;
    int id; // landmark id, data association is provided
};

class EKF_SLAM
{
public:
    EKF_SLAM(int num_landmarks);

    // v = velocity
    // w = yaw  velocity
    void update(double v, double w, const std::vector<Sensor> &sensors, double dt);

    void set_state(double x, double y, double yaw);

    // getters
    double x() { return m_mu(0); }
    double y() { return m_mu(1); }
    double yaw() { return m_mu(2); }

    Eigen::MatrixXd mu() { return m_mu; }
    Eigen::MatrixXd cov() { return m_cov; }

    void pose_ellipse(double &major, double &minor, double &theta);
    void landmark_ellipse(int id, double &x, double &y, double &major, double &minor, double &theta);

private:
    void init_ekf(int num_landmarks);
    void calc_error_ellipse();
    double constrain_angle(double radian);
    void ellipse(Eigen::MatrixXd X, double &major, double &minor, double &theta);

private:
    Eigen::MatrixXd m_mu; // estimated state = [robot pose, N*landmark]
    Eigen::MatrixXd m_cov; // state covariance
    std::vector<bool> m_landmark_seen;

    double m_dt;
};
