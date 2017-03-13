#include "EKF_SLAM.h"

#include <iostream>
#include <limits>

#include "config.h"
#include "robot.h"

using namespace std;
using namespace Eigen;

EKF_SLAM::EKF_SLAM(int num_landmarks)
{
    init_ekf(num_landmarks);
}

void EKF_SLAM::init_ekf(int num_landmarks)
{
    int N = 3 + 2*num_landmarks;

    m_cov = MatrixXd::Zero(N, N);
    m_mu = MatrixXd::Zero(N, 1);

    // initialize landmarks to have high uncertainty
    for (int i=0; i < num_landmarks; i++) {
        int idx = 3 + 2*i;

        // NOTE: if the value is too large you'll run into numerical issues!
        m_cov(idx, idx) = 1e10;
        m_cov(idx + 1, idx + 1) = 1e10;
    }

    m_landmark_seen.resize(num_landmarks, false);
}

void EKF_SLAM::set_state(double x, double y, double yaw)
{
    m_mu(0) = x;
    m_mu(1) = y;
    m_mu(2) = yaw;
}

void EKF_SLAM::update(double v, double w, const std::vector<Sensor> &sensors, double dt)
{
    // I added special handling for w ~ 0. The original implementation does not handle this.
    const double EPS = 1e-4;

    int N = m_cov.rows();

    MatrixXd G = MatrixXd::Identity(N, N); // Jacobian
    MatrixXd MU = m_mu;
    MatrixXd COV;

    Matrix<double, 3, 2> V; // Jacobian (motion noise / state variable)
    Matrix<double, 3, 3> R; // motion noise with Jacobian applied
    Matrix<double, 2, 2> M = Matrix<double, 2, 2>::Zero(); // motion noise
    Matrix<double, 3, 3> Gx = Matrix<double, 3, 3>::Identity(); // Jacobian (motion update / state variable)
    Matrix<double, 2, 2> Q = Matrix<double, 2, 2>::Zero(); // measurement noise
    Matrix<double, 2, 1> Z; // sensor measurement
    Matrix<double, 2, 1> Zhat; // expected sensor measurement

    m_dt = dt;

    double theta = yaw(); // previous yaw

    M(0, 0) = pow(ALPHA1*fabs(v) + ALPHA2*fabs(w), 2); // velocity
    M(1, 1) = pow(ALPHA3*fabs(v) + ALPHA4*fabs(w), 2); // angular velocity

    Q(0, 0) = pow(DETECTION_RANGE*DETECTION_RANGE_ALPHA, 2);
    Q(1, 1) = pow(DETECTION_ANGLE_SIGMA, 2);

    if (fabs(w) > EPS) { // we have some angular velocity
        MU(0) = m_mu(0) - (v/w)*sin(theta) + (v/w)*sin(theta + w*dt);
        MU(1) = m_mu(1) + (v/w)*cos(theta) - (v/w)*cos(theta + w*dt);
        MU(2) = m_mu(2) +  w*dt;

        Gx(0, 2) = -(v/w)*cos(theta) + (v/w)*cos(theta + w*dt);
        Gx(1, 2) = -(v/w)*sin(theta) + (v/w)*sin(theta + w*dt);

        G.block(0, 0, 3, 3) = Gx;

        // Jacobian for motion noise model
        V(0, 0) = (-sin(theta) + sin(theta + w*dt))/w;
        V(1, 0) = ( cos(theta) - cos(theta + w*dt))/w;
        V(0, 1) =  v*(sin(theta) - sin(theta + w*dt))/(w*w) + v*cos(theta + w*dt)*dt/w;
        V(1, 1) = -v*(cos(theta) - cos(theta + w*dt))/(w*w) + v*sin(theta + w*dt)*dt/w;
        V(2, 0) = 0;
        V(2, 1) = dt;

        R = V*M*V.transpose();

        COV = G*m_cov*G.transpose();
        COV.block(0, 0, 3, 3) += R;
    } else { // zero angular velocity
        // Handle case when w ~ 0
        // Use L'Hopital rule with lim w -> 0

        MU(0) = m_mu(0) + v*cos(theta)*dt;
        MU(1) = m_mu(1) + v*sin(theta)*dt;

        Gx(0, 2) = -v*sin(theta)*dt;
        Gx(1, 2) =  v*cos(theta)*dt;

        G.block(0, 0, 3, 3) = Gx;

        V(0, 0) = cos(theta)*dt;
        V(1, 0) = sin(theta)*dt;
        V(0, 1) = -v*sin(theta)*dt*dt*0.5;
        V(1, 1) =  v*cos(theta)*dt*dt*0.5;
        V(2, 0) = 0;
        V(2, 1) = dt;

        R = V*M*V.transpose();

        COV = G*m_cov*G.transpose();
        COV.block(0, 0, 3, 3) += R;
    }

    for (const auto &z : sensors) {
        int j = z.id;
        int idx = 3 + j*2;

        if (!m_landmark_seen[j]) {
            MU(idx)     = MU(0) + z.range*cos(z.bearing + theta);
            MU(idx + 1) = MU(1) + z.range*sin(z.bearing + theta);

            m_landmark_seen[j] = true;
        }

        double dx = MU(idx) - MU(0);
        double dy = MU(idx + 1) - MU(1);
        double q = dx*dx + dy*dy;

        Z(0) = z.range;
        Z(1) = z.bearing;

        Zhat(0) = sqrt(q);
        Zhat(1) = constrain_angle(atan2(dy, dx) - theta);

        MatrixXd Fxj = MatrixXd::Zero(5, COV.rows());

        Fxj(0, 0) = 1;
        Fxj(1, 1) = 1;
        Fxj(2, 2) = 1;
        Fxj(3, idx) = 1;
        Fxj(4, idx + 1) = 1;

        MatrixXd Hi(2, 5);

        Hi << -sqrt(q)*dx, -sqrt(q)*dy, 0, sqrt(q)*dx, sqrt(q)*dy,
                dy, -dx, -q, -dy, dx;

        MatrixXd H = (1/q) * Hi * Fxj;
        MatrixXd K = COV*H.transpose()*(H*COV*H.transpose() + Q).inverse();
        MatrixXd d = (Z - Zhat);

        d(1) = constrain_angle(d(1));

        MU = MU + K*d;
        COV = (MatrixXd::Identity(COV.rows(), COV.rows()) - K*H)*COV;
    }

    MU(2) = constrain_angle(MU(2));

    m_mu = MU;
    m_cov = COV;
}

double EKF_SLAM::constrain_angle(double radian)
{
    if (radian < -M_PI) {
        radian += 2*M_PI;
    } else if (radian > M_PI) {
        radian -= 2*M_PI;
    }

    return radian;
}

void EKF_SLAM::ellipse(Eigen::MatrixXd X, double &major, double &minor, double &theta)
{
    SelfAdjointEigenSolver<Matrix2d> a(X);

    double e0 = sqrt(a.eigenvalues()(0));
    double e1 = sqrt(a.eigenvalues()(1));

    if (e0 > e1) {
        theta = atan2(a.eigenvectors()(1, 0), a.eigenvectors()(0, 0));
        major = e0;
        minor = e1;
    } else {
        theta = atan2(a.eigenvectors()(1, 1), a.eigenvectors()(0, 1));
        major = e1;
        minor = e0;
    }
}

void EKF_SLAM::pose_ellipse(double &major, double &minor, double &theta)
{
    ellipse(m_cov.block(0, 0, 2, 2), major, minor, theta);
}

void EKF_SLAM::landmark_ellipse(int id, double &x, double &y, double &major, double &minor, double &theta)
{
    int idx = 3 + id*2;

    x = m_mu(idx);
    y = m_mu(idx + 1);

    ellipse(m_cov.block(idx, idx, 2, 2), major, minor, theta);
}
