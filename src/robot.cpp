#include "robot.h"
#include <cmath>
#include <random>
#include <iostream>

#include "config.h"

using namespace std;

Robot::Robot()
{
    m_random_engine.seed(m_random_device());
    m_dice = normal_distribution<double>(0.0 ,1.0);
}

void Robot::update(double dt)
{
    m_yaw = m_yaw + m_yaw_vel*dt;

    m_x = m_x + m_vel*cos(m_yaw)*dt;
    m_y = m_y + m_vel*sin(m_yaw)*dt;
}

bool Robot::landmark_in_view(Landmark &l)
{
    double range, bearing;

    landmark_range_bearing(l, m_x, m_y, m_yaw, range, bearing);

    if (range < DETECTION_RANGE && fabs(bearing) < FOV*0.5) {
        double range_sigma = range*DETECTION_RANGE_ALPHA;

        l.range = range + m_dice(m_random_engine) * range_sigma;
        l.bearing = bearing + m_dice(m_random_engine) * DETECTION_ANGLE_SIGMA;

        return true;
    }

    return false;
}

void Robot::landmark_range_bearing(const Landmark &l, double x, double y, double yaw, double &range, double &bearing)
{
    double heading_x = cos(yaw);
    double heading_y = sin(yaw);

    double dx = l.x - x;
    double dy = l.y - y;
    range = sqrt(dx*dx + dy*dy);

    dx /= range;
    dy /= range;

    double dot = dx*heading_x + dy*heading_y;

    bearing = acos(dot);

    // correct for sign
    double tangent_dx = heading_y;
    double tangent_dy = -heading_x;

    if (tangent_dx*dx + tangent_dy*dy > 0) {
        bearing = -bearing;
    }
}

double Robot::vel_noisy()
{
    double sigma = ALPHA1*fabs(m_vel) + ALPHA2*fabs(m_yaw);

    return m_vel + sigma*m_dice(m_random_engine);
}

double Robot::yaw_vel_noisy()
{
    double sigma = ALPHA3*fabs(m_vel) + ALPHA4*fabs(m_yaw);

    return m_yaw_vel + sigma*m_dice(m_random_engine);
}

bool Robot::is_moving()
{
    static const double eps = 0.0001; // kinda arbitrary

    if (fabs(m_vel) > eps || fabs(m_yaw_vel) > eps) {
        return true;
    }

    return false;
}
