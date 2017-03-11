#pragma once

#include <cmath>
#include <random>
#include "landmark.h"

class Robot
{
public:
    Robot();

    bool landmark_in_view(Landmark &l);
    void update(double dt);
    bool is_moving();

    static void landmark_range_bearing(const Landmark &l, double x, double y, double yaw, double &range, double &bearing);

    // setters/getters for the robot's state
    void x(double _x) { m_x = _x; }
    void y(double _y) { m_y = _y; }
    void yaw(double _yaw) { m_yaw = _yaw; }
    void vel(double vel) { m_vel = vel; }
    void yaw_vel(double yaw_vel) { m_yaw_vel = yaw_vel; }

    double x() { return m_x; }
    double y() { return m_y; }
    double yaw() { return m_yaw; }
    double vel() { return m_vel; }
    double yaw_vel() { return m_yaw_vel; }

    // noisy sensor reading
    double vel_noisy();
    double yaw_vel_noisy();

private:
    // robot state
    double m_x = 0;
    double m_y = 0;
    double m_vel = 0;
    double m_yaw = 0;
    double m_yaw_vel = 0;

    // random number generation
    std::random_device m_random_device;
    std::default_random_engine m_random_engine;
    std::normal_distribution<double> m_dice;
};
