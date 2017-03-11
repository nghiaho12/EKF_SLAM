#pragma once

#include <GL/gl.h>
#include <SDL2/SDL.h>
#include <string>
#include <vector>

#include "robot.h"
#include "landmark.h"
#include "EKF_SLAM.h"

class OpenGLDisplay
{
public:
    OpenGLDisplay();

    void main(std::string title, int width, int height);

private:
    // Display stuff
    void setup_opengl(int width, int height);
    void process_events();
    void display();

    void render_landmarks();
    void render_robot();
    void render_robot(float x, float y, float yaw, float r, float g, float b);
    void render_boundary();
    void render_ellipse(float x, float y, float major, float minor, float angle, float r, float g, float b);

    void init_landmarks();
    void init_robot();
    void update_robot();

    void print_help();

private:
    static constexpr float ROBOT_RADIUS = 10;
    static constexpr float ROBOT_HEADING = 20;
    static constexpr float LANDMARK_RADIUS = 5;

    std::vector<Landmark> m_landmarks;

    bool m_quit = false;
    int m_width;
    int m_height;

    SDL_Window *m_window = nullptr;
    SDL_GLContext m_context;
    uint32_t m_last_tick = 0;

    int m_yaw_vel_sp = 0;
    int m_vel_sp = 0;

    Robot m_robot;
    EKF_SLAM m_ekf;

    double m_dt = 0;
};
