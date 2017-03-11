#include "opengl_display.h"
#include <iostream>
#include <GL/glu.h>
#include <unistd.h>

#include "config.h"

using namespace std;

OpenGLDisplay::OpenGLDisplay() :
    m_ekf(TOTAL_LANDMARKS)
{
}

void OpenGLDisplay::main(std::string title, int width, int height)
{
    m_width = width;
    m_height = height;

    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        cerr <<  "Video initialization failed: " << SDL_GetError() << endl;
        return;
    }

    m_window = SDL_CreateWindow(title.c_str(), SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, width, height, SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
    if (m_window == NULL) {
        cerr << "Window could not be created! SDL Error: " << SDL_GetError() << endl;
        return;
    }

    //Create context
    m_context = SDL_GL_CreateContext(m_window);
    if (m_context == NULL) {
        cerr << "OpenGL context could not be created! SDL Error: " << SDL_GetError() << endl;
        return;
    }

    setup_opengl(width, height);

    print_help();

    init_landmarks();
    init_robot();

    m_last_tick = SDL_GetTicks();

    while (!m_quit) {
        m_dt = (SDL_GetTicks() - m_last_tick) / 1000.0;

        process_events();
        update_robot();
        display();

        m_last_tick = SDL_GetTicks();

        usleep(10000);
    }
}

void OpenGLDisplay::setup_opengl(int width, int height)
{
    // for 2d drawing
    glDepthMask(GL_FALSE);  // disable writes to Z-Buffer
    glDisable(GL_DEPTH_TEST);

    // for font
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    // for transparent texture
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void OpenGLDisplay::process_events()
{
    SDL_Event event;

    while (SDL_PollEvent(&event)) {
        switch (event.type) {
            case SDL_KEYDOWN: {
                if (event.key.keysym.sym == SDLK_ESCAPE) {
                    m_quit = true;
                } else if (event.key.keysym.sym == SDLK_LEFT) {
                    m_yaw_vel_sp = ROBOT_YAW_VEL;
                } else if (event.key.keysym.sym == SDLK_RIGHT) {
                    m_yaw_vel_sp = -ROBOT_YAW_VEL;
                } else if (event.key.keysym.sym == SDLK_UP) {
                    m_vel_sp = ROBOT_VEL;
                } else if (event.key.keysym.sym == SDLK_DOWN) {
                    m_vel_sp = -ROBOT_VEL;
                }

                break;
            }

            case SDL_KEYUP: {
                if (event.key.keysym.sym == SDLK_LEFT || event.key.keysym.sym == SDLK_RIGHT) {
                    m_yaw_vel_sp = 0;
                } else if (event.key.keysym.sym == SDLK_UP || event.key.keysym.sym == SDLK_DOWN) {
                    m_vel_sp = 0;
                }

                break;
            }
        }
    }
}

void OpenGLDisplay::display()
{
    glViewport(0, 0, m_width, m_height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, m_width, 0, m_height);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    render_boundary();
    render_landmarks();
    render_robot();

    SDL_GL_SwapWindow(m_window);
}

void OpenGLDisplay::init_landmarks()
{
    // random number generation
    random_device rd;
    default_random_engine random_engine;
    uniform_real_distribution<> dice(0, 1);

    random_engine.seed(rd());

    m_landmarks.resize(TOTAL_LANDMARKS);

    for (auto &l : m_landmarks) {
        l.x = BOUNDARY_X1 + (BOUNDARY_X2 - BOUNDARY_X1)*dice(random_engine);
        l.y = BOUNDARY_Y1 + (BOUNDARY_Y2 - BOUNDARY_Y1)*dice(random_engine);
        l.r = 1.0;
        l.g = 0.0;
        l.b = 0.0;
    }
}

void OpenGLDisplay::init_robot()
{
    m_robot.x(m_width*2/3);
    m_robot.y(m_height/2);
    m_robot.yaw(M_PI_2);

    m_ekf.set_state(m_robot.x(), m_robot.y(), m_robot.yaw());
}

void OpenGLDisplay::render_robot(float x, float y, float yaw, float r, float g, float b)
{
    glColor3f(r, g, b);

    glPushMatrix();
    glLoadIdentity();

    glBegin(GL_LINE_LOOP);
    for (float a=0; a < 360; a += 10) {
        float _x = x + ROBOT_RADIUS*cos(a*M_PI/180);
        float _y = y + ROBOT_RADIUS*sin(a*M_PI/180);

        glVertex2f(_x, _y);
    }
    glEnd();

    // heading arrow
    glBegin(GL_LINES);
    glVertex2f(x, y);
    glVertex2f(x + ROBOT_HEADING*cos(yaw), y + ROBOT_HEADING*sin(yaw));
    glEnd();

    glPopMatrix();
}

void OpenGLDisplay::render_robot()
{
    render_robot(m_robot.x(), m_robot.y(), m_robot.yaw(), 0, 1, 0);
    render_robot(m_ekf.x(), m_ekf.y(), m_ekf.yaw(), 0.5, 0.5, 0.5);

    glColor3f(0, 1, 0);

    // fov cone
    double fov = FOV;
    double range = DETECTION_RANGE;

    double x1 = m_robot.x() + range*cos(m_robot.yaw() - fov/2);
    double y1 = m_robot.y() + range*sin(m_robot.yaw() - fov/2);

    double x2 = m_robot.x() + range*cos(m_robot.yaw() + fov/2);
    double y2 = m_robot.y() + range*sin(m_robot.yaw() + fov/2);

    glBegin(GL_LINES);
    glVertex2f(m_robot.x(), m_robot.y());
    glVertex2f(x1, y1);
    glEnd();

    glBegin(GL_LINES);
    glVertex2f(m_robot.x(), m_robot.y());
    glVertex2f(x2, y2);
    glEnd();

    glBegin(GL_LINES);
    glVertex2f(x1, y1);
    glVertex2f(x2, y2);
    glEnd();

    // error ellipse
    double major, minor, angle;

    m_ekf.pose_ellipse(major, minor, angle);

    render_ellipse(m_ekf.x(), m_ekf.y(), major*ELLIPSE_SCALE, minor*ELLIPSE_SCALE, angle, 0.5, 0.5, 0.5);

    for (size_t i=0; i < m_landmarks.size(); i++) {
        double x, y;

        m_ekf.landmark_ellipse(i, x, y, major, minor, angle);

        render_ellipse(x, y, major*ELLIPSE_SCALE, minor*ELLIPSE_SCALE, angle, 0.5, 0.5, 0.5);
    }

    glPopMatrix();
}

void OpenGLDisplay::render_ellipse(float x, float y, float major, float minor, float angle, float r, float g, float b)
{
    glPushMatrix();
    glLoadIdentity();

    glColor3f(r, g, b);

    glTranslatef(x, y, 0);
    glRotatef(angle*180/M_PI, 0, 0, 1);

    // draw major axis
    glBegin(GL_LINES);
    glVertex2f(-major, 0);
    glVertex2f(major, 0);
    glEnd();

    // draw minor axis
    glBegin(GL_LINES);
    glVertex2f(0, -minor);
    glVertex2f(0, minor);
    glEnd();

    // draw ellipise
    glBegin(GL_LINE_LOOP);
    for (float a=0; a < 360.0; a += 10.0) {
        double xe = major*cos(a*M_PI/180);
        double ye = minor*sin(a*M_PI/180);

        glVertex2f(xe, ye);
    }
    glEnd();

    glPopMatrix();
}

void OpenGLDisplay::render_boundary()
{
    glPushMatrix();
    glLoadIdentity();

    glColor3f(1, 1, 1);
    glBegin(GL_LINE_LOOP);
    glVertex2f(BOUNDARY_X1, BOUNDARY_Y1);
    glVertex2f(BOUNDARY_X2, BOUNDARY_Y1);
    glVertex2f(BOUNDARY_X2, BOUNDARY_Y2);
    glVertex2f(BOUNDARY_X1, BOUNDARY_Y2);
    glEnd();

    glPopMatrix();
}

void OpenGLDisplay::render_landmarks()
{
    glPushMatrix();
    glLoadIdentity();

    for (auto l : m_landmarks) {
        glColor3f(l.r, l.g, l.b);

        if (m_robot.landmark_in_view(l)) {
            glLineWidth(5.0);
        } else {
            glLineWidth(1.0);
        }

        glBegin(GL_LINE_LOOP);
        glVertex2f(l.x - LANDMARK_RADIUS,  l.y - LANDMARK_RADIUS);
        glVertex2f(l.x - LANDMARK_RADIUS,  l.y + LANDMARK_RADIUS);
        glVertex2f(l.x + LANDMARK_RADIUS,  l.y + LANDMARK_RADIUS);
        glVertex2f(l.x + LANDMARK_RADIUS,  l.y - LANDMARK_RADIUS);
        glEnd();
    }

    glLineWidth(1.0);

    glPopMatrix();
}

void OpenGLDisplay::update_robot()
{
    double prev_x = m_robot.x();
    double prev_y = m_robot.y();

    m_robot.vel(m_vel_sp);
    m_robot.yaw_vel(m_yaw_vel_sp);

    m_robot.update(m_dt);

    // enforce boundary
    if (m_robot.x() < BOUNDARY_X1 ||
        m_robot.y() < BOUNDARY_Y1 ||
        m_robot.x() > BOUNDARY_X2 ||
        m_robot.y() > BOUNDARY_Y2) {

        m_robot.x(prev_x);
        m_robot.y(prev_y);
        m_robot.vel(0);
    }

    if (m_robot.is_moving()) {
        vector<Sensor> sensors;

        for (size_t i=0; i < m_landmarks.size(); i++) {
            auto l = m_landmarks[i];

            if (m_robot.landmark_in_view(l)) {
                Sensor z;

                z.id = i;

                m_robot.landmark_range_bearing(l, m_robot.x(), m_robot.y(), m_robot.yaw(), z.range, z.bearing);

                sensors.push_back(z);
            }
        }

        m_ekf.update(m_robot.vel_noisy(), m_robot.yaw_vel_noisy(), sensors, m_dt);
    }
}

void OpenGLDisplay::print_help()
{
    cout << "EKF SLAM with known data association demo." << endl;
    cout << "Implementation is based on the book Probabilistic Robotics by Thrun et. al." << endl;
    cout << endl;
    cout << "Use the arrow key to move the robot around" << endl;
    cout << "  - Green is the true state of the robot" << endl;
    cout << "  - Gray is the estimate with 99% confidence ellipse" << endl;
    cout << "  - Red is a landmark" << endl;
}
