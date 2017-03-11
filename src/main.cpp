#include "opengl_display.h"
#include "config.h"

int main(int argc, char **argv)
{
    OpenGLDisplay display;

    display.main("EKF_localization", WORLD_WIDTH, WORLD_HEIGHT);

	return 0;
}
