#include <librealsense/rs.hpp>
#include <cstdio>

#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>

double yaw, pitch, lastX, lastY; int ml, mode;
double ipd, vshift,dshift;
static void on_mouse_button(GLFWwindow * win, int button, int action, int mods)
{
    if(button == GLFW_MOUSE_BUTTON_LEFT) ml = action == GLFW_PRESS;
    if(button == GLFW_MOUSE_BUTTON_RIGHT){pitch=0;yaw=0;}
}
static double clamp(double val, double lo, double hi) { return val < lo ? lo : val > hi ? hi : val; }
static void on_cursor_pos(GLFWwindow * win, double x, double y)
{
    if(ml)
    {
        yaw = clamp(yaw - (x - lastX), -120, 120);
        pitch = clamp(pitch + (y - lastY), -80, 80);
    }
    lastX = x;
    lastY = y;
}
static void on_keypress(GLFWwindow * win, int key,int scancode,int action, int mods){
    if(action==GLFW_PRESS)
    switch(key){
        case(GLFW_KEY_RIGHT):
            ipd+=0.01;
            break;
        case(GLFW_KEY_LEFT):
            ipd-=0.01;
            break;
        case(GLFW_KEY_UP):
            vshift-=0.01;
            break;
        case(GLFW_KEY_DOWN):
            vshift+=0.01;
            break;
        case(GLFW_KEY_SPACE):
            pitch=0.0;
            yaw=0.0;
            mode+=1;
            break;
        case(GLFW_KEY_ESCAPE):
            glfwDestroyWindow(win);
            break;
        default:
            break;
    }

}

int main() try
{
    // Turn on logging. We can separately enable logging to console or to file, and use different severity filters for each.
    rs::log_to_console(rs::log_severity::warn);
    //rs::log_to_file(rs::log_severity::debug, "librealsense.log");

    // Create a context object. This object owns the handles to all connected realsense devices.
    rs::context ctx;
    printf("There are %d connected RealSense devices.\n", ctx.get_device_count());
    if(ctx.get_device_count() == 0) return EXIT_FAILURE;

    // This tutorial will access only a single device, but it is trivial to extend to multiple devices
    rs::device * dev = ctx.get_device(0);
    printf("\nUsing device 0, an %s\n", dev->get_name());
    printf("    Serial number: %s\n", dev->get_serial());
    printf("    Firmware version: %s\n", dev->get_firmware_version());

    // Configure depth and color to run with the device's preferred settings
    //dev->enable_stream(rs::stream::depth, rs::preset::best_quality);
    //dev->enable_stream(rs::stream::color, rs::preset::best_quality);
    dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 60);
    dev->enable_stream(rs::stream::color, 640, 480, rs::format::rgb8, 60);
    //dev->enable_stream(rs::stream::color, 640, 480, rs::format::yuyv, 60);
    dev->enable_stream(rs::stream::infrared, 640, 480, rs::format::y8, 60);
    dev->enable_stream(rs::stream::infrared2, 640, 480, rs::format::y8, 60);
    dev->start();

    GLuint tex;
    glGenTextures(1, &tex);

    // Open a GLFW window to display our output
    glfwInit();
    //GLFWwindow * win = glfwCreateWindow(1280, 960, "librealsense tutorial #3", nullptr, nullptr);
    int n_monitors;
    GLFWmonitor** monitors = glfwGetMonitors(&n_monitors);
    GLFWwindow * win = glfwCreateWindow(1920, 1080, "Yeah dude, I rock on a different monitor!", 1 ? monitors[n_monitors - 1] : nullptr, nullptr);
    glfwSetCursorPosCallback(win, on_cursor_pos);
    glfwSetMouseButtonCallback(win, on_mouse_button);
    glfwSetKeyCallback(win,on_keypress);
    glfwMakeContextCurrent(win);

    while(!glfwWindowShouldClose(win))
    {
        // Wait for new frame data
        glfwPollEvents();
        dev->wait_for_frames();

        // Retrieve our images
        const uint16_t * depth_image = (const uint16_t *)dev->get_frame_data(rs::stream::depth);
        const uint8_t * color_image = (const uint8_t *)dev->get_frame_data(rs::stream::color);

        /* How can we fix the depth?
         * Idea:
         *  - create a columnar store for each of x,y,chroma1,chroma2,lumosity_color,lumosity_ir1,lumosity_ir2
         *  - put all pixels with known depth in there
         *  - make a copy table of argsorts
         *  - for each of the unknown depths
         *     - find location of each element in sorted lists
         *     - define an expanding window of size n on both sides of each position in argsort list
         *     - take the intersection of all windows
         *     - increase n until m are in the intersection
         *     - assign the average value to the unknown
         * */


        // Retrieve camera parameters for mapping between depth and color
        rs::intrinsics depth_intrin = dev->get_stream_intrinsics(rs::stream::depth);
        rs::extrinsics depth_to_color = dev->get_extrinsics(rs::stream::depth, rs::stream::color);
        rs::intrinsics color_intrin = dev->get_stream_intrinsics(rs::stream::color);
        float scale = dev->get_depth_scale();

        // Set up a perspective transform in a space that we can rotate by clicking and dragging the mouse

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


        auto make_pointcloud = [&](double vx=0.0,double vy=0.0,double vz=0.0){
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            gluPerspective(90, (float)960/1080, 0.01f, 20.0f);
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
            gluLookAt(vx,vy,vz, vx,vy,1, 0,-1,0);
            glTranslatef(0,0,+0.5f);
            glRotated(pitch, 1, 0, 0);
            glRotated(yaw, 0, 1, 0);
            glTranslatef(0,0,-0.5f);

            // We will render our depth data as a set of points in 3D space
            glPointSize(2);
            glEnable(GL_DEPTH_TEST);
            glBegin(GL_POINTS);

            float last_depth=0.0;
            float last_line[depth_intrin.width];
            for(int dy=0; dy<depth_intrin.height; ++dy)
            {
                for(int dx=0; dx<depth_intrin.width; ++dx)
                {
                    // Retrieve the 16-bit depth value and map it into a depth in meters
                    uint16_t depth_value = depth_image[dy * depth_intrin.width + dx];
                    float depth_in_meters = depth_value * scale;

                    // Skip over pixels with a depth value of zero, which is used to indicate no data
                    if(depth_value == 0)
                        continue;//
                        //depth_in_meters=(last_depth+last_line[dx])/2;
                    else
                        last_depth = depth_in_meters;
                    last_line[dx]=depth_in_meters;
                    // Map from pixel coordinates in the depth image to pixel coordinates in the color image
                    rs::float2 depth_pixel = {(float)dx, (float)dy};
                    rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
                    rs::float3 color_point = depth_to_color.transform(depth_point);
                    rs::float2 color_pixel = color_intrin.project(color_point);

                    // Use the color from the nearest color pixel, or pure white if this point falls outside the color image
                    const int cx = (int)std::round(color_pixel.x), cy = (int)std::round(color_pixel.y);
                    if(cx < 0 || cy < 0 || cx >= color_intrin.width || cy >= color_intrin.height)
                    {
                        //glColor3ub(255, 255, 255);
                        continue;
                    }
                    else
                    {
                        glColor3ubv(color_image + (cy * color_intrin.width + cx) * 3);
                    }

                    // Emit a vertex at the 3D location of this depth pixel
                    glVertex3f(depth_point.x, depth_point.y, depth_point.z);
                }
            }
            glEnd();
        };
        auto make_infrared = [&](){
            glViewport(0,0,1920,1080);
            glDisable(GL_DEPTH_TEST);
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            //glOrtho(0,1920,0,1080,-1,1);
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();

            glPixelZoom(1.5, -1.5);
            glRasterPos2f(-1, .667);
            glDrawPixels(640, 480, GL_LUMINANCE, GL_UNSIGNED_BYTE, dev->get_frame_data(rs::stream::infrared));
            glRasterPos2f(0, .667);
            glDrawPixels(640, 480, GL_LUMINANCE, GL_UNSIGNED_BYTE, dev->get_frame_data(rs::stream::infrared2));
        };

        auto make_color = [&](){
            glViewport(0,0,1920,1080);
            glDisable(GL_DEPTH_TEST);
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            //glOrtho(0,1920,0,1080,-1,1);
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();

            glPixelZoom(1.5, -1.5);
            glRasterPos2f(-1, .667);
            glDrawPixels(640, 480, GL_RGB, GL_UNSIGNED_BYTE, dev->get_frame_data(rs::stream::color));
            glRasterPos2f(0, .667);
            glDrawPixels(640, 480, GL_RGB, GL_UNSIGNED_BYTE, dev->get_frame_data(rs::stream::color));
        };

        auto make_color3 = [&](){
            // my bad attempt at background image

            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            glOrtho(-1,1,-1,1,-1,1);
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();

            glEnable(GL_TEXTURE_2D);
            glBindTexture(GL_TEXTURE_2D,tex);
            glTexImage2D(GL_TEXTURE_2D,0,GL_RGB,640,480,0,GL_RGB,GL_UNSIGNED_BYTE,dev->get_frame_data(rs::stream::color));

            glBegin(GL_QUADS);
            glTexCoord2d(0,1);glVertex2d(-1,-1);
            glTexCoord2d(1,1);glVertex2d(1,-1);
            glTexCoord2d(1,0);glVertex2d(1,1);
            glTexCoord2d(0,0);glVertex2d(-1,1);
            glEnd();
            // enf background image
        };

        switch(mode % 3){
            case(0):
                glViewport(0,0,960,1080);
                make_pointcloud(-ipd,vshift,-0.05);
                glViewport(960,0,960,1080);
                make_pointcloud(ipd,vshift,-0.05);
                break;
            case(1):
                make_infrared();
                break;
             case(2):
                make_color();
                break;
             case(3): //should never get here
                glViewport(0,0,960,1080);
                make_color3();
                glViewport(960,0,960,1080);
                make_color3();
                break;
             default:
                break;
        }

        glfwSwapBuffers(win);
    }

    return EXIT_SUCCESS;
}
catch(const rs::error & e)
{
    // Method calls against librealsense objects may throw exceptions of type rs::error
    printf("rs::error was thrown when calling %s(%s):\n", e.get_failed_function().c_str(), e.get_failed_args().c_str());
    printf("    %s\n", e.what());
    return EXIT_FAILURE;
}


