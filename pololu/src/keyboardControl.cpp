#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include "gainput/gainput.h"
#include <X11/Xlib.h>
#include <X11/Xatom.h>
#include <GL/glx.h>
#include <iostream>
#include <sstream>

enum Button
{
	Down,
	Up,
	Left,
	Right
};

const char* windowName = "Gainput basic sample";
const int width = 800;
const int height = 600;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboardControl");
    ros::NodeHandle n;
    ros::Publisher motor_pub = n.advertise<std_msgs::Int64>("/motor", 1000);
    ros::Publisher steer_pub = n.advertise<std_msgs::Int64>("/steer", 1000);
    ros::Rate loop_rate(100);

    static int attributeListDbl[] = {GLX_RGBA, GLX_DOUBLEBUFFER, /*In case single buffering is not supported*/ GLX_RED_SIZE, 1, GLX_GREEN_SIZE, 1, GLX_BLUE_SIZE, 1, None };

	Display* xDisplay = XOpenDisplay(0);
	if (xDisplay == 0)
	{
		std::cerr << "Cannot connect to X server." << std::endl;
		return -1;
	}

	Window root = DefaultRootWindow(xDisplay);

	XVisualInfo* vi = glXChooseVisual(xDisplay, DefaultScreen(xDisplay), attributeListDbl);
	assert(vi);

	GLXContext context = glXCreateContext(xDisplay, vi, 0, GL_TRUE);

	Colormap cmap = XCreateColormap(xDisplay, root,                   vi->visual, AllocNone);

	XSetWindowAttributes swa;
	swa.colormap = cmap;
	swa.event_mask = ExposureMask
		| KeyPressMask | KeyReleaseMask
		| PointerMotionMask | ButtonPressMask | ButtonReleaseMask;

	Window xWindow = XCreateWindow(
			xDisplay, root,
			0, 0, width, height, 0,
			CopyFromParent, InputOutput,
			CopyFromParent, CWEventMask,
			&swa
			);

	glXMakeCurrent(xDisplay, xWindow, context);

	XSetWindowAttributes xattr;
	xattr.override_redirect = False;
	XChangeWindowAttributes(xDisplay, xWindow, CWOverrideRedirect, &xattr);

	XMapWindow(xDisplay, xWindow);
	XStoreName(xDisplay, xWindow, windowName);

    // Setup Gainput
	gainput::InputManager manager;
	const gainput::DeviceId mouseId = manager.CreateDevice<gainput::InputDeviceMouse>();
	const gainput::DeviceId keyboardId = manager.CreateDevice<gainput::InputDeviceKeyboard>();
	const gainput::DeviceId padId = manager.CreateDevice<gainput::InputDevicePad>();

	gainput::InputMap map(manager);
	map.MapBool(Down, keyboardId, gainput::KeyDown);
    map.MapBool(Up, keyboardId, gainput::KeyUp);
    map.MapBool(Left, keyboardId, gainput::KeyLeft);
    map.MapBool(Right, keyboardId, gainput::KeyRight);

    manager.SetDisplaySize(width, height);

	while (ros::ok())
	{
		// Update Gainput
		manager.Update();
        std_msgs::Int64 msg;

        XEvent event;
		while (XPending(xDisplay))
		{
			XNextEvent(xDisplay, &event);
			manager.HandleEvent(event);
		}

		// Check button states
		if (map.GetBool(Up))
		{   
            msg.data = 6500;
            motor_pub.publish(msg);
            std::cout << "Up!!" << std::endl;
		}
		else if (map.GetBool(Down))
		{   
            msg.data = 5500;
            motor_pub.publish(msg);
			std::cout << "Down!!" << std::endl;
		}
        else
        {
            msg.data = 6000;
            motor_pub.publish(msg);
        }

        if (map.GetBool(Left))
		{   
            msg.data = 5200;
            steer_pub.publish(msg);
            std::cout << "Left!!" << std::endl;
		}
		else if (map.GetBool(Right))
		{   
            msg.data = 6800;
            steer_pub.publish(msg);
			std::cout << "Right!!" << std::endl;
		}
        else
        {
            msg.data = 6000;
            steer_pub.publish(msg);
        }


        ros::spinOnce();
        loop_rate.sleep();
	}

	return 0;
}
