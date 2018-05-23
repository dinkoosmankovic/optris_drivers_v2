#include <stdio.h>
#include <string.h>
#include <iostream>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/SetCameraInfo.h"
#include "std_msgs/String.h"
#include "ros/ros.h"
#include <sstream>
#include <vector>
#include <typeinfo>


// Optris device interface
#include "IRDeviceUVC.h"

// Optris imager interface
#include "IRImager.h"

// Optris frame rate counter
#include "FramerateCounter.h"

// Optris logging interface
#include "IRLogger.h"

// Class wrapping callback routines
#include "IRImagerHandler.h"

using namespace std;
using namespace evo;

int main (int argc, char* argv[])
{
    ros::init(argc, argv, "optris_thermal_node");
    ros::NodeHandle n;
    std::string xmlConfig = "";
    
    // A specific configuration file for each imager device is needed (cf. config directory)
    if (!n.getParam("xmlConfig", xmlConfig))
    {
        ROS_INFO("Please provide xmlConfig parameter!");
        return 0;
    }
    n.getParam("xmlConfig", xmlConfig);

    IRLogger::setVerbosity(IRLOG_ERROR, IRLOG_OFF);
    IRDeviceParams params;
    IRDeviceParamsReader::readXML(xmlConfig.c_str(), params);
    IRDeviceUVC* dev = IRDeviceUVC::createInstance(NULL, params.serial, params.videoFormatIndex);

    // Initialize Optris image processing chain
    IRImager imager;
    if(imager.init(&params, dev->getFrequency(), dev->getWidth(), dev->getHeight()))
    {
        unsigned int w = imager.getWidth();
        unsigned int h = imager.getHeight();
        if(w==0 || h==0)
        {
            cout << "Error: Image streams not available or wrongly configured. Check connection of camera and config file." << endl;
            return -1;
        }

        cout << "Connected camera, serial: " << dev->getSerial()
             << ", HW(Rev.): "               << imager.getHWRevision()
             << ", FW(Rev.): "               << imager.getFWRevision() << endl;
        cout << "Thermal channel: " << w << "x" << h << "@" << params.framerate << "Hz" << endl;
        if(imager.hasBispectralTechnology())
            cout << "Visible channel: " << imager.getVisibleWidth() << "x" << imager.getVisibleHeight() << "@" << params.framerate << "Hz" << endl;

        IRImagerHandler handler(dev, &imager);
        if(dev->startStreaming()==IRIMAGER_DISCONNECTED)
        {
            cout << "Error occurred in starting stream ... aborting. You may need to reconnect the camera." << endl;
            exit(-1);

        }

        ros::Publisher chatter_pub = n.advertise<sensor_msgs::Image>("optris_thermal_image", 1000);
        ros::Rate loop_rate(params.framerate);
        FramerateCounter fpsStream;

        while(ros::ok())
        {
            if(handler.checkForNewFrame())
            {
                //cout << "testing...\n";
                sensor_msgs::Image msg;
		        sensor_msgs::CameraInfo msgCI;
		        sensor_msgs::SetCameraInfo msgSCI;
                std_msgs::String test;

                unsigned short* image = handler.getThermalImage();
                msg.height = h;
                msg.width  = w;
                msg.encoding = "16SC1";
                msg.step = w * 2;
                msg.data.resize(h * w * 2);

		        msgCI.height = h;
                msgCI.width  = w;
		        msgCI.distortion_model = "D";

	
                if (image == NULL) continue;
                
                memcpy(&msg.data[0], image, w * h * sizeof(*image));
                chatter_pub.publish(msg);

                ros::spinOnce();

                loop_rate.sleep();
            }
            //double fps;
            //if(fpsStream.trigger(&fps))
            //    ROS_INFO("FRAMERATE: %.2f fps.", fps);
        }


        dev->stopStreaming();
    }


    cout << "Exiting application" << endl;

    delete dev;
    //raise(SIGTERM);

    return 0;
}
