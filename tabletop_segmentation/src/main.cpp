
/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file main.cpp
 * @authors: Fabrizio Bottarel <fabrizio.bottarel@iit.it>
 */

#include <string>
#include <vector>

//#include <pcl/pcl_base.h>
//#include <pcl/visualization/pcl_visualizer.h>

#include <yarp/os/RpcClient.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/PointCloud.h>
#include <yarp/sig/Image.h>
//#include <yarp/pcl/Pcl.h>

class CameraIntrinsicParams
{
    /*
     *  Parameters of the sensors used for acquisition
     *  Can either be rgb or depth
     */
private:

    //  size
    int     width;
    int     height;

    //  focal length
    float   fx;
    float   fy;
    float   fx_offset;
    float   fy_offset;

    //  principal point (center)
    float   cx;
    float   cy;
    float   cx_offset;
    float   cy_offset;

public:

    //  Methods to retrieve the parameters
    int getw() {return width; }

    int geth() {return height; }

    float getFx() { return fx + fx_offset; }

    float getFy() { return fy + fy_offset; }

    float getCx() { return cx + cx_offset; }

    float getCy() { return cy + cy_offset; }

};



class AcquisitionModule : public yarp::os::RFModule
{
private:

    //  Camera rpc port
    yarp::os::RpcClient camera_rpc_port;

    //  Ports for the camera stream
    yarp::os::BufferedPort <yarp::sig::ImageOf <yarp::sig::PixelRgb> >  camera_in_rgb_port;
    yarp::os::BufferedPort <yarp::sig::ImageOf <yarp::sig::PixelFloat> > camera_in_depth_port;

    //  Reconstructed scene point cloud
    yarp::sig::PointCloud <yarp::sig::DataXYZ> scene_point_cloud;
    yarp::sig::PointCloud <yarp::sig::DataXYZRGBA> scene_point_cloud_rgb;













};



int main(int argc, char** argv) {





    return 0;


}
