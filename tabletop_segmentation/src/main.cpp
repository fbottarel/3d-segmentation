
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

#include <pcl/pcl_base.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#include <yarp/os/RpcClient.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Value.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/LockGuard.h>
#include <yarp/sig/PointCloud.h>
#include <yarp/sig/Image.h>
#include <yarp/dev/IRGBDSensor.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/pcl/Pcl.h>

#include <src/TableSegmentationServices.h>

//  Define some custom types
typedef yarp::sig::ImageOf <yarp::sig::PixelRgb> RGBImage_yarp;
typedef yarp::sig::ImageOf <yarp::sig::PixelFloat> DepthImage_yarp;
typedef yarp::sig::PointCloud <yarp::sig::DataXYZ> PointCloud_yarp;
typedef yarp::sig::PointCloud <yarp::sig::DataXYZRGBA> PointCloud_RGBA_yarp;

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

    CameraIntrinsicParams(yarp::os::Property &config_params)
    {
        this->fx = config_params.find("focalLengthX").asFloat32();
        this->fy = config_params.find("focalLengthY").asFloat32();
        this->cx = config_params.find("principalPointX").asFloat32();
        this->cy = config_params.find("principalPointY").asFloat32();

        this->fx_offset = 0.0;
        this->fy_offset = 0.0;
        this->cx_offset = 0.0;
        this->cy_offset = 0.0;
    }

    //  Methods to set image size

    void setWidth(int w) { this->width = w; }

    void setHeight(int h) { this->height = h; }

    //  Methods to retrieve the offset parameters

    int getWidth() { return this->width; }

    int getHeight() { return this->height; }

    float getFx() { return fx + fx_offset; }

    float getFy() { return fy + fy_offset; }

    float getCx() { return cx + cx_offset; }

    float getCy() { return cy + cy_offset; }

};

class TabletopSegmentationModule : public yarp::os::RFModule, public TableSegmentationServices
{

private:

    yarp::os::Mutex mutex;

    std::string module_id;

    yarp::dev::PolyDriver poly;
    yarp::dev::IRGBDSensor* iRgbd;

    //  Camera rpc port
    yarp::os::RpcClient camera_rpc_port;

    //  Service rpc server
    yarp::os::RpcServer rpc_service;

    //  Camera parameters
    std::unique_ptr <CameraIntrinsicParams> depth_cam_params;
    std::unique_ptr <CameraIntrinsicParams> rgb_cam_params;
    yarp::os::Property depth_intrinsics;
    yarp::os::Property rgb_intrinsics;

    //  Ports for the camera stream
    yarp::os::BufferedPort <RGBImage_yarp>  camera_in_rgb_port;
    yarp::os::BufferedPort <DepthImage_yarp> camera_in_depth_port;

    //  Depth and rgb images from the camera
    RGBImage_yarp img_rgb;
    yarp::sig::FlexImage img_rgb_flex;
    DepthImage_yarp img_depth;

    //  Reconstructed scene point cloud
    PointCloud_yarp scene_point_cloud;
    PointCloud_RGBA_yarp scene_point_cloud_rgb;

    //  PCL Visualizer
    pcl::visualization::PCLVisualizer visualizer;
    int visualizer_x_size;
    int visualizer_y_size;
    int total_pc_viewport = 0;

    // Point cloud cutoff distance
    double workspace_limit_x;
    double workspace_limit_y;
    double workspace_limit_z;

/****************************************************/

public:

    TabletopSegmentationModule():
        module_id("tabletopSegmentation"),        
        iRgbd(nullptr),
        visualizer("Tabletop Segmentation")
    {}

    /****************************************************/

    bool attach(yarp::os::RpcServer &source) override
    {
        return this->yarp().attachAsServer(source);
    }

    /****************************************************/

    bool configure(yarp::os::ResourceFinder &rf) override
    {

        yInfo() << "Configuring module.";

        //  Configure viewer size
        yarp::os::Bottle bottle_viewer_params = rf.findGroup("VIEWER_SETTINGS");
        visualizer_x_size = bottle_viewer_params.check("x_size", yarp::os::Value(800)).asInt();
        visualizer_y_size = bottle_viewer_params.check("y_size", yarp::os::Value(600)).asInt();

        //  Configure cutoff distance; default is no limit
        yarp::os::Bottle bottle_workspace_limits = rf.findGroup("WORKSPACE_LIMITS");
        workspace_limit_x = bottle_workspace_limits.check("x_limit", yarp::os::Value(-1.0)).asDouble();
        workspace_limit_y = bottle_workspace_limits.check("y_limit", yarp::os::Value(-1.0)).asDouble();
        workspace_limit_z = bottle_workspace_limits.check("z_limit", yarp::os::Value(-1.0)).asDouble();

        //  Configure and open rgbd sensor driver
        yarp::os::Property config;
        config.put("device", "RGBDSensorClient");
        config.put("localImagePort", "/" + module_id + "/clientRgbPort:i");
        config.put("localDepthPort", "/" + module_id + "/clientDepthPort:i");
        config.put("localRpcPort", "/" + module_id + "/clientRpcPort:rpc");
        config.put("remoteImagePort", "/depthCamera/rgbImage:o");
        config.put("remoteDepthPort", "/depthCamera/depthImage:o");
        config.put("remoteRpcPort", "/depthCamera/rpc:i");

        if (!poly.open(config))
        {
            yError() << "Unable to open RGBDClient polydriver";
            return false;
        }

        //  Open interface to the sensor driver
        if (!poly.view(iRgbd))
        {
            yError() << "Unable to open IRGBD interface!";
            return false;
        }

        /*
         *  Get camera parameters from the camera driver
         *  instead of config file or user input.
         */

        iRgbd->getDepthIntrinsicParam(depth_intrinsics);
        iRgbd->getRgbIntrinsicParam(rgb_intrinsics);
        depth_cam_params = std::unique_ptr <CameraIntrinsicParams> (new CameraIntrinsicParams(depth_intrinsics));
        rgb_cam_params = std::unique_ptr <CameraIntrinsicParams> (new CameraIntrinsicParams(rgb_intrinsics));

        depth_cam_params->setWidth(iRgbd->getDepthWidth());
        depth_cam_params->setHeight(iRgbd->getDepthHeight());

        rgb_cam_params->setWidth(iRgbd->getRgbWidth());
        rgb_cam_params->setHeight(iRgbd->getRgbWidth());

        yInfo() << "Camera parameters:"
                << "\n\tDEPTH"
                << "\n\tRes:" << depth_cam_params->getWidth() << "x" << depth_cam_params->getHeight()
                << "\n\tfx:" << depth_cam_params->getFx()
                << "\n\tfy:" << depth_cam_params->getFy()
                << "\n\tcx:" << depth_cam_params->getCx()
                << "\n\tcy:" << depth_cam_params->getCy()
                << "\n\tWIDTH"
                << "\n\tRes:" << rgb_cam_params->getWidth() << "x" << rgb_cam_params->getHeight()
                << "\n\tfx:" << rgb_cam_params->getFx()
                << "\n\tfy:" << rgb_cam_params->getFy()
                << "\n\tcx:" << rgb_cam_params->getCx()
                << "\n\tcy:" << rgb_cam_params->getCy();

         //  Set up ports for reading data from the camera
        camera_rpc_port.open("/" + module_id + "/camera:rpc");
        camera_in_depth_port.open("/" + module_id + "/camera_depth:i");
        camera_in_rgb_port.open("/" + module_id + "/camera_rgb:i"); 

        //  Attach the rpc port to the rpcserver
        rpc_service.open("/" + module_id + "/cmd:rpc");
        attach(rpc_service);

        //  Prepare the point cloud
        scene_point_cloud.clear();
        scene_point_cloud_rgb.clear();

        //  Set up the visualizer
        visualizer.createViewPort(0.0, 0.0, 1.0, 1.0, total_pc_viewport);
        visualizer.addCoordinateSystem(0.2, "reference", total_pc_viewport);
        visualizer.initCameraParameters();
        visualizer.setSize(800, 600);
        visualizer.setCameraPosition(0.0, 0.0, 0.0, 0.0, -1.0, 0.0, total_pc_viewport);
        visualizer.spinOnce(1);

        return true;
    }

    /****************************************************/

    double getPeriod() override
    {
        return 0.1;
    }

    /****************************************************/

    bool updateModule() override
    {

        /*  Retrieve rgb and depth image
         *  rgb image needs to be converted from FlexImage to ImageOf
         */


        yarp::os::LockGuard lg(mutex);


//        img_rgb_flex.setPixelCode(VOCAB_PIXEL_RGB);

//        bool ok_acquisition{false};

//        ok_acquisition |= iRgbd->getDepthImage(img_depth);

//        yDebug() << __LINE__;

//        ok_acquisition |= iRgbd->getRgbImage(img_rgb_flex);

//        yDebug() << __LINE__;

//        if (img_rgb_flex.width() > 0 && img_rgb_flex.height() > 0)
//        {
//            img_rgb.setExternal(img_rgb_flex.getRawImage(), img_rgb_flex.width(), img_rgb_flex.height());
//        }
//        else
//        {
//            yWarning() << "Retrieved RGB image has invalid dimensions";
//            return true;
//        }

//        depthToPointCloud(img_depth, img_rgb, scene_point_cloud_rgb);

        RGBImage_yarp *img_rgb_temp;
        DepthImage_yarp *img_depth_temp;

        img_rgb_temp    = camera_in_rgb_port.read(true);
        img_depth_temp  = camera_in_depth_port.read(true);

        depthToPointCloud(*img_depth_temp, *img_rgb_temp, scene_point_cloud_rgb);

        pcl::PointCloud <pcl::PointXYZRGBA>::Ptr pcl_cloud(new pcl::PointCloud <pcl::PointXYZRGBA>);

        //yInfo() << "Point cloud computed. Total points:" << scene_point_cloud_rgb.size();

        if (scene_point_cloud_rgb.size() > 0)
        {
            yarp::pcl::toPCL <yarp::sig::DataXYZRGBA, pcl::PointXYZRGBA> (scene_point_cloud_rgb, *pcl_cloud);
        }

        //yInfo() << "Point cloud conversion complete. Total points:" << pcl_cloud->size();

        visualizer.removePointCloud("complete_point_cloud", total_pc_viewport);
        visualizer.addPointCloud <pcl::PointXYZRGBA> (pcl_cloud, "complete_point_cloud", total_pc_viewport);
        visualizer.spinOnce();

        return (!visualizer.wasStopped() && !isStopping());
    }

    /****************************************************/

    bool interruptModule() override
    {
        camera_rpc_port.interrupt();
        camera_in_rgb_port.interrupt();
        camera_in_depth_port.interrupt();

        rpc_service.interrupt();

        return true;
    }

    /****************************************************/

    bool close() override
    {
        camera_rpc_port.close();
        camera_in_rgb_port.close();
        camera_in_depth_port.close();

        rpc_service.close();

        poly.close();

        return true;
    }

    /****************************************************/

    bool depthToPointCloud(const DepthImage_yarp &depth_map, PointCloud_yarp &pc)
    {
        /*
         *  Create a temporary point cloud
         *  fill it according to the depth map and intrinsic params
         *  and return it if non empty
         */

        PointCloud_yarp pc_temp;

        pc_temp.resize(depth_map.width(), depth_map.height());

        for (size_t u_idx = 0; u_idx < depth_map.width(); ++u_idx)
        {
            for (size_t v_idx = 0; v_idx < depth_map.height(); ++v_idx)
            {
                pc_temp(u_idx, v_idx).x = (u_idx - depth_cam_params->getCx()) / depth_cam_params->getFx() * depth_map.pixel(u_idx, v_idx);
                pc_temp(u_idx, v_idx).y = (v_idx - depth_cam_params->getCy()) / depth_cam_params->getFy() * depth_map.pixel(u_idx, v_idx);
                pc_temp(u_idx, v_idx).z = depth_map.pixel(u_idx, v_idx);
            }
        }

        if (pc_temp.size() > 0)
        {
            //  Copy the point cloud to output
            pc.clear();
            pc.copy(pc_temp);
            return true;
        }
        else
        {
            return false;
        }
    }

    /****************************************************/

    bool depthToPointCloud(const DepthImage_yarp &depth_map, const RGBImage_yarp &rgb_img, PointCloud_RGBA_yarp &pc)
    {
        /*
         *  Create a temporary point cloud
         *  fill it according to the depth map and intrinsic params
         *  and return it if non empty
         *  The point cloud is colored, if the images are not the
         *  same size an error will be thrown
         */

        if (rgb_img.height() != depth_map.height() || rgb_img.width() != depth_map.width())
        {
            yError() << "RGB image and depth image have different resolutions!";
            return false;
        }

        PointCloud_RGBA_yarp pc_temp;

        pc_temp.resize(depth_map.width(), depth_map.height());

        for (size_t u_idx = 0; u_idx < depth_map.width(); ++u_idx)
        {
            for (size_t v_idx = 0; v_idx < depth_map.height(); ++v_idx)
            {
                //  Point coordinates
                pc_temp(u_idx, v_idx).x = (u_idx - depth_cam_params->getCx()) / depth_cam_params->getFx() * depth_map.pixel(u_idx, v_idx);
                pc_temp(u_idx, v_idx).y = (v_idx - depth_cam_params->getCy()) / depth_cam_params->getFy() * depth_map.pixel(u_idx, v_idx);
                pc_temp(u_idx, v_idx).z = depth_map.pixel(u_idx, v_idx);

                //  Point color
                pc_temp(u_idx, v_idx).r = rgb_img.pixel(u_idx, v_idx).r;
                pc_temp(u_idx, v_idx).g = rgb_img.pixel(u_idx, v_idx).g;
                pc_temp(u_idx, v_idx).b = rgb_img.pixel(u_idx, v_idx).b;
                pc_temp(u_idx, v_idx).a = 255;

            }
        }

        if (pc_temp.size() > 0)
        {
            //  Copy the point cloud on output
            pc.clear();
            pc.copy(pc_temp);
            return true;
        }
        else
        {
            yError() << "Reconstructed point cloud is empty";
            return false;
        }
    }

    /****************************************************/

    bool dumpScene(const std::string &filename) override
    {
        /*
         * Save the last processed point cloud to file
         * as an ASCII pcd file
         * (if the point cloud is valid and contains something)
         */

        yarp::os::LockGuard lg(mutex);

        bool success{false};

        if (scene_point_cloud_rgb.size() > 0)
        {
            success = (yarp::pcl::savePCD <yarp::sig::DataXYZRGBA, pcl::PointXYZRGBA> (filename + ".pcd", scene_point_cloud_rgb) == 0);
        }

        return success;

    }

};


int main(int argc, char** argv) {

    using namespace yarp::os;

    Network yarp;

    if (!yarp.checkNetwork(3.0))
    {
        yError() << "YARP seems unavailable!";
        return EXIT_FAILURE;
    }

    yInfo() << "Module startup.";

    ResourceFinder rf;
    rf.setDefaultContext("tabletopSegmentation");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc, argv);

    TabletopSegmentationModule app;
    app.runModule(rf);

    yInfo() << "Module shutdown.";

    return  EXIT_SUCCESS;

}
