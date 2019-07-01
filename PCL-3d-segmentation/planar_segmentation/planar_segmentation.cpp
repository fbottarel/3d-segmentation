#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointCloud<pcl::PointXYZ> PC;
typedef PC::Ptr PC_ptr;

int main(int argc, char *argv[])
{

    //  Read point cloud from file
    pcl::PCDReader reader;
    PC_ptr cloud (new PC);

    if (argc < 2 || reader.read(argv[1], *cloud) < 0 )
    {
        //  If invalid or no point cloud, create new point cloud and fill the data
        //  Generate a plane with outliers
        cloud->width    = 120;
        cloud->height   = 1;
        cloud->points.resize(cloud->height * cloud->width);

        //  Generate data (plane)
        for (size_t idx = 0; idx < cloud->size(); ++idx)
        {
            cloud->points[idx].x = 5.0f * rand() / (RAND_MAX + 1.0f);
            cloud->points[idx].y = 5.0f * rand() / (RAND_MAX + 1.0f);
            cloud->points[idx].z = 1.0f * rand() / (RAND_MAX + 1.0f);
        }

        // Set a few outliers
        cloud->points[0].z = 2.0;
        cloud->points[3].z = -2.0;
        cloud->points[6].z = 4.0;
    }    

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    //  Create segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    //  Segment the point cloud and extract inliers
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        PCL_ERROR("Could not estimate planar model for data set");
        return -1;
    }


    //  Display results
    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                          << coefficients->values[1] << " "
                                          << coefficients->values[2] << " "
                                          << coefficients->values[3] << std::endl;

    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

    pcl::visualization::CloudViewer viewer("Cloud Viewer");

    viewer.showCloud(cloud);

    while(!viewer.wasStopped())
    {}


    return 0;

}
