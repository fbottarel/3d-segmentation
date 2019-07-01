#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ point_xyz;
typedef pcl::PointXYZRGB point_xyzrgb;
typedef pcl::PointCloud<point_xyz> p_cloud;
typedef pcl::PointCloud<point_xyzrgb> p_cloud_rgb;

int main(int argc, char* argv[])
{          
    //  Read point cloud from file
    pcl::PCDReader reader;
    pcl::PCDWriter writer;
    p_cloud_rgb::Ptr source_cloud (new p_cloud_rgb);
    p_cloud::Ptr cloud (new p_cloud), cloud_f (new p_cloud);

    if (argc < 2 || reader.read(argv[1], *source_cloud) < 0 )
        return -1;

    pcl::copyPointCloud(*source_cloud, *cloud);
    std::cout << "Unfiltered point cloud has " << cloud->points.size() << " data points." << std::endl;

    //  Downsample the point set with 1cm min leaf size
    pcl::VoxelGrid<point_xyz> vg;
    p_cloud::Ptr cloud_filtered (new p_cloud);
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.005f, 0.005f, 0.005f);
    vg.filter(*cloud_filtered);
    std::cout << "Filtered point cloud has " << cloud_filtered->points.size() << " data points." << std::endl;

    //  Prepare planar object for segmentation
    pcl::SACSegmentation<point_xyz> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr plane_coefficients (new pcl::ModelCoefficients);
    p_cloud::Ptr cloud_plane (new p_cloud);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.015);

    //  Filter out 70% of the pixels as planes (apparently, this is a weak assumption)
    bool plane_filtering_over = false;

    //while (cloud_filtered->points.size() > 0.3 * nr_points)
    while (!plane_filtering_over)
    {
        int nr_points = cloud_filtered->points.size();
        //  segment largest planar component from the remaining unfiltered pc
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *plane_coefficients);
        if (inliers->indices.size () == 0)
        {
          std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
          break;
        }

        if (inliers->indices.size() > 0.5 * nr_points)
        {
            //  Remove points that fall on the plane from the point cloud
            pcl::ExtractIndices<point_xyz> extract;
            extract.setInputCloud(cloud_filtered);
            extract.setIndices(inliers);
            extract.setNegative(false);

            //  Extract the planar inliers
            extract.filter(*cloud_plane);
            std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

            //   Extract non-planar inliers
            extract.setNegative(true);
            extract.filter(*cloud_f);
            *cloud_filtered = *cloud_f;
        }
        else
        {
            //  Nothing else to eliminate, go on to next phase
            plane_filtering_over = true;
        }

    }

    //  Create the kdtree object for the extraction of clusters
    pcl::search::KdTree<point_xyz>::Ptr tree (new pcl::search::KdTree<point_xyz>);
    tree->setInputCloud(cloud_filtered);

    //  Set up parameters for euclidean clustering
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<point_xyz> ec;
    ec.setClusterTolerance(0.01);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    //  Initialize viewer
    pcl::visualization::PCLVisualizer viewer;
    int total_viewport = 0;
    int segmented_viewport = 1;
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, total_viewport);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, segmented_viewport);
    viewer.addPointCloud<point_xyzrgb>(source_cloud, "Complete point cloud", total_viewport);

    //  Dump each cluster as a standalone point cloud file
    int j=0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it!= cluster_indices.end(); ++it)
    {
        pcl::PointCloud<point_xyz>::Ptr cloud_cluster (new pcl::PointCloud<point_xyz>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        std::stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";
        //writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false);
        //viewer.showCloud(cloud_cluster, ss.str());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_colored (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(*cloud_cluster, *cluster_colored);
        pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZRGB> colored_cloud_handler(cluster_colored);
        viewer.addPointCloud<point_xyzrgb> (cluster_colored, colored_cloud_handler, ss.str(), segmented_viewport);
        j++;

    }

    viewer.addCoordinateSystem(0.1, "ref_base", total_viewport);
    viewer.addCoordinateSystem(0.1, "ref_base", segmented_viewport);

    viewer.spin();

    return 0;

}
