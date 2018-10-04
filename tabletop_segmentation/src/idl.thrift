# Copyright: (C) 2018 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Fabrizio Bottarel
#
# TableSegmentationServices.thrift

/**
 * TableSegmentationServices
 *
 * IDL Interface to \ref TableSegmentation options and services via RPC.
 */
service TableSegmentationServices
{
    /**
     * Create a .pcd file and store the point cloud
     *
     * @param dump_path filename and path where to store the point cloud. Extension .pcd will be added
     *
     * @return true/false on success/failure.
     */
    bool dumpScene(1: string dump_path);
}
