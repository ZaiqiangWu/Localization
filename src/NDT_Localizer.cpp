/**
 * These are the required header files to use Normal Distributions Transform algorithm and
 * a filter used to down sample the data.
 * The filter can be exchanged for other filters but
 * I have found the approximate voxel filter to produce the best results.
 */
#include <iostream>
#include <thread>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <SeqLoader.h>
#include<cmath>
#include "timer.h"
#include <algorithm>

using namespace std::chrono_literals;
using namespace std;
void printMatrix(Eigen::Matrix4f mat);
void evaluateError(Eigen::Matrix4f a, Eigen::Matrix4f b, float& error_pos, float& error_orien);
Eigen::Matrix4f ndt_Localize(Eigen::Matrix4f init_guess, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud);
void ndt_relocalization();
void ndt_slam();
float get_median(vector<float> datas);

int main()
{
    
    ndt_relocalization();
    
    return 0;
}
void ndt_slam()
{

}

void ndt_relocalization()
{
    //pcl::PointCloud<pcl::PointXYZ>::Ptr  globalPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    SeqLoader seqloader("../data/full/seq-07");

    bool is_visualize = false;
    if (is_visualize)
    {
        // Initializing point cloud visualizer
        pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
        viewer.showCloud(seqloader.globalPointCloud);

        // Wait until visualizer window is closed.
        while (!viewer.wasStopped()) {
            //viewer.spinOnce(100);
            std::this_thread::sleep_for(100ms);
        }
    }


    cout << "Global Point Cloud has " << seqloader.globalPointCloud->size() << " points" << endl;

    Eigen::Matrix4f init_guess = seqloader.poses[0];
    vector<Eigen::Matrix4f> estimate_poses;
    vector<float> pos_error;
    vector<float> orien_error;
    Timer timer;
    for (int i = 0; i < seqloader.frames.size(); i++)
    {
        cout << "Step: " << i << "/" << seqloader.frames.size() << endl;
        float t_statr = timer.GetTime();
        estimate_poses.push_back(ndt_Localize(init_guess, seqloader.frames[i], seqloader.globalPointCloud));
        float t_end = timer.GetTime();
        init_guess = estimate_poses[i];
        float pe = 0;
        float oe = 0;
        evaluateError(estimate_poses[i], seqloader.poses[i], pe, oe);
        pos_error.push_back(pe);
        orien_error.push_back(oe);
        cout << "median pos error: " << get_median(pos_error) << endl;
        cout << "median orien error: " << get_median(orien_error) << endl;
        cout << "elapsed time: " << t_end-t_statr << "s" << endl;
    }

}

float get_median(vector<float> datas)
{
    sort(datas.begin(), datas.end());
    int size = datas.size();
    if (size% 2 == 0)
    {
        return (datas[size/2-1]+datas[size/2])*0.5f;
    }
    else
    {
        return datas[size / 2];
    }
}

void printMatrix(Eigen::Matrix4f mat)
{
    
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            cout << mat(i, j) << ",";
        }
        cout << endl;
    }
}

void evaluateError(Eigen::Matrix4f a,Eigen::Matrix4f b,float &error_pos,float &error_orien)
{

    Eigen::Vector3f a_pos(a(0,3), a(1,3), a(2,3));
    Eigen::Vector3f b_pos(b(0, 3), b(1, 3), b(2, 3));
    error_pos = (a_pos - b_pos).norm();
    Eigen::Vector3f a_orien(a(0, 0), a(1, 0), a(2, 0));
    Eigen::Vector3f b_orien(b(0, 0), b(1, 0), b(2, 0));
    error_orien = acos(a_orien.dot(b_orien)) / M_PI * 180.0f;
    
}

Eigen::Matrix4f ndt_Localize(Eigen::Matrix4f init_guess,pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud)
{
    // Filtering input scan to roughly 10% of original size to increase speed of registration.
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    float size = 0.2;
    approximate_voxel_filter.setLeafSize(size,size,size);
    approximate_voxel_filter.setInputCloud(input_cloud);
    approximate_voxel_filter.filter(*filtered_cloud);
    std::cout << "Filtered cloud contains " << filtered_cloud->size()
        << " data points" << std::endl;

    // Initializing Normal Distributions Transform (NDT).
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

    // Setting scale dependent NDT parameters
    // Setting minimum transformation difference for termination condition.
    ndt.setTransformationEpsilon(0.001);
    // Setting maximum step size for More-Thuente line search.
    ndt.setStepSize(0.1);
    //Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt.setResolution(1.0);

    // Setting max number of registration iterations.
    ndt.setMaximumIterations(70);

    // Setting point cloud to be aligned.
    ndt.setInputSource(filtered_cloud);
    // Setting point cloud to be aligned to.
    ndt.setInputTarget(target_cloud);

    
    // Calculating required rigid transform to align the input cloud to the target cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    ndt.align(*output_cloud, init_guess);

    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
        << " score: " << ndt.getFitnessScore() << std::endl;
    return ndt.getFinalTransformation();
}