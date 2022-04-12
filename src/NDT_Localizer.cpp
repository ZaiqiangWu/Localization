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
#include "nlohmann/json.hpp"
using json = nlohmann::json;

using namespace std::chrono_literals;
using namespace std;
void printMatrix(Eigen::Matrix4f mat);
void evaluateError(Eigen::Matrix4f a, Eigen::Matrix4f b, float& error_pos, float& error_orien);
Eigen::Matrix4f ndt_Localize(Eigen::Matrix4f init_guess, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target);
void ndt_relocalization();
void json_test();
float get_median(vector<float> datas);
float get_mean(vector<float> datas);

int main()
{
    
    json_test();
    ndt_relocalization();
    
    return 0;
}
void json_test()
{
    json j;

    // add a number that is stored as double (note the implicit conversion of j to an object)
    j["pi"] = 3.141;

    // add a Boolean that is stored as bool
    j["happy"][0] = true;
    j["happy"][1] = false;

    // add a string that is stored as std::string
    j["name"] = "Niels";

    // add another null object by passing nullptr
    j["nothing"] = nullptr;

    // add an object inside the object
    j["answer"]["everything"] = 42;

    // add an array that is stored as std::vector (using an initializer list)
    vector<float> datas = { 1, 0, 2, 6 };
    j["list"] = datas;
    std::ofstream o("pretty.json");
    o << std::setw(4) << j << std::endl;
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
        viewer.showCloud(seqloader.frames[19]);

        // Wait until visualizer window is closed.
        while (!viewer.wasStopped()) {
            //viewer.spinOnce(100);
            std::this_thread::sleep_for(100ms);
        }
    }
    

    json log;
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
        log["gt"][i] = {seqloader.poses[i](0,3),seqloader.poses[i](1,3), seqloader.poses[i](2,3) };
        log["pred"][i] = { estimate_poses[i](0,3),estimate_poses[i](1,3), estimate_poses[i](2,3) };
        pos_error.push_back(pe);
        orien_error.push_back(oe);
        log["location error"][i] = pe;
        log["rotation error"][i] = oe;
        cout << "mean pos error: " << get_mean(pos_error) << endl;
        cout << "mean orien error: " << get_mean(orien_error) << endl;
        cout << "elapsed time: " << t_end-t_statr << "s" << endl;

        log["elapsed time"][i] = t_end - t_statr;
        
    }
    std::ofstream o("pretty.json");
    o << std::setw(4) << log << std::endl;

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

float get_mean(vector<float> datas)
{
    float result = 0.0f;
    for (int i = 0; i < datas.size(); i++)
    {
        result += datas[i];
    }
    result /= datas.size();
    return result;
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
    float dot_product = 0.0f;
    float a_norm = 0.0f;
    float b_norm = 0.0f;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            dot_product += a(i, j) * b(i, j);
            a_norm += a(i, j) * a(i, j);
            b_norm += b(i, j) * b(i, j);
        }
    }
    float cos_value = dot_product / sqrt(a_norm * b_norm);
    error_orien = acos(cos_value) / M_PI * 180.0f;
    
}

Eigen::Matrix4f ndt_Localize(Eigen::Matrix4f init_guess,pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr target)
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
    ndt.setTransformationEpsilon(0.00001);
    // Setting maximum step size for More-Thuente line search.
    ndt.setStepSize(0.1);
    //Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt.setResolution(1);

    // Setting max number of registration iterations.
    ndt.setMaximumIterations(70);


    // Setting point cloud to be aligned to.
    ndt.setInputTarget(target);
    // Setting point cloud to be aligned.
    ndt.setInputSource(filtered_cloud);
    
    
    // Calculating required rigid transform to align the input cloud to the target cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    ndt.align(*output_cloud, init_guess);

    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
        << " score: " << ndt.getFitnessScore() << std::endl;
    return ndt.getFinalTransformation();
}