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

using namespace std::chrono_literals;

int LoadBinFile(std::string in_file, pcl::PointCloud<pcl::PointXYZ>::Ptr points)
{
    // load point cloud file.
    std::fstream input(in_file.c_str(), std::ios::in | std::ios::binary);// in_file 为点云bin文件
    if (!input.good())
    {
        std::cerr << "Could not read file: " << in_file << std::endl;
        return -1;
    }
    input.seekg(0, std::ios::beg);

    //pcl::PointCloud<pcl::PointXYZI>::Ptr points(new pcl::PointCloud<pcl::PointXYZI>);// 点云存储解析bin后的数据

    for (int i = 0; input.good() && !input.eof(); i++)
    {
        pcl::PointXYZI point;
        input.read((char*)&point.x, 3 * sizeof(float));
        input.read((char*)&point.intensity, sizeof(float));

        points->push_back(point);
    }
    input.close();
    return 0;

}

int main() {
    // Loading first scan of room.
    // 加载首次的房间扫描数据作为目标点云 target_cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/full/seq-01/frame-000000.bin", *target_cloud) == -1)
    if (LoadBinFile("../data/full/seq-01/frame-000000.bin", *target_cloud) == -1)
    {
        PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
        return (-1);
    }
    std::cout << "Loaded " << target_cloud->size() << " data points from room_scan1.pcd" << std::endl;

    // Loading second scan of room from new perspective.
    // 加载从新的视角得到的房间第二次扫描数据作为输入源点云 input_cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/qiuju/CLionProjects/icp_code_practice/room_scan2.pcd", *input_cloud) == -1) {
        PCL_ERROR ("Couldn't read file room_scan2.pcd \n");
        return (-1);
    }
    std::cout << "Loaded " << input_cloud->size() << " data points from room_scan2.pcd" << std::endl;
    // The above code loads the two pcd file into pcl::PointCloud<pcl::PointXYZ> boost shared pointers.
    // The input cloud will be transformed into the reference frame of the target cloud.
    // 以上代码加载了两个 PCD 文件到共享指针，
    // 配准操作是完成 【源点云input_cloud】到【目标点云target_cloud】坐标系变换矩阵的估算，即求出input_cloud变换到target_cloud的变换矩阵

    // Filtering input scan to roughly 10% of original size to increase speed of registration.
    // 过滤输入点云到约10%的原始大小，以提高配准速度。
    // 以上代码将过滤输入点云到约10%的原始大小，以提高配准速度。
    // 这里用任何其他均匀过滤器都可以，目标点云target_cloud不需要进行滤波处理，因为NDT算法在目标点云对应的体素Voxel网格数据结构计算时，
    // 不使用单个点，而是使用体素的点。即已做了降采样处理。
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // https://blog.csdn.net/Small_Munich/article/details/108348164
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    // 设置每个体素的大小, leaf_size没别为lx ly lz的长 宽 高
    approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
    approximate_voxel_filter.setInputCloud(input_cloud);
    approximate_voxel_filter.filter(*filtered_cloud);
    std::cout << "Filtered cloud contains " << filtered_cloud->size() << " data points from room_scan2.pcd" << std::endl;
    /**
     * This section filters the input cloud to improve registration time.
     * Any filter that downsamples the data uniformly can work for this section.
     * The target cloud does not need be filtered because voxel grid data structure used by the NDT algorithm does not use individual points,
     * but instead uses the statistical data of the points contained in each of its data structures voxel cells.
     */

    // Initializing Normal Distributions Transform (NDT).
    // 初始化正态分布变化NDT对象
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

    // Here we create the NDT algorithm with the default values.
    // The internal data structures are not initialized until later.

    // Setting scale dependent NDT parameters
    // Setting minimum transformation difference for termination condition.
    // 前一个变换矩阵和当前变换矩阵的差异小于阈值时，就认为已经收敛了，是一条收敛条件；
    // setEuclideanFitnessEpsilon， 还有一条收敛条件是均方误差和小于阈值， 停止迭代。
    // 根据输入数据的尺度设置NDT相关参数
    ndt.setTransformationEpsilon(0.01);
    // Setting maximum step size for More-Thuente line search.
    // 步长大小
    ndt.setStepSize(0.1);
    // Setting Resolution of NDT grid structure (VoxelGridCovariance).
    // 网格分辨率Resolution
    ndt.setResolution(1.0);

    // Setting max number of registration iterations.
    // 设置匹配迭代的最大次数
    // 这个MaximumIterations参数控制了优化程序运行的最大迭代次数，一 般来说，
    // 在达到这个限制值之前优化程序就会在 epsilon 变换阈值下终止。
    // 添加此最大迭代次数限制能够增加程序鲁棒性,阻止了它在错误的方向运行太久。
    ndt.setMaximumIterations(35);

    /**
     * Next we need to modify some of the scale dependent parameters.
     * Because the NDT algorithm uses a voxelized data structure and More-Thuente line search,
     * some parameters need to be scaled to fit the data set.
     * The above parameters seem to work well on the scale we are working with, size of a room,
     * but they would need to be significantly decreased to handle smaller objects, such as scans of a coffee mug.
     * The Transformation Epsilon parameter defines minimum, allowable, incremental change of the transformation vector,
     * [x, y, z, roll, pitch, yaw] in meters and radians respectively.
     * Once the incremental change dips below this threshold, the alignment terminates.
     * The Step Size parameter defines the maximum step length allowed by the More-Thuente line search.
     * This line search algorithm determines the best step length below this maximum value,
     * shrinking the step length as you near the optimal solution.
     * Larger maximum step lengths will be able to clear greater distances in fewer iterations but
     * run the risk of overshooting and ending up in an undesirable local minimum. Finally,
     * the Resolution parameter defines the voxel resolution of the internal NDT grid structure.
     * This structure is easily searchable and each voxel contain the statistical data, mean,
     * covariance, etc., associated with the points it contains.
     * The statistical data is used to model the cloud as a set of multivariate Gaussian distributions and allows us to calculate and
     * optimize the probability of the existence of points at any position within the voxel.
     * This parameter is the most scale dependent.
     * It needs to be large enough for each voxel to contain at least 6 points but small enough to uniquely describe the environment.
     *
     * This parameter controls the maximum number of iterations the optimizer can run.
     * For the most part, the optimizer will terminate on the Transformation Epsilon before hitting this
     * limit but this helps prevent it from running for too long in the wrong direction.
     *
     * 这里设置一些尺度相关的参数,因为 NDT 算法使用一个体素化数据结构和More-Thuente 线搜索，因此需要缩放一些参数来适应数据集。
     * 以上参数看起来在我们使用的房间尺寸比例下运行地很好，但是它们如果需要处理例如一个咖啡杯的扫描之类更小物体，需要对参数进行很大程度的缩小。
     * 在变换中 Epsilon 参数分别从长度和弧度，定义了变换矢量[ x, y, z,roll,pitch, yaw]的最小许可的递增量，一旦递增量减小到这个临界值以下 ，
     * 那么配准算法就将终止。步长StepSize参数定义了 More-Thuente 线搜索允许的最大步长，这个线搜索算法确定了最大值以下的最佳步长，
     * 当靠近最优解时该算法会缩短迭代步长，在更大的最大步长将会在较少的迭代次数下遍历较大的距离，但是却有过度迭代和在不符合要求的局部最小值处结束的风险。
    */

    // Setting point cloud to be aligned. 设置过滤后的输入源点云（第二次扫描数据）
    ndt.setInputSource(filtered_cloud);
    // Setting point cloud to be aligned to. 设置目标点云（第一次扫描数据），作为对其的目标。
    ndt.setInputTarget(target_cloud);

    /**
     * Here, we pass the point clouds to the NDT registration program.
     * The input cloud is the cloud that will be transformed and the target cloud is the reference frame to which the input cloud will be aligned.
     * When the target cloud is added, the NDT algorithm’s internal data structure is initialized using the target cloud data.
     *
     * 这里，我们把点云赋给 NDT 配准对象，目标点云的坐标系是被匹配的输入点云的参考坐标系，
     * 匹配完成后输入点云将被变换到与目标点云统一坐标系下，当加载目标点云后，NDT 算法的内部数据结构被初始化。
     */

    // Set initial alignment estimate found using robot odometry.
    // 绕　Z 轴旋转　0.6931　的角度
    Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
    // 平移向量
    Eigen::Translation3f init_translation(1.79387, 0.720047, 0);
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

    /**
     * In this section of code, we create an initial guess about the transformation needed to align the point clouds.
     * Though the algorithm can be run without such an initial transformation, you tend to get better results with one,
     * particularly if there is a large discrepancy between reference frames.
     * In robotic applications, such as the ones used to generate this data set,
     * the initial transformation is usually generated using odometry data.
     * 在这部分的代码块中我们创建了一个点云配准变换矩阵的初始估计，虽然算法运行并不需要这样的一个初始变换矩阵，
     * 但是有了它易于得到更好的结果，尤其是当参考坐标系之间有较大差异时（本例即是），
     * 在机器人应用程序（例如用于生成此数据集的应用程序）中，通常使用里程表数据生成初始转换。
     */

    // Calculating required rigid transform to align the input cloud to the target cloud.
    // 计算所需的刚体变换，以使输入云与目标云对齐。
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    ndt.align(*output_cloud, init_guess);
    // 最后，我们准备对齐点云。生成的转换后的输入云存储在输出云中。
    // 然后，我们显示对齐的结果以及欧几里得适合度得分FitnessScore，该分数计算为从输出云到目标云中最近点的距离的平方

    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged() << " score: " << ndt.getFitnessScore() << std::endl;

    /**
     * Finally, we are ready to align the point clouds.
     * The resulting transformed input cloud is stored in the output cloud.
     * We then display the results of the alignment as well as the Euclidean fitness score,
     * calculated as the sum of squared distances from the output cloud to the closest point in the target cloud.
     */
    // Transforming unfiltered, input cloud using found transform.
    // 使用找到的变换矩阵，来对未过滤的输入云进行变换。
    pcl::transformPointCloud(*input_cloud, *output_cloud, ndt.getFinalTransformation());

    // Saving transformed input cloud. 保存变换后的输入点云
    pcl::io::savePCDFileASCII("room_scan2_transformed.pcd", *output_cloud);

    /**
     * Immediately after the alignment process,
     * the output cloud will contain a transformed version of the filtered input cloud because we passed the algorithm a filtered point cloud,
     * as opposed to the original input cloud. To obtain the aligned version of the original cloud,
     * we extract the final transformation from the NDT algorithm and transform our original input cloud.
     * We can now save this cloud to file room_scan2_transformed.pcd for future use.
     *
     * 在对齐之后，输出云output_cloud将立即包含过滤后的输入云的转换版本，因为我们向算法传递了过滤后的点云，而不是原始输入云。
     * 为了获得原始云的对齐版本，我们从NDT算法中提取最终转换矩阵并转换原始输入云。
     * 现在，我们可以将该云保存到文件room_scan2_transformed.pcd中，以备将来使用。
     */
    // Initializing point cloud visualizer
    pcl::visualization::PCLVisualizer::Ptr viewer_final(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer_final->setBackgroundColor(0, 0, 0);

    // Coloring and visualizing target cloud (red).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target_cloud, 255, 0, 0);
    viewer_final->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");

    // Coloring and visualizing transformed input cloud (green).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_color(output_cloud, 0, 255, 0);
    viewer_final->addPointCloud<pcl::PointXYZ>(output_cloud, output_color, "output cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "output cloud");

    // Starting visualizer
    viewer_final->addCoordinateSystem(1.0, "global");
    viewer_final->initCameraParameters();

    // Wait until visualizer window is closed.
    while (!viewer_final->wasStopped()) {
        viewer_final->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }

    /**
     * This next part is unnecessary but I like to visually see the results of my labors.
     * With PCL’s visualizer classes, this can be easily accomplished.
     * We first generate a visualizer with a black background.
     * Then we colorize our target and output cloud, red and green respectively,
     * and load them into the visualizer.
     * Finally we start the visualizer and wait for the window to be closed.
     */
    return (0);
}