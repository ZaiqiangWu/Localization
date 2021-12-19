#pragma once
#include <string>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
using namespace std;
using namespace std::chrono_literals;
class SeqLoader
{
public:
    vector< pcl::PointCloud<pcl::PointXYZ>::Ptr> frames;
    vector< Eigen::Matrix4f> poses;
    pcl::PointCloud<pcl::PointXYZ>::Ptr globalPointCloud;
	SeqLoader(string path)
	{
        int i = 0;
        while (true)
        {
            string frame_path = path + "/" + "frame-"+makeFixedLength(i,6)+".bin";//frame-000000.bin
            string pose_path = path + "/" + "frame-" + makeFixedLength(i, 6) + ".pose.txt";
            pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            //if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/full/seq-01/frame-000000.bin", *target_cloud) == -1)
            if (LoadBinFile(frame_path, target_cloud) == -1)
            {
                break;
            }
            else
            {
                frames.push_back(target_cloud);
                poses.push_back(readMatrix(pose_path.c_str()));
                i++;
            }
        }
        globalPointCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        for (int i = 0; i < frames.size(); i++)
        {
            pcl::PointCloud<pcl::PointXYZ> temp;
            pcl::transformPointCloud(*frames[i], temp, poses[i]);
            *globalPointCloud += temp;
        }
	}
private:
    int LoadBinFile(std::string in_file, pcl::PointCloud<pcl::PointXYZ>::Ptr points)
    {
        // load point cloud file.
        std::fstream input(in_file.c_str(), std::ios::in | std::ios::binary);// in_file 为点云bin文件
        if (!input.good())
        {
            std::cerr << "Finish loading data"<< std::endl;
            return -1;
        }
        input.seekg(0, std::ios::beg);

        //pcl::PointCloud<pcl::PointXYZI>::Ptr points(new pcl::PointCloud<pcl::PointXYZI>);// 点云存储解析bin后的数据

        for (int i = 0; input.good() && !input.eof(); i++)
        {
            pcl::PointXYZ point;
            input.read((char*)&point.x, 3 * sizeof(float));
            //input.read((char*)&point.intensity, sizeof(float));

            points->push_back(point);
        }
        input.close();
        return 0;

    }
    std::string makeFixedLength(const int i, const int length)
    {
        std::ostringstream ostr;

        if (i < 0)
            ostr << '-';

        ostr << std::setfill('0') << std::setw(length) << (i < 0 ? -i : i);

        return ostr.str();
    }
    Eigen::Matrix4f readMatrix(const char filename[])
    {
        Eigen::Matrix4f mat;
        ifstream mmatrix_file(filename);
        string line;
        string num;
        int pos;
        int row = 0;
        int col = 0;

        while (mmatrix_file >> line)
        {
            for (int p = 0; p < 4; p++)
            {
                pos = line.find(",");
                num = line.substr(0, pos);
                line.erase(0, pos + 1);

                stringstream x(num);
                float number = 0;
                x >> number;

                mat(row,p) = number;
                ++col;
            }
            ++row;
        }
        //printMatrix(mat);
        return mat;
    }
    void printMatrix(Eigen::Matrix4f mat)
    {
        cout << "mat:" << endl;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                cout << mat(i, j) << ",";
            }
            cout << endl;
        }
    }
};