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
	SeqLoader(string path)
	{
        int i = 0;
        while (true)
        {
            string file_path = path + "/" + "frame-"+makeFixedLength(i,6)+".bin";//frame-000000.bin
            pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            //if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/full/seq-01/frame-000000.bin", *target_cloud) == -1)
            if (LoadBinFile(file_path, target_cloud) == -1)
            {
                break;
            }
            else
            {
                frames.push_back(target_cloud);
                i++;
            }
        }
	}
private:
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
};