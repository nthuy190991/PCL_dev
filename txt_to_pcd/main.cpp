#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <string>

#include <sstream>
#include <vector>
#include <iterator>

template<typename Out>
void split(const std::string &s, char delim, Out result) {
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        *(result++) = item;
    }
}

std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
}

int main(int argc, char** argv){

	size_t NB_PTS = ::atof(argv[3]);

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

	cloud->width  = NB_PTS;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	std::ifstream file(argv[1]);
	std::string str;

	size_t i=0;
	while ((std::getline(file, str))&&(i<NB_PTS))
	{
  		std::vector<std::string> x = split(str, ',');

  		cloud->points[i].x = ::atof(x[1].c_str());
  		cloud->points[i].y = ::atof(x[2].c_str());
  		cloud->points[i].z = ::atof(x[3].c_str());
		cloud->points[i].intensity = ::atof(x[4].c_str());
  		i=i+1;
	}
	cout << "READ TXT DONE" << std::endl;

	pcl::io::savePCDFileASCII(argv[2], *cloud);
	std::cerr << "Saved " << cloud->points.size () << " data points to " << argv[2] << std::endl;

	return(0);
}
