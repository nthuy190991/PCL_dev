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

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
  
	// Read the cloud data
	if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read file %s \n", argv[1]);
		return (-1);
	}
	std::cout << "Loaded "
			<< cloud->width * cloud->height
			<< " data points from " << argv[1]	<< " with the following fields: "
			<< std::endl;
  
	/*
	std::cerr << "Cloud before projection: " << std::endl;
	for (size_t i=0; i<cloud->points.size(); ++i)
	std::cerr <<  "   " << cloud->points[i].x << " "
						<< cloud->points[i].y << " "
						<< cloud->points[i].z << std::endl;
	*/

  
	// Create a set of planar coefficients with X=Y=0, Z=1
	// ax+by+cz+d=0 where a=b=d=0, c=1, or the XY plane
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[0] = 0;
	coefficients->values[1] = 0;
	coefficients->values[2] = 1.0;
	coefficients->values[3] = 0;

	// Create the filtering object
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(cloud);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cloud_projected);

	cout << "PROJECTION DONE" << std::endl;

	/*std::cerr << "Cloud after projection: " << std::endl;
	for (size_t i=0; i<cloud_projected->points.size(); ++i)
	std::cerr <<  "   " << cloud_projected->points[i].x << " "
						<< cloud_projected->points[i].y << " "
						<< cloud_projected->points[i].z << std::endl;*/

	pcl::io::savePCDFileASCII(argv[2], *cloud_projected);
	std::cerr << "Saved " << cloud_projected->points.size () << " projected data points to " << argv[2] << std::endl;
  
	return(0);
}
