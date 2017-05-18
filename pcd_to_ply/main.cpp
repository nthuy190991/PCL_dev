#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>


int main(int argc, char** argv){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (!strcmp(argv[1],"ply2pcd")){
    if (pcl::io::loadPLYFile<pcl::PointXYZ> (argv[2], *cloud) == -1) //* load the file
    {
  	  PCL_ERROR ("Couldn't read file %s \n", argv[2]);
  	  return (-1);
    }
    std::cerr << "Read from " << argv[2] << std::endl;

    pcl::io::savePCDFileASCII(argv[3], *cloud);
    std::cerr << "Converted to " << argv[3] << std::endl;
  }

  else if (!strcmp(argv[1],"pcd2ply")){
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[2], *cloud) == -1) //* load the file
    {
  	  PCL_ERROR ("Couldn't read file %s \n", argv[2]);
  	  return (-1);
    }
    std::cerr << "Read from " << argv[2] << std::endl;

    pcl::io::savePLYFileASCII(argv[3], *cloud);
    std::cerr << "Converted to " << argv[3] << std::endl;
  }
  return(0);
}
