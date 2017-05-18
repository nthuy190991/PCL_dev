#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

int main (int argc, char* argv[]){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  
  if (!strcmp(argv[2],"ply")){
	  if (pcl::io::loadPLYFile<pcl::PointXYZ> (argv[1], *cloud) == -1){
		PCL_ERROR("Couldn't read file %s\n",argv[1]);
		return(-1);
	  }
  }
  else if (!strcmp(argv[2],"pcd")){
	  if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1){
		PCL_ERROR("Couldn't read file %s\n",argv[1]);
		return(-1);
	  }
  }
  else{
	PCL_ERROR("argument: <filepath> <extension>\n");
	return(-1);
  }

  std::cout << "Loaded "
            << cloud->width*cloud->height
            << " data points from " << argv[1] << " with the following fields: "
            << std::endl;

  for (size_t i=0; i<cloud->points.size(); ++i){
    std::cout << "    " << cloud->points[i].x
              << "    " << cloud->points[i].y
              << "    " << cloud->points[i].z << std::endl;  
  }

  return(0);
}
