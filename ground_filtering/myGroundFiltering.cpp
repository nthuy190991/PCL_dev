#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/octree/octree.h>

#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>

typedef pcl::PointXYZI PointTypeIO;
typedef pcl::PointXYZINormal PointTypeFull;

using namespace cv;
using namespace pcl;

float myThreshHeight = 2.7;

void ConvertXYZI2XYZ(PointCloud<PointXYZI>::Ptr cloud_in, PointCloud<PointXYZ>::Ptr cloud_out)
{
	PointCloud<PointXYZI>::iterator it = cloud_in->begin();
	PointCloud<PointXYZI>::iterator itend = cloud_in->end();

	PointXYZ point;
	for( ;it!=itend; it++)
	{
		point.x = it->x ;
		point.y = it->y ;
		point.z = it->z ;
		cloud_out->points.push_back (point);
	}
	cloud_out->width = (int) cloud_in->points.size ();
	cloud_out->height = 1;

	return;

}

void SelectGroundPoints(PointCloud<PointXYZ>::Ptr cloud_in, PointCloud<PointXYZ>::Ptr cloud_out)
{
	PointCloud<PointXYZ>::iterator it = cloud_in->begin();
	PointCloud<PointXYZ>::iterator itend = cloud_in->end();

	int nbPts = 0;
	for( ;it!=itend; it++)
	{
		if(it->z < myThreshHeight)
		{
			PointXYZ point;
			point.x = it->x ;
			point.y = it->y ;
			point.z = it->z ;
			cloud_out->points.push_back (point);
			nbPts++;
		}
	}
	cloud_out->width = nbPts;
	cloud_out->height = 1;

	std::cerr << ">> Ground: " << cloud_out->points.size () << " points\n";

	return;

}

void my3DRasterize(PointCloud<PointXYZI>::Ptr cloud, PointCloud<PointXYZRGB>::Ptr cloud_RGB)
{
	//-----------------------------------------------------------------------------------------
	//3DRasterize
	//-----------------------------------------------------------------------------------------
	double grid_x;
	double grid_y;
	double length_x;
	double length_y;
	double search_min_x = 0;
	double search_min_y = 0;
	double search_max_x = 0;
	double search_max_y = 0;
	int count_points = 0;
	double average_intesity = 0;

	int cloud_slash_intervalx;
	int cloud_slash_intervaly;

	int rasterSizeX, rasterSizeY;

	PointCloud<PointXYZI>::Ptr cloud_projected(new PointCloud<pcl::PointXYZI>);

	// Create a set of planar coefficients with X=Y=0,Z=1
	std::cout << "Debut Planar projection point clouds.\n";
	ModelCoefficients::Ptr coefficients(new ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[0] = 0;
	coefficients->values[1] = 0; // 0;
	coefficients->values[2] = 1.0; //1.0;
	coefficients->values[3] = 0;

	// Create the filtering object
	ProjectInliers<pcl::PointXYZI> proj;
	proj.setModelType(SACMODEL_PLANE);
	proj.setInputCloud(cloud);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cloud_projected);
	std::cout << "Fin Planar projection point clouds.\n\n";

	//-----------------------------------------------------------------------------------------------------------------------------------------------
	// OcTree

	std::cout << "Debut Octree.\n";
	PointXYZI min_point, max_point;
	getMinMax3D(*cloud_projected, min_point, max_point);

	length_x = max_point.x - min_point.x;
	length_y = max_point.y - min_point.y;

	rasterSizeX = cloud_slash_intervalx = (int)floor(length_x*4);
	rasterSizeY = cloud_slash_intervaly = (int)floor(length_y*4);

	printf("Planar projection: min_x = %f  max_x = %f \n", min_point.x, max_point.x);
	printf("Planar projection: min_y = %f  max_y = %f \n", min_point.y, max_point.y);
	printf("Planar projection: min_z = %f  max_z = %f \n", min_point.z, max_point.z);
	printf("Planar projection: raster Size_x = %d  raster Size_y = %d \n", rasterSizeX, rasterSizeY);

	// create window for raster
	char intensity_raster_window[] = "Drawing Window";
	/// Create black empty images
	Mat intensity_raster_image = Mat::zeros(rasterSizeX, rasterSizeY, CV_8UC3);

	grid_x = length_x / (cloud_slash_intervalx * 1.0f);
	grid_y = length_y / (cloud_slash_intervaly * 1.0f);

	printf("Planar projection: grid_x = %f  grid_y = %f \n", grid_x, grid_y);

	double minpx = min_point.x;
	double minpy = min_point.y;
	double minpz = min_point.z;
	double maxpx = grid_x;
	double maxpy = grid_y;
	double maxpz = max_point.z;

	Eigen::Vector3f minbox(3, 1); //(min_point.x, min_point.y, min_point.z);
	minbox(0, 0) = minpx;
	minbox(1, 0) = minpy;
	minbox(2, 0) = minpz;

	//Eigen::Vector3f maxbox(max_point.x, max_point.y, max_point.z);
	Eigen::Vector3f maxbox(3, 1); //(grid_x, grid_y, max_point.z);
	maxbox(0, 0) = max_point.x;
	maxbox(1, 0) = max_point.y;
	maxbox(2, 0) = max_point.z;

	//std::vector<float> minbox(min_point.x, min_point.y, min_point.z);
	std::vector<int> OcTreeBoxIndicies;

	//float resolution = 128.0f; //size_t i = 0; int j = 0;
	float resolution = grid_x;

	pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree(resolution);

	octree.setInputCloud(cloud_projected);
	octree.addPointsFromInputCloud();

	std::cout << "Fin Octree.\n\n";

	
	// intensity operations for establish way of colorizing a raster
	float act_intensity;
	float min_intensity = std::numeric_limits<float>::max();
	float max_intensity = std::numeric_limits<float>::min();
	float norm_i; // norm_i = (act_intensity-min_intensity)/(max_intensity-act_intenisty);
	int max_color = 255;
	int min_color = 0;
	int scale_color; // scale_color =(norm_i*max_color) + ((1-norm_i)*min_color);

	// establish min and max intensity value in the cloud
	std::cout << "Debut establish min and max intensity value in the cloud.\n";

	/*octree.boxSearch(minbox, maxbox, OcTreeBoxIndicies);
	for (size_t idx = 0; idx < OcTreeBoxIndicies.size(); ++idx)
	{
		min_intensity = (std::min) (min_intensity, cloud_projected->points[OcTreeBoxIndicies[idx]].intensity);
		max_intensity = (std::max) (max_intensity, cloud_projected->points[OcTreeBoxIndicies[idx]].intensity);
	}*/
	
	//pcl::PointCloud<pcl::PointXYZI>::iterator it = cloud_projected->begin();
	//pcl::PointCloud<pcl::PointXYZI>::iterator itend = cloud_projected->end();

	pcl::PointCloud<pcl::PointXYZI>::iterator it = cloud->begin();
	pcl::PointCloud<pcl::PointXYZI>::iterator itend = cloud->end();

	for( ;it!=itend; it++)
	{
		min_intensity = (std::min) (min_intensity, it->z);
		max_intensity = (std::max) (max_intensity, it->z);
	}

	printf("min and max intensity value: min_intensity = %f  max_intensity = %f \n", min_intensity, max_intensity);
	
	minbox(0, 0) = minpx;
	minbox(1, 0) = minpy;
	minbox(2, 0) = minpz;

	maxbox(0, 0) = maxpx;
	maxbox(1, 0) = maxpy;
	maxbox(2, 0) = maxpz;

	int K = 10;
	pcl::PointXYZI searchPoint;
	std::vector<int> pointIdxNKNSearch;
	std::vector<float> pointNKNSquaredDistance;

	for (int it_py = 0; it_py < cloud_slash_intervaly; it_py++)
	{
		minpx = min_point.x;
		maxpx = grid_x;
		minbox(0, 0) = minpx;
		maxbox(0, 0) = maxpx;

		average_intesity = 0;
		count_points = 0;

		for (int it_px = 0; it_px < cloud_slash_intervalx; it_px++)
		{
			// octree box search pixels creation
			if (octree.boxSearch(minbox, maxbox, OcTreeBoxIndicies) > 0)
			{
				for (size_t p = 0; p < OcTreeBoxIndicies.size(); ++p)
				{
					count_points = count_points + 1;
					//average_intesity = average_intesity + cloud_projected->points[OcTreeBoxIndicies[p]].intensity;
					average_intesity = average_intesity + cloud->points[OcTreeBoxIndicies[p]].z;
				}
				// establish a grey scale for intecity on raster
				act_intensity = average_intesity / count_points;
				norm_i = (act_intensity - min_intensity) / (max_intensity - act_intensity);
				scale_color = (norm_i*max_color) + ((1 - norm_i)*min_color);

				intensity_raster_image.at<cv::Vec3b>(it_px, it_py)[0] = scale_color;//R
				intensity_raster_image.at<cv::Vec3b>(it_px, it_py)[1] = scale_color;//G
				intensity_raster_image.at<cv::Vec3b>(it_px, it_py)[2] = scale_color;//B
				average_intesity = 0;
				count_points = 0;
			}
			else
			{
				average_intesity = 0;
				count_points = 0;
				searchPoint.x = maxpx - (grid_x / 2);
				searchPoint.y = maxpy - (grid_y / 2);
				if (octree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
				{
					for (size_t p2 = 0; p2 < pointIdxNKNSearch.size(); ++p2)
					{
						count_points = count_points + 1;
						//average_intesity = average_intesity + cloud_projected->points[pointIdxNKNSearch[p2]].intensity;
						average_intesity = average_intesity + cloud->points[pointIdxNKNSearch[p2]].z;
					}
					// establish a grey scale for intecity on raster
					act_intensity = average_intesity / count_points;// K;
					norm_i = (act_intensity - min_intensity) / (max_intensity - act_intensity);
					scale_color = (norm_i*max_color) + ((1 - norm_i)*min_color);

					intensity_raster_image.at<cv::Vec3b>(it_px, it_py)[0] = scale_color;//R
					intensity_raster_image.at<cv::Vec3b>(it_px, it_py)[1] = scale_color;//G
					intensity_raster_image.at<cv::Vec3b>(it_px, it_py)[2] = scale_color;//B
				}
				else
				{
					//average_intesity = 0;
					//count_points = 0;
					// establish a grey scale for intecity on raster
					//act_intensity = 255;
					norm_i = (act_intensity - min_intensity) / (max_intensity - act_intensity);
					scale_color = (norm_i*max_color) + ((1 - norm_i)*min_color);

					intensity_raster_image.at<cv::Vec3b>(it_px, it_py)[0] = scale_color;//R
					intensity_raster_image.at<cv::Vec3b>(it_px, it_py)[1] = scale_color;//G
					intensity_raster_image.at<cv::Vec3b>(it_px, it_py)[2] = scale_color;//B
				}
			}

			//cloud_3Draster->points[i].x = it_px*1.0f;
			//cloud_3Draster->points[i].y = it_py*1.0f;
			//cloud_3Draster->points[i].z = 0 * 1.0f;
			//cloud_3Draster->points[i++].intensity = average_intesity / count_points;
			
			// establish a grey scale for intecity on raster
			/*act_intensity = average_intesity / count_points;
			norm_i = (act_intensity - min_intensity) / (max_intensity - act_intensity);
			scale_color = (norm_i*max_color) + ((1 - norm_i)*min_color);

			intensity_raster_image.at<cv::Vec3b>(it_px, it_py)[0] = scale_color;//R
			intensity_raster_image.at<cv::Vec3b>(it_px, it_py)[1] = scale_color;//G
			intensity_raster_image.at<cv::Vec3b>(it_px, it_py)[2] = scale_color;//B*/

			minpx = maxpx;
			maxpx = maxpx + grid_x;
			minbox(0, 0) = minpx;
			maxbox(0, 0) = maxpx;
		}
		minpy = maxpy;
		maxpy = maxpy + grid_y;
		minbox(1, 0) = minpy;
		maxbox(1, 0) = maxpy;
	}
	std::cout << "Fin establish min and max intensity value in the cloud.\n\n";

	std::cout << "Debut remplir image.\n";
	for (int j = 1; j < cloud_slash_intervaly - 1; j++)
	{
		for (int i = 1; i < cloud_slash_intervalx - 1; i++)
		{
			if (intensity_raster_image.at<cv::Vec3b>(i, j)[0] == 0)
			{
				intensity_raster_image.at<cv::Vec3b>(i, j)[0] = intensity_raster_image.at<cv::Vec3b>(i, j)[1] = intensity_raster_image.at<cv::Vec3b>(i, j)[2] =
					floor((double)(intensity_raster_image.at<cv::Vec3b>(i - 1, j - 1)[0] +
						intensity_raster_image.at<cv::Vec3b>(i, j - 1)[0] +
						intensity_raster_image.at<cv::Vec3b>(i - 1, j)[0] +
						intensity_raster_image.at<cv::Vec3b>(i + 1, j + 1)[0] +
						intensity_raster_image.at<cv::Vec3b>(i - 1, j)[0] +
						intensity_raster_image.at<cv::Vec3b>(i + 1, j)[0] +
						intensity_raster_image.at<cv::Vec3b>(i - 1, j + 1)[0] +
						intensity_raster_image.at<cv::Vec3b>(i, j + 1)[0]) / 8);
				//m_VoxelSizeEcho.Format(_T("%d"), intensity_raster_image.at<cv::Vec3b>(i, j)[0]);
				//UpdateData(FALSE);
			}
			else
			{
				intensity_raster_image.at<cv::Vec3b>(i, j)[0] = intensity_raster_image.at<cv::Vec3b>(i, j)[1] = intensity_raster_image.at<cv::Vec3b>(i, j)[2] =
					floor((double)(intensity_raster_image.at<cv::Vec3b>(i - 1, j - 1)[0] +
						intensity_raster_image.at<cv::Vec3b>(i, j - 1)[0] +
						intensity_raster_image.at<cv::Vec3b>(i - 1, j)[0] +
						intensity_raster_image.at<cv::Vec3b>(i + 1, j + 1)[0] +
						intensity_raster_image.at<cv::Vec3b>(i - 1, j)[0] +
						intensity_raster_image.at<cv::Vec3b>(i + 1, j)[0] +
						intensity_raster_image.at<cv::Vec3b>(i - 1, j + 1)[0] +
						intensity_raster_image.at<cv::Vec3b>(i, j + 1)[0]) / 8);
				//m_VoxelSizeEcho.Format(_T("%d"), intensity_raster_image.at<cv::Vec3b>(i, j)[0]);
				//UpdateData(FALSE);
			}
		}
	}
	std::cout << "Fin remplir image.\n\n";

	/// Display your stuff!
	namedWindow(intensity_raster_window);
	imshow(intensity_raster_window, intensity_raster_image);
	waitKey(0);

	imwrite("output_image_z.tif", intensity_raster_image);

	return;
}


int myGrdFiltering(PointCloud<PointXYZI>::Ptr cloud, PointCloud<PointXYZRGB>::Ptr cloud_RGB)
{
	// Data containers used
	PointCloud<PointTypeIO>::Ptr cloud_in (new PointCloud<PointTypeIO>), cloud_out (new PointCloud<PointTypeIO>);
	PointCloud<PointXYZ>::Ptr cloud_XYZ (new PointCloud<PointXYZ>);
	PointCloud<PointXYZ>::Ptr cloud_projected (new PointCloud<pcl::PointXYZ>);
	PointCloud<PointXYZ>::Ptr cloud_ground (new PointCloud<pcl::PointXYZ>);
	console::TicToc tt;

	copyPointCloud (*cloud, *cloud_in);

	// Downsample the cloud using a Voxel Grid class
	std::cerr << "Downsampling...\n", tt.tic ();
	std::cerr << ">> Input: " << cloud_in->points.size () << " points\n";
	VoxelGrid<PointTypeIO> vg;
	// http://www.pcl-developers.org/VoxelGrid-Filter-td4681738.html
	vg.setInputCloud (cloud_in);
	vg.setLeafSize (0.5, 0.5, 0.5);
	vg.setDownsampleAllData (true);
	vg.filter (*cloud_out);
	std::cerr << ">> Done: " << tt.toc () << " ms, " << cloud_out->points.size () << " points\n";

	//// Transform the cloud into a MNT (i.e. Mat structure from OpenCV)
	std::cerr << "MNT computation...\n", tt.tic ();
	ConvertXYZI2XYZ(cloud_out, cloud_XYZ);

	// Select Ground
	SelectGroundPoints(cloud_XYZ, cloud_ground);

	// Create a set of planar coefficients with X=Y=0,Z=1
	ModelCoefficients::Ptr coefficients (new ModelCoefficients ());
	coefficients->values.resize (4);
	coefficients->values[0] = coefficients->values[1] = 0;
	coefficients->values[2] = 1.0;
	coefficients->values[3] = 0;

	// Create the filtering object
	pcl::ProjectInliers<PointXYZ> proj;
	proj.setModelType (pcl::SACMODEL_PLANE);
	proj.setInputCloud (cloud_XYZ);
	proj.setModelCoefficients (coefficients);
	proj.filter (*cloud_projected);

	std::cerr << "Cloud after projection: " << std::endl;
	std::cerr << *cloud_projected << std::endl;

	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ> ("output_ground.pcd", *cloud_ground, false);

	return (0);
}

int main(int argc, char** argv){
	cout << "BEGIN" << endl;
	PointCloud<PointXYZI>::Ptr cloud (new PointCloud<PointXYZI>);
	PointCloud<PointXYZRGB>::Ptr cloud_RGB (new PointCloud<PointXYZRGB>);
	if (pcl::io::loadPCDFile<PointXYZI> ("D:/t10d_XYZI.pcd", *cloud) == -1) //* load the file
    {
  	  PCL_ERROR ("Couldn't read file\n");
  	  return (-1);
    }

	myGrdFiltering(cloud, cloud_RGB);
	cout << "DONE" << endl;
}  