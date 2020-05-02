/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	float a,b,c;
	float best_a,best_b,best_c;
	int p1,p2;
	int best_model_count = 0;
	double p1_x,p2_x,p1_y,p2_y;
	float dist =0.0f;
	// For max iterations 
	for (int itr=0;itr<maxIterations;itr++)
	{
		// Randomly sample subset and fit line
		p1 = rand() % cloud->points.size();
		p2 = rand() % cloud->points.size();
		if (p1 == p1) p2 = rand() % cloud->points.size();
		
		// calculate model coeff.
		p1_x = cloud->points[p1].x;
		p2_x = cloud->points[p2].x;
		p1_y = cloud->points[p1].y;
		p2_y = cloud->points[p2].y;
		a = p1_y - p2_y;
		b = p2_x - p1_x;
		c = p1_x * p2_y - p2_x * p1_y;
		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		int inlier_count =0;
		for(int i =0;i<cloud->points.size();++i)
		{	
			dist = fabs(a*cloud->points[i].x + b*cloud->points[i].y + c) / (sqrt(a*a + b*b));
			if (dist <= distanceTol) ++inlier_count;
		}
		std::cout << "iteration  " << itr << " have inliers =" << inlier_count<< std::endl;
		if (inlier_count >= best_model_count)
		{
			best_model_count = inlier_count;
			best_a = a;
			best_b = b;
			best_c = c;
		}
	}
	// Return indicies of inliers from fitted line with most inliers
	dist =0;
	for(int i =0;i<cloud->points.size();++i)
		{	
			dist = abs(best_a*cloud->points[i].x + best_b*cloud->points[i].y + best_c) / sqrt(best_a*best_a + best_b*best_b);
			if (dist <= distanceTol) inliersResult.insert(i);
		}
	return inliersResult;

}


std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	float a,b,c,d;
	double x1,y1,z1,x2,y2,z2,x3,y3,z3;
	float dist =0.0f;
	// loop for max iterations 
	while(maxIterations--)
	{
		// Randomly sample subset and fit potential model
		std::unordered_set<int> inliers;
		while(inliers.size() < 3)
		{
			inliers.insert(rand() % cloud->points.size());
		}
		
		auto itr = inliers.begin(); 
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		// advance for next point
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		// advance for next point
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;
		// calculate model coeff.

		a = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
		b = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
		c = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
		d = -1*(a*x1+b*y1+c*z1);
		
		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		
		for(int i =0; i<cloud->points.size(); ++i)
		{	
			dist = fabs(a*cloud->points[i].x + b*cloud->points[i].y  + c*cloud->points[i].z + d) / (sqrt(a*a + b*b+c*c));
			if (dist <= distanceTol) inliers.insert(i);
		}
		std::cout << "iteration  " << maxIterations << " have inliers =" << inliers.size()<< std::endl;
		if (inliers.size() >= inliersResult.size())
		{
			inliersResult = inliers;
		}
	}
	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	std::cout << "cloud has   " << cloud->points.size() << " points"<< std::endl;
	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	// std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);
	std::unordered_set<int> inliers = Ransac3D(cloud, 50, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
