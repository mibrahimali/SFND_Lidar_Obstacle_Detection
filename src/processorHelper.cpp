#include "processorHelper.h"

template<typename PointT>
std::unordered_set<int> Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
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
		// std::cout << "iteration  " << maxIterations << " have inliers =" << inliers.size()<< std::endl;
		if (inliers.size() >= inliersResult.size())
		{
			inliersResult = inliers;
		}
	}
	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateCloudsInliersSet(std::unordered_set<int> inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
	typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}
	
	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers,cloudInliers);
    return segResult;

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize)
{
	KdTree* tree = new KdTree(3);
  
    for (int i=0; i<cloud->points.size(); i++) 
    	{
			std::vector<float> point{cloud->points[i].x,cloud->points[i].y,cloud->points[i].z};
			tree->insert(point,i);	
		}	

	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
	std::vector<std::vector<int>> clusters_ind = euclideanCluster<PointT>(cloud, tree, clusterTolerance,minSize);
	
	for (auto indices: clusters_ind)
	{
		typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
		for(auto ind : indices)
			{
				cloud_cluster->points.push_back (cloud->points[ind]);
			}
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		clusters.push_back(cloud_cluster);

	}
	return clusters;
}

template<typename PointT>
void buildCluster(std::vector<int> &cluster,const int pointId, typename pcl::PointCloud<PointT>::Ptr cloud,std::vector<bool> &pointsProcessed, KdTree* tree, float distanceTol)
{
	pointsProcessed[pointId] = true;
	cluster.push_back(pointId);
	std::vector<float> searchPoint{cloud->points[pointId].x,cloud->points[pointId].y,cloud->points[pointId].z};
	std::vector<int> nearby = tree->search(searchPoint,distanceTol);
  	for(int index : nearby)
	  if (!pointsProcessed[index])
		{
			buildCluster<PointT>(cluster,index,cloud,pointsProcessed, tree, distanceTol);
		}


}
template<typename PointT>
std::vector<std::vector<int>> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol, int minSize)
{

	std::vector<std::vector<int>> clusters;
	std::vector<bool> pointsProcessed(cloud->points.size(),false);
	for (int pointId=0;pointId <cloud->points.size();++pointId)
	{
		if (!pointsProcessed[pointId])
		{
			std::vector<int> cluster;
			buildCluster<PointT>(cluster,pointId,cloud,pointsProcessed, tree, distanceTol);
			if(cluster.size()>minSize)
			{
				clusters.push_back(cluster);
			}
		}
	}
	return clusters;

}