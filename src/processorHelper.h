#ifndef PROCESSORHELPER_H_
#define PROCESSORHELPER_H_

#include <unordered_set>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "quiz/cluster/kdtree.h"

template<typename PointT>
std::unordered_set<int> Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateCloudsInliersSet(std::unordered_set<int> inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize);

template<typename PointT>
std::vector<std::vector<int>> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol, int minSize);

template<typename PointT>
void buildCluster(std::vector<int> &cluster,const int pointId, typename pcl::PointCloud<PointT>::Ptr cloud,std::vector<bool> &pointsProcessed, KdTree* tree, float distanceTol);
#endif 