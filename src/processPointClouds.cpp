// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
  	pcl::VoxelGrid<PointT> vg;
  	typename pcl::PointCloud<PointT>::Ptr filteredCloud (new pcl::PointCloud<PointT>);
  	
  	vg.setInputCloud(cloud);
  	vg.setLeafSize(filterRes, filterRes, filterRes);
  	vg.filter(*filteredCloud);
  
  	typename pcl::PointCloud<PointT>::Ptr regionCloud (new pcl::PointCloud<PointT>);
  
  	pcl::CropBox<PointT> region (true);
  	region.setMin(minPoint);
  	region.setMax(maxPoint);
  	region.setInputCloud(filteredCloud);
  	region.filter(*regionCloud);
  
  	std::vector<int> indices;
  
  	pcl::CropBox<PointT> roof (true);
  	region.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
  	region.setMax(Eigen::Vector4f (2.6, 1.7, -0.4, 1));
  	region.setInputCloud(regionCloud);
  	region.filter(indices);
  
  	pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
  	for(int point : indices)
      inliers->indices.push_back(point);
	
  	pcl::ExtractIndices<PointT> extract;
  	extract.setInputCloud(regionCloud);
  	extract.setIndices(inliers);
  	extract.setNegative(true);
  	extract.filter(*regionCloud);
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return regionCloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
  typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());
  
  for(int index : inliers->indices)
    planeCloud->points.push_back(cloud->points[index]);
  
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*obstCloud);

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
  return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
  	while(maxIterations--)
    {
      std::unordered_set<int> inliers;
      while(inliers.size() < 3) {
        inliers.insert((rand()%(cloud->points.size())));
      }
      float x1, y1, z1, x2, y2, z2, x3, y3, z3;
                       
      auto itr = inliers.begin();
      x1 = cloud->points[*itr].x;
      y1 = cloud->points[*itr].y;
      z1 = cloud->points[*itr].z;
                       
      itr++;
      x2 = cloud->points[*itr].x;
      y2 = cloud->points[*itr].y;
      z2 = cloud->points[*itr].z;
                       
      itr++;
      x3 = cloud->points[*itr].x;
      y3 = cloud->points[*itr].y;
      z3 = cloud->points[*itr].z;
                       
      float a = (y2-y1) * (z3-z1) - (z2-z1) * (y3-y1);
      float b = (z2-z1) * (x3-x1) - (x2-x1) * (z3-z1);
      float c = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
      float d = - (a*x1 + b*y1 + c*z1);
                       
      for(int index = 0; index < cloud->points.size(); index++)
      {
          if(inliers.count(index) > 0)
            continue;
          
          auto point = cloud->points[index];
          float x4 = point.x;
          float y4 = point.y;
          float z4 = point.z;
          
          float dis = fabs(a*x4+b*y4+c*z4+d)/sqrt(a*a + b*b + c*c);
          
          if(dis <= distanceTol)
            inliers.insert(index);
        }
        if(inliers.size() > inliersResult.size()) {
          inliersResult = inliers;
        }
    }
    typename pcl::PointCloud<PointT>::Ptr cloudInliers (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers (new pcl::PointCloud<PointT>());
                       
	for (int index = 0; index < cloud->points.size(); index++)
    {
        PointT point = cloud->points[index];
        if(inliersResult.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> partedClouds(cloudOutliers, cloudInliers);
    return partedClouds;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
    // TODO:: Fill in this function to find inliers for the cloud.
	pcl::SACSegmentation<PointT> seg;
  	pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
  	pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};
  
  	seg.setOptimizeCoefficients(true);
  	seg.setModelType(pcl::SACMODEL_PLANE);
  	seg.setMethodType(pcl::SAC_RANSAC);
  	seg.setMaxIterations(maxIterations);
  	seg.setDistanceThreshold(distanceThreshold);
  
  	seg.setInputCloud(cloud);
  	seg.segment(*inliers, *coefficients);
  	if(inliers->indices.size() == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset. " << std::endl;
    }
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
void ProcessPointClouds<PointT>::proximity(int index, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int> &cluster, std::vector<bool> &processed, KdTree<PointT>* tree, float distanceTol)
{
  processed[index] = true;
  cluster.push_back(index);
  
  std::vector<int> near = tree->search(cloud->points[index], distanceTol);
  
  for (int id : near)
  {
    if(!processed[id])
      proximity(id, cloud, cluster, processed, tree, distanceTol);
  }
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol, int minSize, int maxSize)
{

  	auto startTime = std::chrono::steady_clock::now();
  
	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
  	std::vector<std::vector<int>> clusterIndices;
  	std::vector<bool> processed(cloud->points.size(), false);
  	typename KdTree<PointT>::KdTree *tree (new KdTree<PointT>);
  
  	for(int index = 0; index < cloud->points.size(); index++)
    {
  		tree->insert(cloud->points[index], index);
    }
  
  	int i = 0;
  	while(i < cloud->points.size()) 
    {
       	if(!processed[i])
        {
          std::vector<int> cluster;
          proximity(i, cloud, cluster, processed, tree, distanceTol);
          if((cluster.size() >= minSize) && (cluster.size() <= maxSize))
          	clusterIndices.push_back(cluster);
          	i++;
        }
        else {
          i++;
          continue;
        }
    }
  
  	for(const auto &ind: clusterIndices)
    {
      typename pcl::PointCloud<PointT>::Ptr cloudCluster (new typename pcl::PointCloud<PointT>);
      
      for(int i : ind) 
        cloudCluster->points.push_back(cloud->points[i]);    

	  cloudCluster->width = cloudCluster->points.size();
      cloudCluster->height = 1;
	  cloudCluster->is_dense = true;
		
	  clusters.push_back(cloudCluster);
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;

}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
  	typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  	tree->setInputCloud(cloud);
  
  	std::vector<pcl::PointIndices> clusterIndices;
  	pcl::EuclideanClusterExtraction<PointT> ec;
  	ec.setClusterTolerance (clusterTolerance);
  	ec.setMinClusterSize (minSize);
  	ec.setMaxClusterSize (maxSize);
  	ec.setSearchMethod (tree);
  	ec.setInputCloud (cloud);
  	ec.extract (clusterIndices);
  
  	for(pcl::PointIndices getIndices: clusterIndices)
    {
      typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
      
      for(int i : getIndices.indices)
        cloudCluster->points.push_back(cloud->points[i]);

	  cloudCluster->width = cloudCluster->points.size();
      cloudCluster->height = 1;
	  cloudCluster->is_dense = true;
		
	  clusters.push_back(cloudCluster);
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}