// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <unordered_set>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

template <typename PointT>
struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}
	
    // Insert helper using poinrt reference 
  	void insertHelper(Node *&node, uint depth, PointT point, int id)
    {
      if(node == NULL)
      {
        std::vector<float> p {point.x, point.y, point.z};
        node = new Node(p, id);
      }
      else
      {
        uint cd = depth % 3;
        
        if(point.data[cd] < node->point[cd])
          insertHelper(node->left, depth+1, point, id);
        else
          insertHelper(node->right, depth+1, point, id);
      }
    }
  
	void insert(PointT point, int id)
	{
      	//for(auto point: points)
      	insertHelper(root, 0, point, id);
	}

  	void searchHelper(PointT target, Node *node, uint depth, float distanceTol, std::vector<int> &ids)
    {
      if(node!= NULL)
      {
      	if( (node->point[0] >= (target.data[0] - distanceTol) && node->point[0] <= (target.data[0] + distanceTol)) && (node->point[1] >= (target.data[1] - distanceTol) && node->point[1] <= (target.data[1] + distanceTol) && (node->point[2] >= (target.data[2] - distanceTol) && node->point[2] <= (target.data[2] + distanceTol))))
        {
          float distance = sqrt((node->point[0] - target.data[0])*(node->point[0] - target.data[0]) + (node->point[1] - target.data[1]) * (node->point[1] - target.data[1]) + (node->point[2] - target.data[2]) * (node->point[2] - target.data[2]));
          if(distance <= distanceTol)
            ids.push_back(node->id);
        }
        
        if((target.data[depth%3] - distanceTol) < node->point[depth%3])
           searchHelper(target, node->left, depth+1, distanceTol, ids);
      	
        if((target.data[depth%3] + distanceTol) > node->point[depth%3])
           searchHelper(target, node->right, depth+1, distanceTol, ids);   
      }
   }
  
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
      	searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
};

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);
    
	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);    

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);
    
    void proximity(int index, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int> &cluster, std::vector<bool> &processed, KdTree<PointT>* tree, float distanceTol);
    
    std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol, int minSize, int maxSize);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
};

#endif /* PROCESSPOINTCLOUDS_H_ */