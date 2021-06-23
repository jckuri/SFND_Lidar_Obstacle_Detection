// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "kdtree.h"

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
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);
    
    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);
    
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f {-1.5, -1.7, -1, 1});
    roof.setMax(Eigen::Vector4f {2.6, 1.7, -0.4, 1});
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);
    
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : indices)
        inliers->indices.push_back(point);
        
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
    ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, 
    typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());

    for(int index : inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac
    (typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

	while(maxIterations--) {
        std::unordered_set<int> inliers;
        while(inliers.size() < 3)
            inliers.insert(rand()%(cloud->points.size()));
        float x1, y1, z1, x2, y2, z2, x3, y3, z3;
		auto idx = inliers.begin();
        x1 = cloud->points[*idx].x;
        y1 = cloud->points[*idx].y;
		z1 = cloud->points[*idx].z;
		idx++;
        x2 = cloud->points[*idx].x;
        y2 = cloud->points[*idx].y;
		z2 = cloud->points[*idx].z;
		idx++;
        x3 = cloud->points[*idx].x;
        y3 = cloud->points[*idx].y;
		z3 = cloud->points[*idx].z;
		float i, j, k;
		i = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
        j = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1); 
        k = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
		float D = -(i*x1 + j*y1 + k*z1);        
        for(int index = 0; index < cloud->points.size(); index++) {
			auto p = cloud->points[index];
			float d = fabs(i * p.x + j * p.y + k * p.z + D) / sqrt(i * i  + j * j + k * k);
			if(d <= distanceTol) inliers.insert(index);
        }
		if(inliers.size() > inliersResult.size())
			inliersResult = inliers;
    }

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
    ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, 
    int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.

    std::unordered_set<int> inliers0 = Ransac(cloud, maxIterations, distanceThreshold);

    typename pcl::PointCloud<PointT>::Ptr inliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr outliers(new pcl::PointCloud<PointT>());
	for(int index = 0; index < cloud->points.size(); index++) {
		PointT point = cloud->points[index];
		if(inliers0.count(index))
			inliers->points.push_back(point);
		else
			outliers->points.push_back(point);
	}

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
        segResult(outliers, inliers);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper
    (int indice, const std::vector<std::vector<float>> points, std::vector<int>& cluster, 
    std::vector<bool>& processed, KdTree* tree, float distanceTol) {

    processed[indice] = true;
    cluster.push_back(indice);
    std::vector<int> nearest = tree->search(points[indice], distanceTol);
    for(int id : nearest) {
        if(!processed[id])
            clusterHelper(id, points, cluster, processed, tree, distanceTol);
    }
    
}

template<typename PointT>
std::vector<std::vector<int>> 
    ProcessPointClouds<PointT>::euclideanCluster
    (const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, 
    int minSize)
{

	// TODO: Fill out this function to return list of indices for each cluster

    std::vector<std::vector<int>> clusters;
    std::vector<bool> processed(points.size(), false);
    int i = 0;
    while(i < points.size()) {
        if(processed[i]) {
            i++;
            continue;
        }
        std::vector<int> cluster;
        clusterHelper(i, points, cluster, processed, tree, distanceTol);
        if(cluster.size() >= minSize)
            clusters.push_back(cluster);
        i++;
    }
    
    return clusters;

}

template<typename PointT> std::vector<typename pcl::PointCloud<PointT>::Ptr> 
    ProcessPointClouds<PointT>::Clustering
    (typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, 
    int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    std::vector<std::vector<int>> clusters2;
    std::vector<std::vector<float>> points2;
    KdTree *kdTree = new KdTree();
    for(int i = 0; i < cloud->points.size(); i++) {
        auto point = cloud->points[i];
        std::vector<float> point2 = {point.x, point.y, point.z};
        kdTree->insert(point2, i);
        points2.push_back(point2);
    }
    clusters2 = euclideanCluster(points2, kdTree, clusterTolerance, minSize);
    printf("CLUSTERING DONE!\n");
    
    for(std::vector<std::vector<int>>::const_iterator cluster_iterator = clusters2.begin(); cluster_iterator != clusters2.end(); cluster_iterator++) {
		typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
	    for (std::vector<int>::const_iterator index_iterator = cluster_iterator->begin(); index_iterator != cluster_iterator->end(); index_iterator++)
	        cluster->points.push_back (cloud->points[*index_iterator]);
	    cluster->width = cluster->points.size ();
	    cluster->height = 1;
	    cluster->is_dense = true;
	    clusters.push_back(cluster);
	}

    printf("CLUSTERS TRANSLATED.\n");

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