#ifndef KDTREE_H_
#define KDTREE_H_
/* \author Aaron Brown */

#include <vector>
#include <cstddef>
#include <math.h>

// Structure to represent node of kd tree
class Node
{
public:
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId);
};

class KdTree
{
public:
	Node* root;

	KdTree();

	void insertHelper(Node *&node, unsigned int depth, std::vector<float> point, int id);

	void insert(std::vector<float> point, int id);

	void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids);

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol);
	
};

#endif