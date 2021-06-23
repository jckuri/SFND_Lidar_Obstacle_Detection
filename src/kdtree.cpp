/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "kdtree.h"

// Structure to represent node of kd tree
Node::Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	KdTree::KdTree()
	: root(NULL)
	{}

	void KdTree::insertHelper(Node *&node, unsigned int depth, std::vector<float> point, int id) {
        if(node == NULL) node = new Node(point, id);
        else {
            unsigned int cd = depth % 3;
            if(point[cd] < node->point[cd])
                insertHelper(node->left, depth + 1, point, id);
            else
                insertHelper(node->right, depth + 1, point, id);
        }
    }

	void KdTree::insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(root, 0, point, id);
	}

	void KdTree::searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids) {
        if(node != NULL) {
			bool tol_x = node->point[0] >= (target[0] - distanceTol) && 
				node->point[0] <= (target[0] + distanceTol);
			bool tol_y = node->point[1] >= (target[1] - distanceTol) && 
				node->point[1] <= (target[1] + distanceTol);
			bool tol_z = node->point[2] >= (target[2] - distanceTol) && 
				node->point[2] <= (target[2] + distanceTol);
            if(tol_x && tol_y && tol_z) {
                float dx = node->point[0] - target[0];
                float dy = node->point[1] - target[1];
				float dz = node->point[2] - target[2];
                float distance = sqrt(dx * dx + dy * dy + dz * dz);
                if(distance <= distanceTol) ids.push_back(node->id);
            }
            int c = depth % 3;
            if((target[c] - distanceTol) < node->point[c])
                searchHelper(target, node->left, depth + 1, distanceTol, ids);
            if((target[c] + distanceTol) > node->point[c])
                searchHelper(target, node->right, depth + 1, distanceTol, ids);
        }
    }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> KdTree::search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
    	searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
	