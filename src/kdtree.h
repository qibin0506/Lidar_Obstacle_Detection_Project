#include "render/render.h"


// Structure to represent node of kd tree
struct Node
{
	pcl::PointXYZI point;
	int id;
	Node* left;
	Node* right;

	Node(pcl::PointXYZI arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(pcl::PointXYZI point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
      insert(root, 0, point, id);
	}
  
  void insert(Node *&node, uint depth, pcl::PointXYZI point, int id) {
    if (node == NULL) {
      node = new Node(point, id);
      return;
    }
    
    if (depth % 3 == 0) {
      if (point.x < node->point.x) {
        insert(node->left, depth+1, point, id);
      } else {
        insert(node->right, depth+1, point, id);
      }
    } else if (depth % 2 == 0) {
      if (point.y < node->point.y) {
        insert(node->left, depth+1, point, id);
      } else {
        insert(node->right, depth+1, point, id);
      }
    }else {
      if (point.z < node->point.z) {
        insert(node->left, depth+1, point, id);
      } else {
        insert(node->right, depth+1, point, id);
      }
    }
  }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(pcl::PointXYZI target, float distanceTol)
	{
		std::vector<int> ids;
        search(root, 0, target, distanceTol, ids);
		return ids;
	}
	
	void search(Node *&node, uint depth, pcl::PointXYZI target, float distanceTol, std::vector<int> &ids) {
      if (node == NULL) {
      	return;
      }
      
      if (node->point.x >= target.x - distanceTol
         && node->point.x <= target.x + distanceTol
         && node->point.y >= target.y - distanceTol
         && node->point.y <= target.y + distanceTol
         && node->point.z >= target.z - distanceTol
         && node->point.z <= target.z + distanceTol) {
        float distance = sqrt(pow(node->point.x-target.x, 2) + pow(node->point.y-target.y, 2) + pow(node->point.z-target.z, 2));
        if (distance <= distanceTol) {
          ids.push_back(node->id);
        }
      }
      
      if (depth % 3 == 0) {
        if (target.x - distanceTol < node->point.x) {
          search(node->left, depth+1, target, distanceTol, ids);
        }
        
        if (target.x + distanceTol > node->point.x) {
       	 search(node->right, depth+1, target, distanceTol, ids);
      	}
      } else if (depth % 2 == 0) {
        if (target.y - distanceTol < node->point.y) {
          search(node->left, depth+1, target, distanceTol, ids);
        }
        
        if (target.y + distanceTol > node->point.y) {
       	 search(node->right, depth+1, target, distanceTol, ids);
      	}
      } else {
        if (target.z - distanceTol < node->point.z) {
          search(node->left, depth+1, target, distanceTol, ids);
        }
        
        if (target.z + distanceTol > node->point.z) {
       	 search(node->right, depth+1, target, distanceTol, ids);
      	}
      }
    }
};
