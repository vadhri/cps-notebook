/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node *left;
	Node *right;

	Node(std::vector<float> arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL)
	{
	}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node *root;

	KdTree()
		: root(NULL) {
	}

	~KdTree() {
		delete root;
	}

	void insertHelper(Node **node, int fd, int id, std::vector<float> point) {
		if (*node == NULL) {
			*node = new Node(point, id);
		}
		else if ((*node)->point[fd] > point[fd]) {
			insertHelper(&(*node)->left, 1 - fd, id, point);
		}
		else {
			insertHelper(&(*node)->right, 1 - fd, id, point);
		}
	}

	void insert(std::vector<float> point, int id) {
		insertHelper(&this->root, 0, id, point);
	}

	void searchHelper(Node *node, std::vector<float> target, float distanceTol, int fd, std::vector<int>* ids) {
		// std::cout<<"searchHelper = " << (*node).point <<std::endl;

		if (node != NULL) {
			// std::cout<<(*node).point[0]<<" - "<<(*node).point[1]<<std::endl;
			// is the target in the bounding box
			float x_edge = target[0]-distanceTol;
			float y_edge = target[1]-distanceTol;

			if ((node->point[0] <= x_edge+(2*distanceTol) && node->point[0] >= x_edge) && (node->point[1] <= y_edge+(2*distanceTol) && node->point[1] >= y_edge)) {
				std::cout<<"Found a target" << (node->point[0], node->point[1])<<std::endl;
				ids->push_back(node->id);
			} 
			// else {
			// 	std::cout<<"Unmatched a target" << (node->point[0], node->point[1]) <<std::endl;
			// }

			if (node->point[fd] > target[fd]-distanceTol) {
				searchHelper(node->left, target, distanceTol, 1-fd, ids);
			} 
			if (node->point[fd] < target[fd]+distanceTol) {
				searchHelper(node->right, target, distanceTol, 1-fd, ids);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int>* ids = new std::vector<int>();
		searchHelper(this->root, target, distanceTol, 0, ids);
		std::cout<<"post search - ids = " << ids->size() <<std::endl;
		return *ids;
	}
};
