/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "./render/render.h"

// Structure to represent node of kd tree
struct Node
{
	std::vector<double> point;
	int id;
	Node *left;
	Node *right;

	Node(std::vector<double> arr, int setId)
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

	void insertHelper(Node **node, int fd, int id, std::vector<double> point) {
		if (*node == NULL) {
			*node = new Node(point, id);
		}
		else if ((*node)->point[fd] > point[fd]) {
			insertHelper(&(*node)->left, (fd+1)%3, id, point);
		}
		else {
			insertHelper(&(*node)->right, (fd+1)%3, id, point);
		}
	}

	void insert(std::vector<double> point, int id) {
		insertHelper(&this->root, 0, id, point);
	}

	void searchHelper(Node *node, std::vector<double> target, double distanceTol, int fd, std::vector<int>* ids) {
		// std::cout<<"searchHelper = " << (*node).point <<std::endl;

		if (node != NULL) {
			double x_edge = target[0]-distanceTol;
			double y_edge = target[1]-distanceTol;
			double z_edge = target[2]-distanceTol;

			long double dist=sqrt(pow(node->point[0]-target[0],2.0)+pow(node->point[1]-target[1],2.0)+pow(node->point[2]-target[2],2.0));
			if (dist <= distanceTol)
				ids->push_back(node->id);

			if (node->point[fd] > target[fd]-distanceTol) {
				searchHelper(node->left, target, distanceTol, (fd+1)%3, ids);
			} 
			if (node->point[fd] < target[fd]+distanceTol) {
				searchHelper(node->right, target, distanceTol, (fd+1)%3, ids);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<double> target, double distanceTol)
	{
		std::vector<int>* ids = new std::vector<int>();
		searchHelper(this->root, target, distanceTol, 0, ids);
		// std::cout<<"post search - ids = " << ids->size() <<std::endl;
		return *ids;
	}
};
