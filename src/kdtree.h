/* \author Aaron Brown */
// kd tree for point cloud clustering

#include <vector>

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

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node** node, std::vector<float> point, int id, uint depth)
	{
		if(*node == NULL)
		{
			*node = (new Node(point,id));
		}
		else
		{
			uint ind = depth%3;

			if(point[ind] < ((*node)->point[ind]))
				insertHelper(&((*node)->left), point, id, depth+1);
			else
			{
				insertHelper(&((*node)->right), point, id, depth+1);
			}
			
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// insert a new point into the tree
		// the function create a new node and place correctly with in the root 
		insertHelper(&root, point, id, 0);
	}

	void searchHelper(std::vector<float> target, Node* node, uint depth, float distanceTol, std::vector<int>& ids)
	{
		if(node!=NULL)
		{
			float delta_x = node->point[0]-target[0];
			float delta_y = node->point[1]-target[1];
			float delta_z = node->point[2]-target[2];
			if((-distanceTol<=delta_x && delta_x<=distanceTol)
				&& (-distanceTol<=delta_y && delta_y<=distanceTol)
				&& (-distanceTol<=delta_z && delta_z<=distanceTol))
			{
				float dist = sqrt(pow((target[0]-node->point[0]),2)
								 +pow((target[1]-node->point[1]),2)
								 +pow((target[2]-node->point[2]),2));
				if (dist <= distanceTol)
					ids.push_back(node->id);
			}

			// checke across boundary
			uint ind = depth%3;
			if(node->point[ind]>(target[ind]-distanceTol))
				searchHelper(target, node->left, depth+1, distanceTol, ids);
			if(node->point[ind]<(target[ind]+distanceTol))
				searchHelper(target, node->right, depth+1, distanceTol, ids);
		}

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
	

};
