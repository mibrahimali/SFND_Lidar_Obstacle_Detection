/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


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

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;
	unsigned int K;
	KdTree(unsigned int k=2)
	: root(NULL),K(k)
	{}

	~KdTree()
	{
		delete root;
	}

	void insertHelper(Node *&rootnode , std::vector<float> point, int id, unsigned int depth)
	{
		if(rootnode == NULL)
		{
			rootnode = new Node(point,id);
		}
		else
		{
			int check_dimension = depth % K;
			if(point[check_dimension] >= rootnode->point[check_dimension])
			{
				insertHelper(rootnode->right , point, id, depth +1);
			}
			else
			{
				insertHelper(rootnode->left , point, id, depth+1);
			}
		}
	}
	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 

		insertHelper(root , point, id, 0);

	}

	bool pointInBox(std::vector<float> &source,std::vector<float> &target,float distanceTol,unsigned dim)
	{
		
		if (source.size() == dim && target.size() == dim)
		{
			bool result = true;
			for(int i=0;i<dim;++i)
			{
				result &= fabs(target[i]-source[i])<=distanceTol;
			}
			return result;
		}
		else
		{
			std::cerr<<"Points dimentions does not match"<<endl;
			return false;
		}
		

	}
	float calculateDist2Points(std::vector<float>& firstPoint, std::vector<float>& secondPoint, unsigned dim)
	{
		float dist = 0.0;
		// sanity check if two points have same dimentions like dim ( 2 incase 2D points and 3 incase 3D points)
		if (firstPoint.size() == dim && secondPoint.size() == dim)
		{
			for(int i=0;i<dim;++i)
			{
				dist += pow(firstPoint[i]-secondPoint[i],2.0);
			}
		}
		else
		{
			std::cerr<<"Points dimentions does not match"<<endl;
		}
		return sqrt(dist);
	}

	void searchhelper(Node *&searchnode ,unsigned int depth,std::vector<float> target, float distanceTol,std::vector<int> &ids)
	{
		if(searchnode != NULL)
		{
			if(pointInBox(searchnode->point,target,distanceTol,K))
			//if(fabs(target[0]-searchnode->point[0])<=distanceTol && fabs(target[1]-searchnode->point[1])<=distanceTol)
			{
				float dist = calculateDist2Points(target,searchnode->point,K);
				if (dist <= distanceTol)
				{
					ids.push_back(searchnode->id);
				}
			}
			int check_dimension = depth % K;
			if(target[check_dimension] - distanceTol < searchnode->point[check_dimension])
			{
				searchhelper(searchnode->left ,depth + 1, target, distanceTol,ids);
			} 
			if(target[check_dimension] + distanceTol >= searchnode->point[check_dimension])
			{
				searchhelper(searchnode->right ,depth + 1, target, distanceTol,ids);
			}
		}
	}
	
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchhelper(root ,0, target, distanceTol,ids);
		return ids;
	}
	

};




