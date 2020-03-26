/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "render/render.h"
#include "math.h"


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
    int stage=0;
	int rem;
	int rem_search;
	KdTree()
	: root(NULL)
	{}

	void inserttree(Node** root,uint depth, std::vector<float> point, int id)
	{		
	  Node* newnode (new Node(point,id));
       if(*root == NULL)
	   {
		    
		   *root=newnode;
	   }
	   else
	   {
		   uint cd=depth%3;
		   if (point[cd] < (*root)->point[cd] )
		   {
			  inserttree(&((*root)->left),depth+1,point,id);
		   }
		   else
		   {
			   inserttree(&((*root)->right),depth+1,point,id);
		   }
		   
		   
	   }
	   /*else if (point[0] <= (*root)->point[0] && rem ==0 )
	   {
   		   stage =stage+1;
		   inserttree(&(*root)->left,point,id);
	   }
	   else if (point[0] > (*root)->point[0] && rem==0)
	   {
		  stage =stage+1;
		   inserttree(&(*root)->right,point,id);
	   }
	   else if (point[1] <= (*root)->point[1] && rem ==1 )
	   {
 		  stage =stage+1;
		  inserttree(&(*root)->left,point,id);
	   }
	   else if (point[1] > (*root)->point[1] && rem==1)
	   {
		  stage =stage+1;
		  inserttree(&(*root)->right,point,id);
	   }*/

	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
        
		inserttree(&root,0,point,id);

	}
	void searchtree(Node* root,std::vector<float> target,std::vector<int>& ids,float distanceTol,int stage_)
	{
		/*if(root==NULL)
		{
  		 std::cout<<"NULL"<<std::endl;
		}*/

		if(root!=NULL)
		{
          if((root->point[0]>=(target[0]-distanceTol))&&(root->point[0]<=(target[0]+distanceTol))&&(root->point[1]>=(target[1]-distanceTol))&&(root->point[1]<=(target[1]+distanceTol))&&(root->point[2]>=(target[2]-distanceTol))&&(root->point[2]<=(target[2]+distanceTol)))
			{
				float distance= sqrt((root->point[0]-target[0])*(root->point[0]-target[0])+(root->point[1]-target[1])*(root->point[1]-target[1])+(root->point[2]-target[2])*(root->point[2]-target[2]));
			if(distance<=distanceTol)
			{
				ids.push_back(root->id); 
			
			}
			}

			if(((target[stage_%3]-distanceTol) <= root->point[stage_%3]))
			{
				
				searchtree(root->left,target,ids,distanceTol,(stage_+1));
				
			}
			if(((target[stage_%3]+distanceTol) > root->point[stage_%3]))
			{
			
				searchtree(root->right,target,ids,distanceTol,(stage_+1));
				
			}
	 
	 }
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
        int stage_=0;
       
		searchtree(root,target,ids,distanceTol,stage_);

		return ids;
	}
	

};




