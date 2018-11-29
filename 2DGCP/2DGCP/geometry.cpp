#include "pch.h"
#include "stdafx.h"
#include "geometry.h"

int QuadTreeNode::append(Edge2D * edge)
{
	if (edge == NULL)
		return -1;

	EdgeRef2D * new_er = new EdgeRef2D;
	new_er->e = edge;
	if (this->edgeRef == NULL)
		new_er->next = NULL;
	else
		new_er->next = this->edgeRef;
	this->edgeRef = new_er;
	return 1;
}

int QuadTreeNode::eliminate(Edge2D * edge)
{
	if (edge == NULL)
		return -1;

	EdgeRef2D * cur_er = this->edgeRef;
	EdgeRef2D * pre_er = NULL;
	while (cur_er)
	{
		if (cur_er->e == edge)
		{
			if (this->edgeRef == cur_er)
			{
				this->edgeRef = cur_er->next;
				delete cur_er;
				cur_er = this->edgeRef;
			}
			else
			{
				pre_er->next = cur_er->next;
				delete cur_er;
				cur_er = pre_er->next;
			}
		}
		else
		{
			pre_er = cur_er;
			cur_er = cur_er->next;
		}
	}

	return 1;
}

void QuadTree::clear(QuadTreeNode * root)
{
	if (root == NULL)
		return;

	//leaf node
	if (root->flag && QUADTREE_LEAF_NODE)
	{
		EdgeRef2D * ref1, *ref2;
		ref1 = root->edgeRef;
		while (ref1)
		{
			ref2 = ref1->next;
			delete ref1;
			ref1 = ref2;
		}
	}
	else if (root->flag && QUADTREE_INTERNAL_NODE)
	{
		clear(root->subnode[0]);
		clear(root->subnode[1]);
		clear(root->subnode[2]);
		clear(root->subnode[3]);
	}
}