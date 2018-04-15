#pragma once
#ifndef _LINK_H_
#define _LINK_H_

/*
* Handles the Link and Nodes for use in the truss class
*/

//Nodes structure. The coordinates are integeres as the nodes are only on the lattice points.
struct Node
{
	int x;
	int y;

	bool operator== (Node& node)
	{
		return ((x == node.x) && (y == node.y));
	}
};

//A link object to store the data about the link. Effectively stores the connectivity. 
class Link
{
	double area;
	double length;
	double slope;
	double E;

	Node nodes[2];
public:
	Link(Node n1 = { 0,0 }, Node n2 = {0,0});
	
	double setArea(double ar) { area = ar; }
	
	double getArea()	{ return area;	}
	double getLength()	{ return length;}
	double getSlope()	{ return slope; }
	double getE()		{ return E;		}
	Node* getNode()		{ return nodes;	}
	
	
	bool operator== (Link);
	
	~Link();
};

#endif