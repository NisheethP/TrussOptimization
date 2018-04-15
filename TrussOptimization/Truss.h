#pragma once
#ifndef _TRUSS_H_
#define _TRUSS_H

#include <vector>
#include "Link.h"

/*
*Class to hold the info of the truss. Stores links and nodes.
*/

class Truss
{
	std::vector<Node> nodes;
	std::vector<Link> links;

public:
	Truss(std::vector<Node>& pNodes, std::vector<Link>&);
	Truss();
	Truss(Truss&);

	std::vector<Node> getNodes() { return nodes; }
	std::vector<Link> getLinks() { return links; }
	int getNodeNum(Node&);
	Truss operator= (Truss&);

	~Truss();
};

#endif