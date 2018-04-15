#include "Truss.h"



Truss::Truss(std::vector<Node>& pNodes , std::vector<Link>& pLinks) :
	nodes(pNodes),
	links(pLinks)
{
}
Truss::Truss():
	nodes(0),
	links(0)
{
}
Truss::Truss(Truss& truss)
{
	nodes = truss.getNodes();
	links = truss.getLinks();
}

int Truss::getNodeNum(Node & pNode)
{
	for (int i = 0; i < nodes.size(); i++)
	{
		if (nodes[i] == pNode)
			return i;
	}

	return -1;
}

Truss Truss::operator=(Truss & truss)
{
	this->nodes = truss.getNodes();
	this->links = truss.getLinks();
	return *this;
}

Truss::~Truss()
{
}
