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

void Truss::initArea(double ar)
{
	for (std::vector<Link>::iterator it = links.begin(); it != links.end(); ++it)
	{
		it->setArea(ar);
	}
}

void Truss::updateArea(int link, double area)
{
	links[link].setArea(area);
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
