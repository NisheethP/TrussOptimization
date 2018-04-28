#include "Link.h"
#include <cmath>


Link::Link(Node n1, Node n2, int ar)
{
	nodes[0] = n1;
	nodes[1] = n2;

	double dx = n2.x - n1.x;
	double dy = n2.y - n1.y;

	length = sqrt(dx*dx + dy * dy);

	slope = atan2(dy,dx);
	area = ar;
	E = 200e9;
}


bool Link::operator==(Link pLink)
{
	if (area == pLink.area && length == pLink.length && slope == pLink.slope)
	{
		if (nodes[0] == pLink.nodes[0] && nodes[1] == pLink.nodes[1])
			return true;
	}

	return false;
}

Link & Link::operator=(Link link)
{
	area = link.area;
	length = link.length;
	slope = link.slope;
	E = link.E;
	nodes[0] = link.nodes[0];
	nodes[1] = link.nodes[1];
	return *this;
}

Link::~Link()
{
}
