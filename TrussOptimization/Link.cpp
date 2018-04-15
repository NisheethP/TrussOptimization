#include "Link.h"
#include <cmath>


Link::Link(Node n1, Node n2)
{
	nodes[0] = n1;
	nodes[1] = n2;

	double dx = n2.x - n1.x;
	double dy = n2.y - n1.y;

	length = sqrt(dx*dx + dy * dy);

	slope = atan2(dy,dx);
	area = 1;
	E = 1;
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

Link::~Link()
{
}
