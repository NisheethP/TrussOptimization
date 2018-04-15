#include "TrussFEM.h"
#include <cmath>


TrussFEM::TrussFEM(Truss* pTruss) :
	truss(pTruss),
	displacement(truss->getNodes().size() * 2),	//2 DOF per node
	force(truss->getNodes().size() * 2),
	globalStiffness(truss->getNodes().size() * 2, truss->getNodes().size() * 2),
	soln(truss->getNodes().size() * 2)
{
	for (int i = 0; i < force.size(); i++)
	{
		force(i) = 0;
		displacement(i) = 0;
	}
	
}

Matrix4d TrussFEM::generateLocalStiffness(Link & link)
{
	double k = link.getArea()*link.getE()/link.getLength(); //STIFFNESS CONSTANT OF LINK
	double q = link.getSlope();	//SLOPE OF THE ROD
	double K1 = cos(q)*cos(q);
	double K2 = cos(q)*sin(q);
	double K3 = sin(q)*sin(q);

	Matrix4d stiffness;

	stiffness(0, 0) = K1;
	stiffness(0, 1) = K2;
	stiffness(0, 2) = -K1;
	stiffness(0, 3) = -K2;
	stiffness(1, 1) = K3;
	stiffness(1, 2) = -K2;
	stiffness(1, 3) = -K3;
	stiffness(2, 2) = K1;
	stiffness(2, 3) = K2;
	stiffness(3, 3) = K3;
	
	for (int i = 0; i < 4; i++)
	{
		for (int j = i; j < 4; j++)
		{
			stiffness(j, i) = stiffness(i, j);
		}
	}
	
	return stiffness;
}

void TrussFEM::assembleGlobal()
{
	using T = Eigen::Triplet<double>;
	std::vector<T> values;
	for (std::vector<Link>::iterator it = truss->getLinks().begin(); it != truss->getLinks().end(); ++it)
	{
		Matrix4d localStiff(generateLocalStiffness(*it));
		
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				T temp;
				temp.value = localStiff(i, j);
				int ni = truss->getNodeNum(truss->getNodes()[static_cast<int>(i / 2)]);
				int nj = truss->getNodeNum(truss->getNodes()[static_cast<int>(j / 2)]);
				
				temp.row = ni * 2 + i % 2;
				temp.col = nj * 2 + j % 2;

				values.push_back(temp);
			}
		}
	}
	globalStiffness.setFromTriplets(values.begin(), values.end());
}

bool TrussFEM::applyForceX(int nodeNum, double value)
{
	if (nodeNum*2 > force.size())
		return false;
	else
		force(nodeNum*2) = value;
	
	return true;
}

bool TrussFEM::applyForceY(int nodeNum, double value)
{
	if (nodeNum*2 > force.size())
		return false;
	else
		force(nodeNum*2+1) = value;

	return true;
}

bool TrussFEM::applySimpleSupport(int nodeNum)
{
	if (nodeNum*2 >= force.size())
		return false;
	else
	{
		displacement(nodeNum * 2) = 0;
		displacement(nodeNum * 2+1) = 0;
		presDisp.push_back(nodeNum * 2);
		presDisp.push_back(nodeNum * 2 +1 );
	}

	return true;
}

bool TrussFEM::applyRollerSupport(int nodeNum)
{
	if (nodeNum * 2 >= force.size())
		return false;
	else
	{
		displacement(nodeNum * 2 + 1) = 0;
		presDisp.push_back(nodeNum * 2);
	}
	
	return true;
}


TrussFEM::~TrussFEM()
{
}
