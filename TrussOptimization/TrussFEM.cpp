#include "TrussFEM.h"
#include <cmath>
#include <iostream>
#include <stdlib.h>

TrussFEM::TrussFEM(Truss* pTruss) :
	truss(pTruss),
	displacement(truss->getNodes().size() * 2),	//2 DOF per node
	force(truss->getNodes().size() * 2),
	globalStiffness(truss->getNodes().size() * 2, truss->getNodes().size() * 2),
	virtualDisp(truss->getNodes().size() * 2)
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
	
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			stiffness(i, j) *= k;
		}
	}

	//std::cout << std::endl << stiffness << std::endl;
	return stiffness;
}

void TrussFEM::assembleGlobal()
{
	using T = Eigen::Triplet<double>;
	std::vector<T> values;
	std::vector<Link> tempLinks(truss->getLinks());
	for (std::vector<Link>::iterator it = tempLinks.begin(); it != tempLinks.end(); ++it)
	{
		Link tempLink = *it;
		Matrix4d localStiff(generateLocalStiffness(tempLink));
		
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				int ni = truss->getNodeNum(tempLink.getNode()[static_cast<int>(i / 2)]);
				int nj = truss->getNodeNum(tempLink.getNode()[static_cast<int>(j / 2)]);
				int R = (ni * 2 + i % 2);
				int C = (nj * 2 + j % 2);
				double val = localStiff(i, j);
				//std::cout << R << '\t' << C << '\t' << val << '\n';
				T temp(R,C,val);

				values.push_back(temp);
			}
		}
	}
	globalStiffness.setFromTriplets(values.begin(), values.end());
	//std::cout << globalStiffness;
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
		presDisp.push_back(nodeNum * 2+1);
	}
	
	return true;
}

bool TrussFEM::setOptNode(int dofNum)
{
	if (dofNum * 2 >= force.size())
		return false;
	else
		optNode = dofNum;
	
	return true;
}


bool TrussFEM::isPrescribed(int dof)
{
	for (std::vector<int>::iterator iter = presDisp.begin(); iter != presDisp.end(); ++iter)
		if (dof == *iter)
			return true;
	return false;
}

bool TrussFEM::solve()
{
	using T = Eigen::Triplet<double>;
	int redSize = force.size() - presDisp.size();
	Eigen::SparseMatrix<double> reducedStiffness(force.size(), force.size());
	std::vector<T> trips;
	for (int k = 0; k < globalStiffness.outerSize(); k++)
	{
		for (Eigen::SparseMatrix<double>::InnerIterator iter(globalStiffness, k); iter; ++iter)
		{
			int i = iter.row();
			int j = iter.col();

			//printf("%d - %d \t %d - %d \t %f => %d \n", i, isPrescribed(i), j, isPrescribed(j), iter.value(), !isPrescribed(i) && !isPrescribed(j));
			if (!isPrescribed(i) && !isPrescribed(j) )
			{
				T temp(iter.row(), iter.col(), iter.value());
				trips.push_back(temp);
			}
		}
		printf("====================\n");
	}
	reducedStiffness.setFromTriplets(trips.begin(), trips.end());

	VectorXd b(force.size()-presDisp.size()), x(force.size() - presDisp.size());
	for (int i = 0; i < force.size(); i++)
	{
		if (!isPrescribed(i))
			b(i) = force(i);
	}

	Eigen::SimplicialLLT<Eigen::SparseMatrix<double> > solver;
	solver.compute(reducedStiffness);
	if (solver.info() != Eigen::Success)
	{
		std::cout << "Decomposition FAILED";
		return false;
	}

	x = solver.solve(b);
	
	if (solver.info() != Eigen::Success)
	{
		std::cout << "Solution FAILED";
		return false;
	}
	
	for (int i = 0; i < displacement.size(); i++)
	{
		if (!isPrescribed(i))
			displacement(i) = x(i);
	}

	//Solving for displacement from virtual force;
	for (int i = 0; i < force.size(); i++)
	{
		if (!isPrescribed(i))
		{
			if (i == optNode)
				b(i) = 1;
			else
				b(i) = 0;
		}
	}


	return true;
}

TrussFEM::~TrussFEM()
{
}
