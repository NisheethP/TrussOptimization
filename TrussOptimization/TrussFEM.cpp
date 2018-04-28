#include "TrussFEM.h"
#include <cmath>
#include <iostream>
#include <stdlib.h>
#include <algorithm>

TrussFEM::TrussFEM(Truss* pTruss, int pOptNode, OptCond pMode) :
	truss(pTruss),
	displacement(truss->getNodes().size() * 2),	//2 DOF per node
	force(truss->getNodes().size() * 2),
	globalStiffness(truss->getNodes().size() * 2, truss->getNodes().size() * 2),
	virtualDisp(truss->getNodes().size() * 2),
	mode(pMode),
	optNode(pOptNode)
{
	force.setZero();
	displacement.setZero();
	virtualDisp.setZero();
	globalStiffness.setZero();


}

Matrix4d TrussFEM::generateLocalStiffness(Link & link)
{
	double k = link.getArea()*link.getE()/link.getLength(); //STIFFNESS CONSTANT OF LINK
	double q = link.getSlope();	//SLOPE OF THE ROD
	double K1 = cos(q)*cos(q);
	double K2 = cos(q)*sin(q);
	double K3 = sin(q)*sin(q);
	if (K1 < 1e-15)
		K1 = 0;
	if (K2 < 1e-15)
		K2 = 0;
	if (K3 < 1e-15)
		K3 = 0;

	Matrix4d stiffness;

	stiffness(0, 0) = K1;
	stiffness(0, 1) = K2;
	stiffness(0, 2) = -1*K1;
	stiffness(0, 3) = -1*K2;
	stiffness(1, 1) = K3;
	stiffness(1, 2) = -1*K2;
	stiffness(1, 3) = -1*K3;
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

double TrussFEM::getVol()
{
	double vol = 0;
	for (int i = 0; i < truss->getLinks().size(); i++)
		vol += truss->getLinks().at(i).getLength()*truss->getLinks().at(i).getArea();

	return vol;
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
	int redSize = force.size() - presDisp.size();
	Eigen::SparseMatrix<double> reducedStiffness(redSize, redSize);
	Eigen::SparseMatrix<double> temp = globalStiffness;

	//std::cout << globalStiffness << std::endl;

	for (int i = 0; i < presDisp.size(); i++)
	{
		std::vector<int> sorted(presDisp);
		std::sort(sorted.begin(), sorted.end());
		temp = removeRowCol(presDisp[i]-i, temp);
	}
	
	reducedStiffness = temp;
	
	//std::cout << "\nReduced stiffness:" <<reducedStiffness << std::endl;

	VectorXd b(redSize), x(redSize);
	int j = 0;
	for (int i = 0; i < force.size(); i++)
	{
		if (!isPrescribed(i))
		{
			b(j) = force(i);
			j++;
		}
	}
	

	Eigen::SimplicialLDLT<Eigen::SparseMatrix<double> > solver;
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
	
	j = 0;
	for (int i = 0; i < displacement.size(); i++)
	{
		if (!isPrescribed(i))
		{
			displacement(i) = x(j);
			j++;
		}
	}
		
	j = 0;
	//Solving for displacement from virtual force;
	if (mode == OptCond::NodeDisplacement)
	{
		VectorXd tempForce;
		tempForce.resize(force.size());
		tempForce.setZero();
		for (int i = 0; i < force.size(); i++)
		{
			if (i == optNode)
				tempForce(i) = 1;
		}

		int j = 0;
		for (int i = 0; i < force.size(); i++)
		{
			if (!isPrescribed(i))
			{
				b(j) = tempForce(i);
				j++;
			}
		}

		x = solver.solve(b);
		if (solver.info() != Eigen::Success)
		{
			std::cout << "Virtual Displacement solve failed";
			return false;
		}
	}
	

	return true;
}

Eigen::SparseMatrix<double> TrussFEM::removeRowCol(int dof, Eigen::SparseMatrix<double> mat)
{
	
	using MatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
	MatrixXd tempMatrix;
	int size = mat.outerSize();
	tempMatrix.resize(size, size);
	
	tempMatrix.setZero(); 
	
	for (int k = 0; k < mat.outerSize(); k++)
	{
		for (Eigen::SparseMatrix<double>::InnerIterator iter(mat, k); iter; ++iter)
		{
			int i = iter.row();
			int j = iter.col();
			tempMatrix(i, j) = iter.value();
		}
	}
	Eigen::SparseMatrix<double> temp2;
	std::vector<Eigen::Triplet<double>> trips;
	if (dof < size)
	{
		temp2.resize(size-1,size-1);
		temp2.setZero();
		
		for (int i = 0; i < size-1; i++)
		{
			for (int j = 0; j < size-1; j++)
			{
				int a = i, b = j;
				if (i < dof && j >= dof)
					b++;
				if (i >= dof && j < dof)
					a++;
				if (i >= dof && j >= dof)
				{
					a++; 
					b++;
				}
				trips.push_back(Eigen::Triplet<double>(i,j,tempMatrix(a,b)));
			}
		}
	}

	temp2.setFromTriplets(trips.begin(), trips.end());
	return temp2;
}

Eigen::Matrix<double, 4, 1> TrussFEM::getLinkDisp(Link & link)
{
	int n1 = truss->getNodeNum(link.getNode()[0]);
	int n2 = truss->getNodeNum(link.getNode()[1]);
	Eigen::Matrix<double, 4, 1> tempVec;
	tempVec(0) = displacement(n1 * 2);
	tempVec(1) = displacement(n1 * 2 + 1);
	tempVec(2) = displacement(n2 * 2);
	tempVec(3) = displacement(n2* 2 + 1);

	return tempVec;
}

TrussFEM::~TrussFEM()
{
}
