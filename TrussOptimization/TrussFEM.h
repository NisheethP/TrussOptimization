#pragma once
#ifndef _TRUSS_FEM_H_
#define _TRUSS_FEM_H_

#include "Truss.h"
#include <Eigen/Sparse>
#include <Eigen/Dense>

/*
*Header file to solve the FEM solution of the Truss
*/

using Matrix4d = Eigen::Matrix<double, 4, 4>;
using VectorXd = Eigen::Matrix<double, Eigen::Dynamic, 1>;

enum OptCond
{
	MeanCompliance,
	NodeDisplacement
};

class TrussFEM
{

	Truss* truss;
	Eigen::SparseMatrix<double> globalStiffness;		//Sparse matrix to store the global stiffness
	VectorXd displacement;
	VectorXd force;
	std::vector<int> presDisp;							//List of node_dofs with prescribed displacements
	VectorXd virtualDisp;								//Displacement for the Virtual force at optNode dof
	int optNode;										//Force at which dof
	OptCond mode;										//Solve for virtual force if mode = NodeDisplacement

public:
	TrussFEM(Truss* truss, int pOptNode = 2, OptCond pMode = OptCond::MeanCompliance);
	
	static Matrix4d generateLocalStiffness(Link& link);	//Calculates the stiffness of the passed link
	void assembleGlobal();								//Assembles the Global stiffness for the given Truss, using the local stiffness data
	
	bool applyForceX(int nodeNum, double value);		//Add 'value' force in X direction to the given 'nodeNum' in the 'force' vector
	bool applyForceY(int nodeNum, double value);		//Add 'value' force in X direction to the given 'nodeNum' in the 'force' vector
	bool applySimpleSupport(int nodeNum);				//Adds ux=0 and uy=0 value to the given 'nodeNum' in the 'displacement' vector
	bool applyRollerSupport(int nodeNum);				//Adds ux=0 value to the given 'nodeNum' in the 'displacement' vector
	bool setOptNode(int dofNum);						//Sets the dof that is to be 

	bool isPrescribed(int dof);
	bool solve();

	static Eigen::SparseMatrix<double> removeRowCol(int dof, Eigen::SparseMatrix<double> mat);

	VectorXd getDisplacement() { return displacement; }
	VectorXd getForce() { return force; }
	VectorXd getVirtualDisp() { return virtualDisp; }
	
	~TrussFEM();
};

#endif // _TRUSS_FEM_H_