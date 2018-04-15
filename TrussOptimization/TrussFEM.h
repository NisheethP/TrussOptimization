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

class TrussFEM
{

	Truss* truss;
	Eigen::SparseMatrix<double> globalStiffness;		//Sparse matrix to store the global stiffness
	VectorXd displacement;
	VectorXd force;
	std::vector<int> presDisp;							//List of node_dofs with prescribed displacements
	VectorXd soln;										//Lists the force on prescribed dofs. Displacements on the other dofs.
public:
	TrussFEM(Truss* truss);
	
	static Matrix4d generateLocalStiffness(Link& link);	//Calculates the stiffness of the passed link
	void assembleGlobal();								//Assembles the Global stiffness for the given Truss, using the local stiffness data
	bool applyForceX(int nodeNum, double value);		//Add 'value' force in X direction to the given 'nodeNum' in the 'force' vector
	bool applyForceY(int nodeNum, double value);		//Add 'value' force in X direction to the given 'nodeNum' in the 'force' vector
	bool applySimpleSupport(int nodeNum);				//Adds ux=0 and uy=0 value to the given 'nodeNum' in the 'displacement' vector
	bool applyRollerSupport(int nodeNum);				//Adds ux=0 value to the given 'nodeNum' in the 'displacement' vector

	~TrussFEM();
};

#endif // _TRUSS_FEM_H_