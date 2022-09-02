#include "trajectory_generator_waypoint.h"
#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;    
using namespace Eigen;

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint(){}
TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint(){}

//define factorial function, input i, output i!
int TrajectoryGeneratorWaypoint::Factorial(int x)
{
    int fac = 1;
    for(int i = x; i > 0; i--)
        fac = fac * i;
    return fac;
}
/*

    STEP 2: Learn the "Closed-form solution to minimum snap" in L5, then finish this PolyQPGeneration function

    variable declaration: input       const int d_order,                    // the order of derivative
                                      const Eigen::MatrixXd &Path                                      const Eigen::MatrixXd &Vel,           // boundary velocity
                                      const Eigen::MatrixXd &Acc,           // boundary acceleration
                                      const Eigen::VectorXd &Time)          // time allocation in each segment
                          output      MatrixXd PolyCoeff(m, 3 * p_num1d);   // position(x,y,z), so we need (3 * p_num1d) coefficients

*/

Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
            const int d_order,                    // the order of derivative
            const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
            const Eigen::MatrixXd &Vel,           // boundary velocity
            const Eigen::MatrixXd &Acc,           // boundary acceleration
            const Eigen::VectorXd &Time)          // time allocation in each segment
{
    // enforce initial and final velocity and accleration, for higher order derivatives, just assume them be 0;
    int p_order   = 2 * d_order - 1;              // the order of polynomial
    int p_num1d   = p_order + 1;                  // the number of variables in each segment

    int m = Time.size();                          // the number of segments
    int n = Path.rows();
    MatrixXd PolyCoeff = MatrixXd::Zero(m, 3 * p_num1d);           // position(x,y,z), so we need (3 * p_num1d) coefficients
    VectorXd Px(p_num1d * m), Py(p_num1d * m), Pz(p_num1d * m);

    /*   Produce Mapping Matrix A to the entire trajectory, A is a mapping matrix that maps polynomial coefficients to derivatives.   */
    MatrixXd A = MatrixXd::Zero(p_num1d * m, p_num1d * m);
    MatrixXd A_j = MatrixXd::Zero(p_num1d, p_num1d);
    for (int i = 0; i < m; i++)
    {
	    double t = Time[i];
	    for (int j = 0; j < d_order; j++)
	    {
		    for (int k = 0; k < p_num1d; k++)
		    {
			    if (k == j) A_j(j, k) = Factorial(k);
		    }
	    }

	    for (int j = d_order; j < p_num1d; j++)
	    {
		    for (int k = 0; k < p_num1d; k++)
		    {
			    if (k >= j - d_order)
			    {
				    A_j(j, k) = pow(t, k - (j - d_order)) * Factorial(k) / Factorial(k - (j - d_order));
			    }
		    }
	    }
	    A.block(i * p_num1d, i * p_num1d, p_num1d, p_num1d) = A_j;
    }
     std::cout << "Matrix A size and content" << std::endl;
     std::cout << A.rows() << "	" << A.cols() << std::endl;
     std::cout << A << std::endl;

    /*   Produce the dereivatives in X, Y and Z axis directly.  */

    int dF_size = n + 2 * (d_order - 1);
    int dP_size = n * d_order - dF_size;
    VectorXd d_Fx = VectorXd::Zero(dF_size),d_Fy = VectorXd::Zero(dF_size),d_Fz = VectorXd::Zero(dF_size); 
    
    d_Fx[0] = Path(0, 0);
    d_Fy[0] = Path(0, 1);
    d_Fz[0] = Path(0, 2);
    d_Fx[d_order] = Path(n - 1, 0);
    d_Fy[d_order] = Path(n - 1, 1);
    d_Fz[d_order] = Path(n - 1, 2);

    d_Fx[1] = Vel(0, 0); // Velocity of first point (start point)
    d_Fy[1] = Vel(0, 1);
    d_Fz[1] = Vel(0, 2);

    d_Fx[d_order + 1] = Vel(1, 0); // Acceleration of first point (start point)
    d_Fy[d_order + 1] = Vel(1, 1);
    d_Fz[d_order + 1] = Vel(1, 2);

    d_Fx[2] = Acc(0, 0); // Velocity of last point (end point)
    d_Fy[2] = Acc(0, 1);
    d_Fz[2] = Acc(0, 2);

    d_Fx[d_order + 2] = Acc(1, 0); // Acceleration of last point (end point)
    d_Fy[d_order + 2] = Acc(1, 1);
    d_Fz[d_order + 2] = Acc(1, 2);

    for (int i = 1; i < n - 1; i++)
    {
	    d_Fx[i + 2 * d_order -1] = Path(i, 0);
	    d_Fy[i + 2 * d_order -1] = Path(i, 1);
	    d_Fz[i + 2 * d_order -1] = Path(i, 2);
    }

    /*   Produce the Minimum Snap cost function, the Hessian Matrix   */
    MatrixXd Q = MatrixXd::Zero(p_num1d * m, p_num1d * m);
    MatrixXd Q_j = MatrixXd::Zero(p_num1d, p_num1d);
    for (int i = 0; i < m; i++)
    {
	    double t = Time[i];
	    for (int j = 0; j < p_num1d; j++)
	    {
		    for (int k = 0; k < p_num1d; k++)
		    {
			    if (j >= d_order && k >= d_order )
			    {
				    Q_j(j, k) = Factorial(j) * Factorial(k) * pow(t, j+k-p_order) / (Factorial(j-d_order ) * Factorial(k-d_order ) * (j+k-p_order));
			    }
			    else Q_j(j, k) = 0;
		    } 
	    }
	    Q.block(p_num1d * i, p_num1d * i, p_num1d, p_num1d) = Q_j;
    }
    std::cout << "Matrix Q size and content" << std::endl;
    std::cout << Q.rows() << "	" << A.cols() << std::endl;
    std::cout << Q << std::endl;

    // std::cout << "Test flag" << std::endl;

    /*	 Produce Matrix C_trans(m * p_num1d, Path.rows() * d_order) & C   */
    MatrixXd C_trans = MatrixXd::Zero(m * 2 * d_order, n * d_order);
    MatrixXd C = MatrixXd::Zero(n * d_order, m * p_num1d);
    std::cout << C.rows() << "	" << C.cols() << std::endl;
    for (int i = 0; i < d_order; i++)  // start point and end point
    {
	    C_trans(i, i) = 1;
	    C_trans(m * p_num1d - d_order + i, i + d_order) =  1;
    }

    for (int i = 0; i < m - 1; i++)
    {
	    for (int j = 0; j < d_order; j++)
	    {
		    C_trans(i * p_num1d + 1 * d_order + j,i + 2 * d_order + j * (n - 2)) = 1; // point in last traj;
		    C_trans(i * p_num1d + 2 * d_order + j,i + 2 * d_order + j * (n - 2)) = 1; // point in last traj;
	    }
    }
    C = C_trans.transpose();

    std::cout << "Matrix C size and content" << std::endl;
    std::cout << C_trans << std::endl;

    /*   To solve Px, Py, Pz, we need to have A_inverse, R, R_block = -R_PP.inverse()*R_FP.transpose()    */
    MatrixXd A_inver = MatrixXd::Zero(p_num1d * m, p_num1d * m);
    MatrixXd R = MatrixXd::Zero(n * d_order, n * d_order);
    MatrixXd R_block = MatrixXd::Zero(dP_size, dF_size);
    A_inver = A.inverse();
    R = C * A_inver.transpose() * Q * A_inver * C_trans;
    R_block = -R.block(dF_size, dF_size, dP_size, dP_size).inverse() * R.block(0, dF_size, dF_size, dP_size).transpose();
    std::cout << "R.block(0, dF_size, dF_size, dP_size).transpose()" << std::endl;
    std::cout << R.block(0, dF_size, dF_size, dP_size).transpose() << std::endl;
    std::cout << "R.block(dF_size, dF_size, dP_size, dP_size).inverse()" << std::endl;
    std::cout << R.block(dF_size, dF_size, dP_size, dP_size).inverse() << std::endl;
    std::cout << "A_inver.transpose()" << std::endl;
    std::cout << A_inver.transpose() << std::endl;
    std::cout << "A_inver" << std::endl;
    std::cout << A_inver << std::endl;
    std::cout << "R" << std::endl;
    std::cout << R << std::endl;

    // Calculate dPx, dPy, dPz, dx, dy, dz
    VectorXd d_Px = VectorXd::Zero(dP_size),d_Py = VectorXd::Zero(dP_size),d_Pz = VectorXd::Zero(dP_size);
    d_Px = R_block * d_Fx;
    d_Py = R_block * d_Fy;
    d_Pz = R_block * d_Fz;
    std::cout << "R_block" << std::endl;
    std::cout << R_block << std::endl;
    std::cout << "d_Px" << std::endl;
    std::cout << d_Px << std::endl;

    VectorXd dx = VectorXd::Zero(dF_size + dP_size);
    VectorXd dy = VectorXd::Zero(dF_size + dP_size);
    VectorXd dz = VectorXd::Zero(dF_size + dP_size);

    dx << d_Fx, d_Px;
    dy << d_Fy, d_Py;
    dz << d_Fz, d_Pz;
    std::cout << "dx" << std::endl;
    std::cout << dx << std::endl;
    std::cout << "dy" << std::endl;
    std::cout << dy << std::endl;
    std::cout << "dz" << std::endl;
    std::cout << dz << std::endl;

    // Calculate Px, Py, Pz
    Px = A_inver * C_trans * dx;
    Py = A_inver * C_trans * dy;
    Pz = A_inver * C_trans * dz;
    std::cout << "A_inver" << std::endl;
    std::cout << A_inver << std::endl;
    std::cout << "C_trans" << std::endl;
    std::cout << C_trans << std::endl;

    for (int i = 0; i < m; i++)
    {
	    std::cout << "i = " << i << std::endl;
	    PolyCoeff.block(i, 0 * p_num1d, 1, p_num1d) = Px.segment(i * p_num1d, p_num1d).transpose(); 
	    PolyCoeff.block(i, 1 * p_num1d, 1, p_num1d) = Py.segment(i * p_num1d, p_num1d).transpose(); 
	    PolyCoeff.block(i, 2 * p_num1d, 1, p_num1d) = Pz.segment(i * p_num1d, p_num1d).transpose(); 
    }

    std::cout << "PolyCoeff" << std::endl;
    std::cout << PolyCoeff.transpose() << std::endl;
    return PolyCoeff;
}
