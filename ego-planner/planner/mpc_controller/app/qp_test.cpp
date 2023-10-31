#include <qpOASES.hpp>
#include <Eigen/Eigen>
#include "iostream"
int main( )
{
    USING_NAMESPACE_QPOASES
/* Setup data of first QP. */
    real_t H[2*2] = { 1.0, 0.0, 0.0, 0.5 };
    real_t A[1*2] = { 1.0, 1.0 };
    real_t g[2] = { 1.5, 1.0 };
    real_t lb[2] = { 0.5, -2.0 };
    real_t ub[2] = { 5.0, 2.0 };
    real_t lbA[1] = { -1.0 };
    real_t ubA[1] = { 2.0 };
/* Setup data of second QP. */
    real_t g_new[2] = { 1.0, 1.5 };
    real_t lb_new[2] = { 0.0, -1.0 };
    real_t ub_new[2] = { 5.0, -0.5 };
    real_t lbA_new[1] = { -2.0 };
    real_t ubA_new[1] = { 1.0 };
/* Setting up QProblem object. */
    QProblem example( 2,1 );
/* Solve first QP. */
    int_t nWSR = 10;
    example.init( H,g,A,lb,ub,lbA,ubA, nWSR );
/* Solve second QP. */
    nWSR = 10;
    example.hotstart( g_new,lb_new,ub_new,lbA_new,ubA_new, nWSR );
/* Get and print solution of second QP. */
    real_t xOpt[2];
    example.getPrimalSolution( xOpt );
//    printf( "\n xOpt = [ %e, %e ]; objVal = %e\n\n",
//            xOpt[0],xOpt[1],example.getObjVal() );
    Eigen::MatrixXd test_matrix = Eigen::MatrixXd::Identity(3,3);
    test_matrix(0,1) = 1;
    //std::cout<<test_matrix;

    return 0;
}