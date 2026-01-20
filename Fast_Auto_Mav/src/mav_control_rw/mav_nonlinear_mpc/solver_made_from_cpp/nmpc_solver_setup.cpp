/*
 *    file:   nmpc_solver_setup.cpp
 *    author: Oskar Ljungqvist
 *    date:   2017-12-21
 *
 *    Comment: modified version of the nmpc_solver_setup.m works directly in ubutu.
 */

#include <acado_code_generation.hpp>

#include <acado_toolkit.hpp>
#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>
#include <matrix_vector/vector.hpp>

//#include <acado_optimal_control.hpp>
//#include <acado_gnuplot.hpp>

USING_NAMESPACE_ACADO

int main( )
{

    double Ts = 0.1;  // prediction sampling time
    double N  = 20;   // Prediction horizon
    double g = 9.8066;
    double PI = 3.1415926535897932;

    DifferentialState velocity1; //velocity x_w
    DifferentialState velocity2; //velocity y_w
    DifferentialState velocity3; //velocity z_w
    DifferentialState roll;
    DifferentialState pitch;
    DifferentialState yaw;
    DifferentialState position1; //x_w
    DifferentialState position2; //y_w
    DifferentialState position3; //z_w
    Control roll_ref;
    Control pitch_ref;
    Control thrust;

    OnlineData roll_tau;
    OnlineData roll_gain;
    OnlineData pitch_tau;
    OnlineData pitch_gain;
    OnlineData linear_drag_coefficient1;
    OnlineData linear_drag_coefficient2;
    OnlineData external_forces1;
    OnlineData external_forces2;
    OnlineData external_forces3;

    // Non-linear drag
    IntermediateState dragacc1 =   sin(pitch)*linear_drag_coefficient1*thrust*velocity3
                                 + cos(pitch)*cos(yaw)*linear_drag_coefficient1*thrust*velocity1
                                 - cos(pitch)*linear_drag_coefficient1*sin(yaw)*thrust*velocity2;
    IntermediateState dragacc2 =   (cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))*linear_drag_coefficient2*thrust*velocity1
                                 - (cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw))*linear_drag_coefficient2*thrust*velocity2
                                 - cos(pitch)*linear_drag_coefficient2*sin(roll)*thrust*velocity3;

    // Model equations:
    DifferentialEquation f;

    f << dot(velocity1)   == ((cos(roll)*cos(yaw)*sin(pitch) + sin(roll)*sin(yaw))*thrust - dragacc1 + external_forces1);
    f << dot(velocity2)   == ((cos(roll)*sin(pitch)*sin(yaw) - cos(yaw)*sin(roll))*thrust - dragacc2 + external_forces2);
    f << dot(velocity3)   == (-g + cos(pitch)*cos(roll)*thrust + external_forces3);
    f << dot( roll )      == (roll_gain*roll_ref - roll)/roll_tau;
    f << dot( pitch )     == (pitch_gain*pitch_ref - pitch)/pitch_tau;
    f << dot( yaw )       == 0;
    f << dot( position1 ) == velocity1;
    f << dot( position2 ) == velocity2;
    f << dot( position3 ) == velocity3;

    // Reference functions and weighting matrices:
    Function h;
    // state
    h << position1 << position2 << position3;
    h << velocity1 << velocity2 << velocity3;
    h << roll      << pitch;
    // control
    h << roll_ref  << pitch_ref << (cos(pitch)*cos(roll)*thrust - g);//acc_z

    Function hN;
    hN << position1 << position2 << position3;
    hN << velocity1 << velocity2 << velocity3;

    // Provide defined weighting matrices:
    BMatrix W  = eye<bool>(h.getDim());
    BMatrix WN = eye<bool>(hN.getDim());

    // Define OCP problem:
    OCP ocp(0.0, N*Ts, N);

    ocp.subjectTo(f);

    BMatrix Q_sparse(h.getDim(), h.getDim());
    Q_sparse.setIdentity();
    BMatrix QN_sparse(hN.getDim(), hN.getDim());
    QN_sparse.setIdentity();
    ocp.minimizeLSQ( Q_sparse, h);
    ocp.minimizeLSQEndTerm( QN_sparse, hN );
    // ocp.minimizeLSQ(W, h);
    // ocp.minimizeLSQEndTerm(WN, hN);

    // Dummy constraints, real ones set online
    ocp.subjectTo(-10 <= velocity1  <= 10);
    ocp.subjectTo(-10 <= velocity2  <= 10);
    ocp.subjectTo(  -10 <= velocity3  <= 10);
    ocp.subjectTo(-45*PI/180 <= roll_ref  <= 45*PI/180);
    ocp.subjectTo(-45*PI/180 <= pitch_ref <= 45*PI/180);
    ocp.subjectTo(     g/2.0 <= thrust    <= g*1.5);

    // Export the code:
    OCPexport mpc( ocp );
    mpc.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON);
    mpc.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
    mpc.set( SPARSE_QP_SOLUTION, FULL_CONDENSING_N2); //FULL_CONDENsinG_N2
    mpc.set( INTEGRATOR_TYPE, INT_IRK_GL4);
    //mpc.set( NUM_INTEGRATOR_STEPS, N);
    mpc.set( QP_SOLVER, QP_QPOASES);
    mpc.set( HOTSTART_QP, YES);
    mpc.set( LEVENBERG_MARQUARDT, 1.0);
    mpc.set( LINEAR_ALGEBRA_SOLVER, GAUSS_LU);
    mpc.set( IMPLICIT_INTEGRATOR_NUM_ITS, 2);
    mpc.set( CG_USE_OPENMP, YES);
    mpc.set( CG_HARDCODE_CONSTRAINT_VALUES, NO);
    mpc.set( CG_USE_VARIABLE_WEIGHTING_MATRIX, NO);

//     mpc.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON);
//     mpc.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
//     mpc.set( SPARSE_QP_SOLUTION, FULL_CONDENSING_N2); //FULL_CONDENsinG_N2
//     mpc.set( INTEGRATOR_TYPE, INT_IRK_GL4);
//     //mpc.set( NUM_INTEGRATOR_STEPS, N);
//     mpc.set( QP_SOLVER, QP_QPOASES);
//     mpc.set( HOTSTART_QP, YES);
//     mpc.set( LEVENBERG_MARQUARDT, 1e-10);
//     mpc.set( LINEAR_ALGEBRA_SOLVER, GAUSS_LU);
//     mpc.set( IMPLICIT_INTEGRATOR_NUM_ITS, 2);
//     mpc.set( CG_USE_OPENMP, YES);
//     mpc.set( CG_HARDCODE_CONSTRAINT_VALUES, NO);
//     mpc.set( CG_USE_VARIABLE_WEIGHTING_MATRIX, NO);


    // mpc.set(HESSIAN_APPROXIMATION,  GAUSS_NEWTON);        // is robust, stable
    // mpc.set(DISCRETIZATION_TYPE,    MULTIPLE_SHOOTING);   // good convergence
    // mpc.set(SPARSE_QP_SOLUTION,     FULL_CONDENSING_N2);  // due to qpOASES
    // mpc.set(INTEGRATOR_TYPE,        INT_IRK_GL4);         // accurate
    // mpc.set(NUM_INTEGRATOR_STEPS,   N);
    // mpc.set(QP_SOLVER,              QP_QPOASES);          // free, source code
    // mpc.set(HOTSTART_QP,            YES);
    // mpc.set(LEVENBERG_MARQUARDT,    1.0);                 // Regularization
    // mpc.set(CG_USE_OPENMP,                    YES);       // paralellization
    // mpc.set(CG_HARDCODE_CONSTRAINT_VALUES,    NO);        // set on runtime
    // mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);       // time-varying costs
    // mpc.set( USE_SINGLE_PRECISION,        YES);           // Single precision

    // // Do not generate tests, makes or matlab-related interfaces.
    // mpc.set( GENERATE_TEST_FILE,          NO);
    // mpc.set( GENERATE_MAKE_FILE,          NO);
    // mpc.set( GENERATE_MATLAB_INTERFACE,   NO);
    // mpc.set( GENERATE_SIMULINK_INTERFACE, NO);
    if (mpc.exportCode( "." ) != SUCCESSFUL_RETURN)
            exit( EXIT_FAILURE );

    mpc.printDimensionsQP();

    return EXIT_SUCCESS;
}
