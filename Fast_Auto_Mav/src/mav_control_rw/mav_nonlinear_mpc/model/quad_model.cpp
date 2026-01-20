/*
 *    file:   quad_model.cpp
 *    author: zk
 *    date:   2024.7.24
 *
 *    Comment: 四旋翼全动态非线性mpc.
 */

#include <acado_code_generation.hpp>

#include <acado_toolkit.hpp>
#include <acado_optimal_control.hpp>
#include <acado_gnuplot.hpp>
#include <matrix_vector/vector.hpp>



USING_NAMESPACE_ACADO

int main( )
{
    /*
    Switch between code generation and analysis.

    If CODE_GEN is true the system is compiled into an optimizaiton problem
    for real-time iteration and all code to run it online is generated.
    Constraints and reference structure is used but the values will be set on
    runtinme.

    If CODE_GEN is false, the system is compiled into a standalone optimization
    and solved on execution. The reference and constraints must be set in here.
    */
    const bool CODE_GEN = true;

    double Ts = 0.1;  // prediction sampling time
    double N  = 20;   // Prediction horizon
    double g = 9.8066;
    double PI = 3.1415926535897932;

    // double J_x = 0.1;
    // double J_y = 0.1;
    // double J_z = 0.1;
    //限幅
    double vxy_max = 1.5;
    double vz_max = 1.5;
    double wxy_max = 1.5;
    double wz_max = 1.5;
    double tau_xy_max = 1;
    double tau_z_max = 1;
    double T_max = 20;
    double T_min = 5;

    DifferentialState p_x; //x_w
    DifferentialState p_y; //y_w
    DifferentialState p_z; //z_w
    DifferentialState v_x; //v x_w
    DifferentialState v_y; //v y_w
    DifferentialState v_z; //v z_w
    DifferentialState q_w;
    DifferentialState q_x;
    DifferentialState q_y;
    DifferentialState q_z;
    DifferentialState w_x;
    DifferentialState w_y;
    DifferentialState w_z;

    Control thrust;
    Control tau_x;
    Control tau_y;
    Control tau_z;


    OnlineData J_x;
    OnlineData J_y;
    OnlineData J_z;
    OnlineData linear_drag_coefficient1;
    OnlineData linear_drag_coefficient2;

    //暂时不添加
    // OnlineData external_forces1;
    // OnlineData external_forces2;
    // OnlineData external_forces3;
    

    // Non-linear drag
    // IntermediateState dragacc1 =   sin(pitch)*linear_drag_coefficient1*thrust*v3
    //                              + cos(pitch)*cos(yaw)*linear_drag_coefficient1*thrust*v1
    //                              - cos(pitch)*linear_drag_coefficient1*sin(yaw)*thrust*v2;
    // IntermediateState dragacc2 =   (cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))*linear_drag_coefficient2*thrust*v1
    //                              - (cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw))*linear_drag_coefficient2*thrust*v2
    //                              - cos(pitch)*linear_drag_coefficient2*sin(roll)*thrust*v3;

    // Model equations:
    DifferentialEquation f;

    f << dot( p_x ) == v_x;
    f << dot( p_y ) == v_y;
    f << dot( p_z ) == v_z;
    f << dot( v_x ) == 2*(q_w*q_y + q_x*q_z)*thrust;
    f << dot( v_y ) == 2*(q_y*q_z - q_w*q_x)*thrust;
    f << dot( v_z ) == (q_w*q_w - q_x*q_x - q_y*q_y + q_z*q_z)*thrust - g;
    f << dot( q_w ) == 0.5*(-w_x*q_x - w_y*q_y - w_z*q_z);
    f << dot( q_x ) == 0.5*(w_x*q_w + w_z*q_y - w_y*q_z);
    f << dot( q_y ) == 0.5*(w_y*q_w - w_z*q_x + w_x*q_z);
    f << dot( q_z ) == 0.5*(w_z*q_w + w_y*q_x - w_x*q_y);
    f << dot( w_x ) == (J_y - J_z)/J_x*w_y*w_z + tau_x/J_x;
    f << dot( w_y ) == (J_z - J_x)/J_y*w_z*w_x + tau_y/J_y;
    f << dot( w_z ) == (J_x - J_y)/J_z*w_x*w_y + tau_z/J_z;

    // Reference functions and weighting matrices:
    Function h;
    // state
    h << p_x << p_y << p_z;
    h << v_x << v_y << v_z;
    h << q_w << q_x << q_y << q_z;
    h << w_x << w_y << w_z;
    // control
    //这里选用的是z轴的加速度作为优化，而不是推力，因为这样可以不用设置u_ref,这样直接将控制量优化到0即可
    h << ((q_w*q_w - q_x*q_x - q_y*q_y + q_z*q_z)*thrust - g) << tau_x << tau_y << tau_z;

    Function hN;        
    hN << p_x << p_y << p_z;
    hN << v_x << v_y << v_z;
    hN << q_w << q_x << q_y << q_z;
    hN << w_x << w_y << w_z;

    // Running cost weight matrix
    DMatrix Q(h.getDim(), h.getDim());
    Q.setIdentity();
    Q(0,0) = 100;   // x
    Q(1,1) = 100;   // y
    Q(2,2) = 100;   // z
    Q(3,3) = 1;   // vx
    Q(4,4) = 1;   // vy
    Q(5,5) = 1;   // vz
    Q(6,6) = 1;   // qw
    Q(7,7) = 1;   // qx
    Q(8,8) = 1;   // qy
    Q(9,9) = 1;   // qz
    Q(10,10) = 1; // wx
    Q(11,11) = 1; // wy
    Q(12,12) = 1; // wz
    Q(13,13) = 1; // thrust
    Q(14,14) = 1; // tau_x
    Q(15,15) = 1; // tau_y
    Q(16,16) = 1; // tau_z

    // End cost weight matrix
    DMatrix QN(hN.getDim(), hN.getDim());
    QN.setIdentity();
    QN(0,0) = Q(0,0);   // x
    QN(1,1) = Q(1,1);   // y
    QN(2,2) = Q(2,2);   // z
    QN(3,3) = Q(3,3);   // vx
    QN(4,4) = Q(4,4);   // vx
    QN(5,5) = Q(5,5);   // vy
    QN(6,6) = Q(6,6);   // qw
    QN(7,7) = Q(7,7);   // qx
    QN(8,8) = Q(8,8);   // qy
    QN(9,9) = Q(9,9);   // qz
    QN(10,10) = Q(10,10); // wx
    QN(11,11) = Q(11,11); // wy
    QN(12,12) = Q(12,12); // wz

    BMatrix W  = eye<bool>(h.getDim());
    BMatrix WN = eye<bool>(hN.getDim());
    // Set a reference for the analysis (if CODE_GEN is false).
    // Reference is at x = 2.0m in hover (qw = 1).
    DVector r(h.getDim());    // Running cost reference
    r.setZero();
    r(0) = 2.0;
    r(1) = 2.0;
    r(2) = 2.0;
    r(6) = 1.0;


    DVector rN(hN.getDim());   // End cost reference
    rN.setZero();
    rN(0) = r(0);
    rN(1) = r(1);
    rN(2) = r(2);
    rN(6) = r(6);


    // Define OCP problem:
    OCP ocp(0.0, N*Ts, N);

    if(!CODE_GEN)
    {
        // For analysis, set references.
        ocp.minimizeLSQ( Q, h, r );
        ocp.minimizeLSQEndTerm( QN, hN, rN );
    }else{
        // // For code generation, references are set during run time.
        // BMatrix Q_sparse(h.getDim(), h.getDim());
        // Q_sparse.setIdentity();
        // BMatrix QN_sparse(hN.getDim(), hN.getDim());
        // QN_sparse.setIdentity();
        // ocp.minimizeLSQ( Q_sparse, h);
        // ocp.minimizeLSQEndTerm( QN_sparse, hN );
            ocp.minimizeLSQ(W, h);
         ocp.minimizeLSQEndTerm(WN, hN);
    }

    // Add system dynamics
    ocp.subjectTo( f );
    // Add constraints
    ocp.subjectTo( -vxy_max <= v_x <= vxy_max );
    ocp.subjectTo( -vxy_max <= v_y <= vxy_max );
    ocp.subjectTo( -vz_max <= v_z <= vz_max );
    ocp.subjectTo( -wxy_max <= w_x <= wxy_max );
    ocp.subjectTo( -wxy_max <= w_y <= wxy_max );
    ocp.subjectTo( -wz_max <= w_z <= wz_max );
    ocp.subjectTo( -tau_xy_max <= tau_x <= tau_xy_max );
    ocp.subjectTo( -tau_xy_max <= tau_y <= tau_xy_max );
    ocp.subjectTo( -tau_z_max <= tau_z <= tau_z_max );
    ocp.subjectTo( T_min <= thrust <= T_max );

    ocp.setNOD(5);


    if(!CODE_GEN)
    {
        // Set initial state
        ocp.subjectTo( AT_START, p_x ==  0.0 );
        ocp.subjectTo( AT_START, p_y ==  0.0 );
        ocp.subjectTo( AT_START, p_z ==  0.0 );
        ocp.subjectTo( AT_START, v_x ==  0.0 );
        ocp.subjectTo( AT_START, v_y ==  0.0 );
        ocp.subjectTo( AT_START, v_z ==  0.0 );
        ocp.subjectTo( AT_START, q_w ==  1.0 );
        ocp.subjectTo( AT_START, q_x ==  0.0 );
        ocp.subjectTo( AT_START, q_y ==  0.0 );
        ocp.subjectTo( AT_START, q_z ==  0.0 );
        ocp.subjectTo( AT_START, w_x ==  0.0 );
        ocp.subjectTo( AT_START, w_y ==  0.0 );
        ocp.subjectTo( AT_START, w_z ==  0.0 );
        ocp.subjectTo( AT_START, thrust ==  g );

        // Setup some visualization
        GnuplotWindow window1( PLOT_AT_EACH_ITERATION );
        window1.addSubplot( p_x,"p x" );
        window1.addSubplot( p_y,"p y" );
        window1.addSubplot( p_z,"p z" );
        window1.addSubplot( v_x,"v x" );
        window1.addSubplot( v_y,"v y" );
        window1.addSubplot( v_z,"v z" );
        GnuplotWindow window2( PLOT_AT_EACH_ITERATION );
        window2.addSubplot( v_x,"v x" );
        window2.addSubplot( v_y,"v y" );
        window2.addSubplot( v_z,"v z" );
        GnuplotWindow window3( PLOT_AT_EACH_ITERATION );
        window3.addSubplot( q_w,"rotation w" );
        window3.addSubplot( q_x,"rotation x" );
        window3.addSubplot( q_y,"rotation y" );
        window3.addSubplot( q_z,"rotation z" );
        window3.addSubplot( w_x,"wx" );
        window3.addSubplot( w_y,"wy" );
        window3.addSubplot( w_z,"wz" );
        GnuplotWindow window4( PLOT_AT_EACH_ITERATION );
        window4.addSubplot( w_x,"wx" );
        window4.addSubplot( w_y,"wy" );
        window4.addSubplot( w_z,"wz" );

        GnuplotWindow window5( PLOT_AT_EACH_ITERATION );
        window5.addSubplot( thrust,"Thrust" );
        window5.addSubplot( tau_x,"tau_x" );
        window5.addSubplot( tau_y,"tau_y" );
        window5.addSubplot( tau_z,"tau_z" );


        // Define an algorithm to solve it.
        OptimizationAlgorithm algorithm(ocp);
        algorithm.set( INTEGRATOR_TOLERANCE, 1e-6 );
        algorithm.set( KKT_TOLERANCE, 1e-3 );
        // algorithm << window1;
        // algorithm << window2;
         algorithm << window3;
        // algorithm << window4;
        algorithm << window5;
        algorithm.solve();

    }else{
        // For code generation, we can set some properties.
        // The main reason for a setting is given as comment.
        OCPexport mpc(ocp);

        // mpc.set(HESSIAN_APPROXIMATION,  GAUSS_NEWTON);        // is robust, stable
        // mpc.set(DISCRETIZATION_TYPE,    MULTIPLE_SHOOTING);   // good convergence
        // mpc.set(SPARSE_QP_SOLUTION,     FULL_CONDENSING_N2);  // due to qpOASES
        // mpc.set(INTEGRATOR_TYPE,        INT_IRK_GL4);         // accurate
        // // mpc.set(NUM_INTEGRATOR_STEPS,   N);
        // mpc.set(QP_SOLVER,              QP_QPOASES);          // free, source code
        // mpc.set(HOTSTART_QP,            YES);
        // mpc.set(CG_USE_OPENMP,                    YES);       // paralellization
        // mpc.set(CG_HARDCODE_CONSTRAINT_VALUES,    NO);        // set on runtime
        // mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);       // time-varying costs
        // mpc.set( USE_SINGLE_PRECISION,        YES);           // Single precision

        // // Do not generate tests, makes or matlab-related interfaces.
        // mpc.set( GENERATE_TEST_FILE,          NO);
        // mpc.set( GENERATE_MAKE_FILE,          NO);
        // mpc.set( GENERATE_MATLAB_INTERFACE,   NO);
        // mpc.set( GENERATE_SIMULINK_INTERFACE, NO);
    mpc.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON);
    mpc.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
    mpc.set( SPARSE_QP_SOLUTION, FULL_CONDENSING_N2); //FULL_CONDENsinG_N2
    mpc.set( INTEGRATOR_TYPE, INT_IRK_GL2);
    //mpc.set( NUM_INTEGRATOR_STEPS, N);
    mpc.set( QP_SOLVER, QP_QPOASES);
    mpc.set( HOTSTART_QP, NO);
    mpc.set( LEVENBERG_MARQUARDT, 1e-10);
    mpc.set( LINEAR_ALGEBRA_SOLVER, GAUSS_LU);
    mpc.set( IMPLICIT_INTEGRATOR_NUM_ITS, 2);
    mpc.set( CG_USE_OPENMP, YES);
    mpc.set( CG_HARDCODE_CONSTRAINT_VALUES, NO);
    mpc.set( CG_USE_VARIABLE_WEIGHTING_MATRIX, NO);
        // Finally, export everything.
        if(mpc.exportCode("quadrotor_mpc_codegen") != SUCCESSFUL_RETURN)
        exit( EXIT_FAILURE );
        mpc.printDimensionsQP( );
    }

    return EXIT_SUCCESS;
}
