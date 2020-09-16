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

Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(const int d_order,           // the order of derivative
                                                              const Eigen::MatrixXd &Path, // waypoints coordinates (3d)
                                                              const Eigen::MatrixXd &Vel,  // boundary velocity
                                                              const Eigen::MatrixXd &Acc,  // boundary acceleration
                                                              const Eigen::VectorXd &Time) // time allocation in each segment
{
    int p_order = 2 * d_order - 1;      // the order of polynomial 
    int p_num1d = p_order + 1;          // the number of Coefficients in each segment
    int n_seg = Time.size();          // the number of segments

    MatrixXd PolyCoeff = MatrixXd::Zero(n_seg, 3 * p_num1d);     // position(x,y,z), so we need (3 * p_num1d) coefficients

    VectorXd Px(p_num1d * n_seg);     // coefficients in each axis
    VectorXd Py(p_num1d * n_seg);
    VectorXd Pz(p_num1d * n_seg);

    //初始点和终止点的初值p0,v0,a0,j0,pf,vf,af,jf都设为0
    MatrixXd startPointState(d_order, 3);
    MatrixXd endPointState(d_order, 3);
    startPointState.row(0) = Path.row(0);
    startPointState.row(1) = Vel.row(0);
    startPointState.row(2) = Acc.row(0);
    endPointState.row(0) = Path.row((Path.rows()-1));
    endPointState.row(1) = Vel.row(1);
    endPointState.row(2) = Acc.row(1);
    if(d_order == 4)//use minimum snap
    {
        startPointState.row(3) = VectorXd::Zero(3); //j0 = 0
        endPointState.row(3) = VectorXd::Zero(3);//jf = 0
    }
    // cout << "Path = " << endl;
    // cout << Path << endl;
    // cout << "startPointState = " << endl;
    // cout << startPointState << endl;
    // cout << "endPointState = " << endl;
    // cout << endPointState << endl;

    _Q = MatrixXd::Zero(p_num1d * n_seg, p_num1d * n_seg);
    _M = MatrixXd::Zero(p_num1d * n_seg, p_num1d * n_seg);
    _Ct = MatrixXd::Zero(2 * d_order * n_seg, d_order * (n_seg + 1));

    //计算每段的Q矩阵
    for(int seg_index = 0; seg_index < n_seg; seg_index++)
    {
        /*
        matrix.block(i,j,p,q)
        表示返回从矩阵的(i, j)开始，每行取p个元素，每列取q个元素所组成的临时新矩阵对象，原矩阵的元素不变
        */
        //计算Q_i矩阵
        _Q.block(seg_index*p_num1d, seg_index*p_num1d, p_num1d, p_num1d) = getQ(p_num1d, d_order, Time, seg_index);
        //计算M_i矩阵
        _M.block(seg_index*p_num1d, seg_index*p_num1d, p_num1d, p_num1d) = getM(p_num1d, d_order, Time, seg_index);
    }
    //计算Ct矩阵
    _Ct = getCt(n_seg, d_order);

    //分别计算x,y,z方向上的closedform                      
    Px = closedFormCalCoeff1D(_Q, _M, _Ct, Path.col(0), startPointState.col(0), endPointState.col(0), n_seg, d_order);
    Py = closedFormCalCoeff1D(_Q, _M, _Ct, Path.col(1), startPointState.col(1), endPointState.col(1), n_seg, d_order);
    Pz = closedFormCalCoeff1D(_Q, _M, _Ct, Path.col(2), startPointState.col(2), endPointState.col(2), n_seg, d_order);
    // cout << " Px = " << endl;
    // cout << Px << endl;
    // cout << " Py = " << endl;
    // cout << Py << endl;
    // cout << " Pz = " << endl;
    // cout << Pz << endl;

    for(int i = 0; i < n_seg; i++)
    {
        PolyCoeff.row(i).segment(0, p_num1d) = Px.segment(p_num1d*i, p_num1d);
        PolyCoeff.row(i).segment(p_num1d, p_num1d) = Py.segment(p_num1d*i, p_num1d);
        PolyCoeff.row(i).segment(2*p_num1d, p_num1d) = Pz.segment(p_num1d*i, p_num1d);
    }
     //cout << " PolyCoeff = " << endl;
     //cout << PolyCoeff << endl;
    
    return PolyCoeff;
}
                                                //p_num1d:多项式系数   d_order:需要求的导数  Time:时间
Eigen::MatrixXd TrajectoryGeneratorWaypoint::getQ(const int p_num1d, const int d_order, const Eigen::VectorXd &Time, const int seg_index)
{
    //cout << "p_num1d = " << p_num1d << endl;
    // calculate Matrix Q_k of the seg_index-th segment
    MatrixXd Q_k = MatrixXd::Zero(p_num1d, p_num1d);
    VectorXd time = VectorXd::Zero(p_num1d);
    for(int i = 0; i < p_num1d; i++)
    {
        time(i) = pow(Time(seg_index),i);
    }
    if(p_num1d == 6)        // minimum jerk
    {
        Q_k << 0,     0     ,     0     ,      0     ,       0     ,       0     ,
               0,     0     ,     0     ,      0     ,       0     ,       0     ,
               0,     0     ,     0     ,      0     ,       0     ,       0     ,
               0,     0     ,     0     ,  36*time(1),   72*time(2),  120*time(3),
               0,     0     ,     0     ,  72*time(2),  192*time(3),  360*time(4),
               0,     0     ,     0     , 120*time(3),  360*time(4),  720*time(5);
    }
    else if(p_num1d == 8)   // minimum snap
    {
        Q_k << 0,     0    ,      0     ,      0     ,       0     ,      0      ,       0       ,        0      ,
               0,     0    ,      0     ,      0     ,       0     ,      0      ,       0       ,        0      ,
               0,     0    ,      0     ,      0     ,       0     ,      0      ,       0       ,        0      ,
               0,     0    ,      0     ,      0     ,       0     ,      0      ,       0       ,        0      ,
               0,     0    ,      0     ,      0     ,  576*time(1),  1440*time(2),  2880*time(3),   5040*time(4),
               0,     0    ,      0     ,      0     , 1440*time(2),  4800*time(3), 10800*time(4),  20160*time(5),
               0,     0    ,      0     ,      0     , 2880*time(3), 10800*time(4), 25920*time(5),  50400*time(6),
               0,     0    ,      0     ,      0     , 5040*time(4), 20160*time(5), 50400*time(6), 100800*time(7);
    }
    // cout << " Q_k = " << endl;
    // cout <<  Q_k << endl;

    return Q_k;
}

Eigen::MatrixXd TrajectoryGeneratorWaypoint::getM(const int p_num1d, const int d_order, const Eigen::VectorXd &Time, const int seg_index)
{
    MatrixXd M_k = MatrixXd::Zero(p_num1d, p_num1d);
    VectorXd time = VectorXd::Zero(p_num1d);
    for(int i = 0; i < p_num1d; i++)
    {
        time(i) = pow(Time(seg_index),i);
    }
    if(p_num1d == 6)        // minimum jerk
    {
        M_k << 1,     0   ,     0     ,     0     ,      0     ,      0     ,
               0,     1   ,     0     ,     0     ,      0     ,      0     ,
               0,     0   ,     2     ,     0     ,      0     ,      0     ,
               1,  time(1),    time(2),    time(3),     time(4),     time(5),
               0,     1   ,  2*time(1),  3*time(2),   4*time(3),   5*time(4),
               0,     0   ,     2     ,  6*time(1),  12*time(2),  20*time(3);
    }
    else if(p_num1d == 8)   // minimum snap
    {
        M_k << 1,     0   ,     0     ,     0     ,      0     ,      0     ,      0     ,      0     ,
               0,     1   ,     0     ,     0     ,      0     ,      0     ,      0     ,      0     ,
               0,     0   ,     2     ,     0     ,      0     ,      0     ,      0     ,      0     ,
               0,     0   ,     0     ,     6     ,      0     ,      0     ,      0     ,      0     ,
               1,  time(1),    time(2),    time(3),     time(4),     time(5),     time(6),     time(7),
               0,     1   ,  2*time(1),  3*time(2),   4*time(3),   5*time(4),   6*time(5),   7*time(6),
               0,     0   ,     2     ,  6*time(1),  12*time(2),  20*time(3),  30*time(4),  42*time(5),
               0,     0   ,     0     ,     6     ,  24*time(1),  60*time(2), 120*time(3), 210*time(4);
    }
    // cout << "M_k = " << endl;
    // cout << M_k << endl;

    return M_k;
}

Eigen::MatrixXd TrajectoryGeneratorWaypoint::getCt(const int n_seg, const int d_order)
{
    int d_num = 2 * d_order * n_seg;
    int dF_dP_num = (n_seg + 1) * d_order;

    //Ct矩阵的行数：每个路标点的各个状态量(每个路标点的导数) 列数：dF和dP的数量
    Eigen::MatrixXd Ct = MatrixXd::Zero(d_num, dF_dP_num);
    
    Eigen::MatrixXd C_temp = MatrixXd::Zero(2 * d_order, dF_dP_num);

    // stratPositionState,即p0,v0,a0,j0对应的Ct
    Ct.block(0, 0, d_order, d_order) = MatrixXd::Identity(d_order, d_order);

    if(d_order == 3)//use minimum jerk
    {
        for(int k = 0; k < n_seg - 1; k++)
        {
            C_temp(0, d_order + k) = 1;
            C_temp(1, d_order + d_order + (n_seg - 1) + 2*k) = 1;
            C_temp(2, d_order + d_order + (n_seg - 1) + 2*k + 1) = 1;

            C_temp(3, d_order + k) = 1;
            C_temp(4, d_order + d_order + (n_seg - 1) + 2*k) = 1;
            C_temp(5, d_order + d_order + (n_seg - 1) + 2*k + 1) = 1;

            Ct.block(d_order + 2 * d_order * k, 0, 2 * d_order, dF_dP_num) = C_temp;
            C_temp = MatrixXd::Zero(2 * d_order, dF_dP_num);
        }
    }

    if(d_order == 4)//use minimum snap
    {
        for(int k = 0; k < n_seg - 1; k++)
        {
            C_temp(0, d_order + k) = 1;
            C_temp(1, d_order + d_order + (n_seg - 1) + 3*k) = 1;
            C_temp(2, d_order + d_order + (n_seg - 1) + 3*k + 1) = 1;
            C_temp(3, d_order + d_order + (n_seg - 1) + 3*k + 2) = 1;

            C_temp(4, d_order + k) = 1;
            C_temp(5, d_order + d_order + (n_seg - 1) + 3*k) = 1;
            C_temp(6, d_order + d_order + (n_seg - 1) + 3*k + 1) = 1;
            C_temp(7, d_order + d_order + (n_seg - 1) + 3*k + 2) = 1;

            Ct.block(d_order + 2 * d_order * k, 0, 2 * d_order, dF_dP_num) = C_temp;
            //清0
            C_temp = MatrixXd::Zero(2 * d_order, dF_dP_num);
            //cout << "看看是否清0了C_temp = " << C_temp << endl;
        }
    }
    Ct.block((n_seg - 1) * 2 * d_order + d_order , d_order + (n_seg - 1) , d_order, d_order) = MatrixXd::Identity(d_order, d_order);

    // cout << "Ct = " << endl;
    // cout << Ct << endl;
    return Ct;
}

Eigen::VectorXd TrajectoryGeneratorWaypoint::closedFormCalCoeff1D(const Eigen::MatrixXd &Q,
                                                                  const Eigen::MatrixXd &M,
                                                                  const Eigen::MatrixXd &Ct,
                                                                  const Eigen::VectorXd &WayPoints1D,
                                                                  const Eigen::VectorXd &startPointState1D,
                                                                  const Eigen::VectorXd &endPointState1D,
                                                                  const int n_seg,
                                                                  const int d_order)
{
    int dF_dP_num = d_order * (n_seg + 1);

    int df_num = 2 * d_order + (n_seg - 1);
    int dp_num = (d_order - 1) * (n_seg - 1);

    Eigen::MatrixXd C = Ct.transpose();//转置
    Eigen::MatrixXd M_inv = M.inverse();//求逆
    Eigen::MatrixXd M_inv_tran = M_inv.transpose();

    Eigen::MatrixXd R = C * M_inv_tran * Q * M_inv * Ct;
    Eigen::MatrixXd R_pp = R.block(df_num, df_num, dp_num, dp_num);
    Eigen::MatrixXd R_fp = R.block(0, df_num, df_num, dp_num);



    //计算dF
    Eigen::VectorXd dF(df_num);
    dF.head(d_order) = startPointState1D;//start state:p0,v0,a0,j0
    dF.segment(d_order, (n_seg - 1)) = WayPoints1D.segment(1,WayPoints1D.rows()-2);//中间点的p
    dF.tail(d_order) = endPointState1D;//end state:pf,vf,af,jf
    // cout << "dF = " << endl;
    // cout << dF << endl;
    
    Eigen::VectorXd dP = -R_pp.inverse() * R_fp.transpose() * dF;//closed form 解法中最优的dP



    Eigen::VectorXd dF_and_dP(dF_dP_num);
    dF_and_dP << dF, dP;

    Eigen::VectorXd PolyCoeff1D = M_inv * Ct * dF_and_dP;//一个方向上的系数

    return PolyCoeff1D;
}