#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
#include <random>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <sensor_msgs/Joy.h>
#include <algorithm>

// Useful customized headers
#include "trajectory_generator_waypoint.h"

using namespace std;
using namespace Eigen;

// Param from launch file
double _vis_traj_width;
double _Vel, _Acc;
int _dev_order, _min_order;

// ros related
ros::Subscriber _way_pts_sub;//接收waypoint_generator中的点信息
ros::Publisher _wp_traj_vis_pub, _wp_path_vis_pub;

//发布jerk的
// ros::Publisher _wp_traj_vis_pub_jerk, _wp_path_vis_pub_jerk;

// for planning
int _poly_num1D;
MatrixXd _polyCoeff;//轨迹系数
VectorXd _polyTime;//时间
Vector3d _startPos  = Vector3d::Zero();
Vector3d _startVel  = Vector3d::Zero();
Vector3d _startAcc  = Vector3d::Zero();
Vector3d _endVel    = Vector3d::Zero();
Vector3d _endAcc    = Vector3d::Zero();

// int _poly_num1D_jerk;


// declare
void visWayPointTraj(MatrixXd polyCoeff, VectorXd time);
void visWayPointPath(MatrixXd path);
Vector3d getPosPoly(MatrixXd polyCoeff, int k, double t);
VectorXd timeAllocation(MatrixXd Path);
void trajGeneration(Eigen::MatrixXd path);
void rcvWaypointsCallBack(const nav_msgs::Path &wp);

// void visWayPointTraj_jerk(MatrixXd polyCoeff, VectorXd time);
// void visWayPointPath_jerk(MatrixXd path);
// Vector3d getPosPoly_jerk(MatrixXd polyCoeff, int k, double t);

//Get the path points 
void rcvWaypointsCallBack(const nav_msgs::Path & wp)
{   
    cout << "enter rcvWaypointsCallBack()" << endl;
    vector<Vector3d> wp_list;
    wp_list.clear();

    for (int k = 0; k < (int)wp.poses.size(); k++)
    {
        Vector3d pt( wp.poses[k].pose.position.x, wp.poses[k].pose.position.y, wp.poses[k].pose.position.z);
        wp_list.push_back(pt);

        // cout << "pt = " << endl;
        // cout << pt << endl;

        ROS_INFO("waypoint%d: (%f, %f, %f)", k+1, pt(0), pt(1), pt(2));
    }

    MatrixXd waypoints(wp_list.size() + 1, 3);  //add the original point
    waypoints.row(0) = _startPos;

    for(int k = 0; k < (int)wp_list.size(); k++)
        waypoints.row(k+1) = wp_list[k];//vector<Vector3d>与matrixXd之间的转化

    // cout << "MatrixXd waypoints = " << endl;
    // cout << waypoints << endl;

    //Trajectory generation: use minimum jerk/snap trajectory generation method
    //waypoints is the result of path planning (Manual in this project)
    trajGeneration(waypoints);
}

void trajGeneration(Eigen::MatrixXd path)
{
    ros::Time time_start = ros::Time::now();

    TrajectoryGeneratorWaypoint trajectoryGeneratorWaypoint;
    
    MatrixXd vel  = MatrixXd::Zero(2, 3); 
    MatrixXd acc  = MatrixXd::Zero(2, 3);
    vel.row(0)  = _startVel;
    vel.row(1)  = _endVel;
    acc.row(0)  = _startAcc;
    acc.row(1)  = _endAcc;

    // use "trapezoidal velocity" time allocation
    _polyTime  = timeAllocation(path);

    // generate a minimum-jerk/snap piecewise monomial polynomial-based trajectory
    _polyCoeff = trajectoryGeneratorWaypoint.PolyQPGeneration(_dev_order, path, vel, acc, _polyTime);

    // cout << "_polyCoeff = " << endl;
    // cout << _polyCoeff << endl;
    ros::Time time_end = ros::Time::now();
    ROS_WARN("Time consumed in trajectory generation is %f ms", (time_end - time_start).toSec() * 1000.0);

    visWayPointPath(path);    // visulize path(这是可视化点信息)
    visWayPointTraj( _polyCoeff, _polyTime);    // visulize trajectory(这是可视化轨迹)


    //minimum jerk 看一下minimum snap和minimum jerk区别
    //cout << "_min_order = " << _min_order << endl;
    // Eigen::MatrixXd _polyCoeff_jerk;
    // TrajectoryGeneratorWaypoint trajectoryGeneratorWaypoint_jerk;
    // _polyCoeff_jerk = trajectoryGeneratorWaypoint_jerk.PolyQPGeneration(_min_order, path, vel, acc, _polyTime);

    // cout << "_polyCoeff_jerk = " << endl;
    // cout << _polyCoeff_jerk << endl;
    // ROS_WARN("Time consumed use minimum jerk trajectory generation is %f ms", (time_end - time_start).toSec() * 1000.0);
    // visWayPointPath_jerk(path);    // visulize path(这是可视化点信息)
    // visWayPointTraj_jerk( _polyCoeff_jerk, _polyTime);    // visulize trajectory(这是可视化轨迹)

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "traj_node");
    ros::NodeHandle nh("~");

    nh.param("planning/vel", _Vel, 1.0);
    nh.param("planning/acc", _Acc, 1.0);
    nh.param("planning/dev_order", _dev_order, 3);  // the order of derivative, _dev_order = 3->minimum jerk, _dev_order = 4->minimum snap
    nh.param("planning/min_order", _min_order, 3);
    nh.param("vis/vis_traj_width", _vis_traj_width, 0.15);

    //_poly_numID is the maximum order of polynomial
    _poly_num1D = 2 * _dev_order;

    //_poly_num1D_jerk = 2 * _min_order;

    _way_pts_sub     = nh.subscribe( "waypoints", 1, rcvWaypointsCallBack );

    _wp_traj_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_trajectory", 1);
    _wp_path_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_waypoint_path", 1);

    //发布jerk
    // _wp_traj_vis_pub_jerk = nh.advertise<visualization_msgs::Marker>("vis_trajectory_jerk", 1);
    // _wp_path_vis_pub_jerk = nh.advertise<visualization_msgs::Marker>("vis_waypoint_path_jerk", 1);

    ros::Rate rate(100);
    bool status = ros::ok();
    while(status) 
    {
        ros::spinOnce();
        status = ros::ok();
        rate.sleep();
    }
    return 0;
}

void visWayPointTraj( MatrixXd polyCoeff, VectorXd time)
{
    visualization_msgs::Marker _traj_vis;

    _traj_vis.header.stamp       = ros::Time::now();
    _traj_vis.header.frame_id    = "world";//原来是"/map"

    _traj_vis.ns = "traj_node/trajectory_waypoints";
    _traj_vis.id = 0;
    _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    _traj_vis.action = visualization_msgs::Marker::ADD;
    _traj_vis.scale.x = _vis_traj_width;
    _traj_vis.scale.y = _vis_traj_width;
    _traj_vis.scale.z = _vis_traj_width;
    _traj_vis.pose.orientation.x = 0.0;
    _traj_vis.pose.orientation.y = 0.0;
    _traj_vis.pose.orientation.z = 0.0;
    _traj_vis.pose.orientation.w = 1.0;

    _traj_vis.color.a = 1.0;
    _traj_vis.color.r = 1.0;
    _traj_vis.color.g = 0.0;
    _traj_vis.color.b = 0.0;

    double traj_len = 0.0;
    int count = 0;
    Vector3d cur, pre;
    cur.setZero();
    pre.setZero();

    _traj_vis.points.clear();
    Vector3d pos;
    geometry_msgs::Point pt;


    for(int i = 0; i < time.size(); i++ )   // go through each segment
    {   
        for (double t = 0.0; t < time(i); t += 0.01, count += 1)
        {
          pos = getPosPoly(polyCoeff, i, t);
          cur(0) = pt.x = pos(0);
          cur(1) = pt.y = pos(1);
          cur(2) = pt.z = pos(2);
          _traj_vis.points.push_back(pt);

          if (count) traj_len += (pre - cur).norm();
          pre = cur;
        }
    }
    ROS_WARN("Trajectory length is %f m", traj_len);
    _wp_traj_vis_pub.publish(_traj_vis);
}

void visWayPointPath(MatrixXd path)
{
    visualization_msgs::Marker points, line_list;
    int id = 0;
    points.header.frame_id    = line_list.header.frame_id    = "world";//原来是"/map"
    points.header.stamp       = line_list.header.stamp       = ros::Time::now();
    points.ns                 = line_list.ns                 = "wp_point";
    points.action             = line_list.action             = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_list.pose.orientation.w = 1.0;
    points.pose.orientation.x = line_list.pose.orientation.x = 0.0;
    points.pose.orientation.y = line_list.pose.orientation.y = 0.0;
    points.pose.orientation.z = line_list.pose.orientation.z = 0.0;

    points.id    = id;
    line_list.id = id;

    points.type    = visualization_msgs::Marker::SPHERE_LIST;
    line_list.type = visualization_msgs::Marker::LINE_STRIP;

    points.scale.x = 0.3;
    points.scale.y = 0.3;
    points.scale.z = 0.3;
    points.color.a = 1.0;
    points.color.r = 0.0;
    points.color.g = 0.0;
    points.color.b = 0.0;

    line_list.scale.x = 0.15;
    line_list.scale.y = 0.15;
    line_list.scale.z = 0.15;
    line_list.color.a = 1.0;

    
    line_list.color.r = 0.0;
    line_list.color.g = 1.0;
    line_list.color.b = 0.0;
    
    line_list.points.clear();

    for(int i = 0; i < path.rows(); i++){
      geometry_msgs::Point p;
      p.x = path(i, 0);
      p.y = path(i, 1); 
      p.z = path(i, 2); 

      points.points.push_back(p);
      line_list.points.push_back(p);
    }

    _wp_path_vis_pub.publish(points);
    _wp_path_vis_pub.publish(line_list);
}

Vector3d getPosPoly( MatrixXd polyCoeff, int k, double t )
{
    Vector3d ret;

    for ( int dim = 0; dim < 3; dim++ )
    {
        VectorXd coeff = (polyCoeff.row(k)).segment( dim * _poly_num1D, _poly_num1D );
        VectorXd time  = VectorXd::Zero( _poly_num1D );
        
        for(int j = 0; j < _poly_num1D; j ++)
          if(j==0)
              time(j) = 1.0;
          else
              time(j) = pow(t, j);

        ret(dim) = coeff.dot(time);
        //cout << "dim:" << dim << " coeff:" << coeff << endl;
    }
    // cout << "ret = " << endl;
    // cout << ret << endl;

    return ret;
}

//时间分配函数  可以给每段时间都分配为1s ，也可以按照每段的距离分配时间
VectorXd timeAllocation( MatrixXd Path)
{ 
    VectorXd time(Path.rows() - 1);
    VectorXd dist = VectorXd::Zero(Path.rows() - 1);
    for(int i=0; i < Path.rows()-1; i++)
    {
        dist(i) = sqrt(pow(Path(i+1, 0) - Path(i, 0), 2) 
                     + pow(Path(i+1, 1) - Path(i, 1), 2) 
                     + pow(Path(i+1, 2) - Path(i, 2), 2));
        //梯形面积dist = (上底+下底)*h/2  t2:上底   （2*t1+t2):下底     _Vel:高
        //cout << "_Vel = " << _Vel << "_Acc = " << _Acc << endl;
        double t1 = _Vel / _Acc;
        double t2 = 0;

        t2 = dist(i)/_Vel - t1;
        time(i) = 2*t1 + t2;       
    }
    return time;
}

// //可视化jerk -----------------------加的---------------------------
// void visWayPointTraj_jerk( MatrixXd polyCoeff, VectorXd time)
// {
//     visualization_msgs::Marker _traj_vis;

//     _traj_vis.header.stamp       = ros::Time::now();
//     _traj_vis.header.frame_id    = "/map";

//     _traj_vis.ns = "traj_node/trajectory_waypoints_jerk";
//     _traj_vis.id = 10; //0
//     _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
//     _traj_vis.action = visualization_msgs::Marker::ADD;
//     _traj_vis.scale.x = _vis_traj_width;
//     _traj_vis.scale.y = _vis_traj_width;
//     _traj_vis.scale.z = _vis_traj_width;
//     _traj_vis.pose.orientation.x = 0.0;
//     _traj_vis.pose.orientation.y = 0.0;
//     _traj_vis.pose.orientation.z = 0.0;
//     _traj_vis.pose.orientation.w = 1.0;

//     _traj_vis.color.a = 1.0;
//     _traj_vis.color.r = 0.0;
//     _traj_vis.color.g = 1.0;
//     _traj_vis.color.b = 1.0;

//     double traj_len = 0.0;
//     int count = 0;
//     Vector3d cur, pre;
//     cur.setZero();
//     pre.setZero();

//     _traj_vis.points.clear();
//     Vector3d pos;
//     geometry_msgs::Point pt;

//     for(int i = 0; i < time.size(); i++ )   // go through each segment
//     {   
//         for (double t = 0.0; t < time(i); t += 0.01, count += 1)
//         {
//           pos = getPosPoly_jerk(polyCoeff, i, t);
//           cur(0) = pt.x = pos(0);
//           cur(1) = pt.y = pos(1);
//           cur(2) = pt.z = pos(2);
//           _traj_vis.points.push_back(pt);

//           if (count) traj_len += (pre - cur).norm();
//           pre = cur;
//         }
//     }
//     ROS_WARN("Trajectory length is %f m", traj_len);
//     _wp_traj_vis_pub_jerk.publish(_traj_vis);
// }

// void visWayPointPath_jerk(MatrixXd path)
// {
//     //cout << "enter visWayPointPath_jerk()" << endl;
//     visualization_msgs::Marker points, line_list;
//     int id = 5; //0
//     points.header.frame_id    = line_list.header.frame_id    = "/map";
//     points.header.stamp       = line_list.header.stamp       = ros::Time::now();
//     points.ns                 = line_list.ns                 = "wp_point";
//     points.action             = line_list.action             = visualization_msgs::Marker::ADD;
//     points.pose.orientation.w = line_list.pose.orientation.w = 1.0;
//     points.pose.orientation.x = line_list.pose.orientation.x = 0.0;
//     points.pose.orientation.y = line_list.pose.orientation.y = 0.0;
//     points.pose.orientation.z = line_list.pose.orientation.z = 0.0;

//     points.id    = id;
//     line_list.id = id;

//     points.type    = visualization_msgs::Marker::SPHERE_LIST;
//     line_list.type = visualization_msgs::Marker::LINE_STRIP;

//     points.scale.x = 0.3;
//     points.scale.y = 0.3;
//     points.scale.z = 0.3;
//     points.color.a = 1.0;
//     points.color.r = 0.0;
//     points.color.g = 0.0;
//     points.color.b = 0.0;

//     line_list.scale.x = 0.15;
//     line_list.scale.y = 0.15;
//     line_list.scale.z = 0.15;
//     line_list.color.a = 1.0;

    
//     line_list.color.r = 0.0;
//     line_list.color.g = 1.0;
//     line_list.color.b = 0.0;
    
//     line_list.points.clear();

//     for(int i = 0; i < path.rows(); i++){
//       geometry_msgs::Point p;
//       p.x = path(i, 0);
//       p.y = path(i, 1); 
//       p.z = path(i, 2); 

//       points.points.push_back(p);
//       line_list.points.push_back(p);
//     }

//     _wp_path_vis_pub_jerk.publish(points);
//     _wp_path_vis_pub_jerk.publish(line_list);
// }

// Vector3d getPosPoly_jerk( MatrixXd polyCoeff, int k, double t )
// {
//     Vector3d ret;
//     _poly_num1D_jerk = 6;
//     for ( int dim = 0; dim < 3; dim++ )
//     {
//         VectorXd coeff = (polyCoeff.row(k)).segment( dim * _poly_num1D_jerk, _poly_num1D_jerk );
//         VectorXd time  = VectorXd::Zero( _poly_num1D_jerk );
        
//         for(int j = 0; j < _poly_num1D_jerk; j ++)
//           if(j==0)
//               time(j) = 1.0;
//           else
//               time(j) = pow(t, j);

//         ret(dim) = coeff.dot(time);
//         //cout << "dim:" << dim << " coeff:" << coeff << endl;
//     }
//     // cout << "ret = " << endl;
//     // cout << ret << endl;
//     return ret;
// }