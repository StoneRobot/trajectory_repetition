#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include <vector>
#include <iostream>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>

#include "hirop_msgs/savePoseData.h"
#include "hirop_msgs/saveDataEnd.h"

#include "hirop_msgs/loadPoseData.h"

using namespace std;

moveit::planning_interface::MoveGroupInterface *group;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "surface_with_demo");
    ros::NodeHandle nh;
    group = new moveit::planning_interface::MoveGroupInterface("arm");
    ros::AsyncSpinner spin(2);
    spin.start();

    ros::ServiceClient add_pose_data = nh.serviceClient<hirop_msgs::savePoseData>("add_pose_data");
    ros::ServiceClient save_pose_data = nh.serviceClient<hirop_msgs::saveDataEnd>("save_data_end");
    ros::ServiceClient load_pose_data = nh.serviceClient<hirop_msgs::loadPoseData>("load_pose_data");

    string end_link;
    nh.param("/end_link", end_link, string("link6"));

    string uri = "trajectory";
    string fileName = "trajectory";


    cout << "1. 加载点位" << endl;
    cout << "2. 记录点位" << endl;
    int flag = 0;
    cin >> flag;
    cin.ignore();
    vector<geometry_msgs::PoseStamped> tra_pose;
    if(flag == 1)
    {
        hirop_msgs::loadPoseData srv;
        srv.request.uri = uri;
        srv.request.name = fileName;
        if(load_pose_data.call(srv))
        {
            tra_pose = srv.response.poses;
        }
        else
        {
            cout << "加载轨迹失败" << endl;
        }
        
    }
    else if(flag == 2)
    {
        int i = 0;
        while(ros::ok())
        {
            cout << "按\"r\"记录, 按\"q\" 取消, 按\"enter\"继续" << endl;
            // int ch = getchar();
            // int ch;
            // ch = cin.get();
            string ch;
            cin >> ch;
            cin.ignore();
            if(ch == "r")
            {
                cout << "正在记录第" << i << "坐标." << endl;
                hirop_msgs::savePoseData srv;
                group->setStartStateToCurrentState();
                srv.request.pose = group->getCurrentPose(end_link);
                tra_pose.push_back(srv.request.pose);
                if(add_pose_data.call(srv))
                {
                    if(srv.response.result == 0)
                    {
                        cout << "记录成功" << endl;
                    }
                }
                else
                {
                    cout << "记录失败" << endl;
                    return -1;
                }
            }
            else if(ch == "q")
            {
                hirop_msgs::saveDataEnd saveEndSrv;
                saveEndSrv.request.uri = uri;
                saveEndSrv.request.name = fileName;
                save_pose_data.call(saveEndSrv);
                cout << "保存成功" << endl;
                break;
            }
            i++;
        }

        // cout << "输入记录点位的数量" << endl;
        // int point_cnt = 0;
        // cin >> point_cnt;
        // cin.ignore();
        // tra_pose.resize(point_cnt);
        // for(int i=0; i<point_cnt; i++)
        // {
        //     cout << "正在记录第" << i << "坐标, 按下/'enter/'键记录" << endl;
        //     cin.ignore();
        //     hirop_msgs::savePoseData srv;
        //     group->setStartStateToCurrentState();
        //     srv.request.pose = group->getCurrentPose(end_link);
        //     tra_pose[i] = srv.request.pose;
        //     if(add_pose_data.call(srv))
        //     {
        //         if(srv.response.result == 0)
        //         {
        //             cout << "记录成功" << endl;
        //         }
        //     }
        //     else
        //     {
        //         cout << "记录失败" << endl;
        //         return -1;
        //     }
        // }
        // hirop_msgs::saveDataEnd saveEndSrv;
        // saveEndSrv.request.uri = uri;
        // saveEndSrv.request.name = fileName;
        // save_pose_data.call(saveEndSrv);
        // cout << "保存成功" << endl;
    }

    // group->setNamedTarget("home");
    // group->move();
    // cout << "moved home" << endl;

    vector<geometry_msgs::Pose> way;
    way.resize(tra_pose.size());
    for(int i=0; i<tra_pose.size(); i++)
    {
        way[i].position = tra_pose[i].pose.position;
        way[i].orientation = tra_pose[i].pose.orientation;
    }
    moveit_msgs::RobotTrajectory tra;
    int cnt = 0;
    cout << "按\"enter\"执行" << endl;
    cin.ignore();
    group->setStartStateToCurrentState();
    while (ros::ok() && cnt < 100)
    {

        if (group->computeCartesianPath(way, 0.005, 0, tra) > 0.8)
        {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = tra;
            group->execute(plan);
            cout << "执行成功" << endl;
            break;
        }
        cnt++;
    }
    return 0;
}