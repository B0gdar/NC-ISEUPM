/*****************************************************************************
 * PROJECT: ISE_INGENIA1516
 * MODULE: ise_localnav: Implements local navigation control.		     *
 *
 * (c) Copyright 2016 Miguel Cordero Sánchez and Francisco Butragueño Martín.*
 * All rights reserved.							     *
 * based on 2D Work by Reid Simmons, Joaquín López, Javier Albertos and      *
 * Miguel Cordero.                                                           *
 *                                                                           *
 * FILE: test_ise_localnav.cpp                                               *
 *                                                                           *
 * ABSTRACT: "main function": Calls method according to the configuration.   *
 * 
 *
 *          Subscribes to sensor_msgs/PointCloud2 and localNav/moveDir msgs.
 *          Publishes 
 *
 *****************************************************************************/

#include <string>
#include <vector>
#include <algorithm>
#include "nav_msgs/Odometry.h"
#include <stdio.h>
#include <std_msgs/String.h>
#include <sstream>
//#include <bebop_msgs::Ardrone3PilotingStateFlyingStateChanged>

#include <ros/ros.h>
//#include <nodelet/loader.h>
//static nav_msgs::Odometry odom_ref
static int bebop_fst;
ros::Publisher pub_cmdvel;

class ise_nLNMain
{
  private:
  double walk_vel, run_vel, yaw_rate, yaw_rate_run;//,z_ref;
  int movestate,cmd_eval_z;
	//walk_vel=0.2;
	//z_ref=1;
  
  public:
  
  void init()
  { 
    double absolute_angle;

  //  std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
    
    //ros::NodeHandle n_/*private("~")*/;
//    n_/*private*/.param("walk_vel", walk_vel, walk_vel);
//    n_/*private*/.param("run_vel", run_vel, run_vel);
//    n_/*private*/.param("yaw_rate", yaw_rate, yaw_rate);
//    n_/*private*/.param("yaw_rate_run", yaw_rate_run, yaw_rate_run);
//    n_/*private*/.param("movestate", movestate, movestate);
//    n_/*private*/.param("cmd_eval_z", cmd_eval_z, cmd_eval_z);
  }
  
  ~ise_nLNMain()  {}
  
};
//TODO
/*void std_position(const nav_msgs::Odometry *a){
	nav_msgs::Odometry * b = a; 
	std::cout<<"ODOM POSITION"<<std::endl;
	std::cout<<b->pose->pose->position<<std::endl;
	std::cout<<"ODOM X"<<std::endl;
std::cout<<b->pose->pose->position->x<<std::endl;
std::cout<<"------------------------------"<<std::endl;
}*/

/*void fst_callback(const bebop_msgs::Ardrone3PilotingStateFlyingStateChanged fst){
	bebop_fst=fst.state;
}*/

void odom_callback(const nav_msgs::Odometry odom_ref){
	std::cout<<"------------------------------"<<std::endl;
	std::cout<<"ODOM UPDATE"<<std::endl;
	//std::cout<<odom_ref<<std::endl;
	std::cout<<"ODOM POSITION"<<std::endl;
	std::cout<<odom_ref.pose.pose.position<<std::endl;
	//TODO
	//std_position(odom_ref);	
//	std::cout<<"FLYING STATE"<<std::endl;
//	std::cout<<bebop_fst<<std::endl;
	float e_z, z_ref;
	z_ref=1;
	geometry_msgs::Twist twistter;
	e_z=odom_ref.pose.pose.position.z-z_ref;
	if(e_z>0.1){
		twistter.linear.z=-0.2;
	}else if(e_z<0.1){
		twistter.linear.z=0.5;
	}else{
		twistter.linear.z=0;
	}
	std::cout<<"TWISTTER"<<std::endl;
	std::cout<<"linear should be: "<<twistter<<std::endl;
	pub_cmdvel.publish(twistter);
}

void cmdvel_callback(const geometry_msgs::Twist cmd_ref){
	std::cout<<"------------------------------"<<std::endl;
	std::cout<<"CMD UPDATE"<<std::endl;
	//std::cout<<odom_ref<<std::endl;
	std::cout<<"CMD_VEL"<<std::endl;
	std::cout<<cmd_ref.linear<<std::endl;
	//TODO
	//std_position(odom_ref);
	
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "error_test");
	
	//set_initial_odometry();
	ros::NodeHandle n;
	ise_nLNMain lnMain;
        lnMain.init();
	//ros::Subscriber sub_fst = n.subscribe("states/ARDrone3/PilotingState/FlyingStateChanged",1,fst_callback);
	ros::Subscriber sub_odom = n.subscribe("/bebop/odom",1,odom_callback);

	ros::Subscriber sub_cmdvel = n.subscribe("/bebop/cmd_vel",1,cmdvel_callback);

ros::Publisher pub_chatter = n.advertise<std_msgs::String>("/chatter", 1);
pub_cmdvel = n.advertise<geometry_msgs::Twist>("/bebop/cmd_vel",1);

std_msgs::String msg;

std::stringstream ss;

ss<<"Yokse no soy 100tifko";
msg.data =ss.str();
pub_chatter.publish(msg);
	ros::spin();
	return 0;
}
