#define _USE_MATH_DEFINES
#include <math.h>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure_test/dyn_reconfig_testConfig.h>
#include <dynamic_reconfigure_test/euler.h>
#include <iostream>
#include <cmath>
#include <std_msgs/Float64MultiArray.h>
//make new file
#include <fstream>
#include <string>
#include <stdio.h>
// #include <Eigen/Dense>

using namespace std;
// using namespace Eigen;

// ros::NodeHandle n;
// ros::Publisher extrinsic_matrix_publisher = n.advertise<std_msgs::Float64MultiArray>("extrinsic_matrix", 1000);//Publish topic wheel_speed_publisher
std_msgs::Float64MultiArray array_msg;
bool pub_flag;
int n = 0;

void callback(dynamic_reconfigure_test::dyn_reconfig_testConfig &config, uint32_t level) {
    // ros::NodeHandle n;
    // ros::Publisher extrinsic_matrix_publisher = n.advertise<std_msgs::Float64MultiArray>("extrinsic_matrix", 1000);//Publish topic wheel_speed_publisher
    // ROS_INFO("x: %d", config.x);
    // ROS_INFO("y: %d", config.y);
    // ROS_INFO("z: %d", config.z);
    // ROS_INFO("roll: %d", config.roll);
    // ROS_INFO("pitch: %d", config.pitch);
    // ROS_INFO("yaw: %d", config.yaw);
    // array_msg.data.resize(16);
    

    const double PI = 3.1415926;


    double roll = static_cast<double>((config.roll)*PI/180); //z
    double pitch = static_cast<double>((config.pitch)*PI/180); //y
    double yaw = static_cast<double>((config.yaw)*PI/180); //x


    double cy = cos(yaw/2);
    double sy = sin(yaw/2);
    double cp = cos(pitch/2);
    double sp = sin(pitch/2);
    double cr = cos(roll/2);
    double sr = sin(roll/2);


    //XYZ before
    // double w = cr * cp * cy - sr * sp * sy;
    // double x = cr * cp * sy + sr * sp * cy;
	// double y = cr * sp * cy - sr * cp * sy;
	// double z = sr * cp * cy + cr * sp * sy;


    //After
    double w = cr * cp * cy - sr * sp * sy;
    double x = sr * cp * cy + cr * sp * sy;
	double y = cr * sp * cy - sr * cp * sy;
	double z = cr * cp * sy + sr * sp * cy;

    // 
    // ROS_INFO("x: %f", x);
    // ROS_INFO("y: %f", y);
    // ROS_INFO("z: %f", z);
    // ROS_INFO("w: %f", w);

    // ROS_INFO("-----------");



    double Rx[3][3] = {{1,0,0},                        //Roll           
                   {0,cos(roll),-sin(roll)},
                   {0,sin(roll),cos(roll)}};
         
    double Ry[3][3] = {{cos(pitch),0,sin(pitch)},      //Pitch           
                   {0,1,0},
                   {-sin(pitch),0,cos(pitch)}};

    double Rz[3][3] = {{cos(yaw),-sin(yaw),0},         //Yaw
                   {sin(yaw),cos(yaw),0},
                   {0,0,1}};        
                   
    double Rzy[3][3] = {0};

    double Rzyx[3][3] = {0};


    //3X3 행렬곱 
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            for(int k=0;k<3;k++){
                Rzy[i][j]+=Rz[i][k]*Ry[k][j];
            }
        }
    }

    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            for(int k=0;k<3;k++){
                Rzyx[i][j]+=Rzy[i][k]*Rx[k][j];
            }
        }
    }

    double Re[3][4] = {{Rzyx[0][0],Rzyx[0][1],Rzyx[0][2],config.x},        
                   {Rzyx[1][0],Rzyx[1][1],Rzyx[1][2],config.y},
                   {Rzyx[2][0],Rzyx[2][1],Rzyx[2][2],config.z}}; 

    
    double Ri[3][3] = {{config.fx,0,config.cx},      
                   {0,config.fy,config.cy},
                   {0,0,1}};

    double Rc[3][4] = {0};


    for(int i=0;i<3;i++){
        for(int j=0;j<4;j++){
            for(int k=0;k<3;k++){
                Rc[i][j]+=Ri[i][k]*Re[k][j];
            }
        }
    }

    

    // double  = R[1][1];

    // ROS_INFO("\n Rx \n %f %f %f \n %f %f %f \n %f %f %f", Rx[0][0],Rx[0][1],Rx[0][2],Rx[1][0],Rx[1][1],Rx[1][2],Rx[2][0],Rx[2][1],Rx[2][2]);

    // ROS_INFO("\n Ry \n %f %f %f \n %f %f %f \n %f %f %f", Ry[0][0],Ry[0][1],Ry[0][2],Ry[1][0],Ry[1][1],Ry[1][2],Ry[2][0],Ry[2][1],Ry[2][2]);

    // ROS_INFO("\n Rz \n %f %f %f \n %f %f %f \n %f %f %f", Rz[0][0],Rz[0][1],Rz[0][2],Rz[1][0],Rz[1][1],Rz[1][2],Rz[2][0],Rz[2][1],Rz[2][2]);

    // ROS_INFO("\n Rzyx \n %f %f %f \n %f %f %f \n %f %f %f", Rzyx[0][0],Rzyx[0][1],Rzyx[0][2],Rzyx[1][0],Rzyx[1][1],Rzyx[1][2],Rzyx[2][0],Rzyx[2][1],Rzyx[2][2]);

    ROS_INFO("\n Ri \n %f %d %f \n %d %f %f \n %d %d %d", config.fx,0,config.cx,0 ,config.fy,config.cy,0,0,1);

    ROS_INFO("\n Re \n %lf %lf %lf %lf \n %lf %lf %lf %lf \n %lf %lf %lf %lf", Rzyx[0][0],Rzyx[0][1],Rzyx[0][2],config.x ,Rzyx[1][0],Rzyx[1][1],Rzyx[1][2],config.y,Rzyx[2][0],Rzyx[2][1],Rzyx[2][2],config.z);

    ROS_INFO("\n Rc \n %f %f %f %f \n %f %f %f %f \n %f %f %f %f \n---------------------------------------", Rc[0][0],Rc[0][1],Rc[0][2],Rc[0][3] ,Rc[1][0],Rc[1][1],Rc[1][2],Rc[1][3],Rc[2][0],Rc[2][1],Rc[2][2],Rc[2][3]);

    
    for(int i=0;i<3;i++){
        for(int j=0;j<4;j++){
            array_msg.data.push_back(Rc[i][j]);
        }
    }    
    array_msg.data.push_back(0);
    array_msg.data.push_back(0);
    array_msg.data.push_back(0);
    array_msg.data.push_back(1);




    //New file save
    n+=1;
    ofstream fout;

    char fl[100] = "\0";
    sprintf(fl, "//home//kwally//GUI_ws//src//dynamic_reconfigure_test//cfg//test-%d.cfg",n);


	fout.open(fl); 
	fout << "# !/usr/bin/env python" << endl;
	fout << "\n" << endl;
	fout << "from dynamic_reconfigure.parameter_generator_catkin import *" << endl;
	fout << "\n" << endl;
	fout << "gen = ParameterGenerator()" << endl;
	fout << "\n" << endl;

	fout << "gen.add(\"fx\", double_t, 0, \"A test parameter\", 0,-180, 180)" << endl;
	fout << "gen.add(\"fy\", double_t, 0, \"A test parameter\", 0, -180, 180)" << endl;
	fout << "gen.add(\"cx\", double_t, 0, \"A test parameter\", 0, -180, 180)" << endl;
	fout << "gen.add(\"cy\", double_t, 0, \"A test parameter\", 0, -180, 180)" << endl;
	fout << "\n" << endl;

    fout << "gen.add(\"x\", double_t, 0, \"A test parameter\", ";
    fout << ("%f",config.x);
    fout << ", -180, 180)" << endl;

    fout << "gen.add(\"y\", double_t, 0, \"A test parameter\", ";
    fout << ("%f",config.y);
    fout << ", -180, 180)" << endl;

    fout << "gen.add(\"z\", double_t, 0, \"A test parameter\", ";
    fout << ("%f",config.z);
    fout << ", -180, 180)" << endl;

	fout << "\n" << endl;

    fout << "gen.add(\"roll\", double_t, 0, \"A test parameter\", ";
    fout << ("%f",config.roll);
    fout << ", -180, 180)" << endl;

    fout << "gen.add(\"pitch\", double_t, 0, \"A test parameter\", ";
    fout << ("%f",config.pitch);
    fout << ", -180, 180)" << endl;

    fout << "gen.add(\"yaw\", double_t, 0, \"A test parameter\", ";
    fout << ("%f",config.yaw);
    fout << ", -180, 180)" << endl;

	fout << "\n" << endl;

    fout << "exit(gen.generate(PACKAGE, \"dynamic_reconfigure_test\", \"dyn_reconfig_test\"))" << endl;

	fout.close();

   


    pub_flag = true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "dynamic_reconfigure_test");
    ros::NodeHandle n;
    ros::Publisher extrinsic_matrix_publisher = n.advertise<std_msgs::Float64MultiArray>("extrinsic_matrix", 1000);//Publish topic wheel_speed_publisher
    dynamic_reconfigure::Server<dynamic_reconfigure_test::dyn_reconfig_testConfig> server;
    dynamic_reconfigure::Server<dynamic_reconfigure_test::dyn_reconfig_testConfig>::CallbackType f;

    ROS_INFO("MAIN\n");
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);
        ros::Rate loop_rate(10);

    while(ros::ok)
    {
        if (pub_flag == true)
        {   ROS_INFO("pub!\n");
            extrinsic_matrix_publisher.publish(array_msg);
            pub_flag = false;
            array_msg.data.clear();
        }
        ros::spinOnce();
    }
    return 0;
  
}