#include <iostream>
#include <fstream>
#include <string>
using namespace std;



int main(){

  // Get tf data
  double xt = 2.5723;
  double yt = -5.9239;
  double zt = -17.1596;  
  double roll = -88.4137;
  double pitch = -88.8211;
  double yaw = 178.32745;



	ofstream fout;
	char x[100];
	char y[100];
	char z[100];
	char troll[100];
	char tpitch[100];
	char tyaw[100];

	fout.open("../../dynamic_reconfigure_test//cfg//test.cfg"); 
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

	sprintf(x, "gen.add(\"x\", double_t, 0, \"A test parameter\", %f, -180, 180)", xt);
	fout << x << endl;
	sprintf(y, "gen.add(\"y\", double_t, 0, \"A test parameter\", %f, -180, 180)", yt);
	fout << y << endl;
	sprintf(z, "gen.add(\"z\", double_t, 0, \"A test parameter\", %f, -180, 180)", zt);
	fout << z << endl;  
	fout << "\n" << endl;

	sprintf(troll, "gen.add(\"roll\", double_t, 0, \"A test parameter\", %f, -180, 180)", roll);
	fout << troll << endl;
	sprintf(tpitch, "gen.add(\"pitch\", double_t, 0, \"A test parameter\", %f, -180, 180)", pitch);
	fout << tpitch << endl;
	sprintf(tyaw, "gen.add(\"yaw\", double_t, 0, \"A test parameter\", %f, -180, 180)", yaw);
	fout << tyaw << endl;  
	fout << "\n" << endl;

	fout.close();


  return 0;
}
