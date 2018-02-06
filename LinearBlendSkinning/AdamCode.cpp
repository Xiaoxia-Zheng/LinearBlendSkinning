//
//  AdamCode.cpp
//  project_3
//
//  Created by Zheng Li on 10/13/17.
//  Copyright © 2017 Xiaoxia Zheng. All rights reserved.
//

#include "AdamCode.hpp"
#include <fstream>
#include <cstring>
#include <vector>
#include <string>
#include <iostream>
#include "slVector.H"
#include "slMatrix.H"
#include <getopt.h>

struct Joint {
	int parent;
	std::vector<int> children;
	SlVector3 offset;
	int weightIndex;
	SlMatrix3x3 Binv, T;
	SlVector3 binv, t;
};

void loadBoneForest(char *fname, std::vector<Joint> &joints) {
	std::ifstream in(fname, std::ios::in);
	Joint j;
	while(in>>j.weightIndex>>j.parent>>j.offset[0]>>j.offset[1]>>j.offset[2]) {
		if (j.parent != -1) joints[j.parent].children.push_back(joints.size());
		joints.push_back(j);
	}
}

void loaddmat(char *fname, std::vector<std::vector<double> > &mat) {
	std::ifstream in(fname, std::ios::in);
	int cols, rows;
	in>>cols>>rows;
	mat.resize(cols);
	for (int i=0; i<cols; i++) {
		mat[i].resize(rows);
		for (int j=0; j<rows; j++) {
	  in>>mat[i][j];
		}
	}
}

void recurse(std::vector<Joint> &joints, const std::vector<double> &frame, int joint, SlMatrix3x3 m, SlVector3 t) {
	Joint &j = joints[joint];
	SlMatrix3x3 r;
	//SlQuaternionToMatrix(SlVector3(frame[4*joint+1], frame[4*joint+2], frame[4*joint+3]), frame[4*joint], r);
	SlEulerAngToMatrixXYZ(SlVector3(frame[3*joint+0]*M_PI/180.0, frame[3*joint+1]*M_PI/180.0, frame[3*joint+2]*M_PI/180.0), r);
	//std::cout<<frame[3*joint+0]<<" "<<frame[3*joint+1]<<" "<<frame[3*joint+2]<<" "<<std::endl;
	//std::cout<<r<<std::endl;
	j.T = m = m*r;
	t += m*j.offset;
	j.t = t;
	std::cout<<j.weightIndex<<std::endl;
	std::cout<<m<<" "<<t<<std::endl<<std::endl;
	for (unsigned int i=0; i<j.children.size(); i++) {
		recurse(joints, frame, j.children[i], m, t);
	}
}

void processPose(char *fname, std::vector<Joint> &joints, const std::vector<double> &pose) {
	SlMatrix3x3 m;
	m.setIdentity();
	recurse(joints, pose, 0, m, SlVector3(0.0,0.0,0.0));
	std::ofstream out(fname, std::ios::out);
	for (unsigned int j = 0; j<joints.size(); j++) {
		out<<joints[j].weightIndex<<" "<<joints[j].t[0]<<" "<<joints[j].t[1]<<" "<<joints[j].t[2]<<std::endl;
	}
}

int main(int argc, char *argv[]) {
	int c;
	bool interpolate = false;
	bool qlerp = false;
	int nframes = 0;
	std::vector<Joint> joints;
	std::vector<std::vector<double> > poses;
	
	while ((c = getopt(argc, argv, "i:q:")) != -1) {
		switch(c) {
			case 'i':
	  nframes = atof(optarg);
	  interpolate = true;
	  break;
			case 'q':
	  nframes = atof(optarg);
	  qlerp = true;
	  break;
			default:
	  abort();
		}
	}
	
	if (argc-optind != 3) {
		std::cout<<"usage: forwardKinematics [options] input.bf pose.dmat output-%05d.pose"<<std::endl;
		for (unsigned int i=0; i<argc; i++) std::cout<<argv[i]<<std::endl;
		exit(0);
	}
	loadBoneForest(argv[optind], joints);
	loaddmat(argv[optind+1], poses);
	
	std::vector<double> rest;
	for (unsigned int i=0; i<joints.size(); i++) {
		rest.push_back(1.0); rest.push_back(0.0); rest.push_back(0.0); rest.push_back(0.0);
	}
	
	SlMatrix3x3 m;
	m.setIdentity();
	
	recurse(joints, rest, 0, m, SlVector3(0.0,0.0,0.0));
	for (unsigned int i=0; i<joints.size(); i++) {
		joints[i].Binv = inverse(joints[i].T);
		joints[i].binv = -joints[i].t;
	}
	
	char fname[500];
	sprintf(fname, argv[optind+2], 0);
	processPose(fname, joints, poses[0]);
	for (unsigned int i=0; i<poses.size()-1; i++) {
		for (unsigned int j=1; j <= nframes+1; j++) {
	  sprintf(fname, argv[optind+2], (nframes+1)*i+j);
	  std::vector<double> pose;
	  double w = ((double)j)/(nframes+1.0);
	  pose.resize(poses[i].size());
	  if (qlerp) {
		  for (unsigned int k=0; k<joints.size(); k++) {
			  SlMatrix3x3 r, r0, r1;
			  SlVector3 q, q0, q1, ea;
			  double d, d0, d1;
			  SlEulerAngToMatrixXYZ(SlVector3(poses[i][3*k+0]*M_PI/180.0, poses[i][3*k+1]*M_PI/180.0, poses[i][3*k+2]*M_PI/180.0), r0);
			  SlEulerAngToMatrixXYZ(SlVector3(poses[i+1][3*k+0]*M_PI/180.0, poses[i+1][3*k+1]*M_PI/180.0, poses[i+1][3*k+2]*M_PI/180.0), r1);
			  SlMatrixToQuaternion(r0, q0, d0);
			  SlMatrixToQuaternion(r1, q1, d1);
			  q = (1.0-w)*q0 + w*q1;
			  d = (1.0-w)*d0 + w*d1;
			  SlQuaternionToMatrix(q,d,r);
			  SlMatrixToEulerAngXYZ(r, ea);
			  pose[3*k+0] = 180.0*ea[0]/M_PI;
			  pose[3*k+1] = 180.0*ea[1]/M_PI;
			  pose[3*k+2] = 180.0*ea[2]/M_PI;
		  }
	  } else {
		  for (unsigned int k=0; k<poses[i].size(); k++) {
			  pose[k] = ((1.0-w) * poses[i][k] + w * poses[i+1][k]);
		  }
	  }
	  processPose(fname, joints, pose);
		}
	}
	// this loop is just a sanity check on the skeleton hierarchy
	for (unsigned int i=0; i<joints.size(); i++) {
		for (unsigned int j=0; j<joints[i].children.size(); j++) {
	  if (joints[joints[i].children[j]].parent != i) std::cout<<"problem"<<std::endl;
		}
	}
}
