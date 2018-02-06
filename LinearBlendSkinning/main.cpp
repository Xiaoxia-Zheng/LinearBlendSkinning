#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <math.h>
#include "slVector.H"
#include <getopt.h>
#include <cstdio>



class SlTri {
	public:
	int indices[3];
	inline int &operator[](const unsigned int &i) { return indices[i];};
	inline int operator[](const unsigned int &i) const { return indices[i];};
	inline void set(int x, int y, int z) {indices[0] = x; indices[1] = y; indices[2] = z;};
	inline SlTri(int x, int y, int z) {indices[0] = x; indices[1] = y; indices[2] = z;};
	inline SlTri() {};
	inline SlTri &operator=(const SlTri &that);
};

bool loadObj(std::string fname, std::vector<SlVector3> &pts, std::vector<SlTri> &triangles) {
	char c[500];
	
	int numVertices=0, numFaces=0;
	bool normals = false, texture = false;
	int tint;
	char ch;
	int p, q, r;
	double x, y, z;
	std::vector<SlVector3>::iterator v;
	std::vector<SlTri>::iterator t;
	std::ifstream in1(fname, std::ios::in);
	//printf("%s", fname);
	if (!in1.is_open()) {
		return false;
	}
	in1.flags(in1.flags() & ~std::ios::skipws);
	
	while (in1>>ch) {
		if (ch == 'v') {
			in1>>ch;
			if (ch == ' ') numVertices++;
			else if (ch == 'n') normals = true;
			else if (ch == 't') texture = true;
			else std::cerr<<"error \'"<<ch<<"\'"<<std::endl;
		} else if (ch == '#') {
	  while (in1 >> ch && ch != '\n') ; // Read to the end of the line.
		} else if (ch == 'f') numFaces++;
	}
	in1.close();
	
	pts.resize(numVertices);
	triangles.resize(numFaces);
	v = pts.begin();
	t = triangles.begin();
	
	std::ifstream in(fname, std::ios::in);
	if (!in.is_open()) {
		return false;
	}
	
	while (in>>ch) {
		if (ch == '#') {
			in.getline(c,500);
			continue;
		}
		if (ch == 'g') {
			in.getline(c,500);
			continue;
		}
		if (ch == 's') {
			in.getline(c,500);
			continue;
		}
		if (ch == 'm') {
			in.getline(c,500);
			continue;
		}
		if (ch == 'u') {
			in.getline(c,500);
			continue;
		}
		if (ch == 'v') {
			ch = in.peek();
			if (ch != 't' && ch != 'n') {
				in>>x>>y>>z;
				(*v).set(x,y,z);
				v++;
			} else {
				in.getline(c, 500);
			}
			continue;
		}
		if (ch == 'f') {
			if (normals && texture) {
				in>>p>>ch>>tint>>ch>>tint>>q>>ch>>tint>>ch>>tint>>r>>ch>>tint>>ch>>tint;
			} else if (normals) {
				in>>p>>ch>>ch>>tint>>q>>ch>>ch>>tint>>r>>ch>>ch>>tint;
			} else if (texture) {
				in>>p>>ch>>tint>>q>>ch>>tint>>r>>ch>>tint;
			} else {
				in>>p>>q>>r;
			}
			(*t)[0] = p-1;
			(*t)[1] = q-1;
			(*t)[2] = r-1;
			t++;
			continue;
		}
	}
	in.close();
	return true;
}

void writeObj(const char *fname, const std::vector<SlVector3> &meshPts, const std::vector<SlTri> &triangles, int pos_num) {
	std::ofstream out;
	std::vector<SlVector3>::const_iterator p;
	std::vector<SlTri>::const_iterator t;
	
	char file_name[200];
	sprintf(file_name, fname, pos_num);
	out.open(file_name);
	
	for (p=meshPts.begin(); p!=meshPts.end(); p++)
	out<<"v "<<(*p)[0]<<" "<<(*p)[1]<<" "<<(*p)[2]<<std::endl;
	
	for (t=triangles.begin(); t!=triangles.end(); t++)
	out<<"f "<<(*t)[0]+1<<" "<<(*t)[1]+1<<" "<<(*t)[2]+1<<std::endl;
	
	out.close();
	
}


//Read the input bf file.
//Store data to new data structure and return a 2D vector.
std::vector<std::vector<double> > read_bf(std::string bf, int rows, int cols){
	std::vector<std::vector<double> >  bf_data(rows, std::vector<double>(cols, 0));
	std::fstream bf_file(bf, std::ios_base::in);
	
	if (bf_file.is_open()) {
		for (int i =0; i<rows; i++) {
			for (int j=0; j<cols; j++) {
				bf_file >> bf_data[i][j];
				//printf("%f ", bf_data[i][j]);
			}
		}
	}
	bf_file.close();
	return bf_data;
}



//Read the input pose file.
//Store data to new data structure and return a 2D vector.
std::vector<std::vector<double> > read_pos(std::string pos){
	std::fstream pos_file(pos, std::ios_base::in);
	
	int pos_cols, pos_rows;
	pos_file>>pos_rows>>pos_cols;
	std::vector<std::vector<double> >  pos_data(pos_rows, std::vector<double>(pos_cols, 0));
	for (int i=0; i<pos_rows; i++) {
		for (int j=0; j<pos_cols; j++) {
			pos_file >> pos_data[i][j];
			//			printf("%f ", pos_data[i][j]);
		}
	}
	pos_file.close();
	return pos_data;
}



//Read the input weights file.
//Store data to new data structure and return a 2D vector.
std::vector<std::vector<double> > read_weights(std::string pos){
	std::fstream weis_file(pos, std::ios_base::in);
	
	int weis_cols, weis_rows;
	weis_file>>weis_rows>>weis_cols;
	std::vector<std::vector<double> >  weis_data(weis_rows, std::vector<double>(weis_cols, 0));
	for (int i=0; i<weis_rows; i++) {
		for (int j=0; j<weis_cols; j++) {
			weis_file >> weis_data[i][j];
		}
	}
	weis_file.close();
	return weis_data;
	
}


//Using a recursive function to iterate all joints' parents' coordinate.
//return all joints' world coordinate.
std::vector<double> locToWorld(std::vector< std::vector<double> > bf, std::vector<double> world, int joint_idx){
	int parent_idx;
	std::vector< std::vector<double> > bf_world;
	
	//If joint's index equals to 0 means the function has iterate to the root.
	//Then the recursive function stops.
	if (joint_idx == 0) {
		return world;
	}else{
		parent_idx = bf[joint_idx][1];
		world[1] += bf[parent_idx][2];
		world[2] += bf[parent_idx][3];
		world[3] += bf[parent_idx][4];
		world = locToWorld(bf, world, parent_idx); //Iterating back to parents'.
		return world;
	}
}




//Define the rotation matrix to trasform euler degree coordinate to matrix.
std::vector<double> rotationMatrix(float radX, float radY, float radZ){
	float cX = cos(radX/180 * M_PI);
	float sX = sin(radX/180 * M_PI);
	
	float cY = cos(radY/180 * M_PI);
	float sY = sin(radY/180 * M_PI);
	
	float cZ = cos(radZ/180 * M_PI);
	float sZ = sin(radZ/180 * M_PI);
	
	std::vector<double> matrix3f=
	{
		cZ*cY, cZ*sY*sX-sZ*cY, cZ*sY*cX+sZ*sX,
		sZ*cY, sZ*sY*sX+cZ*cX, sZ*sY*cX-cZ*sX,
		-sY, cY*sX, cY*cX
	};
	
	return matrix3f;
}


//Using a recursive function to iterate all joints' parents' transforms.
std::vector<double> eulerToWorld(std::vector<std::vector<double> > bf, std::vector<std::vector<double> > pos, std::vector<double> result, int joint_idx){
	int parent_idx;
	std::vector<double> tmp =
	{
		result[0]*pos[joint_idx][0] + result[1]*pos[joint_idx][1] + result[2]*pos[joint_idx][2],
		result[0]*pos[joint_idx][3] + result[1]*pos[joint_idx][4] + result[2]*pos[joint_idx][5],
		result[0]*pos[joint_idx][6] + result[1]*pos[joint_idx][7] + result[2]*pos[joint_idx][8]
	};
	
	//If joint's index equals to 0 means the function has iterate to the root.
	//Then the recursive function stops.
	if (joint_idx == 0) {
		return tmp;
	}else{
		parent_idx = bf[joint_idx][1];
		tmp[0] += bf[parent_idx][2];
		tmp[1] += bf[parent_idx][3];
		tmp[2] += bf[parent_idx][4];
		tmp = eulerToWorld(bf, pos, tmp, parent_idx); //Iterating back to parents'.
		return tmp;
	}
}


//3X3 matrix multiplication
std::vector<double> mat33Mul(std::vector<double> mat1, std::vector<double> mat2){
	std::vector<double> mat_mul = {0,0,0,0,0,0,0,0,0};
	for(int i=0; i<3; i++){
		for (int j=0; j<3; j++) {
			mat_mul[i*3+j] = mat1[i*3+0] * mat2[0+j]
				+ mat1[i*3+1] * mat2[3+j]
				+ mat1[i*3+2] * mat2[6+j];
		}
	}
	
	return mat_mul;
}


int main(int argc, const char * argv[]) {
	
	std::vector<SlVector3> vertex;
	std::vector<SlTri> triangle;
	std::string objFileName = argv[1];
	std::string bf_input = argv[2];
	std::string weis_input = argv[3];
	std::string pos_input = argv[4];
	const char *obj_output = argv[5];
	int bf_cols = 5;
	int bf_rows = 23;
	
	//load .obj file
	loadObj(objFileName, vertex, triangle);
//	printf("vertex size: %lu\n", vertex.size());
	
	//load .bf file
	std::vector< std::vector<double> > bfData;
	bfData = read_bf(bf_input, bf_rows, bf_cols);
	
	//load .pose file
	std::vector<std::vector<double> > posData;
	posData = read_pos(pos_input);
	
	//load weights file
	std::vector<std::vector<double> > weisData;
	weisData = read_weights(weis_input);
	
	
	//Initiate all joints from local coordinates to world coordinates.
	//for calculate vi - ib;
	std::vector< std::vector<double> > bf_world;
	for (int i=0; i<bf_rows; i++) {
		std::vector<double> tmp = {bfData[i][0], bfData[i][2], bfData[i][3], bfData[i][4]};
		std::vector<double> temp = locToWorld(bfData, tmp, i);
		bf_world.push_back(temp);
	}
	
	

	for (int pos_num=0; pos_num<posData.size(); pos_num++) {
		//Covert the pos_data to matrix using method rotationMatrix();
		std::vector<std::vector<double> >  pos_matrix;
		for (int j=0; j<posData[pos_num].size(); j=j+3) {
			std::vector<double> tmp;
			tmp = rotationMatrix(posData[pos_num][j], posData[pos_num][j+1], posData[pos_num][j+2]);
			pos_matrix.push_back(tmp);
		}
		
		//Calculating Chained rotationMatrix
		std::vector<std::vector<double> >  pos_matrix_root;
		for (int j=0; j<pos_matrix.size(); j++) {
			std::vector<double> pos_mat_mul = pos_matrix[j];
			int parent_idx = bfData[j][1];
			while (parent_idx > 0) {
				pos_mat_mul = mat33Mul(pos_matrix[parent_idx], pos_mat_mul);
				parent_idx = bfData[parent_idx][1];
			}
			pos_matrix_root.push_back(pos_mat_mul);
		}
		
		//Iterate all parents' transform using recursive function eulerToWorld();
		//Then finally output all joints' world coordinates.
		std::vector<std::vector<double> > pose_world;
		for (int i=0; i<bf_rows; i++) {
			std::vector<double> tmp = {bfData[i][2], bfData[i][3], bfData[i][4]};
			std::vector<double> temp = eulerToWorld(bfData, pos_matrix, tmp, i);
			pose_world.push_back(temp);
			//printf("%f %f %f\n", temp[0], temp[1], temp[2]);
		}
		

		//V_new = sum( W_iBone * ( R_activePosetoRoot * ( V_rest - i_BoneRest'sParent) + i_BoneNew'sParent) )
		std::vector<SlVector3> output_vertex;
		SlVector3 vertex_j_Slvec;
		for (int i=0; i<vertex.size(); i++) {
			double vertex_x = 0.0;
			double vertex_y = 0.0;
			double vertex_z = 0.0;
			
			for (int j=1; j<bf_world.size(); j++) {
				vertex_x += weisData[j - 1][i] *
					( pos_matrix_root[j][0] * (vertex[i].data[0] - bf_world[ bfData[j][1] ][1])
					+ pos_matrix_root[j][1] * (vertex[i].data[1] - bf_world[ bfData[j][1] ][2])
					+ pos_matrix_root[j][2] * (vertex[i].data[2] - bf_world[ bfData[j][1] ][3])
					+ pose_world[ bfData[j][1] ][0] );
				
				vertex_y += weisData[j - 1][i] *
					( pos_matrix_root[j][3] * (vertex[i].data[0] - bf_world[ bfData[j][1] ][1])
					+ pos_matrix_root[j][4] * (vertex[i].data[1] - bf_world[ bfData[j][1] ][2])
					+ pos_matrix_root[j][5] * (vertex[i].data[2] - bf_world[ bfData[j][1] ][3])
					+ pose_world[ bfData[j][1] ][1] );
				
				vertex_z += weisData[j - 1][i] *
					( pos_matrix_root[j][6] * (vertex[i].data[0] - bf_world[ bfData[j][1] ][1])
					+ pos_matrix_root[j][7] * (vertex[i].data[1] - bf_world[ bfData[j][1] ][2])
					+ pos_matrix_root[j][8] * (vertex[i].data[2] - bf_world[ bfData[j][1] ][3])
					+ pose_world[ bfData[j][1] ][2] );
				
			}
			vertex_j_Slvec = SlVector3(vertex_x, vertex_y, vertex_z);
			output_vertex.push_back(vertex_j_Slvec);
		}
		
		//Output vertex to file.
		writeObj(obj_output, output_vertex, triangle, pos_num);

	}
	
	
}






