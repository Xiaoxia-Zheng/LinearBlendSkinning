----------------------
Linear Blend Skinning
----------------------
For this project, there is a C++ program that will read a an .obj file containing a polygonal/skin mesh, a .bf file containing a skeleton, a .dmat file that contains linear blend skinning weights for each vertex in the mesh and for each bone in the skeleton, and .dmat file containing a new pose for the skeleton. Your output will be an .obj file that contains a deformed mesh compputed through linear-blend skinning / skeletal subspace deformations. The ogre-rigged file contains a simple rigged ogre and a pose.dmat which gives two poses for the ogre.



--------------
Input / Output
--------------
filename		description
ogre-skeleton.bf	.bf "bone forest" file containing the skeleton description. 
			This is a crude file format that stores joint vertex rest positions, corresponding 
			column indices into the weights matrix, and indices of parent joints.
ogre.obj			.obj mesh of model rest pose
ogre-weights.dmat	.dmat "dense matrix" containing the weights matrix W, which is #mesh vertices 
					by #skeletal bones



----------------------
Command Line Arguments
----------------------
Example: ./main ogre.obj ogre-skeleton.bf ogre-weights.dmat pose.dmat output-%05d.obj

	"ogre.obj", "ogre-skeleton.bf", "ogre-weights.dmat" and "pose.dmat" are the files that we need to input
	"output-%05d.pose" is the format that we output the files. For this project, you'll have two output files.


