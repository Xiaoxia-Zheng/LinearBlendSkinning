----------------------
CMSC691 Assignment 3
----------------------
Xiaoxia Zheng / CE83376


----------------------
Command Line Arguments
----------------------
Example: ./main ogre.obj ogre-skeleton.bf ogre-weights.dmat pose.dmat output-%05d.obj

	"ogre.obj", "ogre-skeleton.bf", "ogre-weights.dmat" and "pose.dmat" are the files that we need to input
	"output-%05d.pose" is the format that we output the files. For this project, you'll have two output files.

-----------------
Project sturcture
-----------------
1. Read all input files.
2. Caculate all joints' world coordinate.
3. For each pose, calculate the rotation matrix of each joint, then left multiply to the root according to their parents.
4. Using the equation: V_new = sum( W_iBone * ( R_activePosetoRoot * ( V_rest - i_BoneRest'sParent) + i_BoneNew'sParent) )
   to calculate the output vertex.
6. Output vertext to obj. file.


------------------------------------    
Problems I met and resource I use
------------------------------------
1. I read the slices that the professor provided and I also get some help from the professor. StackoverFlow helps a lot too.
2. The most problem I met in this project is, I first didn't loop the rotation matrix from the joint itself back to the root
   according to their parents. That makes my output joints' position are correct, but the skin is totally wrong. 
3. It's difficult to debug because of the complex data structure. I spend lots of time on debugging.
