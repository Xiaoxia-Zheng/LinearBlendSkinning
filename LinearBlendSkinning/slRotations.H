//-------------------------------------------------------------------
//-------------------------------------------------------------------
//
// Simple Spring Mass System
// -- rotations lib
//
// Primary Author: James F. O'Brien (obrienj@cc.gatech.edu)
//
//-------------------------------------------------------------------
//-------------------------------------------------------------------
//
// Copyright (c) 2003-2005, Regents of the University of California.  All
// rights reserved.
//
// This software is part of the Berkeley Fluid Animation & Simulation
// Toolkit.  The name "Berkeley Fluid Animation & Simulation Toolkit" is
// a trademark of the Regents of the University of California.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//   Redistributions of source code must retain the above copyright
//  notice, this list of conditions and the following disclaimer.
//
//  Redistributions in binary form must reproduce the above copyright
//  notice, this list of conditions and the following disclaimer in the
//  documentation and/or other materials provided with the distribution.
//
//  Redistributions in binary form as an executable program, or part of
//  an executable program must, when run for the first time by a given
//  user, prominently display the above copyright notice, this list of
//  conditions and the following disclaimer.
//
//  Neither the name of the University of California, Berkeley nor the
//  names of its contributors may be used to endorse or promote products
//  derived from this software without specific prior written
//  permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//-------------------------------------------------------------------
//-------------------------------------------------------------------


#ifndef SlROTATIONS_is_defined
#define SlROTATIONS_is_defined

//-------------------------------------------------------------------
//-------------------------------------------------------------------
// External classes

class SlVector3;
class SlMatrix3x3;


//-------------------------------------------------------------------
//-------------------------------------------------------------------
// Moments.. Not a separate class, just a use of a vector.

void SlMomentToPosMatrix(const SlVector3 &v, SlMatrix3x3 &m);
void SlMomentToVelMatrix(const SlVector3 &v, SlMatrix3x3 &m);
// The Pos version converts the moment as an axis-amount
// into a rotation matrix.  The Vel version treats v as
// a rotational velocity and makes a transformation
// matrix that will convert a  point in the local system
// of the moment in to a local linear velocity.  In both
// cases the amount is givent by the magnitude of v in
// radians (or radians/sec).

void SlPosMatrixToMoment(const SlMatrix3x3 &m, SlVector3 &v);
void SlVelMatrixToMoment(const SlMatrix3x3 &m, SlVector3 &v);
// Does the oppisite of SlMomentToPosMatrix or SlMomentToVelMatrix


//-------------------------------------------------------------------
//-------------------------------------------------------------------
// Euler Angles 

void SlEulerAngToMatrixXYZ(const SlVector3 &ea, SlMatrix3x3 &m);
// Converts the Euler Angles in ae (x,y,z) to a rotation
// matrix.  They are transformed in order, x,y,z.

void SlMatrixToEulerAngXYZ(const SlMatrix3x3 &m, SlVector3 &ea);
// Converts a rotation matrix to Euler angles in x,y,z order.
// No error checking to insure that m is a true rotation matrix.


//-------------------------------------------------------------------
//-------------------------------------------------------------------
// Quaternions
// Quaternions should be a full fledged class...

void SlQuaternionToMatrix(const SlVector3 &xyz, double w, SlMatrix3x3 &m); 
// Converts a quaternion to a rotation matrix.  The quaternion is
// automaticly normalized.

void SlMatrixToQuaternion(const SlMatrix3x3 &m, SlVector3 &xyz, double &w);
// Converts a rotation matrix to a quaternion.  No error checking to
// insure that m is a true rotation matrix.


void SlQuaternionToMoment(const SlVector3 &xyz, double w, SlVector3 &m);
// Converts a quaternion to a rotation vector.  The quaternion is
// automaticly normalized.

void SlMomentToQuaternion(const SlVector3 &m, SlVector3 &xyz, double &w);
// Converts a rotaion vector to a quaternion.

void SlSlerpQuaternion(SlVector3  xyz0, double  w0,
		       SlVector3  xyz1, double  w2, 
		       SlVector3 &xyzT, double &wT, double t);
// Interpolates (slerp) between quat0 and quat1.  Result is returned
// in quatT.  t is the interpolation parameter in [0..1].  Values out
// of the range will extrapolate.
		       
//-------------------------------------------------------------------
//-------------------------------------------------------------------
//-------------------------------------------------------------------
//-------------------------------------------------------------------

#endif
