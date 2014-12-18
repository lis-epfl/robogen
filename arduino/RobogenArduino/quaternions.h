/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*******************************************************************************
 * \file quaternions.h
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Geraud L'Eplattenier
 *   
 * \brief Useful functions for quaternions
 *
 ******************************************************************************/


#ifndef QUATERNIONS_H_
#define QUATERNIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include "vectors.h"


/**
 * \brief 		Unit quaternion
 * 
 * \details  	The quaternions are in the form q = [s, v_1, v_2, v_3]
 */
typedef struct 
{
	float s;			///<	Scalar component
	float v[3];			///<	Vector component
} quat_t;


/**
 * \brief 		Quaternion inverse
 * 
 * \param 	q  	Input quaternion
 * 
 * \return  	Inversed quaternion
 */
#define QI(q, out) \
	out.s = q.s;\
	out.v[0] = -q.v[0];\
	out.v[1] = -q.v[1];\
	out.v[2] = -q.v[2];


/**
 * \brief 		Creates a quaternion from a vector of dimension 4, MACRO version
 * 
 * \param 	q 	Output quaternion
 * \param 	s 	Scalar part of the quaternion
 * \param 	v0 	First component of the vector part of the quaternion
 * \param 	v1 	Second component of the vector part of the quaternion
 * \param 	v2 	Third component of the vector part of the quaternion
 */
#define QUAT(q, s, v0, v1, v2) \
	q.s = s;\
	q.v[0] = v0;\
	q.v[1] = v1;\
	q.v[2] = v2;


/**
 * \brief 		Creates a quaternion from a vector of dimension 3
 * 
 * \param 	v 	Array of floats of dimension 3 containing the vector components
 * 
 * \return 		Unit quaternion
 */
quat_t static inline quaternions_create_from_vector(float v[3])
{
	quat_t q;	
	q.s = 0; 
	q.v[0] = v[0]; 
	q.v[1] = v[1]; 
	q.v[2] = v[2];
	return q;
}


/**
 * \brief 			Quaternion multiplication
 * 
 * \param 	q1 		Input quaternion
 * \param 	q2 		Input quaternion
 * \param 	out 	Output quaternion
 */
#define QMUL(q1,q2,out) \
	tmp[0] = q1.v[1] * q2.v[2] - q1.v[2] * q2.v[1];\
	tmp[1] = q1.v[2] * q2.v[0] - q1.v[0] * q2.v[2];\
	tmp[2] = q1.v[0] * q2.v[1] - q1.v[1] * q2.v[0];\
	out.v[0] = q2.s* q1.v[0] + q1.s * q2.v[0] + tmp[0];\
	out.v[1] = q2.s* q1.v[1] + q1.s * q2.v[1] + tmp[1];\
	out.v[2] = q2.s* q1.v[2] + q1.s * q2.v[2] + tmp[2];\
	out.s= q1.s * q2.s - SCP(q1.v, q2.v);


/**
 * \brief 			Multiplies two unit quaternions
 * 
 * \param 	q1 		Input quaternion
 * \param 	q2 		Input quaternion
 * 
 * \return 			Output quaternion
 */
quat_t static inline quaternions_multiply(const quat_t q1, const quat_t q2)
{
	float tmp[3];
	quat_t out;

	tmp[0] = q1.v[1] * q2.v[2] - q1.v[2] * q2.v[1];
	tmp[1] = q1.v[2] * q2.v[0] - q1.v[0] * q2.v[2];
	tmp[2] = q1.v[0] * q2.v[1] - q1.v[1] * q2.v[0];
	
	out.v[0] = q2.s * q1.v[0] + q1.s * q2.v[0] + tmp[0];
	out.v[1] = q2.s * q1.v[1] + q1.s * q2.v[1] + tmp[1];
	out.v[2] = q2.s * q1.v[2] + q1.s * q2.v[2] + tmp[2];
	out.s = q1.s * q2.s - vectors_scalar_product(q1.v, q2.v);
	
	return out;
}


/**
 * \brief 		Inverse quaternion
 * 
 * \param 	q 	Input quaternion
 * \return 		Output quaternion
 */
quat_t static inline quaternions_inverse(const quat_t q)
{
	quat_t qinv;
	qinv.s = q.s;
	int32_t  i;
	for(i = 0; i < 3; i++)
	{
		qinv.v[i] = -q.v[i];
	}
	
	return qinv;
}


/**
 * \brief 			Rotates a vector from global frame to local frame according to an attitude quaternion
 * 
 * \details 		The vector is given in the form of a quaternion with the scalar term equal to 0
 * 
 * \param 	qe 		Attitude quaternion
 * \param 	qvect 	Quaternion containing the vector to be rotated
 * 
 * \return 			Output quaternion
 */
quat_t static inline quaternions_global_to_local(const quat_t qe, const quat_t qvect)
{
	quat_t qinv, qtmp;
	
	qinv = quaternions_inverse(qe);
	qtmp = quaternions_multiply(qinv,qvect);
	qtmp = quaternions_multiply(qtmp,qe);

	return qtmp;
}


/**
 * \brief 			Rotates a vector from local frame to global frame according to an attitude quaternion
 * 
 * \details 		The vector is given in the form of a quaternion with the scalar term equal to 0
 * 
 * \param 	qe 		Attitude quaternion
 * \param 	qvect 	Quaternion containing the vector to be rotated
 * 
 * \return 			Output quaternion
 */
quat_t static inline quaternions_local_to_global(const quat_t qe, const quat_t qvect)
{
	quat_t qinv, qtmp;
	
	qinv = quaternions_inverse(qe);
	qtmp = quaternions_multiply(qe, qvect);
	qtmp = quaternions_multiply(qtmp, qinv);
	
	return qtmp;
}


/**
 * \brief 			Rotates a vector according to a unit quaternion
 * 
 * \details 		This is an optimized implementation that does not require quaternion multiplications
 * 					It should run more than 2 times faster than the standard implementation
 * 
 * \param 	q 		unit quaternion
 * \param 	u 		input vector
 * \param 	v 		rotated vector (output)
 * 
 */
void static inline quaternions_rotate_vector(const quat_t q, const float u[3], float v[3])
{
	float tmp1[3], tmp2[3];

	vectors_cross_product(q.v, u, tmp1);
	tmp1[0] = 2 * tmp1[0];
	tmp1[1] = 2 * tmp1[1];
	tmp1[2] = 2 * tmp1[2];

	vectors_cross_product(q.v, tmp1, tmp2);
	
	v[0] = u[0] + q.s * tmp1[0] + tmp2[0];
	v[1] = u[1] + q.s * tmp1[1] + tmp2[1];
	v[2] = u[2] + q.s * tmp1[2] + tmp2[2];
}


/**
 * \brief 		Normalises a quaternion
 * 
 * \param 	q 	Input quaternion
 * \return 		Unit quaternion
 */
static inline quat_t quaternions_normalise(const quat_t q)
{
	quat_t result;
	
	float snorm = sq(q.s) + sq(q.v[0]) + sq(q.v[1]) + sq(q.v[2]);

	if (snorm > 0.0000001f) 
	{
		float norm = sqrt(snorm);
		result.s = q.s / norm;
		result.v[0] = q.v[0] / norm;		
		result.v[1] = q.v[1] / norm;		
		result.v[2] = q.v[2] / norm;
	}
	else
	{
		result.s = 1.0f;
		result.v[0] = 0.0f;
		result.v[1] = 0.0f;
		result.v[2] = 0.0f;
	}

	return result;
}


#ifdef __cplusplus
}
#endif

#endif /* QUATERNIONS_H_ */

