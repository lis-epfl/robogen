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
 * \file vectors.h
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Geraud L'Eplattenier
 *   
 * \brief Stream
 *
 ******************************************************************************/


#ifndef VECTORS_H_
#define VECTORS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
//#include "maths.h"


/**
 * \brief 			Cross product between two 3-vectors
 * 
 * \param 	u 		Input vector (dim 3)
 * \param 	v  		Input vector (dim 3)
 * \param 	out 	Output vector (dim 3)
 */
#define CROSS(u,v,out) \
	out[0] = u[1] * v[2] - u[2] * v[1];\
	out[1] = u[2] * v[0] - u[0] * v[2];\
	out[2] = u[0] * v[1] - u[1] * v[0];

	
/**
 * \brief 			Scalar product between two 3-vectors
 * 
 * \param 	v 		Input vector (dim 3)
 * \param 	u 		Input vector (dim 3)
 * 
 * \return 			Scalar product
 */
#define SCP(u,v) \
	(u[0] * v[0] + u[1] * v[1] + u[2] * v[2])


/**
 * \brief 		Computes the scalar product of two vectors of dimension 3
 * 
 * \param 	u 	Input vector (dim 3)
 * \param 	v 	Input vector (dim 3)
 * 
 * \return 		Scalar product
 */
float static inline vectors_scalar_product(const float u[3], const float v[3]) //maths_scalar_product(const float u[3], const float v[3])
{
	float scp = (u[0] * v[0] + u[1] * v[1] + u[2] * v[2]);
	return scp;
}


/**
 * \brief 			Computes the cross product of two vectors of dimension 3
 * 
 * \param 	u 		Input vector (dim 3)
 * \param 	v 		Input vector (dim 3)
 * \param 	out 	Output vector (dim 3)
 */
void static inline vectors_cross_product(const float u[3], const float v[3], float out[3]) //maths_cross_product(const float u[3], const float v[3], float out[3])
{
	out[0] = u[1] * v[2] - u[2] * v[1];
	out[1] = u[2] * v[0] - u[0] * v[2];
	out[2] = u[0] * v[1] - u[1] * v[0];
}


/**
 * \brief 		Computes the squared norm of a vector (dim 3)
 * 
 * \param 	u 	Input vector
 * \return 		Squared norm
 */
float static inline vectors_norm_sqr(const float u[3]) //maths_vector_norm_sqr(float u[3])
{
	float norm = vectors_scalar_product(u, u);
	return norm;
}


/**
 * \brief 		Computes the norm of a vector of dimension 3
 * 
 * \param 	u 	Input vector (dim 3)
 * \return 		Norm of the vector
 */

float static inline vectors_norm(const float u[3]) //maths_vector_norm(float u[3])
{
	return sqrt(vectors_norm_sqr(u));
}


/**
 * \brief 		Normalizes a vector of dimension 3
 * 
 * \param 	v 	Input vector (dim 3)
 * \param 	u 	Output vector (dim 3)
 */
 /*
void static inline vectors_normalize(const float v[3], float u[3]) //maths_vector_normalize(float v[3], float u[3])
{
	float norm = vectors_norm(v);
	int32_t i;
	for (i = 0; i < 3; ++i)
	{
		u[i] = v[i] / norm;
	}
}*/


#ifdef __cplusplus
}
#endif

#endif /* VECTORS_H_ */

