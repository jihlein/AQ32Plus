/*
  October 2012

  aq32Plus Rev -

  Copyright (c) 2012 John Ihlein.  All rights reserved.

  Open Source STM32 Based Multicopter Controller Software

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)S.O.H. Madgwick
  6)UAVX

  Designed to run on the AQ32 Flight Control Board

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

///////////////////////////////////////////////////////////////////////////////

#pragma once

///////////////////////////////////////////////////////////////////////////////
//  Vector Dot Product
//  Return the Dot product of vectors a and b with length m
//
//  Call as: vectorDotProduct(m, a, b)
///////////////////////////////////////////////////////////////////////////////

float vectorDotProduct(int length, float vector1[], float vector2[]);

///////////////////////////////////////////////////////////////////////////////
//  Vector Cross Product
//  Compute the cross product of vectors a and b with length 3
//  Place result in vector C
//
//  Call as: vectorDotProduct(c, a, b)
///////////////////////////////////////////////////////////////////////////////

void vectorCrossProduct(float vectorC[3], float vectorA[3], float vectorB[3]);

///////////////////////////////////////////////////////////////////////////////
//  Multiply a vector by a scalar
//  Mulitply vector a with length m by a scalar
//  Place result in vector b
//
//  Call as: vectorScale(m, b, a, scalar)
///////////////////////////////////////////////////////////////////////////////

void vectorScale(int length, float scaledVector[], float inputVector[], float scalar);

///////////////////////////////////////////////////////////////////////////////
//  Compute sum of 2 vectors
//  Add vector a to vector b, both of length m
//  Place result in vector c
//
//  Call as: vectorAdd(m, c, b, a)
///////////////////////////////////////////////////////////////////////////////

void vectorAdd(int length, float vectorC[], float vectorA[], float vectorB[]);

///////////////////////////////////////////////////////////////////////////////
//  Compute difference of 2 vectors
//  Subtract vector a from vector b, both of length m
//  Place result in vector c
//
//  Call as: vectorSubtract(m, c, b, a)
///////////////////////////////////////////////////////////////////////////////

void vectorSubtract(int length, float vectorC[], float vectorA[], float vectorB[]);

///////////////////////////////////////////////////////////////////////////////
//  Matrix Multiply
//  Multiply matrix A times matrix B, matrix A dimension m x n, matrix B dimension n x p
//  Result placed in matrix C, dimension m x p
//
//  Call as: matrixMultiply(m, n, p, C, A, B)
///////////////////////////////////////////////////////////////////////////////

void matrixMultiply(int aRows, int aCols_bRows, int bCols, float matrixC[], float matrixA[], float matrixB[]);

///////////////////////////////////////////////////////////////////////////////
//  Matrix Addition
//  Add matrix A to matrix B, dimensions m x n
//  Result placed in matrix C, dimension m x n
//
//  Call as: matrixAdd(m, n, C, A, B)
///////////////////////////////////////////////////////////////////////////////

void matrixAdd(int rows, int cols, float matrixC[], float matrixA[], float matrixB[]);

///////////////////////////////////////////////////////////////////////////////
//  Matrix Subtraction
//  Subtract matrix A from matrix B, dimensions m x n
//  Result placed in matrix C, dimension m x n
//
//  Call as: matrixSubtract(m, n, C, A, B)
///////////////////////////////////////////////////////////////////////////////

void matrixSubtract(int rows, int cols, float matrixC[], float matrixA[], float matrixB[]);


///////////////////////////////////////////////////////////////////////////////
//  Matrix Scaling
//  Scale matrix A, dimensions m x n, by a scaler, S
//  Result placed in matrix C, dimension m x n
//
//  Call as: matrixScale(m, n, C, S, B)
///////////////////////////////////////////////////////////////////////////////

void matrixScale(int rows, int cols, float matrixC[], float scaler, float matrixA[]);

///////////////////////////////////////////////////////////////////////////////
//  3 x 3 Matrix Transpose
//  Compute 3 x 3 Transpose of A
//  Result placed in matrix C, dimension 3 x 3
//
//  Call as: Transpose3x3(C, A)
///////////////////////////////////////////////////////////////////////////////

void matrixTranspose3x3(float matrixC[9], float matrixA[9]);

///////////////////////////////////////////////////////////////////////////////
//  3 x 3 Matrix Inverse
//  Compute 3 x 3 Inverse of A
//  Result placed in matrix C, dimension 3 x 3
//
//  Call as: Inverse3x3(C, A)
///////////////////////////////////////////////////////////////////////////////

void matrixInverse3x3(float matrixC[9], float matrixA[9]);


///////////////////////////////////////////////////////////////////////////////