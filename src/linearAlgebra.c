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

#include "board.h"

///////////////////////////////////////////////////////////////////////////////
//  Vector Dot Product
//  Return the Dot product of vectors a and b with length m
//
//  Call as: vectorDotProduct(m, a, b)
///////////////////////////////////////////////////////////////////////////////

float vectorDotProduct(int length, float vector1[], float vector2[])
{
  float dotProduct = 0.0f;
  int   i;

  for (i = 0; i < length; i++)
  {
  dotProduct += vector1[i] * vector2[i];
  }

  return dotProduct;
}

///////////////////////////////////////////////////////////////////////////////
//  Vector Cross Product
//  Compute the cross product of vectors a and b with length 3
//  Place result in vector C
//
//  Call as: vectorDotProduct(c, a, b)
///////////////////////////////////////////////////////////////////////////////

void vectorCrossProduct(float vectorC[3], float vectorA[3], float vectorB[3])
{
  vectorC[0] = (vectorA[1] * vectorB[2]) - (vectorA[2] * vectorB[1]);
  vectorC[1] = (vectorA[2] * vectorB[0]) - (vectorA[0] * vectorB[2]);
  vectorC[2] = (vectorA[0] * vectorB[1]) - (vectorA[1] * vectorB[0]);
}

///////////////////////////////////////////////////////////////////////////////
//  Multiply a vector by a scalar
//  Mulitply vector a with length m by a scalar
//  Place result in vector b
//
//  Call as: vectorScale(m, b, a, scalar)
///////////////////////////////////////////////////////////////////////////////

void vectorScale(int length, float scaledVector[], float inputVector[], float scalar)
{
  uint8_t i;

  for (i = 0; i < length; i++)
  {
   scaledVector[i] = inputVector[i] * scalar;
  }
}

///////////////////////////////////////////////////////////////////////////////
//  Compute sum of 2 vectors
//  Add vector a to vector b, both of length m
//  Place result in vector c
//
//  Call as: vectorAdd(m, c, b, a)
///////////////////////////////////////////////////////////////////////////////

void vectorAdd(int length, float vectorC[], float vectorA[], float vectorB[])
{
  uint8_t i;

  for(i = 0; i < length; i++)
  {
     vectorC[i] = vectorA[i] + vectorB[i];
  }
}

///////////////////////////////////////////////////////////////////////////////
//  Compute difference of 2 vectors
//  Subtract vector a from vector b, both of length m
//  Place result in vector c
//
//  Call as: vectorSubtract(m, c, b, a)
///////////////////////////////////////////////////////////////////////////////

void vectorSubtract(int length, float vectorC[], float vectorA[], float vectorB[])
{
  uint8_t i;

  for(i = 0; i < length; i++)
  {
     vectorC[i] = vectorA[i] - vectorB[i];
  }
}

///////////////////////////////////////////////////////////////////////////////
//  Matrix Multiply
//  Multiply matrix A times matrix B, matrix A dimension m x n, matrix B dimension n x p
//  Result placed in matrix C, dimension m x p
//
//  Call as: matrixMultiply(m, n, p, C, A, B)
///////////////////////////////////////////////////////////////////////////////

void matrixMultiply(int aRows, int aCols_bRows, int bCols, float matrixC[], float matrixA[], float matrixB[])
{
  uint8_t i, j, k;

  for (i = 0; i < aRows * bCols; i++)
  {
    matrixC[i] = 0.0;
  }

  for (i = 0; i < aRows; i++)
  {
    for(j = 0; j < aCols_bRows; j++)
    {
      for(k = 0;  k < bCols; k++)
      {
       matrixC[i * bCols + k] += matrixA[i * aCols_bRows + j] * matrixB[j * bCols + k];
      }
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
//  Matrix Addition
//  Add matrix A to matrix B, dimensions m x n
//  Result placed in matrix C, dimension m x n
//
//  Call as: matrixAdd(m, n, C, A, B)
///////////////////////////////////////////////////////////////////////////////

void matrixAdd(int rows, int cols, float matrixC[], float matrixA[], float matrixB[])
{
  uint8_t i;

  for (i = 0; i < rows * cols; i++)
  {
    matrixC[i] = matrixA[i] + matrixB[i];
  }
}

///////////////////////////////////////////////////////////////////////////////
//  Matrix Subtraction
//  Subtract matrix A from matrix B, dimensions m x n
//  Result placed in matrix C, dimension m x n
//
//  Call as: matrixSubtract(m, n, C, A, B)
///////////////////////////////////////////////////////////////////////////////

void matrixSubtract(int rows, int cols, float matrixC[], float matrixA[], float matrixB[])
{
  uint8_t i;

  for (i = 0; i < rows * cols; i++)
  {
    matrixC[i] = matrixA[i] - matrixB[i];
  }
}


///////////////////////////////////////////////////////////////////////////////
//  Matrix Scaling
//  Scale matrix A, dimensions m x n, by a scaler, S
//  Result placed in matrix C, dimension m x n
//
//  Call as: matrixScale(m, n, C, S, B)
///////////////////////////////////////////////////////////////////////////////

void matrixScale(int rows, int cols, float matrixC[], float scaler, float matrixA[])
{
  uint8_t i;

  for (i = 0; i < rows * cols; i++)
  {
    matrixC[i] = scaler * matrixA[i];
  }
}

///////////////////////////////////////////////////////////////////////////////
//  3 x 3 Matrix Transpose
//  Compute 3 x 3 Transpose of A
//  Result placed in matrix C, dimension 3 x 3
//
//  Call as: Transpose3x3(C, A)
///////////////////////////////////////////////////////////////////////////////

void matrixTranspose3x3(float matrixC[9], float matrixA[9])
{
  matrixC[0] = matrixA[0];
  matrixC[1] = matrixA[3];
  matrixC[2] = matrixA[6];
  matrixC[3] = matrixA[1];
  matrixC[4] = matrixA[4];
  matrixC[5] = matrixA[7];
  matrixC[6] = matrixA[2];
  matrixC[7] = matrixA[5];
  matrixC[8] = matrixA[8];
}

///////////////////////////////////////////////////////////////////////////////
//  3 x 3 Matrix Inverse
//  Compute 3 x 3 Inverse of A
//  Result placed in matrix C, dimension 3 x 3
//
//  Call as: Inverse3x3(C, A)
///////////////////////////////////////////////////////////////////////////////

void matrixInverse3x3(float matrixC[9], float matrixA[9])
{

  float det;
  float transposeA[9];
  float minors[9];
  float transposeMinors[9];

  det = matrixA[0] * (matrixA[4] * matrixA[8] - matrixA[5] * matrixA[7]) -
        matrixA[1] * (matrixA[3] * matrixA[8] - matrixA[5] * matrixA[6]) +
        matrixA[2] * (matrixA[3] * matrixA[7] - matrixA[4] * matrixA[6]);

  matrixTranspose3x3(transposeA, matrixA);

  minors[0] = matrixA[4] * matrixA[8] - matrixA[5] * matrixA[7];
  minors[1] = matrixA[5] * matrixA[6] - matrixA[3] * matrixA[8];
  minors[2] = matrixA[3] * matrixA[7] - matrixA[4] * matrixA[6];
  minors[3] = matrixA[2] * matrixA[7] - matrixA[1] * matrixA[8];
  minors[4] = matrixA[0] * matrixA[8] - matrixA[2] * matrixA[6];
  minors[5] = matrixA[1] * matrixA[6] - matrixA[0] * matrixA[7];
  minors[6] = matrixA[1] * matrixA[5] - matrixA[2] * matrixA[4];
  minors[7] = matrixA[2] * matrixA[3] - matrixA[0] * matrixA[5];
  minors[8] = matrixA[0] * matrixA[4] - matrixA[1] * matrixA[3];

  matrixTranspose3x3(transposeMinors, minors);

  det = 1/det;

  matrixScale(3,3, matrixC, det, transposeMinors);
}


///////////////////////////////////////////////////////////////////////////////
