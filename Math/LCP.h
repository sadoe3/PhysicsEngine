//
//	LCP.h
//
#pragma once
#include "Vector.h"
#include "Matrix.h"


/*
====================================================
Solve_LCP_GaussSeidel
====================================================
*/
VecN Solve_LCP_GaussSeidel( const MatN & A, const VecN & b );
