//
//	LCP.cpp
//
#include "PCH.h"
#include "LCP.h"

/*
====================================================
Solve_LCP_GaussSeidel
====================================================
*/
VecN Solve_LCP_GaussSeidel( const MatN & A, const VecN & b ) {
	const int N = b.N;
	VecN x( N );
	x.Zero();

	for ( int iter = 0; iter < N; iter++ ) {
		for ( int i = 0; i < N; i++ ) {
			float dx = ( b[ i ] - A.rows[ i ].Dot( x ) ) / A.rows[ i ][ i ];
			if ( dx * 0.0f == dx * 0.0f ) {
				x[ i ] = x[ i ] + dx;
			}
		}
	}
	return x;
}
