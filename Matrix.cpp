// Taken from -> http://savingyoutime.wordpress.com/2009/09/21/c-matrix-inversion-boostublas/

#include <boost/numeric/ublas/matrix.hpp>       // this uses the "old boost" boost_1_31_0
#include <boost/numeric/ublas/lu.hpp>

 /* Matrix inversion routine.
  * Uses lu_factorize and lu_substitute in uBLAS to invert a matrix */
template<class T>
bool InvertMatrix(const boost::numeric::ublas::matrix<T>& input, boost::numeric::ublas::matrix<T>& inverse)
{
	typedef boost::numeric::ublas::permutation_matrix<std::size_t> pmatrix;

	// create a working copy of the input
	boost::numeric::ublas::matrix<T> A(input);

	// create a permutation matrix for the LU-factorization
	pmatrix pm(A.size1());

	// perform LU-factorization
	int res = lu_factorize(A, pm);
	if (res != 0)
		return false;

	// create identity matrix of "inverse"
	inverse.assign(boost::numeric::ublas::identity_matrix<T> (A.size1()));

	// backsubstitute to get the inverse
	boost::numeric::ublas::lu_substitute(A, pm, inverse);

	return true;
}

