// Taken from -> http://savingyoutime.wordpress.com/2009/09/21/c-matrix-inversion-boostublas/

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/lu.hpp>

using namespace boost::numeric::ublas;

 /* Matrix inversion routine.
 Uses lu_factorize and lu_substitute in uBLAS to invert a matrix */
template<class T>
bool InvertMatrix(const matrix<T>& input, matrix<T>& inverse)
{
	typedef permutation_matrix<std::size_t> pmatrix;

	// create a working copy of the input
	matrix<T> A(input);

	// create a permutation matrix for the LU-factorization
	pmatrix pm(A.size1());

	// perform LU-factorization
	int res = lu_factorize(A, pm);
	if (res != 0)
		return false;

	// create identity matrix of "inverse"
	inverse.assign(identity_matrix<T> (A.size1()));

	// backsubstitute to get the inverse
	lu_substitute(A, pm, inverse);

	return true;
}

void insertToMatrix(matrix<double>& mat, double vec[])
{
    int idx = 0;
    for(unsigned int i = 0; i < mat.size1(); i++)
        for(unsigned int j = 0; j < mat.size2(); j++)
        {
            mat(i,j) = vec[idx];
            idx++;
        }
}

void printMatrix(matrix<double>& mat)
{
    for(unsigned int i = 0; i < mat.size1(); i++) 
    {
        for(unsigned int j = 0; j < mat.size2(); j++)
        {
            std::cout << mat(i,j) << "\t";
        }
        std::cout << "\n";
    }
}

