#include <path_planner/polyfit.hpp>
#include "math.h"
#include "path_planner/path_planner.h"


poly::poly()
{

}
/*
    Finds the coefficients of a polynomial p(x) of degree n that fits the data,
    p(x(i)) to y(i), in a least squares sense. doublehe result p is a row vector of
    length n+1 containing the polynomial coefficients in incremental powers.

    param:
        oX				x axis values
        oY				y axis values
        nDegree			polynomial degree including the constant

    return:
        coefficients of a polynomial starting at the constant coefficient and
        ending with the coefficient of power to nDegree. C++0x-compatible
        compilers make returning locally created vectors very efficient.

*/

void poly::polyfit(int nDegree )
{
    if(x_raw_pts.size() < nDegree)
        return;

    using namespace boost::numeric::ublas;

    if ( x_raw_pts.size() != y_raw_pts.size() )
        throw std::invalid_argument( "X and Y vector sizes do not match" );

    // more intuative this way
    nDegree++;

    size_t nCount =  x_raw_pts.size();
    matrix<float> oXMatrix( nCount, nDegree );
    matrix<float> oYMatrix( nCount, 1 );

    // copy y matrix
    for ( size_t i = 0; i < nCount; i++ )
    {
        oYMatrix(i, 0) = y_raw_pts[i];
    }

    // create the X matrix
    for ( size_t nRow = 0; nRow < nCount; nRow++ )
    {
        float nVal = 1.0f;
        for ( int nCol = 0; nCol < nDegree; nCol++ )
        {
            oXMatrix(nRow, nCol) = nVal;
            nVal *= x_raw_pts[nRow];
        }
    }
    // transpose X matrix
    matrix<float> oXtMatrix( trans(oXMatrix) );
    // multiply transposed X matrix with X matrix
    matrix<float> oXtXMatrix( prec_prod(oXtMatrix, oXMatrix) );
    // multiply transposed X matrix with Y matrix
    matrix<float> oXtYMatrix( prec_prod(oXtMatrix, oYMatrix) );

    // lu decomposition
    permutation_matrix<int> pert(oXtXMatrix.size1());
    const std::size_t singular = lu_factorize(oXtXMatrix, pert);

    // must be singular
    BOOST_ASSERT( singular == 0 );

    // backsubstitution
    lu_substitute(oXtXMatrix, pert, oXtYMatrix);

    // copy the result to coeff

    std::vector<float>vec( oXtYMatrix.data().begin(), oXtYMatrix.data().end() );
    coeff=vec;
}

float poly::polyval(float x)
{
    size_t nDegree = coeff.size();
    float output;

    double nY = 0;
    double nXdouble = 1;
    double nX = x;

    for ( size_t j = 0; j < nDegree; j++ )
    {
        // multiply current x by a coefficient
        nY += coeff[j] * nXdouble;
        // power up the X
        nXdouble *= nX;
    }
    output = nY;
    return nY;
}

void poly::fit_middle(poly left, poly right, int degree)
{
    this->x_raw_pts.clear();
    this->y_raw_pts.clear();
    float avg = 0;

    if(left.x_raw_pts.empty() || right.x_raw_pts.empty())
        return;

    float min_size = 0;
    float left_size = left.x_raw_pts[left.x_raw_pts.size()-1];
    float right_size = right.x_raw_pts[right.x_raw_pts.size()-1];

    if(left_size>=right_size)
        min_size = right_size;
    else
        min_size = left_size;

    for(int i = 0;i<(int)min_size;i+=4)
    {
        avg = (left.polyval(i) - right.polyval(i))/2 + right.polyval(i);
        this->x_raw_pts.push_back(i);
        this->y_raw_pts.push_back(avg);
    }

    this->polyfit(degree);
}

std_msgs::Float64 poly::get_pos_offset(float x, float y)
{
    std_msgs::Float64 message;
    message.data = y - polyval(x);

    return message;
}

//TANGENT METHODS/////////////////////
tangent::tangent(float a,float b)
{
    this->coeff[1] = a;
    this->coeff[0] = b;
}

void tangent::calc_coeff(poly polynom,float x)
{
    double derive = 0;
    int degree = polynom.coeff.size();
    double nXdouble = 1;
    double nX = x;

    for ( size_t j = 1; j < degree; j++ )
    {
        // multiply current x by a coefficient
        derive += j*polynom.coeff[j] * nXdouble;
        // power up the X
        nXdouble *= nX;
    }

    this->coeff[1] = derive; //a
    this->coeff[0] = - derive*x+polynom.polyval(x); //b
}

void tangent::set_coeff(float a, float b)
{
    this->coeff[0] = b;
    this->coeff[1] = a;
}

std_msgs::Float64 tangent::get_head_offset(tangent tg)
{
    std_msgs::Float64 message;
    message.data = atan((this->coeff[1]-tg.coeff[1])/(1+this->coeff[1]*tg.coeff[1]));
    return message;

}
void poly::adjust(poly good_poly)
{
    this->coeff = good_poly.coeff;
    float avg = 0;
    float sum = 0;

    for(int i = 0;i<this->y_raw_pts.size();i++)
    {
        sum+=y_raw_pts[i];
    }

    avg = sum/y_raw_pts.size();

    float est_sum = 0;
    float est = 0;

    for(int i = 0;i<this->y_raw_pts.size();i++)
    {
        est_sum+=(y_raw_pts[i] - avg)*(y_raw_pts[i] - avg);
    }

    est = est_sum/y_raw_pts.size();

    float est_inc = y_raw_pts[0] - est;
    float est_max = y_raw_pts[0] + est;

    float sum_error = 0;
    float min_error = (this->polyval(x_raw_pts[0]) - y_raw_pts[0])*(this->polyval(x_raw_pts[0]) - y_raw_pts[0]);
    float offset;

    for(est_inc;est_inc<est_max;est_inc+=1)
    {
        this->coeff[0] = est_inc;
        sum_error = 0;
        for(int i = 0;i<this->x_raw_pts.size();i++)
        {
            sum_error += (this->polyval(x_raw_pts[i]) - y_raw_pts[i])*(this->polyval(x_raw_pts[i]) - y_raw_pts[i]);
        }

        if(sum_error<min_error)
        {
            min_error = sum_error;
            offset = est_inc;
        }

    }
    this->coeff[0] = offset;
}

void poly::get_coeff(const std::vector<float> coeff_vec)
{
    this->coeff = coeff_vec;
}
