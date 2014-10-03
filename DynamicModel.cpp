//------------------------
//
// See article:
//     Modeling and Simulation of the LAUV Autonomus Underwater Vehicle
//     Author:
//     Jorge Estrela dea Silva, Bruno Terra, Ricardo Martins and Joao Borges de Sousa
//
//
#include <iostream>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/assignment.hpp>
#include "Matrix.cpp"
#include <fstream>

#include <boost/array.hpp>              // not used?
#include <boost/numeric/odeint.hpp>

using namespace std;
using namespace boost::numeric::odeint;

using namespace boost::numeric::ublas;

struct streaming_observer
{
     std::ostream &m_out;
     streaming_observer( std::ostream &out ) : m_out( out ) {}

     void operator()( const matrix<double> &x , double t ) const
     {
          m_out << t;
          for( size_t i=0 ; i < x.size1() ; ++i )
              m_out << "\t" << x(i,0);
          m_out << "\n";
     }
};


struct AUVModel
{
public:
    AUVModel() : matM(6, 6), invM(6,6), matL(6, 6), matC(6, 6), matD(6, 6), matg(6, 1), matA(6, 6), matU(6, 1), matTa(6, 1), matK(6, 5), matInput(5, 1)
    {
        // Initializing the matrixes needed for the Dynamic Model of the AUV
        m = 46.27, W = 453.5, B = 454.5;                                         // W - B should be approximately 1N
        Iy = 18.0, Iz = 18.0, Ix = 0.0;  // I values gotten from simulator see I0 matrix
        Xud = m - 19, Yvd = m - 34, Zwd = m - 34, Kpd = Ix - 0.04, Mqd = Iy - 2.1, Nrd = Iz - 2.1, zG = 0.1, ZG = zG;

        Xu = -2.4, Yv = -23, Zw = -23, Kp = -0.3, Mq = -9.7, Nr = -9.7, Yr = 11.5, Zq = -11.5, Mw = 3.1, Nv = -3.1;
        Xuu = -2.4, Yvv = -80, Zww = -80, Kpp = -0.0006, Mqq = -9.1, Nrr = -9.1, Zqq = -0.3, Yrr = 0.3, Mww = 3.1, Nvv = -3.1;

        seaDensity = 1.028, airfoilArea = 0.03*0.04, radiusBlade = 0.04, xRudder = 0.9674;         // airfoilArea, radiusBlade and xRudde are estimates. xRudder is the rudders x distance from center of the AUV.

        u0 = 2.0;
        v0 = 0.0;
        w0 = 0.0;
        p0 = 0.0;
        q0 = 0.0;
        r0 = 0.0;

        theta0 = 0.0;
        phi0   = 0.0;

        finUp = 0.0;
        finDown = 0.0;
        finStb = 0.0;
        finPort = 0.0;

        
        double initL[6*6] = {0, 0, 0, 0, 0, 0,
                            0, 30*u0, 0, 0, 0, -7.7*u0,
                            0, 0, 30*u0, 0, 7.7*u0, 0,
                            0, 0, 0, 0, 0, 0,
                            0, 0, 9.9*u0, 0, 3.1*u0, 0,
                            0, -9.9*u0, 0, 0, 0, 3.1*u0};

        insertToMatrix(matL, initL);

        double initM[6*6] = {m - Xud, 0, 0, 0, m*zG, 0,
                            0, m - Yvd, 0, -m*zG, 0, 0,
                            0, 0, m - Zwd, 0, 0, 0,
                            0, -(m*zG), 0, Ix - Kpd, 0, 0,
                            m*zG, 0, 0, 0, Iy - Mqd, 0,
                            0, 0, 0, 0, 0, Iz - Nrd};
        insertToMatrix(matM, initM);

        double initg[6*1] = {(W - B)*sin(theta0),
                            -(W - B)*cos(theta0)*sin(phi0),
                            -(W - B)*cos(theta0)*cos(phi0),
                            zG*W*cos(theta0)*sin(phi0),
                            zG*W*sin(theta0),
                            0};
        insertToMatrix(matg, initg);

        // ATH skoða betur gildin fyrir breytistærðinar í C
        double initC[6*6] = {0, 0, 0, m*zG*r0, (m-Zwd)*w0, -(m-Yvd)*v0,
                            0, 0, 0, -(m-Zwd)*w0, m*zG*r0, (m-Xud)*u0,
                            0, 0, 0, -m*ZG*p0+(m-Yvd)*v0, -m*ZG*q0-(m-Xud)*u0, 0,
                            -m*zG*r0, (m-Zwd)*w0, m*ZG*p0-(m-Yvd)*v0, 0, (Iz-Nrd)*r0, -(Iy-Mqd)*q0,
                            -(m-Zwd)*w0, -m*zG*r0, m*ZG*q0+(m-Xud)*u0, -(Iz-Nrd)*r0, 0, (Ix-Kpd)*p0,
                            (m-Yvd)*v0, -(m-Xud)*u0, 0, (Iy-Mqd)*q0, -(Ix-Kpd)*p0, 0};
        insertToMatrix(matC, initC);

        double initD[6*6] = {-Xu - Xuu*abs(u0), 0, 0, 0, 0, 0,
                            0, -Yv - Yvv*abs(v0), 0, 0, 0, -Yr - Yrr*abs(r0),
                            0, 0, -Zw - Zww*abs(w0), 0, -Zq - Zqq*abs(q0), 0,
                            0, 0, 0, -Kp - Kpp*abs(p0), 0, 0,
                            0, 0, -Mw - Mww*abs(w0), 0, -Mq - Mqq*abs(q0), 0,
                            0, -Nv - Nvv*abs(v0), 0, 0, 0, -Nr - Nrr*abs(r0)};
        insertToMatrix(matD, initD);

        InvertMatrix(matM, invM);

        matA = -prod(invM, matC + matD + matL);

        double fLift = 1/2*seaDensity*airfoilArea*radiusBlade*radiusBlade*2*3.1415;
        double initK[6*5] = {0.000095, 0, 0, 0, 0,                                  // K_prop = 0.000095 ~= 1/2*seaDesity*airfoilArea*r^2_blade*sin(attackAngle)
                             0, fLift, fLift, 0, 0,
                             0, 0, 0, fLift, fLift,
                             0, 0, 0, 0, 0,
                             0, 0, 0, -xRudder*fLift, -xRudder*fLift,
                             0, -xRudder*fLift, -xRudder*fLift, 0, 0};
        insertToMatrix(matK, initK);
        
        // probably not needed
        double initInput[5*1] = {0, 0, 0, 0, 0};
        insertToMatrix(matInput, initInput);
        
    }

    void input(double rpm, double finUp, double finDown, double finStb, double finPort)
    {
        matInput(0, 0) = 1;
        matInput(1, 0) = finUp;
        matInput(2, 0) = finDown;
        matInput(3, 0) = finStb;
        matInput(4, 0) = finPort;
        matInput = matInput*rpm*rpm;
    }

    matrix<double> getMatTa()
    {
        matTa = prod(matK, matInput);
        return matTa;
    }

    void operator() ( const matrix<double> &v , matrix<double> &dvdt , double t)
    {
        dvdt = prod(matA, v) + prod(invM, getMatTa() - matg);
    }

    boost::numeric::ublas::matrix<double> matM, invM, matL, matC, matD, matg, matA, matU, matTa, matK, matInput;

    double m, W, B;
    double Iy, Iz, Ix;
    double Xud, Yvd, Zwd, Kpd, Mqd, Nrd, zG, ZG;

    double Xu, Yv, Zw, Kp, Mq, Nr, Yr, Zq, Mw, Nv;
    double Xuu, Yvv, Zww, Kpp, Mqq, Nrr, Zqq, Yrr, Mww, Nvv;

    double seaDensity, airfoilArea, radiusBlade, xRudder;

    double u0;
    double v0;
    double w0;
    double p0;
    double q0;
    double r0;

    double theta0;
    double phi0;

    double finUp;
    double finDown;
    double finStb;
    double finPort;
};

int main(int argc, char **argv)
{
    double t = 0.0;
    const double dt = 0.1;
    AUVModel model = AUVModel();
    matrix<double> v(6, 1);
    double surge = 0.5;
    double initv[6*1] = {surge , 0.0 , 0.0 , 0.0, 0.0, 0.0};
    insertToMatrix(v, initv);

    ofstream file;
    file.open ("DynamicModel.csv");

    
    runge_kutta4< matrix<double> > rk4;
    file << "t" << ',' << "u" << ',' << "v" << ',' << "w" << ',' << "p" << ',' << "q" << ',' << "r" << endl;
    for( size_t i=0 ; i<30 ; ++i,t+=dt )
    {
        rk4.do_step(model , v , t , dt);
        file << t << ',' << v(0,0) << ',' << v(1,0) << ',' << v(2,0) << ',' << v(3,0) << ',' << v(4,0) << ',' << v(5,0) << endl;
        model.input(surge/0.0025, 0, 0, 0, 0);
    }
    file.close();

    
    cout << "M" << endl;
    printMatrix(model.matM);
    cout << endl;

    cout << "C" << endl;
    printMatrix(model.matC);
    cout << endl;

    cout << "D" << endl;
    printMatrix(model.matD);
    cout << endl;

    cout << "L" << endl;
    printMatrix(model.matL);
    cout << endl;



    cout << "A" << endl;
    printMatrix(model.matA);
    cout << endl;

    cout << "invM" << endl;
    printMatrix(model.invM);
    cout << endl;

    cout << "g" << endl;
    printMatrix(model.matg);
    cout << endl;

    cout << "K" << endl;
    printMatrix(model.matK);
    cout << endl;

    cout << "input" << endl;
    printMatrix(model.matInput);
    cout << endl;
    

    return 0;
}






