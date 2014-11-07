//------------------------
//
// See article:
//     Modeling and Simulation of the LAUV Autonomus Underwater Vehicle
//     Author:
//     Jorge Estrela dea Silva, Bruno Terra, Ricardo Martins and Joao Borges de Sousa
//
//
#include <iostream>
#include <boost/numeric/odeint.hpp>         // this uses the "new boost" boost_1_56_0
#include "DynamicModel.h"
#include "OdeintIntegrator.h"

#include <boost/numeric/ublas/io.hpp>

void printMatrix(boost::numeric::ublas::matrix<double>& mat)
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


AUVModel* AUVModel::create(AUVModel*& model, double initState[], double initTime, double dt)
{
    return new OdeintIntegrator(model, initState, initTime , dt);
}

void AUVModel::dostep(AUVModel*& model)
{
    std::cout << "AUVModel::dostep, not the correct step though" << std::endl;
}

void AUVModel::initializeSate(double state[])
{
    insertToMatrix(this->state, state);
}


void AUVModel::doWork()
{
    //integrator->dostep(this);
    // sum up current state
}

//double initState[]=0, double initTime=0, double dt=0
AUVModel::AUVModel() : matM(6, 6), invM(6,6), matL(6, 6), matC(6, 6), matD(6, 6), matg(6, 1), matA(12, 12), matA11(6, 6), matA12(6, 6), matB(12, 6), matTa(6, 1), matK(6, 5),
matInput(5, 1), matJ(6, 6), position(6, 1), matG(6, 6), state(12, 1)
{
    //insertToMatrix(state, initState);
    //integrator = new OdeintIntegrator(this, initState, initTime , dt);

    // Initializing the matrixes needed for the Dynamic Model of the AUV
    m = 46.27, W = 453.5, B = 454.5;                                         // W - B should be approximately 1N
    Iy = 18.0, Iz = 18.0, Ix = 0.0;  // I values gotten from simulator see I0 matrix
    Xud = m - 19, Yvd = m - 34, Zwd = m - 34, Kpd = Ix - 0.04, Mqd = Iy - 2.1, Nrd = Iz - 2.1, zG = 0.1, ZG = zG;

    Xu = -2.4, Yv = -23, Zw = -23, Kp = -0.3, Mq = -9.7, Nr = -9.7, Yr = 11.5, Zq = -11.5, Mw = 3.1, Nv = -3.1;
    Xuu = -2.4, Yvv = -80, Zww = -80, Kpp = -0.0006, Mqq = -9.1, Nrr = -9.1, Zqq = -0.3, Yrr = 0.3, Mww = 3.1, Nvv = -3.1;

    seaDensity = 1.028, airfoilArea = 0.03*0.04, radiusBlade = 0.04, xRudder = 0.9674;         // airfoilArea, radiusBlade and xRudde are estimates. xRudder is the rudders x distance from center of the AUV.

    u = 2.0;
    v = 0.0;
    w = 0.0;
    p = 0.0;
    q = 0.0;
    r = 0.0;

    x     = 0.0;
    y     = 0.0;
    z     = 0.0;
    phi   = 0.0;
    theta = 0.0;
    psi   = 0.0;

    finUp = 0.0;
    finDown = 0.0;
    finStb = 0.0;
    finPort = 0.0;

        
    double initL[6*6] = {0, 0, 0, 0, 0, 0,
                        0, 30*u, 0, 0, 0, -7.7*u,
                        0, 0, 30*u, 0, 7.7*u, 0,
                        0, 0, 0, 0, 0, 0,
                        0, 0, 9.9*u, 0, 3.1*u, 0,
                        0, -9.9*u, 0, 0, 0, 3.1*u};

    insertToMatrix(matL, initL);

    double initM[6*6] = {m - Xud, 0, 0, 0, m*zG, 0,
                        0, m - Yvd, 0, -m*zG, 0, 0,
                        0, 0, m - Zwd, 0, 0, 0,
                        0, -(m*zG), 0, Ix - Kpd, 0, 0,
                        m*zG, 0, 0, 0, Iy - Mqd, 0,
                        0, 0, 0, 0, 0, Iz - Nrd};
    insertToMatrix(matM, initM);

    // TODO: Use Thor Fossens G matrix
    /*
    double initg[6*1] = {(W - B)*sin(theta),
                        -(W - B)*cos(theta)*sin(phi),
                        -(W - B)*cos(theta)*cos(phi),
                        zG*W*cos(theta)*sin(phi),
                        zG*W*sin(theta),
                        0};
    insertToMatrix(matg, initg);
    */

    // This G matrix is from Guidance and Control of Ocean Vehicles, Thor I. Fossen on page 101.
    // Set zB = xG = xB = yG = yB = 0
    double initG[6*6] = {0, 0, 0,   0,  W-B, 0,
                        0, 0, 0, -(W-B), 0,  0,
                        0, 0, 0,    0,   0,  0,
                        0, 0, 0, zG*W,   0,  0,
                        0, 0, 0,    0, zG*W, 0,
                        0, 0, 0,    0,   0,  0};
    insertToMatrix(matG, initG);

    // ATH skoða betur gildin fyrir breytistærðinar í C
    double initC[6*6] = {0, 0, 0, m*zG*r, (m-Zwd)*w, -(m-Yvd)*v,
                        0, 0, 0, -(m-Zwd)*w, m*zG*r, (m-Xud)*u,
                        0, 0, 0, -m*ZG*p+(m-Yvd)*v, -m*ZG*q-(m-Xud)*u, 0,
                        -m*zG*r, (m-Zwd)*w, m*ZG*p-(m-Yvd)*v, 0, (Iz-Nrd)*r, -(Iy-Mqd)*q,
                        -(m-Zwd)*w, -m*zG*r, m*ZG*q+(m-Xud)*u, -(Iz-Nrd)*r, 0, (Ix-Kpd)*p,
                        (m-Yvd)*v, -(m-Xud)*u, 0, (Iy-Mqd)*q, -(Ix-Kpd)*p, 0};
    insertToMatrix(matC, initC);

    double initD[6*6] = {-Xu - Xuu*abs(u), 0, 0, 0, 0, 0,
                        0, -Yv - Yvv*abs(v), 0, 0, 0, -Yr - Yrr*abs(r),
                        0, 0, -Zw - Zww*abs(w), 0, -Zq - Zqq*abs(q), 0,
                        0, 0, 0, -Kp - Kpp*abs(p), 0, 0,
                        0, 0, -Mw - Mww*abs(w), 0, -Mq - Mqq*abs(q), 0,
                        0, -Nv - Nvv*abs(v), 0, 0, 0, -Nr - Nrr*abs(r)};
    insertToMatrix(matD, initD);

    InvertMatrix(matM, invM);

    matA11 = -prod(invM, matC + matD + matL);
    matA12 = -prod(invM, matG);

    double fLift = 1/2*seaDensity*airfoilArea*radiusBlade*radiusBlade*2*3.1415;
    double initK[6*5] = {0.000095, 0, 0, 0, 0,                                  // K_prop = 0.000095 ~= 1/2*seaDesity*airfoilArea*r^2_blade*sin(attackAngle)
                         0, fLift, fLift, 0, 0,
                         0, 0, 0, fLift, fLift,
                         0, 0, 0, 0, 0,
                         0, 0, 0, -xRudder*fLift, -xRudder*fLift,
                         0, -xRudder*fLift, -xRudder*fLift, 0, 0};
    insertToMatrix(matK, initK);
    
    double initJ[6*6] = {cos(psi)*cos(theta), cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi), sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta), 0, 0, 0,
                         sin(psi)*cos(theta), cos(psi)*cos(phi)+sin(phi)*sin(theta)*sin(psi), sin(theta)*sin(psi)*cos(phi)-cos(psi)*sin(phi), 0, 0, 0,
                        -sin(theta)         , cos(theta)*sin(phi)                           , cos(theta)*cos(phi)                           , 0, 0, 0,
                         0, 0, 0, 1, sin(phi)*tan(theta), cos(phi)*tan(theta),
                         0, 0, 0, 0, cos(phi)           , -sin(phi),
                         0, 0, 0, 0, sin(phi)/cos(theta), cos(phi)/cos(theta) };
    insertToMatrix(matJ, initJ);


    subrange(matA, 0,6, 0,6) = matA11;
    subrange(matA, 0,6, 6,12) = matA12;
    subrange(matA, 6,12, 0,6) = matJ;
    subrange(matB, 0,6, 0,6) = invM;

    // probably not needed/The user should start with an input.
    double initInput[5*1] = {0, 0, 0, 0, 0};
    insertToMatrix(matInput, initInput);
        
}

void AUVModel::input(double rpm, double finUp, double finDown, double finStb, double finPort)
{
    matInput(0, 0) = 1;
    matInput(1, 0) = finUp;
    matInput(2, 0) = finDown;
    matInput(3, 0) = finStb;
    matInput(4, 0) = finPort;
    matInput = matInput*rpm*rpm;
}

boost::numeric::ublas::matrix<double> AUVModel::getMatTa()
{
    matTa = prod(matK, matInput);
    return matTa;
}

boost::numeric::ublas::matrix<double> AUVModel::getMatJ()
{
    u = state(0, 0);
    v = state(1, 0);
    w = state(2, 0);
    p = state(3, 0);
    q = state(4, 0);
    r = state(5, 0);
    matJ(0, 0) = cos(psi)*cos(theta); matJ(0, 1) = cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi); matJ(0, 2) = sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta);
    matJ(1, 0) = sin(psi)*cos(theta); matJ(1, 1) = cos(psi)*cos(phi)+sin(phi)*sin(theta)*sin(psi); matJ(1, 2) = sin(theta)*sin(psi)*cos(phi)-cos(psi)*sin(phi);
    matJ(2, 0) = -sin(theta);         matJ(2, 1) = cos(theta)*sin(phi);                            matJ(2, 2) = cos(theta)*cos(phi);
        matJ(3, 3) = 1;                   matJ(3, 4) = sin(phi)*tan(theta);                            matJ(3, 5) = cos(phi)*tan(theta);
        matJ(4, 3) = 0;                   matJ(4, 4) = cos(phi);                                       matJ(4, 5) = -sin(phi);
        matJ(5, 3) = 0;                   matJ(5, 4) = sin(phi)/cos(theta);                            matJ(5, 5) = cos(phi)/cos(theta);
    return matJ;
}

void AUVModel::operator() ( const boost::numeric::ublas::matrix<double>& x , boost::numeric::ublas::matrix<double> &dxdt , double t)
{
    dxdt = prod(matA, x) + prod(matB, getMatTa());//prod(matA11, x) + prod(invM, getMatTa() - matg);
}

struct streaming_observer
{
     std::ostream &m_out;
     streaming_observer( std::ostream &out ) : m_out( out ) {}

     void operator()( const boost::numeric::ublas::matrix<double> &x , double t ) const
     {
          m_out << t;
          for( size_t i=0 ; i < x.size1() ; ++i )
              m_out << "\t" << x(i,0);
          m_out << "\n";
     }
};

void insertToMatrix(boost::numeric::ublas::matrix<double>& mat, double vec[])
{
    int idx = 0;
    for(unsigned int i = 0; i < mat.size1(); i++)
        for(unsigned int j = 0; j < mat.size2(); j++)
        {
            mat(i,j) = vec[idx];
            idx++;
        }
}





