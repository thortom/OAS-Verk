#ifndef DYNAMIC_MODEL_H
#define DYNAMIC_MODEL_H

#include <iostream>
#include <fstream>

#include <boost/numeric/ublas/matrix.hpp>

#include "Matrix.cpp"                   // .h

class OdeintIntegrator;

class AUVModel
{
public:
    AUVModel();
    //AUVModel(const AUVModel& test) {std::cout << "hello" << std::endl;}
    virtual ~AUVModel() {/*TODO*/}

    void doWork();

    virtual void dostep(AUVModel*& model);
    AUVModel* create(AUVModel*& model, double initState[], double initTime, double dt);
    void initializeSate(double state[]);

    void input(double rpm, double finUp, double finDown, double finStb, double finPort);

    boost::numeric::ublas::matrix<double> getMatTa();
    boost::numeric::ublas::matrix<double> getMatU();
    boost::numeric::ublas::matrix<double> getMatJ();

    void operator() (const boost::numeric::ublas::matrix<double>& x , boost::numeric::ublas::matrix<double> &dxdt , double t);

    // get functions for state components
    double surge() {return state(0,0);}
    double sway() {return state(1,0);}
    double heave() {return state(2,0);}
    double roll() {return state(3,0);}
    double pitch() {return state(4,0);}
    double yaw() {return state(5,0);}

    // get functions for position components
    double xPos()     {return state(6,0);}
    double yPos()     {return state(7,0);}
    double zPos()     {return state(8,0);}
    double phiRads()   {return state(9,0);}
    double thetaRads() {return state(10,0);}
    double psiRads()   {return state(11,0);}

    boost::numeric::ublas::matrix<double> matM, invM, matL, matC, matD, matg, matA, matA11, matA12, matB, matTa, matK, matInput, matJ, n, matG, state;
    boost::numeric::ublas::matrix<double> position;

    AUVModel* integrator;

    double m, W, B;
    double Iy, Iz, Ix;
    double Xud, Yvd, Zwd, Kpd, Mqd, Nrd, zG, ZG;

    double Xu, Yv, Zw, Kp, Mq, Nr, Yr, Zq, Mw, Nv;
    double Xuu, Yvv, Zww, Kpp, Mqq, Nrr, Zqq, Yrr, Mww, Nvv;

    double seaDensity, airfoilArea, radiusBlade, xRudder;

    double u;
    double v;
    double w;
    double p;
    double q;
    double r;

    double x;
    double y;
    double z;
    double phi;
    double theta;
    double psi;

    double finUp;
    double finDown;
    double finStb;
    double finPort;
};

void insertToMatrix(boost::numeric::ublas::matrix<double>& mat, double vec[]);


#endif
