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
    void initializeSate(double velocity[]);

    void input(double rpm, double finUp, double finDown, double finStb, double finPort);

    boost::numeric::ublas::matrix<double> getMatTa();
    boost::numeric::ublas::matrix<double> getMatJ();

    void operator() (const boost::numeric::ublas::matrix<double>& velocity , boost::numeric::ublas::matrix<double> &dvdt , double t);

    // get functions for velocity components
    double surge() {return velocity(0,0);}
    double sway() {return velocity(1,0);}
    double heave() {return velocity(2,0);}
    double roll() {return velocity(3,0);}
    double pitch() {return velocity(4,0);}
    double yaw() {return velocity(5,0);}

    // get functions for position components
    double xPos()     {return x;}
    double yPos()     {return y;}
    double zPos()     {return z;}
    double phiRads()   {return phi;}
    double thetaRads() {return theta;}
    double psiRads()   {return psi;}

    boost::numeric::ublas::matrix<double> matM, invM, matL, matC, matD, matg, matA, matU, matTa, matK, matInput, matJ, n;
    boost::numeric::ublas::matrix<double> velocity, position;
    boost::numeric::ublas::matrix<double> dndt;

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
