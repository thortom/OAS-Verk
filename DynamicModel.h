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
    virtual ~AUVModel() {/*std::cout << "AUVModel destructor" << std::endl;*/}

    virtual void dostep(AUVModel*& model);
    AUVModel* create(AUVModel*& model, double initState[], double initTime, double dt);
    void initializeSate(double velocity[]);

    void input(double rpm, double finUp, double finDown, double finStb, double finPort);

    boost::numeric::ublas::matrix<double> getMatTa();
    boost::numeric::ublas::matrix<double> getMatJ();

    void operator() ( const boost::numeric::ublas::matrix<double> &v , boost::numeric::ublas::matrix<double> &dvdt , double t);

    boost::numeric::ublas::matrix<double> matM, invM, matL, matC, matD, matg, matA, matU, matTa, matK, matInput, matJ, n;
    boost::numeric::ublas::matrix<double> velocity, position;

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

void insertToMatrix(boost::numeric::ublas::matrix<double>& mat, double vec[]);


#endif
