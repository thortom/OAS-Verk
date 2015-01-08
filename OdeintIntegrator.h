#ifndef ODEINT_INTEGRATOR_H
#define ODEINT_INTEGRATOR_H
#include <iostream>

#include "DynamicModel.h"

class OdeintIntegrator : public AUVModel
{
public:
    OdeintIntegrator();
    OdeintIntegrator(AUVModel*& model, double state[], double time , double dt)
    {
        model->initializeSate(state);
        this->time = time;
        this->dt = dt;
    }

    ~OdeintIntegrator() {/*std::cout << "OdeintIntegrator destructor" << std::endl;*/}

    virtual void dostep(AUVModel*& model)
    {
        integrator.do_step(*model, model->state , time , dt);
        time += dt;   
    }

    double time;
    double dt;

private:    
    boost::numeric::odeint::runge_kutta4< boost::numeric::ublas::matrix<double> > integrator;
    
};

#endif
