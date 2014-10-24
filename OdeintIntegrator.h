#ifndef ODEINT_INTEGRATOR_H
#define ODEINT_INTEGRATOR_H
#include <iostream>

#include "DynamicModel.h"

class OdeintIntegrator : public AUVModel
{
public:
    OdeintIntegrator();
    OdeintIntegrator(AUVModel& model, boost::numeric::ublas::matrix<double>& state, double time , double dt) : state(6, 1)
    {
        this->model = model;
        this->state = state;
        this->time = time;
        this->dt = dt;
    }
    virtual boost::numeric::ublas::matrix<double> dostep()
    {
        integrator.do_step(model , state , time , dt);
        time += dt;
        return state;    
    }

    AUVModel model;
    boost::numeric::ublas::matrix<double> state;
    double time;
    double dt;

private:    
    boost::numeric::odeint::runge_kutta4< boost::numeric::ublas::matrix<double> > integrator;
    
};

#endif
