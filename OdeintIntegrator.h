#ifndef ODEINT_INTEGRATOR_H
#define ODEINT_INTEGRATOR_H
#include <iostream>

#include "DynamicModel.h"

class OdeintIntegrator : public AUVModel
{
public:
    OdeintIntegrator();
    OdeintIntegrator(AUVModel& model, double state[], double time , double dt)
    {
        model.initializeSate(state);
        this->time = time;
        this->dt = dt;
    }
    virtual void dostep(AUVModel& model)
    {
        //std::cout << model.matInput(0,0) << "\t" << model.matInput(1,0) << "\t" << model.matInput(2,0) << "\t" << model.matInput(3,0) << "\t" << model.matInput(4,0) << std::endl; 
        integrator.do_step(model , model.state , time , dt);
        time += dt;   
    }

    double state[];
    double time;
    double dt;

private:    
    boost::numeric::odeint::runge_kutta4< boost::numeric::ublas::matrix<double> > integrator;
    
};

#endif
