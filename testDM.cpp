#include <iostream>
#include <fstream>
#include "DynamicModel.h"


int main(int argc, char **argv)
{   
    double t = 0.0;
    const double dt = 0.1;
    double surge = 0.5;
    double initState[12*1] = {surge , 0.0 , 0.0 , 0.0, 0.0, 0.0, 0.0, 0.0 , 0.0 , 0.0, 0.0, 0.0};
    AUVModel* model = new AUVModel(); //initState, t, dt);
    AUVModel* integrator = model->create(model, initState, t, dt);

    std::ofstream file;
    file.open ("DynamicModel.csv");
    file << "t" << ',' << "u" << ',' << "v" << ',' << "w" << ',' << "p" << ',' << "q" << ',' << "r" << ',' << "x" << ',' << "y" << ',' << "z" << ',' << "phi" << ',' << "theta" << ',' << "psi" << std::endl;
    for(size_t i=0 ; i<30 ; ++i,t+=dt )
    {
        integrator->dostep(model);      // TODO: change to model->doWork() {upDataMatrixes(matJ, matTa), integrator->dostep(model)}
        //model->doWork();
        file << t << ',' << model->surge() << ',' << model->sway() << ',' << model->heave() << ',' << model->roll() << ',' << model->pitch() << ',' << model->yaw();
        file << ',' << model->xPos() << ',' << model->yPos() << ',' << model->zPos() << ',' << model->phiRads() << ',' << model->thetaRads() << ',' << model->psiRads() << std::endl;        
        std::cout << t << '\t' << model->surge() << '\t' << model->sway() << '\t' << model->heave() << '\t' << model->roll() << '\t' << model->pitch() << '\t' << model->yaw() << '\t' << model->xPos();
        std::cout << '\t' << model->xPos() << '\t' << model->yPos() << '\t' << model->zPos() << '\t' << model->phiRads() << '\t' << model->thetaRads() << '\t' << model->psiRads() << std::endl;
        model->input(surge/0.0025, 0, 0, 0, 0);
    }
    file.close();
    delete integrator;

    return 0;
}

