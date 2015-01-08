#include <iostream>
#include <fstream>
#include "DynamicModel.h"


int main(int argc, char **argv)
{   
    int x = 200;
    bool growX = true;
    double convToDeg = 180.0/3.1415;

    double t = 0.0;
    const double dt = 0.04;
    double surge = 1.5;
    double initState[12*1] = {surge , 0.0 , 0.0 , 0.0, 0.0, 0.0, 0.0, 0.0 , 0.0 , 0.0, 0.0, 0.0};
    AUVModel* model = new AUVModel(); //initState, t, dt);
    AUVModel* integrator = model->create(model, initState, t, dt);

    // TODO: surge and xPos are not saved to the DynamicModel.csv file
    std::ofstream file;
    file.open ("DynamicModel.csv");
    file << "t" << ',' << "u" << ',' << "v" << ',' << "w" << ',' << "p" << ',' << "q" << ',' << "r" << ',' << "y" << ',' << "z" << ',' << "phi" << ',' << "theta" << ',' << "psi" << std::endl;

    int numbSteps = 1000;
    for(size_t i=0 ; t<numbSteps ; ++i,t+=dt )
    {
        integrator->dostep(model);      // TODO: change to model->doWork() {updateDataMatrixes(matJ, matTa), integrator->dostep(model)}

        file << t << ',' << model->surge() << ',' << model->sway() << ',' << model->heave() << ',' << model->roll()*convToDeg << ',' << model->pitch()*convToDeg << ',' << model->yaw()*convToDeg;
        file << ',' << model->yPos() << ',' << model->zPos() << ',' << model->phiRads()*convToDeg << ',' << model->thetaRads()*convToDeg << ',' << model->psiRads()*convToDeg << std::endl;
        
        model->input(surge/0.0025, 0, 0, 20*3.1415/180.0, 20*3.1415/180.0); //surge/0.0025, 0, 0, 0, 0);
        
    }
    file.close();
    delete integrator;

    return 0;
}

