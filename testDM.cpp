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
    // The system seems to be unstable
    std::ofstream file;
    file.open ("DynamicModel.csv");
    file << "t" << ',' << "u" << ',' << "v" << ',' << "w" << ',' << "p" << ',' << "q" << ',' << "r" << ',' << "y" << ',' << "z" << ',' << "phi" << ',' << "theta" << ',' << "psi" << std::endl;
    //file << "t" << ',' << "u" << ',' << "v" << ',' << "w" << std::endl;
    int numbSteps = 1000;
    for(size_t i=0 ; t<numbSteps ; ++i,t+=dt )
    {
        integrator->dostep(model);      // TODO: change to model->doWork() {upDataMatrixes(matJ, matTa), integrator->dostep(model)}
        //model->doWork();
        file << t << ',' << model->surge() << ',' << model->sway() << ',' << model->heave() << ',' << model->roll()*convToDeg << ',' << model->pitch()*convToDeg << ',' << model->yaw()*convToDeg;
        file << ',' << model->yPos() << ',' << model->zPos() << ',' << model->phiRads()*convToDeg << ',' << model->thetaRads()*convToDeg << ',' << model->psiRads()*convToDeg << std::endl;
        //file << t << ',' << model->surge() << ',' << model->sway() << ',' << model->heave() << std::endl;        
        //std::cout << t << '\t' << model->surge() << '\t' << model->sway() << '\t' << model->heave() << '\t' << model->roll() << '\t' << model->pitch() << '\t' << model->yaw() << '\t' << model->xPos();
        //std::cout << '\t' << model->xPos() << '\t' << model->yPos() << '\t' << model->zPos() << '\t' << model->phiRads() << '\t' << model->thetaRads() << '\t' << model->psiRads() << std::endl;
        
        //model->input(surge/0.0025, 80*3.1415/180.0, 80*3.1415/180.0, 80*3.1415/180.0, 80*3.1415/180.0);
        
        model->input(surge/0.0025, 0, 0, 20*3.1415/180.0, 20*3.1415/180.0); //surge/0.0025, 0, 0, 0, 0);
        
    }
    file.close();
    delete integrator;

    return 0;
}

