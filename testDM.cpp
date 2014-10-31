#include <iostream>
#include <fstream>
#include "DynamicModel.h"


int main(int argc, char **argv)
{   
    double t = 0.0;
    const double dt = 0.1;
    double surge = 0.5;
    double initState[6*1] = {surge , 0.0 , 0.0 , 0.0, 0.0, 0.0};
    AUVModel* model = new AUVModel(); //initState, t, dt);
    AUVModel* integrator = model->create(model, initState, t, dt);

    std::ofstream file;
    file.open ("DynamicModel.csv");
    file << "t" << ',' << "u" << ',' << "v" << ',' << "w" << ',' << "p" << ',' << "q" << ',' << "r" << std::endl;
    for(size_t i=0 ; i<30 ; ++i,t+=dt )
    {
        integrator->dostep(model);      // TODO: change to model->doWork() {integrator->dostep(model), updatePos } 
        //model->doWork();
        file << t << ',' << model->surge() << ',' << model->sway() << ',' << model->heave() << ',' << model->roll() << ',' << model->pitch() << ',' << model->yaw() << std::endl;
        //std::cout << t << '\t' << model->surge() << '\t' << model->sway() << '\t' << model->heave() << '\t' << model->roll() << '\t' << model->pitch() << '\t' << model->yaw() << '\t' << model->xPos() << std::endl;        
        model->input(surge/0.0025, 0, 0, 0, 0);
    }
    file.close();
    delete integrator;

    return 0;
}

