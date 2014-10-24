#include <iostream>
#include <fstream>
#include "DynamicModel.h"


int main(int argc, char **argv)
{   
    double t = 0.0;
    const double dt = 0.1;
    double surge = 0.5;
    double initState[6*1] = {surge , 0.0 , 0.0 , 0.0, 0.0, 0.0};
    AUVModel* model = new AUVModel();
    AUVModel* integrator = model->create(model, initState, t, dt);

    std::ofstream file;
    file.open ("DynamicModel.csv");
    file << "t" << ',' << "u" << ',' << "v" << ',' << "w" << ',' << "p" << ',' << "q" << ',' << "r" << std::endl;
    for(size_t i=0 ; i<30 ; ++i,t+=dt )
    {
        integrator->dostep(model);
        file << t << ',' << model->velocity(0,0) << ',' << model->velocity(1,0) << ',' << model->velocity(2,0) << ',' << model->velocity(3,0) << ',' << model->velocity(4,0) << ',' << model->velocity(5,0) << std::endl;
        //std::cout << t << '\t' << state(0,0) << '\t' << state(1,0) << '\t' << state(2,0) << '\t' << state(3,0) << '\t' << state(4,0) << '\t' << state(5,0) << std::endl;        
        model->input(surge/0.0025, 0, 0, 0, 0);
    }
    file.close();
    delete integrator;

    return 0;
}

