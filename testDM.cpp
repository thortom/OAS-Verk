#include <iostream>
#include <fstream>
//#include "Matrix.h"
#include "DynamicModel.h"
//#include "DynamicModel.cpp"


int main(int argc, char **argv)
{   
    double t = 0.0;
    const double dt = 0.1;
    boost::numeric::ublas::matrix<double> state(6, 1);
    double surge = 0.5;
    double initState[6*1] = {surge , 0.0 , 0.0 , 0.0, 0.0, 0.0};
    insertToMatrix(state, initState);
    AUVModel model = AUVModel();

    AUVModel* integrator = model.create(model, state, t, dt);

    std::ofstream file;
    file.open ("DynamicModel.csv");
    file << "t" << ',' << "u" << ',' << "v" << ',' << "w" << ',' << "p" << ',' << "q" << ',' << "r" << std::endl;
    for(size_t i=0 ; i<30 ; ++i,t+=dt )
    {
        state = integrator->dostep();
        file << t << ',' << state(0,0) << ',' << state(1,0) << ',' << state(2,0) << ',' << state(3,0) << ',' << state(4,0) << ',' << state(5,0) << std::endl;
        std::cout << t << '\t' << state(0,0) << '\t' << state(1,0) << '\t' << state(2,0) << '\t' << state(3,0) << '\t' << state(4,0) << '\t' << state(5,0) << std::endl;        
        model.input(surge/0.0025, 0, 0, 0, 0);
    }
    file.close();
    delete integrator;

    return 0;
}

