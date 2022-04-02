#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include "Boids.h"

using Eigen::Vector3d;
using namespace std;

int main(int argc, char *argv[]) 
{
    if (argc < 2) 
    {
        printf("Usage:\n./fishtank sample.in\n");
        exit(1);
    }

    string fname(argv[1]);
    Flock flock(fname);
    flock.simulate();

    return 0;
}