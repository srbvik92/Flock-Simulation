#ifndef _BOIDS_H_
#define _BOIDS_H_

#include <eigen3/Eigen/Dense>
#include <vector>
#include "kdTree.H"
using Eigen::Vector3d;
#define MAX_FRC 15
#define MAX_VEL 0.022 * MAX_FRC
using namespace std;

class Flock;

struct Food 
{
    Vector3d position; // The position vector of the food
    Vector3d velocity; // The velocity vector of the food
    double time; // time the food should appear
};


class Boid 
{
    friend class Flock;
    public:
        Boid(Vector3d position=Vector3d(0,0,0), Vector3d velocity=Vector3d(0,0,0)) 
            : position(position), velocity(velocity) {};
        inline bool inBox();

    private:
        Vector3d position;     
        Vector3d velocity;     
        vector<int> neighs; 
        Vector3d force;        
        bool wasInBox = true;
};


class Flock 
{
    public:
        Flock(string fname);
        void readFile(string fname);
        void simulate();

    private:
        void getForce();
        inline void getLineHelper(ifstream &file, string &line);
        void writeOutput();

        vector<Boid> boids;
        vector<Food> foods; 
        double size;       
        double radius;     
        double neigbors;   
        double mass;        
        double collision;  
        double centering;  
        double velocity;   
        double hunger;    
        double damping;     
        double dt;          
        double duration;    
};

#endif