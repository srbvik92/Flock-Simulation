#include "Boids.h"
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <math.h>

using Eigen::Vector3d;


// Boid box parameters
bool Boid::inBox() 
{
    return abs(position[0]) <= 0.5 && abs(position[1]) <= 0.25 && abs(position[2]) <= 0.5;
}


// Default constructor for the Flock class
Flock::Flock(string fname) 
{
    // Read in and parse the file
    readFile(fname);

    // Check that variables were read in properly
    cerr << "size " << size << endl;
    cerr << "radius " << radius << endl;
    cerr << "neigbors " << neigbors << endl;
    cerr << "mass " << mass << endl;
    cerr << "collision " << collision << endl;
    cerr << "centering " << centering << endl;
    cerr << "velocity " << velocity << endl;
    cerr << "hunger " << hunger << endl;
    cerr << "damping " << damping << endl;
    cerr << "dt " << dt << endl;
    cerr << "duration " << duration << endl;
}

// Run the simulation for the flock
void Flock::simulate() 
{

    int nFrames = 0;
    for (double t = 0; t < duration; t += dt) 
    {
        nFrames++;
    }

    // The number of frames
    cout << nFrames << endl;

    for (double t = 0; t < duration; t += dt) 
    {
        getForce(); 

        // Move the food
        for (unsigned int s = 0; s < foods.size(); s++)
        {
            if (foods[s].time <= t)
            {
                foods[s].position += foods[s].velocity * 0.1;
            }

            for (unsigned int j = 0; j < boids.size(); j++)
            {
                if ((boids[j].position - foods[s].position).norm() < radius/3)
                {
                    foods.erase(foods.begin() + s);
                }
            }
        }


        for (unsigned int i = 0; i < boids.size(); i++) 
        {
            // cap max force
            if (boids[i].force.norm() > MAX_FRC) {
                boids[i].force = MAX_FRC * boids[i].force.normalized();
            }

            // flip the velocity if the Boid is out of the box
            if (boids[i].inBox()) 
            {
                boids[i].wasInBox = true;

                // acceleration -> change in velocity
                boids[i].velocity += damping * (boids[i].force * dt);
            }
            else if (boids[i].wasInBox) 
            {
                // only flip the velocity one time (right after it first left the box 
                boids[i].velocity *= -1;
                boids[i].wasInBox = false;
            }

            // cap max velocity
            if (boids[i].velocity.norm() > MAX_VEL) 
            {
                boids[i].velocity = MAX_VEL * boids[i].velocity.normalized();
            }

            // Update the boid position
            boids[i].position += boids[i].velocity * dt;
        }

        writeOutput();
    }
}

void Flock::getForce() 
{
    vector<Eigen::Vector3d> niegh;
    for (unsigned int i = 0; i < boids.size(); i++) 
    {
        // declare variables
        boids[i].force = Vector3d(0, 0, 0);
        Vector3d center = Vector3d(0, 0, 0);
        vector<int> n;

        KDTree kdTree(niegh);
        kdTree.neighbors(niegh, boids[i].position, 0, radius, n);

        // search for neighbors of this boid using KDTrees
        for (unsigned int j = 0; j < n.size(); j++) 
        {
            if (j == i)
                continue;
           
            double dist = (boids[n[j]].position - boids[i].position).norm();
            center += boids[n[j]].position;

            // Set velocity
            boids[i].force += velocity * (boids[n[j]].velocity - boids[i].velocity);
            
            // Set collision avoidance
            boids[i].force += collision * ((boids[j].position - boids[n[i]].position) / pow(dist, 3));
        }

        // now calculate the combined force for this boid
        // calculate the center of this boid's neighbors
        if (n.size() == 0)
        {
            center = boids[i].position;
        }
        else 
        {
            center = center/n.size();
        }

        // Calculate the final force
        boids[i].force += centering * (center - boids[i].position);
    }
}

// Reads in the file and parses the information
void Flock::readFile(std::string fname) 
{
    string line;
    ifstream f(fname);
    int flockSize;
    int nFood;

    // reset class
    boids.clear();
    foods.clear();

    // read first line
    getLineHelper(f, line);
    sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", 
        &size, &radius, &neigbors, &mass, &collision, &centering,
        &velocity, &hunger, &damping, &dt, &duration);

    // read all boids
    getLineHelper(f, line);
    sscanf(line.c_str(), "%d", &flockSize);
    for (int i = 0; i < flockSize; i++) 
    {
        getLineHelper(f, line);
        Boid b;
        sscanf(line.c_str(), "[%lf,%lf,%lf] [%lf,%lf,%lf]", &b.position[0], &b.position[1],
                &b.position[2], &b.velocity[0], &b.velocity[1], &b.velocity[2]);
        boids.push_back(b);
    }

    // read line with nFood
    getLineHelper(f, line);
    sscanf(line.c_str(), "%d", &nFood);

    // read all food
    for (int i = 0; i < nFood; i++) 
    {
        getLineHelper(f, line);
        Food food;
        sscanf(line.c_str(), "[%lf,%lf,%lf] [%lf,%lf,%lf] %lf", &food.position[0], &food.position[1],
                &food.position[2], &food.velocity[0], &food.velocity[1], &food.velocity[2], &food.time);
        foods.push_back(food);
    }
}


// helper function that reads the next line and skips blank lines
void Flock::getLineHelper(ifstream &file, string &line) 
{
    do 
    {
        if (!getline(file, line)) 
        {
            printf("Error: while reading file.\n");
            exit(1);
        }
    }
    while (line == "");
}

// print the output
void Flock::writeOutput() 
{
    cout << boids.size() << endl;
    for (unsigned int i = 0; i < boids.size(); i++) 
    {
        cout << "[" << boids[i].position[0] << "," << boids[i].position[1] << "," << boids[i].position[2] << "] ";
        cout << "[" << boids[i].velocity[0] << "," << boids[i].velocity[1] << "," << boids[i].velocity[2] << "]" << endl;
    }
    cout << foods.size() << endl;
    for (unsigned int i = 0; i < foods.size(); i++) 
    {
        cout << "[" << foods[i].position[0] << "," << foods[i].position[1] << "," << foods[i].position[2] << "]" << endl;
    }
}