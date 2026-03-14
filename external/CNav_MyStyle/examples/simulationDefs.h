#ifndef __SIMULATIONDEFS_H_INCLUDED__   // if x.h hasn't been included yet...
#define __SIMULATIONDEFS_H_INCLUDED__ 

#include "RVOSimulator.h"

#include "Agent.h"
#include "KdTree.h"
#include "Obstacle.h"
#include <stdlib.h>
#include <cstdlib>
#include <iostream>

#ifdef _OPENMP
#include <omp.h>
#endif

//**************Default Simulation Parameters*********************//

float updateProbability=0.25; // In average, each agent makes decision every (1/updateProbability) timesteps (directly dep in timestep length) 
float simTimeStep=0.05; // Length of the timestep, in each second there are (1/simTimeStep) timesteps
int sideSize=25; //Size of each side of the environment of the Crowd scenario
bool randomPert=1; //Enables small random perturbations on the preferred velocities computed
bool finalize=0; //  marks the end of the simulation
bool followNeighbors; /* 追従行動の有無(false:禁止, true:可能) */

const int numNeighbors=10;//number of neighbors that each agent considers
const float neighborDistance=15.0f; //max distance that agents can perceive neighbors
const float timeHorizonORCA=5.0f;  // time horizon to determine collisions with other agents
const float timeHorizonObstORCA=1.3f;// time horizon to determine collisions with obstacles
const float radiusORCA=0.5f;  // distance that the agents want to keep from other agents
const float maxSpeedORCA=1.5f; //maximum speed that agents can move with

int agentViewed;
float SimScore=0;

float coordFactor = 0.1; // Coordination factor, that balances between the goal progress and the politeness of the agents
float ooparts=0.4; 
static const int lengthSimulateTimeSteps=2; //Number of (future) timesteps to simulate to evaluate the effect of each action
int allNeigh = 1; //0 makes agents consider only neighbors closer than itself to its goal. 1 makes agents consider all neighbors around it, with limit numNeighbors
int contadourX = 1;
int threshold=20000; //Maximum number of timesteps before the simulation is declared not finished.
float baseScore;
int collisions=0;
//General variable declaration
RVO::RVOSimulator* sim;
int algo = 2;
int numAgents = 320;
int scenario = 2;
int notFinished;
int Actions;
int chosenAction[320];
int bestSimilarNeigh[320];
int currentVel[320];
int totalConsideredNeighbors[320];
int mostSimilarAgentID[320];
int ascendingSimilarAgentIDList[320][50];
int iteration;
int timestep;
int lastFrameTime = 0;
int numAgentNotInGoal;
int numAgentInGoal;
int finalIteration = 10;
bool isinGoal[320];
bool visualizer = true;
bool evalMethod[320];
bool loop[320];
float ori[750], prefVelProg[350], energy[350], avgEnergy[350],totalAvgEnergy, energyPerProg[350];
RVO::Vector2 goalVectors[320],goalVector, initPos[320];
std::vector<RVO::Vector2> finalGoals;
float totalMotion = 0;
float angle;
float dist;
float actionVector[50];
float actionVectorMag[50];
float finalTime=-100;
float oriMostSimilar[320][50];
float totalReward[320][50];
float timeToGoal[320];
float goalDistance[320];
float ActionSpeed[320][50];
float ActionDirection[320][50];
float goalX[320];
float goalY[320];
float totalRewardSimulation = 0.0f;
float totalRewardIteration;
float needToUpdate[320];
float LastActionEstimate[320][20];
float Boltz[320][20];
float defaultValue[320][20];
int totalDeviation=0;

#endif
