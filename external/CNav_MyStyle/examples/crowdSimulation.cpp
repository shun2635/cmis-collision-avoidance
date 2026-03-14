/*Code used for the C-Nav vs ORCA experiments
 * Based on the RVO2 library.
 * Author: Julio Godoy Del Campo
 * Please send all comments and/or questions to juliogodoy@gmail.com
 *  
 * */
#include <iostream>
#include <vector>
#include <fstream>
//#include <gl/glut.h>
#include <stdlib.h>
#include <math.h>
#include <cmath>
#include <vector>
#include <cstdlib>
#include <time.h>
#include <string.h>
#include <algorithm>
#include "simulationDefs.h"


#if HAVE_OPENMP || _OPENMP
#include <omp.h>
#endif

#ifdef __APPLE__
#include "/Users/tamurashuntarou/laboratory_code/CNav/src/RVO.h"
#else
#include "RVO.h"
#endif

#ifndef M_PI
static const float M_PI = 3.14159265358979323846f;
#endif


void countCollisions() /* 衝突判定をし、カウントを増やす */
{
    /* ゴールに入っていないエージェントi,jに対して実行 */
    for(int i = 0; i < numAgents; i++){
        for(int j = 0; j < numAgents; j++){
            if( !(isinGoal[i]) && !(isinGoal[j]) && (i != j)){ 
                /* エージェント間の距離が半径の和より小さければ衝突カウントを増やす */
                float distance = RVO::abs(sim->getAgentPosition(i)-sim->getAgentPosition(j));  
                if(distance<sim->getAgentRadius(i) + sim->getAgentRadius(j)){	
                    collisions++;
                }
            }
	    }
    } 
} 


int getClosestToGoal() /* ゴールに到達していないエージェントの中で、最もゴールに近いエージェントを返す */
{	
	float minDistanceToGoal = 10000.0f;
    float distToGoal;
	int closest = -1;		
    for (int i = 0; i < sim->getNumAgents(); ++i) {
		if(!isinGoal[i]){
			distToGoal= RVO::abs(finalGoals[i] - sim->getAgentPosition(i));
			if(distToGoal < minDistanceToGoal){
				minDistanceToGoal = distToGoal;
				closest = i;
			}	
		}	
	}
	return closest;
}

bool checkOverlap(size_t myID)  /* エージェントiの位置が、以前に位置したエージェントの位置と重なるかどうかをチェックする(その場合はfalseを返す)*/
{
	bool flag = true;
    float myDiameter = 2 * sim->getAgentRadius(myID);
    for (size_t j = 0; j < myID; j++){
        float distToYou = RVO::abs(sim->getAgentPosition(j) - sim->getAgentPosition(myID));
		if(distToYou < myDiameter){
			flag = false;	
		}
	}
	return flag;
}

void generateScenarioCrowd(std::ofstream initPositionsFile) /* 群衆シナリオにおけるエージェントの位置を生成する */
{
	float xInitPos, xGoalPos, yInitPos,yGoalPos;
	srand (time(NULL));
	RVO::Vector2 tempGoal,initPos;
	for (int i=0; i<numAgents; i++) {
		sim->addAgent(RVO::Vector2());
		do{
			xInitPos= rand()%sideSize;
			yInitPos= rand()%sideSize;
			
			initPos=RVO::Vector2(xInitPos,yInitPos);
			sim->setAgentPosition(i,initPos);
			do{
				xGoalPos= rand()%(sideSize);
				yGoalPos= rand()%(sideSize);
				tempGoal=RVO::Vector2(xGoalPos,yGoalPos);
			}while((RVO::abs(tempGoal-initPos)<((float)sideSize/(float)2))||((xGoalPos>=(((float)sideSize/(float)2)-3))&&(xGoalPos<=(((float)sideSize/(float)2)+3))&&(yGoalPos>=(((float)sideSize/(float)2)-3))&&(yGoalPos<=(((float)sideSize/(float)2)+3))));
			
		}while((!checkOverlap(i))||((xInitPos>=(((float)sideSize/(float)2)-3))&&(xInitPos<=(((float)sideSize/(float)2)+3))&&(yInitPos>=(((float)sideSize/(float)2)-3))&&(yInitPos<=(((float)sideSize/(float)2)+3))));
		
		initPositionsFile << "Agent" << i << ",xInitPos:" << sim->getAgentPosition(i).x() << ",yInitPos:"<< sim->getAgentPosition(i).y() << ",xGoalPos:" << xGoalPos <<",yGoalPos:" << yGoalPos << std::endl;
		finalGoals.push_back(RVO::Vector2(xGoalPos,yGoalPos));
		sim->setAgentGoal(i, RVO::Vector2(xGoalPos,yGoalPos));

        }
 	
    initPositionsFile.close();
}

float getOriSimilarNeighbor(int myID, int yourID) /*  */
{	
	float myRadius = sim->getAgentRadius(myID);
    
    RVO::Vector2 myPosition = sim->getAgentPosition(yourID);
    RVO::Vector2 yourPosition = sim->getAgentPosition(yourID);
    RVO::Vector2 myGoal = finalGoals[myID];
    RVO::Vector2 yourNormalizeVelocity = RVO::normalize(sim->getAgentVelocity(yourID));
	RVO::Vector2 correctedYourPosition = (yourPosition - yourNormalizeVelocity * myRadius);//
	RVO::Vector2 relativeYourPosition = correctedYourPosition - myPosition;
    RVO::Vector2 myNormalizePrefV = RVO::normalize(myGoal - myPosition);
	RVO::Vector2 prefVPosition = myPosition +  myNormalizePrefV;//prefVの到達先
	
	float l = myNormalizePrefV.x() * (correctedYourPosition.y() - myPosition.y()) 
                - myNormalizePrefV.y() * (correctedYourPosition.x() - myPosition.x());
	//相手の相対位置と自分の希望速度の内積
    float tempPosition = ( (relativeYourPosition * RVO::normalize(myGoal - myPosition)) / (RVO::abs(relativeYourPosition) * RVO::abs(RVO::normalize(myGoal - myPosition))) ); 
    float rad = 0.0f;
	
	if(!((tempPosition - 1) < 0.000001 ) && ((tempPosition-1) > -0.000001)){	
        rad = acos(tempPosition);
	}
	
	if((l > 0) && (rad < M_PI)){ // it is on the other side:
		rad = -rad;
	}

	return rad;	
}

void computeSimilarAgentsDir(int myID)
{
	std::vector< std::pair<float, int> > vector;
    RVO::Vector2 myPosition = sim->getAgentPosition(myID);
    RVO::Vector2 myGoal = sim->getAgentGoal(myID);
    RVO::Vector2 myGoalPrefVelocity = sim->getAgentGoalPrefVelocity(myID);
    size_t you;
    RVO::Vector2 yourPosition;
    RVO::Vector2 yourVelocity;
    RVO::Vector2 yourPrefVelocity;
    totalConsideredNeighbors[myID] = 0;


	for(int yourID = 0; yourID < sim->getAgentNumAgentNeighbors(myID); yourID++)
	{
		you = sim->getAgentAgentNeighbor(myID,yourID);
        yourPosition = sim->getAgentPosition(you);
        yourVelocity = sim->getAgentVelocity(you);
        yourPrefVelocity = sim->getAgentPrefVelocity(you);

        float GoalDist = RVO::abs(myGoal - myPosition) - RVO::abs(myGoal - yourPosition);
		float prefVelocitySimilarity = myGoalPrefVelocity * yourPrefVelocity;
		
		if(myID == agentViewed)
        {
			std::cout << "Agent " << you << " has GoalDist " << GoalDist << " and prefVelocitySimilarity " << prefVelocitySimilarity <<  " cause my goalVpref is " << myGoalPrefVelocity <<" \n";
		}
			
		if((!isinGoal[you]) && (GoalDist > 0.0) && (prefVelocitySimilarity > 0)){ //ここでは、ゴールに近いエージェントのみを類似性の対象とする
            if((goalVectors[myID] * yourVelocity) > 0 ){ // If neighbor is actually going in the same direction as my goal
                vector.push_back(std::make_pair((goalVectors[myID] * yourVelocity)/(RVO::abs(goalVectors[myID] * 1.5)), you));
                totalConsideredNeighbors[myID]++;		   
            }
		}	
	}

	sort(vector.begin(),vector.end());		
    
	float politeness = 0;
	mostSimilarAgentID[myID] = -1;
	for (int i = 0; i < totalConsideredNeighbors[myID]; i++) 
    { 
		ascendingSimilarAgentIDList[myID][i] = -1;
		mostSimilarAgentID[myID] = vector[i].second;
        ascendingSimilarAgentIDList[myID][i] = vector[i].second;
        oriMostSimilar[myID][i] = getOriSimilarNeighbor(myID,ascendingSimilarAgentIDList[myID][i]);
        politeness += vector[i].first;          
    } 
	
	if(mostSimilarAgentID[myID] > -1)
	{
		float aver = getOriSimilarNeighbor(myID,mostSimilarAgentID[myID]);
	 	if(myID == agentViewed){
			std::cout << " The most similar is... " << mostSimilarAgentID[myID] << " with an angle of " << aver << std::endl;	
		}
	}
}

float computeSimilarAgentSpeed(int similar, RVO::Vector2 goalVector) /*  */
{	
	return (goalVector * sim->getAgentVelocity(similar));
}

void evaluateEachAction()  /* timeHorizonで定義された将来のタイムステップ数にわたり、各action/velocityについてシミュレーションする */
{
    sim->buildTree();
    sim->setVvalues();
    int maxEvalActions = Actions;
	
    for (int i = 0; i < static_cast<int>(sim->getNumAgents()); ++i){
        if(needToUpdate[i]){ /* 更新の必要があるなら実行 */
            float finalReward = 0.0f;
            float finalAction = 0.0f;
			if(!evalMethod[i]){	/* 基本実行するようになっている */
                if(followNeighbors){ /* 追従OKなら実行 */
					computeSimilarAgentsDir(i);
					maxEvalActions = Actions - 1;
				}else{ /* 追従禁止なら実行 */
				    totalConsideredNeighbors[i] = 0;	
				}

		        if(followNeighbors){ /* 追従OKなら実行 */
			        bestSimilarNeigh[i] = -1;
			        float bestFollowMove = -100;
                    float rewardFollow = -1;
			        if(totalConsideredNeighbors[i] > 0){	
				        for (int e = 0; e < totalConsideredNeighbors[i]; e++) {
					        ActionDirection[i][Actions-1]= getOriSimilarNeighbor(i,ascendingSimilarAgentIDList[i][e]);
					        ActionSpeed[i][Actions-1] = computeSimilarAgentSpeed(ascendingSimilarAgentIDList[i][e],goalVectors[i]);
					        rewardFollow= sim->SimulateVelocity(i, finalGoals[i], lengthSimulateTimeSteps, Actions-1,numNeighbors, numAgents,  coordFactor, ooparts,ActionSpeed[i][Actions-1],ActionDirection[i][Actions-1], allNeigh, contadourX);
                            std::cout << "************ " << e << " similar neigh is : " << ascendingSimilarAgentIDList[i][e] << " and value is " <<rewardFollow << "\n";
					        if(rewardFollow>bestFollowMove){
						        bestSimilarNeigh[i]= ascendingSimilarAgentIDList[i][e];
						        bestFollowMove = rewardFollow;
					        }
				        }
	
                        ActionDirection[i][Actions-1]= getOriSimilarNeighbor(i,bestSimilarNeigh[i]);
                        ActionSpeed[i][Actions-1]=computeSimilarAgentSpeed(bestSimilarNeigh[i],goalVectors[i]);
                        totalReward[i][Actions-1]= bestFollowMove;

                        std::cout << "************ Best Similar neigh is : " << bestSimilarNeigh[i] << " and value is " <<bestFollowMove << " Dir " <<ActionDirection[i][Actions-1] << " Mag: " <<ActionSpeed[i][Actions-1] <<  "\n";
			        }
    	        }
	

                if((totalConsideredNeighbors[i] < 1) && (followNeighbors)){
                    ActionDirection[i][Actions-1]= 0.0000;
                    ActionSpeed[i][Actions-1] = 1.5;
                    totalReward[i][Actions-1] = -1000;	
                }
                   
                for (int a = 0; a < maxEvalActions; a++){   
                    totalReward[i][a]=0;
                    totalReward[i][a] = sim->SimulateVelocity(i, finalGoals[i], lengthSimulateTimeSteps, a,numNeighbors, numAgents,  coordFactor, ooparts,ActionSpeed[i][a],ActionDirection[i][a], allNeigh, contadourX);
                }
                        
                finalReward = -1000;
                float totalCNavPoliteness = 0;
                for (int action = 0; action < Actions; action++){
                    if(i == agentViewed){
                        std::cout << " Action " << action << " value: " << 	totalReward[i][action] << " goalProg: " << sim->getAgentGoalProg(i,action) << " CNAVPol: " << sim->getAgentCnavPoliteness(i,action) /*<< " ALANPol: " <<sim->getAgentALANPoliteness(i,ac)*/ << "\n";
                    }			
                    totalCNavPoliteness+=sim->getAgentCnavPoliteness(i,action);        
                    if(totalReward[i][action]>=(finalReward)){
                        finalReward=totalReward[i][action];
                        finalAction = action;				
                    }
                }
                            
                float avgCNavPoliteness=totalCNavPoliteness/(float)Actions;
            
                float tempStd = 0;
                for(int ac=0;ac<Actions;ac++){
                    tempStd = tempStd + ((sim->getAgentCnavPoliteness(i,ac)-avgCNavPoliteness)*(sim->getAgentCnavPoliteness(i,ac)-avgCNavPoliteness));
                }
                tempStd = sqrt(tempStd/(float)(Actions - 1));
                        
                if(avgCNavPoliteness > 0.99) //means that neighbors are immune to actions of this agent 
                {
                    //use ALAN politeness	
                    finalReward = -1000;
                    for (int ac=0;ac<Actions; ac++){
                        if(totalReward[i][ac]>=(finalReward)){
                            finalReward=totalReward[i][ac];
                            finalAction=ac;
                        }
                    }				
                }                          
                chosenAction[i] = finalAction;            
                std::cout << " CHOSEN ACTION: " << chosenAction[i] << "\n";	
		    }		
		} 
    }
}

void setActions(int myID)  /* エージェントの行動をセットする */
{
    float noVector;
    float fullVector;
    float sumA = 0;
    RVO::Vector2 myNormalizeGoalVector = RVO::normalize(goalVectors[myID]);
    
    for(int action = 0; action < Actions; action++){
        float myThisActionSpeed = ActionSpeed[myID][action];
        float myThisActionDirection = ActionDirection[myID][action];
        ActionSpeed[myID][action] = actionVectorMag[action];
        ActionDirection[myID][action] = actionVector[action];
        goalVectors[myID] = RVO::normalize(finalGoals[myID] - sim->getAgentPosition(myID)); 
        
        RVO::Vector2 velocityPosition = myThisActionSpeed * RVO::Vector2(
                            myNormalizeGoalVector.x() * std::cos(myThisActionDirection)
                            + myNormalizeGoalVector.y() * std::sin(myThisActionDirection),
                            myNormalizeGoalVector.y() * std::cos(myThisActionDirection)
                            + myNormalizeGoalVector.x() * -std::sin(myThisActionDirection)); 
		noVector = (1 - coordFactor) * (RVO::Vector2(0,0) * myNormalizeGoalVector)/1.5f + coordFactor * (RVO::Vector2(0,0) * velocityPosition)/2.25f ;
		fullVector = (1 - coordFactor) * (velocityPosition * myNormalizeGoalVector)/1.5f + coordFactor * (velocityPosition * velocityPosition)/2.25f ; 		
        
        if(fullVector > noVector){
            defaultValue[myID][action] = fullVector;
        }else{
            defaultValue[myID][action] = noVector;
        }
        std::cout << "Default value action " << action << " is " << defaultValue[myID][action] << std::endl;		  		 
		sumA += defaultValue[myID][action];
		Boltz[myID][action] = 100 * (exp(defaultValue[myID][action]/0.05f))/sumA;
    }
    std::cout << "SUM " << sumA << " AVG " << sumA / (float)Actions << std::endl;
}

void setPreferredVelocities(int algorithm) /* 選択されたアクションに基づいてエージェントの優先速度をセットする */
{	
    for (int i = 0; i <numAgents ; ++i) {
		if(isinGoal[i] == false){ /* ゴール未到達エージェントに対して実行 */

            RVO::Vector2 myPrefVelocity = sim->getAgentPrefVelocity(i);
            RVO::Vector2 myGoalVector = finalGoals[i] - sim->getAgentPosition(i);
            
            goalDistance[i] = RVO::abs(myGoalVector);
            goalVectors[i]= RVO::normalize(myGoalVector);
            goalX[i]=goalVectors[i].x();
            goalY[i]=goalVectors[i].y();
        
            sim->setAgentGoalPrefVelocity(i, goalVectors[i] * 1.5);
			
			if(algorithm == 1){ /* ORCAの場合 */	
				angle = std::rand() * 2.0f * M_PI / RAND_MAX;
				dist = std::rand() * 0.01f / RAND_MAX;
				sim->setAgentPrefVelocity(i, myPrefVelocity + dist * RVO::Vector2(std::cos(angle), std::sin(angle)));
			}

			if(algorithm == 2){ /* C-Navの場合 */
				angle =  ActionDirection[i][chosenAction[i]];
				dist = std::rand() * 0.01f / RAND_MAX;
				sim->setAgentPrefVelocity(i, 
                                        RVO::Vector2(myPrefVelocity.x() * std::cos(angle) + myPrefVelocity.y()*std::sin(angle),
                                                    myPrefVelocity.y() * std::cos(angle) + myPrefVelocity.x()*-std::sin(angle)));
	            /* 摂動 */
                if(randomPert){
                    angle = std::rand() * 2.0f * M_PI / RAND_MAX;
                    sim->setAgentPrefVelocity(i, myPrefVelocity + dist * RVO::Vector2(std::cos(angle), std::sin(angle)));
                }
			}		
		}
        else{ /* ゴールに到達していたら場外に飛ばして停止 */
			sim->setAgentPosition(i,RVO::Vector2(-1000.0f,-1000.0f));
			sim->setAgentVelocity(i, RVO::Vector2());		
		}	
	}	
}

void setupScenario(RVO::RVOSimulator* sim) //Initialize the Simulation, positions of the obstacles and the agents
{
	// Specify the global time step of the simulation. 
	sim->setTimeStep(simTimeStep);
	
	int j;
	numAgentInGoal=0;
	timestep=0;
	totalRewardSimulation=0;
  
	
	for (int i = 0; i < numAgents; ++i) 
	{
		
		needToUpdate[i] = 0;
		sim->setAgentDefaults(neighborDistance, numNeighbors, timeHorizonORCA , timeHorizonObstORCA , radiusORCA , maxSpeedORCA);
		chosenAction[i] = 0;
		currentVel[i] = 0;
		evalMethod[i] = false; /* false:シミュレートされた報酬を使用, true:実行に基づく報酬を使用 */
		timeToGoal[i] = 0;
		isinGoal[i] = 0;

		for(j=0;j<Actions;j++)
		{
			totalReward[i][j]=0;
			LastActionEstimate[i][j]=0;
		}						
		
		if(algo==1)
		{
			ActionSpeed[i][0]=1.5;
			ActionDirection[i][0]=0;
			chosenAction[i]=0;
		}	
	}
	
	// Adding Obstacles for each scenario
	std::vector<RVO::Vector2> obstacle1, obstacle2, obstacle3, obstacle4,obstacle5, obstacle6, obstacle7, obstacle8, obstacle9, obstacle10, obstacle11, obstacle12, obstacle13;
	
	if(scenario==3)
    {
        obstacle1.push_back(RVO::Vector2(200.0f,   31.0f));
        obstacle1.push_back(RVO::Vector2(-200.0f,   31.0f));
        obstacle1.push_back(RVO::Vector2(-200.0f, 30.6f));
        obstacle1.push_back(RVO::Vector2(200.0f, 30.6f));
        
        obstacle2.push_back(RVO::Vector2(200.0f,   27.4f));
        obstacle2.push_back(RVO::Vector2(-200.0f, 27.4f));
        obstacle2.push_back(RVO::Vector2(-200.0f, 27.0f));
        obstacle2.push_back(RVO::Vector2(200.0f, 27.0f));
        
        sim->addObstacle(obstacle1);
        sim->addObstacle(obstacle2);
        
        sim->processObstacles();
    }
    
  	if(scenario==4)
	{
	    obstacle1.push_back(RVO::Vector2(200.0f,   3.0f));
        obstacle1.push_back(RVO::Vector2(2.0f,   3.0f));
        obstacle1.push_back(RVO::Vector2(2.0f, 2.0f));
        obstacle1.push_back(RVO::Vector2(200.0f, 2.0f));
        
        obstacle2.push_back(RVO::Vector2(200.0f,   -2.0f));
        obstacle2.push_back(RVO::Vector2(2.0f, -2.0f));
        obstacle2.push_back(RVO::Vector2(2.0f, -3.0f));
        obstacle2.push_back(RVO::Vector2(200.0f, -3.0f));

        obstacle3.push_back(RVO::Vector2(-2.0f,   3.0f));
        obstacle3.push_back(RVO::Vector2(-200.0f,   3.0f));
        obstacle3.push_back(RVO::Vector2(-200.0f, 2.0f));
        obstacle3.push_back(RVO::Vector2(-2.0f, 2.0f));
        
        obstacle4.push_back(RVO::Vector2(-2.0f, -2.0f));
        obstacle4.push_back(RVO::Vector2(-200.0f,   -2.0f));
        obstacle4.push_back(RVO::Vector2(-200.0f, -3.0f));
        obstacle4.push_back(RVO::Vector2(-2.0f, -3.0f));

	    obstacle5.push_back(RVO::Vector2(3.0f,200.0f));
        obstacle5.push_back(RVO::Vector2(2.0f, 200.0f));
        obstacle5.push_back(RVO::Vector2(2.0f, 3.0f));
        obstacle5.push_back(RVO::Vector2(3.0f, 3.0f));
        
        obstacle6.push_back(RVO::Vector2(-2.0f,   200.0f));
        obstacle6.push_back(RVO::Vector2(-3.0f, 200.0f));
        obstacle6.push_back(RVO::Vector2(-3.0f, 3.0f));
        obstacle6.push_back(RVO::Vector2(-2.0f, 3.0f));
         
        obstacle7.push_back(RVO::Vector2(3.0f,   -3.0f));
        obstacle7.push_back(RVO::Vector2(2.0f,   -3.0f));
        obstacle7.push_back(RVO::Vector2(2.0f, -200.0f));
        obstacle7.push_back(RVO::Vector2(3.0f, -200.0f));
        
        obstacle8.push_back(RVO::Vector2(-2.0f, -3.0f));
        obstacle8.push_back(RVO::Vector2(-3.0f,   -3.0f));
        obstacle8.push_back(RVO::Vector2(-3.0f, -200.0f));
        obstacle8.push_back(RVO::Vector2(-2.0f, -200.0f));
        
        sim->addObstacle(obstacle1);
        sim->addObstacle(obstacle2);
        sim->addObstacle(obstacle3);
        sim->addObstacle(obstacle4);
        
        sim->addObstacle(obstacle5);
        sim->addObstacle(obstacle6);
        sim->addObstacle(obstacle7);
        sim->addObstacle(obstacle8);
        
        sim->processObstacles(); 
	  
	}  
	
	if(scenario==5)
	{ 
		obstacle1.push_back(RVO::Vector2(-10.0f,   350.0f));
		obstacle1.push_back(RVO::Vector2(-11.0f,   350.0f));
		obstacle1.push_back(RVO::Vector2(-11.0f, 0.7f));
		obstacle1.push_back(RVO::Vector2(-10.0f, 0.7f));
					
		obstacle2.push_back(RVO::Vector2(-10.0f,   -0.7f));
		obstacle2.push_back(RVO::Vector2(-11.0f, -0.7f));
		obstacle2.push_back(RVO::Vector2(-11.0f, -150.0f));
		obstacle2.push_back(RVO::Vector2(-10.0f, -150.0f));
							
		sim->addObstacle(obstacle1);
		sim->addObstacle(obstacle2);
				
		sim->processObstacles();
	}
			
	if(scenario==10) 
		{
			obstacle1.push_back(RVO::Vector2(0.0f,   12.0f));
			obstacle1.push_back(RVO::Vector2(-2.0f,   12.0f));
			
			obstacle1.push_back(RVO::Vector2(-2.0f,   -2.0f));
			obstacle1.push_back(RVO::Vector2(0.0f,   -2.0f));
			

			obstacle2.push_back(RVO::Vector2(30.0f,   0.0f));
			obstacle2.push_back(RVO::Vector2(0.0f,   0.0f));
			
			obstacle2.push_back(RVO::Vector2(0.0f,   -2.0f));
			obstacle2.push_back(RVO::Vector2(30.0f,   -2.0f));
			
			
			obstacle3.push_back(RVO::Vector2(30.0f,   12.0f));
			obstacle3.push_back(RVO::Vector2(0.0f,   12.0f));
			
			obstacle3.push_back(RVO::Vector2(0.0f,   10.0f));
			obstacle3.push_back(RVO::Vector2(30.0f,   10.0f));
			
			
			
			obstacle4.push_back(RVO::Vector2(32.0f,   12.0f));
			obstacle4.push_back(RVO::Vector2(30.0f,   12.0f));
			
			obstacle4.push_back(RVO::Vector2(30.0f,   -2.0f));
			obstacle4.push_back(RVO::Vector2(32.0f,   -2.0f));
			
			
			
			
			
			obstacle5.push_back(RVO::Vector2(11.0f,   2.95f));
			obstacle5.push_back(RVO::Vector2(5.0f,   2.95f));
			
			obstacle5.push_back(RVO::Vector2(5.0f,   1.05f));
			obstacle5.push_back(RVO::Vector2(11.0f,   1.05f));
			
			
			obstacle6.push_back(RVO::Vector2(11.0f,   5.95f));
			obstacle6.push_back(RVO::Vector2(5.0f,   5.95f));
			
			obstacle6.push_back(RVO::Vector2(5.0f,   4.05f));
			obstacle6.push_back(RVO::Vector2(11.0f,   4.05f));
			
			
			obstacle7.push_back(RVO::Vector2(11.0f,   8.95f));
			obstacle7.push_back(RVO::Vector2(5.0f,   8.95f));
			
			obstacle7.push_back(RVO::Vector2(5.0f,   7.05f));
			obstacle7.push_back(RVO::Vector2(11.0f,   7.05f));
			
			
			
			obstacle8.push_back(RVO::Vector2(18.0f,   8.95f));
			obstacle8.push_back(RVO::Vector2(12.0f,   8.95f));
			
			obstacle8.push_back(RVO::Vector2(12.0f,   7.05f));
			obstacle8.push_back(RVO::Vector2(18.0f,   7.05f));
			
			
			obstacle9.push_back(RVO::Vector2(18.0f,   2.95f));
			obstacle9.push_back(RVO::Vector2(12.0f,   2.95f));
			
			obstacle9.push_back(RVO::Vector2(12.0f,   1.05f));
			obstacle9.push_back(RVO::Vector2(18.0f,   1.05f));
			
			
			
			obstacle10.push_back(RVO::Vector2(18.0f,   5.95f));
			obstacle10.push_back(RVO::Vector2(12.0f,   5.95f));
			
			obstacle10.push_back(RVO::Vector2(12.0f,   4.05f));
			obstacle10.push_back(RVO::Vector2(18.0f,   4.05f));
			
			
			
			obstacle11.push_back(RVO::Vector2(25.0f,   8.95f));
			obstacle11.push_back(RVO::Vector2(19.0f,   8.95f));
			
			obstacle11.push_back(RVO::Vector2(19.0f,   7.05f));
			obstacle11.push_back(RVO::Vector2(25.0f,   7.05f));
			
			
			obstacle12.push_back(RVO::Vector2(25.0f,   2.95f));
			obstacle12.push_back(RVO::Vector2(19.0f,   2.95f));
			
			obstacle12.push_back(RVO::Vector2(19.0f,   1.05f));
			obstacle12.push_back(RVO::Vector2(25.0f,   1.05f));
			
			
			
			obstacle13.push_back(RVO::Vector2(25.0f,   5.95f));
			obstacle13.push_back(RVO::Vector2(19.0f,   5.95f));
			
			obstacle13.push_back(RVO::Vector2(19.0f,   4.05f));
			obstacle13.push_back(RVO::Vector2(25.0f,   4.05f));
			
			
			
		sim->addObstacle(obstacle1);
        sim->addObstacle(obstacle2);
        
        sim->addObstacle(obstacle3);
        sim->addObstacle(obstacle4);
        
        sim->addObstacle(obstacle5);
        sim->addObstacle(obstacle6);
        sim->addObstacle(obstacle7);
        
        sim->addObstacle(obstacle8);
        sim->addObstacle(obstacle9);
        sim->addObstacle(obstacle10);
        
        sim->addObstacle(obstacle11);
        sim->addObstacle(obstacle12);
        sim->addObstacle(obstacle13);
        
        sim->processObstacles();
			
		}
	
	if(scenario==1) 
		{		
			obstacle1.push_back(RVO::Vector2(-1.5f,   -0.5f));
			obstacle1.push_back(RVO::Vector2(-1.5f,   -1.5f));
			obstacle1.push_back(RVO::Vector2(sideSize+1.5f, -1.5f));
			obstacle1.push_back(RVO::Vector2(sideSize+1.5f, -0.5f));
				
			obstacle2.push_back(RVO::Vector2(sideSize+0.5f,   sideSize+1.5f));
			obstacle2.push_back(RVO::Vector2(sideSize+0.5f,   -1.5f));
			obstacle2.push_back(RVO::Vector2(sideSize+1.5f, -1.5f));
			obstacle2.push_back(RVO::Vector2(sideSize+1.5f, sideSize+1.5f));
				
			obstacle3.push_back(RVO::Vector2(-1.5f,   sideSize+1.5f));
			obstacle3.push_back(RVO::Vector2(-1.5f,   sideSize+0.5f));
			obstacle3.push_back(RVO::Vector2(sideSize+1.5f,  sideSize+0.5f));
			obstacle3.push_back(RVO::Vector2(sideSize+1.5f,  sideSize+1.5f));
				
			obstacle4.push_back(RVO::Vector2(-1.5f,   sideSize+1.5f));
			obstacle4.push_back(RVO::Vector2(-1.5f,   -1.5f));
			obstacle4.push_back(RVO::Vector2(-0.5f, -1.5f));
			obstacle4.push_back(RVO::Vector2(-0.5f, sideSize+1.5f));
					
			sim->addObstacle(obstacle1);
			sim->addObstacle(obstacle2);
			sim->addObstacle(obstacle3);
			sim->addObstacle(obstacle4);
		
				
			sim->processObstacles();
				
			if(iteration>1)
			{
				getScenarioCrowd();
				
			}
				 
			if(iteration==1)
			{
				genScenario();
			
			}
			
		}
	   
    if(scenario==7)
    {
        obstacle1.push_back(RVO::Vector2(-10.0f,   2.0f));
        obstacle1.push_back(RVO::Vector2(-10.0f,   0.6f));
        obstacle1.push_back(RVO::Vector2(10.0f, 0.6f));
        obstacle1.push_back(RVO::Vector2(10.0f, 2.0f));
        
        obstacle2.push_back(RVO::Vector2(-10.0f,   -0.6f));
        obstacle2.push_back(RVO::Vector2(-10.0f,   -2.0f));
        obstacle2.push_back(RVO::Vector2(10.0f, -2.0f));
        obstacle2.push_back(RVO::Vector2(10.0f, -0.6f));
        
        obstacle3.push_back(RVO::Vector2(-100.0f,   100.0f));
        obstacle3.push_back(RVO::Vector2(-100.0f,   10.0f));
        obstacle3.push_back(RVO::Vector2(-10.0f, 0.6f));
        obstacle3.push_back(RVO::Vector2(-10.0f, 100.0f));
        
         obstacle4.push_back(RVO::Vector2(-100.0f,   -10.0f));
        obstacle4.push_back(RVO::Vector2(-100.0f,   -100.0f));
        obstacle4.push_back(RVO::Vector2(-10.0f, -100.0f));
        obstacle4.push_back(RVO::Vector2(-10.0f, -0.6f));
        
        
        obstacle5.push_back(RVO::Vector2(10.0f,   100.0f));
        obstacle5.push_back(RVO::Vector2(10.0f,   0.6f));
        obstacle5.push_back(RVO::Vector2(100.0f, 10.0f));
        obstacle5.push_back(RVO::Vector2(100.0f, 100.0f));
        
         obstacle6.push_back(RVO::Vector2(10.0f,   -0.6f));
        obstacle6.push_back(RVO::Vector2(10.0f,   -100.0f));
        obstacle6.push_back(RVO::Vector2(100.0f, -100.0f));
        obstacle6.push_back(RVO::Vector2(100.0f, -10.0f));
        
        sim->addObstacle(obstacle1);
        sim->addObstacle(obstacle2);
        
         sim->addObstacle(obstacle3);
        sim->addObstacle(obstacle4);
        
          sim->addObstacle(obstacle5);
        sim->addObstacle(obstacle6);
        
        sim->processObstacles();
    }
 
    if(scenario==8)
    {   
        obstacle1.push_back(RVO::Vector2(-1.0f,   0.5f));
        obstacle1.push_back(RVO::Vector2(-1.0f,   -1.5f));
        obstacle1.push_back(RVO::Vector2(1.0f, -1.5f));
        obstacle1.push_back(RVO::Vector2(1.0f, 0.5f));
        
        obstacle2.push_back(RVO::Vector2(3.0f,   3.5f));
        obstacle2.push_back(RVO::Vector2(3.0f,   1.5f));
        obstacle2.push_back(RVO::Vector2(5.0f, 1.5f));
        obstacle2.push_back(RVO::Vector2(5.0f, 3.5f));
        
        obstacle3.push_back(RVO::Vector2(-4.5f,   -2.0f));
        obstacle3.push_back(RVO::Vector2(-4.5f,   -4.0f));
        obstacle3.push_back(RVO::Vector2(-2.5f, -4.0f));
        obstacle3.push_back(RVO::Vector2(-2.5f, -2.0f));
        
         obstacle4.push_back(RVO::Vector2(-1.5f,   6.5f));
        obstacle4.push_back(RVO::Vector2(-1.5f,   4.5f));
        obstacle4.push_back(RVO::Vector2(0.5f, 4.5f));
        obstacle4.push_back(RVO::Vector2(0.5f, 6.5f));
        
  
                          
        sim->addObstacle(obstacle1);
        sim->addObstacle(obstacle2);
        sim->addObstacle(obstacle3);
        sim->addObstacle(obstacle4);
        sim->processObstacles();
        
	}
	
	if(scenario==9)
	{ 
		obstacle1.push_back(RVO::Vector2(-10.0f,   350.0f));
		obstacle1.push_back(RVO::Vector2(-11.0f,   350.0f));
		obstacle1.push_back(RVO::Vector2(-11.0f, 0.7f));
		obstacle1.push_back(RVO::Vector2(-10.0f, 0.7f));
					
		obstacle2.push_back(RVO::Vector2(-10.0f,   -0.7f));
		obstacle2.push_back(RVO::Vector2(-11.0f, -0.7f));
		obstacle2.push_back(RVO::Vector2(-11.0f, -150.0f));
		obstacle2.push_back(RVO::Vector2(-10.0f, -150.0f));
							
		sim->addObstacle(obstacle1);
		sim->addObstacle(obstacle2);
				
		sim->processObstacles();
	}
	//Adding agents for each scenario
	
	if(scenario==2) //Circle Scenario
	{	
		for (int i = 0; i < numAgents; ++i) 
			{
			sim->addAgent(float(numAgents) *
						  RVO::Vector2(std::cos(i * 2.0f * M_PI / float(numAgents))+(std::rand() * 0.01f /(float)RAND_MAX) , std::sin(i * 2.0f * M_PI / float(numAgents))+(std::rand() * 0.01f /(float)RAND_MAX) ));
			
			finalGoals.push_back(-sim->getAgentPosition(i));
			    sim->setAgentGoal(i, -sim->getAgentPosition(i));
			}		
	}
	
	if(scenario==3)//Bidirectional Flow
	{
		
		
			sim->addAgent(RVO::Vector2(-20.0f,  30.0f));
			sim->addAgent(RVO::Vector2(-17.0f,  30.0f));
			sim->addAgent(RVO::Vector2(-14.0f,  30.0f));
			sim->addAgent(RVO::Vector2(-20.0f,  29.0f));
			sim->addAgent(RVO::Vector2(-17.0f,  29.0f));
			
			sim->addAgent(RVO::Vector2(-14.0f,  29.0f));
			sim->addAgent(RVO::Vector2(-20.0f,  28.0f));
			sim->addAgent(RVO::Vector2(-17.0f,  28.0f));
			sim->addAgent(RVO::Vector2(-14.0f,  28.0f));
			sim->addAgent(RVO::Vector2(20.0f,  30.0f));
			
			sim->addAgent(RVO::Vector2(17.0f,  30.0f));
			sim->addAgent(RVO::Vector2(14.0f,  30.0f));
			sim->addAgent(RVO::Vector2(20.0f,  29.0f));
			sim->addAgent(RVO::Vector2(17.0f,  29.0f));
			sim->addAgent(RVO::Vector2(14.0f,  29.0f));
			
			sim->addAgent(RVO::Vector2(20.0f,  28.0f));
			sim->addAgent(RVO::Vector2(17.0f,  28.0f));
			sim->addAgent(RVO::Vector2(14.0f,  28.0f));
			
		
			
		
		
		
		for (int i = 0; i < numAgents; ++i) 
		{			
			finalGoals.push_back(RVO::Vector2(-sim->getAgentPosition(i).x(),sim->getAgentPosition(i).y() ) );
			sim->setAgentGoal(i, RVO::Vector2(-sim->getAgentPosition(i).x(),sim->getAgentPosition(i).y() ));
					
		}
	}
	
	if(scenario==4)
    {      
        sim->addAgent(RVO::Vector2(-30.0f,  0.5f));
        sim->addAgent(RVO::Vector2(-26.0f,  0.5f));
        sim->addAgent(RVO::Vector2(-22.0f,   0.5f));
        sim->addAgent(RVO::Vector2(-18.0f,   0.5f));
        sim->addAgent(RVO::Vector2(-14.0f,   0.5f));
        
        sim->addAgent(RVO::Vector2(-30.0f, -0.5f));
        sim->addAgent(RVO::Vector2(-26.0f,  -0.5f));
        sim->addAgent(RVO::Vector2(-22.0f,  -0.5f));
        sim->addAgent(RVO::Vector2(-18.0f,  -0.5f));
        sim->addAgent(RVO::Vector2(-14.0f,  -0.5f));
        
        sim->addAgent(RVO::Vector2(-30.0f, -1.5f));
        sim->addAgent(RVO::Vector2(-26.0f,  -1.5f));
        sim->addAgent(RVO::Vector2(-22.0f,  -1.5f));
        sim->addAgent(RVO::Vector2(-18.0f,  -1.5f));
        sim->addAgent(RVO::Vector2(-14.0f,  -1.5f));
        
        sim->addAgent(RVO::Vector2(-30.0f,  1.5f));
        sim->addAgent(RVO::Vector2(-26.0f,  1.5f));
        sim->addAgent(RVO::Vector2(-22.0f,   1.5f));
        sim->addAgent(RVO::Vector2(-18.0f,   1.5f));
        sim->addAgent(RVO::Vector2(-14.0f,   1.5f));
      
        sim->addAgent(RVO::Vector2(30.0f,  0.5f));
        sim->addAgent(RVO::Vector2(26.0f,  0.5f));
        sim->addAgent(RVO::Vector2(22.0f,  0.5f));
        sim->addAgent(RVO::Vector2(18.0f,  0.5f));
        sim->addAgent(RVO::Vector2(14.0f,  0.5f));
        
        
        sim->addAgent(RVO::Vector2(30.0f,  -0.5f));
        sim->addAgent(RVO::Vector2(26.0f,  -0.5f));
        sim->addAgent(RVO::Vector2(22.0f,  -0.5f));
        sim->addAgent(RVO::Vector2(18.0f,  -0.5f));
        sim->addAgent(RVO::Vector2(14.0f,  -0.5f));
        
        sim->addAgent(RVO::Vector2(30.0f,  -1.5f));
        sim->addAgent(RVO::Vector2(26.0f,  -1.5f));
        sim->addAgent(RVO::Vector2(22.0f,  -1.5f));
        sim->addAgent(RVO::Vector2(18.0f,  -1.5f));
        sim->addAgent(RVO::Vector2(14.0f,  -1.5f));
        
        sim->addAgent(RVO::Vector2(30.0f,  1.5f));
        sim->addAgent(RVO::Vector2(26.0f,  1.5f));
        sim->addAgent(RVO::Vector2(22.0f,  1.5f));
        sim->addAgent(RVO::Vector2(18.0f,  1.5f));
        sim->addAgent(RVO::Vector2(14.0f,  1.5f));
       
        sim->addAgent(RVO::Vector2(0.5f,  30.0f));
        sim->addAgent(RVO::Vector2(0.5f,  26.0f));
        sim->addAgent(RVO::Vector2(0.5f,   22.0f));
        sim->addAgent(RVO::Vector2(0.5f,   18.0f));
        sim->addAgent(RVO::Vector2(0.5f,   14.0f));
        
        sim->addAgent(RVO::Vector2(-0.5f, 30.0f));
        sim->addAgent(RVO::Vector2(-0.5f,  26.0f));
        sim->addAgent(RVO::Vector2(-0.5f,  22.0f));
        sim->addAgent(RVO::Vector2(-0.5f,  18.0f));
        sim->addAgent(RVO::Vector2(-0.5f,  14.0f));
        
        sim->addAgent(RVO::Vector2(-1.5f, 30.0f));
        sim->addAgent(RVO::Vector2(-1.5f,  26.0f));
        sim->addAgent(RVO::Vector2(-1.5f,  22.0f));
        sim->addAgent(RVO::Vector2(-1.5f,  18.0f));
        sim->addAgent(RVO::Vector2(-1.5f,  14.0f));
       
        sim->addAgent(RVO::Vector2(1.5f,  30.0f));
        sim->addAgent(RVO::Vector2(1.5f,  26.0f));
        sim->addAgent(RVO::Vector2(1.5f,   22.0f));
        sim->addAgent(RVO::Vector2(1.5f,   18.0f));
        sim->addAgent(RVO::Vector2(1.5f,   14.0f));
        
        sim->addAgent(RVO::Vector2(0.5f,  -30.0f));
        sim->addAgent(RVO::Vector2(0.5f,  -26.0f));
        sim->addAgent(RVO::Vector2(0.5f,  -22.0f));
        sim->addAgent(RVO::Vector2(0.5f,  -18.0f));
        sim->addAgent(RVO::Vector2(0.5f,  -14.0f));
        
        
        sim->addAgent(RVO::Vector2(-0.5f,  -30.0f));
        sim->addAgent(RVO::Vector2(-0.5f,  -26.0f));
        sim->addAgent(RVO::Vector2(-0.5f,  -22.0f));
        sim->addAgent(RVO::Vector2(-0.5f,  -18.0f));
        sim->addAgent(RVO::Vector2(-0.5f,  -14.0f));
        
        
        sim->addAgent(RVO::Vector2(-1.5f,  -30.0f));
        sim->addAgent(RVO::Vector2(-1.5f,  -26.0f));
        sim->addAgent(RVO::Vector2(-1.5f,  -22.0f));
        sim->addAgent(RVO::Vector2(-1.5f,  -18.0f));
        sim->addAgent(RVO::Vector2(-1.5f,  -14.0f));
        
        sim->addAgent(RVO::Vector2(1.5f,  -30.0f));
        sim->addAgent(RVO::Vector2(1.5f,  -26.0f));
        sim->addAgent(RVO::Vector2(1.5f,  -22.0f));
        sim->addAgent(RVO::Vector2(1.5f,  -18.0f));
        sim->addAgent(RVO::Vector2(1.5f,  -14.0f));
 
        
        for (int i = 0; i < numAgents; ++i)
        {
            initPos[i]=sim->getAgentPosition(i);
            
            if(i<(numAgents/4))
            {
                finalGoals.push_back(RVO::Vector2(10, sim->getAgentPosition(i).y() ));
                sim->setAgentGoal(i, RVO::Vector2(10, sim->getAgentPosition(i).y() ));
            }
            
            if((i>=(numAgents/4))&&(i<(numAgents/2))) 
            {                
                  finalGoals.push_back(RVO::Vector2(-10, sim->getAgentPosition(i).y() ));
                   sim->setAgentGoal(i, RVO::Vector2(-10, sim->getAgentPosition(i).y() ));
                
            }
            
            if((i>=(numAgents/2))&&(i<(numAgents/((float)4/(float)3) ))) 
            {
                finalGoals.push_back(RVO::Vector2(sim->getAgentPosition(i).x() ,-10));
                 sim->setAgentGoal(i, RVO::Vector2(sim->getAgentPosition(i).x() ,-10));
            }
            
            
            if((i>=(numAgents/((float)4/(float)3) ))&&(i<numAgents)) 
            {
                finalGoals.push_back(RVO::Vector2(sim->getAgentPosition(i).x() ,10));
                 sim->setAgentGoal(i, RVO::Vector2(sim->getAgentPosition(i).x() ,10));
            }
            
            
            
            
        }
        
    }
  
    if(scenario==10)
    {
		finalGoals.clear();
		sim->addAgent( RVO::Vector2(3.0f,  0.5f));
        sim->addAgent( RVO::Vector2(27.0f,  0.5f));
        
        initPos[0]= RVO::Vector2(3.0f,  0.5f);
        initPos[1]= RVO::Vector2(27.0f,  0.5f);
        
        
        finalGoals.push_back(RVO::Vector2(27.0f, 0.5f ) );
        sim->setAgentGoal(0,RVO::Vector2(27.0f, 0.5f ) );
        loop[0]=0;
        finalGoals.push_back(RVO::Vector2(3.0f, 0.5f ) );
        sim->setAgentGoal(1,RVO::Vector2(3.0f, 0.5f ) );
        loop[1]=0;
        
        
        
        sim->addAgent( RVO::Vector2(3.0f,  3.5f));
        sim->addAgent(  RVO::Vector2(27.0f,  3.5f));
        
        initPos[2]= RVO::Vector2(3.0f,  3.5f);
        initPos[3]= RVO::Vector2(27.0f,  3.5f);
        
        finalGoals.push_back(RVO::Vector2(27.0f, 3.5f ) );
        sim->setAgentGoal(2,RVO::Vector2(27.0f, 3.5f ) );
        loop[2]=0;
        finalGoals.push_back(RVO::Vector2(3.0f, 3.5f ) );
        sim->setAgentGoal(3,RVO::Vector2(3.0f, 3.5f ) );
        loop[3]=0;
        
        
        
        sim->addAgent( RVO::Vector2(3.0f,  6.5f));
        sim->addAgent(  RVO::Vector2(27.0f,  6.5f));
        
        initPos[4]= RVO::Vector2(3.0f,  6.5f);
        initPos[5]= RVO::Vector2(27.0f,  6.5f);
        
        finalGoals.push_back(RVO::Vector2(27.0f, 6.5f ) );
        sim->setAgentGoal(4,RVO::Vector2(27.0f, 6.5f ) );
        loop[4]=0;        
        finalGoals.push_back(RVO::Vector2(3.0f, 6.5f ) );
        sim->setAgentGoal(5,RVO::Vector2(3.0f, 6.5f ) );
        loop[5]=0;
        
        
        
        
        sim->addAgent( RVO::Vector2(3.0f,  9.5f));
        sim->addAgent(  RVO::Vector2(27.0f,  9.5f));
        
        initPos[6]= RVO::Vector2(3.0f,  9.5f);
        initPos[7]= RVO::Vector2(27.0f,  9.5f);
        
         finalGoals.push_back(RVO::Vector2(27.0f, 9.5f ) );
        sim->setAgentGoal(6,RVO::Vector2(27.0f, 9.5f ) );
        loop[6]=0; 
        finalGoals.push_back(RVO::Vector2(3.0f, 9.5f ) );
        sim->setAgentGoal(7,RVO::Vector2(3.0f, 9.5f ) );
        loop[7]=0;      
                
        
        
		
		
	}
		
	if(scenario==5)
    {
		
				 
			
		generateScenarioCrowd();
			
        for (int i = 0; i < numAgents; ++i)
        {
            finalGoals.push_back(RVO::Vector2(-10.0,0.0));
             sim->setAgentGoal(i,RVO::Vector2(-10.0,0.0));
            
        }
        
    }
    
    if(scenario==6)
    {
        
        sim->addAgent( RVO::Vector2(-500.0f,  -10.0f));
        sim->addAgent(  RVO::Vector2(-400.0f,  -10.0f));
        sim->addAgent( RVO::Vector2(-300.0f,  -10.0f));
        sim->addAgent( RVO::Vector2(-200.0f,  -10.0f));
        sim->addAgent( RVO::Vector2(-100.0f,  -10.0f));
        
        sim->addAgent( RVO::Vector2(-500.0f,  -9.0f));
        sim->addAgent(  RVO::Vector2(-4.0f,  -9.0f));
        sim->addAgent( RVO::Vector2(-3.0f,  -9.0f));
        sim->addAgent(  RVO::Vector2(-2.0f,  -9.0f));
        sim->addAgent(  RVO::Vector2(-1.0f,  -9.0f));
        
        sim->addAgent( RVO::Vector2(-5.0f,  -8.0f));
        sim->addAgent(  RVO::Vector2(-4.0f,  -8.0f));
        sim->addAgent(  RVO::Vector2(-3.0f,  -8.0f));
        sim->addAgent( RVO::Vector2(-2.0f,  -8.0f));
        sim->addAgent(  RVO::Vector2(-1.0f,  -8.0f));
      
        sim->addAgent(  RVO::Vector2(-3.0f,  3.0f));
        
        
        for (int i = 0; i < numAgents-1; ++i)
        {
            finalGoals.push_back(RVO::Vector2(sim->getAgentPosition(i).x(),24+sim->getAgentPosition(i).y() ) );
            sim->setAgentGoal(i, RVO::Vector2(sim->getAgentPosition(i).x(),24+sim->getAgentPosition(i).y() ) );
        }
        
        finalGoals.push_back(RVO::Vector2(sim->getAgentPosition(15).x(),-20 ) );
        sim->setAgentGoal(15, RVO::Vector2(sim->getAgentPosition(15).x(),-20 )  );
       
        
    }

    if(scenario==7)
    {
        
        sim->addAgent(  RVO::Vector2(-21.0f,  -0.0f));
        sim->addAgent( RVO::Vector2(21.0f,  0.0f));
        
        sim->addAgent(  RVO::Vector2(-23.0f,  0.0f));
        sim->addAgent(  RVO::Vector2(23.0f,  0.0f));
        
        sim->addAgent(  RVO::Vector2(-25.0f,  0.0f));
        sim->addAgent(  RVO::Vector2(25.0f,  0.0f));
        
        sim->addAgent(  RVO::Vector2(-27.0f,  0.0f));
        sim->addAgent(  RVO::Vector2(27.0f,  0.0f));
        
        sim->addAgent(  RVO::Vector2(-29.0f,  0.0f));
        sim->addAgent(  RVO::Vector2(29.0f,  0.0f));
        
        finalGoals.push_back(RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) );
        finalGoals[0]=RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) ;
        sim->setAgentGoal(0,RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) );
        
        
        finalGoals.push_back(RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        finalGoals[1]=(RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        sim->setAgentGoal(1,RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        
        finalGoals.push_back(RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) );
        finalGoals[2]=(RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) );
        sim->setAgentGoal(2,RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) );
        
        
        finalGoals.push_back(RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        finalGoals[3]=(RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        sim->setAgentGoal(3,RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        
        finalGoals.push_back(RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) );
        finalGoals[4]=(RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) );
        sim->setAgentGoal(4,RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) );
        
        
        finalGoals.push_back(RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        finalGoals[5]=(RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        sim->setAgentGoal(5,RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        
        finalGoals.push_back(RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) );
        finalGoals[6]=(RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) );
        sim->setAgentGoal(6,RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) );
        
        
        finalGoals.push_back(RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        finalGoals[7]=(RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        sim->setAgentGoal(7,RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        
        
        finalGoals.push_back(RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) );
        finalGoals[8]=(RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) );
        sim->setAgentGoal(8,RVO::Vector2(21.0f,sim->getAgentPosition(0).y() ) );
        
        finalGoals.push_back(RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        finalGoals[9]=(RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        sim->setAgentGoal(9,RVO::Vector2(-21.0f,sim->getAgentPosition(0).y() ) );
        
        
    }
    
    if(scenario==8)
    {
		
	sim->addAgent(  RVO::Vector2(-10.0f,  0.0f));	
	sim->addAgent(  RVO::Vector2(-15.0f,  5.0f));	
	sim->addAgent(  RVO::Vector2(-12.0f,  -1.0f));	
	sim->addAgent(  RVO::Vector2(-15.0f,  2.5f));	
	sim->addAgent(  RVO::Vector2(-13.0f,  -2.5f));	
	
		sim->addAgent(  RVO::Vector2(-7.0f,  0.0f));	
	sim->addAgent(  RVO::Vector2(-11.0f,  5.0f));	
	sim->addAgent(  RVO::Vector2(-10.0f,  -1.0f));	
	sim->addAgent(  RVO::Vector2(-9.0f,  2.5f));	
	sim->addAgent(  RVO::Vector2(-11.0f,  -2.5f));
	
		
	
	finalGoals.push_back(RVO::Vector2(15.0f,sim->getAgentPosition(0).y() ) );
	sim->setAgentGoal(0, RVO::Vector2(15.0f,sim->getAgentPosition(0).y() ));
	
	
	finalGoals.push_back(RVO::Vector2(15.0f,sim->getAgentPosition(1).y() ) );
	sim->setAgentGoal(1, RVO::Vector2(15.0f,sim->getAgentPosition(1).y() ));
	
	finalGoals.push_back(RVO::Vector2(15.0f,sim->getAgentPosition(2).y() ) );
	sim->setAgentGoal(2, RVO::Vector2(15.0f,sim->getAgentPosition(2).y() ));
	
	
	finalGoals.push_back(RVO::Vector2(15.0f,sim->getAgentPosition(3).y() ) );
	sim->setAgentGoal(3, RVO::Vector2(15.0f,sim->getAgentPosition(3).y() ));
	
	
	finalGoals.push_back(RVO::Vector2(15.0f,sim->getAgentPosition(4).y() ) );
	sim->setAgentGoal(4, RVO::Vector2(15.0f,sim->getAgentPosition(4).y() ));
	
	
	
	finalGoals.push_back(RVO::Vector2(15.0f,sim->getAgentPosition(5).y() ) );
	sim->setAgentGoal(5, RVO::Vector2(15.0f,sim->getAgentPosition(5).y() ));
	
	
	finalGoals.push_back(RVO::Vector2(15.0f,sim->getAgentPosition(6).y() ) );
	sim->setAgentGoal(6, RVO::Vector2(15.0f,sim->getAgentPosition(6).y() ));
	
	finalGoals.push_back(RVO::Vector2(15.0f,sim->getAgentPosition(7).y() ) );
	sim->setAgentGoal(7, RVO::Vector2(15.0f,sim->getAgentPosition(7).y() ));
	
	
	finalGoals.push_back(RVO::Vector2(15.0f,sim->getAgentPosition(8).y() ) );
	sim->setAgentGoal(8, RVO::Vector2(15.0f,sim->getAgentPosition(8).y() ));
	
	
	finalGoals.push_back(RVO::Vector2(15.0f,sim->getAgentPosition(9).y() ) );
	sim->setAgentGoal(9, RVO::Vector2(15.0f,sim->getAgentPosition(9).y() ));
	
	
	
	
	}
	
	if(scenario==9)
    {
		
		sim->addAgent( RVO::Vector2(-9.5f,  1.5f));
        sim->addAgent( RVO::Vector2(-9.5f,  2.5f));
        sim->addAgent( RVO::Vector2(-9.5f,  3.5f));
        sim->addAgent( RVO::Vector2(-9.5f,  4.5f));
       
        for (int i = 0; i < numAgents; ++i)
        {
            finalGoals.push_back(RVO::Vector2(-10.0,0.0));
             sim->setAgentGoal(i,RVO::Vector2(-10.0,0.0));
            
        }
        
    }

	for (int i = 0; i < numAgents; ++i) 
	{		
		setActions(i);
		sim->setAgentInGoal(i,0);	
	}
	
	setPreferredVelocities(algo);

   	float orcaavgtime=0;
	for (int i = 0; i < numAgents; ++i)
    {   
		loop[i]=0;
		
		orcaavgtime=orcaavgtime-0.3 + RVO::abs(finalGoals[i]-sim->getAgentPosition(i))/(float)1.5;
		
		sim->setAgentVelocity(i,sim->getAgentPrefVelocity(i));
		ori[i] = atan2(sim->getAgentVelocity(i).y(),sim->getAgentVelocity(i).x());
	}
	
	orcaavgtime=orcaavgtime/(float)numAgents;
	
	float tempstandard=0;
	for (int i = 0; i < numAgents; ++i)
    {
		energy[i]=0;
		prefVelProg[i]=0;
		energyPerProg[i]=0;
		tempstandard=tempstandard+ ((  (RVO::abs(finalGoals[i]-sim->getAgentPosition(i))/(float)1.5)  -orcaavgtime-0.3)*(  (RVO::abs(finalGoals[i]-sim->getAgentPosition(i))/(float)1.5)   -orcaavgtime-0.3));
	 

    }
    
    if(numAgents>1)
    {
      tempstandard=tempstandard/(float)(numAgents-1);
	}else{
		tempstandard=0;
	}

	tempstandard=sqrt(tempstandard);
		
	orcaavgtime=orcaavgtime+3*tempstandard;
	baseScore=orcaavgtime;
}

//******** Check if agents have reached their destination
bool reachedGoal() /* ゴールに到達しているかチェック */
{
	bool allInGoal = true;
    float currentGlobalTime = sim->getGlobalTime();
	numAgentNotInGoal = 0;

	for (size_t i = 0; i < sim->getNumAgents(); ++i) {	
		if (!isinGoal[i]){ /* まだゴール判定がなされていなければ実行 */
            float distToGoal = RVO::abs(sim->getAgentPosition(i) - finalGoals[i]);	
            if (distToGoal > 0.5f){
                //Agent is consireded to reach its goal if it is 0.5 or less meters from it
                isinGoal[i] = false;
                allInGoal = false;
                numAgentNotInGoal++;
            }else{
                numAgentInGoal++;
                isinGoal[i] = true;
                sim->setAgentInGoal(i,true);
                timeToGoal[i] = currentGlobalTime;
            }
        }	
	}
	return allInGoal;
}

//**** Update the agents in the simulation ****** //
void updateVisualization() /* エージェントを表示し、選択したアルゴリズム（ORCAまたはC-NAV）のメソッドを呼び出す */
{
    float currentGlobalTime = sim->getGlobalTime();
	
	timestep++;
	
	if((!finalize)){//To prevent iterations from occuring when the iteration is setting up
		if ((!reachedGoal())&&(timestep < threshold)){// Only enters if there are agents that have not reach the goal
			float nowTime = currentGlobalTime;
			for (size_t i = 0; i < sim->getNumAgents(); ++i) {
				needToUpdate[i]=0;				
				if(timestep > 1){
					if(((std::rand()/(float)RAND_MAX) <= updateProbability)||(timestep==2)){//If true, a new motion decision will be made, otherwise the agent keeps its previous action
						needToUpdate[i] = 1;
					}
				}	
			}

			if(algo == 1){/* ORCA */
                setPreferredVelocities(algo);
			}
			if(algo == 2){/* C-Nav */
                if(timestep > 1){
					evaluateEachAction(); 
				}
                setPreferredVelocities(algo);
			}
			
            sim->doStep(); //calls a method in RVOSimulator.cpp to update the simulation
			countCollisions();
			
            std::cout << " Real Vel: " << sim->getAgentVelocity(agentViewed) << " magnitude " << RVO::abs(sim->getAgentVelocity(agentViewed))<< " with prefVel " << sim->getAgentPrefVelocity(agentViewed) << std::endl;	
            std::cout << "Compared to Expected Vel: " << sim->getAgentInitialVelocity(agentViewed, chosenAction[agentViewed]) << " magnitude " << RVO::abs(sim->getAgentInitialVelocity(agentViewed, chosenAction[agentViewed]))<< std::endl;
            std::cout << "Compared to Expected PrefVel: " << sim->getAgentInitialPrefVelocity(agentViewed, chosenAction[agentViewed]) << " magnitude " << RVO::abs(sim->getAgentInitialPrefVelocity(agentViewed, chosenAction[agentViewed]))<< std::endl;
            std::cout << " DIFFERENCE: "<< RVO::abs(sim->getAgentInitialVelocity(agentViewed, chosenAction[agentViewed])- sim->getAgentVelocity(agentViewed)) << std::endl;

			int closest = getClosestToGoal();
			
			totalMotion += RVO::abs(sim->getAgentPrefVelocity(closest) - sim->getAgentVelocity(closest));
			totalDeviation++;
			
			for (size_t i = 0; i < sim->getNumAgents(); ++i) {
				if(!isinGoal[i]){
                    std::cout << "Agent " << i << " Real vel is " << sim->getAgentVelocity(i) << std::endl;			
					prefVelProg[i]=  sim->getAgentVelocity(i) * RVO::normalize(finalGoals[i]-sim->getAgentPosition(i));
					energy[i] = 2.25 + RVO::abs(sim->getAgentVelocity(i))*RVO::abs(sim->getAgentVelocity(i));  
					energyPerProg[i] += energy[i];
                    if(needToUpdate[i]){
                        if(RVO::abs(sim->getAgentInitialVelocity(i, chosenAction[i]) - sim->getAgentVelocity(i))>0.5){
						    std::cout << "(" << i << ") Cant trust prediction, switching to execution-based reward at t " << timestep <<"\n";	
						}
					}
				}		
			}
		}
        else{ /* 全てのエージェントがゴールに達した場合 */	
            finalize=true;		      
            if(timestep == threshold){   
                notFinished++;
                finalTime = 0; 
            }else{
				float totalTimeToGoal=0;  
                totalAvgEnergy = 0; 
                for (size_t i = 0; i < sim->getNumAgents(); ++i){ 
				    avgEnergy[i]=0;
					if(isinGoal[i]){
                        totalTimeToGoal += timeToGoal[i];  //timeToGoal[i] is the time agent i reached its goal
                        avgEnergy[i] = energyPerProg[i];
                        totalAvgEnergy += avgEnergy[i];
					}else{
                        timeToGoal[i] = currentGlobalTime;
                        totalTimeToGoal +=	timeToGoal[i];
					}
				}
				
                totalRewardSimulation = totalTimeToGoal/(float)numAgents;

				float AvgStd = 0;
                float tempStd=0;
				AvgStd += totalRewardSimulation;
            
				for (size_t i = 0; i < sim->getNumAgents(); ++i){
					tempStd += ((timeToGoal[i] - totalRewardSimulation) * (timeToGoal[i] - totalRewardSimulation));		
				}
            
				if(numAgents>1){
				    tempStd /= (float)(numAgents-1);
				}else{
				    tempStd = 0;
				}

				tempStd = sqrt(tempStd);
				AvgStd += 3 * tempStd;
				finalTime = AvgStd;
				SimScore += finalTime;  
				totalAvgEnergy /= (float)numAgents;
			}
		}
	}
}



float crowd_simulation_eval(const float* actionsDir, const float* actionsMag, int numActions)
{ /* 実験の各反復に対してアクションとRVOパラメータを設定する */
	Actions = numActions;
	for (int i = 0; i < Actions; i++){
		actionVector[i] = actionsDir[i];
		actionVectorMag[i] = actionsMag[i];
	}
	sim = new RVO::RVOSimulator();
	iteration = 1;
	setupScenario(sim);
	do {
        updateVisualization();
    }while(!finalize); 
	
    delete sim;
	return finalTime;
}



int main(int argc, char* argv[])
{
    
	algo = atoi(argv[1]);
	int hoge = atoi(argv[2]);
	visualizer= atoi(argv[3]);
	finalIteration= atoi(argv[4]);
	coordFactor= atof(argv[5]);
	allNeigh=atoi(argv[6]); // all neighbors(1) or just closer to goal (0)
	contadourX=atoi(argv[7]);
	followNeighbors=atoi(argv[8]);
    
    srand(time(NULL));
      
    ////////////////////////////////////////////////////////////////////
    //////Sample actions: 8 velocities whose direction is in radians (velDir) and the magnitudes are in meters per second (velMag)
    
    float velDir[9] = {0.00000, 1/4 * M_PI, 1/2 * M_PI, 3/4 * M_PI, M_PI, -3/4 * M_PI, -1/2 * M_PI, -1/4 * M_PI, 0.00000};
    float velMag[9]= {1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5};
   
    int accountActions;
    if(followNeighbors){
		accountActions=9;
	}else{
		accountActions=8;		
	}
   
    float Result[400];
    float resultEnergy[400];
    notFinished = 0;

    float sumTotalEnergies=0;
    for(int i = 0; i < finalIteration; i++){//The code below is executed for each iteration
        finalize = 0;
        switch(hoge){
            case 1: numAgents = 300, scenario = 1;
                break;
            case 2: numAgents = 128, scenario = 2;
                break;  
            case 3: numAgents=18, scenario=3;
                break;    
            case 4: numAgents=80, scenario=4;
                break;         
            case 5: numAgents=64, scenario=5;
                break;             
            case 6: numAgents=16, scenario=6;
                break;
            case 7: numAgents=10, scenario=7;
                break;
            case 8: numAgents=10, scenario=8;
                break;
            case 9: numAgents=4, scenario=9;
                break;
            case 10:numAgents=8,scenario=10;
                break;
        }
        Result[i] = crowd_simulation_eval(velDir, velMag, accountActions);
        resultEnergy[i] = totalAvgEnergy;
        sumTotalEnergies +=totalAvgEnergy;
    }
    
    float ResultStd = 0.0f;
    float ResultTotal = 0.0f;
    float ResultEnergyStd = 0.0f;
  
    for(int i = 0; i < finalIteration; i++){
		if(Result[i] > 0){
			ResultEnergyStd += ((resultEnergy[i] - (sumTotalEnergies/((float)finalIteration-notFinished))) * (resultEnergy[i] - (sumTotalEnergies/((float)finalIteration-notFinished))));
			ResultStd += ((Result[i] - (SimScore/((float)finalIteration-notFinished)))* (Result[i] - (SimScore/((float)finalIteration-notFinished))));
			ResultTotal += Result[i];
		}                
    }
					
	ResultEnergyStd /= (float(finalIteration-notFinished - 1));
	ResultEnergyStd /= sqrt(float(ResultEnergyStd));
	ResultEnergyStd /= sqrt(float(finalIteration - notFinished));
	ResultStd /= float(finalIteration - notFinished - 1);
	ResultStd = sqrt(ResultStd);
	ResultStd /= sqrt(float(finalIteration-notFinished));
    
    //Output the average interaction overhead accross all iterations, its std dev, the percentage of goal reachibility, the average of the energy computation and its standard deviation 
   	std::cout << "AvgIO: " << (SimScore/((float)finalIteration-notFinished)) -baseScore<< ", StdDevIO: " << ResultStd<< ", " << 100*(float)(finalIteration-notFinished)/(float)finalIteration << "% goal reachibility, AvgEnergy " << (float)sumTotalEnergies/(float)finalIteration << " StdDevEnergy: " <<ResultEnergyStd << std::endl;
   	return 0;
}