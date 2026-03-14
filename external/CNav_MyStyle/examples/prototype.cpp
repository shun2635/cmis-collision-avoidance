#include <iostream>
#include <vector>
#include <fstream>
#include <stdlib.h>
#include <math.h>
#include <cmath>
#include <vector>
#include <cstdlib>
#include <time.h>
#include <string.h>
#include <algorithm>
#include "simulationDefs.h"


#include "/Users/tamurashuntarou/laboratory_code/CNav/src/RVO.h"

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

void generateScenarioCrowd(std::ofstream& initPositionsFile) /* 群衆シナリオにおけるエージェントの位置を生成する */
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

void computeSimilarAgentsDir(int myID, std::ofstream &outputFile)
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
		
        outputFile << "Agent " << you << " has GoalDist " << GoalDist << " and prefVelocitySimilarity " << prefVelocitySimilarity <<  " cause my goalVpref is " << myGoalPrefVelocity <<" \n";
			
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
			outputFile << " The most similar is... " << mostSimilarAgentID[myID] << " with an angle of " << aver << std::endl;	
		}
	}
}

float computeSimilarAgentSpeed(int similar, RVO::Vector2 goalVector) /*  */
{	
	return (goalVector * sim->getAgentVelocity(similar));
}

void evaluateEachAction(std::ofstream &outputFile)  /* timeHorizonで定義された将来のタイムステップ数にわたり、各action/velocityについてシミュレーションする */
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
					computeSimilarAgentsDir(i,outputFile);
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
                            outputFile << "************ " << e << " similar neigh is : " << ascendingSimilarAgentIDList[i][e] << " and value is " <<rewardFollow << "\n";
					        if(rewardFollow>bestFollowMove){
						        bestSimilarNeigh[i]= ascendingSimilarAgentIDList[i][e];
						        bestFollowMove = rewardFollow;
					        }
				        }
	
                        ActionDirection[i][Actions-1]= getOriSimilarNeighbor(i,bestSimilarNeigh[i]);
                        ActionSpeed[i][Actions-1]=computeSimilarAgentSpeed(bestSimilarNeigh[i],goalVectors[i]);
                        totalReward[i][Actions-1]= bestFollowMove;

                        outputFile << "************ Best Similar neigh is : " << bestSimilarNeigh[i] << " and value is " <<bestFollowMove << " Dir " <<ActionDirection[i][Actions-1] << " Mag: " <<ActionSpeed[i][Actions-1] <<  "\n";
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
                        outputFile << " Action " << action << " value: " << 	totalReward[i][action] << " goalProg: " << sim->getAgentGoalProg(i,action) << " CNAVPol: " << sim->getAgentCnavPoliteness(i,action) /*<< " ALANPol: " <<sim->getAgentALANPoliteness(i,ac)*/ << "\n";
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
                outputFile << " CHOSEN ACTION: " << chosenAction[i] << "\n";	
		    }		
		} 
    }
}

void setActions(int myID, std::ofstream &outputFile)  /* エージェントの行動をセットする */
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
        outputFile << "Default value action " << action << " is " << defaultValue[myID][action] << std::endl;		  		 
		sumA += defaultValue[myID][action];
		Boltz[myID][action] = 100 * (exp(defaultValue[myID][action]/0.05f))/sumA;
    }
    outputFile << "SUM " << sumA << " AVG " << sumA / (float)Actions << std::endl;
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

void setupScenario(RVO::RVOSimulator* sim, std::ofstream &outputFile) //Initialize the Simulation, positions of the obstacles and the agents
{
	sim->setTimeStep(simTimeStep);
	
	numAgentInGoal = 0;
	timestep = 0;
	totalRewardSimulation = 0;
  	
	for (size_t i = 0; i < numAgents; ++i) {
		needToUpdate[i] = 0.0f;
		chosenAction[i] = 0.0f;
		currentVel[i] = 0.0f;
		evalMethod[i] = false; /* false:シミュレートされた報酬を使用, true:実行に基づく報酬を使用 */
		timeToGoal[i] = 0.0f;
		isinGoal[i] = 0.0f;

		for (int j = 0; j < Actions; j++){
            totalReward[i][j]=0;
			LastActionEstimate[i][j]=0;
		}						
		
		if(algo == 1){ /* ORCAの場合 */
			ActionSpeed[i][0] = 1.5f;
			ActionDirection[i][0] = 0.0f;
			chosenAction[i] = 0.0f;
		}	
	}
	
	// Adding Obstacles for each scenario
	std::vector<RVO::Vector2> obstacle1, obstacle2, obstacle3, obstacle4,obstacle5, obstacle6, obstacle7, obstacle8, obstacle9, obstacle10, obstacle11, obstacle12, obstacle13;

    /* エージェント追加 */
    sim->setAgentDefaults(neighborDistance, numNeighbors, timeHorizonORCA , timeHorizonObstORCA , radiusORCA , maxSpeedORCA);
    for (size_t i = 0; i < numAgents; ++i){
		sim->addAgent(float(numAgents) * RVO::Vector2(std::cos(i * 2.0f * M_PI / float(numAgents)) + (std::rand() * 0.01f /(float)RAND_MAX),
                                                    std::sin(i * 2.0f * M_PI / float(numAgents)) + (std::rand() * 0.01f /(float)RAND_MAX)));
		finalGoals.push_back(-sim->getAgentPosition(i));
		sim->setAgentGoal(i, -sim->getAgentPosition(i));	
		setActions(i,outputFile);
		sim->setAgentInGoal(i,false);	
	}
	
	setPreferredVelocities(algo);

   	float orcaAverageTime = 0;
    for (size_t i = 0; i < sim->getNumAgents(); ++i){   
		loop[i] = 0;
		orcaAverageTime -= 0.3 + RVO::abs(finalGoals[i]-sim->getAgentPosition(i))/1.5f;
		sim->setAgentVelocity(i, sim->getAgentPrefVelocity(i));
		ori[i] = atan2(sim->getAgentVelocity(i).y(),sim->getAgentVelocity(i).x());
	}
	orcaAverageTime = orcaAverageTime/(float)numAgents;
	
	float tempStandard = 0;
    for (size_t i = 0; i < sim->getNumAgents(); ++i){
		energy[i] = 0;
		prefVelProg[i] = 0;
		energyPerProg[i] = 0;
		tempStandard = tempStandard + (((RVO::abs(finalGoals[i] - sim->getAgentPosition(i))/ 1.5f) - orcaAverageTime - 0.3) * ((RVO::abs(finalGoals[i] - sim->getAgentPosition(i))/1.5f) - orcaAverageTime - 0.3));
    }

    if(numAgents > 1){
        tempStandard = tempStandard/(float)(numAgents-1);
	}else{
		tempStandard = 0;
	}
	tempStandard = sqrt(tempStandard);
	orcaAverageTime = orcaAverageTime + 3 * tempStandard;
	baseScore = orcaAverageTime;
}

bool reachedGoal(){ /* ゴールに到達しているかチェック */
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

void updateVisualization(std::ofstream &outputFile) /* エージェントを表示し、選択したアルゴリズム（ORCAまたはC-NAV）のメソッドを呼び出す */
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
					evaluateEachAction(outputFile); 
				}
                setPreferredVelocities(algo);
			}
			
            sim->doStep(); //calls a method in RVOSimulator.cpp to update the simulation
			
            countCollisions();
			
            outputFile << "Real Vel: " << sim->getAgentVelocity(agentViewed) << " magnitude " << RVO::abs(sim->getAgentVelocity(agentViewed))<< " with prefVel " << sim->getAgentPrefVelocity(agentViewed) << std::endl;	
            outputFile << "Compared to Expected Vel: " << sim->getAgentInitialVelocity(agentViewed, chosenAction[agentViewed]) << " magnitude " << RVO::abs(sim->getAgentInitialVelocity(agentViewed, chosenAction[agentViewed]))<< std::endl;
            outputFile << "Compared to Expected PrefVel: " << sim->getAgentInitialPrefVelocity(agentViewed, chosenAction[agentViewed]) << " magnitude " << RVO::abs(sim->getAgentInitialPrefVelocity(agentViewed, chosenAction[agentViewed]))<< std::endl;
            outputFile << "DIFFERENCE: "<< RVO::abs(sim->getAgentInitialVelocity(agentViewed, chosenAction[agentViewed])- sim->getAgentVelocity(agentViewed)) << std::endl;
            outputFile << std::endl;

			int closest = getClosestToGoal();
			
			totalMotion += RVO::abs(sim->getAgentPrefVelocity(closest) - sim->getAgentVelocity(closest));
			totalDeviation++;
			
			for (size_t i = 0; i < sim->getNumAgents(); ++i) {
				if(!isinGoal[i]){
                    outputFile << "Agent " << i << " Real vel is " << sim->getAgentVelocity(i) << std::endl;			
					prefVelProg[i]=  sim->getAgentVelocity(i) * RVO::normalize(finalGoals[i]-sim->getAgentPosition(i));
					energy[i] = 2.25 + RVO::abs(sim->getAgentVelocity(i))*RVO::abs(sim->getAgentVelocity(i));  
					energyPerProg[i] += energy[i];
                    if(needToUpdate[i]){
                        if(RVO::abs(sim->getAgentInitialVelocity(i, chosenAction[i]) - sim->getAgentVelocity(i))>0.5){
						    outputFile << "(" << i << ") Cant trust prediction, switching to execution-based reward at t " << timestep <<"\n";	
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

float crowdSimulationEval(const float* actionsDir, const float* actionsMag, int numActions, std::ofstream &outputFile)
{ /* 実験の各反復に対してアクションとRVOパラメータを設定する */
	Actions = numActions;
	for (int i = 0; i < Actions; i++){
		actionVector[i] = actionsDir[i];
		actionVectorMag[i] = actionsMag[i];
	}
	sim = new RVO::RVOSimulator();
	iteration = 1;
	setupScenario(sim, outputFile);
	do {
        updateVisualization(outputFile);
    }while(!finalize); 
	
    delete sim;
	return finalTime;
}

int main()
{
    std::ofstream outputFile;
    outputFile.open("/Users/tamurashuntarou/Downloads/CML/results/CNav/circle.csv");
    
    algo = 2;
    /* 1.ORCA, 2.C-Nav */
    scenario = 2;
    /*scenario: 1.- Crowd,          2.- Circle,     3.- Bidirectional
                4.- PerpCrossing,   5.- Congested,  6.- Crossing
                7.- Deadlock,       8.- Blocks,     9.- Line              */
    numAgents = 5;
	visualizer = 0; /* 0.Off, 1.On */
	finalIteration = 10;
	coordFactor = 0.1f; /* 0(自分の目標達成) - 1(相手への礼儀) */
	allNeigh = 0; /* 全てのエージェント(1) or ゴールに近い側だけ(0) */
	contadourX = 1;
	followNeighbors = false; /* 追従行動の有無(false:禁止, true:可能) */
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
        Result[i] = crowdSimulationEval(velDir, velMag, accountActions, outputFile);
        resultEnergy[i] = totalAvgEnergy;
        sumTotalEnergies += totalAvgEnergy;
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
   	std::cout << "AvgIO: " << (SimScore/((float)finalIteration-notFinished)) -baseScore<< ", StdDevIO: " << ResultStd<< ", " << 100 * (float)(finalIteration-notFinished)/(float)finalIteration << "% goal reachibility, AvgEnergy " << (float)sumTotalEnergies/(float)finalIteration << " StdDevEnergy: " <<ResultEnergyStd << std::endl;
   	return 0;
}
