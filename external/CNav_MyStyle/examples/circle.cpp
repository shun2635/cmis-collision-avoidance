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
#include "circle.h"


#include "/Users/tamurashuntarou/laboratory_code/CNav/src/RVO.h"

#ifndef M_PI
static const float M_PI = 3.14159265358979323846f;
#endif
#include <string>
#include <sstream>

//**************Default Simulation Parameters*********************//

float updateProbability = 0.25; // In average, each agent makes decision every (1/updateProbability) timesteps (directly dep in timestep length) 
float simTimeStep = 0.05; // Length of the timestep, in each second there are (1/simTimeStep) timesteps
int sideSize = 25; //Size of each side of the environment of the Crowd scenario
bool randomPert = 1; //Enables small random perturbations on the preferred velocities computed

const int numNeighbors = 3;//考慮する近傍エージェントの数, エージェント全体の数より多いとバグる。要改良。
const float neighborDistance=15.0f; //max distance that agents can perceive neighbors
const float timeHorizonORCA=5.0f;  // time horizon to determine collisions with other agents
const float timeHorizonObstORCA=1.3f;// time horizon to determine collisions with obstacles
const float radiusORCA=0.5f;  // distance that the agents want to keep from other agents
const float maxSpeedORCA=1.5f; //maximum speed that agents can move with

float ooparts = 0.4; 
static const int lengthSimulateTimeSteps = 2; //Number of (future) timesteps to simulate to evaluate the effect of each action

//General variable declaration
int chosenAction[320];
int currentVel[320];
int totalConsideredNeighbors[320];
int mostSimilarAgentID[320];
int ascendingSimilarAgentIDList[320][50];
int lastFrameTime = 0;

RVO::Vector2 goalVectors[320];
RVO::Vector2 initPos[320];
std::vector<RVO::Vector2> finalGoals;

float actionVector[50];
float actionVectorMag[50];
float totalReward[320][50];
float timeToGoal[320];
float ActionSpeed[320][50];
float ActionDirection[320][50];
float totalRewardIteration;
float defaultValue[320][20];
//**************Default Simulation Parameters*********************//

int countCollisions(RVO::RVOSimulator* sim, int currentCollisions, std::vector<bool> isInGoal) /* 衝突判定をし、カウントを増やす */
{
    int nextCollisions = currentCollisions;
	/* ゴールに入っていないエージェントi,jに対して実行 */
    for (size_t i = 0; i < sim->getNumAgents(); ++i){
        for(size_t j = 0; j < sim->getNumAgents(); ++j){
            if(!(isInGoal[i]) && !(isInGoal[j]) && (i != j)){ 
                /* エージェント間の距離が半径の和より小さければ衝突カウントを増やす */
                float distance = RVO::abs(sim->getAgentPosition(i)-sim->getAgentPosition(j));
				float sumOfRadius =  sim->getAgentRadius(i) + sim->getAgentRadius(j);
                if(distance < sumOfRadius){	
                    nextCollisions++;
                }
            }
	    }
    }
	return nextCollisions;
} 

int getClosestToGoal(RVO::RVOSimulator* sim, std::vector<bool> isInGoal) /* ゴールに到達していないエージェントの中で、最もゴールに近いエージェントを返す */
{	
	float minDistanceToGoal = 10000.0f;
    float distToGoal;
	int closestID = -1;		
    for (int i = 0; i < sim->getNumAgents(); ++i) {
		if(!isInGoal[i]){
			distToGoal= RVO::abs(finalGoals[i] - sim->getAgentPosition(i));
			if(distToGoal < minDistanceToGoal){
				minDistanceToGoal = distToGoal;
				closestID = i;
			}	
		}	
	}
	return closestID;
}

bool checkOverlap(RVO::RVOSimulator* sim, size_t myID)  /* エージェントiの位置が、以前に位置したエージェントの位置と重なるかどうかをチェックする(その場合はfalseを返す)*/
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

float getOriSimilarNeighbor(RVO::RVOSimulator* sim, int myID, int yourID) /*  */
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

void computeSimilarAgentsDir(RVO::RVOSimulator* sim, int myID, std::ofstream &outputFile, std::vector<bool> isInGoal) /* 近傍の類似エージェントを探して記録する */
{
	std::vector< std::pair<float, int> > innerList;
    RVO::Vector2 myPosition = sim->getAgentPosition(myID);
    RVO::Vector2 myGoal = sim->getAgentGoal(myID);
    RVO::Vector2 myGoalPrefVelocity = sim->getAgentGoalPrefVelocity(myID);
	RVO::Vector2 myNormalizeGoalVector = RVO::normalize(goalVectors[myID]);
    size_t you;
    RVO::Vector2 yourPosition;
    RVO::Vector2 yourVelocity;
    RVO::Vector2 yourPrefVelocity;

	/* 1. 近傍の類似エージェントを数えて 自分の totalConsideredNeighbors にセット */
	/* 2. 自分のゴールベクトルと相手の速度の内積をvectorListに追加 */
	totalConsideredNeighbors[myID] = 0;
	for(int yourID = 0; yourID < sim->getAgentNumAgentNeighbors(myID); yourID++){
		you = sim->getAgentAgentNeighbor(myID,yourID);
        yourPosition = sim->getAgentPosition(you);
        yourVelocity = sim->getAgentVelocity(you);
        yourPrefVelocity = sim->getAgentPrefVelocity(you);

        bool nearThanMe = (RVO::abs(myGoal - myPosition) > RVO::abs(myGoal - yourPosition));
		float prefVelocitySimilarity = myGoalPrefVelocity * yourPrefVelocity;
			
		if((!isInGoal[you]) && (nearThanMe) && (prefVelocitySimilarity > 0)){ //ここでは、ゴールに近いエージェントのみを類似性の対象とする
            if((myNormalizeGoalVector * yourVelocity) > 0 ){ //自分のゴールと同じ方向に向かっていたら実行
                innerList.push_back(std::make_pair(myNormalizeGoalVector * yourVelocity / 1.5f , you ));
                totalConsideredNeighbors[myID]++;		   
            }
		}	
	}

	/* 内積で昇順にソート */
	sort(innerList.begin(),innerList.end());		
    
	/* 類似エージェントIDをリストとして記録 */
	for (int i = 0; i < totalConsideredNeighbors[myID]; i++){ 	
		if (innerList.size() > 0){
			int thiSimilarAgentInnerID = innerList[i].second;		
			mostSimilarAgentID[myID] = thiSimilarAgentInnerID;
			ascendingSimilarAgentIDList[myID][i] = thiSimilarAgentInnerID;         
		}else{
			mostSimilarAgentID[myID] = -1;
			ascendingSimilarAgentIDList[myID][i] = -1;
		}
    }
}

void evaluateEachAction(RVO::RVOSimulator* sim, std::ofstream &outputFile, std::vector<bool> isInGoal,
						float coordFactor, int allNeigh, int contadourX, int numActions, bool followNeighbors)  /* timeHorizonで定義された将来のタイムステップ数にわたり、各action/velocityについてシミュレーションする */
{
    sim->buildTree();
	//std::cout << "buildTree:completed" << std::endl;
    sim->setVvalues(); /* for (int i = 0; i < static_cast<int>(agents_.size()); ++i){
							agents_[i]->Vposition_ = agents_[i]->position_;
							agents_[i]->Vvelocity_ = agents_[i]->velocity_;		
						} */
	//std::cout << "setVvalues:completed" << std::endl;
    int maxEvaluateActions = numActions;
    
	/* 各エージェントに対して実行 */
	for (int myID = 0; myID < static_cast<int>(sim->getNumAgents()); ++myID){
		//std::cout << "for ID:" << myID << std::endl;
		
		float finalReward = 0.0f;
		float finalAction = 0.0f;
		
		/* 追従行動について、シミュレーション+報酬計算 */
		if (followNeighbors){ /* 追従OKなら実行 */
			/* 類似エージェントの順位表を作成 */
			/* totalConsideredNeighbors, mostSimilarAgentID, ascendingSimilarAgentIDList */
			computeSimilarAgentsDir(sim,myID,outputFile,isInGoal);
			maxEvaluateActions = numActions - 1;

			if(totalConsideredNeighbors[myID] < 1){ /* 追従対象が存在しない場合 */
				ActionDirection[myID][numActions-1] = 0.0000f;
				ActionSpeed[myID][numActions-1] = 1.5f;
				totalReward[myID][numActions-1] = -1000.0f;
			}else{ /* 追従対象が存在する場合 */
				std::vector<int> mostSimilarNeighbor(sim->getNumAgents(),-1);
				float bestFollowMove = -100;
				float rewardFollow = -1;
				/* 近傍エージェントに対して実行 */
				for (int thisNeighborID = 0; thisNeighborID < totalConsideredNeighbors[myID]; thisNeighborID++) {
					/* ActionのDirection,Speedの最後 */
					ActionDirection[myID][numActions-1]= getOriSimilarNeighbor(sim,myID,ascendingSimilarAgentIDList[myID][thisNeighborID]);
					ActionSpeed[myID][numActions-1] = goalVectors[myID] * sim->getAgentVelocity(ascendingSimilarAgentIDList[myID][thisNeighborID]);
					/* 追従行動について、シミュレーション+報酬計算 */
					rewardFollow = sim->SimulateVelocity(myID, finalGoals[myID], lengthSimulateTimeSteps, numActions - 1, numNeighbors, sim->getNumAgents(), coordFactor, ooparts, ActionSpeed[myID][numActions-1],ActionDirection[myID][numActions-1], allNeigh, contadourX);
					/* 報酬を更新 */
					if(rewardFollow > bestFollowMove){
						mostSimilarNeighbor[myID] = ascendingSimilarAgentIDList[myID][thisNeighborID];
						bestFollowMove = rewardFollow;
					}
				}
				ActionDirection[myID][numActions-1] = getOriSimilarNeighbor(sim, myID, mostSimilarNeighbor[myID]);
				ActionSpeed[myID][numActions-1] = goalVectors[myID] * sim->getAgentVelocity(mostSimilarNeighbor[myID]);
				totalReward[myID][numActions-1] = bestFollowMove;
			}
		}

		//std::cout << "followSimulation:completed" << std::endl;

		/* 追従以外の行動について、シミュレーション+報酬計算 */
		for (int thisAction = 0; thisAction < maxEvaluateActions; thisAction++){   
			//std::cout << "action:" << thisAction << std::endl;
			totalReward[myID][thisAction] = 0.0f;
			totalReward[myID][thisAction] = sim->SimulateVelocity(myID, finalGoals[myID], lengthSimulateTimeSteps, thisAction, numNeighbors, sim->getNumAgents(), coordFactor, ooparts,ActionSpeed[myID][thisAction],ActionDirection[myID][thisAction], allNeigh, contadourX);
		}

		//std::cout << "mainSimulation:completed" << std::endl;
				
		/* 報酬最大のactionを選択 */
		finalReward = -1000.0f;
		float totalCNavPoliteness = 0.0f;
		for (int thisAction = 0; thisAction < numActions; thisAction++){		
			totalCNavPoliteness += sim->getAgentCnavPoliteness(myID,thisAction);        
			if(totalReward[myID][thisAction] >= finalReward){
				finalReward = totalReward[myID][thisAction];
				finalAction = thisAction;				
			}
		}
		chosenAction[myID] = finalAction;            
		outputFile << " CHOSEN ACTION: " << chosenAction[myID] << std::endl;				 
    }
}

void setActions(RVO::RVOSimulator* sim, int myID, std::ofstream &outputFile, float coordFactor, int numActions)  /* エージェントの行動をセットする */
{
    float noVector;
    float fullVector;
    float sumA = 0;
    RVO::Vector2 myNormalizeGoalVector = RVO::normalize(goalVectors[myID]);
    
    for(int action = 0; action < numActions; action++){
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
	}
    outputFile << "SUM " << sumA << " AVG " << sumA / (float)numActions << std::endl;
}

void setPreferredVelocities(RVO::RVOSimulator* sim, int algorithm, std::vector<bool> isInGoal) /* 選択されたアクションに基づいてエージェントの優先速度をセットする */
{	
    for (size_t i = 0; i < sim->getNumAgents(); ++i) {
		if(isInGoal[i] == false){ /* ゴール未到達エージェントに対して実行 */
            
			RVO::Vector2 myPrefVelocity = sim->getAgentPrefVelocity(i);
            RVO::Vector2 myGoalVector = finalGoals[i] - sim->getAgentPosition(i);
            
            goalVectors[i] = RVO::normalize(myGoalVector);
        
            sim->setAgentGoalPrefVelocity(i, goalVectors[i] * 1.5);
			
			if(algorithm == 1){ /* ORCAの場合 */	
				float angle = std::rand() * 2.0f * M_PI / RAND_MAX;
				float dist = std::rand() * 0.01f / RAND_MAX;
				sim->setAgentPrefVelocity(i, myPrefVelocity + dist * RVO::Vector2(std::cos(angle), std::sin(angle)));
			}

			if(algorithm == 2){ /* C-Navの場合 */
				float angle =  ActionDirection[i][chosenAction[i]];
				float dist = std::rand() * 0.01f / RAND_MAX;
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
			sim->setAgentVelocity(i, RVO::Vector2(0.0f, 0.0f));		
		}	
	}	
}

void setupScenario(RVO::RVOSimulator* sim, std::ofstream &outputFile, int numAgents, int algorithm,
					float coordFactor, std::vector<bool> isInGoal, int numActions) //Initialize the Simulation, positions of the obstacles and the agents
{
	sim->setTimeStep(simTimeStep);
  		
	// Adding Obstacles for each scenario
	std::vector<RVO::Vector2> obstacle1, obstacle2, obstacle3, obstacle4,obstacle5, obstacle6, obstacle7, obstacle8, obstacle9, obstacle10, obstacle11, obstacle12, obstacle13;

    /* エージェント追加 */
    sim->setAgentDefaults(neighborDistance, numNeighbors, timeHorizonORCA , timeHorizonObstORCA , radiusORCA , maxSpeedORCA);
    for (size_t i = 0; i < numAgents; ++i){
		sim->addAgent(100.0f * RVO::Vector2(std::cos(i * 2.0f * M_PI / float(numAgents)) + (std::rand() * 0.01f /(float)RAND_MAX),
                                                    std::sin(i * 2.0f * M_PI / float(numAgents)) + (std::rand() * 0.01f /(float)RAND_MAX)));
		finalGoals.push_back(-sim->getAgentPosition(i));
		sim->setAgentGoal(i, -sim->getAgentPosition(i));	
		setActions(sim, i, outputFile, coordFactor, numActions);
		sim->setAgentInGoal(i,false);
	
		chosenAction[i] = 0.0f;
		currentVel[i] = 0.0f;
		timeToGoal[i] = 0.0f;
		isInGoal[i] = 0.0f;

		for (int j = 0; j < numActions; j++){
            totalReward[i][j] = 0;
		}						
		
		if(algorithm == 1){ /* ORCAの場合 */
			ActionSpeed[i][0] = 1.5f;
			ActionDirection[i][0] = 0.0f;
			chosenAction[i] = 0.0f;
		}
	}
	
	setPreferredVelocities(sim, algorithm, isInGoal);
		

   	float orcaAverageTime = 0;
	float tempStandard = 0;
    
	for (size_t i = 0; i < sim->getNumAgents(); ++i){   
		orcaAverageTime += RVO::abs(finalGoals[i]-sim->getAgentPosition(i))/1.5f - 0.3f;
		sim->setAgentVelocity(i, sim->getAgentPrefVelocity(i));
	}
	
	orcaAverageTime = orcaAverageTime/(float)numAgents;
	
    for (size_t i = 0; i < sim->getNumAgents(); ++i){

		tempStandard += (((RVO::abs(finalGoals[i] - sim->getAgentPosition(i))/ 1.5f) - orcaAverageTime - 0.3) * ((RVO::abs(finalGoals[i] - sim->getAgentPosition(i))/1.5f) - orcaAverageTime - 0.3));
    }

    if(numAgents > 1){
        tempStandard = tempStandard/(float)(numAgents-1);
	}else{
		tempStandard = 0;
	}
	tempStandard = sqrt(tempStandard);
	orcaAverageTime = orcaAverageTime + 3 * tempStandard;
}

bool reachedGoal(RVO::RVOSimulator* sim, std::vector<bool> isInGoal){ /* ゴールに到達しているかチェック */
	bool allInGoal = true;
    float currentGlobalTime = sim->getGlobalTime();

	for (size_t i = 0; i < sim->getNumAgents(); ++i) {	
		if (!isInGoal[i]){ /* まだゴール判定がなされていなければ実行 */
            float distToGoal = RVO::abs(sim->getAgentPosition(i) - finalGoals[i]);	
            if (distToGoal > 0.5f){
                //Agent is consireded to reach its goal if it is 0.5 or less meters from it
                isInGoal[i] = false;
                allInGoal = false;
            }else{
                isInGoal[i] = true;
                sim->setAgentInGoal(i,true);
                timeToGoal[i] = currentGlobalTime;
            }
        }	
	}
	return allInGoal;
}

bool judgeIfFinalize(RVO::RVOSimulator* sim, std::vector<bool> isInGoal, int timeStep, int timeLimit) /* シミュレーションがまだ実行されている場合、終了条件を満たしているかをboolで返す */
{
	bool finalize = true;
	if ((!reachedGoal(sim, isInGoal)) && (timeStep < timeLimit)){
		finalize = false;
	}
	return finalize;
}

std::vector<float> setNeedToUpdate(RVO::RVOSimulator* sim, int timeStep) /* timeStepを見てneedToUpdateを返す */
{
	std::vector<float> needToUpdate;
	for (size_t i = 0; i < sim->getNumAgents(); ++i){
		needToUpdate.push_back(0);
		if((timeStep > 1) && ((std::rand()/(float)RAND_MAX) <= updateProbability) || (timeStep == 2)){ //If true, a new motion decision will be made, otherwise the agent keeps its previous action				
			needToUpdate.push_back(1);
		}	
	}
	return needToUpdate;
}

void recordPosition(RVO::RVOSimulator* sim, std::ofstream &outputFile, int interval)
{
	int currentTime = sim->getGlobalTime();
	if (currentTime % interval == 0){
		outputFile << "[" << sim->getGlobalTime();
		for (size_t i = 0; i < sim->getNumAgents(); ++i) {
			outputFile << "," << sim->getAgentPosition(i).x() << "," << sim->getAgentPosition(i).y();
		}
		outputFile << "]," << std::endl;
	}
}


int main()
{
    /* -------------Setting----------------- */
    
    int algorithm = 2; /* 1.ORCA, 2.C-Nav */
    int scenario = 2; /*scenario: 	1.- Crowd,          2.- Circle,     3.- Bidirectional
                					4.- PerpCrossing,   5.- Congested,  6.- Crossing
                					7.- Deadlock,       8.- Blocks,     9.- Line         */
    const int numAgents = 5;
	const bool visualizer = false; /* 0.Off, 1.On */
	const int finalIteration = 10;
	float coordFactor = 0.1f; /* 0(自分の目標達成) - 1(相手への礼儀) */
	int allNeigh = 0; /* 全てのエージェント(1) or ゴールに近い側だけ(0) */
	int contadourX = 1;
	bool followNeighbors = false; /* 追従行動の有無(false:禁止, true:可能) */
    srand(time(NULL));
      
    ////////////////////////////////////////////////////////////////////
    //////Sample actions: 8 velocities whose direction is in radians (velDir) and the magnitudes are in meters per second (velMag)

    float velDir[9] = {0.00000, 1/4 * M_PI, 1/2 * M_PI, 3/4 * M_PI, M_PI, -3/4 * M_PI, -1/2 * M_PI, -1/4 * M_PI, 0.00000};
    float velMag[9]= {1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5};
   
    int numActions;
    if(followNeighbors){
		numActions = 9;
	}else{
		numActions = 8;		
	}
   
    float Result[400];
    float resultEnergy[400];
    float sumTotalEnergies = 0;
	int timeLimit = 10000;
    /* -------------Setting----------------- */
    





	
	/* -------------Simulation----------------- */
		
	/* 終了キーをfalseにセット */
	bool finalize = false;
	/* シミュレーションを作成 */
	RVO::RVOSimulator* sim = new RVO::RVOSimulator();
	/* actionをセット */
	for (int i = 0; i < numActions; i++){
		actionVector[i] = velDir[i];
		actionVectorMag[i] = velMag[i];
	}
	/* 出力ファイルをセット */
	std::ofstream outputFile;
	std::string filePath = "/Users/tamurashuntarou/Downloads/CML/results/CNav/circle/result.csv";
	outputFile.open(filePath);
	
	std::ofstream positionFile;
	std::string positionFilePath = "/Users/tamurashuntarou/Downloads/CML/results/CNav/circle/position.csv";
	positionFile.open(positionFilePath);
	
	/* シナリオをセット */
	std::vector<bool> isInGoal(numAgents, false);
	setupScenario(sim, outputFile, numAgents, algorithm, coordFactor, isInGoal, numActions);
	/* シミュレーション */
	int timeStep = 0;
	int collisions = 0;
	
	recordPosition(sim,positionFile,100);
	do{
		std::cout << "timeStep:" << timeStep << std::endl;
		/* 位置出力 */
		recordPosition(sim,positionFile,10);
		/* ゴール到達判定 */
		finalize = judgeIfFinalize(sim, isInGoal, timeStep, timeLimit); /* シミュレーションがまだ実行されている場合、終了条件を満たしているかをboolで返す */
		//std::cout << "judgeIfFinalize:completed" << std::endl;
		/* C-Navの場合速度シミュレーション (目的：chosenActionを更新) */
		if((algorithm == 2) && (timeStep > 1)){
			evaluateEachAction(sim, outputFile, isInGoal, coordFactor, allNeigh, contadourX, numActions, followNeighbors); 
		}
		//std::cout << "evaluateEachAction:completed" << std::endl;
		for (size_t i = 0; i < sim->getNumAgents(); ++i){
			std::cout << "chosenAction of " << i << " : " << chosenAction[i] << std::endl;	
		}
		/* 希望速度セット (反映：chosenActionを使ってgoalVectorを回転) */
		setPreferredVelocities(sim, algorithm, isInGoal);
		//std::cout << "setPreferredVelocities:completed" << std::endl;
		/* doStep() */
		sim->doStep();

		for (size_t i = 0; i < sim->getNumAgents(); ++i){
			std::cout << "velocity of " << i << " : " << sim->getAgentVelocity(i) << std::endl;	
		}
		for (size_t i = 0; i < sim->getNumAgents(); ++i){
			std::cout << "position of " << i << " : " << sim->getAgentPosition(i) << std::endl;	
		}
		for (size_t i = 0; i < sim->getNumAgents(); ++i){
			std::cout << "goal of " << i << " : " << finalGoals[i] << std::endl;	
		}
		for (size_t i = 0; i < sim->getNumAgents(); ++i){
			std::cout << "goalVector of " << i << " : " << goalVectors[i] << std::endl;	
		}
		/* countCollisions() */
		collisions = countCollisions(sim, collisions, isInGoal);
		/* 時間を進める */
		timeStep++;

		std::cout << std::endl;
	}while(!finalize);
	
	delete sim; /* シミュレーションを消す */
	outputFile.close();
	positionFile.close();
    /* -------------Simulation----------------- */

    return 0;
}
