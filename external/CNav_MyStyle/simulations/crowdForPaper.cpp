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
#include "../src/RVOSimulator.h"
#include "../src/Agent.h"
#include "../src/KdTree.h"
#include "../src/Obstacle.h"
#include <stdlib.h>

#include "../src/RVO.h"
#include "legacy_trace_export.h"

#include <string>
#include <sstream>

#ifndef M_PI
static const float M_PI = 3.14159265358979323846f;
#endif

//**************Default Simulation Parameters*********************//
float simTimeStep = 1.0f; // タイムステップの長さ
float ooparts = 0.4f; //謎のパラメータ。使わないけど除去が面倒だから残っている。

//General variable declaration
int totalConsideredNeighbors[320];
int ascendingSimilarAgentIDList[320][50];

std::vector<RVO::Vector2> finalGoals;

float actionVector[50];
float actionVectorMag[50];
float totalReward[320][50];
float ActionSpeed[320][50];
float ActionDirection[50] = {0.0f,1.0f / 4.0f * M_PI, 2.0f / 4.0f * M_PI,3.0f / 4.0f * M_PI,-3.0f / 4.0f * M_PI,-2.0f / 4.0f * M_PI,-1.0f / 4.0f * M_PI,M_PI,
							0.0f,1.0f / 4.0f * M_PI, 2.0f / 4.0f * M_PI,3.0f / 4.0f * M_PI,-3.0f / 4.0f * M_PI,-2.0f / 4.0f * M_PI,-1.0f / 4.0f * M_PI,M_PI,
							0.0f};
//**************Default Simulation Parameters*********************//

float averageGoalTime;

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
	
	//std::cout << "tempPosition : " << tempPosition << std::endl;
	if(((tempPosition - 1) > 0.000001 ) || ((tempPosition-1) < -0.000001)){	
        //std::cout << "tempPosition : " << tempPosition << std::endl;
		rad = acos(tempPosition);
	}
	
	if((l > 0) && (rad < M_PI)){ // it is on the other side:
		rad = -rad;
	}
	return rad;	
}

void computeSimilarAgentsDir(RVO::RVOSimulator* sim, int myID, std::ofstream &outputFile, std::vector<bool> isInGoal, std::vector<RVO::Vector2> temporaryGoals) /* 近傍の類似エージェントを探して記録する */
{
	std::vector< std::pair<float, int> > innerList;
    RVO::Vector2 myPosition = sim->getAgentPosition(myID);
    RVO::Vector2 myGoal = sim->getAgentGoal(myID);
    RVO::Vector2 myGoalPrefVelocity = sim->getAgentGoalPrefVelocity(myID);
	RVO::Vector2 myNormalizeGoalVector = RVO::normalize(temporaryGoals[myID]);
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
		float prefVelocitySimilarity = myNormalizeGoalVector * yourPrefVelocity;
			
		//std::cout << "isInGoal : " << isInGoal[you] << std::endl;
		//std::cout << "nearThanMe : " << nearThanMe << std::endl;
		//std::cout << "prefVelocitySimilarity : " << prefVelocitySimilarity << std::endl;
		if((!isInGoal[you]) && (nearThanMe) && (prefVelocitySimilarity > 0)){ //ここでは、ゴールに近いエージェントのみを類似性の対象とする
            //std::cout << "find follow" << std::endl;
			float thisInner = myNormalizeGoalVector * yourVelocity;
			if(thisInner > 0 ){ //自分のゴールと同じ方向に向かっていたら実行
				innerList.push_back(std::make_pair(thisInner / 1.5f , you ));
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
			ascendingSimilarAgentIDList[myID][i] = thiSimilarAgentInnerID;       
		}else{
			ascendingSimilarAgentIDList[myID][i] = -1;
		}
    }
}

void evaluateEachAction(RVO::RVOSimulator* sim, std::ofstream &outputFile, std::vector<bool> isInGoal,
						float coordFactor, bool allNeigh, int contadourX, int numActions, float ActionDirection[9],
						bool followNeighbors, std::vector<RVO::Vector2> temporaryGoals,
						int lengthSimulateTimeSteps, int maxNumSimulateNeighbors, int maxNumThinkNeighbors,
						float sameWeight, std::vector<int> chosenAction)  /* timeHorizonで定義された将来のタイムステップ数にわたり、各action/velocityについてシミュレーションする */
{	
	sim->buildTree();
	//std::cout << "buildTree:completed" << std::endl;
    sim->setVvalues(); /* for (int i = 0; i < static_cast<int>(agents_.size()); ++i){
							agents_[i]->Vposition_ = agents_[i]->position_;
							agents_[i]->Vvelocity_ = agents_[i]->velocity_;		
						} */
	//std::cout << " setVvalues:completed" << std::endl;
    int maxEvaluateActions = numActions;
    
	/* 各エージェントに対して実行 */
	for (int myID = 0; myID < static_cast<int>(sim->getNumAgents()); ++myID){
		//std::cout << " for ID : " << myID << std::endl;
		if (!isInGoal[myID]){					
			/* 追従以外の行動について、シミュレーション+報酬計算 */
			for (int thisAction = 0; thisAction < maxEvaluateActions; thisAction++){   
				//std::cout << "action:" << thisAction << std::endl;
				/* 予測時の考慮エージェント数調整 */
				totalReward[myID][thisAction] = sim->SimulateVelocity(myID, finalGoals[myID], lengthSimulateTimeSteps, thisAction, sim->getNumAgents(),
																		sim->getNumAgents(), coordFactor, ooparts, ActionSpeed[myID][thisAction], ActionDirection[thisAction],
																		allNeigh, contadourX, maxNumSimulateNeighbors, maxNumThinkNeighbors,sameWeight,chosenAction);
			}
			//std::cout << " mainSimulation : completed " << std::endl;						 
		}
    }
}

std::vector<int> choiceAction(RVO::RVOSimulator* sim, std::vector<bool> isInGoal, int numActions)
{
	std::vector<int> currentChosenAction(sim->getNumAgents(), 0);
	for (int myID = 0; myID < static_cast<int>(sim->getNumAgents()); ++myID){
		//std::cout << " for ID : " << myID << std::endl;
		if (!isInGoal[myID]){
			/* 報酬最大のactionを選択 */
			float finalReward = -10000.0f;
			float finalAction = 0.0f;
			float totalCNavPoliteness = 0.0f;
			for (int thisAction = 0; thisAction < numActions; thisAction++){		
				totalCNavPoliteness += sim->getAgentCnavPoliteness(myID,thisAction);        
				if(totalReward[myID][thisAction] >= finalReward){
					finalReward = totalReward[myID][thisAction];
					finalAction = thisAction;				
				}
			}
			currentChosenAction[myID]  = finalAction;            
		}
	}
	return currentChosenAction;
}

void setActions(RVO::RVOSimulator* sim, int myID, std::ofstream &outputFile, float coordFactor, int numActions)  /* エージェントの行動をセットする */
{
    RVO::Vector2 myNormalizeGoalVector = RVO::normalize(sim->getAgentGoal(myID)-sim->getAgentPosition(myID));
    
    for(int action = 0; action < numActions; action++){
        float myThisActionSpeed = ActionSpeed[myID][action];
        ActionSpeed[myID][action] = actionVectorMag[action];	
    }
}

void setPreferredVelocities(RVO::RVOSimulator* sim, int algorithm, std::vector<bool> isInGoal, std::vector<RVO::Vector2> temporaryGoals, std::vector<int> chosenAction) /* 選択されたアクションに基づいてエージェントの優先速度をセットする */
{	
    for (size_t i = 0; i < sim->getNumAgents(); ++i) {
		if(isInGoal[i] == false){ /* ゴール未到達エージェントに対して実行 */
            RVO::Vector2 myTemporaryGoalVector = sim->getAgentMaxSpeed(i) * RVO::normalize(temporaryGoals[i] - sim->getAgentPosition(i));
		
			if(algorithm == 1){ /* ORCAの場合 */	
				float angle = std::rand() * 2.0f * M_PI / RAND_MAX;
				float dist = std::rand() * 0.01f / RAND_MAX;
				sim->setAgentPrefVelocity(i, myTemporaryGoalVector + dist * RVO::Vector2(std::cos(angle), std::sin(angle)));
			}
			
			if(algorithm == 2){ /* C-Navの場合 */
				
				float angle =  ActionDirection[chosenAction[i]];
				float randomAngle = std::rand() * 2.0f * M_PI / RAND_MAX;
				float randomDist = std::rand() * 0.01f / RAND_MAX;
				
				//std::cout << "chosenAction : " << chosenAction[i] << std::endl;
				//std::cout << "angle : " << angle << std::endl;
				sim->setAgentPrefVelocity(i, 
                                        RVO::Vector2(myTemporaryGoalVector.x() * std::cos(angle) + myTemporaryGoalVector.y()*std::sin(angle),
                                                    myTemporaryGoalVector.y() * std::cos(angle) + myTemporaryGoalVector.x()*-std::sin(angle))
										+ randomDist * RVO::Vector2(std::cos(randomAngle), std::sin(randomAngle)));
			}		
		}
        else{ /* ゴールに到達していたら場外に飛ばして停止 */
			sim->setAgentPosition(i,RVO::Vector2(-1000.0f,-1000.0f));
			sim->setAgentVelocity(i, RVO::Vector2(0.0f, 0.0f));		
		}	
	}
}

RVO::Vector2 computeTraceIntendedVelocity(
	RVO::RVOSimulator *sim,
	size_t agentIndex,
	const std::vector<bool> &isInGoal,
	const std::vector<RVO::Vector2> &temporaryGoals,
	int actionIndex)
{
	if (isInGoal[agentIndex]) {
		return RVO::Vector2(0.0f, 0.0f);
	}

	RVO::Vector2 myTemporaryGoalVector =
		sim->getAgentMaxSpeed(agentIndex) *
		RVO::normalize(temporaryGoals[agentIndex] - sim->getAgentPosition(agentIndex));
	const float angle = ActionDirection[actionIndex];

	return RVO::Vector2(
		myTemporaryGoalVector.x() * std::cos(angle) + myTemporaryGoalVector.y() * std::sin(angle),
		myTemporaryGoalVector.y() * std::cos(angle) + myTemporaryGoalVector.x() * -std::sin(angle));
}

std::vector<int> collectRankedNeighbors(int agentIndex)
{
	std::vector<int> rankedNeighbors;
	for (int neighborIndex = 0; neighborIndex < totalConsideredNeighbors[agentIndex]; ++neighborIndex) {
		rankedNeighbors.push_back(ascendingSimilarAgentIDList[agentIndex][neighborIndex]);
	}
	return rankedNeighbors;
}

std::vector<LegacyTraceCandidateAction> collectCandidateActions(
	RVO::RVOSimulator *sim,
	size_t agentIndex,
	const std::vector<bool> &isInGoal,
	const std::vector<RVO::Vector2> &temporaryGoals,
	int numActions)
{
	std::vector<LegacyTraceCandidateAction> candidateActions;
	if (isInGoal[agentIndex]) {
		return candidateActions;
	}

	for (int actionIndex = 0; actionIndex < numActions; ++actionIndex) {
		LegacyTraceCandidateAction candidate = {
			actionIndex,
			computeTraceIntendedVelocity(sim, agentIndex, isInGoal, temporaryGoals, actionIndex),
			totalReward[agentIndex][actionIndex],
		};
		candidateActions.push_back(candidate);
	}
	return candidateActions;
}

void setupScenario(RVO::RVOSimulator* sim, std::ofstream &outputFile, int algorithm,
					float coordFactor, int numActions, int numNeighbors) //Initialize the Simulation, positions of the obstacles and the agents
{
	sim->setTimeStep(simTimeStep);

	std::vector<RVO::Vector2> obstacle1,obstacle2, obstacle3,obstacle4;
	
	const float neighborDistance = 100.0f; //max distance that agents can perceive neighbors
	const float timeHorizonORCA = 10.0f;  // time horizon to determine collisions with other agents
	const float timeHorizonObstORCA = 10.0f;// time horizon to determine collisions with obstacles
	const float radiusORCA = 5.0f;  // distance that the agents want to keep from other agents
	const float maxSpeedORCA = 1.5f; //maximum speed that agents can move with

	float widthOfWall = 250.0f;
	float lengthOfWall = 1000.0f;
	
	/* |       | */
	/* |--- ---| */
	/* |       | */
	/* ↑ ↑   ↑ ↑ */
	/* 1 3   4 2 */

	obstacle1.push_back(RVO::Vector2(-widthOfWall, lengthOfWall/2));
	obstacle1.push_back(RVO::Vector2(-widthOfWall, -lengthOfWall/2));
	obstacle1.push_back(RVO::Vector2(-widthOfWall/2, -lengthOfWall/2));
	obstacle1.push_back(RVO::Vector2(-widthOfWall/2, lengthOfWall/2));
	
	obstacle2.push_back(RVO::Vector2(widthOfWall/2, lengthOfWall/2));
	obstacle2.push_back(RVO::Vector2(widthOfWall/2, -lengthOfWall/2));
	obstacle2.push_back(RVO::Vector2(widthOfWall, -lengthOfWall/2));
	obstacle2.push_back(RVO::Vector2(widthOfWall, lengthOfWall/2));
	
	sim->addObstacle(obstacle1);
	sim->addObstacle(obstacle2);
	
	sim->processObstacles();
	
	/* エージェント追加 */
    sim->setAgentDefaults(neighborDistance, numNeighbors, timeHorizonORCA , timeHorizonObstORCA , radiusORCA , maxSpeedORCA);

	/* 左側 */
	for (int i = 0; i < 5; i++){
		for (int j = 0; j < 5; j++){
			sim->addAgent(RVO::Vector2(-100 + i * 50.0f, -200 - j * 50.0f));
			finalGoals.push_back(RVO::Vector2(-100 + i * 50.0f, 400));
			sim->setAgentGoal(5*i+j, RVO::Vector2(-100 + i * 50.0f, 400));
		}
	}

	/* 右側 */
	for (int i = 0; i < 5; i++){
		for (int j = 0; j < 5; j++){
			sim->addAgent(RVO::Vector2(-100 + i * 50.0f, 200 + j * 50.0f));
			finalGoals.push_back(RVO::Vector2(-100 + i * 50.0f, -400));
			sim->setAgentGoal(5*i+j+25, RVO::Vector2(-100 + i * 50.0f, -400));
		}
	}
				
	for (int i = 0; i < sim->getNumAgents(); ++i){			
		setActions(sim, i, outputFile, coordFactor, numActions);
		sim->setAgentInGoal(i,false);

		for (int j = 0; j < numActions; j++){
			totalReward[i][j] = 0.0f;
		}		
		
		if(algorithm == 1){ /* ORCAの場合 */
			ActionSpeed[i][0] = 1.5f;
		}
	}	
}

bool judgeIfFinalize(RVO::RVOSimulator* sim, std::vector<bool> isInGoal, int timeStep, int timeLimit) /* シミュレーションがまだ実行されている場合、終了条件を満たしているかをboolで返す */
{
	/* 時間切れならその時点で切る */
	if (timeStep > timeLimit){
		return true;
	}
	/* 時間内なら実行, ゴール未到達エージェントが一つでもいる限りfalseを返す */
	for (int i = 0; i < sim->getNumAgents(); ++i){
		if (!isInGoal[i]){
			return false;
		}
	}
	/* 上に引っ掛からなかったらtrue (時間内だがゴール全員到達) */
	return true;
}

std::vector<bool> judgeInGoal(RVO::RVOSimulator* sim, std::vector<bool> currentIsInGoal) /* timeStepを見てneedToUpdateを返す */
{
	std::vector<bool> nextIsInGoal = currentIsInGoal;
	for (size_t i = 0; i < sim->getNumAgents(); ++i){
		if (!nextIsInGoal[i]){
			float goalDist = RVO::abs(sim->getAgentGoal(i)-sim->getAgentPosition(i));
			bool thisIsInGoal = goalDist < 10.0f;		
			nextIsInGoal[i] = thisIsInGoal;
			sim->setAgentInGoal(i,thisIsInGoal);
			if (thisIsInGoal){averageGoalTime+=sim->getGlobalTime();}
		}
	}
	return nextIsInGoal;
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

std::vector<RVO::Vector2> calculateTemporaryGoals(RVO::RVOSimulator* sim){
	std::vector<RVO::Vector2> temporaryGoals;
	RVO::Vector2 O = RVO::Vector2(0.0f, 0.0f);
	for (size_t i = 0; i < sim->getNumAgents(); ++i) {
		RVO::Vector2 thisTemporaryGoal = sim->getAgentGoal(i);
		sim->setAgentPrefVelocity(i,sim->getAgentMaxSpeed(i) * RVO::normalize(sim->getAgentGoal(i)-sim->getAgentPosition(i)));
		/*
		bool goalIsPositiveSide = sim->getAgentGoal(i).y() > 0.0f;
		bool positionIsPositiveSide = sim->getAgentPosition(i).y() > 0.0f;
		bool nearO = RVO::abs(sim->getAgentPosition(i) - O) < 10.0f;
		if (goalIsPositiveSide){
			if ((!positionIsPositiveSide) && (!nearO)){
				thisTemporaryGoal = RVO::Vector2(0.0f, -5.0f);
			}
		}else{
			if ((positionIsPositiveSide) && (!nearO)){
				thisTemporaryGoal = RVO::Vector2(0.0f, 5.0f);
			}
		}
		*/
		temporaryGoals.push_back(thisTemporaryGoal);
	}
	return temporaryGoals;
}


void mainLoop(int iterNum, float coordFactor, int numNeighbors, int lengthSimulateTimeSteps,
			int maxNumSimulateNeighbors, int maxNumThinkNeighbors,
			float sameWeight)
{
    /* -------------Setting----------------- */
    
	int algorithm = 2; /* 1.ORCA, 2.C-Nav */
	bool allNeigh = false; /* 全てのエージェント(1) or ゴールに近い側だけ(0) */
	int contadourX = 1;
	bool followNeighbors = false; /* 追従行動の有無(false:禁止, true:可能) */
	LegacyTraceConfig traceConfig = loadLegacyTraceConfig();
	if (traceConfig.enabled) {
		srand(traceConfig.seed);
	} else {
		srand(time(NULL));
	}
    ////////////////////////////////////////////////////////////////////
    //////Sample actions: 8 velocities whose direction is in radians (velDir) and the magnitudes are in meters per second (velMag)


	float velMag[17]= {1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5,
						0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
						0.0};
   
    int numActions;
	numActions = 17;		
	int timeLimit = 10000;
    /* -------------Setting----------------- */

	/* -------------Simulation----------------- */
	//std::cout << "coord,iter,time" << std::endl;
	std::ofstream traceFile;
	if (traceConfig.enabled) {
		traceFile.open(traceConfig.output_path.c_str());
	}
	for (int thisIter = 0; thisIter < iterNum; thisIter++){
		/* 終了キーをfalseにセット */
		bool finalize = false;
		averageGoalTime = 0.0f;
		/* シミュレーションを作成 */
		RVO::RVOSimulator* sim = new RVO::RVOSimulator();
		/* actionをセット */
		for (int i = 0; i < numActions; i++){
			actionVectorMag[i] = velMag[i];
		}
		/* 出力ファイルをセット */
		std::ofstream actionFile;
		if (!traceConfig.enabled) {
			std::string actionFilePath = "/Users/tamurashuntarou/Downloads/CML/results/CNav_MyStyle/crowd/action.csv";
			actionFile.open(actionFilePath);
		}

		std::ofstream prefVFile;
		if (!traceConfig.enabled) {
			std::string prefVFilePath = "/Users/tamurashuntarou/Downloads/CML/results/CNav_MyStyle/crowd/prefV.csv";
			prefVFile.open(prefVFilePath);
		}
		
		std::ofstream positionFile;
		if (!traceConfig.enabled) {
			std::string positionFilePath = "/Users/tamurashuntarou/Downloads/CML/results/CNav_MyStyle/crowd/position.csv";
			positionFile.open(positionFilePath);
		}
		
		/* シナリオをセット */
		setupScenario(sim, actionFile, algorithm, coordFactor, numActions, numNeighbors);
		std::vector<bool> isInGoal(sim->getNumAgents(), false);
		std::vector<int> chosenAction(sim->getNumAgents(), 0);
		setPreferredVelocities(sim, algorithm, isInGoal, finalGoals, chosenAction);
		/* シミュレーション */
		int timeStep = 0;
		int collisions = 0;

		//recordPosition(sim,positionFile,1);
		
		/*
		actionFile << "timeStep";
		for (size_t i = 0; i < sim->getNumAgents(); ++i){
			actionFile << ",Action of " << i ;
		}
		actionFile << std::endl;

		prefVFile << "timeStep";
		for (size_t i = 0; i < sim->getNumAgents(); ++i){
			prefVFile << ",Action of " << i ;
		}
		prefVFile << std::endl;
		*/
		
		do{
			const float traceGlobalTime = sim->getGlobalTime();
			//std::cout << " timeStep : " << timeStep << std::endl;
			/* 位置出力 */
			recordPosition(sim,positionFile,1);
			/* isInGoalの更新 */
			isInGoal = judgeInGoal(sim, isInGoal);
			//std::cout << " judgeInGoal:OK" << std::endl;
			/* 終了判定 */
			finalize = judgeIfFinalize(sim, isInGoal, timeStep, timeLimit); /* シミュレーションがまだ実行されている場合、終了条件を満たしているかをboolで返す */
			//std::cout << " judgeIfFinalize:OK" << std::endl;
			/* 一時的な目標を作る */
			std::vector<RVO::Vector2> temporaryGoals = calculateTemporaryGoals(sim);
			//std::cout << " calculateTemporaryGoals:OK" << std::endl;
			/* C-Navの場合速度シミュレーション (目的：chosenActionを更新) */
			std::vector<int> appliedChosenAction = chosenAction;
			std::vector<int> nextChosenAction = chosenAction;
			std::vector<RVO::Vector2> communicatedIntendedVelocity(sim->getNumAgents(), RVO::Vector2(0.0f, 0.0f));
			std::vector<std::vector<int> > rankedNeighbors(sim->getNumAgents());
			std::vector<std::vector<LegacyTraceCandidateAction> > candidateActions(sim->getNumAgents());
			const bool actionUpdated = (algorithm == 2) && (timeStep > 1);
			if((algorithm == 2) && (timeStep > 1)){
				evaluateEachAction(sim, actionFile, isInGoal, coordFactor, allNeigh, contadourX,
									numActions, ActionDirection, followNeighbors, temporaryGoals,
									lengthSimulateTimeSteps,maxNumSimulateNeighbors,maxNumThinkNeighbors,
									sameWeight, chosenAction); 
				nextChosenAction = choiceAction(sim, isInGoal,numActions);
				for (size_t agentIndex = 0; agentIndex < sim->getNumAgents(); ++agentIndex) {
					rankedNeighbors[agentIndex] = collectRankedNeighbors(static_cast<int>(agentIndex));
					candidateActions[agentIndex] = collectCandidateActions(
						sim,
						agentIndex,
						isInGoal,
						temporaryGoals,
						numActions);
				}
				chosenAction = nextChosenAction;
			}

			/* 希望速度セット (反映：chosenActionを使ってgoalVectorを回転) */
			setPreferredVelocities(sim, algorithm, isInGoal, temporaryGoals, chosenAction);
			for (size_t agentIndex = 0; agentIndex < sim->getNumAgents(); ++agentIndex) {
				communicatedIntendedVelocity[agentIndex] = sim->getAgentPrefVelocity(agentIndex);
			}
			/* doStep */
			sim->doStep();
			if (traceConfig.enabled) {
				for (size_t agentIndex = 0; agentIndex < sim->getNumAgents(); ++agentIndex) {
					writeLegacyTraceRecord(
						traceFile,
						timeStep,
						traceGlobalTime,
						static_cast<int>(agentIndex),
						"agent_" + std::to_string(agentIndex),
						actionUpdated,
						rankedNeighbors[agentIndex],
						communicatedIntendedVelocity[agentIndex],
						chosenAction[agentIndex],
						communicatedIntendedVelocity[agentIndex],
						sim->getAgentVelocity(agentIndex),
						candidateActions[agentIndex],
						actionUpdated,
						nextChosenAction[agentIndex],
						computeTraceIntendedVelocity(
							sim,
							agentIndex,
							isInGoal,
							temporaryGoals,
							nextChosenAction[agentIndex]));
				}
			}

			/*
			actionFile << timeStep;
			for (size_t i = 0; i < sim->getNumAgents(); ++i){
				actionFile << "," << chosenAction[i];	
			}
			actionFile << std::endl;

			prefVFile << timeStep;
			for (size_t i = 0; i < sim->getNumAgents(); ++i){
				prefVFile << "," << sim->getAgentPrefVelocity(i);	
			}
			prefVFile << std::endl;
			if (timeStep % 100 == 0){
				std::cout << "timeStep:" << timeStep << std::endl;
				for (size_t i = 0; i < sim->getNumAgents(); ++i){
					std::cout << "Agent : " << i << std::endl; 
					for (size_t j = 0; j < numActions; ++j){
						std::cout << " reward of action " << j << " : " << totalReward[i][j] << std::endl;
					}
					std::cout << " chosenAction of " << i << " : " << chosenAction[i] << std::endl;
					std::cout << " prefVelocity of " << i << " : " << sim->getAgentPrefVelocity(i) << std::endl;
					std::cout << " temporaryGoal : " << temporaryGoals[i] << std::endl;
					std::cout << " prefVelocity : " << sim->getAgentPrefVelocity(i) << std::endl;
					std::cout << " velocity : " << sim->getAgentVelocity(i) << std::endl;	
					std::cout << " position : " << sim->getAgentPosition(i) << std::endl;	
					//std::cout << "goal : " << goals[i] << std::endl;	
					std::cout << " isInGoal : " << isInGoal[i] << std::endl;
				}
				std::cout << std::endl;
			}
			*/
			/* countCollisions() */
			//collisions = countCollisions(sim, collisions, isInGoal);
			/* 時間を進める */
			timeStep++;
			if (traceConfig.enabled && traceConfig.max_steps > 0 && timeStep >= traceConfig.max_steps) {
				finalize = true;
			}
	

		}while(!finalize);
		
		std::cout << coordFactor << "," << maxNumSimulateNeighbors << "," << maxNumThinkNeighbors << "," << lengthSimulateTimeSteps << "," << thisIter << "," << sim->getGlobalTime() << "," << averageGoalTime/sim->getNumAgents() << std::endl;
		//if(sim->getGlobalTime()>300){break;}

		delete sim; /* シミュレーションを消す */
		actionFile.close();
		positionFile.close();
		prefVFile.close();
		/* -------------Simulation----------------- */
	}
}

int main()
{
	std::cout << "coord,simNeighbor,thinkNeighbor,step,iter,time,avgTime" << std::endl;
	if (loadLegacyTraceConfig().enabled) {
		mainLoop(1,0.9f,10,3,5,1,10);
		return 0;
	}
	
	/*
	mainLoop(10,0.0f,10,3);
	mainLoop(10,0.1f,10,3);
	mainLoop(10,0.2f,10,3);
	mainLoop(10,0.3f,10,3);
	mainLoop(10,0.4f,10,3);
	mainLoop(10,0.5f,10,3);
	mainLoop(10,0.6f,10,3);
	mainLoop(10,0.7f,10,3);
	mainLoop(10,0.8f,10,3);
	mainLoop(10,0.9f,10,3);
	mainLoop(10,0.99f,10,3);
	*/

	mainLoop(1,0.9f,10,3,5,1,10);
	//mainLoop(10,0.9f,10,3,10,10,1);
	//mainLoop(10,0.9f,10,3,10,10,0);
	//mainLoop(10,0.1f,9,3,9,3);
	//mainLoop(10,0.1f,9,3,9,5);
	//mainLoop(10,0.1f,9,3,9,9);
	
	return 0;
}
