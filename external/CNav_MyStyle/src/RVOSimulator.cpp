/*
 * RVOSimulator.cpp
 * RVO2 Library
 *
 * Copyright (c) 2008-2010 University of North Carolina at Chapel Hill.
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for educational, research, and non-profit purposes, without
 * fee, and without a written agreement is hereby granted, provided that the
 * above copyright notice, this paragraph, and the following four paragraphs
 * appear in all copies.
 *
 * Permission to incorporate this software into commercial products may be
 * obtained by contacting the authors <geom@cs.unc.edu> or the Office of
 * Technology Development at the University of North Carolina at Chapel Hill
 * <otd@unc.edu>.
 *
 * This software program and documentation are copyrighted by the University of
 * North Carolina at Chapel Hill. The software program and documentation are
 * supplied "as is," without any accompanying services from the University of
 * North Carolina at Chapel Hill or the authors. The University of North
 * Carolina at Chapel Hill and the authors do not warrant that the operation of
 * the program will be uninterrupted or error-free. The end-user understands
 * that the program was developed for research purposes and is advised not to
 * rely exclusively on the program for any reason.
 *
 * IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR THE
 * AUTHORS BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS
 * SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF NORTH CAROLINA AT
 * CHAPEL HILL OR THE AUTHORS HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 * THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS SPECIFICALLY
 * DISCLAIM ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND ANY
 * STATUTORY WARRANTY OF NON-INFRINGEMENT. THE SOFTWARE PROVIDED HEREUNDER IS ON
 * AN "AS IS" BASIS, AND THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE
 * AUTHORS HAVE NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT, UPDATES,
 * ENHANCEMENTS, OR MODIFICATIONS.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
 */

#include "RVOSimulator.h"

#include "Agent.h"
#include "KdTree.h"
#include "Obstacle.h"
#include <stdlib.h>
#include <cstdlib>
#include <utility>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <map>
#include <set>
#include <algorithm>
#include <functional>
#include <stdio.h>


#ifdef _OPENMP
#include <omp.h>
#endif

namespace RVO {
	RVOSimulator::RVOSimulator() : defaultAgent_(NULL), globalTime_(0.0f), kdTree_(NULL), timeStep_(0.0f)
	{
		kdTree_ = new KdTree(this);
	}
	
	RVOSimulator::RVOSimulator(float timeStep, float neighborDist, size_t maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, const Vector2 &velocity) : defaultAgent_(NULL), globalTime_(0.0f), kdTree_(NULL), timeStep_(timeStep)
	{
		kdTree_ = new KdTree(this);
		defaultAgent_ = new Agent(this);
		
		defaultAgent_->maxNeighbors_ = maxNeighbors;
		defaultAgent_->maxSpeed_ = maxSpeed;
		defaultAgent_->neighborDist_ = neighborDist;
		defaultAgent_->radius_ = radius;
		defaultAgent_->timeHorizon_ = timeHorizon;
		defaultAgent_->timeHorizonObst_ = timeHorizonObst;
		defaultAgent_->velocity_ = velocity;
		
		defaultAgent_->hypoVelocity_ = velocity;
		defaultAgent_->Vvelocity_ = velocity;
	}
	
	RVOSimulator::~RVOSimulator()
	{
		if (defaultAgent_ != NULL) {
			delete defaultAgent_;
		}
		
		for (size_t i = 0; i < agents_.size(); ++i) {
			delete agents_[i];
		}
		
		for (size_t i = 0; i < obstacles_.size(); ++i) {
			delete obstacles_[i];
		}
		
		delete kdTree_;
	}
		
	bool RVOSimulator::sortinrev(const std::pair<int,int> &a,  
               const std::pair<int,int> &b) { 
       return (a.first > b.first); 
	}

	float RVOSimulator::SimulateVelocity(size_t agentID, RVO::Vector2 goal, int lengthSimulationTimeSteps, int actionNum, const int numNeighbors, int numAgents,
										float coordFactor, float coord_factor_alan, float ActionSpeed, float ActionDirection, int allNeigh, int contadourX,
										int maxNumSimulateNeighbors, int maxNumThinkNeighbors, float sameWeight, std::vector<int> chosenAction)
	{
		float VprefVelocity_;
		float politeness;
		float goalProgress = 0.0f;
		float reward = 0.0f;
		int NeighborList[numNeighbors];
		int r = 0;
		int endOfPlan = lengthSimulationTimeSteps;
		Vector2 prefVelocityBackup_ = agents_[agentID]->prefVelocity_;
		bool goalFound = false;
		
		float avgPolitenessCNav = 0.0f;
		float avgPolitenessALAN = 0.0f;
		float avgGoalProg = 0.0f;
		float changeVel = 0.0f;
		
		std::vector<std::pair<float,float> > velocityChangeList;
		float politeness4 = 0.0f;
		
		//std::cout << "SimulateVelocity:start" << std::endl;	
		
		/* 初期設定 */
		for (int n = 0; n < numNeighbors; n++){
			//std::cout << "neighbor:" << n << std::endl;
			NeighborList[n] = -1;
			//std::cout << "setting neighborlist:completed" << std::endl;
			agents_[n]->vOpt_ = agents_[n]->Vvelocity_;
			//std::cout << "setting vopt:completed" << std::endl;
			agents_[n]->Vposition_ = agents_[n]->position_;
			//std::cout << "setting vposition:completed" << std::endl;
			agents_[n]->Vvelocity_ = agents_[n]->velocity_;
			//std::cout << "setting vvelocity:completed" << std::endl;
			agents_[n]->VprefVelocity_ = agents_[n]->prefVelocity_;
			//std::cout << "setting VprefVelocity_:completed" << std::endl;
		}
		//std::cout << "setting:completed" << std::endl;
			
		/* 近傍エージェントのツリーを作成 */
		agents_[agentID]->computeNeighbors();
		//std::cout << "treeBuild:completed" << std::endl;	
			
		/* 近傍エージェントに対して,,, */
		int numCalculateNeighbors = std::min(maxNumSimulateNeighbors,(int)agents_[agentID]->agentNeighbors_.size());

		for (size_t j = 0; j < numCalculateNeighbors; ++j){    
			int neighborID = agents_[agentID]->agentNeighbors_[j].second->id_;
			if(globalTime_<5){
				//std::cout << "I am " << agentID << ", and neighbor is " << neighborID << std::endl;
			}
			//std::cout<<neighborID<<std::endl;
			agents_[neighborID]->VcomputeNeighborsModel(agents_[agentID], agents_[agentID]->timeHorizonObst_, agents_[agentID]->maxSpeed_, agents_[agentID]->radius_ ,agents_[agentID]->neighborDist_, agents_[agentID]->timeHorizonObst_ , agents_[agentID]->maxSpeed_, agents_[agentID]->radius_, agents_[agentID]->maxNeighbors_ ,agents_[agentID]->neighborDist_);
		}
		//std::cout << "VcomputeNeighborsModel:completed" << std::endl;
		

		/* 近傍エージェントの存在フラグを作成 */
		bool existNeighbor = false;
		if(agents_[agentID]->agentNeighbors_.size() != 0){			
			existNeighbor = true;	
		}
			
		#pragma omp parallel for

		
		/* 考慮タイムステップ分だけ予測 */
		for(int t = 0; t < lengthSimulationTimeSteps; t++){
			int neighbors = 0;
			kdTree_->VbuildAgentTree();
			Vector2 goalVector = ActionSpeed * RVO::normalize(goal - agents_[agentID]->Vposition_);
			velocityChangeList.clear();
			
			/* 希望速度をセット（goalVectorをActionDirectionの方へ回転） */
			agents_[agentID]->prefVelocity_ 
				= Vector2(goalVector.x() * std::cos(ActionDirection) + goalVector.y() * std::sin(ActionDirection),
						goalVector.y() * std::cos(ActionDirection) + goalVector.x() * -std::sin(ActionDirection));
									
			/* 各近傍エージェントに対し、安全速度を計算する */	
			for (size_t j = 0; j < numCalculateNeighbors; ++j){ 
				/* 取り扱う近傍エージェントのIDを取得 */
				int thisNeighborID = agents_[agentID]->agentNeighbors_[j].second->id_;
				//std::cout<<thisNeighborID<<std::endl;
				/* この近傍エージェントが今回見る近傍リストの中に入っているかチェック */
				bool exists = false;
				for (int thisNeighbor = 0; thisNeighbor < numNeighbors; thisNeighbor++) {
					if (thisNeighborID == NeighborList[thisNeighbor]){
						exists = true;
						break;
					}
				}
				/* 入っていない場合追加 */
				if(exists == false){ 
					NeighborList[r] = thisNeighborID;					
					if(r < numNeighbors - 1){
						r++;
					}
				}
									
				if(t == 0){ /* 最初のタイムステップでは、近隣のエージェントは事前に観測されたエージェントの速度に反応する */
					agents_[thisNeighborID]->computeHypoVelocityNeighbor();
				}else{/* 近傍エージェントへの影響を計算 */			
					float myGoalDist = RVO::abs(goal- agents_[agentID]->position_);
					float yourGoalDist = RVO::abs(goal- agents_[thisNeighborID]->position_);
					float distToYou = RVO::abs(agents_[agentID]->position_ - agents_[thisNeighborID]->position_);
					bool counterDirection = (goal- agents_[agentID]->position_).x() * agents_[thisNeighborID]->prefVelocity_.x()
											+ (goal- agents_[agentID]->position_).y() * agents_[thisNeighborID]->prefVelocity_.y()
											 < 0.5f;
					//counterDirection = true;
					bool slowMove = absSq(agents_[thisNeighborID]->velocity_) / agents_[thisNeighborID]->maxSpeed_ < 0.1f;
					//slowMove = true;
					bool changeDirection = chosenAction[thisNeighborID] == 0;
					bool nearToMe = distToYou < agents_[agentID]->radius_ * 5.0f;
					/* 自分よりゴールに近い,かつ反対向きのエージェントに対し、速度変化を計算 */
					if((allNeigh)||((myGoalDist > yourGoalDist))){
						neighbors++;
						changeVel = agents_[agentID]->VcomputeVelChange(j,counterDirection,slowMove);	
						/* オリジナル : maxSpeed_ - RVO::abs(other->prefVelocity_-neighNewVelocity_); */
						if (!counterDirection){changeVel = changeVel * sameWeight;}
						//changeVel = changeVel * (agents_[agentID]->maxSpeed_-absSq(agents_[agentID]->velocity_));
						//std::cout << counterDirection << std::endl;
						
						//ガチオリジナル（報酬ソート）
						velocityChangeList.push_back(std::make_pair(changeVel/1.5f,distToYou));
						
						//改良（距離ソート）
						//velocityChangeList.push_back(std::make_pair(distToYou, changeVel/1.5f));
						
						agents_[thisNeighborID]->newVelocity_= agents_[agentID]->neighNewVelocity_;					
					}
				}					
			}

			
			/* t>0のとき、考慮するエージェント数の分politenessを加算  */
			int currentPoliteness = 0.0f;
			if(t > 0){		
				sort(velocityChangeList.begin(), velocityChangeList.end());	
				int contadour2 = 0;
				int numThinkNeighbors = std::min(maxNumThinkNeighbors,neighbors);
				for (int e = 0; e < numThinkNeighbors; e++){ 
					//std::cout << "agent " << agentID << " see " << velocityChangeList[e].second << ", and the v change is " << velocityChangeList[e].first << std::endl;
					//if (velocityChangeList[e].second == 0){	
					
					
					// ガチオリジナル（報酬ソート）
					currentPoliteness += velocityChangeList[e].first / (float)numThinkNeighbors;

					//改良（距離ソート）
					//currentPoliteness += velocityChangeList[e].second / (float)numThinkNeighbors;
					
					
					//std::cout << velocityChangeList[e].second << std::endl;
					//}
					contadour2++;
					if(contadour2 == contadourX){
						break;
					}	
				}
				neighbors = contadourX;
			}
			politeness += currentPoliteness;
				
			/* Compute the projected collision free velocity of the agent in the future */
			agents_[agentID]->computeHypoVelocityAgent();
			/* finalVel is the projected velocity */
			
			if(t == 0){
				setAgentInitialVelocity(agentID,agents_[agentID]->newVelocity_,actionNum);
				setAgentInitialPrefVelocity(agentID,agents_[agentID]->prefVelocity_,actionNum);
			}
							
			/* エージェントの速度と位置を更新 */
			agents_[agentID]->Vupdate();
			for (size_t j = 0; j < numCalculateNeighbors; ++j) {
				int thisNeighborID =  agents_[agentID]->agentNeighbors_[j].second->id_;
				agents_[thisNeighborID]->Vupdate();
			}

			
			/* ??? */
			float politeness2 = agents_[agentID]->prefVelocity_ * agents_[agentID]->newVelocity_ / 2.25f;		
			avgPolitenessALAN += politeness2;
				
			/////////////////////////////////////////////////////////////////
			///Reward function computation: Rg*coordFactor + (1-coordFactor)*Rp, where Rg: Goal progress, and Rp: Politeness
				
			Vector2 finalVel = agents_[agentID]->newVelocity_;
			goalProgress = normalize(goalVector) * finalVel / ((float)agents_[agentID]->maxSpeed_);
			
			//if(globalTime_<10 && agentID ==1){std::cout <<"goalProgress:" << goalProgress << std::endl;}

			//std::cout << "Agent : " << agentID << ", Action : " << actionNum << ", TimeStep : " << t << std::endl;
			//std::cout << " actionNum : " << actionNum << std::endl;
			//std::cout << " ActionDirection : " << ActionDirection << std::endl;
			//std::cout << " goalVector : " << goalVector << std::endl;
			//std::cout << " finalVel : " << finalVel << std::endl;
			//std::cout << std::endl;
			//std::cout << " goalProgress : " << goalProgress << std::endl;
			//std::cout << " currentPoliteness : " << currentPoliteness << std::endl;
			//std::cout << " goalVector : " << goalVector << std::endl;
			//std::cout << " finalVel : " << finalVel << std::endl;
			//std::cout << goalProgress << std::endl;
			
			avgGoalProg += goalProgress / (float)lengthSimulationTimeSteps;
				
			if(t == 0){
				politeness = 0.0f;
				goalProgress = 0.0f;
			}else{
				if(neighbors > 0){
					avgPolitenessCNav += politeness/(float)neighbors;
					reward += (goalProgress) * (1 - coordFactor) +  (politeness) * /*politeness2*/ (coordFactor);
					//std::cout << "reward : " << reward << std::endl;
				}else{
					politeness = 1.0f;
					avgPolitenessCNav += politeness/(float)neighbors;
					reward += (goalProgress) * (1 - coordFactor) +  (politeness) * /*politeness2*/ (coordFactor);
				}						
			}

			
			/* ゴールに到達したらendOfPlanを1足す */
			if((abs(agents_[agentID]->Vposition_ - goal) < 0.15f) && (!goalFound)){		
				goalFound = true;
				endOfPlan = t + 1;
			}
				
		}// End of time Horizon

		//Reset previous parameters of the neighbors
		for(int n = 0; n < numNeighbors; n++){
			if(NeighborList[n] > -1){  
				int p = agents_[NeighborList[n]]->id_;
				agents_[p]->Vposition_ = agents_[p]->position_;
				agents_[n]->VprefVelocity_ = agents_[n]->velocity_;
				agents_[p]->Vvelocity_ = agents_[n]->prefVelocity_;
			}
		}
		
		agents_[agentID]->prefVelocity_ = prefVelocityBackup_;
		setCnavPoliteness(agentID,avgPolitenessCNav,actionNum);
		setALANPoliteness(agentID,avgPolitenessALAN,actionNum);
		setGoalProg(agentID,avgGoalProg,actionNum);
		
		//std::cout << "reward : " << reward << std::endl;
		return reward / (endOfPlan - 1); //Returns the average reward for all timesteps
	}
			
	
	float RVOSimulator::SimulateVelNoComm(size_t agentID, RVO::Vector2 goal, int timeHorizon, int actionNum, int numNeighbors, int numAgents, float coordFactor, float ActionSpeed, float ActionDirection){
		float VprefVelocity_;
		float politeness = 0.0f;
		float goalProgress = 0.0f;
		float reward = 0.0f;
		int NeighborList[numNeighbors];
		int neighbors = 0;
		int r = 0;
		int endOfPlan = timeHorizon;
		int foundAt = 0;
		Vector2 goalVector,prefVelocityBackup_= agents_[agentID]->prefVelocity_;
		bool goalFound = false; // When the goal is reached, the agent just stays there
		float avgPolitenessCNav = 0;
		float avgPolitenessALAN = 0;
		float avgGoalProg = 0;
				
		for (int n = 0; n < numNeighbors; n++) {
			NeighborList[n] = -1;
			agents_[n]->vOpt_ = agents_[n]->Vvelocity_;
			agents_[n]->Vposition_ = agents_[n]->position_;
			agents_[n]->Vvelocity_ = agents_[n]->velocity_;
			agents_[n]->VprefVelocity_ = agents_[n]->prefVelocity_;
		}
		
		agents_[agentID]->computeNeighbors();	
		
		#pragma omp parallel for
		
		kdTree_->VbuildAgentTree();
			
		for(int t=0;t<timeHorizon;t++){// For each timestep in the time horizon...
				
			goalVector = RVO::normalize(goal- agents_[agentID]->Vposition_);
			
			if (t < timeHorizon){//...the agent computes a new preferred velocity
				agents_[agentID]->prefVelocity_= goalVector*ActionSpeed;
				agents_[agentID]->prefVelocity_=Vector2(agents_[agentID]->prefVelocity_.x()*std::cos(ActionDirection)+(agents_[agentID]->prefVelocity_.y())*std::sin(ActionDirection), (agents_[agentID]->prefVelocity_.y())*std::cos(ActionDirection)+(agents_[agentID]->prefVelocity_.x())*-std::sin(ActionDirection));
			}
			
			//For each neighbor of the agent, we need to compute its collision-free velocity		
			for (size_t j = 0; j < agents_[agentID]->agentNeighbors_.size(); ++j) { 
				int k =  agents_[agentID]->agentNeighbors_[j].second->id_;
				int exists = false;
				
				for (int u=0; u<numNeighbors; u++) {
					if(k == NeighborList[u]){
						exists = true;
						break;
					}
				}
				
				if(exists == false){ 
					NeighborList[r] = k;					
					if(r < (numNeighbors-1)){
						r++;
					}
				}		
				neighbors++;
				agents_[k]->newVelocity_=agents_[k]->velocity_;					
			}
					
			agents_[agentID]->computeHypoVelocityAgent_all(); // Compute the projected collision free velocity of the agent in the future
			
			Vector2 finalVel = agents_[agentID]->newVelocity_;  //finalVel is the projected velocity
			
			politeness = agents_[agentID]->prefVelocity_ * finalVel / 2.25f;			
			goalProgress = normalize(goalVector) * finalVel / 1.5f;
				
			agents_[agentID]->Vupdate(); //Agent agentID uses the finalVel

			for (size_t j = 0; j < agents_[agentID]->agentNeighbors_.size(); ++j) {
				int k =  agents_[agentID]->agentNeighbors_[j].second->id_;
				agents_[k]->Vupdate();
			}

			if(t == 0){
				setAgentInitialVelocity(agentID,agents_[agentID]->newVelocity_,actionNum);
				setAgentInitialPrefVelocity(agentID,agents_[agentID]->prefVelocity_,actionNum);
			}
			
			float politeness2 = agents_[agentID]->prefVelocity_*agents_[agentID]->newVelocity_ / 2.25f;
			
			if(neighbors == 0){
				avgPolitenessCNav += politeness;
			}else{
				avgPolitenessCNav += politeness/(float)neighbors;
			}
				
			avgPolitenessALAN += politeness2;
			avgGoalProg += normalize(goalVector) * finalVel / (float)agents_[agentID]->maxSpeed_;
				
			///Reward function computation: Rg*coordFactor + (1-coordFactor)*Rp, where Rg: Goal progress, and Rp: Politeness
			reward += (goalProgress)*coordFactor +  (politeness)*(1 - coordFactor);
							
							
			if((abs(agents_[agentID]->Vposition_-goal) < 0.15f)&&(!goalFound)){		
				goalFound = true;
				foundAt = t;
				endOfPlan = t + 1;
			}
				
		} // End of time Horizon
				
			
			
		//Reset previous parameters of the neighbors
		int p;
		for(int n = 0; n < numNeighbors; n++){
			if(NeighborList[n] > -1){  
				p=agents_[NeighborList[n]]->id_;
				agents_[p]->Vposition_=agents_[p]->position_;
				agents_[p]->VprefVelocity_= agents_[p]->prefVelocity_ ;
				agents_[p]->Vvelocity_=agents_[p]->velocity_;
			}
		}
			
		setCnavPoliteness(agentID,avgPolitenessCNav,actionNum);
		setALANPoliteness(agentID,avgPolitenessALAN,actionNum);
		setGoalProg(agentID,avgGoalProg,actionNum);

		agents_[agentID]->prefVelocity_= prefVelocityBackup_;

		return reward/(endOfPlan); //Returns the average reward for all timesteps
		
	}	
		
	
	const Vector2 &RVOSimulator::getNeighPredVel(size_t agentNo, size_t neighbor, int t) const
	{
		return agents_[agentNo]->predVelocityNeigh_[neighbor][t];
	}
	

	void RVOSimulator::buildTree()
	{
		kdTree_->buildAgentTree();
	}
    
	void RVOSimulator::setVvalues()
	{
		for (int i = 0; i < static_cast<int>(agents_.size()); ++i){
			agents_[i]->Vposition_ = agents_[i]->position_;
			agents_[i]->Vvelocity_ = agents_[i]->velocity_;		
		}
	} 
	
	
	float RVOSimulator::getSimComp(size_t agentNo, int component, int action, int timestep){
		if(component == 1){
			return agents_[agentNo]->SimComp1_t[action][timestep];
		}else{
			return agents_[agentNo]->SimComp2_t[action][timestep];
		}
	}
	
	
	const Vector2 &RVOSimulator::getSimNeighVel(size_t agentNo, size_t neighbor, int action, int timestep) const
	{
		return  agents_[agentNo]->SimVelNeigh_t[neighbor][action][timestep];	
	}
	
	
	const Vector2 &RVOSimulator::getSimNeighPos(size_t agentNo, size_t neighbor, int action, int timestep) const
	{
		
		return  agents_[agentNo]->SimPosNeigh_t[neighbor][action][timestep];
		
	}

	size_t RVOSimulator::addAgent(const Vector2 &position)
	{
		if (defaultAgent_ == NULL) {
			return RVO_ERROR;
		}
		
		Agent *agent = new Agent(this);
		
		agent->position_ = position;
		agent->maxNeighbors_ = defaultAgent_->maxNeighbors_;
		agent->maxSpeed_ = defaultAgent_->maxSpeed_;
		agent->neighborDist_ = defaultAgent_->neighborDist_;
		agent->radius_ = defaultAgent_->radius_;
		agent->timeHorizon_ = defaultAgent_->timeHorizon_;
		agent->timeHorizonObst_ = defaultAgent_->timeHorizonObst_;
		agent->velocity_ = defaultAgent_->velocity_;
		
		agent->id_ = agents_.size();
		
		agents_.push_back(agent);
		
		return agents_.size() - 1;
	}
	
	size_t RVOSimulator::addAgent(const Vector2 &position, float neighborDist, size_t maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, const Vector2 &velocity)
	{
		Agent *agent = new Agent(this);		
		agent->position_ = position;
		agent->maxNeighbors_ = maxNeighbors;
		agent->maxSpeed_ = maxSpeed;
		agent->neighborDist_ = neighborDist;
		agent->radius_ = radius;
		agent->timeHorizon_ = timeHorizon;
		agent->timeHorizonObst_ = timeHorizonObst;
		agent->velocity_ = velocity;
		agent->id_ = agents_.size();
		agents_.push_back(agent);
		
		return agents_.size() - 1;
	}
	
	size_t RVOSimulator::addObstacle(const std::vector<Vector2> &vertices)
	{
		if (vertices.size() < 2) {
			return RVO_ERROR;
		}
		
		const size_t obstacleNo = obstacles_.size();
		
		for (size_t i = 0; i < vertices.size(); ++i) {
			Obstacle *obstacle = new Obstacle();
			obstacle->point_ = vertices[i];
			
			if (i != 0) {
				obstacle->prevObstacle_ = obstacles_.back();
				obstacle->prevObstacle_->nextObstacle_ = obstacle;
			}
			
			if (i == vertices.size() - 1) {
				obstacle->nextObstacle_ = obstacles_[obstacleNo];
				obstacle->nextObstacle_->prevObstacle_ = obstacle;
			}
			
			obstacle->unitDir_ = normalize(vertices[(i == vertices.size() - 1 ? 0 : i + 1)] - vertices[i]);
			
			if (vertices.size() == 2) {
				obstacle->isConvex_ = true;
			}
			else {
				obstacle->isConvex_ = (leftOf(vertices[(i == 0 ? vertices.size() - 1 : i - 1)], vertices[i], vertices[(i == vertices.size() - 1 ? 0 : i + 1)]) >= 0.0f);
			}
			
			obstacle->id_ = obstacles_.size();
			
			obstacles_.push_back(obstacle);
		}
		
		return obstacleNo;
	}
	
	void RVOSimulator::doStep()
	{
		kdTree_->buildAgentTree();
		
#ifdef _OPENMP
#pragma omp parallel for
#endif
		
		for (int i = 0; i < static_cast<int>(agents_.size()); ++i) 
		{			
			agents_[i]->computeNeighbors();
			agents_[i]->computeNewVelocity();
		}
#ifdef _OPENMP
#pragma omp parallel for
#endif
		for (int i = 0; i < static_cast<int>(agents_.size()); ++i) 
		{
			agents_[i]->update();
		}
		
		globalTime_ += timeStep_;
	}
	
	size_t RVOSimulator::getAgentAgentNeighbor(size_t agentNo, size_t neighborNo) const
	{
		return agents_[agentNo]->agentNeighbors_[neighborNo].second->id_;
	}
	
	int RVOSimulator::getAgentAgentNeighborConstraint(size_t agentNo, size_t neighborNo) const
	{
		return agents_[agentNo]->constraint_[neighborNo];
	}
	
	float RVOSimulator::getAgentConstraint(size_t agentNo, size_t neighborNo) const
	{
		return agents_[agentNo]->NonOrderConstraint_[neighborNo];
	}
	
	size_t RVOSimulator::getAgentAgentSimilarity(size_t agentNo, size_t neighborNo) const
	{
		return agents_[agentNo]->similarity_[neighborNo];
	}
	
	size_t RVOSimulator::getAgentMaxNeighbors(size_t agentNo) const
	{
		return agents_[agentNo]->maxNeighbors_;
	}
	
	float RVOSimulator::getAgentMaxSpeed(size_t agentNo) const
	{
		return agents_[agentNo]->maxSpeed_;
	}
	
	float RVOSimulator::getAgentNeighborDist(size_t agentNo) const
	{
		return agents_[agentNo]->neighborDist_;
	}
	
	float RVOSimulator::getAgentNeighborObsSim(size_t agentNo,size_t neighborNo) const
	{
		return agents_[agentNo]->observedSim[neighborNo];
	}
	
	float RVOSimulator::getAgentNeighborExtSim(size_t agentNo,size_t neighborNo) const
	{
		return agents_[agentNo]->externalSim[neighborNo];
	}
	
	size_t RVOSimulator::getAgentNumAgentNeighbors(size_t agentNo) const
	{
		return agents_[agentNo]->agentNeighbors_.size();
	}
	
	size_t RVOSimulator::getAgentNumObstacleNeighbors(size_t agentNo) const
	{
		return agents_[agentNo]->obstacleNeighbors_.size();
	}
	
	size_t RVOSimulator::getAgentNumORCALines(size_t agentNo) const
	{
		return agents_[agentNo]->orcaLines_.size();
	}
	
	size_t RVOSimulator::getAgentObstacleNeighbor(size_t agentNo, size_t neighborNo) const
	{
		return agents_[agentNo]->obstacleNeighbors_[neighborNo].second->id_;
	}
	
	const Line &RVOSimulator::getAgentORCALine(size_t agentNo, size_t lineNo) const
	{
		return agents_[agentNo]->orcaLines_[lineNo];
	}
	
	const Vector2 &RVOSimulator::getAgentPosition(size_t agentNo) const
	{
		return agents_[agentNo]->position_;
	}
	
	const Vector2 &RVOSimulator::getAgentPrefVelocity(size_t agentNo) const
	{
		return agents_[agentNo]->prefVelocity_;	
	}
	
	const Vector2 &RVOSimulator::getAgentGoal(size_t agentNo) const
	{
		return agents_[agentNo]->goal_;
	}
	
	
	const Vector2 &RVOSimulator::getAgentGoalPrefVelocity(size_t agentNo) const
	{
		return agents_[agentNo]->goalprefVelocity_;
	}
	
	
	const Vector2 &RVOSimulator::getAgentPrefVelocity0(size_t agentNo) const
	{	
		return agents_[agentNo]->prefVelocity0_;	
	}
	
	const Vector2 &RVOSimulator::getAgentPredVelocity(size_t agentNo, int t) const
	{
		return agents_[agentNo]->predVelocity_[t];	
	}
	
	const Vector2 &RVOSimulator::getAgentPrefVelocity1(size_t agentNo) const
	{
		return agents_[agentNo]->prefVelocity1_;
	}
	
	const Vector2 &RVOSimulator::getAgentPrefVelocity2(size_t agentNo) const
	{
		return agents_[agentNo]->prefVelocity2_;
	}
	
	const Vector2 &RVOSimulator::getAgentPrefVelocity3(size_t agentNo) const
	{
		return agents_[agentNo]->prefVelocity3_;
	}
	
	const Vector2 &RVOSimulator::getAgentInitialPrefVelocity(size_t agentNo, int actionNum) const
	{
		return agents_[agentNo]->initprefvelocity_[actionNum];
	}
	
	const Vector2 &RVOSimulator::getAgentPrefVelocity4(size_t agentNo) const
	{
		return agents_[agentNo]->prefVelocity4_;	
	}
	
	float RVOSimulator::getAgentRadius(size_t agentNo) const
	{
		return agents_[agentNo]->radius_;
	}
	
	float RVOSimulator::getAgentTimeHorizon(size_t agentNo) const
	{
		return agents_[agentNo]->timeHorizon_;
	}
	
	float RVOSimulator::getAgentTimeHorizonObst(size_t agentNo) const
	{
		return agents_[agentNo]->timeHorizonObst_;
	}
	
	const Vector2 &RVOSimulator::getAgentVelocity(size_t agentNo) const
	{
		return agents_[agentNo]->velocity_;
	}
	
	const Vector2 &RVOSimulator::getAgentVelocity0(size_t agentNo) const
	{
		return agents_[agentNo]->velocity0_;
	}
	
	const Vector2 &RVOSimulator::getAgentInitialVelocity(size_t agentNo, int actionNum) const
	{
		return agents_[agentNo]->initvelocity_[actionNum];
	}
	
	const Vector2 &RVOSimulator::getAgentVelocity1(size_t agentNo) const
	{
		return agents_[agentNo]->velocity1_;
	}
	
	const Vector2 &RVOSimulator::getAgentVelocity2(size_t agentNo) const
	{
		return agents_[agentNo]->velocity2_;
	}
	
	const Vector2 &RVOSimulator::getAgentVelocity3(size_t agentNo) const
	{
		return agents_[agentNo]->velocity3_;
	}
	
	const Vector2 &RVOSimulator::getAgentVelocity4(size_t agentNo) const
	{
		return agents_[agentNo]->velocity4_;
	}
	
	
	void RVOSimulator::setAgentPrefVelocity0(size_t agentNo, const Vector2 &prefVelocity)
	{
		agents_[agentNo]->prefVelocity0_ = prefVelocity;
	}
	
	void RVOSimulator::setAgentPrefVelocity1(size_t agentNo, const Vector2 &prefVelocity)
	{
		agents_[agentNo]->prefVelocity1_ = prefVelocity;
	}
	
	void RVOSimulator::setAgentPrefVelocity2(size_t agentNo, const Vector2 &prefVelocity)
	{
		agents_[agentNo]->prefVelocity2_ = prefVelocity;
	}
	
	void RVOSimulator::setAgentPrefVelocity3(size_t agentNo, const Vector2 &prefVelocity)
	{
		agents_[agentNo]->prefVelocity3_ = prefVelocity;
	}
	
	
	void RVOSimulator::setAgentPrefVelocity4(size_t agentNo, const Vector2 &prefVelocity)
	{
		agents_[agentNo]->prefVelocity4_ = prefVelocity;
	}
	
	float RVOSimulator::getGlobalTime() const
	{
		return globalTime_;
	}
	
	size_t RVOSimulator::getNumAgents() const
	{
		return agents_.size();
	}
	
	size_t RVOSimulator::getNumObstacleVertices() const
	{
		return obstacles_.size();
	}
	
	size_t RVOSimulator::getAgentMostSimilarAgent(size_t agentNo)
	{
		return agents_[agentNo]->MostSimilar_;
	}
	
	size_t RVOSimulator::getAgentMostConstrainedAgent(size_t agentNo)
	{
		return agents_[agentNo]->MostConstrained_;
	}
	
	size_t RVOSimulator::getAgentLeastConstrainedAgent(size_t agentNo)
	{
		return agents_[agentNo]->LeastConstrained_;
	}
	
	const Vector2 &RVOSimulator::getObstacleVertex(size_t vertexNo) const
	{
		return obstacles_[vertexNo]->point_;
	}
	
	size_t RVOSimulator::getNextObstacleVertexNo(size_t vertexNo) const
	{
		return obstacles_[vertexNo]->nextObstacle_->id_;
	}
	
	size_t RVOSimulator::getPrevObstacleVertexNo(size_t vertexNo) const
	{
		return obstacles_[vertexNo]->prevObstacle_->id_;
	}
	
	float RVOSimulator::getTimeStep() const
	{
		return timeStep_;
	}
	
	void RVOSimulator::processObstacles()
	{
		kdTree_->buildObstacleTree();
	}
	
	bool RVOSimulator::queryVisibility(const Vector2 &point1, const Vector2 &point2, float radius) const
	{
		return kdTree_->queryVisibility(point1, point2, radius);
	}
	
	void RVOSimulator::setAgentInGoal(size_t agentNo,bool pos)
	{
		agents_[agentNo]->inGoal_ = pos;
		
	}
	
	void RVOSimulator::setAgentDefaults(float neighborDist, size_t maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, const Vector2 &velocity)
	{
		if (defaultAgent_ == NULL) {
			defaultAgent_ = new Agent(this);
		}
		
		defaultAgent_->maxNeighbors_ = maxNeighbors;
		defaultAgent_->maxSpeed_ = maxSpeed;
		defaultAgent_->neighborDist_ = neighborDist;
		defaultAgent_->radius_ = radius;
		defaultAgent_->timeHorizon_ = timeHorizon;
		defaultAgent_->timeHorizonObst_ = timeHorizonObst;
		defaultAgent_->velocity_ = velocity;
		defaultAgent_->numInteractions = 0;
		defaultAgent_->MostSimilar_ = -1000;
		
		int i;
		
		for (i=0; i<100; i++)
		{
			defaultAgent_->avgNeighInfluence[i]=0;	
			defaultAgent_->numNeighInfluence[i]=0;
			defaultAgent_->intentionSim[i]=0;
			defaultAgent_->observedSim[i]=0;
			defaultAgent_->externalSim[i]=0;
			
		}
		
		for (i=0; i<15; i++)
		{
			defaultAgent_->constraint_[i]=-1000;	
			defaultAgent_->similarity_[i]=-1000;
		}
	}
	
	void RVOSimulator::setAgentMaxNeighbors(size_t agentNo, size_t maxNeighbors)
	{
		agents_[agentNo]->maxNeighbors_ = maxNeighbors;
	}
	
	void RVOSimulator::setAgentMaxSpeed(size_t agentNo, float maxSpeed)
	{
		agents_[agentNo]->maxSpeed_ = maxSpeed;
	}
	
	void RVOSimulator::setAgentGoal(size_t agentNo, const Vector2 &goal)
	{
		agents_[agentNo]->goal_ = goal;
	}
	
	void RVOSimulator::setAgentNeighborDist(size_t agentNo, float neighborDist)
	{
		agents_[agentNo]->neighborDist_ = neighborDist;
	}
	
	void RVOSimulator::setAgentPosition(size_t agentNo, const Vector2 &position)
	{
		agents_[agentNo]->position_ = position;
	}
	
	void RVOSimulator::setAgentPrefVelocity(size_t agentNo, const Vector2 &prefVelocity)
	{
		agents_[agentNo]->prefVelocity_ = prefVelocity;
	}
	
	void RVOSimulator::setAgentGoalPrefVelocity(size_t agentNo, const Vector2 &goalprefVelocity)
	{
		agents_[agentNo]->goalprefVelocity_ = goalprefVelocity;
	}
	
	void RVOSimulator::setAgentAgentNeighborConstraint(size_t agentNo, size_t rank, size_t rankid) 
	{ // agentNo is the base agent, neighor
		agents_[agentNo]->constraint_[rank]=rankid;
	}
	
	void RVOSimulator::setAgentAgentSimilarity(size_t agentNo, size_t rank, size_t rankid) 
	{ // agentNo is the base agent, neighor
		agents_[agentNo]->similarity_[rank]=rankid;
	}
	
	void RVOSimulator::setAgentRadius(size_t agentNo, float radius)
	{
		agents_[agentNo]->radius_ = radius;
	}
	
	void RVOSimulator::setAgentMostSimilarAgent(size_t agentNo, size_t agentNo2)
	{
		agents_[agentNo]->MostSimilar_ = agentNo2;
	}
	
	void RVOSimulator::setAgentMostConstrainedAgent(size_t agentNo, size_t agentNo2)
	{
		agents_[agentNo]->MostConstrained_ = agentNo2;
	}
	
	void RVOSimulator::setAgentLeastConstrainedAgent(size_t agentNo, size_t agentNo2)
	{
		agents_[agentNo]->LeastConstrained_ = agentNo2;
	}
	
	void RVOSimulator::setAgentTimeHorizon(size_t agentNo, float timeHorizon)
	{
		agents_[agentNo]->timeHorizon_ = timeHorizon;
	}
	
	void RVOSimulator::setAgentTimeHorizonObst(size_t agentNo, float timeHorizonObst)
	{
		agents_[agentNo]->timeHorizonObst_ = timeHorizonObst;
	}
	
	void RVOSimulator::setAgentNeighAngle(size_t agentNo, size_t Neigh, float angle)
	{
		agents_[agentNo]->NeighAngle[Neigh] = angle;
	}
	
	
	
	void RVOSimulator::setAgentVelocity(size_t agentNo, const Vector2 &velocity)
	{
		agents_[agentNo]->velocity_ = velocity;
	}
	
	
	void RVOSimulator::setCnavPoliteness(size_t agentNo, float avgPolitenessCNav,int actionNum)
	{
		agents_[agentNo]->cnavpoliteness_[actionNum] =avgPolitenessCNav;
		
	}
	
	void RVOSimulator::setALANPoliteness(size_t agentNo, float avgPolitenessALAN,int actionNum)
	{
		agents_[agentNo]->alanpoliteness_[actionNum] =avgPolitenessALAN;
		
	}
	
	float  RVOSimulator::getAgentCnavPoliteness(size_t agentNo, int actionNum)
	{
		return agents_[agentNo]->cnavpoliteness_[actionNum];
	}
	
	float  RVOSimulator::getAgentALANPoliteness(size_t agentNo, int actionNum)
	{
		return agents_[agentNo]->alanpoliteness_[actionNum];
	}
	
	float  RVOSimulator::getAgentGoalProg(size_t agentNo, int actionNum)
	{
		return agents_[agentNo]->goalprog_[actionNum];
	}
	
	void RVOSimulator::setGoalProg(size_t agentNo, float avgGoalProg,int actionNum)
	{
		agents_[agentNo]->goalprog_[actionNum] =avgGoalProg;
		
	}
	
	void RVOSimulator::setAgentInitialVelocity(size_t agentNo, const Vector2 &velocity, int actionNum)
	{
		agents_[agentNo]->initvelocity_[actionNum] = velocity;
	}
	
	
	void RVOSimulator::setAgentInitialPrefVelocity(size_t agentNo, const Vector2 &velocity, int actionNum)
	{
		agents_[agentNo]->initprefvelocity_[actionNum] = velocity;
	}
	
	
	void RVOSimulator::setTimeStep(float timeStep)
	{
		timeStep_ = timeStep;
	}
}
