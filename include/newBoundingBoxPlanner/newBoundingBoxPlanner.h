/*
* Copyright 2010, 2011
*
* Nicolas Perrin,
* Olivier Stasse,
* Florent Lamiraux,
* Eiichi Yoshida
*
*
* JRL/LAAS, CNRS/AIST
*
* This file is part of newBoundingBoxPlanner.
* newBoundingBoxPlanner is a free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* newBoundingBoxPlanner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Lesser Public License for more details.
* You should have received a copy of the GNU Lesser General Public License
* along with newSliderPG. If not, see <http://www.gnu.org/licenses/>.
*
* Research carried out within the scope of the Associated
* International Laboratory: Joint Japanese-French Robotics
* Laboratory (JRL)
*
*/

/*Now on the "newstructure" branch.*/

#ifndef NEWBOUNDINGBOXPLANNER_H
#define NEWBOUNDINGBOXPLANNER_H

#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>
#include <math>

// #include <stdio.h>

// assimp include files. These three are usually needed.
#include "assimp.h"
#include "aiPostProcess.h"
#include "aiScene.h"

#include "PQP.h"

#include <hrp2Robot/hrp2Robot.h>
#include <newSliderPG/newSliderPG.h>

#include <ompl/base/Planner.h>
#include <ompl/base/Goal.h>
#include <ompl/base/Path.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ValidStateSampler.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/StateManifold.h>
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/base/ManifoldStateSampler.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/PathSimplifier.h>

#include <ompl/control/SpaceInformation.h>
#include <ompl/base/GoalState.h>
#include <ompl/control/manifolds/RealVectorControlManifold.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

using namespace std;

struct model3d {
    PQP_model * pqp;
    const struct aiScene * ai_object; 
    const aiMatrix4x4 * init_Tmatrix;
}

class CnewBoundingBoxPlanner
{

	public:

		/*!
		 * Constructor.
		 */
		CnewBoundingBoxPlanner() {
// 			    ai_env = NULL;
// 			    ai_lowerBBOX = NULL;
// 			    ai_upperBBOX = NULL;
// 			    ai_zoneRight = NULL;
// 			    ai_zoneLeft = NULL;
// 			    ai_zoneRightGOAL = NULL;
// 			    ai_zoneLeftGOAL = NULL;     
		}

		/*!
		 * Destructor.
		 */
		~CnewBoundingBoxPlanner() {
/*			    delete ai_env;
			    delete ai_lowerBBOX;
			    delete ai_upperBBOX;
			    delete ai_zoneRight;
			    delete ai_zoneLeft;
			    delete ai_zoneRightGOAL;
			    delete ai_zoneLeftGOAL;  */   
		}
		
// 		const struct aiScene* ai_env;
// 		PQP_Model pqp_env;
		
// 		const struct aiScene* ai_lowerBBOX;
// 		const struct aiScene* ai_upperBBOX;
// 		PQP_Model pqp_lowerBBOX;
// 		PQP_Model pqp_upperBBOX;
// 		aiMatrix4x4 lowerBBOX_init_matrix;
// 		aiMatrix4x4 upperBBOX_init_matrix;
		
// 		const struct aiScene* ai_zoneRight;
// 		const struct aiScene* ai_zoneLeft;
// 		const struct aiScene* ai_zoneRightGOAL;
// 		const struct aiScene* ai_zoneLeftGOAL;
// 		aiMatrix4x4 ai_zoneRight_init_matrix;
// 		aiMatrix4x4 ai_zoneLeft_init_matrix;
// 		aiMatrix4x4 ai_zoneRightGOAL_init_matrix;
// 		aiMatrix4x4 ai_zoneLeftGOAL_init_matrix;
		
// 		vector<aiMatrix4x4> footprint_matrixes;
		
		model3d leftBbox;
		model3d rightBbox,
		model3d upperBbox,
		vector< model3d > obstacle_list;
		
		SE2 randomGoal();
		
// 		void plan(
// 			    SE2 & startSE2, 
// 			    SE2 & goalSE2,
// 			    LoR firstsupportfoot) {
// 		    
// 		    plan_and_build_discrete_phi_trajectory(
// 			    startSE2, 
// 			    goalSE2);
// 		    from_discrete_to_continuous_phi_trajectory(
// 			    discrete_phi_trajectory, 
// 			    continuous_phi_trajectory);
// 		    build_lowerBBOX_trajectory(
// 			    firstsupportfoot,
// 			    startSE2,
// 			    continuous_phi_trajectory, 
// 			    lowerBBOXTrajectory, 
// 			    whichlowerBBOX, 
// 			    footprint_matrixes, 
// 			    footprint_vector,
// 			    pqp_lowerBBOX, 
// 			    lowerBBOX_init_matrix, 
// 			    ai_env, 
// 			    pqp_env);    
// 		}
		
		void plan_phi_trajectory(
			    SE2 & start_state, 
			    SE2 & goal_state);
		
		
// 		void plan_phi(
// 			    SE2 & startSE2, 
// 			    SE2 & goalSE2,
// 			    LoR firstsupportfoot) {
// 		    
// 		    plan_and_build_discrete_phi_trajectory(
// 			    startSE2, 
// 			    goalSE2);
// 		    from_discrete_to_continuous_phi_trajectory(
// 			    discrete_phi_trajectory, 
// 			    continuous_phi_trajectory);  
// 		}
		
		//When it's done, startSE2, firstLeftFoot and firstRightFoot are updated.
// 		void plan_steps(
// 			    vector<HalfStep> & v,
// 			    trajFeatures & t, 
// 			    CnewSliderPG * sliderPG,
// 			    const char * pg_config,
// 			    SE2 & startSE2,
// 			    SE2 & firstLeftFoot,
// 			    SE2 & firstRightFoot,
// 			    LoR firstsupportfoot) {
// 		    
// 		   progressive_build_lowerBBOX_trajectory(
// 			    v,
// 			    t,
// 			    sliderPG,
// 			    pg_config,
// 			    firstsupportfoot,
// 			    startSE2,
// 			    firstLeftFoot,
// 			    firstRightFoot,
// 			    continuous_phi_trajectory, 
// 			    lowerBBOXTrajectory, 
// 			    whichlowerBBOX, 
// 			    footprint_matrixes, 
// 			    footprint_vector,
// 			    pqp_lowerBBOX, 
// 			    lowerBBOX_init_matrix, 
// 			    ai_env, 
// 			    pqp_env);        
// 		}
		    
// 		void build_robot_footsteps (
// 			    CnewSliderPG * sliderPG,
// 			    Chrp2Robot & robo,
// 			    const SE2 & robo_startSE2,
// 			    const char * pg_config, 
// 			    const char * pos_file, 
// 			    const char * zmp_file, 
// 			    const char * wst_file) {
// 		    
// 		    build_robot_footsteps(
// 			    sliderPG, 
// 			    robo, 
// 			    footprint_vector,
// 			    robo_startSE2, 
// 			    pg_config, 
// 			    pos_file, 
// 			    zmp_file, 
// 			    wst_file);   
// 		}
		
// 		void do_motion_phi (
// 			    int time_elapsed) {
// 		    
// 		    do_motion_phi (
// 			    time_elapsed,
// 			    discrete_phi_trajectory,
// 			    continuous_phi_trajectory,
// 			    ai_lowerBBOX,    
// 			    lowerBBOXTrajectory,
// 			    lowerBBOX_init_matrix,
// 			    ai_zoneRight,
// 			    ai_zoneLeft,
// 			    ai_zoneRight_init_matrix,
// 			    ai_zoneLeft_init_matrix,
// 			    ai_zoneRightGOAL,
// 			    ai_zoneLeftGOAL);       
// 		}

	private:
	    
// 		vector< vector<float> > footprint_vector;
		vector< SE2 > continuous_phi_trajectory;
// 		vector< vector<float> > lowerBBOXTrajectory;
// 		vector< int > whichlowerBBOX;
		
		deque<SE2> leftBbox_valid_configs;
		deque<SE2> rightBbox_valid_configs;
		
		float distanceQuery(model3d & object, SE2 state, vector< model3d > & list_of_obstacles);
	    
		SE2 phi_sampler(   
			    //for now, SE2 is defined in newSliderPG, in halfStep_creation.h
			    SE2 & sample_result,
			    SE2 & state,
			    //for now, LoR (LEFT or RIGHT) is defined in newSliderPG, in halfStep_creation.h
			    LoR left_or_right);
		
		bool phi_verifier(
			    SE2 state_phi, 
			    SE2 state_Bbox,
			    LoR left_or_right);
// 		bool phi_verifier(
// 			    float dx, 
// 			    float dy, 
// 			    float dyaw, 
// 			    float yaw, 
// 			    float zcoef, 
// 			    int right0_left1);
		
// 		bool phi_verifier2(
// 			    float dx, 
// 			    float dy, 
// 			    float dyaw, 
// 			    float yaw, 
// 			    float zcoef, 
// 			    int right0_left1);
		
		bool isStateValid(
			    int number_of_attempts,
			    SE2 & state);
		
		bool isStateValid(
			    int number_of_attempts,
			    const ob::State *state);
		
		bool isStateValid_default(  
			    const ob::State *state);
		
		
// 		bool isStateValid_default(
// 			    const ob::State *state);
		
// 		void plan_and_build_discrete_phi_trajectory(
// 			    SE2 & start_state, 
// 			    SE2 & goal_state);
// 		
// 		void from_discrete_to_continuous_phi_trajectory(
// 			    vector< vector<float> > & d_phi_trajectory, 
// 			    vector< vector<float> > & c_phi_trajectory);
		
// 		int update_lowerBBOX_config(
// 			    int index, 
// 			    vector< vector<float> > & c_phi_trajectory,
// 			    vector< vector<float> > & lower_bounding_box_trajectory,
// 			    vector< int > & which_lower_bounding_box,
// 			    vector<aiMatrix4x4> & footprints,
// 			    vector< vector<float> > & fprints_vector,
// 			    PQP_Model & pqp_lower_bounding_box, 
// 			    aiMatrix4x4 & ai_lower_bounding_box_init_matrix, 
// 			    const struct aiScene* &ai_env, 
// 			    PQP_Model &pqp_env);

// 		int progressive_update_lowerBBOX_config(
// 			    vector<HalfStep> & v,
// 			    trajFeatures & t,    
// 			    CnewSliderPG * sliderPG,
// 			    int index, 
// 			    vector< vector<float> > & c_phi_trajectory,
// 			    vector< vector<float> > & lower_bounding_box_trajectory,
// 			    vector< int > & which_lower_bounding_box,
// 			    vector<aiMatrix4x4> & footprints,
// 			    vector< vector<float> > & fprints_vector,
// 			    PQP_Model & pqp_lower_bounding_box, 
// 			    aiMatrix4x4 & ai_lower_bounding_box_init_matrix, 
// 			    const struct aiScene* &ai_env, 
// 			    PQP_Model &pqp_env);
		
// 		int build_lowerBBOX_trajectory(
// 			    LoR firstsupportfoot,
// 			    SE2 & start_pos_and_orient,
// 			    vector< vector<float> > & c_phi_trajectory,
// 			    vector< vector<float> > & lower_bounding_box_trajectory,
// 			    vector< int > & which_lower_bounding_box,
// 			    vector<aiMatrix4x4> & footprints,
// 			    vector< vector<float> > & fprints_vector,
// 			    PQP_Model & pqp_lower_bounding_box, 
// 			    aiMatrix4x4 & ai_lower_bounding_box_init_matrix, 
// 			    const struct aiScene* &ai_env, 
// 			    PQP_Model &pqp_env);
		
// 		int progressive_build_lowerBBOX_trajectory(
// 			    vector<HalfStep> & v,
// 			    trajFeatures & t,   
// 			    CnewSliderPG * sliderPG,
// 			    const char * pg_config,
// 			    LoR firstsupportfoot,
// 			    SE2 & start_pos_and_orient,
// 			    SE2 & firstLeftFoot,
// 			    SE2 & firstRightFoot,
// 			    vector< vector<float> > & c_phi_trajectory,
// 			    vector< vector<float> > & lower_bounding_box_trajectory,
// 			    vector< int > & which_lower_bounding_box,
// 			    vector<aiMatrix4x4> & footprints,
// 			    vector< vector<float> > & fprints_vector,
// 			    PQP_Model & pqp_lower_bounding_box, 
// 			    aiMatrix4x4 & ai_lower_bounding_box_init_matrix, 
// 			    const struct aiScene* &ai_env, 
// 			    PQP_Model &pqp_env);
		
// 		void do_motion_phi (
// 			    int time_elapsed,
// 			    vector< vector<float> > & d_phi_trajectory, 
// 			    vector< vector<float> > & c_phi_trajectory,
// 			    const struct aiScene* ai_lower_bounding_box, 
// 			    vector< vector<float> > & lower_bounding_box_trajectory,    
// 			    aiMatrix4x4 & ai_lower_bouding_box_init_matrix,
// 			    const struct aiScene* ai_phizone_right, 
// 			    const struct aiScene* ai_phizone_left, 
// 			    aiMatrix4x4 & ai_phizone_right_init_matrix,
// 			    aiMatrix4x4 & ai_phizone_left_init_matrix,      
// 			    const struct aiScene* ai_phizone_right_GOAL,  
// 			    const struct aiScene* ai_phizone_left_GOAL);
		
// 		void build_robot_footsteps (
// 			    CnewSliderPG * sliderPG,
// 			    Chrp2Robot & robo,
// 			    vector< vector<float> > & fprints_vector,
// 			    const SE2 & robo_startSE2,
// 			    const char * pg_config, 
// 			    const char * pos_file, 
// 			    const char * zmp_file, 
// 			    const char * wst_file);
};

#endif
