/*
 *  Copyright AIST-CNRS Joint Robotics Laboratory
 *  Author: Nicolas Perrin
 *
 */

#ifndef NEWBOUNDINGBOXPLANNER_H
#define NEWBOUNDINGBOXPLANNER_H

#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>

#include <stdio.h>

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

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace std;

class CnewBoundingBoxPlanner
{

	public:

		/*!
		 * Constructor.
		 */
		CnewBoundingBoxPlanner() {
			    ai_env = NULL;
			    ai_lowerBBOX = NULL;
			    ai_upperBBOX = NULL;
			    ai_zoneRight = NULL;
			    ai_zoneLeft = NULL;
			    ai_zoneRightGOAL = NULL;
			    ai_zoneLeftGOAL = NULL;     
		}

		/*!
		 * Destructor.
		 */
		~CnewBoundingBoxPlanner() {
			    delete ai_env;
			    delete ai_lowerBBOX;
			    delete ai_upperBBOX;
			    delete ai_zoneRight;
			    delete ai_zoneLeft;
			    delete ai_zoneRightGOAL;
			    delete ai_zoneLeftGOAL;     
		}
		
		const struct aiScene* ai_env;
		PQP_Model pqp_env;
		
		const struct aiScene* ai_lowerBBOX;
		const struct aiScene* ai_upperBBOX;
		PQP_Model pqp_lowerBBOX;
		PQP_Model pqp_upperBBOX;
		aiMatrix4x4 lowerBBOX_init_matrix;
		aiMatrix4x4 upperBBOX_init_matrix;
		
		const struct aiScene* ai_zoneRight;
		const struct aiScene* ai_zoneLeft;
		const struct aiScene* ai_zoneRightGOAL;
		const struct aiScene* ai_zoneLeftGOAL;
		aiMatrix4x4 ai_zoneRight_init_matrix;
		aiMatrix4x4 ai_zoneLeft_init_matrix;
		aiMatrix4x4 ai_zoneRightGOAL_init_matrix;
		aiMatrix4x4 ai_zoneLeftGOAL_init_matrix;
		
		vector<aiMatrix4x4> footprint_matrixes;
		
		void plan(
			    SE2 & startSE2, 
			    SE2 & goalSE2,
			    LoR firstsupportfoot
 			) {
		    
		    plan_and_build_discrete_phi_trajectory(
			    startSE2, 
			    goalSE2);
		    from_discrete_to_continuous_phi_trajectory(
			    discrete_phi_trajectory, 
			    continuous_phi_trajectory);
		    build_lowerBBOX_trajectory(
			    firstsupportfoot,
			    startSE2,
			    continuous_phi_trajectory, 
			    lowerBBOXTrajectory, 
			    whichlowerBBOX, 
			    footprint_matrixes, 
			    footprint_vector,
			    pqp_lowerBBOX, 
			    lowerBBOX_init_matrix, 
			    ai_env, 
			    pqp_env);    
		}
			    
		void build_robot_footsteps (
			    CnewSliderPG * sliderPG,
			    Chrp2Robot & robo,
			    const SE2 & robo_startSE2,
			    const char * pg_config, 
			    const char * pos_file, 
			    const char * zmp_file, 
			    const char * wst_file) {
		    
		    build_robot_footsteps(
			    sliderPG, 
			    robo, 
			    footprint_vector,
			    robo_startSE2, 
			    pg_config, 
			    pos_file, 
			    zmp_file, 
			    wst_file);   
		}
		
		void do_motion_phi (
			    int time_elapsed) {
		    
		    do_motion_phi (
			    time_elapsed,
			    discrete_phi_trajectory,
			    continuous_phi_trajectory,
			    ai_lowerBBOX,    
			    lowerBBOXTrajectory,
			    lowerBBOX_init_matrix,
			    ai_zoneRight,
			    ai_zoneLeft,
			    ai_zoneRight_init_matrix,
			    ai_zoneLeft_init_matrix,
			    ai_zoneRightGOAL,
			    ai_zoneLeftGOAL);       
		}

	private:
	    
		vector< vector<float> > footprint_vector;
		vector< vector<float> > discrete_phi_trajectory;
		vector< vector<float> > continuous_phi_trajectory;
		vector< vector<float> > lowerBBOXTrajectory;
		vector< int > whichlowerBBOX;
	    
		vector<float>  phi_sampler(
			    float x, 
			    float y, 
			    float yaw, 
			    float zcoef, 
			    int right0_left1);
		
		bool phi_verifier(
			    float dx, 
			    float dy, 
			    float dyaw, 
			    float yaw, 
			    float zcoef, 
			    int right0_left1);
		
		bool isStateValid(
			    float xx, 
			    float yy, 
			    float zz,
			    float yawy, 
			    PQP_Model &pqp_objectLOWER, 
			    PQP_Model &pqp_objectUPPER,
			    aiMatrix4x4 & ai_object_init_matrixLOWER, 
			    aiMatrix4x4 & ai_object_init_matrixUPPER, 
			    const struct aiScene* &ai_env,
			    PQP_Model &pqp_env);
		
		bool isStateValid(
			    const ob::State *state, 
			    PQP_Model &pqp_objectLOWER, 
			    PQP_Model &pqp_objectUPPER, 
			    aiMatrix4x4 & ai_object_init_matrixLOWER, 
			    aiMatrix4x4 & ai_object_init_matrixUPPER,
			    const struct aiScene* &ai_env, 
			    PQP_Model &pqp_env);
		
		bool isStateValid_default(
			    const ob::State *state);
		
		void plan_and_build_discrete_phi_trajectory(
			    SE2 & startSE2, 
			    SE2 & goalSE2);
		
		int update_lowerBBOX_config(
			    int index, 
			    vector< vector<float> > & c_phi_trajectory,
			    vector< vector<float> > & lower_bounding_box_trajectory,
			    vector< int > & which_lower_bounding_box,
			    vector<aiMatrix4x4> & footprints,
			    vector< vector<float> > & fprints_vector,
			    PQP_Model & pqp_lower_bounding_box, 
			    aiMatrix4x4 & ai_lower_bounding_box_init_matrix, 
			    const struct aiScene* &ai_env, 
			    PQP_Model &pqp_env);
		
		void from_discrete_to_continuous_phi_trajectory(
			    vector< vector<float> > & d_phi_trajectory, 
			    vector< vector<float> > & c_phi_trajectory);
		
		int build_lowerBBOX_trajectory(
			    LoR firstsupportfoot,
			    SE2 & start_pos_and_orient,
			    vector< vector<float> > & c_phi_trajectory,
			    vector< vector<float> > & lower_bounding_box_trajectory,
			    vector< int > & which_lower_bounding_box,
			    vector<aiMatrix4x4> & footprints,
			    vector< vector<float> > & fprints_vector,
			    PQP_Model & pqp_lower_bounding_box, 
			    aiMatrix4x4 & ai_lower_bounding_box_init_matrix, 
			    const struct aiScene* &ai_env, 
			    PQP_Model &pqp_env);
		
		void do_motion_phi (
			    int time_elapsed,
			    vector< vector<float> > & d_phi_trajectory, 
			    vector< vector<float> > & c_phi_trajectory,
			    const struct aiScene* ai_lower_bounding_box, 
			    vector< vector<float> > & lower_bounding_box_trajectory,    
			    aiMatrix4x4 & ai_lower_bouding_box_init_matrix,
			    const struct aiScene* ai_phizone_right, 
			    const struct aiScene* ai_phizone_left, 
			    aiMatrix4x4 & ai_phizone_right_init_matrix,
			    aiMatrix4x4 & ai_phizone_left_init_matrix,      
			    const struct aiScene* ai_phizone_right_GOAL,  
			    const struct aiScene* ai_phizone_left_GOAL);
		
		void build_robot_footsteps (
			    CnewSliderPG * sliderPG,
			    Chrp2Robot & robo,
			    vector< vector<float> > & fprints_vector,
			    const SE2 & robo_startSE2,
			    const char * pg_config, 
			    const char * pos_file, 
			    const char * zmp_file, 
			    const char * wst_file);
};

#endif
