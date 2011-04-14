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

#include "newBoundingBoxPlanner/newBoundingBoxPlanner.h"
#include "KPIECE1.h"
#include "BasicPRMmodif.h"
#include "SE2plusStateManifold.h"
#include "MatVec.h"

#ifndef PI
#define PI 3.1415926
#endif

#define aisgl_min(x,y) (x<y?x:y)
#define aisgl_max(x,y) (y>x?y:x)

// float modul(float a, float b)
// {
// int result = static_cast<int>( a / b );
// return a - static_cast<float>( result ) * b;
// }
float distanceQuery(PQP_Model &pqp_object, aiMatrix4x4 & ai_object_init_matrix, float x, float y, float theta, const struct aiScene* &ai_env, PQP_Model &pqp_env) {    
    
	aiVector3D vtest(x, 0.0, y);
	aiMatrix4x4 matr2;
	aiMatrix4x4::Translation(vtest, matr2);
	aiMatrix4x4 matrFPR;
	aiMatrix4x4::RotationY(theta, matrFPR);

	aiMatrix4x4 matr_env = ai_env->mRootNode->mTransformation;
	aiMatrix4x4 matr_obj = (matr2 * matrFPR) * ai_object_init_matrix;
	PQP_REAL Renv[3][3],Robj[3][3],Tenv[3],Tobj[3];
	for(unsigned int i=0; i<3; i++) {
		for(unsigned int j=0; j<3; j++) {
			Renv[i][j] = matr_env[i][j];
			Robj[i][j] = matr_obj[i][j];
		}
		Tenv[i] = matr_env[i][3];
		Tobj[i] = matr_obj[i][3];
	}
	PQP_REAL rel_err = 0.0;
	PQP_REAL abs_err = 0.0;
	PQP_DistanceResult res;
	PQP_Distance(&res,Renv,Tenv,&pqp_env,Robj,Tobj,&pqp_object,rel_err,abs_err);    
	
	return res.Distance();
}
// ----------------------------------------------------------------------------
vector<float>  CnewBoundingBoxPlanner::phi_sampler(float x, float y, float yaw, float zcoef, int right0_left1) { //0: we sample for the right foot, 1: we sample for the left foot
    vector<float> result(3);
    float lr_coef = right0_left1 ? -1.0 : 1.0;    
    float dx, dy, dyaw;
    for(unsigned int i = 0; i < 100; i++) {
	dx = (((float) rand())/((float) RAND_MAX + 1.0)*0.40*2 - 0.40);
	dy = (lr_coef * ( ((float) rand())/((float) RAND_MAX + 1.0)*(0.37-0.16) + 0.16 )); //0.37 != 0.40 => NOT CIRCULAR ANYMORE !!
	dyaw = 0.0; //((float) rand())/((float) RAND_MAX + 1.0)*20.0*PI/180.0 - 10*PI/180.0;
	if(dx*dx + dy*dy < 0.40*0.40 && (lr_coef * dy) > 0.16) {
	    result[0] = x + (dx * cos(yaw) + dy * sin(yaw)) * zcoef; 
	    result[1] = y + (dx * sin(yaw) - dy * cos(yaw)) * zcoef;
	    result[2] = yaw + dyaw;
	    return result;
	}
    }
    dx = 0.0;
    dy = 0.16 * lr_coef;
    result[0] = x + (dx * cos(yaw) + dy * sin(yaw)) * zcoef; 
    result[1] = y + (dx * sin(yaw) - dy * cos(yaw)) * zcoef;
    result[2] = yaw;
    return result;    
}
bool CnewBoundingBoxPlanner::phi_verifier(float dx, float dy, float dyaw, float yaw, float zcoef, int right0_left1) {    
    float lr_coef = right0_left1 ? -1.0 : 1.0; 
    float ddx = cos(yaw)*dx + sin(yaw)*dy;
    float ddy = sin(yaw)*dx - cos(yaw)*dy;
    return (ddx*ddx + ddy*ddy < 0.40*0.40*zcoef*zcoef && (lr_coef * ddy) > 0.16*zcoef && (lr_coef * ddy) < 0.37*zcoef && abs(dyaw) < 16.0*PI/180.0);
}
// ----------------------------------------------------------------------------
bool CnewBoundingBoxPlanner::isStateValid(float xx, float yy, float zz, float yawy, 
		  PQP_Model &pqp_objectLOWER, 
		  PQP_Model &pqp_objectUPPER,
		  aiMatrix4x4 & ai_object_init_matrixLOWER, 
		  aiMatrix4x4 & ai_object_init_matrixUPPER, 
		  const struct aiScene* &ai_env,
		  PQP_Model &pqp_env) {

    float x,y;
//     float yaw = 0.0;
    
    bool RightOK = false;
    bool LeftOK = false;
    
    float zcoefRIGHT = (zz + 1.0)/2.0;
    float zcoefLEFT = (zz - 1.0)/-2.0;
    
//     float xl = xx + ( -0.16 * sin(yawy)) * zcoefLEFT; 
//     float yl = yy - ( -0.16 * cos(yawy)) * zcoefLEFT; 
//     float xr = xx + ( 0.16 * sin(yawy)) * zcoefRIGHT; 
//     float yr = yy - ( 0.16 * cos(yawy)) * zcoefRIGHT;
//     
//     float resUp = distanceQuery(pqp_objectUPPER, ai_object_init_matrixUPPER, (xl + xr)/2.0, (yl + yr)/2.0, -yawy - PI/2.0, ai_env, pqp_env);
//     if(resUp <= 0.0001) return 0;
    
    for(unsigned int k = 0; k < 18; k++) {
	vector<float> vc = phi_sampler(xx, yy, yawy, zcoefRIGHT, 0);
	x = vc[0]; 
	y = vc[1];

	float res = distanceQuery(pqp_objectLOWER, ai_object_init_matrixLOWER, x, y, -yawy - PI/2.0, ai_env, pqp_env);
	
	if(res > 0.0001) {
	    RightOK = true;
	    break;
	}
    }
    for(unsigned int k = 0; k < 18; k++) {
	vector<float> vc = phi_sampler(xx, yy, yawy, zcoefLEFT, 1);
	x = vc[0]; 
	y = vc[1];

	float res = distanceQuery(pqp_objectLOWER, ai_object_init_matrixLOWER, x, y, -yawy - PI/2.0, ai_env, pqp_env);
	
	if(res > 0.0001) {
	    LeftOK = true;
	    break;
	}
    }
    
    if( RightOK && LeftOK ) return 1;
    
    return 0;        
}
bool CnewBoundingBoxPlanner::isStateValid(const ob::State *state, 
		  PQP_Model &pqp_objectLOWER, 
		  PQP_Model &pqp_objectUPPER, 
		  aiMatrix4x4 & ai_object_init_matrixLOWER, 
		  aiMatrix4x4 & ai_object_init_matrixUPPER,
		  const struct aiScene* &ai_env, 
		  PQP_Model &pqp_env) {
    
    const ob::SE2plusStateManifold::StateType *s = state->as<ob::SE2plusStateManifold::StateType>();
    
    float xx = s->getX();
    float yy = s->getY();
    float zz = s->getZ(); //+1 is for full right, -1 is for full left
    
    float yawy = s->getYaw();
 
    return isStateValid(xx, yy, zz, yawy, pqp_objectLOWER, pqp_objectUPPER, ai_object_init_matrixLOWER, ai_object_init_matrixUPPER, ai_env, pqp_env);
    
}
bool CnewBoundingBoxPlanner::isStateValid_default(const ob::State *state) {
    
    return isStateValid(state, pqp_lowerBBOX, pqp_upperBBOX, lowerBBOX_init_matrix, upperBBOX_init_matrix, ai_env, pqp_env);
    
}

bool testt(const ob::State * state) {
    return true;
}

void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
{
     const ob::SE2plusStateManifold::StateType *se2state = start->as<ob::SE2plusStateManifold::StateType>();
     const ob::RealVectorStateManifold::StateType *pos = se2state->as<ob::RealVectorStateManifold::StateType>(0);
     const ob::SO2StateManifold::StateType *rot = se2state->as<ob::SO2StateManifold::StateType>(1);
     const oc::RealVectorControlManifold::ControlType *rctrl = control->as<oc::RealVectorControlManifold::ControlType>();
 
     result->as<ob::SE2plusStateManifold::StateType>()->as<ob::RealVectorStateManifold::StateType>(0)->values[0] =
         (*rctrl)[0] > 0 ? (*pos)[0] + (*rctrl)[0] * duration * cos(rot->value) : (*pos)[0];
 
     result->as<ob::SE2plusStateManifold::StateType>()->as<ob::RealVectorStateManifold::StateType>(0)->values[1] =
         (*rctrl)[0] > 0 ? (*pos)[1] + (*rctrl)[0] * duration * sin(rot->value) : (*pos)[1];
	 
     result->as<ob::SE2plusStateManifold::StateType>()->as<ob::RealVectorStateManifold::StateType>(0)->values[2] =
         aisgl_min(0.99, aisgl_max(-0.99, (*pos)[1] + (*rctrl)[1] * duration));
 
     result->as<ob::SE2plusStateManifold::StateType>()->as<ob::SO2StateManifold::StateType>(1)->value =
         (*rctrl)[0] > 0 ? rot->value : rot->value + (2.0 - (*rctrl)[0]) * duration;
}


// ----------------------------------------------------------------------------
void CnewBoundingBoxPlanner::plan_and_build_discrete_phi_trajectory(SE2 & startSE2, SE2 & goalSE2)
{ 
    
    // construct the manifold we are planning in    
    ob::StateManifoldPtr manifold(new ob::SE2plusStateManifold()); 
    
    ob::RealVectorBounds bounds(3);
    bounds.setLow(0, -3.0);
    bounds.setHigh(0, 1.0);
    bounds.setLow(1, -2.0);
    bounds.setHigh(1, 2.0);
    
    bounds.setLow(2, -0.99);
    bounds.setHigh(2, 0.99);

    manifold->as<ob::SE2plusStateManifold>()->setBounds(bounds);

    // create a control manifold
    oc::ControlManifoldPtr cmanifold(new oc::RealVectorControlManifold(manifold, 3));
    // set the bounds for the control manifold
    ob::RealVectorBounds cbounds(3);
    cbounds.setLow(-4.0);
    cbounds.setHigh(4.0);
    
    cmanifold->as<oc::RealVectorControlManifold>()->setBounds(cbounds);
    // set the state propagation routine
    cmanifold->setPropagationFunction(boost::bind(&propagate, _1, _2, _3, _4));
    
    oc::SpaceInformationPtr si(new oc::SpaceInformation(manifold, cmanifold));

//     si->setStateValidityChecker(boost::bind(&CnewBoundingBoxPlanner::isStateValid_default, this, _1));
    si->setStateValidityChecker(boost::bind(&CnewBoundingBoxPlanner::isStateValid_default, this, _1));
//     si->setStateValidityCheckingResolution(0.03);
    
    ob::ScopedState<ob::SE2plusStateManifold> start(manifold);
//     start.random(); 
    start->setXYZ(startSE2.x,startSE2.y,0.0);
    start->setYaw(startSE2.theta);
    
    ob::ScopedState<ob::SE2plusStateManifold> goal(manifold);
//     goal.random();
    goal->setXYZ(goalSE2.x,goalSE2.y,0.0);
    goal->setYaw(goalSE2.theta);
    
    aiVector3D vtest(goal->getX(), 0.0, goal->getY());
    aiMatrix4x4 matr2;
    aiMatrix4x4::Translation(vtest, matr2);
    aiMatrix4x4 matr3;
    aiMatrix4x4::RotationY(-goal->getYaw(), matr3);    
    aiVector3D vscaleRIGHT( 0.5, 0.5, 1.3);
    aiMatrix4x4 matrRIGHT;
    aiMatrix4x4::Scaling(vscaleRIGHT, matrRIGHT);
    aiVector3D vscaleLEFT( 0.5, 0.5, 1.3);
    aiMatrix4x4 matrLEFT;
    aiMatrix4x4::Scaling(vscaleLEFT, matrLEFT);

    ai_zoneRightGOAL->mRootNode->mTransformation = (matr2 * matr3) * ai_zoneRightGOAL_init_matrix * matrRIGHT;
    ai_zoneLeftGOAL->mRootNode->mTransformation = (matr2 * matr3) * ai_zoneLeftGOAL_init_matrix * matrLEFT;   

    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

    pdef->setStartAndGoalStates(start, goal, 0.1);
    
//  ob::PlannerPtr planner(new og::RRTConnect(si));
//     ob::PlannerPtr planner(new og::BasicPRMmodif(si));
// 	
//     planner->setProblemDefinition(pdef);
// 
//     planner->setup();
    
    ob::PlannerPtr planner(new oc::KPIECE1(si));
	
    planner->setProblemDefinition(pdef);

    planner->setup();
    
    bool solved = planner->solve(10.0);
    
    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path   
        cout << "Found solution:" << endl;
        
	const ob::PathPtr &path = pdef->getGoal()->getSolutionPath();
	
	og::PathGeometric solution_path = static_cast<og::PathGeometric&>(*path);

	ob::ScopedState<ob::SE2plusStateManifold> state_curr(manifold);
	cout << "length: " << solution_path.length() << endl;
	for(unsigned int i = 0; i < solution_path.states.size(); i++) {	    
	    state_curr = *solution_path.states[i];
	    vector< float > tmpV(4);
	    tmpV[0] = state_curr->getX();
	    tmpV[1] = state_curr->getY();
	    tmpV[2] = state_curr->getZ();
	    tmpV[3] = state_curr->getYaw();
	    discrete_phi_trajectory.push_back(tmpV);
	}
	
// 	og::PathSimplifier psimpl(si);
// 	psimpl.simplifyMax(static_cast<og::PathGeometric&>(*path));

        // print the path to screen
        path->print(cout);
    }
}
// ----------------------------------------------------------------------------
//(assuming that the lowerBBOX configurations are known and stored in lowerBBOXTrajectory until index-1, and the same for whichlowerBBOX, footprints and fprints_vector;
// the c_phi_trajectory must also have been built)
int CnewBoundingBoxPlanner::update_lowerBBOX_config(int index, 
			    vector< vector<float> > & c_phi_trajectory,
			    vector< vector<float> > & lower_bounding_box_trajectory,
			    vector< int > & which_lower_bounding_box,
			    vector<aiMatrix4x4> & footprints,
			    vector< vector<float> > & fprints_vector,
			    PQP_Model & pqp_lower_bounding_box, 
			    aiMatrix4x4 & ai_lower_bounding_box_init_matrix, 
			    const struct aiScene* &ai_env, 
			    PQP_Model &pqp_env) {        
    
    int right0_left1 = 1 - which_lower_bounding_box[index - 1];
    
    float xcyl_prev, ycyl_prev, yawInit;
    
    xcyl_prev = lower_bounding_box_trajectory[index-1][0];
    ycyl_prev = lower_bounding_box_trajectory[index-1][1];
    yawInit = lower_bounding_box_trajectory[index-1][2];
    
    float xxNEXT, yyNEXT, zzNEXT, yawyNEXT, zcoefNEXT;
//     int indexMax = c_phi_trajectory.size()-1;
    int indexMax = aisgl_min((int) c_phi_trajectory.size()-1, index+10000); 
    int indexMin = index;
    int indexNEXT = (indexMax + indexMin)/2;
    int somethingFound;
    
    vector<float> saveRes = lower_bounding_box_trajectory[index-1];
    int saveIndex;
    
    int is_ok = 0;  
//     while(!is_ok) {
	while( indexMax - indexMin > 1 ) {
	    
	    somethingFound = 0;
	    
	    xxNEXT = c_phi_trajectory[indexNEXT][0];
	    yyNEXT = c_phi_trajectory[indexNEXT][1];
	    zzNEXT = c_phi_trajectory[indexNEXT][2];
	    yawyNEXT = c_phi_trajectory[indexNEXT][3];
	    zcoefNEXT = right0_left1 ? (zzNEXT - 1.0)/-2.0 : (zzNEXT + 1.0)/2.0;
	
	    for(unsigned int k = 0; k < 50; k++) {
		
		vector<float> vc = phi_sampler(xxNEXT, yyNEXT, yawyNEXT, zcoefNEXT, right0_left1);	    
		vc[2] = yawyNEXT;
		
		if( phi_verifier(vc[0] - xcyl_prev, vc[1] - ycyl_prev, vc[2] - yawInit, yawInit, 1.0, right0_left1) ) {
	 
		    float res = distanceQuery(pqp_lower_bounding_box, ai_lower_bounding_box_init_matrix, vc[0], vc[1], -vc[2] - PI/2.0, ai_env, pqp_env);
		    
		    if(res > 0.0001) {
			saveRes = vc;
			saveIndex = indexNEXT;
			is_ok = 1;  
			somethingFound = 1;
			break;
		    }
		}
	    }	  
	    
	    if(somethingFound) {
		indexMin = indexNEXT;
		indexNEXT = (indexMax + indexMin)/2;
	    }
	    else {
		indexMax = indexNEXT;
		indexNEXT = (indexMax + indexMin)/2;	    
	    }
	    
	}
//     }
    if(!is_ok) {
	while(!is_ok) {
		vector<float> vc = phi_sampler(c_phi_trajectory[index][0], c_phi_trajectory[index][1], c_phi_trajectory[index][3], 
			right0_left1 ? (c_phi_trajectory[index][2] - 1.0)/-2.0 : (c_phi_trajectory[index][2] + 1.0)/2.0,
			right0_left1);
		vc[2] = c_phi_trajectory[index][3];
		
		float res = distanceQuery(pqp_lower_bounding_box, ai_lower_bounding_box_init_matrix, vc[0], vc[1], -vc[2] - PI/2.0, ai_env, pqp_env);
		
		if(res > 0.0001) {
		    saveRes = vc;
		    saveIndex = index + 1;
		    is_ok = 1;  
		    break;
		}	     
	}
    }
    
    which_lower_bounding_box[index] = 1 - which_lower_bounding_box[index - 1];
    lower_bounding_box_trajectory[index] = saveRes;
        
    //new footprint matrix:
    aiVector3D vtestBIS(lower_bounding_box_trajectory[index][0], 0.0, lower_bounding_box_trajectory[index][1]);
    aiMatrix4x4 matr2BIS;
    aiMatrix4x4::Translation(vtestBIS, matr2BIS);
    aiMatrix4x4 matrFPR;
    aiMatrix4x4::RotationY(-lower_bounding_box_trajectory[index][2] - PI/2.0, matrFPR);
    footprints.push_back( (matr2BIS * matrFPR) * lowerBBOX_init_matrix );
//     saveRes[2] = modul(saveRes[2], 2*PI);
//     cout << saveRes[2] << endl;
    fprints_vector.push_back( saveRes );
    
    return saveIndex;
}
// ----------------------------------------------------------------------------
void CnewBoundingBoxPlanner::from_discrete_to_continuous_phi_trajectory(vector< vector<float> > & d_phi_trajectory, 
						 vector< vector<float> > & c_phi_trajectory)
{    
    c_phi_trajectory.clear();    
    for(unsigned int idex = 0; idex < d_phi_trajectory.size(); idex++) {	
	for(unsigned int timeremainder = 0; timeremainder < 1000; timeremainder++) {	 
	    float x,y,z,yaw;    
	    if(idex == d_phi_trajectory.size()-1) {
		x = d_phi_trajectory[idex][0];
		y = d_phi_trajectory[idex][1];
		z = d_phi_trajectory[idex][2];    
		yaw = d_phi_trajectory[idex][3];
	    }
	    else {
		x = d_phi_trajectory[idex][0] * (1.0 - (timeremainder/1000.0)) + d_phi_trajectory[idex+1][0] * (timeremainder/1000.0);
		y = d_phi_trajectory[idex][1] * (1.0 - (timeremainder/1000.0)) + d_phi_trajectory[idex+1][1] * (timeremainder/1000.0);
		z = d_phi_trajectory[idex][2] * (1.0 - (timeremainder/1000.0)) + d_phi_trajectory[idex+1][2] * (timeremainder/1000.0);
		float theta1 = d_phi_trajectory[idex][3];
		float theta2 = d_phi_trajectory[idex+1][3];
		
		float A = abs(theta2 - theta1);
		float B = abs( ((theta2 - 2*PI) - theta1) ); 
		float C = abs( ((theta2 + 2*PI) - theta1) );
		
		if(B < C && B < A) theta2 = (theta2 - 2*PI);
		else if(C < B && C < A) theta2 = (theta2 + 2*PI);
		
		yaw = theta1 * (1.0 - (timeremainder/1000.0)) + theta2 * (timeremainder/1000.0);
		
	    }	    
	    vector<float> vToAdd(4);	    
	    vToAdd[0] = x;
	    vToAdd[1] = y;
	    vToAdd[2] = z;
	    vToAdd[3] = yaw;	    
	    c_phi_trajectory.push_back(vToAdd);	
	}
    } 
}

//only c_phi_trajectory must have been built!
int CnewBoundingBoxPlanner::build_lowerBBOX_trajectory(
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
			    PQP_Model &pqp_env)
{
    int is_ok = 0;    
    footprints.clear();
    fprints_vector.clear();
    
    lower_bounding_box_trajectory.clear();
    lower_bounding_box_trajectory.resize(c_phi_trajectory.size());
   
    which_lower_bounding_box.clear();
    which_lower_bounding_box.resize(c_phi_trajectory.size());
    which_lower_bounding_box[0] = firstsupportfoot == RIGHT ? 0 : 1;
    
    vector<float> vfpi(3);
    //compute the starting config. for the lower bounding box:
    vfpi[0] = start_pos_and_orient.x + sin(start_pos_and_orient.theta) * -0.095; //TODO: this is hrp2-dependent: WRONG INITIALIZATION (and it should also depend on the first support foot)
    vfpi[1] = start_pos_and_orient.y - cos(start_pos_and_orient.theta) * -0.095; //TODO: this is hrp2-dependent
    vfpi[2] = start_pos_and_orient.theta;
    fprints_vector.push_back(vfpi);
    
    float xx = c_phi_trajectory[0][0];
    float yy = c_phi_trajectory[0][1];
    float zz = c_phi_trajectory[0][2];
    float yawy = c_phi_trajectory[0][3]; 
    float zcoef = which_lower_bounding_box[0] ? (zz - 1.0)/-2.0 : (zz + 1.0)/2.0;
    
    for(unsigned int k = 0; k < 200; k++) {
	vector<float> vc = phi_sampler(xx, yy, yawy, zcoef, 1); //1: left
	
	float res = distanceQuery(pqp_lower_bounding_box, ai_lower_bounding_box_init_matrix, vc[0], vc[1], 0.0, ai_env, pqp_env);
	
	if(res > 0.0001) {
	    is_ok = 1;
	    lower_bounding_box_trajectory[0] = vc;
	    break;
	}
    }
    
    if(!is_ok) return 0;
    
    int nextIndex = 1;
    for(unsigned int k = 1; k < lower_bounding_box_trajectory.size(); k++) {
	if((int) k >= nextIndex) {
	    nextIndex = update_lowerBBOX_config(k, c_phi_trajectory, lower_bounding_box_trajectory, which_lower_bounding_box, footprints, fprints_vector,
		pqp_lower_bounding_box, ai_lower_bounding_box_init_matrix, ai_env, pqp_env);
	} else {
	    which_lower_bounding_box[k] = which_lower_bounding_box[k-1];
	    lower_bounding_box_trajectory[k] = lower_bounding_box_trajectory[k-1];
	}
    }   
    
    return 1;
}
// ----------------------------------------------------------------------------
void CnewBoundingBoxPlanner::build_robot_footsteps (CnewSliderPG * sliderPG,
			    Chrp2Robot & robo, 
			    vector< vector<float> > & fprints_vector,
			    const SE2 & robo_startSE2,
			    const char * pg_config, 
			    const char * pos_file, 
			    const char * zmp_file, 
			    const char * wst_file)
{
    
    vector<float> stepsVect;
    
    stepsVect.push_back(0);
    stepsVect.push_back(0.095); //first stable foot parameter: L       TODO: this is hrp2-dependent: WRONG INITIALIZATION
    stepsVect.push_back(0);
    stepsVect.push_back(0);
    stepsVect.push_back(-0.095);
    stepsVect.push_back(0);
    
    for(unsigned int i = 1; i<fprints_vector.size(); i++) {
	float radV = -fprints_vector[i-1][2];

	//sliding coefficients:	
	if(i==1) stepsVect.push_back(0);
	else stepsVect.push_back(-1.6);
	
	stepsVect.push_back(-1.6);
	
	stepsVect.push_back( cos(radV)*(fprints_vector[i][0] - fprints_vector[i-1][0]) - sin(radV)*(fprints_vector[i][1] - fprints_vector[i-1][1])  );
	    
	stepsVect.push_back( sin(radV)*(fprints_vector[i][0] - fprints_vector[i-1][0]) + cos(radV)*(fprints_vector[i][1] - fprints_vector[i-1][1])  );
		
	float A = (fprints_vector[i][2] - fprints_vector[i-1][2])+2*PI;
	float B = (fprints_vector[i][2] - fprints_vector[i-1][2])-2*PI;
	float C = (fprints_vector[i][2] - fprints_vector[i-1][2]);
	float D;
	if(abs(A) < abs(B) && abs(A) < abs(C)) D = A;
	else if(abs(B) < abs(A) && abs(B) < abs(C)) D = B;
	else D = C;
	stepsVect.push_back( 180.0/PI * D );
	
    }

    filebuf fb;
    fb.open (pg_config,ios::in); 
    istream is(&fb);
    string bufstring;

    float body_height;
    char left_or_right;
    float t_start, t_total;
    float incrTime, gravity;
    float hDistance, maxHeight;
    
    is >> bufstring >> body_height; 
    is >> bufstring >> left_or_right;
    is >> bufstring >> incrTime;
    is >> bufstring >> gravity;
    is >> bufstring >> hDistance;
    is >> bufstring >> maxHeight;
    is >> bufstring >> t_start;
    is >> bufstring >> t_total;

    sliderPG->set_incrTime(incrTime);  
    if(left_or_right == 'L') sliderPG->set_first_support_foot(LEFT);
    else sliderPG->set_first_support_foot(RIGHT);
    
    viaPointConfig vpc;
    vpc.hDistance = hDistance;
    vpc.maxHeight = maxHeight;
    sliderPG->set_vp_config(vpc);

    feetDimensions fdim;
    fdim.width = 0.13; //TODO: still hrp2-dependent
    fdim.length = 0.23;
    sliderPG->set_ft_dim(fdim);
    
    hsConstants hsc;
    hsc.g = gravity;
    hsc.standard_height = body_height;
    hsc.t_start = t_start;
    hsc.t_total = t_total;
    sliderPG->set_constants(hsc);
     
    hrp2Trajectory hrp2t;
    hrp2t.trajFeats.size = 0;
    hrp2t.trajFeats.incrTime = incrTime;    

    sliderPG->produceTraj(hrp2t.trajFeats, stepsVect, robo_startSE2.x, robo_startSE2.y, robo_startSE2.theta);
    
    robo.push_back_Trajectory(hrp2t);
    robo.print_trajectory(0, pos_file, zmp_file, wst_file);
  
}
// ----------------------------------------------------------------------------
void CnewBoundingBoxPlanner::do_motion_phi (int time_elapsed,
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
		    const struct aiScene* ai_phizone_left_GOAL)
{
   
    int time = time_elapsed / 3;
    float stopwatch = 0.001 * 3.0 * (float) time;
    int timeIndex = (int) (stopwatch / 0.005) % (continuous_phi_trajectory.size());
    
    if(d_phi_trajectory.size() > 0) {

	time = timeIndex * 5 / 3;
	if(time >= (int) d_phi_trajectory.size() * 1000) time = d_phi_trajectory.size() * 1000 - 1;
	int timegroup = time / 1000;
	int idex = timegroup % d_phi_trajectory.size();
	
	int timeremainder = time % 1000;
	
	int current_index = idex * 1000 + timeremainder;
	
	float x,y,z,yaw;
	x = c_phi_trajectory[current_index][0];
	y = c_phi_trajectory[current_index][1];
	z = c_phi_trajectory[current_index][2];
	yaw = c_phi_trajectory[current_index][3];

	aiVector3D vtest(x, 0.0, y);
	aiMatrix4x4 matr2;
	aiMatrix4x4::Translation(vtest, matr2);

	aiMatrix4x4 matr3;
	aiMatrix4x4::RotationY(-yaw, matr3);
	
	aiVector3D vscaleRIGHT( (z+1.0)/2.0, (z+1.0)/2.0, 1.0);
	aiMatrix4x4 matrRIGHT;
	aiMatrix4x4::Scaling(vscaleRIGHT, matrRIGHT);
	aiVector3D vscaleLEFT( (z-1.0)/-2.0, (z-1.0)/-2.0, 1.0);
	aiMatrix4x4 matrLEFT;
	aiMatrix4x4::Scaling(vscaleLEFT, matrLEFT);

	ai_phizone_right->mRootNode->mTransformation = (matr2 * matr3) * ai_phizone_right_init_matrix * matrRIGHT;
	ai_phizone_left->mRootNode->mTransformation = (matr2 * matr3) * ai_phizone_left_init_matrix * matrLEFT;	
	
	// update the lowerBBOX transformation  
	aiVector3D vtestBIS(lower_bounding_box_trajectory[current_index][0], 0.0, lower_bounding_box_trajectory[current_index][1]);
	aiMatrix4x4 matr2BIS;
	aiMatrix4x4::Translation(vtestBIS, matr2BIS);
	aiMatrix4x4 matrFPR;
	aiMatrix4x4::RotationY(-lower_bounding_box_trajectory[current_index][2] - PI/2.0, matrFPR);    
	ai_lower_bounding_box->mRootNode->mTransformation = (matr2BIS * matrFPR) * ai_lower_bouding_box_init_matrix;    
    }   
    
}
