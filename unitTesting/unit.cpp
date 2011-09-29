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
#include <assimpBasedViewer/assimpBasedViewer.h>
#include <sys/time.h>
#include <pthread.h>

CassimpBasedViewer *global_pointer;
CnewBoundingBoxPlanner *global_BboxPlanner;

int64_t ts_now (void) {
  struct timeval tv;
  gettimeofday (&tv,NULL);
  return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

void display(void) {
    if (global_pointer) {
	global_pointer->display();
    }    
}

void reshape(int width, int height) {
    if (global_pointer) {
	global_pointer->reshape(width, height);
    }   
}

void MouseCB(int _b, int _s, int _x, int _y) {
   if (global_pointer) {
	global_pointer->MouseCB(_b,_s,_x,_y);
    }       
}

void MotionCB(int _x, int _y) {
    if (global_pointer) {
	global_pointer->MotionCB(_x,_y);
    }        
}

void loop(void) {
    if (global_pointer) {
	global_pointer->loop();
    }    
}

int load_ai (const struct aiScene* &ai_scene, const char* path, bool display)
{
	// we are taking one of the postprocessing presets to avoid
	// writing 20 single postprocessing flags here.
	ai_scene = aiImportFile(path,aiProcessPreset_TargetRealtime_Quality);
	if(display) global_pointer->addObject(ai_scene, path);
	
	if (ai_scene) {
		return 0;
	}
	return -1;
}
void recursive_build_pqp (int &count_pqp_index, const struct aiScene *sc, const struct aiNode* nd, PQP_Model *mdl)
{
	unsigned int n = 0, t;

	// for all the meshes
	for (; n < nd->mNumMeshes; ++n) {
		const struct aiMesh* mesh = sc->mMeshes[nd->mMeshes[n]];

		for (t = 0; t < mesh->mNumFaces; ++t) {
			const struct aiFace* face = &mesh->mFaces[t];

			if(face->mNumIndices != 3) printf("ERROR: Not a triangle.\n");
			PQP_REAL p1[3],p2[3],p3[3];
			p1[0] = (PQP_REAL) mesh->mVertices[face->mIndices[0]].x;
			p1[1] = (PQP_REAL) mesh->mVertices[face->mIndices[0]].y; 
			p1[2] = (PQP_REAL) mesh->mVertices[face->mIndices[0]].z;
			p2[0] = (PQP_REAL) mesh->mVertices[face->mIndices[1]].x;
			p2[1] = (PQP_REAL) mesh->mVertices[face->mIndices[1]].y;
			p2[2] = (PQP_REAL) mesh->mVertices[face->mIndices[1]].z;
			p3[0] = (PQP_REAL) mesh->mVertices[face->mIndices[2]].x;
			p3[1] = (PQP_REAL) mesh->mVertices[face->mIndices[2]].y; 
			p3[2] = (PQP_REAL) mesh->mVertices[face->mIndices[2]].z;
			mdl->AddTri(p1,p2,p3,count_pqp_index);
			count_pqp_index++;
		}
	}

	// search for the triangles of the children
	for (n = 0; n < nd->mNumChildren; ++n) {
		recursive_build_pqp(count_pqp_index, sc, nd->mChildren[n], mdl);
	}
}
void build_pqp ( const struct aiScene * &sc, PQP_Model & pqp_m)
{
	int count_pqp_index = 0;
	pqp_m.BeginModel();
	recursive_build_pqp(count_pqp_index, sc, sc->mRootNode, &pqp_m);
	pqp_m.EndModel();
}


void *thread_1(void *ptr) {
    
    glutCreateWindow("assimpBasedViewer");    
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutMouseFunc(MouseCB);
    glutMotionFunc(MotionCB);
    
    if (global_pointer) {
	global_pointer->loop();
    }    
    
    return NULL;
}

void *thread_2(void *ptr) {
    
    while(1) {
/*      double tnow = (ts_now() - tstart) * 0.000001;
      int idex = ((int) (tnow / 0.005) ) % global_BboxPlanner->continuous_phi_trajectory.size();
      cout << idex << endl;  */    
    }
    
    return NULL;
}

int main (int argc, char *argv[]) {
    
    CassimpBasedViewer aBViewer;
    global_pointer = &aBViewer;
    
    CnewBoundingBoxPlanner aBBPlanner;
    global_BboxPlanner = &aBBPlanner;
    
    const struct aiScene * ai_leftBbox;
    const struct aiScene * ai_rightBbox;
    const struct aiScene * ai_upperBbox;
    const struct aiScene * ai_env;
    
    if( 0 != load_ai(ai_leftBbox, "/home/perrin/Desktop/blenderStuff/BIGcylinder.3ds", 1)) return -1;
    if( 0 != load_ai(ai_rightBbox, "/home/perrin/Desktop/blenderStuff/BIGcylinder.3ds", 1)) return -1;
    if( 0 != load_ai(ai_upperBbox, "/home/perrin/Desktop/blenderStuff/upperBBOX.3ds", 1)) return -1;
    if( 0 != load_ai(ai_env, "/home/perrin/Desktop/blenderStuff/env.3ds", 1)) return -1;
    
    PQP_Model pqp_leftBbox, pqp_rightBbox, pqp_upperBbox, pqp_env;
    
    build_pqp(ai_leftBbox, pqp_leftBbox); build_pqp(ai_rightBbox, pqp_rightBbox); build_pqp(ai_upperBbox, pqp_upperBbox); build_pqp(ai_env, pqp_env);
    
    aiMatrix4x4 leftBbox_init_Tmatrix, rightBbox_init_Tmatrix, upperBbox_init_Tmatrix, env_init_Tmatrix;  
    model3d leftBbox, rightBbox, upperBbox, env;
    
    leftBbox.pqp = &pqp_leftBbox;
    leftBbox.ai_object = ai_leftBbox;
    leftBbox.init_Tmatrix = &leftBbox_init_Tmatrix;
    
    rightBbox.pqp = &pqp_rightBbox;
    rightBbox.ai_object = ai_rightBbox;
    rightBbox.init_Tmatrix = &rightBbox_init_Tmatrix;
    
    upperBbox.pqp = &pqp_upperBbox;
    upperBbox.ai_object = ai_upperBbox;
    upperBbox.init_Tmatrix = &upperBbox_init_Tmatrix;
    
    env.pqp = &pqp_env;
    env.ai_object = ai_env;
    env.init_Tmatrix = &env_init_Tmatrix;
        
    pthread_t thread1, thread2;
    const char *message1 = "Thread 1";
    const char *message2 = "Thread 2";
    
    pthread_create( &thread1, NULL, thread_1, (void*) message1);
    pthread_create( &thread2, NULL, thread_2, (void*) message2);
    pthread_join( thread1, NULL);
    pthread_join( thread2, NULL); 
    
    return 0;

}