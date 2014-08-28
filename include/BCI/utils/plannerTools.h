#ifndef PLANNER_TOOLS_H_
#define PLANNER_TOOLS_H_
#include <vector>
#include "DBase/DBPlanner/db_manager.h"
class EGPlanner;
class Body;
class GraspPlanningState;
class OnLinePlanner;

namespace bci_experiment{

    namespace planner_tools{


        //! Verify that the pregrasp and grasp states are not in collision
        bool testGraspCollisions(Hand * h, const GraspPlanningState * s);


        //! Verify that the pregrasp is not in collision
        bool testPreGraspCollisions(Hand * h, float pregrasp_dist);


        //! Count number of collisions on the hand
        int getNumHandCollisions(Hand * h);


        //! Determine whether collisions are allowed between the hand and all bodies in the world independently
        bool setCollisionState(Hand * h, std::vector<bool> & collisionStatus);

        //! Set the collision status after storing the current collision status between all objects in the world and the hand
        void resetHandCollisions(Hand * h, bool setting,
                                 std::vector<bool> & collisionStatus);

        //! Import grasps from a database to an egPlanner
        void importGraspsFromDBMgr( OnLinePlanner * mPlanner,
                                    db_planner::DatabaseManager * mDbMgr);

        OnLinePlanner * createDefaultPlanner();

    }


}
#endif

