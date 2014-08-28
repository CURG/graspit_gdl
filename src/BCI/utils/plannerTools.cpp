#include "world.h"
#include "robot.h"
#include "searchState.h"
#include "BCI/utils/worldElementTools.h"
#include "BCI/utils/plannerTools.h"
#include "DBPlanner/db_manager.h"
#include "EGPlanner/egPlanner.h"
#include "EGPlanner/onLinePlanner.h"
#include "debug.h"
#include "DBase/graspit_db_grasp.h"


using bci_experiment::world_element_tools::getWorld;
using bci_experiment::world_element_tools::getOrAddExperimentTable;

namespace bci_experiment
{
    namespace planner_tools
    {


        bool testPreGraspCollisions(Hand * h, float pregrasp_dist)
        {
          h->autoGrasp(false, -2.0, true);
          h->approachToContact(pregrasp_dist, false);
          return (getNumHandCollisions(h));
        }


        bool testGraspCollisions(Hand * h, const GraspPlanningState * s)
        {
          bool result = false;
          std::vector<bool> currentCollisionState;
          resetHandCollisions(h, true, currentCollisionState);
          s->execute(h);
          World * w = getWorld();
          w->toggleCollisions(false, h, s->getObject());
          if(getNumHandCollisions(h))
            result = true;
          if(testPreGraspCollisions(h, -50.0))
            result = true;

          setCollisionState(h, currentCollisionState);
          return result;
        }


        int getNumHandCollisions(Hand * h)
        {
          CollisionReport colReport;
          std::vector<Body *> body_list;
          h->getBodyList(&body_list);
          getWorld()->getCollisionReport(&colReport, &body_list);
          return colReport.size();
        }

        bool setCollisionState(Hand * h, std::vector<bool> & collisionStatus)
        {
          World * w = getWorld();
          if(collisionStatus.size() != w->getNumGB() + 1)
            return false;

          Body * experiment_table = getOrAddExperimentTable();
          w->toggleCollisions(collisionStatus[0], h, experiment_table);
          for (int i = 0; i < w->getNumGB(); ++i)
          {
            w->toggleCollisions(collisionStatus[i+1], h, w->getGB(i));
          }
          return true;
        }


        void resetHandCollisions(Hand * h, bool setting,
                                 std::vector<bool> & collisionStatus)
        {
          World * w = getWorld();

          Body * experiment_table = getOrAddExperimentTable();
          collisionStatus.push_back(!w->collisionsAreOff(h, experiment_table));
          w->toggleCollisions(setting, h, experiment_table);

          for (int i = 0; i < w->getNumGB(); ++i)
          {
            collisionStatus.push_back(!w->collisionsAreOff(h, w->getGB(i)));
            w->toggleCollisions(setting, h, w->getGB(i));
          }
        }





        void importGraspsFromDBMgr( OnLinePlanner * mPlanner, db_planner::DatabaseManager * mDbMgr)
        {
            Hand*mHand = mPlanner->getRefHand();

            // Get corresponding model from database
            std::vector<db_planner::Model*> modelList;
            QString *objectFilename = new QString('/' + mHand->getGrasp()->getObject()->getFilename().split('/').back());

            mDbMgr->ModelList(&modelList,db_planner::FilterList::USE_FILE_NAME, *objectFilename);


            if(modelList.empty())
            {
              DBGA("No Models Found \n");
              return;
            }

            // Using the found model, retrieve the grasps
            std::vector<db_planner::Grasp*> grasps;
            mDbMgr->GetGrasps(*modelList.back(), GraspitDBGrasp::getHandDBName(mHand).toStdString(), &grasps);

            mHand->saveState();

            // Load the grasps into the grasp planner list.
            unsigned int numGrasps = std::min<unsigned int>(grasps.size(), 10);
            for (unsigned int gNum = 0; gNum < numGrasps; ++gNum)
            {
                GraspPlanningState *s = new GraspPlanningState(static_cast<GraspitDBGrasp *>
                                       (grasps[gNum])->getFinalGraspPlanningState());

                s->setObject(mHand->getGrasp()->getObject());
                s->setRefTran(mHand->getGrasp()->getObject()->getTran());
                float testResult = -2*bci_experiment::planner_tools::testGraspCollisions(mHand, s);
                s->addAttribute("graspId", gNum);
                s->addAttribute("testResult", testResult);
                s->addAttribute("testTime", 0);
                mPlanner->addSolution(s);
            }

            //reorders the solutions we have found.
            mPlanner->updateSolutionList();

            //needed to return hand to aligned with object, since it was used to testGraspCollisions
            mHand->restoreState();

        }

        OnLinePlanner * createDefaultPlanner(){

             World * w = getWorld();
             if(!w->getCurrentHand())
             {
                 DBGA("plannerTools::createDefaultPlanner::No current Hand!");
                 return NULL;
             }

             GraspPlanningState *mHandObjectState = new GraspPlanningState(w->getCurrentHand());
             mHandObjectState->setPositionType(SPACE_AXIS_ANGLE);
             mHandObjectState->setObject(w->getGB(0));
             mHandObjectState->setRefTran(w->getGB(0)->getTran());
             mHandObjectState->reset();

             OnLinePlanner * op = new OnLinePlanner(w->getCurrentHand());
             op->setContactType(CONTACT_PRESET);
             op->setEnergyType(ENERGY_CONTACT_QUALITY);
             op->setMaxSteps(2000);
             op->setModelState(mHandObjectState);

             w->setCurrentPlanner(op);
             op->showSolutionClone(true);
             op->resetPlanner();

            return op;

        }

    }

}
