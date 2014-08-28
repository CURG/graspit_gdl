#ifndef WORLD_ELEMENT_TOOLS_H_
#define WORLD_ELEMENT_TOOLS_H_

#include "world.h"
#include "body.h"
#include "ivmgr.h"
#include "robot.h"
#include "graspitGUI.h"
#include "grasp.h"

namespace bci_experiment{
namespace world_element_tools{

World * getWorld();


//! Gets first object that matches the given name.
Body * getObjectByName(const QString & objectName);

//! Brief helper to add an object to the world from relative directory, model type, and model filename.
Body * addToWorld(const QString & relativeModelDir,
                  const QString & modelType,
                  const QString & modelFilename);

//! If the named body doesn't exist, add it to the world
Body * addBodyIfUnique(const QString & bodyName);


//! Get the next body in the graspable body list
GraspableBody * getNextGraspableBody(GraspableBody * b);

//! Find the index of a given body in the world's graspable body vector
int getGraspableBodyIndex(Body * b);



//! Move the given body by some relative transform
void moveBody(Body * b, const transf & relTran);

//!Move all graspable bodies by some relative transform
void moveAllBodies(const transf & relTran);

//! Add an experiment table to world or get a pointer to an existing one
Body * getOrAddExperimentTable();

//!Move this body to the origin of the world and shift all other bodies to
//!maintain their position relative to this body
void setObjectCentral(Body * b);


//! Tell the hand to ignore all bodies that aren't the target.
 void setNontargetCollisions(Hand * h, GraspableBody * target, bool on);


//! Determine if the experiment table can collide with objects
void setTableObjectCollisions(bool setting);

//!Tell the table not to collide with any graspable objects
void disableTableObjectCollisions();



//! Determine whether collisions are allowed between the hand and all bodies in the world independently
bool setCollisionState(Hand * h, std::vector<bool> & collisionStatus);

//! Set the collision status after storing the current collision status between all objects in the world and the hand
void resetHandCollisions(Hand * h, bool setting, std::vector<bool> & collisionStatus);

//! Realigns the hand so that it is facing the object and at a reasonable distance
void realignHand(Hand * h);


void setNonLinkCollisions(Hand * h, bool on);
}

}


#endif
