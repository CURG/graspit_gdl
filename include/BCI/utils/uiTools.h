#ifndef UITOOLS_H
#define UITOOLS_H
#include <Inventor/SbColor.h>
#include <QString>
#include <vector>

class World;
class Body;
class Hand;
class transf;
class GraspableBody;
class GraspPlanningState;
class EGPlanner;

namespace bci_experiment
{
    namespace ui_tools
    {

        EGPlanner * getCurrentPlanner();

        //! Make a particular body a particular color
        void highlightBody(Body * b, SbColor color);

        //! Remove any existing highlighting on a body
        void unhighlightBody(Body * b);

        //! Highlight the next graspable body according to graspable body index
        void highlightCurrentGraspableBody(GraspableBody *b);

        //! Unhighlight all graspable bodies
        bool unhighlightAll();

        //! Highlight all graspable bodies
        bool highlightAll();

        //! Focus the camera on the hand
        void viewHand(Hand * h);

        //! Focus the hand on a target object
        void viewTarget(Body * b);

        //! Change the point cloud transparency
        bool setPointcloudTransparency(double transparency);

//! Change the view to focus on all of the bodies given
void viewBodies(std::vector<Body *> & body_vec);


//! Change the view to focus on all of the graspable bodies in the scene
void viewGraspableBodies();

void updateCircularGuides(Hand * hand, Body * object);

void destroyGuideSeparator();

}
}
#endif // UITOOLS_H
