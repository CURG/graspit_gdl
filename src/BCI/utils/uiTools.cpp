#include "BCI/utils/uiTools.h"
#include "world.h"
#include "body.h"
#include "ivmgr.h"
#include "graspitGUI.h"
#include <Inventor/nodes/SoMaterial.h>
#include "robot.h"
#include <Inventor/nodes/SoCamera.h>
#include <searchState.h>
#include "BCI/utils/worldElementTools.h"
#include "SoRing.h"

using bci_experiment::world_element_tools::getWorld;
using bci_experiment::world_element_tools::getObjectByName;
using bci_experiment::world_element_tools::getNextGraspableBody;

namespace bci_experiment
{
    namespace ui_tools
    {

        void highlightBody(Body * b, SbColor color)
        {
          b->getIVMat()->emissiveColor.setIgnored(false);
          b->getIVMat()->emissiveColor.setValue(color);
          b->getIVMat()->transparency.setIgnored(true);
        }



        void unhighlightBody(Body * b)
        {
          b->getIVMat()->emissiveColor.setIgnored(true);
          b->getIVMat()->transparency.setIgnored(false);
        }


        void highlightCurrentGraspableBody(GraspableBody *b)
        {
            highlightAll();

            if(b)
            {
                highlightBody(b, SbColor(0,1,0));
            }
        }


        bool unhighlightAll()
        {
          for(int i = 0; i < getWorld()->getNumGB(); ++i)
          {
            unhighlightBody(getWorld()->getGB(i));
          }
          return true;
        }


        bool highlightAll()
        {
          for(int i = 0; i < getWorld()->getNumGB(); ++i)
          {
            highlightBody(getWorld()->getGB(i), SbColor(1,0,0));
          }
          return true;
        }


        void viewHand(Hand * h)
        {
          graspItGUI->getIVmgr()->getViewer()->getCamera()->viewAll(h->getIVRoot(), graspItGUI->getIVmgr()->getViewer()->getViewportRegion(), 0.5);
        }


        void viewTarget(Body * b)
        {
          graspItGUI->getIVmgr()->getViewer()->getCamera()->viewAll(b->getIVRoot(), graspItGUI->getIVmgr()->getViewer()->getViewportRegion(), 1.0);
          Body * tableBody= getObjectByName("experiment_table");
          SbVec3f table_z = tableBody->getTran().affine().transpose().row(2).toSbVec3f();

          graspItGUI->getIVmgr()->getViewer()->getCamera()->pointAt(SbVec3f(0,0,0), table_z);
        }


        bool setPointcloudTransparency(double transparency)
        {
          SoMaterial * mat = static_cast<SoMaterial *>(SoMaterial::getByName("PointCloudColorMaterial"));
          if(!mat)
            return false;
          mat->transparency = transparency;
          return true;
        }

        // Create a new node set representing an axis grounded at an object
        SoSeparator * createNewGuideAxis(std::string axisName, const vec3 & ringEuler)
        {
            SoSeparator * axisSep = new SoSeparator(3);
            SoMaterial * axisMat = new SoMaterial();
            axisMat->transparency = 0.0;
            axisMat->ambientColor.setValue(1.0,0.0,0.0);
            SoTransform * axisTran = new SoTransform();
            SoRing * axisGeom = new SoRing();
            axisGeom->sweepAngle = 360.0;
            SbString baseName(axisName.c_str());
            axisSep->setName(baseName + "Sep");
            axisMat->setName(baseName + "Mat");
            axisTran->setName(baseName + "Tran");
            axisGeom->setName(baseName + "Geom");
            axisSep->addChild(axisMat);
            axisSep->addChild(axisTran);
            axisSep->addChild(axisGeom);
            rotXYZ(ringEuler[0], ringEuler[1], ringEuler[2]).toSoTransform(axisTran);
            return axisSep;
        }

        // Update the rings
        void setGuideRingRadius(SoSeparator * ringSep,
                                float radius, float thickness)
        {
            static_cast<SoRing *>(ringSep->getChild(2))->outerRadius = radius;
            static_cast<SoRing *>(ringSep->getChild(2))->innerRadius = radius - thickness;
        }

        void setRingTransform(SoSeparator * ringSep,
                              transf & handObjectTran,
                              int axisSelector)
        {
            vec3 position = vec3::ZERO;

            position[axisSelector] = handObjectTran.translation()[axisSelector];

            static_cast<SoTransform *>(ringSep->getChild(1))->translation.setValue(position.toSbVec3f());
        }


        SoSeparator * createCircleGuides(SoSeparator * sceneRoot)
        {

            SoSeparator * guideSeparator = new SoSeparator(3);
            guideSeparator->setName("BCIGuideSeparator");
            //Create new root transform for the guide
            SoTransform * objectTransform = new SoTransform();
            objectTransform->setName("BCIGuideTransform");
            //Create new guide axes
            SoSeparator * xAxisSep =  createNewGuideAxis("XAxisGuide", vec3(0,M_PI/2,0));
            SoSeparator * zAxisSep = createNewGuideAxis("ZAxisGuide", vec3(0,0,0));

            //Add children to root
            guideSeparator->addChild(objectTransform);
            guideSeparator->addChild(xAxisSep);
            guideSeparator->addChild(zAxisSep);

            //Add root to scene root
            sceneRoot->addChild(guideSeparator);
            return guideSeparator;

        }


        void updateCircularGuides(Hand *hand, Body *object)
        {
            // In this case, we really do want the ivmgr owned by the gui,
            // So it is ok to get it this way



            SoSeparator * pointerRoot =
                    GraspItGUI::getInstance()->getIVmgr()->getWorld()->getIVRoot();
            SoSeparator * guideSeparator = static_cast<SoSeparator *>
                    (pointerRoot->getByName("BCIGuideSeparator"));
            SoSeparator * xAxisSep = NULL;
            SoSeparator * zAxisSep = NULL;
            SoTransform * objectTransform = NULL;

            float ringThickness = 10.0f;

            //If there is no guide separator, create it
            if(!guideSeparator)
            {
                guideSeparator = createCircleGuides(pointerRoot);
            }

            objectTransform = static_cast<SoTransform *>(guideSeparator->getChild(0));
            xAxisSep = static_cast<SoSeparator *>(guideSeparator->getChild(1));
            zAxisSep = static_cast<SoSeparator *>(guideSeparator->getChild(2));

            //Update object transform
            object->getTran().toSoTransform(objectTransform);


            //Get relative hand to object transform
            transf handToObjectTran = hand->getTran() * object->getTran().inverse();

            //update guide radii
            vec3 HandToObjectPos = handToObjectTran.translation();
            float xAxisRadius = sqrt(pow(HandToObjectPos.y(),2) + pow(HandToObjectPos.z(),2));
            float zAxisRadius = sqrt(pow(HandToObjectPos.x(),2) + pow(HandToObjectPos.y(),2));

            //Set ring size
            setGuideRingRadius(xAxisSep, xAxisRadius, ringThickness);
            setGuideRingRadius(zAxisSep, zAxisRadius, ringThickness);

            //Set Z ring offset
            setRingTransform(xAxisSep, handToObjectTran, 0);
            setRingTransform(zAxisSep, handToObjectTran, 2);
        }

        void destroyGuideSeparator()
        {
            SoSeparator * pointerRoot =
                    GraspItGUI::getInstance()->getIVmgr()->getPointers();
            SoSeparator * guideSeparator = static_cast<SoSeparator *>
                    (pointerRoot->getByName("BCIGuideSeparator"));
            if(guideSeparator)
                pointerRoot->removeChild(guideSeparator);
        }
    }
}
