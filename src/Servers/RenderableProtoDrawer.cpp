#include "Servers/RenderableProtoDrawer.h"
#include <Inventor/lists/SoPathList.h>

#include <Inventor/nodes/SoScale.h>
#include <Inventor/nodes/SoNode.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/nodes/SoPointSet.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/fields/SoSFVec3f.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoCoordinate3.h>

#include "debug.h"


SoNodeList Renderer::getChildByName(SoSeparator * ivRoot, SbName & childName,
                                            SoType targetType, int maxResultsExpected)
{
    assert(ivRoot);
    SoNodeList resultList;

    SoSearchAction sa;
    sa.setSearchingAll(true);

    sa.setType(targetType, true);
    sa.setInterest( SoSearchAction::ALL);
    sa.setName(childName);
    sa.setFind(SoSearchAction::NAME);
    sa.apply(ivRoot);

    SoPathList &pathList = sa.getPaths();
    int numPaths = pathList.getLength();
    if (numPaths > maxResultsExpected)
    {
        DBGA(this->className() << "::getChildByName::Found too many children of node: "
             << ivRoot->getName().getString()  << " with name: "
             <<childName.getString() << " " );
        DBGA(this->className() << "::getChildByName:: Expected:" << maxResultsExpected
             << " Found:" << numPaths);
        resultList.append(static_cast<SoNode *>(NULL));
        return resultList;
    }

    for(int i = 0; i < numPaths; ++i)
    {
        resultList.append(pathList[i]->getTail());
    }
    return resultList;
}



 SoSeparator * Renderer::getOrAddSeparator(SoSeparator * ivRoot, SbName & childName)
{
    SoNodeList children = getChildByName(ivRoot, childName);
    if(children.getLength())
        return static_cast<SoSeparator *>(children[0]);
    else
    {
        SoSeparator * newChild = new SoSeparator();
        newChild->setName(childName);
        ivRoot->addChild(newChild);
        return newChild;
    }
}

SoSeparator *Renderer::getRenderRoot(SbName &childName, Renderable &renderable)
{
    SoSeparator * ivRoot = GraspItGUI::getInstance()->getIVmgr()->getWorld()->getIVRoot();

    if(renderable.has_renderableframe())
    {
        SbName frameName(renderable.renderableframe().c_str());
        ivRoot = getOrAddSeparator(ivRoot, frameName);
    }
    if(ivRoot)
    {
        ivRoot = getOrAddSeparator(ivRoot, childName);
    }
    return ivRoot;
}

bool Renderer::render(Renderable &renderable, QString & renderableRootName)
{
    SbName renderableSeparatorName(renderableRootName.toStdString().c_str());
    SoSeparator * ivRoot = getRenderRoot(renderableSeparatorName, renderable);
    if(ivRoot)
        return renderImpl(ivRoot, renderable);
    else
        return false;

}




bool PointCloudRenderer::createNodes(SoSeparator *ivRoot, Renderable &renderable)
{

    SoTransform * coord_tran = new SoTransform();
    coord_tran->setName("PointCloudTransform");
    SoCoordinate3 * coord = new SoCoordinate3();
    coord->setName("PointCloudCoordinates");
    SoPointSet * pointSet = new SoPointSet();
    SoDrawStyle * drawStyle = new SoDrawStyle();

    SoMaterial * mat = new SoMaterial();
    mat->setName("PointCloudColorMaterial");
    SoMaterialBinding * matBinding = new SoMaterialBinding();
    matBinding->value = SoMaterialBinding::PER_PART;
    ivRoot->addChild(coord_tran);
    setScale(ivRoot, renderable);
    ivRoot->addChild(coord);
    ivRoot->addChild(mat);
    ivRoot->addChild(matBinding);

    drawStyle->pointSize = 3;
    ivRoot->addChild(drawStyle);

    ivRoot->addChild(pointSet);
    return true;
}

void PointCloudRenderer::setScale(SoSeparator *ivRoot, Renderable &renderable)
{
    float factor = 1.0;

    if(renderable.pointcloud().has_units())
    {
        factor = 1000.0/renderable.pointcloud().units();
    }


    if (ivRoot->getNumChildren() < 2 || ivRoot->getChild(1)->getTypeId() != SoScale::getClassTypeId())
    {
        ivRoot->insertChild(new SoScale(),1);
    }

    SoScale * cloudScale = static_cast<SoScale *>(ivRoot->getChild(1));
    cloudScale->setName("PointCloudScale");
    cloudScale->scaleFactor.setValue(factor, factor, factor);
}

bool PointCloudRenderer::renderImpl(SoSeparator *ivRoot, Renderable &renderable)
{
    //Being here without a point cloud is a bug.
    assert(renderable.has_pointcloud());

    //Unpack the pointcloud just for future code berevity
    const PointCloudXYZRGB & pc(renderable.pointcloud());

    //Create the node subtree of the renderable if necessary.
    if(!ivRoot->getNumChildren())
    {
        createNodes(ivRoot, renderable);
    }
    setScale(ivRoot, renderable);
    //Get the nodes to insert the point data into.
    SoMaterial * mat = static_cast<SoMaterial *>(ivRoot->getChild(3));
    SoCoordinate3 * coord =
            static_cast<SoCoordinate3 *>(ivRoot->getChild(2));

    //Read the point data into Inventor structures.
    std::vector<SbVec3f> points;
    std::vector<SbColor> colors;
    int pointNum = pc.points_size();
    points.reserve(pointNum);
    colors.reserve(pointNum);

    for(int i = 0; i < pointNum; ++i)
    {
        //Unpack the data from the nested data structures
        const PointXYZRGB & pc_colored_point(pc.points(i));
        const PointXYZ & pc_point(pc_colored_point.point());
        const ColorRGB & pc_color(pc_colored_point.color());

        //Put the data in the data structures
        points.push_back(SbVec3f(pc_point.x(), pc_point.y(), pc_point.z()));
        colors.push_back(SbColor(pc_color.red(), pc_color.green(), pc_color.blue()));
    }

    //Insert the inventor structures in to the
    coord->point.setValues(0,points.size(), &points[0]);
    mat->diffuseColor.setValues(0,colors.size(), &colors[0]);
    return true;
}


void RenderableProtoDrawer::renderMessage(Renderable &renderable)
{
    if(renderable.has_pointcloud())
    {
        PointCloudRenderer renderer;
        QString ivRootName("PointCloudRoot");
        renderer.render(renderable,ivRootName);
    }
}
