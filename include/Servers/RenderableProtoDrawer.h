#ifndef RENDERABLEPROTODRAWER_H
#define RENDERABLEPROTODRAWER_H
#include "Renderable.pb.h"
#include "ivmgr.h"
#include "graspitGUI.h"
#include "world.h"
#include <Inventor/nodes/SoSeparator.h>



class Renderer : public QObject
{
protected:

    /*!
     * Helper function to get a list of children with a particular name
     * @param[in] ivRoot              The root to start the search from
     * @param[in] childName           The name of children to return
     * @param[in] targetType          The type of node to search for. Defaults to SoSeparator
     * @param[in] maxResultsExpected  The maximum number of hits allowed. Violating this results in logging
     *                                output and returns a list containing a single null pointer.
    */
    virtual SoNodeList getChildByName(SoSeparator * ivRoot, SbName & childName,
                                      SoType targetType = SoSeparator::getClassTypeId(), int maxResultsExpected = 1);

    /*!
     * Find a separator node below a given subtree or insert it as the last child of
     * of the root node if it doesn't exist. If more than one exists, this fails and returns null.
     *
     * @param[in] ivRoot     The root to start the search from
     * @param[in] childName  The name of the child to insert or return
     *
     */
    virtual SoSeparator * getOrAddSeparator(SoSeparator * ivRoot, SbName & childName);

    /*!
     * @brief getRenderRoot Find or insert the root separator to use to build this renderable on.
     *
     * @param[in] childName   The name of the renderables root geometry
     * @param[in] renderable  The renderable proto to render
     * @return the root separator to use to build this renderable on or
     *         NULL if none can be created.
     */
    virtual SoSeparator * getRenderRoot(SbName &childName, Renderable & renderable);

    /*!
     * @brief renderImpl The function to override to actually render the renderable
     * @param ivRoot     The root to start rendering from
     * @param renderable The proto message containing the renderable.
     * @return True if the rendering succeeds.
     */
    virtual bool renderImpl(SoSeparator * ivRoot, Renderable & renderable) = 0;
public:
    /*!
     * @brief render      Builds an ivroot that draws a given proto
     * @param renderable  The proto message containing the renderable
     * @return True if the rendering succeeds.
     */
    virtual bool render(Renderable & renderable, QString & renderableRootName);
};

class PointCloudRenderer : public Renderer
{
    typedef Renderable_PointCloudXYZRGB PointCloudXYZRGB;
    typedef Renderable_PointXYZRGB PointXYZRGB;
    typedef Renderable_PointXYZ PointXYZ;
    typedef Renderable_ColorRGB ColorRGB;

    /*!
     * @brief createNodes   Create the nodes that underlye the structure
     *                      of the renderable
     * @param ivRoot        The root to build the structure on
     * @param renderable    The proto message to build.
     * @return True if creating the nodes succeeds.
     */
    virtual bool createNodes(SoSeparator * ivRoot, Renderable & renderable);

    /*!
     * @brief setScale      Set the scale of the point clouds to draw
     * @param ivRoot        The root to build the structure on
     * @param renderable    The proto message to build.
     */
    virtual void setScale(SoSeparator * ivRoot, Renderable & renderable);

    /*!
     * @brief  renderImpl The function to override to actually render the renderable
     * @param  ivRoot     The root to start rendering from
     * @param  renderable The proto message containing the renderable.
     * @return True if the rendering succeeds.
     */
    virtual bool renderImpl(SoSeparator *ivRoot, Renderable &renderable);

};

class RenderableProtoDrawer : public QObject
{
    /*!
     * \brief renderMessage  Render a proto message containing multiple geometry types
     * \param renderable     The proto message containing the renderable.
     */
    public:
        void renderMessage(Renderable & renderable);
};

#endif // RENDERABLEPROTODRAWER_H
