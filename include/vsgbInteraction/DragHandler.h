/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * vsgBullet is (C) Copyright 2025 by Julien Valentin
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License version 2.1 as published by the Free Software Foundation.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 *************** <auto-copyright.pl END do not edit this line> ***************/

#ifndef __VSGBINTERACTION_DRAG_HANDLER_H__
#define __VSGBINTERACTION_DRAG_HANDLER_H__ 1


#include <vsgbInteraction/Export.h>
#include <vsg/app/Trackball.h>
#include <vsgbDynamics/MotionState.h>
#include <vsgbDynamics/World.h>
#include <btBulletDynamicsCommon.h>


#include <vsg/utils/LineSegmentIntersector.h>
// Forward
namespace vsg {
    class Camera;
}

namespace vsgbDynamics {
    class PhysicsThread;
}


namespace vsgbInteraction
{


/** \class DragHandler DragHandler.h <vsgbInteraction\DragHandler.h>
\brief An event handler for selecting and dragging rigid bodies.

To use this event handler, simply create an instance of it and add it to your
vsg::Viewer.

During a ctrl-leftmouse click, DragHandler does an intersection test with
the \c scene. The test succeeds if the picked object has a Node in its NodePath
containing a RefRigidBody stored in the Node's UserData. DragHandler then adds
the picked btRigidBody to a new btPoint2PointConstraint and adds it to the
dynamics world. DragHandler also computes a drag plane, orthogonal to the view
direction and containing the intersection point.

On subsequent ctrl-leftmouse drag events, DragHandler back-transform the mouse
position to create a world space ray, and intersects it with the DragPlane.
DragHandler then sets this intersection point in the constraint.

On a leftmouse release event, DragHandler removes the constraint and deletes it.
*/
class VSGBINTERACTION_EXPORT DragHandler : public vsg::Inherit<vsg::Trackball, DragHandler>
{
public:
    /** \brief Constructor.
    \param dw The Bullet dynamics world. When the DragHandler creates a
    btPoint2PointConstraint, it adds it to this dynamics world.
    \param scene Scene graph used for picking. \c scene must be a Camera node
    to allow DragHandler to properly convert from window to world coordinates
    during selection and dragging. */
    DragHandler(vsg::ref_ptr<vsgbDynamics::World> w,vsg::ref_ptr<vsg::Camera> camera, vsg::ref_ptr<vsg::EllipsoidModel> ellipsoidModel = {});

    /** \brief Handle events.

    Controls:
    \li ctrl-left-mouse Select and drag a rigid body.
    */

    void apply(vsg::ButtonPressEvent& buttonPress) override;
    void apply(vsg::ButtonReleaseEvent& buttonRelease) override;
    void apply(vsg::MoveEvent& moveEvent) override;

    void apply(vsg::KeyPressEvent& keyPress) override;
    void apply(vsg::KeyReleaseEvent& keyRelease) override;
    /** \brief Support for running the Bullet physics simultation in a separate thread.

    Call this function to specify the vsgbDynamics::PhysicsThread. DragHandler pauses
    and unpauses the thread during constraint creation, modification, and deletion. */
    void setThreadedPhysicsSupport( vsgbDynamics::PhysicsThread* pt );


    void setWorld(vsgbDynamics::World * w){_dw=w;}
    vsgbDynamics::World * getWorld()const{return _dw;}
protected:
    ~DragHandler();

    /** \brief Picking support.
    \param wx Normalized (-1.0 to 1.0) x mouse position
    \param wy Normalized (-1.0 to 1.0) Y mouse position
    */
    bool pick( vsg::Node*,float wx, float wy );
    bool pick(vsg::PointerEvent& pointerEvent);
    bool _ctrlpressed=false;
    vsg::ref_ptr<vsg::PointerEvent> lastPointerEvent;
    vsg::ref_ptr<vsg::LineSegmentIntersector::Intersection>        lastIntersection;
    vsgbDynamics::World * _dw;
    //vsg::ref_ptr< vsg::Camera > _scene;

    btPoint2PointConstraint* _constraint;
    const vsgbDynamics::MotionState* _constrainedMotionState;
    vsg::vec4 _dragPlane;

    vsgbDynamics::PhysicsThread* _pt;
};


// vsgbInteraction
}


// __VSGBINTERACTION_DRAG_HANDLER_H__
#endif
