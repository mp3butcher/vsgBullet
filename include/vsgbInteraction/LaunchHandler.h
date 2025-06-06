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

#ifndef __VSGBINTERACTION_LAUNCH_HANDLER_H__
#define __VSGBINTERACTION_LAUNCH_HANDLER_H__ 1


#include <vsgbInteraction/Export.h>
#include <vsgbDynamics/MotionState.h>
#include <vsgbDynamics/World.h>
#include <vsg/app/Trackball.h>
#include <btBulletDynamicsCommon.h>
#include <list>

// Forward
namespace vsg {
class Node;
class Group;
class Camera;
}

namespace vsgbDynamics {
class PhysicsThread;
class TripleBuffer;
}


namespace vsgbInteraction
{


/** \class LaunchHandler LaunchHandler.h <vsgbInteraction\LaunchHandler.h>
\brief An event handler to support throwing objects into a dynamics simulation.

This class allows users to launch btRigidBody objects into the scene using
shift-leftmouse.

By default, the launched object is a sphere with radius 1.0 at a velocity of 10.0
units/sec. However, the application can specify an arbitrary model to launch, and
also change the initial velocity.

LaunchHandler uses multiparenting so that only one incarnation of the launch model
is present in the scene graph, regardless of how many objects the user has launched.
LaunchHandler also shares the same btCollisionShape between all launched objects.

LaunchHandler is compatible with SaveRestoreHandler, so that hitting the Delete key
to trigger a SaveRestoreHandler::reset() will remove all launched rigid bodies. */
class VSGBINTERACTION_EXPORT LaunchHandler : public vsg::Inherit<vsg::Trackball,LaunchHandler>
{
public:
    /** \brief Constructor.
    \param dw LaunchHandler adds launched rigid bodies to this dynamics world.
    Note that if you intend to set collision filters, \c dw must be a
    btDiscreteDynamicsWorld. See setCollisionFilters().
    \param attachPoint LaunchHandler adds instances of the launch model to this node
    in the scene graph.
    \param camera LaunchHandler uses this Camera to compute the launch direction
    vector from the mouse position.
    */

    LaunchHandler(vsg::ref_ptr<vsgbDynamics::World> w, vsg::ref_ptr<vsg::Group> attachPoint, vsg::observer_ptr<vsg::Viewer> viewer, vsg::ref_ptr<vsg::Camera> camera,vsg::ref_ptr<vsg::EllipsoidModel> ellipsoidModel = {});;


    void apply(vsg::ButtonPressEvent& buttonPress) override;
    void apply(vsg::ButtonReleaseEvent& buttonRelease) override;
    void apply(vsg::MoveEvent& moveEvent) override;


    void apply(vsg::KeyPressEvent& keyPress) override;
    void apply(vsg::KeyReleaseEvent& keyRelease) override;
    /** \brief Handle events.

    Controls:
    \li shift-leftmouse Computes a launch vector from the mouse click position,
    creates a new btRigidBody for the launch model, adds an instance of the
    launch model to the scene attach point, specifies the initial velocity of
    the btRigidBody, and adds the rigid body to the dynamics world.
    */
    //bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& );

    /** \brief Support for running the Bullet physics simultation in a separate thread.

    Call this function to specify vsgBullet threaded physics objects.
    \param pt LaunchHandler pauses and unpauses this thread before adding or removing rigid
    bodies from the dynamics world.
    \param tb The launched btRigidBody's MotionState registers this TripleBuffer for thread-safe
    transform modification in the scene graph.
    \param msl The launched btRigidBody's MotionState is added to this list of MotionState objects
    for processung during scene graph update.
    */
    void setThreadedPhysicsSupport( vsgbDynamics::PhysicsThread* pt, vsgbDynamics::TripleBuffer* tb, vsgbDynamics::MotionStateList* msl );

    /** \brief Specify a non-default launch model.

    The default launch model is a sphere with radius 1. Use this routine to specify a non-default
    launch model.
    \param shape Specify a collision shape for the model. If \c shape is nullptr (the default),
    this function creates a convex hull collision shape from \c model, using its bounding
    sphere for center of mass, and default scaling of 1.0 in all axes.

    Future work: Calling this function with a nullptr \c model should cause the default launch
    model to be recreated and used for subsequent launches. */
    void setLaunchModel( vsg::Node* model, btCollisionShape* shape=nullptr );

    /** \brief Access the initial launch velocity.

    Units are in physics simultation units (e.g., m/sec^2). Default is 10.0. */
    void setInitialVelocity( double velocity ) { _initialVelocity = velocity; }
    double getInitialVelocity() const { return( _initialVelocity ); }

    void setLaunchedMass( double m ) { _launchedMass = m; }
    double getLaunchedMass() const { return( _launchedMass ); }



    /** \brief Specify collision filters for Bullet's addRigidBody() call.

    By default, there are no collision filters, and LaunchHandler uses the simple form
    of addRigidBody(btRigidBody*) when adding a launched object. Call this function
    to specify collision filters, which will cause LaunchHandler to use the extended
    form of addRigidBody(btRigidBody*,short,short). Note that when using collision
    filters, the constructor's dynamics world parameter \c dw must be an instance of a
    btDiscreteDynamicsWorld. */
    void setCollisionFilters( short group, short mask ) { _group = group; _mask = mask; }

    /** \brief Specify the camera used to compute the launch direction vector, if not
    specified in the constructor. */
    // void setCamera( vsg::Camera* camera ) { _camera = camera; }

    /** \brief Remove all launched models from the scene graph and physics simultation.
    */
    void reset();

    void setWorld(vsgbDynamics::World*w){_world=w;}
    vsgbDynamics::World* getWorld()const {return _world;}
    void setAttachPoint( vsg::Group *w){_attachPoint=w;}
    vsg::Group * getAttachPoint()const {return _attachPoint;}
protected:
    ~LaunchHandler();
    bool _ctrlpressed=false;
    vsg::observer_ptr<vsg::Viewer> _refviewer;
    vsg::ref_ptr<vsgbDynamics:: World> _world;
    //btDynamicsWorld* _dw;
    vsg::ref_ptr< vsg::Group > _attachPoint;
    //vsg::ref_ptr< vsg::Camera > _camera;

    vsg::ref_ptr< vsg::Node > _launchModel;
    btCollisionShape* _launchCollisionShape;
    bool _ownsCollisionShape;
    double _initialVelocity;
    double _launchedMass;
    short _group, _mask;

    typedef std::list< vsg::ref_ptr< vsg::Node > > NodeList;
    NodeList _nodeList;

    vsgbDynamics::PhysicsThread* _pt;
    vsgbDynamics::TripleBuffer* _tb;
    vsgbDynamics::MotionStateList* _msl;
};


// vsgbInteraction
}


// __VSGBINTERACTION_LAUNCH_HANDLER_H__
#endif
