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

#ifndef __VSGBDYNAMICS_MOTIONSTATE_H__
#define __VSGBDYNAMICS_MOTIONSTATE_H__
#include <vector>
#include <set>

#include <vsg/nodes/MatrixTransform.h>

#include <vsg/nodes/AbsoluteTransform.h>
#include <btBulletCollisionCommon.h>
#include <vsgbDynamics/Export.h>




namespace vsgbDynamics
{



/** \class MotionStateCallback MotionState.h <vsgbDynamics/MotionState.h>
\brief Application notification of changes to a MotionState world transform.

Derive a struct from MotionStateCallback and add it to the MotionState class.
The operator() method will get called whenever Buller sets the world transform.
*/
struct VSGBDYNAMICS_EXPORT MotionStateCallback
{
    MotionStateCallback() {}
    virtual ~MotionStateCallback() {}

    virtual void operator()( const btTransform& worldTrans ) = 0;
};
typedef std::vector< MotionStateCallback* > MotionStateCallbackList;


// forward declaration
class TripleBuffer;

/** \class MotionState MotionState.h <vsgbDynamics/MotionState.h>
\brief A btMotionState that works with VSG Transforms.

A btMotionState that allows Bullet to set the ransformation
of an VSG subgraph corresponding to a rigid body.

This class can interface with an vsg::MatrixTransform
or an vsg::AbsoluteTransform.

In typical usage, your application uses the \ref rigidbody routines to create
a rigid body, which implicitly creates a MotionState. However, if your application
doesn't use the \ref rigidbody routines, you will need to explicitly create
a MotionState in order to keep your visual and physical representations in sync.

To attache a MotionState to your VSG subgraph:
\li Call setTransform() and pass in the root node of your subgraph.
   The node must be an vsg::MatrixTransform or an vsg::AbsoluteTransform.
\li Call setParentTransform() to specify the initial transformation
   for the subgraph (usually the VSG local to world matrix from the
   subgraph parent's NodePath).
\li Call setCenterOfMass() to specify the xyz point corresponding
   to the origin of the Bullet collision shape used by the rigid body.
*/
class VSGBDYNAMICS_EXPORT MotionState : public btMotionState
{
public:
    /** \brief Constructor.

    \param parentTransform See setParentTransform().
    \param centerOfMass See setCenterOfMass().
    */
    MotionState( const vsg::mat4& parentTransform = vsg::mat4(),
        const vsg::vec3& centerOfMass = vsg::vec3( 0., 0., 0. ) );
    /** \brief Destructor. */
    virtual ~MotionState( void ) { }


    /** \brief Bullet interface routine for changing the rigid body (and corresponding
    VSG visual representation) transformation.

    Bullet sets and gets the rigid body world transformation using
    these routines (setWorldTransform() and getWorldTransform() ). They are
    promarily for use by Bullet only, but setWorldTransform() is also called
    internally by resetTransform. */
    virtual void setWorldTransform(const btTransform& worldTrans);
    /** \copybrief setWorldTransform */
    virtual void getWorldTransform(btTransform& worldTrans ) const;

    /** \brief Get a matrix that transforms from collision object local coordinates to VSG local coordinate.

    Use this function to convert a point in collision object local coordinate space to its
    equivalent VSG object coordinate, taking center of mass and scale into account. */
    vsg::mat4 computeCOLocalToVsgLocal() const;
    /** \brief Get a matrix that transforms from VSG local coordinates to collision object local coordinate.

    Use this function to convert a point on an VSG model into its equivalent Bullet
    collision object location. */
    vsg::mat4 computeVsgLocalToCOLocal() const;
    /** \brief Get a matrix that transforms from VSG world coordinates to collision object local coordinate.

    Use this function to convert a point in VSG world space to its equivalent Bullet
    collision object location. */
    vsg::mat4 computeVsgWorldToCOLocal() const;
    /** \brief Get a matrix that transforms from VSG world coordinates to Bullet world coordinates.

    This function creates a matrix that uses the center of mass and scale values to transform
    from VSG world coordinates to Bullet world coordinates. If the center of mass is (0,0,0) and
    the scale is (1,1,1), the returned matrix is the identity. */
    vsg::mat4 computeVsgWorldToBulletWorld() const;


    /** \brief Set and get a subgraph that corresponds to the rigid body owning this MotionState.

    \param transform The vsg::Transform root of the subgraph. Bullet will use MotionState
    to set this transformation directly. \c transform must be a vsg::MatrixTransform
    or a vsg::AbsoluteTransform. */
    void setTransform( vsg::Transform* transform );
    /** \copybrief setTransform */
    vsg::Transform* getTransform();
    /** \copybrief setTransform */
    const vsg::Transform* getTransform() const;

    /** \brief Set and get the initial transformation for the MotionState.

    When making a subgraph into a rigid body, use this function to specify the initial local
    to world transformation of the subgraph so that the physics simultation starts with the
    rigid body in its correct initial location and orientation.

    \param m The initial transformation is typically the accumulated VSG local to world
    transformation obtained from the NodePath leading up to (but not including) the
    subgraph root passed to setTransform(). */
    void setParentTransform( const vsg::dmat4 m );
    /** \copybrief setParentTransform */
    vsg::dmat4 getParentTransform() const;

    /** Set and get the center of mass.

    Bullet assume the origin is the rigid body center of mass. This can
    be problematic for rigid bodies constructed from arbitrary VSG models. Use this function
    to specify a non-origin center of mass.
    \param com An \e xyz point in the subgraph's local coordinates that corresponds to
    the origin in the collision shape's local coordinates. */
    void setCenterOfMass( const vsg::vec3& com );
    /** \copybrief setCenterOfMass */
    vsg::vec3 getCenterOfMass() const;

    /** Set and get the geometric scale.

    Unlike VSG, Bullet does not support non-unit scaling. This can be problematic for
    rigid bodies constructed from VSG models that employ scale transforms. Use this function
    to specify a non-unit scale factor.
    \param scale An \e xyz scale vector, usually extracted by decomposing the parent
    transformation matrix. */
    void setScale( const vsg::vec3& scale );
    /** \copybrief setScale */
    vsg::vec3 getScale() const;

    /** \brief Support for application notification of changes to the world transformation.

    Derive a class from MotionStateCallback and override MotionStateCallback::operator()()
    with the code that you want to be executed when Bullet calls setWorldTransform(). Then
    push an instance of your callback onto this list. Your callback will be executed at
    the beginning of the setWorldTransform() function. */
    MotionStateCallbackList& getCallbackList();

    /** Transformation reset due to changes in center of mass and parent transformation.

    This is a convenience routine that calls setWorldTransform with the
    concatenation of the center of mass and parent transform. It is called by
    setCenterOfMass() and setParentTransform() to set the initial world
    transformation. See also setWorldTransformation().

    Applications typically do not need to call this function. */
    void resetTransform();


    /** \brief Register a MotionState for use with a TripleBuffer.

    Allows a MotionState to keep its world transform in a TripleBuffer object,
    which enables multithreaded physics simulation. */
    void registerTripleBuffer( vsgbDynamics::TripleBuffer* tb );

    /** \brief Gets the latest updated world transform value from the TripleBuffer
    and pushes it out to the MotionState object's VSG Transform.

    Called by TripleBufferMotionStateUpdate() and not inteded for application use. */
    void updateTripleBuffer( const char* addr );

protected:
    void setWorldTransformInternal( const btTransform& worldTrans );

    // One or the other of these will be valid, depending on whether the
    // MotionState is associated with an AbsoluteTransform or a
    // plain old MatrixTransform.
    vsg::ref_ptr< vsg::MatrixTransform > _mt;
    vsg::ref_ptr< vsg::AbsoluteTransform > _amt;

    // This is the accumulated model-to-world matrix of parent Transform nodes
    // in the scene graph.
    vsg::dmat4 _parentTransform;
    // _com is used to align the origin-centered collision shape with
    // an off-origin VSG visual representation.
    vsg::vec3 _com;

    vsg::vec3 _scale;

    // This is the transformation of the collision shape / rigid body within the Bullet physics simulation.
    // See setWorldTransformation for more details.
    btTransform _transform;

    MotionStateCallbackList _mscl;

    // TripleBuffer support.
    TripleBuffer* _tb;
    unsigned int _tbIndex;
};


/** \relates vsgbDynamics::TripleBuffer
\brief Container class for multiple MotionState objects.

*/
typedef std::set< vsgbDynamics::MotionState* > MotionStateList;

/** \relates vsgbDynamics::TripleBuffer
\brief Class for updating a list of MotionState objects from a TripleBuffer.

TripleBuffer support. Apps running Bullet is a thread separate from VSG
rendering should keep a list of all MotionState objects. During update,
the app should call TripleBufferMotionStateUpdate to update all MotionState
objects with data from the TripleBuffer (and push those matrices out to
the VSG scene graph Transform nodes).
*/
bool VSGBDYNAMICS_EXPORT TripleBufferMotionStateUpdate( vsgbDynamics::MotionStateList& msl, vsgbDynamics::TripleBuffer* tb );


// vsgbDynamics
}


// __VSGBDYNAMICS_MOTIONSTATE_H__
#endif
