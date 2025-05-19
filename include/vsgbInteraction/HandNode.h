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

#ifndef __VSGBINTERACTION_HAND_NODE_H__
#define __VSGBINTERACTION_HAND_NODE_H__ 1


#include <vsgbInteraction/Export.h>
#include <vsgbInteraction/GestureHandler.h>
#include <vsg/core/Visitor.h>
#include <vsg/nodes/MatrixTransform.h>

#include <btBulletCollisionCommon.h>

#include <vector>
#include <map>
#include <string>


// Uncomment to replace the hand btCompoundShape with a btBoxShape.
// This is a debugging aid to track down problems with hand penetration
// caused by use of btCompoundShape.
//#define USE_SIMPLE_BOX


// Forward
class btDynamicsWorld;
class btRigidBody;
struct btDefaultMotionState;
class btCompoundShape;
#ifdef USE_SIMPLE_BOX
class btBoxShape;
#endif
class btPairCachingGhostObject;
namespace vsgbDynamics {
    class PhysicsThread;
}


namespace vsgbInteraction
{


// Forward
class FindArticulations;

/** \class HandNode HandNode.h <vsgbInteraction/HandNode.h>
\brief A Node that renders a hand model in the VSG scene and maintains a Bullet
rigid body and collision shape representation of the hand.

This version of HandNode has a calibration feature. call
setCalibrateMode( true ), then move your physical hand to match
the position of the virtual hand. The virtual hand will not move
while in calibrate mode. After your physical hand matches the
cirtual hand, call setCalibrateMode( false ).
*/
class VSGBINTERACTION_EXPORT HandNode : public vsg::Transform
{
public:
    enum Handedness {
        RIGHT, LEFT
    };

    HandNode( btDynamicsWorld* bulletWorld, const HandNode::Handedness rightOrLeft=HandNode::RIGHT, float handLength=HandNode::_defaultLength );
    HandNode( const HandNode& rhs, const vsg::CopyOp& copyop={} );

    HandNode(); // Required for META_Node, but not intended for actual use.

   // virtual void traverse( vsg::NodeVisitor& nv );

  //  virtual vsg::BoundingSphere computeBound() const;

  //  virtual bool computeLocalToWorldMatrix( vsg::mat4& matrix, vsg::NodeVisitor* nv ) const;
   // virtual bool computeWorldToLocalMatrix( vsg::mat4& matrix, vsg::NodeVisitor* nv ) const;

    struct AllParams {
        AllParams() 
          : _spread01( 0.f ), _spread12( 0.f ), _spread23( 0.f ), _spread34( 0.f )
        {}

        vsg::vec3 _pos;
        vsg::quat _att;
        float _spread01, _spread12, _spread23, _spread34;
        vsg::vec2 _finger0; // Thumb knuckle 1 and 2
        vsg::vec2 _finger1; // Pointer knuckle 0 and 1/2
        vsg::vec2 _finger2; // Middle knuckle 0 and 1/2
        vsg::vec2 _finger3; // Ring knuckle 0 and 1/2
        vsg::vec2 _finger4; // Pinky knuckle 0 and 1/2
    };
    // Set articulations, position, and attitude with a single function call,
    // stopping the physics thread only once for all changes.
    void setAll( const AllParams& params );
    void getAll( AllParams& params ) const;

    // Deprecated. Use setAll().
    void setPosition( const vsg::vec3& pos );
    inline const vsg::vec3& getPosition() const { return _requestedPosition; }

    // Deprecated. Use setAll().
    void setAttitude( const vsg::quat& quat );
    inline const vsg::quat& getAttitude() const { return _attitude; }

    inline void setTraverseHand( const bool traverse ) { _traverseHand = traverse; dirtyBound(); }
    inline bool getTraverseHand() const { return _traverseHand; }

    // Set right or left hand.
    // Set this in the constructor for maximum efficiency.
    void setHandedness( const HandNode::Handedness rightOrLeft );
    HandNode::Handedness getHandedness() const;

    // Set the desired world coordinate length of the hand. For example:
    //   about 0.15 if working in meters, or about 0.5 if working in feet.
    //   The default is about 6.5 (the hand was modeled in inches).
    void setHandLength( float length );
    float getHandLength() const;

    typedef int Articulation;
    enum {
        //
        // Actual articulations
        //

        // lateral rotation / flexure
        FINGER_0_TRANSLATE = 0, // thumb
        FINGER_1_TRANSLATE, // pointer
        FINGER_2_TRANSLATE, // middle
        FINGER_3_TRANSLATE, // ring
        FINGER_4_TRANSLATE, // pinky

        // rotation at inner knuckle
        FINGER_0_ROTATE_INNER,
        FINGER_1_ROTATE_INNER,
        FINGER_2_ROTATE_INNER,
        FINGER_3_ROTATE_INNER,
        FINGER_4_ROTATE_INNER,

        // rotation at middle knuckle
        FINGER_0_ROTATE_MIDDLE,
        FINGER_1_ROTATE_MIDDLE,
        FINGER_2_ROTATE_MIDDLE,
        FINGER_3_ROTATE_MIDDLE,
        FINGER_4_ROTATE_MIDDLE,

        // rotation at outer knuckle
        FINGER_0_ROTATE_OUTER,
        FINGER_1_ROTATE_OUTER,
        FINGER_2_ROTATE_OUTER,
        FINGER_3_ROTATE_OUTER,
        FINGER_4_ROTATE_OUTER,

        LAST_ACTUAL_ARTICULATION,

        //
        // Vistual articulations
        //

        // For fingers, data glove gets one angle for both the
        // middle and outer knucles. These virtual articulations
        // will set the rotation angle for both the middle and
        // outer knuckles.
        FINGER_1_MIDDLE_OUTER,
        FINGER_2_MIDDLE_OUTER,
        FINGER_3_MIDDLE_OUTER,
        FINGER_4_MIDDLE_OUTER,
        // NOTE: FINGER_0 (thumb) isn't included because the
        // glove provides angles for both the middle and outer
        // knuckles.

        // Spread angles. Setting these virtual articulations will
        // override the "lateral rotation / flexure" articulations.
        SPREAD_0_1,
        SPREAD_1_2,
        SPREAD_2_3,
        SPREAD_3_4,

        MAX_ARTICULATIONS
    };

    // Deprecated. Use setAll().
    void setArticulation( const HandNode::Articulation part, const float radians );
    float getArticulation( const HandNode::Articulation part ) const;

    void setCalibrateMode( const bool calibrate );
    bool getCalibrateMode() const;

    // Gesture support. GestureHandler objects are stored in
    // the HandNode's GenstureHandlerVector. When the apps calls
    // sendGestureCode(), the code is passed to each handler in
    // the vector until one of the handlers returns true.
    void setGestureHandlerVector( vsgbInteraction::GestureHandlerVector ghv );
    vsgbInteraction::GestureHandlerVector& getGestureHandlerVector();
    void sendGestureCode( const unsigned int gestureCode );

    // Return the closest rigid body that does not have the STATIC collision flag set.
    btRigidBody* findClosest() const;

    // Verify the hand initialized successfully.
    bool good() const { return( _hand.valid() ); }

    // Support for vsgBullet::PhysicsThread multithreaded physics.
    void registerMultiThreaded( vsgbDynamics::PhysicsThread* pt );

    // Access various internal physics objects.
    btDynamicsWorld* getDynamicsWorld() { return( _bulletWorld ); }
    btRigidBody* getRigidBody() { return( _body ); }
    btPairCachingGhostObject* getGhostObject() { return( _ghost ); }

    // Enable/disable use of ghost object. On by default.
    void setUseGhost( bool useGhost );
    bool getUseGhost() const;



    //
    // Debug / test
    //

    // When on, display _requestedPosition as a red point.
    void setDebug( bool debug ) { _debug = debug; init(); }

    // NOTE: Poses are primarily available as a debugging/testing aid.
    // Do not confuse poses with gestures. Poses move the hand to exercise
    // articulations, gestures are a signal from an external app to trigger an action.
    // Predefined positions, called "Pose" to avoid collision with base class "setPosition".
    typedef int Pose;
    enum {
        POSE_DEFAULT = 0,
        POSE_HOOK,
        POSE_POINT,
        POSE_FIST
    };

    void setPose( Pose pose, float radiansPerSec=(float)(vsg::PI) );

    // Dumps VSG files for children, and displays info to std::out
    //   including the current PAT matrix and the articulation angles.
    void dump() const;


    //
    // Made public for the benefit of helper NodeVisitors.
    //

    struct ArticulationInfo
    {
        ArticulationInfo();
        ~ArticulationInfo();

        void setAngle( float angle );
        float getAngle() const;

        void setBulletTransform();

        vsg::ref_ptr< vsg::MatrixTransform > _mt;
        vsg::vec3 _axis;
        vsg::vec3 _pivotPoint;

        int _btChildIdx;

        vsg::NodePath _l2wNodePath;
        float _angle;
        float _calibrateBaseAngle;

        ArticulationInfo* _dependent;
        btCompoundShape* _cs;

        void dump() const;

        bool _valid;
    };
    typedef std::vector< ArticulationInfo > ArticulationInfoList;

protected:
    void init();
    // Destructor
    virtual ~HandNode();
    void cleanup();

    // Incorporate transformation changes into the Bullet collision shapes.
    void adjustPosition();
    void updateTransformAndAdjustPosition( const vsg::vec3& deltaMotion=vsg::vec3(0.,0.,0.) );
    bool recoverFromPenetration();

    // Note: the "internal" functions assume the physics thread is already stopped.
    bool adjustPositionInternal( const vsg::vec3& deltaMotion=vsg::vec3(0.,0.,0.) );
    void updateTransformInternal( const vsg::vec3& deltaMotion=vsg::vec3(0.,0.,0.) );
    void setArticulationInternal( const HandNode::Articulation part, const float radians );

    // Called by setArticulation() and getArticulation() to handle all virtual articulations.
    void setVirtualArticulation( const HandNode::Articulation part, const float radians );
    float getVirtualArticulation( const HandNode::Articulation part ) const;


    vsg::ref_ptr< vsg::Node > _hand;
    ArticulationInfoList _ail;
    ArticulationInfo _palm;
    bool _calibrate;

    vsg::vec3 _requestedPosition;
    vsg::vec3 _correctedPosition;

    Handedness _rightOrLeft;
    vsg::Quat _attitude;
    float _length;
    static float _defaultLength;

    btDynamicsWorld* _bulletWorld;
    btRigidBody* _body;
#ifdef USE_SIMPLE_BOX
    btBoxShape* _shape;
#else
    btCompoundShape* _shape;
#endif
    bool _useGhost;
    btPairCachingGhostObject* _ghost;
    btManifoldArray	_manifoldArray;

    vsgbInteraction::GestureHandlerVector _ghv;


    bool _traverseHand;


    vsgbDynamics::PhysicsThread* _pt;


    // For displaying the _requestedPosition as a fat red point.
    bool _debug;
    vsg::ref_ptr< vsg::vec3Array > _debugVerts;
};

// vsgbInteraction
}

// __VSGBINTERACTION_HAND_NODE_H__
#endif
