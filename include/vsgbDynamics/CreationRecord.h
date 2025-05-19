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

#ifndef __VSGBDYNAMICS_CREATION_RECORD_H__
#define __VSGBDYNAMICS_CREATION_RECORD_H__ 1


#include <vsgbDynamics/Export.h>
#include <vsgbCollision/CollisionShapes.h>

#include <vsg/core/Object.h>
#include <vsg/core/Inherit.h>
#include <vsg/maths/mat4.h>
#include <btBulletDynamicsCommon.h>

namespace vsgbDynamics
{



/** \class CreationRecord CreationRecord.h <vsgbDynamics/CreationRecord.h>
\brief Data record for rigid body construction information.

Fill in this struct and pass it to the createRigidBody() functions
to specify rigid body (and collision shape) construction parameters.
See the \link rigidbody rigid body creation utilities \endlink
for more information.

This record can be stored in UserData on the rigid body subgraph
root node to facilitate saving and restoring physics state.

This struct can be serialized to/from the .vsg file format. */
struct VSGBDYNAMICS_EXPORT CreationRecord : public vsg::Inherit<vsg::Object,CreationRecord>
{
    CreationRecord();
    CreationRecord( const CreationRecord& rhs, vsg::CopyOp copyop={});

    void setSceneGraphNode(vsg::ref_ptr<vsg::Node> s ){_sceneGraph=s;}
    vsg::ref_ptr<vsg::Node> getSceneGraphNode()const{return _sceneGraph;}
    vsg::ref_ptr<vsg::Node> _sceneGraph;

    /** The dot VSG representation of this class maintains a version number for
    debugging purposes, allow deprecation or old data, and allow introduction
    of new data. Do not explicitly set this value. The constructor initializes
    this to the current version, and the dot VSG read function Creation_readLocalData()
    will set it based on the value found in the dot VSG file being loaded. */
    unsigned int _version;

    /** Specify the center of mass. If not set, vsgbDynamics::createRigidBody
    will use the scene graph bounding volume center as the center of mass. */
    void setCenterOfMass( const vsg::vec3& com );
    const vsg::vec3& getCenterOfMass()const{return _com;}
    vsg::vec3 _com;
    void setIsCOMSet( bool com ){_comSet=com;}
    bool getIsCOMSet()const{return _comSet;}
    bool _comSet;

    void setMass( float com ){_mass=com;}
    float getMass()const{return _mass;}
    float _mass;



    /** Specify the collision shape margin for convex hull and convex tri mesh
    collision shapes.

    Note: Margin is currently used only if _overall is true. */
    void setMargin(  float margin );
    float getMargin()const{return _margin;}
    float _margin;



    void setScale( const vsg::vec3& com ){_scale=com;};
    const vsg::vec3& getScale()const{return _scale;}
    vsg::vec3 _scale;

    void setParentTransform( const vsg::mat4& com ){_parentTransform=com;};
    const vsg::mat4& getParentTransform()const{return _parentTransform;}
    vsg::mat4 _parentTransform;


    bool _marginSet;
    BroadphaseNativeTypes _shapeType;

    void setRestitution(  float f ){_restitution=f;};
    float getRestitution()const{return _restitution;}
    float _restitution;

    void setFriction(  float f ){_friction=f;};
    float getFriction()const{return _friction;}
    float _friction;


    void setRollingFriction(  float f ){_rollingFriction=f;};
    float getRollingFriction()const{return _rollingFriction;}
    float _rollingFriction;


    void setLinearDamping(  float f ){_linearDamping=f;};
    float getLinearDamping()const{return _linearDamping;}
    float _linearDamping;
    void setAngularDamping(  float f ){_angularDamping=f;};
    float getAngularDamping()const{return _angularDamping;}
    float _angularDamping;

    /** For _shapeType == CYLINDER_SHAPE_PROXYTYPE only. */
    vsgbCollision::AXIS _axis;

    bool _overall;
    /** Corresponds to the \c _reductionLevel parameter for
    vsgbCollision::btCompoundShapeFromVSGGeodes(). */
    typedef enum {
        NONE = 0,
        MINIMAL = 1,
        INTERMEDIATE = 2,
        AGGRESSIVE = 3
    } ReductionLevel;
    /** \brief Specify optional geometry reduction. Default is NONE.
    */
    void setReductionLevel(ReductionLevel com ){_reductionLevel=com;};
    ReductionLevel getReductionLevel()const{return _reductionLevel;}
    ReductionLevel _reductionLevel;

};


// vsgbDynamics
}


// __VSGBDYNAMICS_CREATION_RECORD_H__
#endif
