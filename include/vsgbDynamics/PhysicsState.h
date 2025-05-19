/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * osgBullet is (C) Copyright 2025 by Julien Valentin
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

#ifndef __OSGBDYNAMICS_PHYSICS_STATE_H__
#define __OSGBDYNAMICS_PHYSICS_STATE_H__ 1

#include <vsgbDynamics/Export.h>
#include <vsgbDynamics/CreationRecord.h>
/*#include <osgwTools/RefID.h>
#include <osg/Object>
#include <osg/Group>
#include <osgDB/Output>
*/
#include <btBulletDynamicsCommon.h>


namespace vsgbDynamics {

/** \brief A reference counter string-based identifier
*/
class VSGBDYNAMICS_EXPORT RefID : public vsg::Inherit<vsg::Object, RefID>
{
public:
    RefID();
    RefID( const std::string& id );

    RefID( const RefID& rhs, const vsg::CopyOp copyop= {} );


    bool operator==( const RefID& rhs ) const;
    bool operator<( const RefID& rhs ) const;

    void set( const std::string& id );
    const std::string& str() const;

protected:
    ~RefID();

    std::string _str;
};

/** \class PhysicsData PhysicsState.h <vsgbDynamics\PhysicsState.h>
\brief TBD

*/
class VSGBDYNAMICS_EXPORT PhysicsData : public vsg::Inherit<vsg::Object, PhysicsData>
{
public:
    PhysicsData();
    PhysicsData( const PhysicsData& rhs, vsg::CopyOp copyop={});

    PhysicsData& operator=( const PhysicsData& rhs );

    /** \brief Obtains rigid body information from the physics simulation.

    If \c _body is non-nullptr, this function stores the rigid body's current world
    transform, linear velocity, and angulat velocity in the analogous fields. */
    void loadState();

    /** \brief Restore rigid body information to the physics simultation.

    If \c _body is non-nullptr, this function restores the saves \c _bodyWorldTransform,
    |c _angularVelocity, and \c _linearVelocity. */
    void restoreState() const;

    std::string _fileName;
    vsg::ref_ptr< vsgbDynamics::CreationRecord > _cr;
    btRigidBody* _body;

    // For save / restore use only
    vsg::mat4 _osgTransform;
    vsg::mat4 _bodyWorldTransform;
    vsg::vec3 _linearVelocity;
    vsg::vec3 _angularVelocity;

    /** \deprecated Existed in \c _version 2 only.
    Superceded by CreationRecord::_friction. */
    double _friction;
    /** \deprecated Existed in \c _version 2 only.
    Superceded by CreationRecord::_restitution. */
    double _restitution;

    unsigned int getVersion() const { return( _version ); }

protected:
    ~PhysicsData();

    unsigned int _version;
};

/** \class PhysicsState PhysicsState.h <vsgbDynamics\PhysicsState.h>
\brief TBD

*/
class VSGBDYNAMICS_EXPORT PhysicsState : public vsg::Inherit<vsg::Object, PhysicsState>
{
public:
    PhysicsState();
    PhysicsState( const vsgbDynamics::PhysicsState& rhs, vsg::CopyOp copyop={} );
    ~PhysicsState();


    void addPhysicsData( const RefID* id, PhysicsData* pd );
    void addPhysicsData( const RefID* id, const btRigidBody* body );
    void addPhysicsData( const RefID* id, const vsgbDynamics::CreationRecord* cr );
    void addPhysicsData( const RefID* id, const std::string& fileName );
    //void addPhysicsData( const RefID* id, const btConstraint& constraint );

    void addPhysicsData( const std::string& id, PhysicsData* pd );
    void addPhysicsData( const std::string& id, const btRigidBody* body );
    void addPhysicsData( const std::string& id, const vsgbDynamics::CreationRecord* cr );
    void addPhysicsData( const std::string& id, const std::string& fileName );
    //void addPhysicsData( const std::string& id, const btConstraint& constraint );

    unsigned int getNumEntries() const;
    const PhysicsData* getPhysicsData( const RefID* id ) const;
    const PhysicsData* getPhysicsData( const std::string& id ) const;
    PhysicsData* getPhysicsData( const std::string& id );

    void removePhysicsData( const RefID* id );
    void removePhysicsData( const std::string& id );

    typedef std::map< std::string, vsg::ref_ptr< PhysicsData > > DataMap;
    //void exportEntired( osgDB::Output& out ) const;

    /** \brief Load current state from the physics simultation.

    This function calls PhysicsData::loadState() for each element of \c _dataMap.
    Use this function to take a "snapshot" of the current physics simultaion.
    osgBullet uses this functionality to save physics state to disk. */
    void loadState();

    /** \brief Restore physics state to the physics simulation.

    This function calls PhysicsData::restoreState() for each element of \c _dataMap.
    Use this function to reset the physics simulation to a previous state. For example,
    this function enalbes a "reset" button in your application.

    Together, loadState() and restoreState() support the ability for an app to save
    a physics simulation to disk and restore it later in mid-simulation. */
    void restoreState() const;

protected:
    DataMap _dataMap;
};


// vsgbDynamics
}

// __OSGBDYNAMICS_PHYSICS_STATE_H__
#endif
