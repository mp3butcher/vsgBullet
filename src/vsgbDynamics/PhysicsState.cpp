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

#include <vsgbDynamics/PhysicsState.h>
#include <vsgbCollision/Utils.h>

#include <btBulletDynamicsCommon.h>

namespace vsgbDynamics
{

RefID::RefID()
{
}
RefID::RefID( const std::string& id )
{
    set( id );
}

RefID::RefID( const RefID& rhs, const vsg::CopyOp copyop )
{
    _str = rhs._str;
}
RefID::~RefID()
{
}

bool RefID::operator==( const RefID& rhs ) const
{
    return( _str == rhs._str );
}
bool RefID::operator<( const RefID& rhs ) const
{
    return( _str < rhs._str );
}

void RefID::set( const std::string& id )
{
    _str = id;
}
const std::string&
RefID::str() const
{
    return( _str );
}


PhysicsData::PhysicsData()
    : 
    _fileName( std::string( "" ) ),
    _cr( nullptr ),
    _body( nullptr ),
    _friction( 0.1 ),
    _restitution( 1 ),
    _version( 3 )
{
    ;
}
PhysicsData::PhysicsData( const PhysicsData& rhs, vsg::CopyOp copyop )
{
    (*this) = rhs;
}
PhysicsData::~PhysicsData()
{
}

PhysicsData&
PhysicsData::operator=( const PhysicsData& rhs )
{
    _version = rhs._version;
    _fileName = rhs._fileName;
    _cr = rhs._cr;
    _body = rhs._body;

    return( *this );
}


void PhysicsData::loadState()
{
    if( _body == nullptr )
        return;

    _bodyWorldTransform = vsgbCollision::asVsgMatrix( _body->getWorldTransform() );
    _linearVelocity = vsgbCollision::asVsgVec3( _body->getLinearVelocity() );
    _angularVelocity = vsgbCollision::asVsgVec3( _body->getAngularVelocity() );
}

void PhysicsData::restoreState() const
{
    if( _body == nullptr )
        return;

    _body->setWorldTransform( vsgbCollision::asBtTransform( _bodyWorldTransform ) );
    _body->setLinearVelocity( vsgbCollision::asBtVector3( _linearVelocity ) );
    _body->setAngularVelocity( vsgbCollision::asBtVector3( _angularVelocity ) );
}



////////////////////////////////////////////////////////////////////////////////

PhysicsState::PhysicsState()
{
}
PhysicsState::PhysicsState( const vsgbDynamics::PhysicsState& rhs, vsg::CopyOp copyop )
{
}
PhysicsState::~PhysicsState()
{
}

void PhysicsState::addPhysicsData( const RefID* id, PhysicsData* pd )
{
    addPhysicsData( id->str(), pd );
}
void PhysicsState::addPhysicsData( const RefID* id, const btRigidBody* body )
{
    addPhysicsData( id->str(), body );
}
void PhysicsState::addPhysicsData( const RefID* id, const vsgbDynamics::CreationRecord* cr )
{
    addPhysicsData( id->str(), cr );
}
void PhysicsState::addPhysicsData( const RefID* id, const std::string& fileName )
{
    addPhysicsData( id->str(), fileName );
}
//void PhysicsState::addPhysicsData( const osgwTools::RefID* id, const btConstraint& constraint )
//{
//    addPhysicsData( id, constraint );
//}

void PhysicsState::addPhysicsData( const std::string& id, PhysicsData* pd )
{
    if( _dataMap.find( id ) != _dataMap.end() )
        vsg::warn("Overwriting physics data for \"" , id , "\"" );

    _dataMap[ id ] = pd;
}
void PhysicsState::addPhysicsData( const std::string& id, const btRigidBody* body )
{
    DataMap::iterator it = _dataMap.find( id );
    if( it == _dataMap.end() )
    {
        vsg::ref_ptr< PhysicsData > pd = PhysicsData::create();
        pd->_body = const_cast< btRigidBody* >( body );
        _dataMap[ id ] = pd.get();
    }
    else
    {
        it->second->_body = const_cast< btRigidBody* >( body );
    }
}
void PhysicsState::addPhysicsData( const std::string& id, const vsgbDynamics::CreationRecord* cr )
{
    DataMap::iterator it = _dataMap.find( id );
    if( it == _dataMap.end() )
    {
        vsg::ref_ptr< PhysicsData > pd = PhysicsData::create();
        pd->_cr = const_cast< CreationRecord* >( cr );
        _dataMap[ id ] = pd.get();
    }
    else
    {
        it->second->_cr = const_cast< CreationRecord* >( cr );
    }
}
void PhysicsState::addPhysicsData( const std::string& id, const std::string& fileName )
{
    DataMap::iterator it = _dataMap.find( id );
    if( it == _dataMap.end() )
    {
        vsg::ref_ptr< PhysicsData > pd = PhysicsData::create();
        pd->_fileName = fileName;
        _dataMap[ id ] = pd.get();
    }
    else
    {
        it->second->_fileName = fileName;
    }
}
//void PhysicsState::addPhysicsData( const std::string& id, const btConstraint& constraint )
//{
//}

unsigned int PhysicsState::getNumEntries() const
{
    return( _dataMap.size() );
}
/*void PhysicsState::exportEntired( osgDB::Output& out ) const
{
    DataMap::const_iterator it;
    for( it = _dataMap.begin(); it != _dataMap.end(); ++it )
    {
        vsg::ref_ptr< osgwTools::RefID > rid = new osgwTools::RefID( it->first );
        out.writeObject( *rid );
        out.writeObject( *( it->second ) );
    }
}*/

const PhysicsData* PhysicsState::getPhysicsData( const RefID* id ) const
{
    return( getPhysicsData( id->str() ) );
}
const PhysicsData* PhysicsState::getPhysicsData( const std::string& id ) const
{
    DataMap::const_iterator it = _dataMap.find( id );
    if( it != _dataMap.end() )
        return( it->second.get() );
    else
        return( nullptr );
}
PhysicsData* PhysicsState::getPhysicsData( const std::string& id )
{
    DataMap::const_iterator it = _dataMap.find( id );
    if( it != _dataMap.end() )
        return( it->second.get() );
    else
        return( nullptr );
}

void PhysicsState::removePhysicsData( const RefID* id )
{
    removePhysicsData( id->str() );
}
void PhysicsState::removePhysicsData( const std::string& id )
{
    DataMap::iterator it = _dataMap.find( id );
    if( it == _dataMap.end() )
        vsg::warn("Can't erase non-extant RefID (RefID::operator<<() TBD)." );
    else
        _dataMap.erase( it );
}


void PhysicsState::loadState()
{
    DataMap::iterator it;
    for( it = _dataMap.begin(); it != _dataMap.end(); ++it )
        it->second->loadState();
}

void PhysicsState::restoreState() const
{
    DataMap::const_iterator it;
    for( it = _dataMap.begin(); it != _dataMap.end(); ++it )
        it->second->restoreState();
}



// vsgbDynamics
}
