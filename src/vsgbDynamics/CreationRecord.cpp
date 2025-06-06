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

#include <vsgbDynamics/CreationRecord.h>

#include <vsgbCollision/CollisionShapes.h>
#include <vsg/nodes/Node.h>


#include <btBulletDynamicsCommon.h>


namespace vsgbDynamics
{

CreationRecord::CreationRecord()
  : _sceneGraph( nullptr ),
    _version( 3 ),
    _com( 0., 0., 0. ),
    _comSet( false ),
    _mass( 1.f ),
    _margin( 0.f ),
    _scale( vsg::vec3( 1., 1., 1. ) ),
    _parentTransform( vsg::mat4() ),
    _marginSet( false ),
    _shapeType( BOX_SHAPE_PROXYTYPE ),
    _restitution( 0.f ),
    _friction( 1.f ),
    _rollingFriction( -1.f ),
    _linearDamping( -1.f ),
    _angularDamping( -1.f ),
    _axis( vsgbCollision::Z ),
    _overall( false ),
    _reductionLevel( CreationRecord::NONE )
{
}
CreationRecord::CreationRecord( const CreationRecord& rhs, vsg::CopyOp copyop )
  : _sceneGraph( rhs._sceneGraph ),
    _version( rhs._version ),
    _com( rhs._com ),
    _comSet( rhs._comSet ),
    _margin( rhs._margin ),
    _marginSet( rhs._marginSet ),
    _scale( rhs._scale ),
    _parentTransform( rhs._parentTransform ),
    _shapeType( rhs._shapeType ),
    _mass( rhs._mass ),
    _restitution( rhs._restitution ),
    _friction( rhs._friction ),
    _rollingFriction( rhs._rollingFriction ),
    _linearDamping( rhs._linearDamping ),
    _angularDamping( rhs._angularDamping ),
    _axis( rhs._axis ),
    _reductionLevel( rhs._reductionLevel ),
    _overall( rhs._overall )
{
}

void CreationRecord::setMargin( const float margin )
{
    _margin = margin;
    _marginSet = true;
}

void CreationRecord::setCenterOfMass( const vsg::vec3& com )
{
    _com = com;
    _comSet = true;
}


// vsgbDynamics
}
