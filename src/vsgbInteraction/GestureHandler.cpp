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

#include <vsgbInteraction/GestureHandler.h>
#include <vsgbInteraction/HandNode.h>
#include <vsg/io/Logger.h>

#include <btBulletDynamicsCommon.h>
#include <btBulletCollisionCommon.h>


namespace vsgbInteraction
{


// Static gesture codes
const unsigned int GestureHandler::Unknown( 0 );
const unsigned int GestureHandler::Default( 1 );
const unsigned int GestureHandler::Point( 2 );
const unsigned int GestureHandler::Fist( 3 );


GripRelease::GripRelease()
  : _constraint( nullptr )
{
}
GripRelease::~GripRelease()
{
    if( _constraint != nullptr )
        delete _constraint;
}

bool
GripRelease::operator()( const unsigned int gestureCode, HandNode& handNode )
{
    switch( gestureCode ) {
    case Default:
    {
        vsg::log(vsg::Logger::LOGGER_INFO,  "Received Default." );

        btDynamicsWorld* bulletWorld = handNode.getDynamicsWorld();
        if( _constraint != nullptr )
        {
            bulletWorld->removeConstraint( _constraint );
            _constraint = nullptr;
        }

        return( true );
        break;
    }
    case Fist:
    {
        vsg::log(vsg::Logger::LOGGER_INFO, "Received Fist." );

        // Physics thread should already be stopped; see HandNode::sendGestureCode.
        btRigidBody* closest = handNode.findClosest();
        if( closest == nullptr )
        {
            vsg::log(vsg::Logger::LOGGER_ERROR, "GripRelease got nullptr from HandNode::findClosest()." );
            return( false );
        }

        // Constraint parameters: The transform for the selected object in
        // the hand's coordinate space is: HandWTInv * BodyWT.
        btTransform xformA = handNode.getRigidBody()->getWorldTransform().inverse() *
            closest->getWorldTransform();
        btTransform xformB;
        xformB.setIdentity();

        // Constrain.
        _constraint = new btGeneric6DofConstraint( *( handNode.getRigidBody() ),
            *closest, xformA, xformB, false );
        // Constraint rotations.
        _constraint->setAngularLowerLimit( btVector3(0,0,0) );
        _constraint->setAngularUpperLimit( btVector3(0,0,0) );
        handNode.getDynamicsWorld()->addConstraint( _constraint, true );

        return( true );
        break;
    }
    default:
    {
        vsg::log(vsg::Logger::LOGGER_WARN,  "Unknown gesture code: " + gestureCode );
        return( false );
        break;
    }
    }
}


// vsgbInteraction
}
