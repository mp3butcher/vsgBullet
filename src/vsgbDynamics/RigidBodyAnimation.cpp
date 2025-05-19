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

#include <vsgbDynamics/RigidBodyAnimation.h>
#include <vsgbCollision/RefBulletObject.h>
#include <vsgbCollision/Utils.h>
#include <btBulletDynamicsCommon.h>
#include <vsg/nodes/MatrixTransform.h>
#include <vsgbDynamics/RigidBody.h>


#include <iostream>

#include <btBulletCollisionCommon.h>


namespace vsgbDynamics
{


RigidBodyAnimation::RigidBodyAnimation( void )
{
}

void RigidBodyAnimation::operator()( vsg::Node* node, vsg::Visitor* nv )
{
    /*
vsg::MatrixTransform* matTrans = static_cast< vsg::MatrixTransform* >( node);

vsgbDynamics::RigidBody* rig=dynamic_cast<vsgbDynamics::RigidBody*> (matTrans->getUpdateCallback());
    btRigidBody* body = rig->getRigidBody();
    if( body->getInvMass() != 0.0 ||!rig->getParentWorld())
    {
        traverse( node, nv );;
   }

    vsg::mat4 mat = computeLocalToWorld( node->getParentalNodePaths()[0] );
    //vsg::mat4 mat = matTrans->getMatrix();
    body->getMotionState()->setWorldTransform(
        vsgbCollision::asBtTransform( mat ) );
    traverse( node, nv );
*/
}


// vsgbDynamics
}
