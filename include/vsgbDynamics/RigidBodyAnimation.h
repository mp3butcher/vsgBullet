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

#ifndef __VSGBDYAMICS_RIGID_BODY_ANIMATION_H__
#define __VSGBDYAMICS_RIGID_BODY_ANIMATION_H__ 1

#include <vsg/core/Visitor.h>
#include <vsg/core/Inherit.h>

#include <vsgbDynamics/Export.h>

namespace vsgbDynamics {


/** \class RigidBodyAnimation RigidBodyAnimation.h <vsgbDynamics/RigidBodyAnimation.h>
\brief An update callback to reposition a btRigidBody in the Bullet simulation.

This callback repositions an object within the Bullet simulation. Attach it as
an update callback to an VSG MatrixTransform. The MatrixTransform must have an
vsgbCollision::RefBulletObject< btRigidBody > attached as UserData. */
class VSGBDYNAMICS_EXPORT RigidBodyAnimation : public vsg::Inherit<vsg::Visitor,RigidBodyAnimation>
{
public:
    RigidBodyAnimation(   );
    RigidBodyAnimation(const RigidBodyAnimation& a,vsg::CopyOp op={}  ){}

    virtual void operator()( vsg::Node* node, vsg::Visitor* nv );

protected:
    virtual ~RigidBodyAnimation() { }
};


// vsgbDynamics
}

// __VSGBDYAMICS_RIGID_BODY_ANIMATION_H__
#endif
