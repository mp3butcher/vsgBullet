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

#include <vsgbDynamics/Joint.h>
#include <vsgbDynamics/CreationRecord.h>
#include <vsgbDynamics/MotionState.h>
#include <vsgbDynamics/World.h>
#include <vsgbCollision/Utils.h>
#include <vsgbCollision/CollisionShapes.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>

#include <vsgbCollision/Utils.h>
#include <vsgbDynamics/TripleBuffer.h>

namespace vsgbDynamics
{

Joint::Joint():_rigA(0), _rigB(0), _btConstraint(0){}
Joint::~Joint(){
//if(_rigA)_rigA->removeJoint(this);
//if(_rigB)_rigA->removeJoint(this);
std::cerr<<"Constraint DELETED"<<std::endl;
delete _btConstraint;
}

    Joint::Joint( const Joint& copy, const vsg::CopyOp& copyop ){}
const btTypedConstraint* Joint::getConstraint()const
{
return _btConstraint;
  //  const   vsgbCollision::RefBulletObject<  btTypedConstraint >*  refptr=
   //     dynamic_cast<  const   vsgbCollision::RefBulletObject<  btTypedConstraint >* >( getUserData() );
   // return (refptr?refptr->get():0);
}

btTypedConstraint* Joint::getConstraint()
{
return _btConstraint;
  //  vsgbCollision::RefBulletObject<  btTypedConstraint >*  refptr=
   //     dynamic_cast<     vsgbCollision::RefBulletObject<  btTypedConstraint >* >( getUserData() );

  //  return (refptr?refptr->get():0);
}

void Joint::setConstraint( btTypedConstraint *b )
{
 _btConstraint=b;
    //setUserData(new vsgbCollision::RefBulletObject<btTypedConstraint>(b));
    }
  void Joint::setBodyA(RigidBody *c)
    {
    if(_rigA==c)return;
    if(_rigA!=c && _rigA)_rigA->removeJoint(this);
        _rigA=c;
    if(_rigA)_rigA->addJoint(this);
    }

    void Joint::setBodyB(RigidBody *c)
    {
    if(_rigB==c)return;
    if(_rigB!=c && _rigB)_rigB->removeJoint(this);
        _rigB=c;
    if(_rigB)_rigB->addJoint(this);
    }

// vsgbDynamics
}
