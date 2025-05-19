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

#ifndef __VSGBDYNAMICS_Joint_H__
#define __VSGBDYNAMICS_Joint_H__  1

#include <vsgbDynamics/Export.h>
#include <vsg/core/Objects.h>
#include <BulletDynamics/ConstraintSolver/btTypedConstraint.h>

namespace vsgbDynamics
{
class World;
class RigidBody;

class VSGBDYNAMICS_EXPORT Joint : public  vsg::Object
{
public:
    Joint();
    Joint( const Joint& copy, const vsg::CopyOp& copyop={} );

    const btTypedConstraint* getConstraint()const;
    btTypedConstraint* getConstraint();

    void setConstraint( btTypedConstraint *b );
    const vsgbDynamics::RigidBody *getBodyA()const    {        return _rigA;    }
     vsgbDynamics::RigidBody *getBodyA()    {        return _rigA;    }
    void setBodyA(RigidBody *c);
    const RigidBody *getBodyB()const    {        return _rigB;    }
     RigidBody *getBodyB()    {        return _rigB;    }
    void setBodyB(RigidBody *c);

protected:
    ~Joint();

    RigidBody* _rigA;
    RigidBody* _rigB;
    btTypedConstraint*_btConstraint;

};

// vsgbDynamics
}


#endif
