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

#ifndef __VSGBCOLLISION_COMPUTECYLINDERVISITOR_H__
#define __VSGBCOLLISION_COMPUTECYLINDERVISITOR_H__ 1

#include <vsgbCollision/Export.h>
#include <vsg/core/Visitor.h>
#include <vsgbCollision/BoundingCylinder.h>

namespace vsgbCollision
{


/** \class ComputeCylinderVisitor ComputeCylinderVisitor.h <vsgbCollision/ComputeCylinderVisitor.h>
\brief Computes the extents of a cylinder around specified VSG data.

TBD Consider using VSG localtoworld method instead of keeping a matrix stack.
*/
class VSGBCOLLISION_EXPORT ComputeCylinderVisitor : public vsg::Visitor
{
public:
    ComputeCylinderVisitor( );


    virtual void reset();


    virtual void setAxis( const vsg::vec3 a )
    {
        axis = normalize(a);
        bc.setAxis( axis );
    }

    vsgbCollision::BoundingCylinder& getBoundingCylinder()
    {
        return( bc );
    }

    void apply( vsg::Transform & transform );


    inline void pushMatrix( vsg::dmat4 & matrix )
    {
        stack.push_back( matrix );
    }

    inline void popMatrix()
    {
        stack.pop_back();
    }

    void applyDrawable( vsg::Command * drawable );

protected:
    typedef std::vector< vsg::dmat4 >   MatrixStack;

    MatrixStack stack;
    BoundingCylinder bc;
    vsg::vec3 axis;
};


// vsgbCollision
}


// __VSGBCOLLISION_COMPUTECYLINDERVISITOR_H__
#endif
