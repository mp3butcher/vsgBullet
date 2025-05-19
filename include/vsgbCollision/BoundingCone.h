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

#ifndef __VSGBCOLLISION_BOUNDINGCONE_H__
#define __VSGBCOLLISION_BOUNDINGCONE_H__ 1


#include <vsgbCollision/Export.h>
#include <vsg/maths/vec3.h>


namespace vsgbCollision
{


/** \class BoundingCone BoundingCone.h <vsgbCollision/BoundingCone.h>
\brief Used internally to store cone parameters.

TBD Not currently used. when we do support cones, consider not encapsulating parameters in a class. */
class VSGBCOLLISION_EXPORT BoundingCone
{
public:
    BoundingCone( void );
    virtual ~BoundingCone( void );

    void init()
    {
        length = radius = 0.0f;
    }

    void setAxis( const vsg::vec3 & a )
    {
        axis = normalize(a);
    }
    const vsg::vec3 & getAxis() const
    {
        return( axis );
    }

    void setRadius( float r )
    {
        radius = r;
    }
    float getRadius() const
    {
        return( radius );
    }

    void setLength( float l )
    {
        length = l;
    }
    float getLength() const
    {
        return( length );
    }

    void expandBy( const vsg::vec3& v );


    void expandBy( float x,
                   float y,
                   float z );

    void expandBy( const BoundingCone& bc );

protected:
    float length;
    float radius;
    vsg::vec3 axis;
};


// vsgbCollision
}


// __VSGBCOLLISION_BOUNDINGCONE_H__
#endif
