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

#include <vsgbCollision/BoundingCylinder.h>


namespace vsgbCollision
{


BoundingCylinder::BoundingCylinder( void )
{
    axis = vsg::vec3( 0, 1, 0 );
    length = 0.0;
    radius = 0.0;
}

BoundingCylinder::~BoundingCylinder( void )
{
}

void BoundingCylinder::expandBy( const vsg::vec3& v )
{
    float nl, nr;

    nl = dot( v ,axis );
    nl = nl<0?-nl:nl;
    if( nl > length )
    {
        length = nl;
    }

    nr = sqrtf( length2(v) - nl * nl );
    if( nr > radius )
    {
        radius = nr;
    }
}

void BoundingCylinder::expandBy( float x,
                                 float y,
                                 float z )
{
    expandBy( vsg::vec3( x, y, z ) );
}

void BoundingCylinder::expandBy( const BoundingCylinder& bc )
{
    float a, b;

    a = vsg::dot( bc.getAxis() , axis );
    a = a<0?-a:a;
    b = sqrtf( 1 - a * a );

    float nl = a * bc.getLength() + b * bc.getRadius();
    float nr = sqrtf( b * b * bc.getLength() * bc.getLength() + bc.getRadius() * bc.getRadius() );

    if( nl > length )
    {
        length = nl;
    }
    if( nr > radius )
    {
        radius = nr;
    }

    return;
}


// vsgbCollision
}
