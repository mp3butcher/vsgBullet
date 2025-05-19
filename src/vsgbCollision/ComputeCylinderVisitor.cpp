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

#include <vsgbCollision/ComputeCylinderVisitor.h>
#include <vsgbCollision/BoundingCylinder.h>

#include <vsg/nodes/Transform.h>
#include <vsg/maths/vec2.h>

using namespace vsg;

namespace vsgbCollision
{


/* \cond */
struct ComputeCylinderBound : public vsg::Visitor
{
    ComputeCylinderBound()
    {
        _vertices2f = 0;
        _vertices3f = 0;
        _vertices4f = 0;
        _vertices2d = 0;
        _vertices3d = 0;
        _vertices4d = 0;
    }

    virtual void setAxis( const vsg::vec3 a )
    {
        _bb.setAxis( a );
    }

    virtual void setVertexArray( unsigned int,
                                 const vec2 * vertices )
    {
        _vertices2f = vertices;
    }

    virtual void setVertexArray( unsigned int,
                                 const vec3 * vertices )
    {
        _vertices3f = vertices;
    }

    virtual void setVertexArray( unsigned int,
                                 const vec4 * vertices )
    {
        _vertices4f = vertices;
    }

    virtual void setVertexArray( unsigned int,
                                 const dvec2 * vertices )
    {
        _vertices2d = vertices;
    }

    virtual void setVertexArray( unsigned int,
                                 const dvec3 * vertices )
    {
        _vertices3d = vertices;
    }

    virtual void setVertexArray( unsigned int,
                                 const dvec4 * vertices )
    {
        _vertices4d = vertices;
    }

    template< typename T >
    void _drawArrays( T * vert,
                      T * end )
    {
        for( ; vert < end; ++vert )
        {
            vertex( *vert );
        }
    }

    template< typename T, typename I >
    void _drawElements( T * vert,
                        I * indices,
                        I * end )
    {
        for( ; indices < end; ++indices )
        {
            vertex( vert[ *indices ] );
        }
    }

    virtual void drawArrays(
                                                                                                            uint first,
                                                                                                            uint count )
    {
        if( _vertices3f )
        {
            _drawArrays( _vertices3f + first, _vertices3f + ( first + count ) );
        }
        else if( _vertices2f )
        {
            _drawArrays( _vertices2f + first, _vertices2f + ( first + count ) );
        }
        else if( _vertices4f )
        {
            _drawArrays( _vertices4f + first, _vertices4f + ( first + count ) );
        }
        else if( _vertices2d )
        {
            _drawArrays( _vertices2d + first, _vertices2d + ( first + count ) );
        }
        else if( _vertices3d )
        {
            _drawArrays( _vertices3d + first, _vertices3d + ( first + count ) );
        }
        else if( _vertices4d )
        {
            _drawArrays( _vertices4d + first, _vertices4d + ( first + count ) );
        }
    }

    virtual void drawElements(
                                                                                                                                                                uint count,
                                                                                                                                                                const uint8_t * indices )
    {
        if( _vertices3f )
        {
            _drawElements( _vertices3f, indices, indices + count );
        }
        else if( _vertices2f )
        {
            _drawElements( _vertices2f, indices, indices + count );
        }
        else if( _vertices4f )
        {
            _drawElements( _vertices4f, indices, indices + count );
        }
        else if( _vertices2d )
        {
            _drawElements( _vertices2d, indices, indices + count );
        }
        else if( _vertices3d )
        {
            _drawElements( _vertices3d, indices, indices + count );
        }
        else if( _vertices4d )
        {
            _drawElements( _vertices4d, indices, indices + count );
        }
    }

    virtual void drawElements(  uint count, const ushort* indices )
    {
        if( _vertices3f )
        {
            _drawElements( _vertices3f, indices, indices + count );
        }
        else if( _vertices2f )
        {
            _drawElements( _vertices2f, indices, indices + count );
        }
        else if( _vertices4f )
        {
            _drawElements( _vertices4f, indices, indices + count );
        }
        else if( _vertices2d )
        {
            _drawElements( _vertices2d, indices, indices + count );
        }
        else if( _vertices3d )
        {
            _drawElements( _vertices3d, indices, indices + count );
        }
        else if( _vertices4d )
        {
            _drawElements( _vertices4d, indices, indices + count );
        }
    }

    virtual void drawElements(   uint count, const uint* indices )
    {
        if( _vertices3f )
        {
            _drawElements( _vertices3f, indices, indices + count );
        }
        else if( _vertices2f )
        {
            _drawElements( _vertices2f, indices, indices + count );
        }
        else if( _vertices4f )
        {
            _drawElements( _vertices4f, indices, indices + count );
        }
        else if( _vertices2d )
        {
            _drawElements( _vertices2d, indices, indices + count );
        }
        else if( _vertices3d )
        {
            _drawElements( _vertices3d, indices, indices + count );
        }
        else if( _vertices4d )
        {
            _drawElements( _vertices4d, indices, indices + count );
        }
    }

    virtual void vertex( const vec2 & vert )
    {
        _bb.expandBy( vsg::vec3( vert[ 0 ], vert[ 1 ], 0.0f ) );
    }

    virtual void vertex( const vec3 & vert )
    {
        _bb.expandBy( vert );
    }

    virtual void vertex( const vec4 & vert )
    {
        if( vert[ 3 ] != 0.0f )
        {
            _bb.expandBy( vsg::vec3( vert[ 0 ], vert[ 1 ], vert[ 2 ] ) / vert[ 3 ] );
        }
    }

    virtual void vertex( const dvec2 & vert )
    {
        _bb.expandBy( vsg::vec3( vert[ 0 ], vert[ 1 ], 0.0f ) );
    }

    virtual void vertex( const dvec3 & vert )
    {
        _bb.expandBy( vsg::vec3(vert[ 0 ], vert[ 1 ], 0.0f )  );
    }

    virtual void vertex( const dvec4 & vert )
    {
        if( vert[ 3 ] != 0.0f )
        {
            vec3 v( vert[ 0 ], vert[ 1 ], vert[ 2 ] );
            v/=vert[ 3 ];
            _bb.expandBy( v );
        }
    }

    virtual void vertex( float x,
                         float y )
    {
        _bb.expandBy( x, y, 1.0f );
    }

    virtual void vertex( float x,
                         float y,
                         float z )
    {
        _bb.expandBy( x, y, z );
    }

    virtual void vertex( float x,
                         float y,
                         float z,
                         float w )
    {
        if( w != 0.0f )
        {
            _bb.expandBy( x / w, y / w, z / w );
        }
    }

    virtual void vertex( double x,
                         double y )
    {
        _bb.expandBy( x, y, 1.0f );
    }

    virtual void vertex( double x,
                         double y,
                         double z )
    {
        _bb.expandBy( x, y, z );
    }

    virtual void vertex( double x,
                         double y,
                         double z,
                         double w )
    {
        if( w != 0.0f )
        {
            _bb.expandBy( x / w, y / w, z / w );
        }
    }

    virtual void end()
    {
    }

    const vsg::vec2 *      _vertices2f;
    const vec3 *      _vertices3f;
    const vec4 *      _vertices4f;
    const dvec2 *     _vertices2d;
    const dvec3 *     _vertices3d;
    const dvec4 *     _vertices4d;
    BoundingCylinder _bb;
};
/* \endcond */


ComputeCylinderVisitor::ComputeCylinderVisitor(  )
{
}

void ComputeCylinderVisitor::reset()
{
    stack.clear();
    bc.init();
    bc.setAxis( axis );
}

void ComputeCylinderVisitor::apply( vsg::Transform & transform )
{
    vsg::dmat4 matrix;

    if( !stack.empty() )
    {
        matrix = stack.back();
    }

    transform.transform(matrix); //prem

    pushMatrix( matrix );

    transform.traverse( *this );

    popMatrix();
}

/*
void ComputeCylinderVisitor::applyDrawable( vsg::VertexIndexDraw * drawable )
{
    ComputeCylinderBound cbc;

    cbc.setAxis( axis );
    drawable->accept( cbc );

    if( stack.empty() )
    {
        bc.expandBy( cbc._bb );
    }
    else
    {
        BoundingCylinder newbc;
        vsg::mat4 & matrix = stack.back();
        newbc.setAxis( vsg::mat4::transform3x3( cbc._bb.getAxis(), matrix ) );
        newbc.setLength( cbc._bb.getLength() );
        newbc.setRadius( cbc._bb.getRadius() );
        bc.expandBy( newbc );
    }
}
*/

// vsgbCollision
}
