/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * vsgWorks is (C) Copyright 2025 by Julien Valentin
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

#include "vsg/nodes/StateGroup.h"
#include <vsgbCollision/GeometryModifier.h>
#include <vsgbCollision/GeometryOperation.h>

#include <vsg/nodes/VertexIndexDraw.h>
#include <ostream>

namespace vsgbCollision {

GeometryModifier::GeometryModifier(  )
    : vsg::Inherit<vsg::Visitor, GeometryModifier>()
{
    reset();
}
GeometryModifier::GeometryModifier( GeometryOperation* geomOp )
    : vsg::Inherit<vsg::Visitor, GeometryModifier>(),
    _geomOp( geomOp )
{
    reset();
}

void
GeometryModifier::reset()
{
    _drawableCount = _geometryCount = 0;
    _preVertices = _preIndices = _preTriangles = 0;
    _postVertices = _postIndices = _postTriangles = 0;
    _attemptDrawableMerge = 0;
}

GeometryModifier::~GeometryModifier()
{
}

void
GeometryModifier::apply( vsg::StateGroup& geode )
{
    std::vector<vsg::ref_ptr<vsg::Node>> nstg;
    for(auto geom:geode.children)
    {
        if(auto vi=geom->cast<vsg::VertexIndexDraw>() ){

            auto n=vsg::ref_ptr<vsg::VertexIndexDraw> ( (*_geomOp)( *vi ));
            vi->arrays[0]->data=n->arrays[0]->data;
            vi->indexCount=n->indexCount;
            vi->assignIndices(n->indices->data);
        }

        nstg.push_back(geom);
    }
    geode.children.clear();
    for(auto geom:nstg)
        geode.addChild(geom);
}/*
    // merge drawables if possible for best results
    if (getDrawableMerge())
    {
       // vsgUtil::Optimizer::MergeGeometryVisitor mgv;
       // mgv.setTargetMaximumNumberOfVertices(1000000);
       // mgv.mergeGeode(geode);
    }

 for(unsigned int i=0;i<geode.getNumDrawables();++i)
    {
        _drawableCount++;
        vsg::ref_ptr< vsg::Geometry > geometry = geode.getDrawable(i)->asGeometry();
        if( geometry.valid() )
        {
            _geometryCount++;
            if( geometry->containsSharedArrays() )
                vsg::notify( vsg::DEBUG_INFO ) << "Warning! Geometry contains shared arrays" << std::endl;

            // Get statistics before
            incStatistics( geometry.get(), _preVertices, _preIndices, _preTriangles );

            vsg::ref_ptr< vsg::Geometry > newGeom = (*_geomOp)( *geometry );
            geode.replaceDrawable( geometry.get(), newGeom.get() );

            // Get statistics after
            incStatistics( newGeom.get(), _postVertices, _postIndices, _postTriangles );
        }
    }
}
*/
void
GeometryModifier::incStatistics( const vsg::Geometry* geom, unsigned int& vert, unsigned int& ind, unsigned int& tris )
{
    /*vert += geom->getVertexArray()->getNumElements();

    unsigned int idx;
    for( idx=0; idx < geom->getNumPrimitiveSets(); idx++ )
    {
        const vsg::PrimitiveSet* ps( geom->getPrimitiveSet( idx ) );
        ind += ps->getNumIndices();

        switch( ps->getMode() )
        {
        case GL_TRIANGLES:
            tris += ps->getNumPrimitives();
            break;
        case GL_TRIANGLE_STRIP:
        case GL_QUAD_STRIP:
            tris += ps->getNumIndices() - 2;
            break;
        case GL_TRIANGLE_FAN:
        case GL_POLYGON:
            tris += ps->getNumIndices() - 1;
            break;
        case GL_QUADS:
            tris += ps->getNumPrimitives() * 2;
            break;
        default:
            break;
        }
    }*/
}

void
GeometryModifier::displayStatistics( std::ostream& ostr ) const
{
    ostr << "GeometryModifier statistics" << std::endl;
    ostr << "  GeometryOperation type: " << _geomOp->className() << std::endl;
    ostr << "  # Drawable: " << _drawableCount << ", # Geometry: " << _geometryCount << std::endl;
    ostr << "              Before\tAfter" << std::endl;
    ostr << "  Vertices:   " << _preVertices << "\t" << _postVertices << std::endl;
    ostr << "  Indices:    " << _preIndices << "\t" << _postIndices << std::endl;
    ostr << "  Triangles:  " << _preTriangles << "\t" << _postTriangles << std::endl;
}

}
