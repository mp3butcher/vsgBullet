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

#include <vsgbCollision/VertexAggOp.h>
#include <vsgbCollision/CollisionShapes.h>
#include "BulletCollision/CollisionShapes/btShapeHull.h"
#include <btBulletCollisionCommon.h>
#include <vsgbCollision/Utils.h>
#include <vsg/nodes/Geometry.h>

namespace vsgbCollision
{


typedef std::vector< vsg::ref_ptr< Octree > > OctreeVec;
struct Octree : public vsg::Object
{
    Octree() {}

    vsg::dbox _bb;
    std::vector<vsg::ref_ptr<vsg::vec3Array> > _verts;

    OctreeVec _children;
};



VertexAggOp::VertexAggOp()
  : _maxVertsPerCell( 50 ),
    _minCellSize( vsg::vec3( 0., 0., 0. ) ),
    _useMinCellSize( false ),
    _createHull( true ),
    _psm( VertexAggOp::GEOMETRIC_MEAN )
{
}
VertexAggOp::VertexAggOp( const VertexAggOp& rhs, const vsg::CopyOp& copyOp )
{
}
VertexAggOp::~VertexAggOp()
{
}

vsg::Geometry*
VertexAggOp::operator()( vsg::Geometry& geom )
{
   /* vsg::Array* vArray = geom.getVertexArray();

    vsg::vec3Array* verts = dynamic_cast< vsg::vec3Array* >( vArray );
    if( verts != nullptr )
    {
        vsg::ref_ptr< Octree > oct = new Octree;
        {
            vsg::BoundingBox& bb = oct->_bb;
            for( unsigned int jdx=0; jdx<verts->size(); jdx++ )
                bb.expandBy( (*verts)[ jdx ] );
        }
        oct->_verts = verts;
        recurseBuild( oct.get() );

        vsg::vec3Array* newV = new vsg::vec3Array;
        gatherVerts( oct.get(), newV );

        if( _createHull )
        {
            geom.setVertexArray( newV );
            createHull( geom );
        }
        else
        {
            geom.setVertexArray( newV );
            geom.removePrimitiveSet( 0, geom.getNumPrimitiveSets() );
            geom.addPrimitiveSet( new vsg::DrawArrays( GL_POINTS, 0, newV->size() ) );
        }
    }
*/
    return( &geom );
}

void
VertexAggOp::createHull( vsg::Geometry& geom )
{
   /* vsg::vec3Array* oldV = dynamic_cast< vsg::vec3Array* >( geom.getVertexArray() );
    if( !oldV )
    {
        std::cerr << "VertexAggOp: Can't create convex hull." << std::endl;
        return;
    }
    btConvexHullShape* chs = new btConvexHullShape;
    vsg::vec3Array::const_iterator itr;
    for( itr = oldV->begin(); itr != oldV->end(); ++itr )
        chs->addPoint( vsgbCollision::asBtVector3( *itr ) );

    vsg::ref_ptr< vsg::Node > n = vsgbCollision::vsgNodeFromBtCollisionShape( chs );
    vsg::Geode* newGeode = dynamic_cast< vsg::Geode* >( n.get() );
    
    if( newGeode == nullptr )
    {
        std::cerr << "Got nullptr geode from vsgNodeFromBtCollisionShape" << std::endl;
        return;
    }
    
    vsg::Drawable* newDraw = newGeode->getDrawable( 0 );
    vsg::Geometry* newGeom = dynamic_cast< vsg::Geometry* >( newDraw );
    
    if( newGeom == nullptr )
    {
        std::cerr << "Got nullptr geometry from vsgNodeFromBtCollisionShape" << std::endl;
        return;
    }

    geom.setVertexArray( newGeom->getVertexArray() );
    geom.setColorArray( newGeom->getColorArray() );
    geom.setColorBinding( newGeom->getColorBinding() );
    geom.removePrimitiveSet( 0, geom.getNumPrimitiveSets() );
    geom.addPrimitiveSet( newGeom->getPrimitiveSet( 0 ) );*/
}

void
VertexAggOp::recurseBuild( Octree* cell ) const
{
  /*  vsg::vec3Array* verts = cell->_verts.get();
    if( verts->size() <= _maxVertsPerCell )
        return;

    const vsg::vec3 center( cell->_bb.center() );
    const vsg::vec3 cellMax( cell->_bb._max );
    const unsigned int posX( 1<<0 );
    const unsigned int posY( 1<<1 );
    const unsigned int posZ( 1<<2 );

    cell->_children.resize( 8 );
    unsigned int idx;
    for( idx=0; idx<8; idx++ )
    {
        Octree* oct = new Octree;
        cell->_children[ idx ] = oct;

        vsg::vec3 cMin = cell->_bb._min;
        vsg::vec3 cMax = center;
        if( idx & posX )
        {
            cMin.x() = center.x();
            cMax.x() = cellMax.x();
        }
        if( idx & posY )
        {
            cMin.y() = center.y();
            cMax.y() = cellMax.y();
        }
        if( idx & posZ )
        {
            cMin.z() = center.z();
            cMax.z() = cellMax.z();
        }
        oct->_bb.set( cMin, cMax );
        oct->_verts = new vsg::vec3Array;
    }
    for( idx=0; idx<verts->size(); idx++ )
    {
        vsg::vec3& v = (*verts)[ idx ];
        unsigned int childIdx( 0 );
        if( v.x() > center.x() )
            childIdx |= posX;
        if( v.y() > center.y() )
            childIdx |= posY;
        if( v.z() > center.z() )
            childIdx |= posZ;
        cell->_children[ childIdx ]->_verts->push_back( v );
    }

    verts->clear();

    for( idx=0; idx<8; idx++ )
        recurseBuild( cell->_children[ idx ].get() );*/
}

void
VertexAggOp::gatherVerts( Octree* cell, std::vector<vsg::ref_ptr<vsg::vec3Array> > *verts ) const
{
  /*  if( cell->_verts->size() > 0 )
        verts->push_back( representativeVert( cell->_verts.get() ) );
    else if( cell->_children.size() > 0 )
    {
        int idx;
        for( idx=0; idx<8; idx++ )
        {
            Octree* child = cell->_children[ idx ].get();
            if( child != nullptr )
                gatherVerts( child, verts );
        }
    }*/
}

vsg::vec3
VertexAggOp::representativeVert(std::vector<vsg::ref_ptr<vsg::vec3Array> >* verts ) const
{
   vsg::vec3 rep( 0., 0., 0. );
/*
    if( _psm == GEOMETRIC_MEAN )
    {
        unsigned int idx;
        for( idx=0; idx<verts->size(); idx++ )
            rep += (*verts)[ idx ];
        rep /= verts->size();
    }
    else if( _psm == BOUNDING_BOX_CENTER )
    {
        vsg::BoundingBox bb;
        unsigned int idx;
        for( idx = 0; idx < verts->size(); idx++ )
            bb.expandBy( (*verts)[ idx ] );
        rep = bb.center();
    }
*/
    return( rep );
}


// vsgbCollision
}
