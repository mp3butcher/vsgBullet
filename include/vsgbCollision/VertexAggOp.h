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

#ifndef __VSGBCOLLISION_VERTEX_AGG_OP_H__
#define __VSGBCOLLISION_VERTEX_AGG_OP_H__ 1


#include <vsgbCollision/Export.h>
#include <vsgbCollision/GeometryOperation.h>
#include <vsg/core/Object.h>
#include <vsg/maths/vec3.h>


namespace vsgbCollision
{

// Forward
struct Octree;


/** \class VertexAggOp VertexAggOp.h <vsgbCollision/VertexAggOp.h>
\brief A geometry reduction method that produces a convex hull, suitable for
low-resolution collision shapes.

*/
class VSGBCOLLISION_EXPORT VertexAggOp : public vsg::Inherit<vsgbCollision::GeometryOperation ,VertexAggOp>
{
public:
    VertexAggOp();
    VertexAggOp( const VertexAggOp& rhs, const vsg::CopyOp& copyOp={} );

    virtual vsg::Geometry* operator()( vsg::Geometry& geom );

    void setMaxVertsPerCell( unsigned int n ) { _maxVertsPerCell = n; }
    unsigned int getMaxVertsPerCell() const { return( _maxVertsPerCell ); }

    void setMinCellSize( vsg::vec3 v ) { _minCellSize = v; _useMinCellSize = true; }
    const vsg::vec3& getMinCellSize() const { return( _minCellSize ); }
    void setUseMinCellSize( bool use ) { _useMinCellSize = use; }
    bool getUseMinCellSize() const { return( _useMinCellSize ); }

    void setCreateHullPerGeometry( bool createHull ) { _createHull = createHull; }
    bool setCreateHullPerGeometry() const { return( _createHull ); }

    typedef enum {
        GEOMETRIC_MEAN,
        BOUNDING_BOX_CENTER
    } PointSelectionMethod;

    void setPointSelectionMethod( PointSelectionMethod psm ) { _psm = psm; }
    PointSelectionMethod getPointSelectionMethod() const { return( _psm ); }

protected:
    ~VertexAggOp();

    void recurseBuild( Octree* cell ) const;
    void gatherVerts( Octree* cell, std::vector<vsg::ref_ptr<vsg::vec3Array> >* verts ) const;
    vsg::vec3 representativeVert( std::vector<vsg::ref_ptr<vsg::vec3Array> >* verts ) const;
    void createHull( vsg::Geometry& geom );

    unsigned int _maxVertsPerCell;
    vsg::vec3 _minCellSize;
    bool _useMinCellSize;
    bool _createHull;
    PointSelectionMethod _psm;
};


// vsgbCollision
}


// __VSGBCOLLISION_VERTEX_AGG_OP_H__
#endif
