/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * osgWorks is (C) Copyright 2025 by Julien Valentin
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

#ifndef __REDUCER_OP_H__
#define __REDUCER_OP_H__


#include <vsgbCollision/Export.h>
#include <vsgbCollision/GeometryOperation.h>

#include <vsg/utils/Intersector.h>

namespace vsgbCollision {


/** \brief Reduces geometry by removing vertices without adding new vertices.

For more information, see \ref geomopt

Reducer algorithm:

\code
Consider each vertex
  Get a list of all triangles that share that vertex.
  Use the group threshold to break that list of triangles into
        possibly multiple groups of triangles.

  If the vertex is completely contained within a group, remove it.
  Else if the vertex is not completely contained, it is an edge vertex.

  Handle edge vertex:
  (Repeat this for each group that shares the vertex)
    Idenify the two triangle edges that share the vertex and are on the same edge as the vertex.
    If the two edges subtend an angle less than maxEdgeError, remove the vertex.

  Handle removing a vertex:
    Retessellate the remaining vertices by sorting the group's triangles
          into an order that preserves the normals, then treat it as an OpenGL
          polygon for purposes of triangulation.
    Change the map entries for all vertices of affected triangles.
    Add the removed vertex to a delete list.

  Cleanup:
    Erase (from the map) all vertices on the delete list.
    Delete the vertices and associaed data from the Geometry.
    Create a new TRIANGLES DEUI to replace the existing PrimitiveSet.
\endcode

<b>Work To Be Done</b>

Assume a vertex V is shared by multiple triangles, but due to the group threshold,
the triangles are broken into multiple Groups. The edge threshold allows it to be
removed from one group, but not from the remaining groups. This can cause incorrect
holes in the output. This can be seen with the cow model with default reduction
settings.

*/
class VSGBCOLLISION_EXPORT ReducerOp : public  vsg::Inherit<GeometryOperation, ReducerOp>
{
public:
    ReducerOp();
    ReducerOp( const ReducerOp& rhs, const vsg::CopyOp& copyOp );


    virtual vsg::VertexIndexDraw* operator()( vsg::VertexIndexDraw& geom );

    /** \brief Specify the group threshold in degrees. Default is 10.
    Larger values result in greater geometry reduction. */
    void setGroupThreshold( float groupThreshold );
    float getGroupThreshold() const { return( _groupThreshold ); }

    /** \brief Specify the maximum edge error in degrees. Default is 10.
    Larger values result in greater geometry reduction. */
    void setMaxEdgeError( float maxEdgeError );
    float getMaxEdgeError() const { return( _maxEdgeError ); }

    /** \brief Default is true */
    void setRemoveDegenerateAndRedundantTriangles( bool remove ) { _removeDegenerateAndRedundantTriangles = remove; }
    bool getRemoveDegenerateAndRedundantTriangles() const { return( _removeDegenerateAndRedundantTriangles ); }

protected:
    ~ReducerOp();

    float _groupThreshold;
    float _groupThresholdRad;
    float _maxEdgeError;
    float _maxEdgeErrorRad;
    bool _removeDegenerateAndRedundantTriangles;


    /* \cond */
    struct Tri
    {
        Tri( int v0, int v1, int v2, const vsg::vec3Array* verts=nullptr )
          : _v0(v0), _v1(v1), _v2(v2)
        {
            if( verts != nullptr )
            {
                vsg::vec3 u( (*verts)[ v1 ] - (*verts)[ v0 ] );
                vsg::vec3 v( (*verts)[ v2 ] - (*verts)[ v0 ] );
                _norm = cross(u , v);
                _norm=normalize(_norm);
            }
        }

        bool operator==( const Tri& rhs ) const
        {
            if( _v0 == rhs._v0 )
                return( (_v1 == rhs._v1) && (_v2 == rhs._v2) );
            if( _v0 == rhs._v1 )
                return( (_v1 == rhs._v2) && (_v2 == rhs._v0) );
            if( _v0 == rhs._v2 )
                return( (_v1 == rhs._v0) && (_v2 == rhs._v1) );
            else
                return( false );
        }

        unsigned int _v0, _v1, _v2;
        vsg::vec3 _norm;
    }; /* \endcond */
    typedef std::vector< Tri > TriList;
    typedef std::vector< TriList > TriListList;
    typedef std::map< unsigned int, TriList > VertToTriMap;

    /* \cond */
    struct Edge
    {
        Edge( int a, int b, bool sort=true )
        {
            _a = a;
            _b = b;
            if( sort && (a>b) )
            {
                // Sort indices so that (a,b) and (b,a) edges will be identical.
                _b = a;
                _a = b;
            }
        }

        bool operator<( const Edge& rhs ) const
        {
            if( _a < rhs._a )
                return( true );
            else if( _a == rhs._a )
                return( _b < rhs._b );
            else
                return( false );
        }

        unsigned int _a, _b;
    }; /* \endcond */
    typedef std::vector< Edge > EdgeList;
    typedef std::map< Edge, TriList > EdgeToTriMap;

    typedef std::vector< unsigned int > IndexList;

    bool convertToDEUITriangles( vsg::VertexIndexDraw* geom );
    bool makeMap( VertToTriMap& v2t, const vsg::VertexIndexDraw& geom );
    void makeGroups( TriListList& tll, const TriList& tl );
    EdgeList findBoundaryEdges( const TriList& tl, unsigned int vertIdx );
    bool removeableEdge( const EdgeList& el, const vsg::vec3Array* verts );
    void orderVerts( unsigned int removeIdx, const TriList& tl, IndexList& idxList );
    void removeTri( const Tri& tri, TriList& tl );
    bool removeableVertex( unsigned int removeIdx, const TriList& tl, vsg::vec3Array* verts );
    void deleteVertex( unsigned int removeIdx, const TriList& tl, VertToTriMap& v2t, vsg::vec3Array* verts );

    void reduce( vsg::VertexIndexDraw& geom );
};

class VSGBCOLLISION_EXPORT ReducerVisitor : public vsg::Inherit<vsg::Intersector, ReducerVisitor>
{
public:
    /// create intersector for specified polytope.
    explicit ReducerVisitor( vsg::ref_ptr<vsg::ArrayState> initialArrayData = {});

    void pushTransform(const vsg::Transform& transform) override;
    void popTransform() override;

    /// check for intersection with sphere
    bool intersects(const vsg::dsphere& bs) override;

    bool intersectDraw(uint32_t firstVertex, uint32_t vertexCount, uint32_t firstInstance, uint32_t instanceCount) override;
    bool intersectDrawIndexed(uint32_t firstIndex, uint32_t indexCount, uint32_t firstInstance, uint32_t instanceCount) override;

    vsg::ref_ptr< vsg::uintArray > indices;
};
}

#endif
