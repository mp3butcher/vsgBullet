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

#include <vsgbCollision/ComputeTriMeshVisitor.h>
#include <vsg/all.h>
#include <vsg/nodes/Transform.h>
#include <vsg/utils/PrimitiveFunctor.h>



#include <iostream>
#include <vector>

using namespace vsg;


namespace vsgbCollision
{


/* \cond */
struct ComputeTriMeshFunc
{
    ArrayState& arrayState;
    ref_ptr<const vec3Array> sourceVertices;
    uint32_t instanceIndex = 0;

    ComputeTriMeshFunc( ArrayState& in_arrayState) :
        arrayState(in_arrayState)
    {
    }

    bool instance(uint32_t index)
    {
        sourceVertices = arrayState.vertexArray(index);
        instanceIndex = index;
        return sourceVertices.valid();
    }

    void point(uint32_t i0){}
    void line(uint32_t i0, uint32_t i1){}
    void triangle(uint32_t i0, uint32_t i1, uint32_t i2)
    {
        vertices.push_back(sourceVertices->at(i0));
        vertices.push_back(sourceVertices->at(i1));
        vertices.push_back(sourceVertices->at(i2));
    }


    std::vector< vsg::vec3 > vertices;
};
/* \endcond */


ComputeTriMeshVisitor::ComputeTriMeshVisitor(vsg::ref_ptr<vsg::ArrayState> initialArrayData)
    //vsg::Inherit
{
    mesh = new vsg::vec3Array;

    localToWorldStack().push_back(vsg::dmat4());
    worldToLocalStack().push_back(vsg::dmat4());
}
/*
void ComputeTriMeshVisitor::reset()
{
    mesh->clear();
}
*/

void ComputeTriMeshVisitor::pushTransform(const Transform& transform)
{
    auto& l2wStack = localToWorldStack();
    auto& w2lStack = worldToLocalStack();

    dmat4 localToWorld = l2wStack.empty() ? transform.transform(dmat4{}) : transform.transform(l2wStack.back());
    dmat4 worldToLocal = inverse(localToWorld);

    l2wStack.push_back(localToWorld);
    w2lStack.push_back(worldToLocal);

   // const auto& worldLineSegment = _lineSegmentStack.front();
   // _lineSegmentStack.push_back(LineSegment{worldToLocal * worldLineSegment.start, worldToLocal * worldLineSegment.end});
}

void ComputeTriMeshVisitor::popTransform()
{
    //_lineSegmentStack.pop_back();
    localToWorldStack().pop_back();
    worldToLocalStack().pop_back();
}

bool ComputeTriMeshVisitor::intersects(const dsphere& bs)
{
    //debug("intersects( center = ", bs.center, ", radius = ", bs.radius, ")");
    //if (!bs.valid())
        return true;


}
bool ComputeTriMeshVisitor::intersectDraw(uint32_t firstVertex, uint32_t vertexCount, uint32_t firstInstance, uint32_t instanceCount)
{
    auto& arrayState = *arrayStateStack.back();

    vsg::PrimitiveFunctor<ComputeTriMeshFunc> printPrimitives(arrayState);
    printPrimitives.draw(arrayState.topology, firstVertex, vertexCount, firstInstance, instanceCount);

    return true;
}

bool ComputeTriMeshVisitor::intersectDrawIndexed(uint32_t firstIndex, uint32_t indexCount, uint32_t firstInstance, uint32_t instanceCount)
{

    auto& arrayState = *arrayStateStack.back();
    ;
    vsg::PrimitiveFunctor<ComputeTriMeshFunc> printPrimtives(arrayState);
    if (ushort_indices)
        printPrimtives.drawIndexed(arrayState.topology, ushort_indices, firstIndex, indexCount, firstInstance, instanceCount);
    else if (uint_indices)
        printPrimtives.drawIndexed(arrayState.topology, uint_indices, firstIndex, indexCount, firstInstance, instanceCount);

    mesh=vsg::vec3Array::create(printPrimtives.vertices.size());
    uint cpt(0);
    for(auto iter = printPrimtives.vertices.begin(); iter != printPrimtives.vertices.end(); ++iter )
    {
        mesh->at(cpt++) = vsg::mat4( arrayState.localToWorldStack.back())*(*iter) ;
    }
    return true;
}
/*
void ComputeTriMeshVisitor::apply( vsg::Geode & geode )
{
    unsigned int idx;
    for( idx = 0; idx < geode.getNumDrawables(); idx++ )
        applyDrawable( geode.getDrawable( idx ) );
}

void ComputeTriMeshVisitor::applyDrawable( vsg::VertexIndexDraw * drawable )
{
   // vsg::Geometry::topo
    vsg::PrimitiveFunctor< ComputeTriMeshFunc > functor;
   functor.drawIndexed(arrayStateStack.back()->topology,drawable->indices->data<vsg::ubvec2Array.cast())
    drawable->accept( functor );

    vsg::mat4 m = vsg::computeLocalToWorld( getNodePath() );
    vsg::vec3Array::iterator iter;
    for( iter = functor.vertices->begin(); iter != functor.vertices->end(); ++iter )
    {
        mesh->push_back( *iter * m );
    }
}
*/

// vsgbCollision
}
