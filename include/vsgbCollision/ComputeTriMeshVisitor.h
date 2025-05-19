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

#ifndef __VSGBCOLLISION_COMPUTETRIMESHVISITOR_H__
#define __VSGBCOLLISION_COMPUTETRIMESHVISITOR_H__ 1


#include <vsgbCollision/Export.h>
#include <vsg/all.h>

namespace vsgbCollision
{


/** \class ComputeTriMeshVisitor ComputeTriMeshVisitor.h <vsgbCollision/ComputeTriMeshVisitor.h>
\brief Creates a list of triangles from VSG data, suitable for constructing Bullet triangle mesh collision shapes.

*/
class VSGBCOLLISION_EXPORT ComputeTriMeshVisitor : public vsg::Inherit<vsg::Intersector, ComputeTriMeshVisitor>
{
public:
    /// create intersector for specified polytope.
    explicit ComputeTriMeshVisitor( vsg::ref_ptr<vsg::ArrayState> initialArrayData = {});

    /// create intersector for a polytope with window space dimensions, projected into world coords using the Camera's projection and view matrices.
   /* ComputeTriMeshVisitor(const Camera& camera, double xMin, double yMin, double xMax, double yMax, ref_ptr<ArrayState> initialArrayData = {});

    class VSG_DECLSPEC Intersection : public Inherit<Object, Intersection>
    {
    public:
        Intersection() {}
        Intersection(const dvec3& in_localIntersection, const dvec3& in_worldIntersection, const dmat4& in_localToWorld, const NodePath& in_nodePath, const DataList& in_arrays, const std::vector<uint32_t>& in_indexRatios, uint32_t in_instanceIndex);

        dvec3 localIntersection;
        dvec3 worldIntersection;

        dmat4 localToWorld;
        NodePath nodePath;
        DataList arrays;
        std::vector<uint32_t> indices;
        uint32_t instanceIndex = 0;

        // return true if Intersection is valid
        operator bool() const { return !nodePath.empty(); }
    };

    using Intersections = std::vector<ref_ptr<Intersection>>;
    Intersections intersections;

    ref_ptr<Intersection> add(const dvec3& coord, const std::vector<uint32_t>& indices, uint32_t instanceIndex);
    */
    void pushTransform(const vsg::Transform& transform) override;
    void popTransform() override;

    /// check for intersection with sphere
    bool intersects(const vsg::dsphere& bs) override;

    bool intersectDraw(uint32_t firstVertex, uint32_t vertexCount, uint32_t firstInstance, uint32_t instanceCount) override;
    bool intersectDrawIndexed(uint32_t firstIndex, uint32_t indexCount, uint32_t firstInstance, uint32_t instanceCount) override;
    vsg::vec3Array* getTriMesh()
    {
        return( mesh.get() );
    }
    vsg::ref_ptr< vsg::vec3Array > mesh;
};
/*
class VSGBCOLLISION_EXPORT ComputeTriMeshVisitor : public vsg::Visitor
{
public:
    ComputeTriMeshVisitor( );


    virtual void reset();

    vsg::vec3Array* getTriMesh()
    {
        return( mesh.get() );
    }

    void apply( vsg::Command & geode );

protected:
    void applyDrawable( vsg::Command * drawable );

    vsg::ref_ptr< vsg::vec3Array > mesh;
};

*/
// vsgbCollision
}


// __VSGBCOLLISION_COMPUTETRIMESHVISITOR_H__
#endif
