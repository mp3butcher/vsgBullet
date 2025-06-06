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

#ifndef __VSGBCOLLISION_COLLECT_VERTICES_VISITOR_H__
#define __VSGBCOLLISION_COLLECT_VERTICES_VISITOR_H__ 1

#include <vsgbCollision/Export.h>
#include <vsg/core/Visitor.h>
#include <vsg/nodes/Geometry.h>

namespace vsgbCollision
{


/** \class CollectVerticesVisitor CollectVerticesVisitor.h <vsgbCollision/CollectVerticesVisitor.h>
\brief A NodeVisitor to collect a set of transformed vertices.

*/
class VSGBCOLLISION_EXPORT CollectVerticesVisitor : public vsg::Inherit<vsg::ConstVisitor, CollectVerticesVisitor>
{
public:
    CollectVerticesVisitor( );

    virtual void reset();

    std::vector<vsg::vec3>& getVertices()
    {
        return( verts_ );
    }
   /** \brief Builds CollectVerticesVisitor::_localNodePath (a NodePath) from all Transforms,
    excluding AbsoluteTransform.

    This visitor saves the transformed (world space) vertices from the scene graph.
    However, in order to be compatible with the
    \link rigidbody rigid body creation utilities, \endlink the visitor can't consider
    AbsoluteTransforms in such a transformation, as they ignore all parent transforms.

    To support this, we override Visitor::apply(vsg::Transform&) to build our own
    NodePath (CollectVerticesVisitor::_localNodePath) that contains all Transform nodes encountered during traversal
    except AbsoluteTransform nodes. */
    virtual void apply(const vsg::Transform& node ) override;
    virtual void apply(const vsg::Group& node ) override;
    virtual void apply(const vsg::Geometry& node ) override;
    void apply(const vsg::Node& node) override;
    void apply(const vsg::StateGroup& stategroup) override;
   // void apply(const vsg::LOD& lod) override;
   // void apply(const vsg::PagedLOD& plod) override;
   // void apply(const vsg::CullNode& cn) override;
   // void apply(const vsg::CullGroup& cn) override;
    //void apply(const vsg::DepthSorted& cn) override;

    //void apply(const vsg::VertexDraw& vid) override;
    void apply(const vsg::VertexIndexDraw& vid) override;

protected:

    std::vector<vsg::vec3> verts_;

    /** NodePath containing only Transform nodes, but excluding AbsoluteTransform. */
    std::vector< vsg::ref_ptr<const vsg::Transform > > _localNodePath;
};


// vsgbCollision
}


// __VSGBCOLLISION_COLLECT_VERTICES_VISITOR_H__
#endif
