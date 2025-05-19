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

#include <vsgbCollision/CollectVerticesVisitor.h>
 #include <vsg/nodes/AbsoluteTransform.h>
#include <vsg/nodes/Geometry.h>
#include <vsg/all.h>

#include <iostream>

using namespace vsg;


namespace vsgbCollision
{


CollectVerticesVisitor::CollectVerticesVisitor(   )
{

    reset();
}

void CollectVerticesVisitor::reset()
{
    //verts_->clear();
}

void CollectVerticesVisitor::apply(const vsg::Transform& node )
{
    // Override apply(Transform&) to avoid processing AMT nodes.
    const bool nonAMT = ( dynamic_cast<const vsg::AbsoluteTransform* >( (const vsg::Transform*) (& node) ) == nullptr );
    if( nonAMT )
        _localNodePath.push_back( vsg::ref_ptr<const vsg::Transform>( (const vsg::Transform*) (& node)) );

    node.traverse(*this);

    if( nonAMT )
        _localNodePath.pop_back();
}

void CollectVerticesVisitor::apply(const vsg::Node& node )
{
    node.traverse(*this);// node.t_traverse(node,*this);
}
void CollectVerticesVisitor::apply(const vsg::StateGroup& node )
{
    node.traverse(*this);
}

void CollectVerticesVisitor::apply(const vsg::Group& node )
{
    node.traverse(*this);
}


void CollectVerticesVisitor::apply(const vsg::Geometry& geom )
{


    vsg::ref_ptr<vec3Array> in =( geom.arrays.at(0).cast<vsg::vec3Array>());
    if( in == nullptr )
    {
        std::cerr<< "CollectVerticesVisitor: Non-vec3Array vertex array encountered." << std::endl;
        return;
    }

    dmat4 m;
    for(auto it=_localNodePath.begin(); it!=_localNodePath.end(); ++it)
        m = (*it)->transform(m);

}
void CollectVerticesVisitor::apply(const vsg::VertexIndexDraw& cmd )
{
    dmat4 m;
    for(auto it=_localNodePath.begin(); it!=_localNodePath.end(); ++it)
        m = (*it)->transform(m);
    mat4 mf(m);
    vsg::ref_ptr<vsg::vec3Array> verts = cmd.arrays[0]->data.cast<vsg::vec3Array>();
    for(uint i=0; i<verts->size(); ++i)
        verts_.push_back(mf*verts->at(i));

}

// vsgbCollision
}
