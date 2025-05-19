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

#include "vsg/nodes/StateGroup.h"
#include <vsgbCollision/ComputeShapeVisitor.h>
#include <vsg/utils/ComputeBounds.h>
#include <vsgbCollision/Utils.h>
#include <vsgbCollision/ReducerOp.h>
#include <vsgbCollision/GeometryModifier.h>
#include <iostream>
#include <vsg/nodes/AbsoluteTransform.h>

using namespace vsg;

namespace vsgbCollision
{


ComputeShapeVisitor::ComputeShapeVisitor(
    const BroadphaseNativeTypes shapeType,
    const vsgbCollision::AXIS axis, const unsigned int reductionLevel )
    :
    _shapeType( shapeType ),
    _axis( axis ),
    _reductionLevel( reductionLevel ),
    _shape( new btCompoundShape() )
{
}

void ComputeShapeVisitor::apply( vsg::Group& node )
{
    // If this is the root node, the bounding sphere will be invalid. Compute it.
    if( !( _bs.valid() ) )
    {
        vsg::ComputeBounds cb;
        node.accept(cb);
        _bs =  cb.bounds; //node.getbounds
    }


    node.traverse( *this );
}
void ComputeShapeVisitor::apply( vsg::Node& node )
{
    // If this is the root node, the bounding sphere will be invalid. Compute it.
    if( !( _bs.valid() ) )
    {
        vsg::ComputeBounds cb;
        node.accept(cb);
        _bs =  cb.bounds; //node.getbounds
    }

    node.traverse( *this );
}
void ComputeShapeVisitor::apply( vsg::Transform& node )
{
    // If this is the root node, the bounding sphere will be invalid. Compute it.

    if( !( _bs.valid() ) )
    {
        vsg::ComputeBounds cb;
        node.accept(cb);
        _bs =  cb.bounds; //node.getbounds
    }

    /* Override apply(Transform&) to avoid processing AMT nodes. */
    const bool nonAMT = ( dynamic_cast< vsg::AbsoluteTransform* >( &node ) == nullptr );
    dmat4 d;
    mat4 f;
    d=node.transform(d);
    f=d;
    if( nonAMT )
        _localNodePath.push_back( f);

    node.traverse( *this );

    if( nonAMT )
        _localNodePath.pop_back();
}

void ComputeShapeVisitor::apply( vsg::StateGroup& node )
{
    if( !( _bs.valid() ) )
    {
        vsg::ComputeBounds cb;
        node.accept(cb);
        _bs =  cb.bounds; //node.getbounds
    }

    mat4 f;
    for(auto it=_localNodePath.begin();it!=_localNodePath.end();++it)
        f = f * (*it);
    createAndAddShape( node, f );
}

btCollisionShape* ComputeShapeVisitor::getShape()
{
    return( _shape );
}
const btCollisionShape* ComputeShapeVisitor::getShape() const
{
    return( _shape );
}

void ComputeShapeVisitor::createAndAddShape( vsg::StateGroup& node, const vsg::mat4& m )
{
    std::cerr << "In createAndAddShape" << std::endl;

    btCollisionShape* child = createShape( node, m );
    if( child )
    {
        btCompoundShape* master = static_cast< btCompoundShape* >( _shape );
        btTransform transform; transform.setIdentity();
        master->addChildShape( transform, child );
    }
}


btCollisionShape* ComputeShapeVisitor::createShape( vsg::StateGroup& node, const vsg::mat4& m )
{
    std::cerr << "In createShape" << std::endl;

    // Make a copy of the incoming node and its data. The copy witll be transformed by the
    // specified matrix, and possibly geometry reduced.

    vsg::CopyOp op;
    auto geodeCopy=node.clone(op).cast<vsg::StateGroup>();
    geodeCopy->children.clear();
    for(auto nodestate: node.children )
    {
        if(auto vi=nodestate->cast<vsg::VertexIndexDraw>() )
        {
            geodeCopy->addChild(vsg::VertexIndexDraw::create(*vi,op));

        }else geodeCopy->addChild(vsg::ref_ptr<vsg::Node>(nodestate->clone(op)->cast<vsg::Node>()));
    }
    /**/
    //flatten transform
    for (auto nodestate: geodeCopy->children)
    {
        if(auto vi=nodestate->cast<vsg::VertexIndexDraw>() )
        {
            //copy verts
            auto verts = vsg::ref_ptr<vsg::vec3Array>(vi->arrays[0]->data->clone(op).cast<vsg::vec3Array>());
            vi->arrays[0]= vsg::BufferInfo::create(verts);
            auto ind=vi->indices->data->clone(op);
            auto uindex=vsg::ref_ptr<vsg::uintArray>(ind.cast<vsg::uintArray>());
            if(uindex)vi->assignIndices(uindex);
            auto sindex=vsg::ref_ptr<vsg::ushortArray>(ind.cast<vsg::ushortArray>());
            if(sindex)vi->assignIndices(sindex);
            auto bindex=vsg::ref_ptr<vsg::ubyteArray>(ind.cast<vsg::ubyteArray>());
            if(bindex)vi->assignIndices(bindex);

            for(auto it=verts->begin();it!=verts->end();++it)
                (*it)= m*(*it);
            verts->dirty();
        }
    }

    btCollisionShape* collision( nullptr );
    vsg::vec3 center;

    switch( _shapeType )
    {
    case BOX_SHAPE_PROXYTYPE:
    {
        vsg::ComputeBounds cbv;
        geodeCopy->accept( cbv );
        vsg::dbox bb = cbv.bounds;
        vsg::warn(bb.max , bb.min);
        center = (bb.max + bb.min) * 0.5;
        collision = vsgbCollision::btBoxCollisionShapeFromVSG( geodeCopy, &bb );
        break;
    }
    case SPHERE_SHAPE_PROXYTYPE:
    {
        vsg::ComputeBounds cbv;
        geodeCopy->accept( cbv );
        vsg::dbox bb = cbv.bounds;
        center = (bb.max + bb.min) * 0.5;
        collision = vsgbCollision::btSphereCollisionShapeFromVSG( geodeCopy );
        break;
    }
    case CYLINDER_SHAPE_PROXYTYPE:
    {
        vsg::ComputeBounds cbv;
        geodeCopy->accept( cbv );
        vsg::dbox bb = cbv.bounds;
        center = (bb.max + bb.min) * 0.5;
        collision = vsgbCollision::btCylinderCollisionShapeFromVSG( geodeCopy, _axis );
        break;
    }
    case TRIANGLE_MESH_SHAPE_PROXYTYPE:
    {
        // Reduce geometry.
        reduce( *geodeCopy );
        collision = vsgbCollision::btTriMeshCollisionShapeFromVSG( geodeCopy );
        break;
    }
    case CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE:
    {
        // Reduce geometry.
        reduce( *geodeCopy );
        collision = vsgbCollision::btConvexTriMeshCollisionShapeFromVSG( geodeCopy );
        break;
    }
    case CONVEX_HULL_SHAPE_PROXYTYPE:
    {
        collision = vsgbCollision::btConvexHullCollisionShapeFromVSG( geodeCopy );
        break;
    }
    default:
    {
        std::cerr<< "ComputeShapeVisitor: Error, unknown shape type, using tri mesh." << std::endl;
        break;
    }
    }

    if( collision && ( center != vsg::vec3( 0., 0., 0. ) ) )
    {
        btTransform trans; trans.setIdentity();
        trans.setOrigin( vsgbCollision::asBtVector3( center ) );
        btCompoundShape* masterShape = new btCompoundShape();
        masterShape->addChildShape( trans, collision );
        collision = masterShape;
    }

    return( collision );
}

void ComputeShapeVisitor::reduce( vsg::Node& node )
{
    auto currentparent=curparrentstategr;
    curparrentstategr=nullptr;
    if(!currentparent)return;//node already threated

    if( !( _bs.valid() ) )
    {
        std::cerr<< "ComputeShapeVisitor: Can't reduce with invalid bound." << std::endl;
        return;
    }

    float seFeature;
    float sePercent;
    float grpThreshold;
    float edgeError;
    switch( _reductionLevel )
    {
    case 1:
        seFeature = .15f;
        sePercent = .9f;
        grpThreshold = 8.f;
        edgeError = 8.f;
        break;
    case 2:
        seFeature = .25f;
        sePercent = .6f;
        grpThreshold = 17.f;
        edgeError = 17.f;
        break;
    case 3:
        seFeature = .535f;
        sePercent = .1f;
        grpThreshold = 280.f;
        edgeError = 28.f;
        break;
    case 0:
    default:
        // No reduction.
        return;
        break;
    }
    vsg::dvec3 l=_bs.max-_bs.min;

    if( l.x>l.y && l.x>l.z)seFeature *= l.x * 2.;
    else if( l.y>l.x && l.y>l.z)seFeature *= l.y * 2.;
    else
        seFeature *= l.z * 2.;

    std::cerr<< "ComputeShapeVisitor: Reducing..." << std::endl;

    {
        ReducerOp* redOp = new ReducerOp;
        redOp->setGroupThreshold( grpThreshold );
        redOp->setMaxEdgeError( edgeError );

        GeometryModifier modifier( redOp );
        node.accept( modifier );
        modifier.displayStatistics( std::cerr );
    }
}
// vsgbCollision
}
