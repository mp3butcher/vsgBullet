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

#include <BulletCollision/CollisionShapes/btShapeHull.h>

#include <string>
#include <iostream>
#include <algorithm>

#include <vsgbCollision/CollisionShapes.h>
#include <vsgbCollision/ComputeShapeVisitor.h>
#include <vsgbCollision/ComputeTriMeshVisitor.h>
#include <vsgbCollision/ComputeCylinderVisitor.h>
#include <vsgbCollision/CollectVerticesVisitor.h>
#include <vsgbCollision/Utils.h>

#include <vsg/all.h>
#include <vsg/core/Visitor.h>
namespace vsgbCollision
{

class ForceFlattenTransforms : public vsg::Inherit< vsg::Visitor ,ForceFlattenTransforms>
{
public:
    ForceFlattenTransforms();


    void apply( vsg::Transform& node ) override;
   virtual void apply( vsg::MatrixTransform& node ) override;

protected:
    void flattenDrawable( vsg::Command * drawable, const vsg::mat4& matrix );
};




ForceFlattenTransforms::ForceFlattenTransforms()
{}

void ForceFlattenTransforms::apply( vsg::Transform& node )
{
    // This is a Transform that isn't a MatrixTransform and isn't a PAT.
    // If it is not an AMT, display a warning.

    if( dynamic_cast<vsg::AbsoluteTransform*>(&node))
        std::cerr << "VSGToCollada: Warning: Non-MatrixTransform encountered: (" <<
            node.className() << ") " << node.className() << std::endl;
    node.traverse( *this );
}
void ForceFlattenTransforms::apply( vsg::MatrixTransform& node )
{
    node.traverse( *this );
    node.matrix= vsg::mat4() ;
}
/*void ForceFlattenTransforms::apply( vsg::PositionAttitudeTransform& node )
{
    traverse( node );
    node.setPosition(vsg::vec3(0.0f,0.0f,0.0f));
    node.setAttitude(vsg::quat());
    node.setPivotPoint(vsg::vec3(0.0f,0.0f,0.0f));
    node.setScale(vsg::vec3(1.0f,1.0f,1.0f));
}

void ForceFlattenTransforms::apply( vsg::Geode& node )
{
    vsg::mat4 l2w = vsg::computeLocalToWorld( getNodePath() );
    unsigned int idx;
    for( idx=0; idx<node.getNumDrawables(); idx++ )
    {
        vsg::Drawable* draw( node.getDrawable( idx ) );

        vsg::Geometry* geom( dynamic_cast< vsg::Geometry* >( draw ) );
        if( geom )
        {
            // Requires 2.6.1, the necessary Geometry support didn't exist in 2.6.
            if( geom->containsSharedArrays() )
                geom->duplicateSharedArrays();
        }

        flattenDrawable( draw, l2w );
    }
}
*/
void ForceFlattenTransforms::flattenDrawable( vsg::Command* drawable, const vsg::mat4& matrix )
{
   /* vsg::VertexIndexDraw
    if( drawable )
    {
        TransformAttributeFunctor tf(matrix);
        drawable->accept(tf);
        drawable->dirtyBound();
        drawable->dirtyDisplayList();
    }*/
}

btSphereShape* btSphereCollisionShapeFromVSG( vsg::Node* node )
{
    vsg::ComputeBounds cbv;
    node->accept( cbv );
    const vsg::box bb( cbv.bounds );
    vsg::vec3 ext( bb.max - bb.min );
    ext *= 0.5f;
    float radius = 0.;
    for( size_t i = 0; i < 3; ++i )
    {
        radius = std::max( radius, ext[ i ] );
    }
    btSphereShape* shape = new btSphereShape( radius );

    return( shape );
}

btBoxShape* btBoxCollisionShapeFromVSG( vsg::Node* node, const vsg::dbox* bb )
{
    vsg::dbox bbox;
    if (bb)
        bbox = *bb;
    else
    {
        vsg::ComputeBounds visitor;
        node->accept( visitor );
        bbox = visitor.bounds;
    }

    btBoxShape* shape = new btBoxShape( btVector3( ( bbox.max.x - bbox.min.x ) * 0.5,
        ( bbox.max.y - bbox.min.y ) * 0.5, ( bbox.max.z - bbox.min.z ) * 0.5 ) );
    return( shape );
}

btCylinderShape* btCylinderCollisionShapeFromVSG( vsg::Node* node, AXIS axis )
{
    ComputeCylinderVisitor visitor;
    switch( axis )
    {
    case X:
        visitor.setAxis( vsg::vec3(1,0,0) );
        break;
    case Y:
        visitor.setAxis( vsg::vec3(0,1,0) );
        break;
    case Z:
        visitor.setAxis( vsg::vec3(0,0,1) );
        break;
    }
    node->accept( visitor );

    BoundingCylinder cyl = visitor.getBoundingCylinder();
    if( cyl.getRadius() <= 0. )
    {
        std::cerr << "nullptr bounding cylinder." << std::endl;
        return( nullptr );
    }

    btCylinderShape* shape = 0;
    switch( axis )
    {
    case X:
        shape = new btCylinderShapeX( btVector3( cyl.getLength(), cyl.getRadius(), cyl.getRadius() ) );
        break;
    case Y:
        shape = new btCylinderShape( btVector3( cyl.getRadius(), cyl.getLength(), cyl.getRadius() ) );
        break;
    case Z:
        shape = new btCylinderShapeZ( btVector3( cyl.getRadius(), cyl.getRadius(), cyl.getLength() ) );
        break;
    }
    return( shape );
}

btTriangleMeshShape* btTriMeshCollisionShapeFromVSG( vsg::Node* node )
{
    ComputeTriMeshVisitor visitor;
    node->accept( visitor );

    vsg::vec3Array* vertices = visitor.getTriMesh();
    if( vertices->size() < 3 )
    {
        std::cerr << "vsgbCollision::btTriMeshCollisionShapeFromVSG, no triangles found" << std::endl;
        return( nullptr );
    }

    btTriangleMesh* mesh = new btTriangleMesh;
    for( size_t i = 0; i + 2 < vertices->size(); i += 3 )
    {
        vsg::vec3& p1 = ( *vertices )[ i ];
        vsg::vec3& p2 = ( *vertices )[ i + 1 ];
        vsg::vec3& p3 = ( *vertices )[ i + 2 ];
        mesh->addTriangle( vsgbCollision::asBtVector3( p1 ),
            vsgbCollision::asBtVector3( p2 ), vsgbCollision::asBtVector3( p3 ) );
    }

    btBvhTriangleMeshShape* meshShape = new btBvhTriangleMeshShape( mesh, true );
    return( meshShape );
}

btConvexTriangleMeshShape* btConvexTriMeshCollisionShapeFromVSG( vsg::Node* node )
{
    ComputeTriMeshVisitor visitor;
    node->accept( visitor );

    vsg::vec3Array* vertices = visitor.getTriMesh();

    btTriangleMesh* mesh = new btTriangleMesh;
    vsg::vec3 p1, p2, p3;
    for( size_t i = 0; i + 2 < vertices->size(); i += 3 )
    {
        p1 = vertices->at( i );
        p2 = vertices->at( i + 1 );
        p3 = vertices->at( i + 2 );
        mesh->addTriangle( vsgbCollision::asBtVector3( p1 ),
            vsgbCollision::asBtVector3( p2 ), vsgbCollision::asBtVector3( p3 ) );
    }

    btConvexTriangleMeshShape* meshShape = new btConvexTriangleMeshShape( mesh );
    return( meshShape );
}

btConvexHullShape* btConvexHullCollisionShapeFromVSG( vsg::Node* node )
{
    CollectVerticesVisitor cvv;
    node->accept( cvv );
    std::vector<vsg::vec3>*v = &cvv.getVertices();
    std::cerr << "CollectVerticesVisitor: " << v->size() << std::endl;

    // Convert verts to array of Bullet scalars.
    btScalar* btverts = new btScalar[ v->size() * 3 ];
    if( btverts == nullptr )
    {
        std::cerr << "nullptr btverts" << std::endl;
        return( nullptr );
    }
    btScalar* btvp = btverts;

    for(auto itr = v->begin(); itr != v->end(); ++itr )
    {
        const vsg::vec3& s( *itr );
        *btvp++ = (btScalar)( s[ 0 ] );
        *btvp++ = (btScalar)( s[ 1 ] );
        *btvp++ = (btScalar)( s[ 2 ] );
    }
    btConvexHullShape* chs = new btConvexHullShape( btverts,
        (int)( v->size() ), (int)( sizeof( btScalar ) * 3 ) );
    delete[] btverts;

    return( chs );
}

btCompoundShape* btCompoundShapeFromVSGGeodes( vsg::Node* node,
    const BroadphaseNativeTypes shapeType, const vsgbCollision::AXIS axis,
    const unsigned int reductionLevel )
{
    ComputeShapeVisitor csv( shapeType, axis, reductionLevel );
    node->accept( csv );

    btCompoundShape* cs = static_cast< btCompoundShape* >( csv.getShape() );
    return( cs );
}
btCompoundShape* btCompoundShapeFromVSGGeometry( vsg::Node*)
{
    std::cerr << "btCompoundShapeFromVSGGeometry: This function is not currently implemented." << std::endl;
    throw( std::string( "btCompoundShapeFromVSGGeometry not implemented" ) );
    return( nullptr );
}

btCompoundShape* btCompoundShapeFromBounds( vsg::Node* node,
    const BroadphaseNativeTypes shapeType, const vsgbCollision::AXIS axis )
{
    btCollisionShape* shape( nullptr );
    switch( shapeType )
    {
    case BOX_SHAPE_PROXYTYPE:
        shape = btBoxCollisionShapeFromVSG( node );
        break;
    case SPHERE_SHAPE_PROXYTYPE:
        shape = btSphereCollisionShapeFromVSG( node );
        break;
    case CYLINDER_SHAPE_PROXYTYPE:
        shape = btCylinderCollisionShapeFromVSG( node, axis );
        break;
    default:
        std::cerr<< "btCompoundShapeFromBounds: Unsupported shapeType: " << (int)shapeType << std::endl;
        break;
    }

    vsg::ComputeBounds cbv;
    node->accept( cbv );
    vsg::dvec3 c(cbv.bounds.max+cbv.bounds.min);
    c*=0.5f;
    btVector3 center( vsgbCollision::asBtVector3( vsg::vec3(c.x,c.y,c.z) ) );

    btTransform wt; wt.setIdentity();
    wt.setOrigin( center );

    btCompoundShape* xformShape = new btCompoundShape;
    xformShape->addChildShape( wt, shape );
    return( xformShape );
}




vsg::ref_ptr<vsg::Node> vsgNodeFromBtCollisionShape( const btCollisionShape* btShape, const btTransform& trans )
{
    if( btShape->getShapeType() == BOX_SHAPE_PROXYTYPE )
    {
        const btBoxShape* btBox = static_cast< const btBoxShape* >( btShape );
        return( vsgNodeFromBtCollisionShape( btBox, trans ) );
    }
    else if( btShape->getShapeType() == SPHERE_SHAPE_PROXYTYPE )
    {
        const btSphereShape* btSphere = static_cast< const btSphereShape* >( btShape );
        return( vsgNodeFromBtCollisionShape( btSphere, trans ) );
    }
    else if( btShape->getShapeType() == CYLINDER_SHAPE_PROXYTYPE )
    {
        const btCylinderShape* btCylinder = static_cast< const btCylinderShape* >( btShape );
        return( vsgNodeFromBtCollisionShape( btCylinder, trans ) );
    }
    else if( btShape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE )
    {
        const btBvhTriangleMeshShape* btTriMesh = static_cast< const btBvhTriangleMeshShape* >( btShape );
        // Do NOT pass in a transform. Unlike cylinder, sphere, and box,
        // tri meshes are always in absolute space.
        return( vsgNodeFromBtCollisionShape( btTriMesh ) );
    }
    else if( btShape->getShapeType() == CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE )
    {
        const btConvexTriangleMeshShape* btConvexTriMesh = static_cast< const btConvexTriangleMeshShape* >( btShape );
        // Do NOT pass in a transform. Unlike cylinder, sphere, and box,
        // tri meshes are always in absolute space.
        return( vsgNodeFromBtCollisionShape( btConvexTriMesh ) );
    }
    else if( btShape->getShapeType() == CONVEX_HULL_SHAPE_PROXYTYPE )
    {
        const btConvexHullShape* convexHull = static_cast< const btConvexHullShape* >( btShape );
        // Do NOT pass in a transform. Unlike cylinder, sphere, and box,
        // tri meshes are always in absolute space.
        return( vsgNodeFromBtCollisionShape( convexHull ) );
    }
    else if( btShape->getShapeType() == COMPOUND_SHAPE_PROXYTYPE )
    {
        const btCompoundShape* masterShape = static_cast< const btCompoundShape* >( btShape );
        vsg::ref_ptr<vsg::Group> grp =  vsg::Group::create();
        int idx;
        for (idx=0; idx< masterShape->getNumChildShapes(); idx++)
        {
            const btCollisionShape* s = masterShape->getChildShape( idx );
            const btTransform t = masterShape->getChildTransform( idx );
            const btTransform accumTrans = trans * t;
            grp->addChild( vsgNodeFromBtCollisionShape( s, accumTrans ) );
        }
        return( grp );
    }
    else
    {
        std::cerr << "vsgNodeFromBtCollisionShape: Unsupported shape type: " <<
        btShape->getShapeType() << std::endl;
        return( nullptr );
    }
}

vsg::ref_ptr<vsg::Node> vsgNodeFromBtCollisionShape( const btBoxShape* btBox, const btTransform& trans )
{
    vsg::ref_ptr<vsg::Group> geode = vsg::Group::create();
    geode->addChild(vsg::ref_ptr<vsg::Node>(vsgDrawableFromBtCollisionShape( btBox )));

    vsg::mat4 m = asVsgMatrix( trans );
    if (m==vsg::mat4())//m.isIdentity())
        return( geode );
    else
    {
        vsg::ref_ptr<vsg::MatrixTransform> mt = vsg::MatrixTransform::create();
        mt->matrix = m;
        mt->addChild( geode );
        return mt;
    }
}

vsg::ref_ptr<vsg::Node> vsgNodeFromBtCollisionShape( const btSphereShape* btSphere, const btTransform& trans )
{
    vsg::ref_ptr<vsg::Group> gr = vsg::Group::create();
    gr->addChild(vsgDrawableFromBtCollisionShape( btSphere ) );

    vsg::mat4 m = asVsgMatrix( trans );
    if (m == vsg::mat4())//m.isIdentity())
        return( gr );
    else
    {
        vsg::ref_ptr<vsg::MatrixTransform> mt = vsg::MatrixTransform::create();
        mt->matrix= m;
        mt->addChild( gr );
        return gr;
    }
}

vsg::ref_ptr<vsg::Node> vsgNodeFromBtCollisionShape( const btCylinderShape * btCylinder, const btTransform& trans )
{
    vsg::Builder builder;
    vsg::GeometryInfo geomInfo;
    vsg::StateInfo stateInfo;

    geomInfo.color = vsg::vec4{1, 1, 1, 1};

    switch( btCylinder->getUpAxis() )
    {
    case X:
        geomInfo.dy*=( 2 * btCylinder->getHalfExtentsWithMargin().getX() );
        geomInfo.transform = vsg::rotate(vsg::quat( vsg::PIf*0.5, vsg::vec3( 0, 1, 0 ) )) ;
        break;
    case Y:
        geomInfo.dy*=( 2 * btCylinder->getHalfExtentsWithMargin().getY() );
        geomInfo.transform = vsg::rotate(vsg::quat(vsg::PIf*0.5, vsg::vec3( 1, 0, 0 ) ) );
        break;
    case Z:
        geomInfo.dy*=( 2 * btCylinder->getHalfExtentsWithMargin().getZ() );
        geomInfo.transform = vsg::rotate(vsg::quat( vsg::PIf*0.5, vsg::vec3( 0, 0, 1 ) ) );
    }
    geomInfo.dx*=btCylinder->getRadius();
    geomInfo.dz*=btCylinder->getRadius();
    //cylinder->setRadius( btCylinder->getRadius() );

    auto node = builder.createCylinder(geomInfo, stateInfo);


   /* switch( btCylinder->getUpAxis() )
    {
        case X:
            cylinder->setHeight( 2 * btCylinder->getHalfExtentsWithMargin().getX() );
            cylinder->setRotation( vsg::quat( vsg::PI_2, vsg::vec3( 0, 1, 0 ) ) );
            break;
        case Y:
            cylinder->setHeight( 2 * btCylinder->getHalfExtentsWithMargin().getY() );
            cylinder->setRotation( vsg::quat( vsg::PI_2, vsg::vec3( 1, 0, 0 ) ) );
            break;
        case Z:
            cylinder->setHeight( 2 * btCylinder->getHalfExtentsWithMargin().getZ() );
    }
*/

    auto stateGroup = builder.createStateGroup(stateInfo);
    stateGroup->addChild( node );

    vsg::mat4 m = asVsgMatrix( trans );
   if (m == vsg::mat4())//m.isIdentity())
        return( stateGroup );
    else
    {
        vsg::ref_ptr<vsg::MatrixTransform> mt = vsg::MatrixTransform::create();
        mt->matrix=( m );
        mt->addChild( stateGroup );
        return mt;
    }
}

vsg::ref_ptr<vsg::Node> vsgNodeFromBtCollisionShape( const btTriangleMeshShape* btTriMesh, const btTransform& trans )
{
    const btTriangleMesh* mesh = dynamic_cast< const btTriangleMesh* >( btTriMesh->getMeshInterface() );
    if( !mesh )
    {
        std::cerr << "vsgNodeFromBtCollisionShape: No triangle mesh." << std::endl;
        return( nullptr );
    }

    btVector3* verts;
    int* indices;
    int numVerts;
    int numFaces;
    PHY_ScalarType vt, ft;
    int vs, fs;

    mesh->getLockedReadOnlyVertexIndexBase( ( const unsigned char** )&verts, numVerts, vt, vs, ( const unsigned char** )&indices, fs, numFaces, ft );

    vsg::ref_ptr<vsg::vec3Array> vec =  vsg::vec3Array::create(numVerts);
    int idx;
    for( idx = 0; idx < numVerts; idx++ )
    {
        const btVector3& bulletVert = verts[ idx ];
        ( *vec )[ idx ].set( bulletVert.getX(), bulletVert.getY(), bulletVert.getZ() );
    }
    vsg::ref_ptr<vsg::vec4Array> color =  vsg::vec4Array::create({vsg::vec4( 1., 1., 1., 1. )});

    auto di = vsg::VertexIndexDraw::create();
    di->indexCount = numFaces * 3;
    di->instanceCount = 1;

    di->assignArrays({vec, color});
    auto vind = vsg::ushortArray::create(numFaces * 3);

    for( idx = 0; idx < numFaces * 3; idx++ )
        vind->at(idx)= indices[ idx ] ;
    di-> assignIndices(vind);

    vsg::Builder builder;
    vsg::StateInfo stateInfo;

    auto stateGroup = builder.createStateGroup(stateInfo);
    stateGroup->addChild(di);

  /*   auto vertexShader = vsg::ShaderStage::create(VK_SHADER_STAGE_VERTEX_BIT, "main", VERT);
    auto fragmentShader = vsg::ShaderStage::create(VK_SHADER_STAGE_FRAGMENT_BIT, "main", FRAG);
    auto shaderSet = vsg::ShaderSet::create(vsg::ShaderStages{vertexShader, fragmentShader});
    shaderSet->addPushConstantRange("pc", "", VK_SHADER_STAGE_VERTEX_BIT, 0, 128);
    shaderSet->addAttributeBinding("vertex", "", 0, VK_FORMAT_R32G32B32_SFLOAT, vsg::vec3Array::create(1));

   auto gpConf = vsg::GraphicsPipelineConfigurator::create(shaderSet);

    auto vertices = vsg::vec3Array::create({
                                            {0, 0, 0},
                                            {1, 0, 0},
                                            {0, 1, 0},
                                            {0, 0, 1},
                                            });
    vsg::DataList vertexArrays;
    gpConf->assignArray(vertexArrays, "vertex", VK_VERTEX_INPUT_RATE_VERTEX, vertices);
    auto vertexDraw = vsg::VertexDraw::create();
    vertexDraw->assignArrays(vertexArrays);
    vertexDraw->vertexCount = vertices->width();
    vertexDraw->instanceCount = 1;
    stateGroup->addChild(vertexDraw);

    struct SetPipelineStates : public vsg::Visitor
    {
        void apply(vsg::Object& object) { object.traverse(*this); }
        void apply(vsg::RasterizationState& rs)
        {
            rs.lineWidth = 10.0f;
            rs.cullMode = VK_CULL_MODE_NONE;
        }
        void apply(vsg::InputAssemblyState& ias)
        {
            ias.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLES_STRIP;
        }
    } sps;

    auto rs = vsg::RasterizationState::create();
    gpConf->pipelineStates.push_back(rs);

    gpConf->accept(sps);
    gpConf->init();
    gpConf->copyTo(stateGroup);
    */

    vsg::mat4 m = asVsgMatrix( trans );
   if (m == vsg::mat4())//m.isIdentity())
        return( stateGroup );
    else
    {
        vsg::ref_ptr<vsg::MatrixTransform> mt = vsg::MatrixTransform::create();
        mt->matrix= m;
        mt->addChild( stateGroup );
        return mt;
    }
}

vsg::ref_ptr<vsg::Node> vsgNodeFromBtCollisionShape( const btConvexTriangleMeshShape* btTriMesh, const btTransform& trans )
{
    const btTriangleMesh* mesh = dynamic_cast< const btTriangleMesh* >( btTriMesh->getMeshInterface() );
    if( !mesh )
    {
        std::cerr << "vsgNodeFromBtCollisionShape: No triangle mesh." << std::endl;
        return( nullptr );
    }

    btVector3* verts;
    int* indices;
    int numVerts;
    int numFaces;
    PHY_ScalarType vt, ft;
    int vs, fs;

    mesh->getLockedReadOnlyVertexIndexBase( ( const unsigned char** )&verts, numVerts, vt, vs, ( const unsigned char** )&indices, fs, numFaces, ft );

    vsg::ref_ptr<vsg::vec3Array> vec = vsg::vec3Array::create(numVerts);
    int idx;
    for( idx = 0; idx < numVerts; idx++ )
    {
        const btVector3& bulletVert = verts[ idx ];
        ( *vec )[ idx ].set( bulletVert.getX(), bulletVert.getY(), bulletVert.getZ() );
    }

    vsg::ref_ptr<vsg::vec4Array> color =  vsg::vec4Array::create({vsg::vec4( 1., 1., 1., 1. )});

    auto di=vsg::VertexIndexDraw::create();
    di->indexCount = numFaces * 3;
    di->instanceCount = 1;

    di->assignArrays({vec, color});
    auto vind = vsg::ushortArray::create(numFaces * 3);

    for( idx = 0; idx < numFaces * 3; idx++ )
        vind->at(idx)= indices[ idx ] ;
    di-> assignIndices(vind);

    vsg::Builder builder;
    vsg::StateInfo stateInfo;

    auto stateGroup = builder.createStateGroup(stateInfo);
    stateGroup->addChild(di);

    vsg::mat4 m = asVsgMatrix( trans );
    if (m == vsg::mat4())//m.isIdentity())
        return( stateGroup );
    else
    {
        vsg::ref_ptr<vsg::MatrixTransform> mt = vsg::MatrixTransform::create();
        mt->matrix = m;
        mt->addChild( stateGroup );
        return mt;
    }
}

vsg::ref_ptr<vsg::Node> vsgNodeFromBtCollisionShape( const btConvexHullShape* hull, const btTransform& trans )
{
    btShapeHull sh( hull );
    sh.buildHull( btScalar( 0. ) );
	int nVerts( sh.numVertices () );
	int nIdx( sh.numIndices() );
    if( (nVerts <= 0) || (nIdx <= 0) )
        return( nullptr );

    const btVector3* bVerts( sh.getVertexPointer() );
    const unsigned int* bIdx( sh.getIndexPointer() );

    auto v =  vsg::vec3Array::create(nVerts);
    unsigned int idx;
    for( idx = 0; idx < (unsigned int)nVerts; idx++ )
        ( *v )[ idx ] = asVsgVec3( bVerts[ idx ] );

    vsg::ref_ptr<vsg::vec4Array> color =  vsg::vec4Array::create({vsg::vec4( 1., 1., 1., 1. )});

    auto gr=vsg::Group::create();
    auto di=vsg::VertexIndexDraw::create();
    di->indexCount = nIdx;
    di->instanceCount = 1;

    di->assignArrays(vsg::DataList{v, color});
    auto vind = vsg::ushortArray::create(nIdx);

    for( idx = 0; idx < (unsigned int)nIdx; idx++ )
        vind->at(idx)= bIdx[ idx ] ;

    di-> assignIndices(vind);
    vsg::Builder builder;
    vsg::StateInfo stateInfo;

    auto stateGroup = builder.createStateGroup(stateInfo);
    stateGroup->addChild(di);
    //  b ->setHalfLengths(vsgbCollision::asVsgvec3( btBox->getHalfExtentsWithMargin() ));


    vsg::mat4 m = asVsgMatrix( trans );
    if (m == vsg::mat4())//m.isIdentity())
       return( stateGroup );
    else
    {
        vsg::ref_ptr<vsg::MatrixTransform> mt = vsg::MatrixTransform::create();
        mt->matrix = m;
        mt->addChild( stateGroup );
        return mt;
    }
}


vsg::ref_ptr<vsg::Node> vsgDrawableFromBtCollisionShape( const btBoxShape* btBox )
{
    vsg::Builder builder;
    vsg::GeometryInfo geomInfo;
    vsg::StateInfo stateInfo;

    geomInfo.color = vsg::vec4{1, 1, 1, 1};
    geomInfo.dx *= btBox->getHalfExtentsWithMargin()[0]*2.f;
    geomInfo.dy *= btBox->getHalfExtentsWithMargin()[1]*2.f;
    geomInfo.dz *= btBox->getHalfExtentsWithMargin()[2]*2.f;

    auto node = builder.createBox(geomInfo, stateInfo);
    auto stateGroup = builder.createStateGroup(stateInfo);

    stateGroup->addChild(node);
  //  b ->setHalfLengths(vsgbCollision::asVsgvec3( btBox->getHalfExtentsWithMargin() ));
    return stateGroup;
}

vsg::ref_ptr<vsg::Node> vsgDrawableFromBtCollisionShape( const btSphereShape* btSphere )
{
    vsg::Builder builder;
    vsg::GeometryInfo geomInfo;
    vsg::StateInfo stateInfo;

    geomInfo.color = vsg::vec4{1, 1, 1, 1};
    geomInfo.dx *= 2.f*btSphere->getRadius();
    geomInfo.dy *= 2.f*btSphere->getRadius();
    geomInfo.dz *= 2.f*btSphere->getRadius();

    auto node = builder.createSphere(geomInfo, stateInfo);

    auto stateGroup = builder.createStateGroup(stateInfo);
    stateGroup->addChild(node);
    //vsg::ref_ptr<vsg::Sphere> b=new vsg::Sphere(vsg::vec3(), btSphere->getRadius());

    return stateGroup;
}

static vsg::vec3Array*
generateCircleVertices( unsigned int approx, double radius, vsg::vec4 plane, bool close=false )
{
    // Find ideal base vector (at 90 degree angle to normalVec)
    vsg::vec3 normalVec( plane[0], plane[1], plane[2] );
    normalVec=normalize(normalVec);
    vsg::vec3 crossVec( 1., 0., 0. );
    float a = dot(normalVec , crossVec );
    a = a<0 ? -a : a;
    if( a > .9 )
        crossVec = vsg::vec3( 0., 1., 0. );
    vsg::vec3 baseVec = vsg::cross(normalVec , crossVec);
    baseVec=normalize(baseVec);
    // baseVec = ( baseVec * radius ) + ( normalVec * plane[ 3 ] );
    vsg::vec3 t = normalVec;
    t*= plane[ 3 ] ;
    baseVec*=radius;
    baseVec+=t;

    unsigned int totalVerts = approx + ( close ? 1 : 0 );
    vsg::ref_ptr< vsg::vec3Array > verts =  vsg::vec3Array::create(totalVerts);

    unsigned int idx;
	for( idx=0; idx < approx; idx++ )
    {
        const double angle( 2. * vsg::PI * (double)idx / (double)approx );
        vsg::mat4 m( vsg::rotate( float(angle), normalVec ) );
        ( *verts )[ idx ] = baseVec * m;
    }
    if( close )
        ( *verts )[ idx ] = ( *verts )[ 0 ];

    return( verts );
}
/*
static bool
buildCylinderData( const double length, const double radius0, const double radius1, const vsg::uivec2 & subdivisions, vsg::Geometry* geometry, const bool wire )
{
    int subCylinders = subdivisions[ 0 ];
    if( subCylinders < 1 )
        subCylinders = 1;
    const double radiusDelta = ( radius1 - radius0 ) / subCylinders;

    vsg::vec3Array* vertices;
    if( geometry->getVertexArray() != nullptr )
    {
        vertices = dynamic_cast< vsg::vec3Array* >( geometry->getVertexArray() );
        if( vertices == nullptr )
            return( false );
    }
    else
    {
        vertices = new vsg::vec3Array;
        geometry->setVertexArray( vertices );
    }

    vsg::vec3Array* normals;
    vsg::vec2Array* texCoords;
    if( !wire )
    {
        if( geometry->getNormalArray() != nullptr )
        {
            normals = dynamic_cast< vsg::vec3Array* >( geometry->getNormalArray() );
            if( normals == nullptr )
                return( false );
        }
        else
        {
            normals = new vsg::vec3Array;
            geometry->setNormalArray( normals );
        }
        geometry->setNormalBinding( vsg::Geometry::BIND_PER_VERTEX );

        if( geometry->getTexCoordArray( 0 ) != nullptr )
        {
            texCoords = dynamic_cast< vsg::vec2Array* >( geometry->getTexCoordArray( 0 ) );
            if( texCoords == nullptr )
                return( false );
        }
        else
        {
            texCoords = new vsg::vec2Array;
            geometry->setTexCoordArray( 0, texCoords );
        }
    }

    vsg::vec4Array* vsgC = new vsg::vec4Array;
    vsgC->push_back( vsg::vec4( 1., 1., 1., 1. ) );
    geometry->setColorArray( vsgC );
    geometry->setColorBinding( vsg::Geometry::BIND_OVERALL );


    // Generate a set of normals that we'll map to each cylinder hoop.
    vsg::vec4 plane( 0., 0., 1., 0. );
    vsg::ref_ptr< vsg::vec3Array > cNorms;
    if( !wire )
        cNorms = generateCircleVertices( subdivisions[ 1 ], 1., plane, true );

    int idx;
    for( idx=0; idx <= subCylinders; idx++ )
    {
        const double percent( (double)idx / (double)subCylinders );
        plane[ 3 ] = length * percent;

        const double radius = radius0 + ( idx * radiusDelta );
        vsg::ref_ptr< vsg::vec3Array > cVerts = generateCircleVertices( subdivisions[ 1 ], radius, plane, !wire );
        vertices->insert( vertices->end(), cVerts->begin(), cVerts->end() );

        if( !wire )
        {
            normals->insert( normals->end(), cNorms->begin(), cNorms->end() );

            const double tVal( percent );
            texCoords->reserve( vertices->size() );
            unsigned int tcIdx;
            for( tcIdx = 0; tcIdx < cVerts->size(); tcIdx++ )
            {
                const double sVal( (double)tcIdx / (double)( cVerts->size() - 1 ) );
                texCoords->push_back( vsg::vec2( sVal, tVal ) );
            }
        }
    }


    // Add PrimitiveSets

    if( !wire )
    {
        const unsigned int vertCount = vertices->size() / ( subCylinders + 1 );
        for( idx=0; idx < subCylinders; idx++ )
        {
            vsg::DrawElementsUShort* deus = new vsg::DrawElementsUShort( GL_TRIANGLE_STRIP );
            unsigned int vIdx = vertCount * ( idx + 1 );
            unsigned int innerIdx;
            for( innerIdx = 0; innerIdx < vertCount; innerIdx++ )
            {
                deus->push_back( vIdx );
                deus->push_back( vIdx - vertCount );
                vIdx++;
            }
            geometry->addPrimitiveSet( deus );
        }
    }
    else
    {
        const unsigned int vertCount = vertices->size() / ( subCylinders + 1 );
        unsigned int vIdx = 0;
        for( idx=0; idx <= subCylinders; idx++ )
        {
            vsg::DrawElementsUShort* deus = new vsg::DrawElementsUShort( GL_LINE_LOOP );
            deus->reserve( vertCount );
            unsigned int innerIdx;
            for( innerIdx = 0; innerIdx < vertCount; innerIdx++ )
            {
                deus->push_back( vIdx );
                vIdx++;
            }
            geometry->addPrimitiveSet( deus );
        }

        const unsigned int lineIndex = vertices->size() - vertCount;
        vsg::DrawElementsUShort* deusl = new vsg::DrawElementsUShort( GL_LINES );
        deusl->reserve( vertCount * 2 );
        for( vIdx=0; vIdx<vertCount; vIdx++ )
        {
            deusl->push_back( vIdx );
            deusl->push_back( vIdx + lineIndex );
        }
        geometry->addPrimitiveSet( deusl );
    }

    return( true );
}
*/
vsg::ref_ptr<vsg::Node> vsgDrawableFromBtCollisionShape( const btCylinderShape* btCylinder )
{
  /*  const vsg::vec3 defaultOrientation( 0., 0., 1. );
    vsg::mat4 m;
    double length;
    const btVector3 halfExtents( btCylinder->getHalfExtentsWithMargin() );
    switch( btCylinder->getUpAxis() )
    {
        case X:
            m = vsg::mat4::rotate( defaultOrientation, vsg::vec3( 1., 0., 0. ) );
            length = halfExtents.getX();
            break;
        case Y:
            m = vsg::mat4::rotate( defaultOrientation, vsg::vec3( 0., 1., 0. ) );
            length = halfExtents.getY();
            break;
        case Z:
            // Leave m set to the identity matrix.
            length = halfExtents.getZ();
            break;
    }
    const double radius( btCylinder->getRadius() );

    vsg::ref_ptr<vsg::Geometry>drawable=new vsg::Geometry();
    buildCylinderData( length, radius, radius ,  vsg::vec2s( 1, 8 ),drawable.get(),false);
  TransformAttributeFunctor tf(m);
        drawable->accept(tf);
    return drawable;//( makeOpenCylinder( m, length, radius, radius ) );
            */

        vsg::Builder builder;
        vsg::GeometryInfo geomInfo;
        vsg::StateInfo stateInfo;

        geomInfo.color = vsg::vec4{1, 1, 1, 1};

        switch( btCylinder->getUpAxis() )
        {
        case X:
            geomInfo.dy *= ( 2 * btCylinder->getHalfExtentsWithMargin().getX() );
            geomInfo.transform = vsg::rotate(vsg::quat( vsg::PIf*0.5, vsg::vec3( 0, 1, 0 ) )) ;
            break;
        case Y:
            geomInfo.dy *= ( 2 * btCylinder->getHalfExtentsWithMargin().getY() );
            geomInfo.transform = vsg::rotate(vsg::quat(vsg::PIf*0.5, vsg::vec3( 1, 0, 0 ) ) );
            break;
        case Z:
            geomInfo.dy *= ( 2 * btCylinder->getHalfExtentsWithMargin().getZ() );
            geomInfo.transform = vsg::rotate(vsg::quat( vsg::PIf*0.5, vsg::vec3( 0, 0, 1 ) ) );
        }
        geomInfo.dx *= btCylinder->getRadius()*2.;
        geomInfo.dz *= btCylinder->getRadius()*2.;
        //cylinder->setRadius( btCylinder->getRadius() );

        auto node = builder.createCylinder(geomInfo, stateInfo);

        auto stateGroup = builder.createStateGroup(stateInfo);
        stateGroup->addChild( node );
        return stateGroup;
}


// vsgbCollision
}
