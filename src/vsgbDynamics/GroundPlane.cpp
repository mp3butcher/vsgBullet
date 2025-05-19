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

#include <vsgbDynamics/GroundPlane.h>
#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <vsgbCollision/Utils.h>
#include <vsg/utils/Builder.h>


namespace vsgbDynamics
{


vsg::ref_ptr<vsg::Node> generateGroundPlane( const vsg::vec4& plane, btDynamicsWorld* bulletWorld, btRigidBody** rb, const short group, const short mask )
{
    vsg::vec3 n(plane.x,plane.y,plane.z);
    n = normalize(n);
    float d (plane.w);
    vsg::vec3 v (1.f,0,0);

    // TBD consider using vsg::vec3::operator^ for cross product: (v^n)
    vsg::vec3 u1 = n;
    u1*=(v.x*n.x +v.y*n.y + v.z*n.z);
    u1=v - u1;
    vsg::vec3 u2;
    if (length(u1)==0){
        u1 = vsg::vec3(0.f,1.f,0.f);
        u2 = vsg::vec3(0.f,0.f,1.f);
    }
    else{
        u1 = normalize(u1);
        u2 = cross(n,u1);
        u2 = normalize(u2);
    }

    vsg::vec3 p =  n;
    p*= d;

    // TBD use new stuff in Shapes.
    const btVector3 planeNormal( plane.x, plane.y, plane.z );
    btCollisionShape* groundShape = new btStaticPlaneShape( planeNormal, plane.w );
    btRigidBody::btRigidBodyConstructionInfo rbInfo( 0., nullptr, groundShape, btVector3(0,0,0) );
    btRigidBody* ground = new btRigidBody(rbInfo);

    btDiscreteDynamicsWorld* dw = dynamic_cast< btDiscreteDynamicsWorld* >( bulletWorld );
    if( ( dw != nullptr ) && ( ( group != 0 ) || ( mask != 0 ) ) )
        dw->addRigidBody( ground, group, mask );
    else
        bulletWorld->addRigidBody( ground );

    if( rb != nullptr )
        *rb = ground;


    vsg::Builder builder;
    vsg::GeometryInfo geomInfo;
    vsg::StateInfo stateInfo;

    geomInfo.color = vsg::vec4{1, 1, 1, 1};


    //p + u1*30. + u2*30
                   geomInfo.position=p;
    geomInfo.dx*=u1.x+u2.x;
    geomInfo.dy*=u1.y+u2.y;
    geomInfo.dz*=u1.z+u2.z;
    geomInfo.dx*=60;
    geomInfo.dy*=60;
    geomInfo.dz*=60;
    auto node = builder.createQuad(geomInfo, stateInfo);


    return node;
    /*

    vsg::ref_ptr<vsg::StateGroup> groundPlane =  vsg::StateGroup::create();
    //gr->addChild( node );
    auto shaderset=vsg::createFlatShadedShaderSet();
    gr->add(shaderset);


    vsg::ref_ptr< vsg::Geode > groundPlane = new vsg::Geode;
    vsg::Geometry* groundGeom = new vsg::Geometry;
    groundPlane->addDrawable(groundGeom);
    vsg::ref_ptr<vsg::vec3Array> vertarray = new vsg::vec3Array;
    groundGeom->setVertexArray( vertarray.get() );

    int width(30);
    vsg::vec3 point;
    const int nVerts( 4*width+2 );
    for(int i = -width; i < width; i++)
    {
        for(int j = -width; j < width+1; j ++)
        {  
            vertarray->push_back(p + u1*i + u2*j);
            vertarray->push_back(p + u1*(i+1) + u2*j);
        }
        groundGeom->addPrimitiveSet( new vsg::DrawArrays(
            vsg::PrimitiveSet::TRIANGLE_STRIP, (i+width)*nVerts, nVerts ) );
    }

    vsg::ref_ptr<vsg::vec3Array> norm = new vsg::vec3Array;
    groundGeom->setNormalArray( norm.get() );
    norm->push_back( n );
    groundGeom->setNormalBinding( vsg::Geometry::BIND_OVERALL );

    vsg::ref_ptr<vsg::vec4Array> c = new vsg::vec4Array;
    groundGeom->setColorArray( c.get() );
    c->push_back( vsg::Vec4( 1.f, 1.f, 1.f, 1.f ) );
    groundGeom->setColorBinding( vsg::Geometry::BIND_OVERALL );

    return 0;*/
}



// vsgbDynamics
}
