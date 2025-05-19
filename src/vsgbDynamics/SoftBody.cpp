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

#include <vsgbDynamics/SoftBody.h>
#include <vsgbDynamics/CreationRecord.h>
#include <vsgbDynamics/MotionState.h>
#include <vsgbDynamics/World.h>
#include <vsgbCollision/Utils.h>
#include <vsgbCollision/CollisionShapes.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>

#include <vsgbCollision/Utils.h>
#include <vsgbDynamics/TripleBuffer.h>
#include <assert.h>
using namespace vsgbCollision;
namespace vsgbDynamics
{
/*
class FindParentVisitor : public vsg::NodeVisitor
{
public:
    FindParentVisitor()
        : vsg::NodeVisitor(vsg::NodeVisitor::TRAVERSE_PARENTS), foundWorld(nullptr)  {}

    void apply( vsg::Node& node )
    {

        if( !foundWorld )
            foundWorld = dynamic_cast<World*>(&node);

        traverse( node );
    }

    World* foundWorld;
};
*/
Anchor::Anchor():_collisionenabled(true),_rig(0),_nodeindex(0){

}
Anchor::Anchor( const Anchor& copy, const vsg::CopyOp& copyop ){}
Anchor::~Anchor(){

}
void Anchor::setRigidBody(RigidBody*rig){
_rig=rig;
}
SoftBody::SoftBody():_parentWorld(0),_body(0) {
    //forceUpdate
    //setNumChildrenRequiringUpdateTraversal(1);
}
SoftBody::~SoftBody() {
    if(_parentWorld&&_body){
        btSoftRigidDynamicsWorld * w=0;
        if(w=dynamic_cast<btSoftRigidDynamicsWorld*>(_parentWorld->getDynamicsWorld())){
            w->removeSoftBody(_body);
            delete _body;
        }
    }
}
SoftBody::SoftBody( const SoftBody& copy, const vsg::CopyOp& copyop) {}
/*const btSoftBody* SoftBody::getSoftBody()const
{
    const   vsgbCollision::RefBulletObject<  btSoftBody >*  refptr=
        dynamic_cast<  const   vsgbCollision::RefBulletObject<  btSoftBody >* >( getUserData() );

    return (refptr?refptr->get():0);
}

btSoftBody* SoftBody::getSoftBody()
{
    vsgbCollision::RefBulletObject<  btSoftBody >*  refptr=
        dynamic_cast<     vsgbCollision::RefBulletObject<  btSoftBody >* >( getUserData() );

    return (refptr?refptr->get():0);
}
*/
void SoftBody::setSoftBody( btSoftBody *softBody )
{
    _body=softBody;//setUserData(new vsgbCollision::RefBulletObject<btSoftBody>(softBody));
    _windVelocity=asVsgVec3((softBody)->getWindVelocity());

   if(softBody->m_tetras.size()>0)
    {
        ///tetramesh detected: TODO

    }
    else    if(softBody->m_faces.size()>0)
    {
        ///trimesh detected so bake nodes to geometry
#define FILLINDEX(TYPE)  { vsg::DrawElements##TYPE * primset=new vsg::DrawElements##TYPE(GL_TRIANGLES);\
    for(int i=0;i<softBody->m_faces.size();i++)\
        for(short ni=0;ni<3;ni++)\
            primset->push_back(softBody->m_faces[i].m_n[ni]-&softBody->m_nodes[0]);\
    addPrimitiveSet(primset);}
       // this->removePrimitiveSet(0,getNumPrimitiveSets());
        unsigned int numIndices=softBody->m_faces.size()*3;
   /*     if(numIndices<128)          FILLINDEX(UShort)
            else if(numIndices<65535)   FILLINDEX(UByte)
                else                        FILLINDEX(UInt)*/
#undef FILLINDEX
        vsg::vec3Array *verts=new vsg::vec3Array();
        vsg::vec3Array *norms=new vsg::vec3Array();
       /* verts->resize(softBody->m_nodes.size());
        norms->resize(softBody->m_nodes.size());
        setVertexArray(verts);
        setNormalArray(norms);*/
    }
    else if(softBody->m_links.size()>0)
    {
        ///rope detected
#define FILLINDEX(TYPE)  { vsg::DrawElements##TYPE * primset=new vsg::DrawElements##TYPE(GL_LINES);\
    for(int i=0;i<softBody->m_links.size();i++)\
        for(short ni=0;ni<2;ni++)\
            primset->push_back(softBody->m_links[i].m_n[ni]-&softBody->m_nodes[0]);\
    addPrimitiveSet(primset);}
       /* this->removePrimitiveSet(0,getNumPrimitiveSets());
        unsigned int numIndices=softBody->m_links.size()*2;
        if(numIndices<128)          FILLINDEX(UShort)
            else if(numIndices<65535)   FILLINDEX(UByte)
                else                        FILLINDEX(UInt)
        vsg::vec3Array *verts=new vsg::vec3Array();
        vsg::vec3Array *norms=new vsg::vec3Array();
        verts->resize(softBody->m_nodes.size());
        norms->resize(softBody->m_nodes.size());
        setVertexArray(verts);
        setNormalArray(norms);*/
#undef FILLINDEX
    }
//getOrCreateVertexBufferObject()->setUsage( GL_DYNAMIC_DRAW );


}


void SoftBody::addAnchor(Anchor*j)
{
///avoid duplicate
    for(std::vector<vsg::ref_ptr<Anchor> >::iterator i=_anchors.begin(); i!=_anchors.end(); i++)
        if(i->get()==j)return;

    btRigidBody * rig=0;
    if(j->getRigidBody() && (rig=j->getRigidBody()->getRigidBody()))
    {

        _anchors.push_back(vsg::ref_ptr<Anchor>(j));
            btVector3 pivot=vsgbCollision::asBtVector3(j->getLocalFrame());
            _body->appendAnchor(j->getSoftBodyNodeIndex(),rig,pivot, !j->getIsCollisionEnable());

    }
    else
    {
///joint constraint not defined yet
//debug
        std::cerr<<"Warning SoftBody anchor RigidBody not defined"<<std::endl;
    }
}
void SoftBody::removeAnchor(Anchor*j)
{
    for(std::vector<vsg::ref_ptr<Anchor> >::iterator i=_anchors.begin(); i!=_anchors.end(); i++)
    {
        if(i->get()==j)
        {

            _anchors.erase(i);
            std::vector<vsg::ref_ptr<Anchor> > tempanchors=_anchors;
            _anchors.clear();
            _body->m_anchors.clear();
            for(std::vector<vsg::ref_ptr<Anchor> >::iterator ix=tempanchors.begin(); ix!=tempanchors.end(); ix++)
                addAnchor(*ix);


            return;
        }
    }
}
void SoftBody::setWindVelocity(const vsg::vec3& w){
    btSoftBody*_body=getSoftBody();
    if(_body){
        _windVelocity=w;
        _body->setWindVelocity(asBtVector3(w));
        }
}
const vsg::vec3& SoftBody::getWindVelocity()const {
    const btSoftBody*_body=getSoftBody();
    if(_body)
        assert(_windVelocity==asVsgVec3(const_cast<btSoftBody*>(_body)->getWindVelocity()));
    return _windVelocity;
}
void SoftBody::traverse(vsg::Visitor &nv)
{
  /* switch(nv.getVisitorType())
    {
    case vsg::NodeVisitor::UPDATE_VISITOR:
    //case vsg::NodeVisitor::NODE_VISITOR:
    {
 if ( !_parentWorld)
        {
            if(getNumParents()!=0)
            {
                FindParentVisitor fpv;
                this->accept(fpv);
                if(fpv.foundWorld){
                _parentWorld=fpv.foundWorld;
                addPhysicalObjectToParentWorld();
                }else{
                 std::cerr<<"SoftBody: parent world not foudn"<<std::endl;
                }
            }
        }

        ///updateVertices according btDoftBody Nodes
        vsg::vec3Array* verts( dynamic_cast< vsg::vec3Array* >( getVertexArray() ) );

        // Update the vertex array from the soft body node array.
        const btSoftBody * _softBody=getSoftBody();
        const btSoftBody::tNodeArray& nodes = _softBody->m_nodes;
        //if(verts->size()<nodes.size())verts->resize(nodes.size());
        vsg::vec3Array::iterator it( verts->begin() );
        unsigned int idx;
        for( idx=0; idx<nodes.size(); idx++)
        {
            *it++ =  vsgbCollision::asVsgVec3( nodes[ idx ].m_x );
//std::cerr<<*(it-1)<<std::endl;
        }
        verts->dirty();
        dirtyBound();
//std::cerr<<"SoftBody: dirty"<<std::endl;

        // Generate new normals.
        SmoothingVisitor smooth;
        smooth.smooth( *this );
        getNormalArray()->dirty();
    }
    }*/
    vsg::Geometry::traverse(   nv );
}
void SoftBody::addPhysicalObjectToParentWorld()
{
    if(_parentWorld)
    {
        btSoftBody*_body=getSoftBody();
        if(_body)
        {
            btSoftRigidDynamicsWorld* w=dynamic_cast<   btSoftRigidDynamicsWorld*>(_parentWorld->getDynamicsWorld());
            if(w)            if(w->getSoftBodyArray().findLinearSearch(_body) == w->getSoftBodyArray().size())
                w->addSoftBody(_body);
            else
            {
                std::cerr<<"SoftBody: try to add SoftBody to a rigid word"<<std::endl;
            }

        }
        else
        {
            std::cerr<<"SoftBody: btSoftBody is not setted"<<std::endl;
        }
    }
    else
    {
        std::cerr<<"SoftBody: parentworld hasn't been found"<<std::endl;
    }

}



void transform(btSoftBody* soft, const vsg::mat4 & m ){
   btSoftBody::tNodeArray& nodes = soft->m_nodes;
   btTransform btm=vsgbCollision::asBtTransform(m);
   for(  unsigned int idx=0; idx<nodes.size(); idx++)
        nodes[ idx ].m_x =btm(nodes[ idx ].m_x);
}

}


