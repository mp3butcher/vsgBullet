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

#ifndef __VSGBDYNAMICS_SOFT_BODY_H__
#define __VSGBDYNAMICS_SOFT_BODY_H__ 1

#include <vsgbDynamics/Export.h>
#include <vsgbDynamics/CreationRecord.h>
#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftBody.h>

#include <vsgbCollision/RefBulletObject.h>

#include <vsg/nodes/Geometry.h>
namespace vsgbDynamics
{
/** use to filter collisions  **/
class World;
class RigidBody;
class SoftBody;
/*
class VSGBDYNAMICS_EXPORT PhysicalObject : public vsg::NodeCallback
{
public:
    PhysicalObject():_parentWorld(0) {};
    PhysicalObject( const PhysicalObject& copy, const vsg::CopyOp& copyop={} );
META_Object(vsgbDynamics,PhysicalObject)


    virtual RigidBody * asRigidBody()
    {
        return 0;
    }
    virtual SoftBody * asSoftBody()
    {
        return 0;
    }

    void operator()( vsg::Node* node, vsg::NodeVisitor* nv );


protected:

    virtual void updatematrix( vsg::Node* node, vsg::NodeVisitor* nv )=0;
    virtual void addPhysicalObjectToParentWorld()=0;
    ~PhysicalObject();
    World * _parentWorld;

};*/

class SoftBody;
class VSGBDYNAMICS_EXPORT Anchor : public  vsg::Inherit<vsg::Object,Anchor>
{
public:
    Anchor();
    Anchor( const Anchor& copy, const vsg::CopyOp& copyop={} );

    void setLocalFrame(const vsg::vec3&m){_localrig=m;}
    const vsg::vec3 & getLocalFrame()const{return _localrig;}

    void setSoftBodyNodeIndex(int i){_nodeindex=i;}
    int getSoftBodyNodeIndex()const{return _nodeindex;}

    const RigidBody *getRigidBody()const    {        return _rig;    }
     RigidBody *getRigidBody()    {        return _rig;    }
    void setRigidBody(RigidBody *c);

    bool getIsCollisionEnable()const {return _collisionenabled;}
    void setIsCollisionEnable(bool b){ _collisionenabled=b;}
protected:
    ~Anchor();
    bool _collisionenabled;
    vsg::vec3 _localrig;
    int _nodeindex;
    RigidBody* _rig;

};
class VSGBDYNAMICS_EXPORT SoftBody : public vsg::Inherit<vsg::Geometry,SoftBody>
{
public:
    SoftBody();
    SoftBody( const SoftBody& copy, const vsg::CopyOp& copyop={});

    virtual void traverse(vsg::Visitor&) override;


    ///set asscociated softbody
    ///if nodes are setted bake them to vsg
    void setSoftBody(btSoftBody*body);
    inline const btSoftBody* getSoftBody()const{return _body;}
    inline btSoftBody* getSoftBody(){return _body;}

    const World *getParentWorld()const
    {
        return _parentWorld;
    }
    void setParentWorld(World *c)
    {
        _parentWorld=c;
    }


     unsigned int getNumAnchors()const
    {
        return _anchors.size();
    }
   const Anchor * getAnchor(unsigned int i)const
    {
        return _anchors[i];
    }
    Anchor * getAnchor(unsigned int i)
    {
        return _anchors[i];
    }
    void addAnchor(Anchor*p);
    void removeAnchor(Anchor*p);

    ///mainly 4 missing bullet serialization
    void setWindVelocity(const vsg::vec3 &w);
    const vsg::vec3& getWindVelocity()const;

    ///transform underlying softbody by a matrix
    //void  transform(vsg::mat4 &)
protected:

    virtual void addPhysicalObjectToParentWorld();
    ~SoftBody();
    btSoftBody *_body;
    World * _parentWorld;
    vsg::vec3 _windVelocity;///redondancy for serializer
    std::vector<vsg::ref_ptr<Anchor> > _anchors;



};
///transform a softbody by a matrix
void VSGBDYNAMICS_EXPORT transform(btSoftBody* soft, const vsg::mat4 & m );

// vsgbDynamics
}


// __VSGBDYNAMICS_RIGID_BODY_H__
#endif
