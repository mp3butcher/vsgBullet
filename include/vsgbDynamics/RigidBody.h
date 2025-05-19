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

#ifndef __VSGBDYNAMICS_RIGID_BODY_H__
#define __VSGBDYNAMICS_RIGID_BODY_H__ 1
#include <vsgbDynamics/Export.h>

#include <vsgbDynamics/CreationRecord.h>
#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftBody.h>

#include <vsg/core/Inherit.h>

#include <vsg/nodes/AbsoluteTransform.h>
#include <vsg/nodes/MatrixTransform.h>
#include <vsg/utils/PrimitiveFunctor.h>
#include <vsgbCollision/RefBulletObject.h>

#include <vsgbDynamics/Joint.h>

namespace vsgbDynamics
{
/** use to filter collisions  **/
class World;
typedef unsigned int CollisionMaskType;
class VSGBDYNAMICS_EXPORT CollisionMask: public vsg::Inherit<vsg::Object,CollisionMask>
{
public:
    CollisionMask(): _group(0),_mask(0) {}
    CollisionMask( const CollisionMask& copy, const vsg::CopyOp& copyop={}):vsg::Inherit<vsg::Object,CollisionMask>(copy,copyop),_group(copy._group),_mask(copy._mask) {    }

    inline CollisionMaskType getGroupID()const
    {
        return _group;
    }
    inline void setGroupID( CollisionMaskType g)
    {
        _group=g;
    }
    inline CollisionMaskType getMask()const
    {
        return _mask;
    }
    inline void setMask( CollisionMaskType g)
    {
        _mask=g;
    }
protected:
    CollisionMaskType _group,_mask;
};


class VSGBDYNAMICS_EXPORT RigidBody : public vsg::Inherit<vsg::MatrixTransform, RigidBody>// vsg::NodeCallback
{
public:

    //dirty flag for Bullet Manager
    uint _bulletmanagerchildcount=0;


    //void traverse(vsg::RecordTraversal& visitor) const override;

    RigidBody();
    RigidBody( const RigidBody& copy, const vsg::CopyOp& copyop={});


    inline void setRigidBody(btRigidBody*body)
    {
        _body=body;
    }
    inline const btRigidBody* getRigidBody()const
    {
        return _body;
    }
    inline btRigidBody* getRigidBody()
    {
        return _body;
    }
    //virtual void operator()( vsg::Node* node, vsg::NodeVisitor* nv );
    const World *getParentWorld()const
    {
        return _parentWorld;
    }
    void setParentWorld(World *c)
    {
        _parentWorld=c;
    }
    unsigned int getNumJoints()const
    {
        return _joints.size();
    }
    const Joint * getJoint(unsigned int i)const
    {
        return _joints[i];
    }
    Joint * getJoint(unsigned int i)
    {
        return _joints[i];
    }
    void addJoint(Joint*p);
    void removeJoint(Joint*p);


    void read(vsg::Input& input) override;
    void write(vsg::Output& output) const override;
protected:
    virtual void addPhysicalObjectToParentWorld();
    // virtual void updatematrix( vsg::Node* node, vsg::NodeVisitor* nv );
    ~RigidBody();

    // vsgbDynamics::CreationRecord* cr;

    //btCollisionShape* _shape ;
    btRigidBody *_body;
    World * _parentWorld;
    std::vector< vsg::ref_ptr<Joint> > _joints;

};

/** \defgroup rigidbody Rigid Body Creation
\brief Convenience routines for creating Bullet rigid bodies from scene graphs.

These functions use the CreationRecord struct to create collision shapes,
rigid bodies, and motion state objects. The application is responsible for
setting the CreationRecord fields as needed. The fields are used as follows:

<ul>
  <li> \c _sceneGraph Geometric source data for collision shape and rigid body creation.
    <ul>
      <li>Passed to vsgbCollision::btCompoundShapeFromVSGGeodes() to create a collision shape.</li>
      <li>If \c _comSet is false, the \c _sceneGraph bounding sphere center is used as the center of mass.</li>
      <li>If \c _sceneGraph is a Transform node, it is set as the managed Transform node in MotionState (MotionState::setTransform()).</li>
    </ul>
  </li>
  <li> \c _com When \c _comSet is true, \c _com is the center of mass. Otherwise, the bounding volume
       center is used as the center of mass. (See CreationRecord::setCenterOfMass().)
    <ul>
      <li>The negated center of mass multiplied by the \c _scale vector is used as a transform for geometric data during collision shape creation.
      <li>The center of mass is passed to the created MotionState object (MotionState::setCenterOfMass()).
    </ul>
  </li>
  <li> \c _scale \em xyz scale vector. (vsgBullet supports non-uniform scaling.)
    <ul>
      <li>The negated center of mass multiplied by the \c _scale vector is used as a transform for geometric data during collision shape creation.
      <li>\c _scale is passed to the created MotionState object (MotionState::setScale()).
    </ul>
  </li>
  <li> \c _parentTransform Used to specify an initial position. It is set as the MotionState parent transform (MotionState::setParentTransform()).
  </li>
  <li> \c _shapeType Passed to vsgbCollision::btCompoundShapeFromVSGGeodes().
  </li>
  <li> \c _mass
    <ul>
      <li>Passed to btCollisionShape::calculateLocalInertia().
      <li>Set in the created rigid body via btRigidBody::btRigidBodyConstructionInfo.
    </ul>
  </li>
  <li> \c _restitution Set in the created rigid body via btRigidBody::btRigidBodyConstructionInfo::m_restitution.
  </li>
  <li> \c _friction Set in the created rigid body via  btRigidBody::btRigidBodyConstructionInfo::m_friction.
  </li>
  <li> \c _axis Passed to vsgbCollision::btCompoundShapeFromVSGGeodes().
    Ultimately, it is referenced only if \c _shapeType is \c CYLINDER_SHAPE_PROXYTYPE.
  </li>
  <li> \c _reductionLevel Passed to vsgbCollision::btCompoundShapeFromVSGGeodes().
    If \c _shapeType is \c TRIANGLE_MESH_SHAPE_PROXYTYPE or CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE,
    this value is used to configure osgWorks geometry reduction utilities to reduce triangle
    count prior to creating the collision shape.
  </li>
</ul>

Note that these are merely convenience routines, and your application can interface
directly with Bullet to create collision shapes and rigid bodies. However, you should
strongly consider using these convenience routines for the following reasons:

\li The routines are based on the CreationRecord class, which supports the dot VSG
file format. This allows your application to save and restore creation paramters.
\li These routines support non-origin center of mass.
\li These routines support scaled collision shapes.
\li These routines support initial transformations (for example, from the
accumulated VSG local-to-world matrix in the subgraph parent's NodePath).
\li These routines automatically create a MotionState object to keep the VSG visual
representation in sync with the Bullet physics representation.

<b>Requirements</b>

\li The root node of the subgraph (CreationRecord::_sceneGraph) must be either an
vsg::MatrixTransform or an vsg::AbsoluteTransform .
Other vsg::Transform-derived nodes are not supported.

<b>Functionality Removed in v2.0</b>

\li The <em>overall</em> feature created a single collision shape around the entire
subgraph. You can still obtain this same functionality by calling the
\link collisionshapes collision shape functions \endlink directly, then
passing the created collision shape to createRigidBody().
\li The <em>named node</em> feature searched the subgraph for the named node, then
used that node as the basis for creating the collision shape. The application is
now responsible for doing this work. The FindNamedNode visitor in osgWorks can be
used to find nodes with a specific name.

*/
/*@{*/


/** \brief Creates a compound collision shape and rigid body from the CreationRecord data.

Uses the vsgbCollision::ComputeShapeVisitor to create a btCompoundShape from CreationRecord::_sceneGraph.
Currently, a shape per Geode is created. CreationRecord::_shapeType specifies the shape type created per Geode.
If CreationRecord::_shapeType is CYLINDER_SHAPE_PROXYTYPE, CreationRecord::_axis specifies the cylinder major axis.
*/
 VSGBDYNAMICS_EXPORT btRigidBody*createRigidBody( vsgbDynamics::CreationRecord* cr );

/** \overload
<p>
Use this function to create a rigid body if you have already created the collision shape.
This is useful for sharing collision shapes.
</p>
*/
 VSGBDYNAMICS_EXPORT btRigidBody*createRigidBody( vsgbDynamics::CreationRecord* cr, btCollisionShape* shape );

/**@}*/

class VSGBDYNAMICS_EXPORT ConvexDecompositionParams
{

public:
    ConvexDecompositionParams( unsigned int depth = 5,
                               float cpercent     = 5,
                               float ppercent     = 15,
                               unsigned int maxv  = 16,
                               float skinWidth=0)
        :  mDepth(depth),mCpercent(cpercent),mPpercent(ppercent),mMaxVertices(maxv),mSkinWidth(skinWidth) {}

    unsigned int getDepth()const
    {
        return mDepth;
    }
    void setDepth(unsigned int d)
    {
        mDepth=d;
    }
    float getConcavityPercentage()const
    {
        return mCpercent;
    }
    void setConcavityPercentage(float d)
    {
        mCpercent=d;
    }
    float getVolumeConservationPercent()const
    {
        return mPpercent;
    }
    void setVolumeConservationPercent(float d)
    {
        mPpercent=d;
    }
    unsigned int getMaxVerticesPerHull()const
    {
        return mMaxVertices;
    }
    void setMaxVerticesPerHull(unsigned int d)
    {
        mMaxVertices=d;
    }
    float getSkinWidth()const
    {
        return mSkinWidth;
    }
    void setSkinWidth(float d)
    {
        mSkinWidth=d;
    }

protected:
    unsigned int  mDepth;    // depth to split, a maximum of 10, generally not over 7.
    float         mCpercent; // the concavity threshold percentage.  0=20 is reasonable.
    float         mPpercent; // the percentage volume conservation threshold to collapse hulls. 0-30 is reasonable.

    // hull output limits.
    unsigned int  mMaxVertices; // maximum number of vertices in  output hulls. Recommended 32 or less.
    float         mSkinWidth;   // a skin width to apply to the output hulls.

};

///recursively create rigidBodies for each child geometry found
///assume convex TODO test concavity
class VSGBDYNAMICS_EXPORT CreateRigidVisitor: public vsg::Inherit<vsg::Visitor,CreateRigidVisitor>
{
public:
    CreateRigidVisitor():vsg::Inherit<vsg::Visitor,CreateRigidVisitor>()
    {
        _result=new vsg::Group;
        _overallcr=new CreationRecord;//prevent crash
        _totalvolume=0;
    }

    ///overall CreationRecord (with the overall mass that will be devided among children)
    void setOverallCreationRecord(CreationRecord*cr){_overallcr=cr;}
    CreationRecord* getOverallCreationRecord()const {return _overallcr;}

    //virtual void apply(vsg::Geode&);

    virtual void apply(vsg::Group&);
    vsg::Group * getResult();
protected:
    std::vector< std::pair<vsg::Command*,vsg::mat4> > _collecteddrawables;
    vsg::ref_ptr<vsg::Group> _result;
    vsg::ref_ptr<CreationRecord> _overallcr;
    float _totalvolume;

};

///create bullet dynamics for osganimation skeleton
///collect riggeometry and bones
class VSGBDYNAMICS_EXPORT CreateRigidFromSkeletonVisitor: public vsg::Inherit<vsg::Visitor,CreateRigidFromSkeletonVisitor>
{
public:
   CreateRigidFromSkeletonVisitor():vsg::Inherit<vsg::Visitor,CreateRigidFromSkeletonVisitor>()
    {
        _result=new vsg::Group;
        _overallcr=new CreationRecord;//prevent crash
        _totalvolume=0;
    }

    ///overall CreationRecord (with the overall mass that will be devided among children)
    void setOverallCreationRecord(CreationRecord*cr){_overallcr=cr;}
    CreationRecord* getOverallCreationRecord()const {return _overallcr;}

    virtual void apply(vsg::Group&);

   // virtual void apply(vsg::Group&);

    ///find the dominant bone of each riggeometry
    ///for each dominant bone create collision based on rigeometries
    ///add a rigidbody callback
    void computeRig();
protected:
    std::vector< std::pair<vsg::Geometry*,vsg::mat4> > _collecteddrawables;
  //  std::vector< Bone* > _collectedbones;
    vsg::ref_ptr<vsg::Group> _result;
    vsg::ref_ptr<CreationRecord> _overallcr;
    float _totalvolume;

};
///Add breakable fixed constraint on all Children (that have rigidBodies overlapping)
///use a temporary btBulletWorld underthe wood (to retrieve overlapping pairs)
class VSGBDYNAMICS_EXPORT AttachRigidVisitor : public vsg::Inherit<vsg::Visitor,AttachRigidVisitor>
{
public:
AttachRigidVisitor();
~AttachRigidVisitor();
    virtual void apply( vsg::MatrixTransform& node );
    float  getBreakThreshold()const {return _breakthreshold;}
    void setBreakThreshold(float thresh)
    {
        _breakthreshold=thresh;
    }
///use a 6DOFConstraint instead of a Fixed Constraint
    bool  getUseGenericConstraint()const {return _useGenericConstraint;}
    void setUseGenericConstraint(bool b)
    {
        _useGenericConstraint=b;
    }
void generateConstraints();
protected:
    bool _useGenericConstraint;
    float _breakthreshold;
    btDiscreteDynamicsWorld* m_dynamicsWorld;
    std::map<btRigidBody*,RigidBody*> rigs;
};


///TriangleFunctor
		/*btScalar volume = btScalar(0.);
		btVector3 com(0., 0., 0.);
		for (j=0; j < numFaces; j++) {
			const btConvexHullComputer::Edge* edge = &convexHC->edges[convexHC->faces[j]];
			v0 = edge->getSourceVertex();
			v1 = edge->getTargetVertex();
			edge = edge->getNextEdgeOfFace();
			v2 = edge->getTargetVertex();
			while (v2 != v0) {
				// Counter-clockwise triangulated voronoi shard mesh faces (v0-v1-v2) and edges here...
				btScalar vol = convexHC->vertices[v0].triple(convexHC->vertices[v1], convexHC->vertices[v2]);
				volume += vol;
				com += vol * (convexHC->vertices[v0] + convexHC->vertices[v1] + convexHC->vertices[v2]);
				edge = edge->getNextEdgeOfFace();
				v1 = v2;
				v2 = edge->getTargetVertex();
			}
		}
		com /= volume * btScalar(4.);
		volume /= btScalar(6.);*/

struct ComputeVolume{
ComputeVolume():_vol(0){};
 inline void  operator()( const vsg::vec3 v1, const vsg::vec3 v2, const vsg::vec3 v3, bool _temp )
    {
    	_vol+=v1[0] * (v2[1] * v3[2] - v2[2] * v3[1]) +
			v1[1] * (v2[2] * v3[0] - v2[0] * v3[2]) +
			v1[2] * (v2[0] * v3[1] - v2[1] * v3[0]);
    }
float getComputedVolume()const{return _vol/6.0;}
protected:
    float _vol;
};
struct ComputeCenterOfMass{
ComputeCenterOfMass():_vol(0),_com(vsg::vec3()){};
 inline void  operator()( const vsg::vec3 v1, const vsg::vec3 v2, const vsg::vec3 v3, bool _temp )
    {float vol=v1[0] * (v2[1] * v3[2] - v2[2] * v3[1]) +
			v1[1] * (v2[2] * v3[0] - v2[0] * v3[2]) +
			v1[2] * (v2[0] * v3[1] - v2[1] * v3[0]);
			_vol+=vol;
            //todo _com += (v1+v2+v3) * vol;
    }
vsg::vec3 getComputedCOM()const{return _com*(1.0f/(4.0f*_vol)) ;}
float getComputedVolume()const{return _vol/6.0f;}
protected:
    float _vol;
    vsg::vec3 _com;
};

typedef vsg::PrimitiveFunctor<ComputeCenterOfMass> ComputeCenterOfMassFunctor;
typedef vsg::PrimitiveFunctor<ComputeVolume> ComputeVolumeFunctor;




///Decompose a Geometry into Convex Geometrys
VSGBDYNAMICS_EXPORT vsg::Group*   convexDecomposition(vsg::Geometry* g,const ConvexDecompositionParams& params);



 VSGBDYNAMICS_EXPORT vsg::Group* fractureCollisionShape(vsg::Geometry* g,vsg::vec3Array*usersamples,bool useGenericConstraint=false, bool useMpr=false );

// vsgbDynamics
}


// __VSGBDYNAMICS_RIGID_BODY_H__
#endif
