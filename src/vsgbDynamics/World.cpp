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

#include <iostream>
#include <vsgbDynamics/World.h>
#include <vsgbDynamics/MotionState.h>

/*#include <vsgbDynamics/MotionState.h>*/
//#include <vsgbCollision/GLDebugDrawer.h>

#include <vsg/core/Visitor.h>

#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include <vsgbDynamics/RigidBody.h>
#include <vsgbDynamics/CreationRecord.h>
#include <vsgbDynamics/MotionState.h>
#include <vsgbDynamics/World.h>
#include <vsgbCollision/Utils.h>
#include <vsgbCollision/CollisionShapes.h>

#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>

#include <vsgbCollision/Utils.h>
#include <vsgbDynamics/TripleBuffer.h>
#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>

#include <vsgbCollision/ComputeTriMeshVisitor.h>

#include <BulletWorldImporter/btBulletWorldImporter.h>

#define max(a,b)    (((a) > (b)) ? (a) : (b))
#define min(a,b)    (((a) < (b)) ? (a) : (b))
#ifndef USE_REFERENCE_TIME
#   define USE_REFERENCE_TIME DBL_MAX
#endif

using namespace vsgbDynamics;

///motion state assuming drawable centered on their center of mass
class RigidBodyMotionState : public MotionState
{
public:
    virtual void setWorldTransform(const btTransform& worldTrans)
    {
        // Call the callback, if registered.
        if( _mscl.size() > 0 )
        {
            // Call only if position changed.
            const btVector3 delta( worldTrans.getOrigin() - _transform.getOrigin() );
            const btScalar eps( (btScalar)( 1e-5 ) );
            const bool quiescent( vsgbCollision::equivalent( delta[ 0 ], btScalar(0.), eps ) &&
                                 vsgbCollision::equivalent( delta[ 1 ], btScalar(0.), eps ) &&
                                 vsgbCollision::equivalent( delta[ 2 ], btScalar(0.), eps ) );
            if( !quiescent )
            {
                MotionStateCallbackList::iterator it;
                for( it = _mscl.begin(); it != _mscl.end(); ++it )
                    (**it)( worldTrans );
            }
        }

        // _transform is the model-to-world transformation used to place collision shapes
        // in the physics simulation. Bullet queries this with getWorldTransform().
        _transform = worldTrans;

        if( _tb == nullptr )
        {
            setWorldTransformInternal( worldTrans );

            const vsg::mat4 dt = vsgbCollision::asVsgMatrix( worldTrans );
            const vsg::mat4 col2ol = computeCOLocalToVsgLocal();
            const vsg::mat4 t = col2ol * dt;

            if( _mt.valid() )
                _mt->matrix=( t );
            else if( _amt.valid() )
                _amt->matrix=( t );


        }
        else
        {
            char* addr( _tb->writeAddress() );
            if( addr == nullptr )
            {
                std::cerr<< "MotionState: No TripleBuffer write address." << std::endl;
                return;
            }
            btScalar* fAddr = reinterpret_cast< btScalar* >( addr + _tbIndex );
            worldTrans.getOpenGLMatrix( fAddr );
        }
    }
};

class  FindRigids : public vsg::Inherit<vsg::Visitor, FindRigids>
{
public:
    std::vector<vsg::ref_ptr<RigidBody>> _foundrigs;

    void apply(vsg::Group& object) override{
        if(auto w=object.cast<RigidBody>())
            _foundrigs.push_back(vsg::ref_ptr<RigidBody>(w));
        object.traverse(*this);
    }
    void apply(vsg::Object& object) override{
        object.traverse(*this);
    }
};

void BulletOperation::run(){
    vsg::clock::time_point lastts = _currenttimestamp;
    _currenttimestamp = vsg::clock::now();
    auto deltatc = _currenttimestamp-lastts;
    float deltaTime = std::chrono::duration_cast<std::chrono::milliseconds>(deltatc).count() * 0.001f;
    if(_worlds.empty()){
        //seek word
        WorldFinderVisitor fw;
        _rootnode->accept(fw);
        _worlds = fw._foundworlds;

        if(_worlds.empty()) vsg::log(vsg::Logger::LOGGER_WARN, "BulletOperation: no World found in the given root");

        for(auto wit = _worlds.begin(); wit!=_worlds.end(); ++wit)
        {
            FindRigids frs;
            (*wit)->accept(frs);            

            for(auto r=frs._foundrigs.begin();r!=frs._foundrigs.end();++r)
            {
                (*wit)->vsgRigidBodies.push_back(*r);
             /*   if(!dynamic_cast<RigidBodyMotionState*>((*r)->getRigidBody()->getMotionState()))
                {
                    ///_body is not compatible with vsgBullet RigidBody so transform it

                    btMotionState * ms=(*r)->getRigidBody()->getMotionState();
                    btTransform trans;
                    trans=(*r)->getRigidBody()->getWorldTransform();

                    RigidBodyMotionState *vsgms= new RigidBodyMotionState();
                    vsg::MatrixTransform* mat=dynamic_cast<vsg::MatrixTransform*>((*r).get());
                    vsgms->setTransform(mat);
                    vsgms->setWorldTransform(trans);
                    //vsgms->setParentTransform(mat->matrix);
                    btTransform t;ms->getWorldTransform(t);

                    vsgms->setWorldTransform(ms->getWorldTransform())
                    (*r)->getRigidBody()->setMotionState(vsgms);
                    if (ms)delete ms;


                }*/
                ///addPhysicalObject
                if((*wit)->getDynamicsWorld()->getCollisionObjectArray().findLinearSearch((*r)->getRigidBody()) == (*wit)->getDynamicsWorld()->getCollisionObjectArray().size())
                {
                    (*wit)->getDynamicsWorld()->addRigidBody((*r)->getRigidBody());
                    for(uint jc=0; jc<(*r)->getNumJoints(); ++jc)
                        (*wit)->addJoint((*r)->getJoint(jc));
                }
            }
        }
    }

    for(auto wit=_worlds.begin();wit!=_worlds.end();++wit)
    {

     /*   for(auto rit=(*wit)->vsgRigidBodies.begin();rit!=(*wit)->vsgRigidBodies.end();++rit)
        {
            if((*rit)->_bulletmanagerchildcount!=(*rit)->children.size()) //physically dirty
            {
                (*rit)->_bulletmanagerchildcount=(*rit)->children.size();
                FindRigids frs;
                (*rit)->accept(frs);

                for(auto r=frs._foundrigs.begin();r!=frs._foundrigs.end();++r)
                {
                    bool dup=false;
                    //check duplicate TODO check removal impossible here?
                    for(auto rit2=(*wit)->vsgRigidBodies.begin();rit2!=(*wit)->vsgRigidBodies.end();++rit2)
                        if((*rit2) == (*r)) dup=true;
                    if(!dup) (*wit)->vsgRigidBodies.push_back(*r);

                }
            }
        }*/

        //update World
     if((*wit)->getDynamicsWorld())
        if (deltaTime >10.)
            (*wit)->getDynamicsWorld()->stepSimulation( min ( max(0.001, 0.0001), 10));
        else
            (*wit)->getDynamicsWorld()->stepSimulation(deltaTime);

        //update rigids
        //(*wit)->vsgRigidBodies;

        FindRigids frs;
        (*wit)->accept(frs);

        for(auto r=frs._foundrigs.begin();r!=frs._foundrigs.end();++r)
        {
            bool dup=false;
            //check duplicate TODO check removal impossible here?
            for(auto rit2=(*wit)->vsgRigidBodies.begin();rit2!=(*wit)->vsgRigidBodies.end();++rit2)
                if((*rit2) == (*r)) dup=true;
            if(!dup) {
                (*wit)->vsgRigidBodies.push_back(*r);
                ///addPhysicalObject
                if((*wit)->getDynamicsWorld()->getCollisionObjectArray().findLinearSearch((*r)->getRigidBody()) == (*wit)->getDynamicsWorld()->getCollisionObjectArray().size())
                {
                    (*wit)->getDynamicsWorld()->addRigidBody((*r)->getRigidBody());
                    for(uint jc=0; jc<(*r)->getNumJoints(); ++jc)
                        (*wit)->addJoint((*r)->getJoint(jc));
                }
            }

        }

    }

    //deltat.count()
   /* while(!_joints2add.empty())
    {
        addJoint(_joints2add.back());
        _joints2add.pop_back();
    }

    if (_debugdraw){

    }
    const vsg::FrameStamp* fs = nv.getFrameStamp();
    if (_deltaTime >10)

    _btworld->stepSimulation( min ( max(fs->getReferenceTime() - _prevousReferenceTime, 0.0001), 10));
    else{
        _btworld->stepSimulation(_deltaTime);

    }
    _prevousReferenceTime = fs->getReferenceTime();*/
}

//vsg::ref_ptr<vsg::Geode> World::_debugdrawable=0;
World::World()
    : vsg::Inherit<vsg::Group, World>(), _debugdraw(false), _btworld(nullptr), _deltaTime( USE_REFERENCE_TIME)
{
    setWorldType(RIGID_ONLY);
    //dataVariance=(vsg::Object::DYNAMIC);
}

World::~World()
{
    if( _btworld)
        delete _btworld;
}

World::World( const World& copy, const vsg::CopyOp& copyop )
    :   vsg::Inherit<vsg::Group, World>(copy, copyop)
{
	_debugdraw = false;
}

vsg::ref_ptr<World> vsgbDynamics::World::currentserializedworld=nullptr;

class vsgbtBulletWorldImporter:public btBulletWorldImporter
{
public:

    vsgbtBulletWorldImporter(btDynamicsWorld* world=0):	btBulletWorldImporter( world) {}
    void convertConstraintFloat(btTypedConstraintFloatData* constraintData, btRigidBody* rbA, btRigidBody* rbB, int fileVersion)
    {
        btBulletWorldImporter::convertConstraintFloat(constraintData,  rbA,  rbB,  fileVersion);
    }
    void convertConstraintDouble(btTypedConstraintDoubleData* constraintData, btRigidBody* rbA, btRigidBody* rbB, int fileVersion)
    {
        btBulletWorldImporter::convertConstraintDouble(constraintData,  rbA,  rbB,  fileVersion);
    }
};

void World::read(vsg::Input& input)
{
    currentserializedworld= (this);
    unsigned int sizel = 0;
    World::WorldType wtype;
    input.readValue<World::WorldType>("worldtype",wtype);
    setWorldType(wtype);

    vsgbtBulletWorldImporter BulletImporter( getDynamicsWorld() );
    vsg::ref_ptr<vsg::ubyteArray> memoryBuffer = input.readObject<vsg::ubyteArray>("bufferString");
    bool result = BulletImporter.loadFileFromMemory((char*)memoryBuffer->dataPointer(), memoryBuffer->size());

    currentserializedworld = (this);
    Group::read(input);
}

class btDiscreteDynamicsWorldHacker :public btDiscreteDynamicsWorld{
public: void serializeWorldInfo(btSerializer* serializer){ serializeDynamicsWorldInfo( serializer); }
};
class btSoftRigidDynamicsWorlddHacker :public btSoftRigidDynamicsWorld{
public: void serializeWorldInfo(btSerializer* serializer){  serializeDynamicsWorldInfo( serializer);}
};

void World::write(vsg::Output& output) const
{


    output.writeValue<World::WorldType>("worldtype",_worldtype);

    btDefaultSerializer* serializer = new btDefaultSerializer();
    currentserializedworld=const_cast<  World *>(this);
    // start the serialization and serialize the trimeshShape
    serializer->startSerialization();
    const btDiscreteDynamicsWorld * w=getDynamicsWorld();
    if(getWorldType()== RIGID_AND_SOFT)
        static_cast<btSoftRigidDynamicsWorlddHacker*>(const_cast<btDiscreteDynamicsWorld *>(w))->serializeWorldInfo(serializer);
    else
        static_cast<btDiscreteDynamicsWorldHacker*>(const_cast<btDiscreteDynamicsWorld *>(w))->serializeWorldInfo(serializer);


    /*btCollisionObject *colObj=psb;
                int len = colObj->calculateSerializeBufferSize();
                btChunk* chunk = serializer->allocate(len,1);
                const char* structType = colObj->serialize(chunk->m_oldPtr, serializer);
                serializer->finalizeChunk(chunk,structType,BT_SOFTBODY_CODE,colObj);
    */


    //psb->serializeSingleObject(serializer);
    //psb->getCollisionShape()->serializeSingleShape(serializer);

    serializer->finishSerialization();

    auto bufferstring = vsg::ubyteArray::create(serializer->getCurrentBufferSize());
    memcpy(bufferstring->dataPointer(), serializer->getBufferPointer(), serializer->getCurrentBufferSize());
    output.writeObject("bufferString", bufferstring);
    output.objectIDMap.erase(bufferstring);//remove local variable from cache (next bufferstring will land on same adress)

    Group::write(output);

}

void World::setWorldType(WorldType t)
{
    if(_worldtype!=t || !_btworld)
    {
        _worldtype=t;
///init world
        // btDefaultCollisionConfiguration * collisionConfiguration =0;
        btCollisionDispatcher * dispatcher = 0;
        btConstraintSolver * solver = new btSequentialImpulseConstraintSolver;

        btVector3 worldAabbMin( -1000, -1000, -1000 );
        btVector3 worldAabbMax( 1000, 1000, 1000 );
        btBroadphaseInterface * inter = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );

        // btDynamicsWorld * dynamicsWorld = new btDiscreteDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );

        /*

        btVector3 gravity( 0, 0, -10.17 );
        _btworld->setGravity( gravity );*/


        switch(_worldtype)
        {
//case RIGID_ONLY: _btworld=new btDiscreteDynamicsWorld();break;
        case RIGID_AND_SOFT:
        {
            btSoftBodyRigidBodyCollisionConfiguration* collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
            dispatcher = new btCollisionDispatcher( collisionConfiguration );
            _btworld=new btSoftRigidDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );
            //   _btworld->getWorldInfo().m_gravity = gravity;
            ((btSoftRigidDynamicsWorld*) _btworld)->getWorldInfo().m_broadphase = inter;
            ((btSoftRigidDynamicsWorld*) _btworld)->getWorldInfo().air_density = btScalar( 1.2 );
            ((btSoftRigidDynamicsWorld*) _btworld)->getWorldInfo().water_density = 0;
            ((btSoftRigidDynamicsWorld*) _btworld)->getWorldInfo().water_offset = 0;
            ((btSoftRigidDynamicsWorld*) _btworld)->getWorldInfo().water_normal = btVector3( 0, 0, 0 );
            ((btSoftRigidDynamicsWorld*) _btworld)->getWorldInfo().m_sparsesdf.Initialize();
            ((btSoftRigidDynamicsWorld*) _btworld)->getWorldInfo().m_dispatcher = dispatcher;
        }
        break;
        default:
        {
            btDefaultCollisionConfiguration*  collisionConfiguration = new btDefaultCollisionConfiguration();
            dispatcher = new btCollisionDispatcher( collisionConfiguration );
            _btworld=new btDiscreteDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );
        }
        }

        _btworld->setGravity( btVector3( 0, 0, -10 ) );
         //      _btworld->stepSimulation( 0.1);
    }

}
/*
void World::update( vsg::Node* node, vsg::NodeVisitor* nv )
{
    const vsg::FrameStamp* fs = nv->getFrameStamp();
    if ( _deltaTime==USE_REFERENCE_TIME )
        _btworld->stepSimulation( fs->getReferenceTime() - _prevousReferenceTime);
    else
        _btworld->stepSimulation( _deltaTime );
    _prevousReferenceTime = fs->getReferenceTime();

}*/

/*
class  DebugDrawable:public vsg::Geometry
{
public:
    DebugDrawable() {}
    DebugDrawable(const DebugDrawable&x,vsg::CopyOp op={}) {}
    META_Node(vsgbDynamics,DebugDrawable)
    DebugDrawable(vsgbCollision::GLDebugDrawer *d,btDiscreteDynamicsWorld*w ):dbgDraw(d),dynamicsWorld(w) {}
    virtual void drawImplementation(vsg::RenderInfo& ) const
    {
     //   dbgDraw->BeginDraw();

        dynamicsWorld->debugDrawWorld();
      //  dbgDraw->EndDraw();

    }

protected:
    vsgbCollision::GLDebugDrawer *dbgDraw;
    btDiscreteDynamicsWorld*dynamicsWorld;

};
*/
void World::addJoint(Joint*j)
{
///avoid duplicate
    for(std::vector<vsg::ref_ptr<Joint> >::iterator i=_joints.begin(); i!=_joints.end(); i++)
        if(i->get()==j)return;

    if(j->getConstraint())
    {
        if(_btworld)
        {
            _joints.push_back(vsg::ref_ptr<Joint>(j));
            _btworld->addConstraint(j->getConstraint());


           // std::cerr<<"joint constraint added"<<std::endl;
        }
        else
        {
            _joints2add.push_back(vsg::ref_ptr<Joint>(j));
        }
        if(j->getBodyA())j->getBodyA()->addJoint(j);
        if(j->getBodyB())j->getBodyB()->addJoint(j);
    }
    else
    {
///joint constraint not defined yet
//debug
        std::cerr<<"Warning joint constraint not defined yet"<<std::endl;
    }
}
void World::removeJoint(Joint*j)
{
    for(std::vector<vsg::ref_ptr<Joint> >::iterator i=_joints.begin(); i!=_joints.end(); i++)
    {
        if(i->get()==j)
        {
            _btworld->removeConstraint(j->getConstraint());
            vsg::ref_ptr<Joint> ptrj=vsg::ref_ptr<Joint>(j);
            _joints.erase(i);
            if(j->getBodyA())j->getBodyA()->removeJoint(j);
            if(j->getBodyB())j->getBodyB()->removeJoint(j);

            return;
        }
    }
}

void World::setDebugEnabled(bool b)
{
    if (_debugdraw==b)return;
    _debugdraw=b;
		/*vsgbCollision::GLDebugDrawer *dbgDraw = (vsgbCollision::GLDebugDrawer*)_debugdrawable->getChild(0);
		dbgDraw->getSceneGraph();
		dbgDraw->setEnabled(b);*/
    if(_debugdraw)
    { 
        /* dbgDraw = new vsgbCollision::GLDebugDrawer();
		dbgDraw->setDebugMode(~btIDebugDraw::DBG_DrawText);
		dbgDraw->setEnabled(b);
       _btworld->setDebugDrawer( dbgDraw );*/
        _debugdrawable=new vsg::Group();
		 //_debugdrawable->addChild(dbgDraw->getSceneGraph());


    }
    else
    {

        if(_btworld->getDebugDrawer())
		{	//vsgbCollision::GLDebugDrawer *dbgDraw = (vsgbCollision::GLDebugDrawer*)_debugdrawable->getChild(0);
		 
	//	dbgDraw->setEnabled(b);
            delete _btworld->getDebugDrawer();
            _btworld->setDebugDrawer( 0);

        }
    }

}
void World::traverse(vsg::RecordTraversal &nv) const
{
   vsg::Inherit<vsg::Group, World>::traverse(nv);
  /*  ///wordupdate done by BulletManager
    switch(nv.getVisitorType())
    {

    case vsg::NodeVisitor::UPDATE_VISITOR:
        ///update the world
    {
        while(!_joints2add.empty())
        {
            addJoint(_joints2add.back());
            _joints2add.pop_back();
        }
		
		if (_debugdraw){

		}
        const vsg::FrameStamp* fs = nv.getFrameStamp();
		if (_deltaTime >10)

			_btworld->stepSimulation( min ( max(fs->getReferenceTime() - _prevousReferenceTime, 0.0001), 10));
		else{
			_btworld->stepSimulation(_deltaTime);
		 
		}
        _prevousReferenceTime = fs->getReferenceTime();

		if (_debugdraw){
			
		}
        vsg::Group::traverse(nv);
		if (_debugdraw){
			dbgDraw->BeginDraw();
	    	_btworld->debugDrawWorld(); dbgDraw->EndDraw();dbgDraw->getSceneGraph()->accept(nv);
		}
    }
    break;
    case vsg::NodeVisitor::CULL_VISITOR:
		vsg::Group::traverse(nv);
		if (_debugdraw)
		{
		dbgDraw->getSceneGraph()->accept(nv); 
		 
		}
		
        break;
	default:	
        vsg::Group::traverse(nv);
		if (_debugdraw)
	{
		dbgDraw->getSceneGraph()->accept(nv);

	}
    }
*/
}


