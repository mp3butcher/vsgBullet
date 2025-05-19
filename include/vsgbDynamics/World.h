
// Written by J Valentin

#ifndef VSGBDYNAMICS_BTWORLD
#define VSGBDYNAMICS_BTWORLD 1

#include <vsg/nodes/Group.h>
#include <vsg/threading/OperationQueue.h>
#include <vsg/ui/UIEvent.h>
#include <vsgbDynamics/RigidBody.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
namespace vsgbCollision{
class GLDebugDrawer;
}
namespace vsgbDynamics
{


static vsg::ref_ptr<World> currentserializedworld;// for read write

/** The basic element of the physics abstract layer */
class VSGBDYNAMICS_EXPORT  World : public vsg::Inherit<vsg::Group, World>
{
public:
    typedef enum
    {
        RIGID_ONLY,
        RIGID_AND_SOFT
    } WorldType;
    World();
    World( const World& copy, const vsg::CopyOp& copyop={});


    void read(vsg::Input& input) override;
    void write(vsg::Output& output) const override;
 /* virtual vsg::Object* cloneType() const { return new World (); }
        virtual vsg::Object* clone(const vsg::CopyOp& copyop) const { return new World (*this,copyop); }
        virtual bool isSameKindAs(const vsg::Object* obj) const { return dynamic_cast<const World *>(obj)!=nullptr; }
        virtual const char* className() const { return "World"; }
        virtual const char* libraryName() const { return "vsgbDynamics"; }
        virtual void accept(vsg::NodeVisitor& nv) { if (nv.validNodeMask(*this)) { nv.pushOntoNodePath(this);
         //nv.apply(*this);
        ///forcedupdatetraversal
        if(nv.getVisitorType()==vsg::NodeVisitor::UPDATE_VISITOR)traverse(nv);
        else nv.apply(*this);
         //if (nv.getTraversalMode()==vsg::NodeVisitor::TRAVERSE_PARENTS) nv.apply(*this);
           // else if (nv.getTraversalMode()!=vsg::NodeVisitor::TRAVERSE_NONE) traverse(nv);
        nv.popFromNodePath();
        } }*/


    btDiscreteDynamicsWorld * getDynamicsWorld()
    {
        return _btworld;
    }
    const btDiscreteDynamicsWorld * getDynamicsWorld()const
    {
        return _btworld;
    }
    void  setDynamicsWorld(btDiscreteDynamicsWorld *w)
    {
        _btworld=w  ;
    }

    WorldType getWorldType()const
    {
        return _worldtype;
    }
    void setWorldType(WorldType t);

    /** Update the element */
//   virtual void update( vsg::Node* node, vsg::NodeVisitor* nv );

    /** Do some post work if required */
    //virtual void postevent( vsg::Node* node, vsg::NodeVisitor* nv ) {}

//    virtual void operator()( vsg::Node* node, vsg::NodeVisitor* nv );


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

    double getDeltaTime()const
    {
        return _deltaTime;
    }
    void setDeltaTime(double d)
    {
        _deltaTime=d;
    }

    void setDebugEnabled(bool b);

    bool getDebugEnabled()const{return _debugdraw;}


    void traverse(vsg::RecordTraversal& visitor) const override;

    std::vector<vsg::ref_ptr<RigidBody> > vsgRigidBodies;
protected:
    virtual ~ World();
    bool _debugdraw;
    WorldType _worldtype;
    double _deltaTime,_prevousReferenceTime;
    btDiscreteDynamicsWorld* _btworld;
    std::vector<vsg::ref_ptr<Joint> > _joints;
    std::vector<vsg::ref_ptr<Joint> > _joints2add; ///temporary store in case world isn't setted  (parsed on update traversal)
    ///inner debug draw
    vsg::ref_ptr<vsg::Group> _debugdrawable;
    vsgbCollision::GLDebugDrawer* dbgDraw;
};

///Node visitor used to find world..
class  VSGBDYNAMICS_EXPORT WorldFinderVisitor  : public vsg::Inherit<vsg::Visitor, WorldFinderVisitor>
{
public:
    std::vector<vsg::ref_ptr<World>> _foundworlds;

    void apply(vsg::Group& object) override{
        if(auto w=object.cast<World>())
            _foundworlds.push_back(vsg::ref_ptr<World>(w));
        object.traverse(*this);
    }
};


class VSGBDYNAMICS_EXPORT  BulletOperation : public vsg::Inherit<vsg::Operation, BulletOperation> {
public:
    BulletOperation(vsg::Group*g):_rootnode(vsg::ref_ptr<vsg::Group>(g)){};

    vsg::clock::time_point _currenttimestamp;
    void run() override;
protected:
    vsg::ref_ptr<vsg::Group> _rootnode;
    std::vector<vsg::ref_ptr<World> > _worlds;
};
}

#endif
