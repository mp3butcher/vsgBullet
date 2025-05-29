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


#include "vsgbDynamics/GroundPlane.h"
#include <vsg/all.h>

#ifdef vsgXchange_FOUND
#    include <vsgXchange/all.h>
#endif

#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>
#include <vsgbDynamics/MotionState.h>
#include <vsgbCollision/CollisionShapes.h>
#include <vsgbCollision/ComputeShapeVisitor.h>
#include <vsgbCollision/RefBulletObject.h>
#include <vsgbCollision/Utils.h>
#include <vsgbDynamics/RigidBodyAnimation.h>
#include <vsgbDynamics/World.h>
#include <vsgbInteraction/LaunchHandler.h>
#include <vsgbInteraction/DragHandler.h>
#include <vsg/app/CompileManager.h>
/*
vsg::AnimationPath * createAnimationPath( const vsg::vec3 & center,
                                          float radius,
                                          double looptime )
{

    vsg::AnimationPath * animationPath = new vsg::AnimationPath;

    animationPath->setLoopMode( vsg::AnimationPath::LOOP );

    int numSamples = 40;
    float yaw = 0.0f;
    float yaw_delta = 2.0f * vsg::PI / ( ( float )numSamples - 1.0f );
    float roll = vsg::inDegrees( 30.0f );

    double time = 0.0f;
    double time_delta = looptime / ( double )numSamples;
    for( int i = 0; i < numSamples; ++i )
    {
        vsg::vec3 position( center + vsg::vec3( sinf( yaw ) * radius, cosf( yaw ) * radius, 0.0f ) );
        vsg::quat rotation( vsg::quat( roll, vsg::vec3( 0.0, 1.0, 0.0 ) ) * vsg::quat( -( yaw + vsg::inDegrees( 90.0f ) ), vsg::vec3( 0.0, 0.0, 1.0 ) ) );

        animationPath->insert( time, vsg::AnimationPath::ControlPoint( position, rotation ) );

        yaw += yaw_delta;
        time += time_delta;
    }
    return( animationPath );
}
*/
vsg::ref_ptr<vsg::MatrixTransform> createVSGBox( vsg::vec3 size )
{
    vsg::Builder builder;
    vsg::GeometryInfo geomInfo;
    vsg::StateInfo stateInfo;

    geomInfo.color = vsg::vec4{1, 1, 1, 1};


    geomInfo.dx*=size[0]*2.;
    geomInfo.dy*=size[1]*2.;
    geomInfo.dz*=size[2]*2.;

    auto node = builder.createBox(geomInfo, stateInfo);

    auto mt=vsg::MatrixTransform::create();
    mt->addChild(node);
    return mt;
}

btRigidBody * createBTBox( vsg::MatrixTransform * box,
                         vsg::dvec3 center )
{
    btCollisionShape * collision = vsgbCollision::btBoxCollisionShapeFromVSG( box );

    vsgbDynamics::MotionState * motion = new vsgbDynamics::MotionState();
    motion->setTransform( box );
    motion->setParentTransform( vsg::translate( center ) );

    btScalar mass( 0.0 );
    btVector3 inertia( 0, 0, 0 );
    btRigidBody::btRigidBodyConstructionInfo rb( mass, motion, collision, inertia );
    btRigidBody * body = new btRigidBody( rb );
    //box->setRigidBody(body);
    return( body );
}

btDiscreteDynamicsWorld * initPhysics()
{
    btDefaultCollisionConfiguration * collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher * dispatcher = new btCollisionDispatcher( collisionConfiguration );
    btConstraintSolver * solver = new btSequentialImpulseConstraintSolver;

    btVector3 worldAabbMin( -10000, -10000, -10000 );
    btVector3 worldAabbMax( 10000, 10000, 10000 );
    btBroadphaseInterface * inter = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );

    btDiscreteDynamicsWorld * dynamicsWorld = new btDiscreteDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );

    dynamicsWorld->setGravity( btVector3( 0, 0, -10 ) );

    return( dynamicsWorld );
}

float randomFloatInRange( std::pair< float, float > range =
                         std::pair< float, float >( -10, 10 ) )
{
    return( range.first + (range.second-range.first)*rand()/static_cast< float >( RAND_MAX ) );
}

btVector3 randomBVec3InRange( std::pair< btVector3, btVector3 > range =
                             std::pair< btVector3, btVector3 >(
                                 btVector3( -1, -1, -1),
                                 btVector3(  1,  1,  1) ) )
{
    return(
        btVector3(
            randomFloatInRange(
                std::pair< float, float >( range.first[0], range.second[0] ) ),
            randomFloatInRange(
                std::pair< float, float >( range.first[1], range.second[1] ) ),
            randomFloatInRange(
                std::pair< float, float >( range.first[2], range.second[2] ) ) ) );
}


/* \cond
class GliderUpdateCallback : public vsg::NodeCallback
{
public:
    GliderUpdateCallback( btRigidBody * body )
        : body_( body )
        , basetime_( 0.0 )
        {}
    virtual void operator()( vsg::Node* node, vsg::NodeVisitor* nv )
    {
        double now = nv->getFrameStamp()->getSimulationTime();
        if ( ( now - basetime_ ) > 5.5 )
        {
            basetime_ = now;
            btVector3 punch = randomBVec3InRange(
                std::pair< btVector3, btVector3 >(
                btVector3( -10, -10, -.5 ), btVector3(  10,  10, .5 ) ) );
            vsg::notify( vsg::NOTICE ) << "punch!"
                << punch[0] << " "
                << punch[1] << " "
                << punch[2] << std::endl;
            body_->setLinearVelocity( punch );
            body_->setAngularVelocity( randomBVec3InRange() );
        }
        traverse( node, nv );
    }
private:
    btRigidBody *   body_;
    double          basetime_;
};*/
/* \endcond */


vsg::ref_ptr<vsg::MatrixTransform>
makeCow2( btDynamicsWorld* bw, vsg::dvec3 pos )
{
    vsg::dmat4 m( vsg::rotate( 1.5, vsg::dvec3( 0., 0., 1. ) ) * vsg::translate( pos ));

    vsg::ref_ptr<vsg::MatrixTransform> root = vsg::MatrixTransform::create();
    //  root->matrix =  m ;
    vsg::ref_ptr<vsg::MatrixTransform> amt =  vsg::MatrixTransform::create();
    //vsg::ref_ptr<vsg::AbsoluteTransform> amt =  vsg::AbsoluteTransform::create();
    // amt->setDataVariance( vsg::Object::DYNAMIC );
    root->addChild( amt );

    //const std::string fileName( "cow.osg" );
    const std::string fileName( "Duck.vsgt" );
    auto options = vsg::Options::create();
    options->sharedObjects = vsg::SharedObjects::create();
    options->fileCache = vsg::getEnv("VSG_FILE_CACHE");
    options->paths = vsg::getEnvPaths("VSG_FILE_PATH");
    vsg::ref_ptr<vsg::Node>node =vsg::read_cast<vsg::Node>(fileName,options);
    //vsg::Node* node = osgDB::readNodeFile( fileName );
    if( node == nullptr )
    {
        std::cerr << "Can't find \"" << fileName << "\". Make sure OSG_FILE_PATH includes the OSG sample data directory." << std::endl;
        exit( 0 );
    }
    amt->addChild( node );
    btCollisionShape* cs = vsgbCollision::btConvexTriMeshCollisionShapeFromVSG( node );
    vsgbDynamics::MotionState* motion = new vsgbDynamics::MotionState();
    motion->setTransform( amt );
    motion->setParentTransform( m );

    btScalar mass( 2. );
    btVector3 inertia( 0, 0, 0 );
    cs->calculateLocalInertia( mass, inertia );
    btRigidBody::btRigidBodyConstructionInfo rb( mass, motion, cs, inertia );

    // Set up for multithreading and triple buffering.
   // motion->registerTripleBuffer( &tBuf );
  //  msl.insert( motion );

    btRigidBody* body = new btRigidBody( rb );
    body->setActivationState( DISABLE_DEACTIVATION );
    bw->addRigidBody( body );

    //srh->add( "cow", body );
    //amt->setUserData( new vsgbCollision::RefRigidBody( body ) );
    amt->setValue("rigidbody", new vsgbCollision::RefRigidBody( body ));

    return( root );
}


vsg::ref_ptr<vsg::Transform> makeGate( btDiscreteDynamicsWorld* bw/*, vsgbInteraction::SaveRestoreHandler* srh*/, vsg::ref_ptr<vsg::Node> nodea,vsg::ref_ptr<vsg::Group> parent, const vsg::mat4& m )
{
    auto options = vsg::Options::create();
    options->sharedObjects = vsg::SharedObjects::create();
    options->fileCache = vsg::getEnv("VSG_FILE_CACHE");
    options->paths = vsg::getEnvPaths("VSG_FILE_PATH");
    vsg::ref_ptr<vsg::Node>node =vsg::read_cast<vsg::Node>("otherWall.vsgt",options);
    //vsgbCollision::AbsoluteModelTransform* amt = new vsgbCollision::AbsoluteModelTransform; amt->setDataVariance( vsg::Object::DYNAMIC );
    vsg::ref_ptr<vsg::MatrixTransform> amt = vsg::MatrixTransform::create();

    //insertAbove( node, amt );
   // parent->children.clear();
    amt->addChild(node);


    vsg::ref_ptr< vsgbDynamics::CreationRecord > cr =  vsgbDynamics::CreationRecord::create();
    cr->_sceneGraph = amt;
    cr->_shapeType = CONVEX_HULL_SHAPE_PROXYTYPE;
    //cr->setCenterOfMass( node->getBound().center() );
    vsg::ComputeBounds computeBounds;
    node->accept(computeBounds);
    cr->setCenterOfMass( vsg::vec3(computeBounds.bounds.max + computeBounds.bounds.min) *0.5f);
    cr->_parentTransform = m;
    cr->_mass = 1.f;
    cr->_restitution = .5f;
    btRigidBody* rb = vsgbDynamics::createRigidBody( cr.get() );


    bw->addRigidBody(rb);
    //bw->addRigidBody( rb, COL_GATE, gateCollidesWith );
    rb->setActivationState( DISABLE_DEACTIVATION );

    // Save RB in global, as AMT UserData (for DragHandler), and in SaveRestoreHandler.

    // amt->setUserData( new vsgbCollision::RefRigidBody( rb ) );
    //  srh->add( "gate", rb );
    vsg::ref_ptr<vsg::Node> debugNode  (vsgbCollision::vsgNodeFromBtCollisionShape( rb->getCollisionShape() ));
   // vsg::ref_ptr<vsg::Group> n=node;
    vsg::ref_ptr<vsg::StateGroup> n2;
   // while( (n->children[0]->cast<vsg::Group>()) != nullptr)
    //    n=n->children[0]->cast<vsg::Group>();
    amt->addChild( debugNode );
    return( amt );
}

vsg::ref_ptr<vsg::Transform>
makeModel( const std::string& fileName, const int index, btDynamicsWorld* bw, vsg::vec3 pos)
{
    vsg::ref_ptr< vsg::Node > modelNode( nullptr );
    vsg::dmat4 m( vsg::translate( pos ) );
    vsg::ref_ptr<vsg::MatrixTransform> root = vsg::MatrixTransform::create();
   // root->matrix =  m ;
    // vsg::ref_ptr<vsgbDynamics::AbsoluteModelTransform> amt =  vsgbDynamics::AbsoluteModelTransform::create();
    vsg::ref_ptr<vsg::MatrixTransform> amt =  vsg::MatrixTransform::create();
    // amt->setDataVariance( vsg::Object::DYNAMIC );
    root->addChild( amt );

    //const std::string fileName( "cow.osg" );
      auto options = vsg::Options::create();
    options->sharedObjects = vsg::SharedObjects::create();
    options->fileCache = vsg::getEnv("VSG_FILE_CACHE");
    options->paths = vsg::getEnvPaths("VSG_FILE_PATH");
    vsg::ref_ptr<vsg::Node>node =vsg::read_cast<vsg::Node>("Duck.vsgt",options);
    //vsg::Node* node = osgDB::readNodeFile( fileName );
    if( node == nullptr )
    {
        std::cerr << "Can't find \"" << fileName << "\". Make sure OSG_FILE_PATH includes the OSG sample data directory." << std::endl;
        exit( 0 );
    }
    amt->addChild( node );

btCollisionShape *collision=vsgbCollision::btCompoundShapeFromVSGGeodes(node,BOX_SHAPE_PROXYTYPE,vsgbCollision::Y,0);
    //collision = vsgbCollision::btConvexTriMeshCollisionShapeFromVSG( node );
//
        vsg::ref_ptr<vsg::Node> debugNode  (vsgbCollision::vsgNodeFromBtCollisionShape( collision ));
     amt->addChild( debugNode );


    vsgbDynamics::MotionState* motion = new vsgbDynamics::MotionState();
    motion->setTransform( amt );
    motion->setParentTransform( m );

    btScalar mass( 2. );
    btVector3 inertia( 0, 0, 0 );
    collision->calculateLocalInertia( mass, inertia );
    btRigidBody::btRigidBodyConstructionInfo rb( mass, motion, collision, inertia );

    // Set up for multithreading and triple buffering.
    //motion->registerTripleBuffer( &tBuf );
    //msl.insert( motion );

    btRigidBody* body = new btRigidBody( rb );
    body->setActivationState( DISABLE_DEACTIVATION );
    bw->addRigidBody( body );

   // body->setLinearVelocity( btVector3( -5, -1, 0 ) );
   // body->setAngularVelocity( btVector3( 1, 0, 0 ) );
    // srh->add( "cow", body );
    //TODO amt->setUserData( new vsgbCollision::RefRigidBody( body ) );
    amt->setValue("rigidbody", new vsgbCollision::RefRigidBody( body ));

    return( root );

}

vsg::ref_ptr<vsg::MatrixTransform>
makeCow( btDynamicsWorld* bw, vsg::dvec3 pos /*,vsgbInteraction::SaveRestoreHandler* srh*/ )
{
    vsg::dmat4 m(  vsg::translate( pos ));

    vsg::ref_ptr<vsg::MatrixTransform> root = vsg::MatrixTransform::create();
  //  root->matrix =  m ;
    // vsg::ref_ptr<vsgbDynamics::AbsoluteModelTransform> amt =  vsgbDynamics::AbsoluteModelTransform::create();
     vsg::ref_ptr<vsg::MatrixTransform> amt =  vsg::MatrixTransform::create();
    // amt->setDataVariance( vsg::Object::DYNAMIC );
    root->addChild( amt );

    //const std::string fileName( "cow.osg" );
    const std::string fileName( "Duck.vsgt" );
    auto options = vsg::Options::create();
    options->sharedObjects = vsg::SharedObjects::create();
    options->fileCache = vsg::getEnv("VSG_FILE_CACHE");
    options->paths = vsg::getEnvPaths("VSG_FILE_PATH");
    vsg::ref_ptr<vsg::Node>node =vsg::read_cast<vsg::Node>(fileName,options);
    //vsg::Node* node = osgDB::readNodeFile( fileName );
    if( node == nullptr )
    {
        std::cerr << "Can't find \"" << fileName << "\". Make sure OSG_FILE_PATH includes the OSG sample data directory." << std::endl;
        exit( 0 );
    }
    amt->addChild( node );
  //  btCollisionShape* cs = vsgbCollision::btConvexTriMeshCollisionShapeFromVSG( node );
    btCollisionShape* cs=vsgbCollision::btCompoundShapeFromVSGGeodes( vsg::read_cast<vsg::Node>("Duck.vsgt",options),BOX_SHAPE_PROXYTYPE,vsgbCollision::Y,0);///buggy
    vsg::ComputeBounds cbv;
    node->accept( cbv );
    vsg::dbox bb = cbv.bounds;
    vsg::dvec3 center = (bb.max+bb.min);
    center *= 0.5;
 //cs = vsgbCollision::btBoxCollisionShapeFromVSG( node, &bb );

    vsg::ref_ptr<vsg::Node> debugNode  (vsgbCollision::vsgNodeFromBtCollisionShape( cs ));
   // amt->addChild( debugNode );
    amt->addChild( vsg::read_cast<vsg::Node>(fileName,options) );
    vsgbDynamics::MotionState* motion = new vsgbDynamics::MotionState();
    motion->setTransform( root );
    motion->setParentTransform( m );

    btScalar mass( 2. );
    btVector3 inertia( 0, 0, 0 );
    cs->calculateLocalInertia( mass, inertia );
    btRigidBody::btRigidBodyConstructionInfo rb( mass, motion, cs, inertia );

    // Set up for multithreading and triple buffering.
   //motion->registerTripleBuffer( &tBuf );
    //msl.insert( motion );

    btRigidBody* body = new btRigidBody( rb );
    body->setActivationState( DISABLE_DEACTIVATION );
    bw->addRigidBody( body );

    // srh->add( "cow", body );
    //TODO amt->setUserData( new vsgbCollision::RefRigidBody( body ) );
    amt->setValue("rigidbody", new vsgbCollision::RefRigidBody( body ));

    return( root );
}

vsg::ref_ptr<vsg::MatrixTransform> createModel( btDynamicsWorld * dynamicsWorld )
{
    /*
 * BEGIN: Create physics object code.
 *  VSG CODE
 */
    //vsg::ref_ptr< vsg::MatrixTransform > node;
    vsg::ref_ptr<vsg::MatrixTransform>node;
    const std::string fileName( "Duck.vsgt" );
    auto options = vsg::Options::create();
    options->sharedObjects = vsg::SharedObjects::create();
    options->fileCache = vsg::getEnv("VSG_FILE_CACHE");
    options->paths = vsg::getEnvPaths("VSG_FILE_PATH");
    auto nodeDB =vsg::read_cast<vsg::Node>(fileName,options); createVSGBox(vsg::vec3(.3, .3, .3));//
    if( !nodeDB.valid() )
    {
        std::cerr << "Can't find \"" << fileName << "\". Make sure VSG_FILE_PATH includes the VSG sample data directory." << std::endl;
        exit( 0 );
    }

    if( ( node = dynamic_cast< vsg::MatrixTransform* >( nodeDB.get() ) ) == nullptr )
    {
        node = new vsg::MatrixTransform;
        node->addChild( nodeDB );
    }

    /*  vsgBullet code */
    vsgbDynamics::MotionState * motion = new vsgbDynamics::MotionState;
    motion->setTransform( node.get() );
    //ConvexHullCollisionShape outperform
    btCollisionShape * collision = vsgbCollision::btConvexTriMeshCollisionShapeFromVSG( node.get() );
    //btCollisionShape * collision =  vsgbCollision::btConvexHullCollisionShapeFromVSG( node.get() );
    vsgbCollision::ComputeShapeVisitor cshapev( TRIANGLE_MESH_SHAPE_PROXYTYPE,vsgbCollision::Y,3 );

    //node->accept(cshapev);
    //collision=cshapev.getShape();
    // Create an VSG representation of the Bullet shape and attach it.
    // This is mainly for debugging (shading is not setted)
    vsg::ref_ptr<vsg::Node> debugNode  (vsgbCollision::vsgNodeFromBtCollisionShape( collision ));
    node->addChild( debugNode );

    /*  BULLET CODE */
    btTransform bodyTransform;
    bodyTransform.setIdentity();
    bodyTransform.setOrigin( btVector3( 0, 0, 5 ) );
    motion->setWorldTransform( bodyTransform );
    //vsg::dmat4 m(  vsg::translate( pos ));

    btScalar mass( 1.0 );
    btVector3 inertia;
    collision->calculateLocalInertia( mass, inertia );
    btRigidBody::btRigidBodyConstructionInfo rbinfo( mass, motion, collision, inertia );

    btRigidBody * body = new btRigidBody( rbinfo );
    body->setLinearVelocity( btVector3( -5, -1, 0 ) );
    body->setAngularVelocity( btVector3( 1, 0, 0 ) );

    //vsg::ref_ptr<vsgbDynamics:: RigidBody> gliderrig = vsgbDynamics::RigidBody::create();
   // gliderrig->setRigidBody( body);
   // gliderrig->addChild(node);

    dynamicsWorld->addRigidBody(body);
    return( node );
}

int main(int argc,
         char * argv[])
{
    // set up defaults and read command line arguments to override them
    vsg::CommandLine arguments(&argc, argv);

    // if we want to redirect std::cout and std::cerr to the vsg::Logger call vsg::Logger::redirect_stdout()
    if (arguments.read({"--redirect-std", "-r"})) vsg::Logger::instance()->redirect_std();

    // set up vsg::Options to pass in filepaths, ReaderWriters and other IO related options to use when reading and writing files.
    auto options = vsg::Options::create();
    options->sharedObjects = vsg::SharedObjects::create();
    options->fileCache = vsg::getEnv("VSG_FILE_CACHE");
    options->paths = vsg::getEnvPaths("VSG_FILE_PATH");

#ifdef vsgXchange_all
    // add vsgXchange's support for reading and writing 3rd party file formats
    options->add(vsgXchange::all::create());
#endif

    arguments.read(options);

    if (uint32_t numOperationThreads = 0; arguments.read("--ot", numOperationThreads)) options->operationThreads = vsg::OperationThreads::create(numOperationThreads);

    auto windowTraits = vsg::WindowTraits::create();
    windowTraits->windowTitle = "vsgviewer";
    windowTraits->debugLayer = arguments.read({"--debug", "-d"});
    windowTraits->apiDumpLayer = arguments.read({"--api", "-a"});
    windowTraits->synchronizationLayer = arguments.read("--sync");
    bool reportAverageFrameRate = arguments.read("--fps");
    if (arguments.read("--double-buffer")) windowTraits->swapchainPreferences.imageCount = 2;
    if (arguments.read("--triple-buffer")) windowTraits->swapchainPreferences.imageCount = 3; // default
    if (arguments.read("--IMMEDIATE")) { windowTraits->swapchainPreferences.presentMode = VK_PRESENT_MODE_IMMEDIATE_KHR; }
    if (arguments.read("--FIFO")) windowTraits->swapchainPreferences.presentMode = VK_PRESENT_MODE_FIFO_KHR;
    if (arguments.read("--FIFO_RELAXED")) windowTraits->swapchainPreferences.presentMode = VK_PRESENT_MODE_FIFO_RELAXED_KHR;
    if (arguments.read("--MAILBOX")) windowTraits->swapchainPreferences.presentMode = VK_PRESENT_MODE_MAILBOX_KHR;
    if (arguments.read({"-t", "--test"}))
    {
        windowTraits->swapchainPreferences.presentMode = VK_PRESENT_MODE_IMMEDIATE_KHR;
        windowTraits->fullscreen = true;
        reportAverageFrameRate = true;
    }
    if (arguments.read({"--st", "--small-test"}))
    {
        windowTraits->swapchainPreferences.presentMode = VK_PRESENT_MODE_IMMEDIATE_KHR;
        windowTraits->width = 192, windowTraits->height = 108;
        windowTraits->decoration = false;
        reportAverageFrameRate = true;
    }

    bool multiThreading = arguments.read("--mt");
    if (arguments.read({"--fullscreen", "--fs"})) windowTraits->fullscreen = true;
    if (arguments.read({"--window", "-w"}, windowTraits->width, windowTraits->height)) { windowTraits->fullscreen = false; }
    if (arguments.read({"--no-frame", "--nf"})) windowTraits->decoration = false;
    if (arguments.read("--or")) windowTraits->overrideRedirect = true;
    auto maxTime = arguments.value(std::numeric_limits<double>::max(), "--max-time");

    if (arguments.read("--d32")) windowTraits->depthFormat = VK_FORMAT_D32_SFLOAT;
    if (arguments.read("--sRGB")) windowTraits->swapchainPreferences.surfaceFormat = {VK_FORMAT_B8G8R8A8_SRGB, VK_COLOR_SPACE_SRGB_NONLINEAR_KHR};
    if (arguments.read("--RGB")) windowTraits->swapchainPreferences.surfaceFormat = {VK_FORMAT_B8G8R8A8_UNORM, VK_COLOR_SPACE_SRGB_NONLINEAR_KHR};

    arguments.read("--screen", windowTraits->screenNum);
    arguments.read("--display", windowTraits->display);
    arguments.read("--samples", windowTraits->samples);
    if (int log_level = 0; arguments.read("--log-level", log_level)) vsg::Logger::instance()->level = vsg::Logger::Level(log_level);
    auto numFrames = arguments.value(-1, "-f");
    auto pathFilename = arguments.value<vsg::Path>("", "-p");
    auto loadLevels = arguments.value(0, "--load-levels");
    auto maxPagedLOD = arguments.value(0, "--maxPagedLOD");
    auto horizonMountainHeight = arguments.value(0.0, "--hmh");
    auto nearFarRatio = arguments.value<double>(0.001, "--nfr");
    if (arguments.read("--rgb")) options->mapRGBtoRGBAHint = false;

    bool depthClamp = arguments.read({"--dc", "--depthClamp"});
    if (depthClamp)
    {
        std::cout << "Enabled depth clamp." << std::endl;
        auto deviceFeatures = windowTraits->deviceFeatures = vsg::DeviceFeatures::create();
        deviceFeatures->get().samplerAnisotropy = VK_TRUE;
        deviceFeatures->get().depthClamp = VK_TRUE;
    }

    // create the viewer and assign window(s) to it
    auto viewer = vsg::Viewer::create();
    auto window = vsg::Window::create(windowTraits);
    if (!window)
    {
        std::cout << "Could not create window." << std::endl;
        return 1;
    }

    viewer->addWindow(window);

    vsg::ref_ptr<    vsgbDynamics::World  > vsgbt_scene;
    vsg::ref_ptr<    vsg::Group  > vsg_scene;
    {
        vsg_scene=vsg::Group::create();
        vsgbt_scene = new    vsgbDynamics::World;
        viewer->addUpdateOperation(vsgbDynamics::BulletOperation::create(vsgbt_scene), vsg::UpdateOperations::ALL_FRAMES);
        vsgbt_scene->setDebugEnabled(true);
        btDiscreteDynamicsWorld * dynamicsWorld = initPhysics();
        vsgbt_scene->setDynamicsWorld(dynamicsWorld);

        //   vsg::ref_ptr<vsg::Transform >   glider=makeGate(dynamicsWorld,nullptr,vsgbt_scene,vsg::translate(vsg::vec3(0,0,5)));
        //   vsg::ref_ptr<vsg::Transform > glider =   makeModel("",0,dynamicsWorld, vsg::vec3(0,0,5));
     vsg::ref_ptr<vsg::Transform > glider =  makeCow(dynamicsWorld, vsg::dvec3(0,0,5));
       //  vsg::ref_ptr<vsg::Transform > glider = createModel(dynamicsWorld);
         vsgbt_scene->addChild(glider);
        vsg_scene->addChild(glider);
        /* BEGIN: Create environment boxes*/
        vsg::ref_ptr<vsg::MatrixTransform> ground = 0;
        btRigidBody * groundBody = 0;

        float thin = 0.5;
        // vsgbDynamics:: World * vsgbtworld=new vsgbDynamics::World;
        // root->addUpdateCallback(vsgbtworld);
        {
            vsg::ref_ptr<vsg::MatrixTransform> ground1 = createVSGBox(vsg::vec3(10, 10, thin));
            btRigidBody *  groundBody1 = createBTBox(ground1, vsg::dvec3(0, 0, -10));


            vsg::ref_ptr<vsgbDynamics::RigidBody> groundBodyrig =  vsgbDynamics::RigidBody::create();
            groundBodyrig->setRigidBody(groundBody1);
            //ground1->addUpdateCallback(groundBodyrig);
            //dynamicsWorld->addRigidBody( groundBody );
            groundBodyrig->addChild(ground1);
            vsgbt_scene->addChild(groundBodyrig);
        }
        ground = createVSGBox(vsg::vec3(10, thin, 5));
        groundBody = createBTBox(ground, vsg::dvec3(0, 10, -5));
        {
            vsg::ref_ptr<vsgbDynamics::RigidBody> groundBodyrig =  vsgbDynamics::RigidBody::create();
            groundBodyrig->setRigidBody(groundBody);
            //ground->addUpdateCallback(groundBodyrig);
            //dynamicsWorld->addRigidBody( groundBody );
            groundBodyrig->addChild(ground);
            vsgbt_scene->addChild(groundBodyrig);
        }

        ground = createVSGBox(vsg::vec3(10, thin, 5));
        groundBody = createBTBox(ground, vsg::dvec3(0, -10, -5));
        {
            vsg::ref_ptr<vsgbDynamics::RigidBody> groundBodyrig =  vsgbDynamics::RigidBody::create();
            groundBodyrig->addChild(ground );
            groundBodyrig->setRigidBody(groundBody);
            //ground->addUpdateCallback(groundBodyrig);
            //dynamicsWorld->addRigidBody( groundBody );
            groundBodyrig->addChild(ground);
            vsgbt_scene->addChild(groundBodyrig);
        }

        ground = createVSGBox(vsg::vec3(thin, 10, 5));
        groundBody = createBTBox(ground, vsg::dvec3(10, 0, -5));
        {
            vsg::ref_ptr<vsgbDynamics::RigidBody> groundBodyrig =  vsgbDynamics::RigidBody::create();
            groundBodyrig->setRigidBody(groundBody);
            //ground->addUpdateCallback(groundBodyrig);
            //dynamicsWorld->addRigidBody( groundBody );
            groundBodyrig->addChild(ground);
            vsgbt_scene->addChild(groundBodyrig);
        }

        ground = createVSGBox(vsg::vec3(thin, 10, 5));
        groundBody = createBTBox(ground, vsg::dvec3(-10, 0, -5));
        {
            vsg::ref_ptr<vsgbDynamics::RigidBody> groundBodyrig =  vsgbDynamics::RigidBody::create();
            groundBodyrig->setRigidBody(groundBody);
            //ground->addUpdateCallback(groundBodyrig);
            //dynamicsWorld->addRigidBody( groundBody );
            groundBodyrig->addChild(ground);
            vsgbt_scene->addChild(groundBodyrig);
        }
        /* END: Create environment boxes */

        /* BEGIN: Create animated box */
        /* VSG Code */
        vsg::ref_ptr<vsg::MatrixTransform> box = createVSGBox(vsg::vec3(.3, .3, .3));
        // vsg::AnimationPathCallback * apc = new vsg::AnimationPathCallback(createAnimationPath(vsg::vec3(0, 0, -9.25), 9.4, 6), 0, 1);
        //vsgbt_scene->addChild(box);

        /* Bullet Code */
        btRigidBody * boxBody = createBTBox(box, vsg::dvec3(-9, -3, -9));
        boxBody->setCollisionFlags(boxBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
        boxBody->setActivationState(DISABLE_DEACTIVATION);
        {
            vsg::ref_ptr<vsgbDynamics::RigidBody> boxBodyrig =  vsgbDynamics::RigidBody::create();
            boxBodyrig->setRigidBody(boxBody);
            //box->addUpdateCallback(boxBodyrig);
            //dynamicsWorld->addRigidBody( boxBody );
            boxBodyrig->addChild(box);
            vsgbt_scene->addChild(boxBodyrig);
        }
        //box->addUpdateCallback(apc);

        // Make ground.
        {
            vsg::vec4 gp( 0, 0, 1, 0 );
          //  vsgbt_scene->addChild( vsgbDynamics::generateGroundPlane( gp, dynamicsWorld ) );
        }


        /* vsgBullet Code*/
        vsgbCollision::RefBulletObject< btRigidBody >* boxRigid =
            new vsgbCollision::RefBulletObject< btRigidBody >(boxBody);
        box->setValue("btRigidBody",boxRigid);

        // compute the bounds of the scene graph to help position camera
        vsg::ComputeBounds computeBounds;
        vsg_scene->accept(computeBounds);
        vsg::dvec3 centre = (computeBounds.bounds.min + computeBounds.bounds.max);
        centre *= 0.5;
        double radius = vsg::length(computeBounds.bounds.max - computeBounds.bounds.min);// * 0.6;

        // set up the camera
        auto lookAt = vsg::LookAt::create(centre + vsg::dvec3(0.0, -radius * 3.5, 0.0), centre, vsg::dvec3(0.0, 0.0, 1.0));

        vsg::ref_ptr<vsg::ProjectionMatrix> perspective;
        auto ellipsoidModel = vsg_scene->getRefObject<vsg::EllipsoidModel>("EllipsoidModel");
        if (ellipsoidModel)
        {
            perspective = vsg::EllipsoidPerspective::create(lookAt, ellipsoidModel, 30.0, static_cast<double>(window->extent2D().width) / static_cast<double>(window->extent2D().height), nearFarRatio, horizonMountainHeight);
        }
        else
        {
            perspective = vsg::Perspective::create(30.0, static_cast<double>(window->extent2D().width) / static_cast<double>(window->extent2D().height), nearFarRatio * radius, radius * 100);
        }

        auto camera = vsg::Camera::create(perspective, lookAt, vsg::ViewportState::create(window->extent2D()));

        // add close handler to respond to the close window button and pressing escape
        viewer->addEventHandler(vsg::CloseHandler::create(viewer));
    //       viewer->addEventHandler(vsgbInteraction::DragHandler::create(vsgbt_scene, camera, ellipsoidModel));
     viewer->addEventHandler(vsgbInteraction::LaunchHandler::create(vsgbt_scene, vsgbt_scene, vsg::observer_ptr<vsg::Viewer>(viewer), camera, ellipsoidModel));

        auto commandGraph = vsg::createCommandGraphForView(window, camera, vsgbt_scene);
        viewer->assignRecordAndSubmitTaskAndPresentation({commandGraph});
        viewer->compile();
        // viewer->addUpdateOperation(vsgbCollision::opMerge::create(vsg::observer_ptr<vsg::Viewer>(viewer),vsgbt_scene, newnode,viewer->compileManager->compile(newnode)));
        viewer->start_point() = vsg::clock::now();

        // rendering main loop
        while (viewer->advanceToNextFrame() && (numFrames < 0 || (numFrames--) > 0) && (viewer->getFrameStamp()->simulationTime < maxTime))
        {
            // pass any events into EventHandlers assigned to the Viewer
            viewer->handleEvents();

            viewer->update();

            viewer->recordAndSubmit();

            viewer->present();
        }
        /*       const btVector3 btPivot( -0.498f, -0.019f, 0.146f );
   vsgbDynamics::RigidBody * rigglider=dynamic_cast<vsgbDynamics::RigidBody *>(glider->getUpdateCallback());
   btVector3 btAxisA( 0., 0., 1. );
   btHingeConstraint* hinge = new btHingeConstraint( *rigglider->getRigidBody(), btPivot, btAxisA );
   hinge->setLimit( -1.5f, 1.5f );
   //bulletWorld->addConstraint( hinge, true );
   vsgbDynamics::Joint *vsghinge=new vsgbDynamics::Joint();
   vsghinge->setConstraint(hinge);
   //      vsghinge->setBodyA(rigglider);


   //rigglider->addJoint(vsghinge);


}

  


 /*
    btRigidBody* hack=((vsgbDynamics::RigidBody*)root->getChild(root->getNumChildren()-1))->getRigidBody();
   ///not serialized
    hack-> setCollisionFlags( boxBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT );
    hack->setActivationState( DISABLE_DEACTIVATION );
*/

        /*vsg::Group* launchHandlerAttachPoint = new vsg::Group;
    vsg_scene->addChild( launchHandlerAttachPoint );

     vsgbInteraction::LaunchHandler* lh = new vsgbInteraction::LaunchHandler();
     lh->setWorld(root);
        lh->setAttachPoint( launchHandlerAttachPoint );
    {
        // Use a custom launch model: Sphere with radius 0.2 (instead of default 1.0).
        vsg::Geode* geode = new vsg::Geode;
        const double radius( 0.2 );
        geode->addDrawable( new vsg::ShapeDrawable(new vsg::Sphere(vsg::vec3(),radius)));
        lh->setLaunchModel( geode, new btSphereShape( radius ) );
        lh->setInitialVelocity( 40. );

        viewer.addEventHandler( lh );
      //   viewer.addEventHandler( new vsgbInteraction::DragHandler );
    }*/

        /*  while( !viewer.done() )
    {
        currSimTime = viewer.getFrameStamp()->getSimulationTime();
  //    dynamicsWorld->stepSimulation( currSimTime - prevSimTime );
        prevSimTime = currSimTime;
        viewer.frame();
    }*/

        return( 0 );
    }
}

