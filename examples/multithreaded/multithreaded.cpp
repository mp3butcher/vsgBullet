/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * osgBullet is (C) Copyright 2025 by Julien Valentin
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

#include "vsgbInteraction/SaveRestoreHandler.h"
#include <vsg/all.h>

#ifdef vsgXchange_FOUND
#    include <vsgXchange/all.h>
#endif

#include <vsgbDynamics/RigidBody.h>
#include <vsgbDynamics/MotionState.h>
#include <vsgbDynamics/GroundPlane.h>
#include <vsgbCollision/CollisionShapes.h>
#include <vsgbCollision/RefBulletObject.h>
#include <vsgbCollision/Utils.h>
#include <vsgbDynamics/TripleBuffer.h>
#include <vsgbDynamics/PhysicsThread.h>
#include <vsgbInteraction/DragHandler.h>
#include <vsgbInteraction/LaunchHandler.h>
//#include <vsgbInteraction/SaveRestoreHandler.h>

//#include <osgwTools/Shapes.h>

#include <btBulletDynamicsCommon.h>

#include <sstream>
#include <osg/io_utils>
#include <string>
#include <map>



vsgbDynamics::TripleBuffer tBuf;
vsgbDynamics::MotionStateList msl;


btDiscreteDynamicsWorld* initPhysics()
{
    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher( collisionConfiguration );
    btConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

    btVector3 worldAabbMin( -10000, -10000, -10000 );
    btVector3 worldAabbMax( 10000, 10000, 10000 );
    btBroadphaseInterface* inter = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );

    btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );
    dynamicsWorld->setGravity( btVector3( 0, 0, -10 ) );

    return( dynamicsWorld );
}


vsg::ref_ptr< vsg::Node > modelNode( nullptr );

vsg::ref_ptr<vsg::Transform>
makeModel( const std::string& fileName, const int index, btDynamicsWorld* bw, vsg::vec3 pos, vsgbInteraction::SaveRestoreHandler* srh )
{
    vsg::dmat4 m( vsg::translate( pos ) );
    vsg::ref_ptr< vsg::MatrixTransform > amt =  vsg::MatrixTransform::create();
    if( !modelNode.valid() )
    {
        const std::string fileName( "Duck.vsgt" );
        auto options = vsg::Options::create();
        options->sharedObjects = vsg::SharedObjects::create();
        options->fileCache = vsg::getEnv("VSG_FILE_CACHE");
        options->paths = vsg::getEnvPaths("VSG_FILE_PATH");
        modelNode =vsg::read_cast<vsg::Node>(fileName,options);
        //modelNode = osgDB::readNodeFile( fileName );
        if( !modelNode.valid() )
        {
            std::cerr << "Can't find \"" << fileName << "\". Make sure OSG_FILE_PATH is set correctly." << std::endl;
            exit( 0 );
        }
    }
    amt->addChild( modelNode );
    amt->matrix=m;

    vsg::ref_ptr< vsgbDynamics::CreationRecord > cr = vsgbDynamics::CreationRecord::create();
    cr->_sceneGraph = amt.get();
    cr->_shapeType = BOX_SHAPE_PROXYTYPE;
    cr->_mass = .2f;
    cr->_restitution = 0.3f;
    cr->_parentTransform = m;
    btRigidBody* rb = vsgbDynamics::createRigidBody( cr.get() );

    rb->setActivationState( DISABLE_DEACTIVATION );


    // Set up for multithreading and triple buffering.
    vsgbDynamics::MotionState* motion = static_cast< vsgbDynamics::MotionState* >( rb->getMotionState() );
    motion->registerTripleBuffer( &tBuf );
    /*    msl.insert( motion );
   // motion->setTransform( amt );
     motion->setParentTransform( m );
    std::ostringstream ostr;
    ostr << fileName << index;
    srh->add( ostr.str(), rb );

    amt->setValue("rigidbody", new vsgbCollision::RefRigidBody( rb ) );//UserData(*/
    bw->addRigidBody( rb );

    return( amt);
}

vsg::ref_ptr<vsg::MatrixTransform>
makeCow( btDynamicsWorld* bw, vsg::dvec3 pos, vsgbInteraction::SaveRestoreHandler *srh )
{
    vsg::dmat4 m( vsg::rotate( 1.5, vsg::dvec3( 0., 0., 1. ) ) * vsg::translate( pos ));

    vsg::ref_ptr<vsg::MatrixTransform> root = vsg::MatrixTransform::create();
    root->matrix =  m ;
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
    motion->registerTripleBuffer( &tBuf );
    msl.insert( motion );

    btRigidBody* body = new btRigidBody( rb );
    body->setActivationState( DISABLE_DEACTIVATION );
    bw->addRigidBody( body );

    srh->add( "cow", body );
    //amt->setUserData( new vsgbCollision::RefRigidBody( body ) );
    amt->setValue("rigidbody", new vsgbCollision::RefRigidBody( body ));

    return( root );
}


int main( int argc, char** argv )
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

    // Increase triple buffer size to hold lots of transform data.
    tBuf.resize( 16384 );

    btDiscreteDynamicsWorld* bulletWorld = initPhysics();
    vsgbDynamics::PhysicsThread pt( bulletWorld, &tBuf );
    vsg::ref_ptr<vsgbDynamics::World> vsg_scene = vsgbDynamics::World::create();
        //vsg::ref_ptr<vsg::Group> vsg_scene = vsg::Group::create();
    vsg_scene->setDynamicsWorld(nullptr);

    vsg::ref_ptr<vsg::Group> launchHandlerAttachPoint = vsg::Group::create();
    vsg_scene->addChild( launchHandlerAttachPoint );
    vsg::ref_ptr< vsgbInteraction::SaveRestoreHandler > srh = vsgbInteraction::SaveRestoreHandler::create();


    std::string fileName( "dice.vsgt" );
    if( argc > 1 )
        // Seconf param is file name.
        fileName = std::string( argv[ 1 ] );

    // Make dice pyramid.
    int xCount( 7 );
    int yCount( 7 );
    float xStart( -4. );
    float yStart( -3. );
    const float zInc( 2.5 );
    float z( 1.75 );
    int index( 0 );
    while( xCount && yCount )
    {
        float x, y;
        int xIdx, yIdx;
        for( y=yStart, yIdx=0; yIdx<yCount; y+=2.25, yIdx++ )
        {
            for( x=xStart, xIdx=0; xIdx<xCount; x+=2.25, xIdx++ )
            {
                vsg::vec3 pos( x, y, z );
                // vsg_scene->addChild( makeModel( fileName, index++, bulletWorld, pos, srh.get() ) );
            }
        }
        xStart += 1.25;
        yStart += 1.25;
        xCount--;
        yCount--;
        z += zInc;
    }

    // Add a duck
    vsg_scene->addChild( makeCow( bulletWorld, vsg::dvec3( -11., 6., 4. ), srh.get() ) );

    // Make ground.
    {
        vsg::vec4 gp( 0, 0, 1, 0 );
        vsg_scene->addChild( vsgbDynamics::generateGroundPlane( gp, bulletWorld ) );
    }



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
    //viewer->addEventHandler(vsgbInteraction::DragHandler::create(vsg_scene, camera, ellipsoidModel));
    auto lh = vsgbInteraction::LaunchHandler::create(vsg_scene, vsg_scene, vsg::observer_ptr<vsg::Viewer>(viewer), camera, ellipsoidModel);
    lh->setThreadedPhysicsSupport( &pt, &tBuf, &msl );
    viewer->addEventHandler(lh);
    viewer->addEventHandler(srh);
    //viewer->addUpdateOperation(vsgbDynamics::BulletOperation::create(vsg_scene), vsg::UpdateOperations::ALL_FRAMES);

    auto commandGraph = vsg::createCommandGraphForView(window, camera, vsg_scene);
    viewer->assignRecordAndSubmitTaskAndPresentation({commandGraph});
    viewer->compile();
    viewer->start_point() = vsg::clock::now();

    pt.setTimeStep(0.04);
    pt.start();
    pt.setProcessorAffinity( 0 );
    // rendering main loop
    while (viewer->advanceToNextFrame() && (numFrames < 0 || (numFrames--) > 0) && (viewer->getFrameStamp()->simulationTime < maxTime))
    {

        viewer->handleEvents();
        // Get the latest transform information from the
        // Bullet simulation.
        // pass any events into EventHandlers assigned to the Viewer
        TripleBufferMotionStateUpdate( msl, &tBuf );
        viewer->update();

        viewer->recordAndSubmit();

        viewer->present();
    }
    pt.stopPhysics();
    pt.join();
}


/** \page multithreaded The multithreaded Example
This examples demonstrates running the Bullet physics simultation in a separate thread.

\section multithreadedcontrols UI Controls

\li Delete: Reset the physics simulation to its initial state.
\li ctrl-leftmouse: Select and drag an object.
\li shift-leftmouse: Launches a sphere into the scene.

*/
