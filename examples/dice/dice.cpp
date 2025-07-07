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


#include "vsg/io/read.h"
#include "vsg/threading/OperationThreads.h"
#include "vsg/utils/Builder.h"
#include "vsg/utils/CommandLine.h"
#include "vsg/utils/ComputeBounds.h"
#include <iostream>
#include <vsgbDynamics/MotionState.h>
#include <vsgbCollision/CollisionShapes.h>
#include <vsgbDynamics/RigidBody.h>
#include <vsgbCollision/Utils.h>

#include "vsg/app/CloseHandler.h"
#include <vsgbInteraction/LaunchHandler.h>
#include <btBulletDynamicsCommon.h>

#include <string>


vsg::ref_ptr<vsg::MatrixTransform>
makeDie( btDynamicsWorld* bw )
{
    vsg::ref_ptr<vsg::MatrixTransform> root =  vsg::MatrixTransform::create();
    const std::string fileName= "dice.vsgt" ;
    auto options = vsg::Options::create();
    options->sharedObjects = vsg::SharedObjects::create();
    options->fileCache = vsg::getEnv("VSG_FILE_CACHE");
    options->paths = vsg::getEnvPaths("VSG_FILE_PATH");
    vsg::ref_ptr<vsg::Node> node =vsg::read_cast<vsg::Node>(fileName, options);
	if( node == nullptr )
	{
        std::cerr << "Can't find \"" << fileName << "\". Make sure VSG_FILE_PATH includes the osgBullet data directory." << std::endl;
		exit( 0 );
	}
    root->addChild( node );

    btCollisionShape* cs = vsgbCollision::btBoxCollisionShapeFromVSG( node );
    
    vsg::ref_ptr< vsgbDynamics::CreationRecord > cr = vsgbDynamics::CreationRecord::create();
    cr->_sceneGraph = root;
    cr->_shapeType = BOX_SHAPE_PROXYTYPE;
    cr->_mass = 1.f;
    cr->_restitution = 1.f;
    btRigidBody* body = vsgbDynamics::createRigidBody( cr.get(), cs );
    bw->addRigidBody( body );

    return( root );
}


btDynamicsWorld*
initPhysics()
{
    btDefaultCollisionConfiguration * collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher * dispatcher = new btCollisionDispatcher( collisionConfiguration );
    btConstraintSolver * solver = new btSequentialImpulseConstraintSolver;

    btVector3 worldAabbMin( -10000, -10000, -10000 );
    btVector3 worldAabbMax( 10000, 10000, 10000 );
    btBroadphaseInterface * inter = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );

    btDynamicsWorld * dynamicsWorld = new btDiscreteDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );

    dynamicsWorld->setGravity( btVector3( 0, 0, 9.8 ) );

    return( dynamicsWorld );
}


/* \cond*/
class ShakeManipulator: public vsg::Inherit<vsg::Trackball, ShakeManipulator>
{
public:
    ShakeManipulator (vsgbDynamics::MotionState *motion,vsg::ref_ptr<vsg::Camera> camera, vsg::ref_ptr<vsg::EllipsoidModel> ellipsoidModel = nullptr)
        : vsg::Inherit<vsg::Trackball, ShakeManipulator>(camera, ellipsoidModel), _motion( motion )
    {}
    virtual void apply(vsg::KeyPressEvent& buttonPress) override
    {
        std::cerr<<buttonPress.keyBase<<std::endl;
        if (buttonPress.keyBase == vsg::KEY_Space)
        {
            btTransform trans; trans.setIdentity();
            _motion->setWorldTransform( trans );

        }
    }

    virtual void apply(vsg::ButtonPressEvent& buttonPress) override
    {
        vsg::dvec2 xy = ndc(buttonPress);
        _lastX=xy.x;_lastY=xy.y;
        btTransform world;
        _motion->getWorldTransform( world );
        btVector3 o = world.getOrigin();
        o[ 2 ] = 0.25;
        world.setOrigin( o );
        _motion->setWorldTransform( world );
    }
    virtual void apply(vsg::MoveEvent& mouseEvent) override
    {
        btVector3 move;
        vsg::dvec2 xy = ndc(mouseEvent);
        move[ 0 ] = _lastX - xy.x;
        move[ 1 ] = xy.y - _lastY;
        move[ 2 ] = 0.;
        move *= 10.;
        btTransform moveTrans; moveTrans.setIdentity();
        moveTrans.setOrigin( move );
        btTransform world;
        _motion->getWorldTransform( world );
        btTransform netTrans = moveTrans * world;
        btVector3 o = netTrans.getOrigin();
        o[ 2 ] = 0.;
        netTrans.setOrigin( o );

        _motion->setWorldTransform( netTrans );
        _lastX=xy.x; _lastY=xy.y;
    }


protected:
    vsgbDynamics::MotionState *_motion;
    float _lastX, _lastY;
};
/* \endcond */




vsg::ref_ptr<vsg::Node> osgBox( const vsg::vec3& center, const vsg::vec3& halfLengths )
{
    vsg::Builder builder;
    vsg::GeometryInfo geomInfo;
    vsg::StateInfo stateInfo;

    geomInfo.color = vsg::vec4{1, 1, 1, 1};
    geomInfo.position=center;
    geomInfo.dx*=halfLengths[0]*2.;
    geomInfo.dy*=halfLengths[1]*2.;
    geomInfo.dz*=halfLengths[2]*2.;

    return builder.createBox(geomInfo, stateInfo);
}



int
main( int argc,
      char ** argv )
{
    btDynamicsWorld* bulletWorld = initPhysics();

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
    windowTraits->windowTitle = "dice";
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

    auto root = vsg::Group::create();

    root->addChild( makeDie( bulletWorld ) );
    root->addChild( makeDie( bulletWorld ) );


    /* BEGIN: Create environment boxes */
    float xDim( 10. );
    float yDim( 10. );
    float zDim( 6. );
    float thick( .1 );

    vsg::ref_ptr<vsg::MatrixTransform> shakeBox = vsg::MatrixTransform::create();
    btCompoundShape* cs = new btCompoundShape;
    { // floor -Z (far back of the shake cube)
        vsg::vec3 halfLengths( xDim*.5, yDim*.5, thick*.5 );
        vsg::vec3 center( 0., 0., zDim*.5 );
        shakeBox->addChild( osgBox( center, halfLengths ) );
        btBoxShape* box = new btBoxShape( vsgbCollision::asBtVector3( halfLengths ) );
        btTransform trans; trans.setIdentity();
        trans.setOrigin( vsgbCollision::asBtVector3( center ) );
        cs->addChildShape( trans, box );
    }
    { // top +Z
        vsg::vec3 halfLengths( xDim*.5, yDim*.5, thick*.5 );
        vsg::vec3 center( 0., 0., -zDim*.5 );
        shakeBox->addChild( osgBox( center, halfLengths ) );
        btBoxShape* box = new btBoxShape( vsgbCollision::asBtVector3( halfLengths ) );
        btTransform trans; trans.setIdentity();
        trans.setOrigin( vsgbCollision::asBtVector3( center ) );
        cs->addChildShape( trans, box );
    }
    { // left -X
        vsg::vec3 halfLengths( thick*.5, yDim*.5, zDim*.5 );
        vsg::vec3 center( -xDim*.5, 0., 0. );
        shakeBox->addChild( osgBox( center, halfLengths ) );
        btBoxShape* box = new btBoxShape( vsgbCollision::asBtVector3( halfLengths ) );
        btTransform trans; trans.setIdentity();
        trans.setOrigin( vsgbCollision::asBtVector3( center ) );
        cs->addChildShape( trans, box );
    }
    { // right +X
        vsg::vec3 halfLengths( thick*.5, yDim*.5, zDim*.5 );
        vsg::vec3 center( xDim*.5, 0., 0. );
        shakeBox->addChild( osgBox( center, halfLengths ) );
        btBoxShape* box = new btBoxShape( vsgbCollision::asBtVector3( halfLengths ) );
        btTransform trans; trans.setIdentity();
        trans.setOrigin( vsgbCollision::asBtVector3( center ) );
        cs->addChildShape( trans, box );
    }
    { // bottom of window -Y (invisible, to allow user to see through)
        vsg::vec3 halfLengths( xDim*.5, thick*.5, zDim*.5 );
        vsg::vec3 center( 0., -yDim*.5, 0. );
     //   shakeBox->addChild( osgBox( center, halfLengths ) );
        btBoxShape* box = new btBoxShape( vsgbCollision::asBtVector3( halfLengths ) );
        btTransform trans; trans.setIdentity();
        trans.setOrigin( vsgbCollision::asBtVector3( center ) );
        cs->addChildShape( trans, box );
    }
    { // bottom of window -Y
        vsg::vec3 halfLengths( xDim*.5, thick*.5, zDim*.5 );
        vsg::vec3 center( 0., yDim*.5, 0. );
        shakeBox->addChild( osgBox( center, halfLengths ) );
        btBoxShape* box = new btBoxShape( vsgbCollision::asBtVector3( halfLengths ) );
        btTransform trans; trans.setIdentity();
        trans.setOrigin( vsgbCollision::asBtVector3( center ) );
        cs->addChildShape( trans, box );
    }
    /* END: Create environment boxes */

    vsgbDynamics::MotionState * shakeMotion = new vsgbDynamics::MotionState();
    shakeMotion->setTransform( shakeBox );
    btScalar mass( 0.0 );
    btVector3 inertia( 0, 0, 0 );
    btRigidBody::btRigidBodyConstructionInfo rb( mass, shakeMotion, cs, inertia );
    btRigidBody* shakeBody = new btRigidBody( rb );
    shakeBody->setCollisionFlags( shakeBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT );
    shakeBody->setActivationState( DISABLE_DEACTIVATION );
    bulletWorld->addRigidBody( shakeBody );

    root->addChild( shakeBox );

    // compute the bounds of the scene graph to help position camera
    vsg::ComputeBounds computeBounds;
    root->accept(computeBounds);
    vsg::dvec3 centre = (computeBounds.bounds.min + computeBounds.bounds.max);
    centre *= 0.5;
    double radius = vsg::length(computeBounds.bounds.max - computeBounds.bounds.min);// * 0.6;

    // set up the camera
    auto lookAt = vsg::LookAt::create(centre + vsg::dvec3(0.0, -radius * 3.5, 0.0), centre, vsg::dvec3(0.0, 0.0, 1.0));

    vsg::ref_ptr<vsg::ProjectionMatrix> perspective;
    auto ellipsoidModel = root->getRefObject<vsg::EllipsoidModel>("EllipsoidModel");
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
    viewer->addEventHandler( ShakeManipulator::create(shakeMotion, camera) );
    auto commandGraph = vsg::createCommandGraphForView(window, camera, root);
    viewer->assignRecordAndSubmitTaskAndPresentation({commandGraph});
    viewer->compile();
    viewer->start_point() = vsg::clock::now();

    // rendering main loop
    auto prevSimTime =  vsg::clock::now();
    while (viewer->advanceToNextFrame() && (numFrames < 0 || (numFrames--) > 0) && (viewer->getFrameStamp()->simulationTime < maxTime))
          {


        auto currSimTime = vsg::clock::now();//viewer.getFrameStamp()->getSimulationTime();
       // double elapsed( currSimTime - prevSimTime );
        double elapsed= std::chrono::duration_cast<std::chrono::milliseconds>(currSimTime - prevSimTime).count() * 0.001f ;
        /*if( viewer.getFrameStamp()->getFrameNumber() < 3 )
            elapsed = 1./60.;*/
        //std::cerr << elapsed / 3. << ", " << 1./180. << std::endl;
        bulletWorld->stepSimulation( elapsed, 4, elapsed/4. );
        prevSimTime = currSimTime;


        // pass any events into EventHandlers assigned to the Viewer
        viewer->handleEvents();

        viewer->update();

        viewer->recordAndSubmit();

        viewer->present();
    }


    return( 0 );
}


/** \page diceexample The Mandatory Dice Example
No physics-based project would be complete without a dice example. Use the
left mouse button to chake the dice shaker.
*/
