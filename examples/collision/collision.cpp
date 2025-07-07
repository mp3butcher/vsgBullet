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

#include <btBulletCollisionCommon.h>

#include "vsg/threading/OperationThreads.h"
#include "vsg/utils/Builder.h"
#include "vsg/app/CloseHandler.h"
#include <vsg/nodes/MatrixTransform.h>
#include <vsg/app/Trackball.h>
#include <vsg/utils/CommandLine.h>
#include <vsg/utils/ComputeBounds.h>

//#include <vsgbDynamics/World.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <vsgbCollision/CollisionShapes.h>
#include <vsgbCollision/Utils.h>
#include <vsgbCollision/GLDebugDrawer.h>
//#include <osgwTools/Shapes.h>
//#include <osgwTools/Version.h

#include <iostream>


class MoveManipulator : public vsg::Inherit<vsg::Trackball, MoveManipulator>
{
public:
    MoveManipulator (vsg::ref_ptr<vsg::Camera> camera, vsg::ref_ptr<vsg::EllipsoidModel> ellipsoidModel = nullptr)
        : vsg::Inherit<vsg::Trackball, MoveManipulator>(camera, ellipsoidModel),_co( nullptr ), _mt( nullptr ) {}
   // MoveManipulator( const MoveManipulator& mm, vsg::CopyOp copyop ) :vsg::Inherit<vsg::Trackball, MoveManipulator>(mm,copyop), _co( mm._co ), _mt( mm._mt ) {}
    ~MoveManipulator() {}

    virtual void apply(vsg::KeyPressEvent& buttonPress) override
    {
        std::cerr<<buttonPress.keyBase<<std::endl;
        if (buttonPress.keyBase == vsg::KEY_Control_L)_ctrl_pressed=true;
    }
    virtual void apply(vsg::KeyReleaseEvent& buttonPress) override
    {
        std::cerr<<buttonPress.keyBase<<std::endl;
        if (buttonPress.keyBase == vsg::KEY_Control_L)_ctrl_pressed=false;
    }

    virtual void apply(vsg::ButtonPressEvent& buttonPress)override

        {
            //_lastX = ea.getXnormalized();
            //_lastY = ea.getYnormalized();
            vsg::dvec2 xy=ndc(buttonPress);
            _lastX=xy.x;_lastY=xy.y;
        }
    virtual void apply(vsg::MoveEvent& mouseEvent) override
        {

            //if( ea.getEventType() == osgGA::GUIEventAdapter::DRAG )
        {
                vsg::dvec2 xy=ndc(mouseEvent);
            double deltaX = xy.x- _lastX;
            double deltaY = xy.y - _lastY;
            _lastX = xy.x;
            _lastY = xy.y;

            deltaX *= 6.;
            deltaY *= 6.;
            vsg::mat4 trans = vsgbCollision::asVsgMatrix( _co->getWorldTransform() );
            trans = trans * vsg::translate<float>( deltaX, 0., deltaY );
            _mt->matrix= trans ;
            _co->setWorldTransform( vsgbCollision::asBtTransform( trans ) );
           // return( true );
        }
     //   return( false );
    }

    void setCollisionObject( btCollisionObject* co ) { _co = co; }
    void setMatrixTransform( vsg::MatrixTransform* mt ) { _mt = mt; }

protected:
    btCollisionObject* _co;
    vsg::MatrixTransform* _mt;
    double _lastX, _lastY;bool _ctrl_pressed;
};
/* \endcond */


btCollisionWorld* initCollision()
{
    btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher( collisionConfiguration );

    btVector3 worldAabbMin( -10000, -10000, -10000 );
    btVector3 worldAabbMax( 10000, 10000, 10000 );
    btBroadphaseInterface* inter = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );

    btCollisionWorld* collisionWorld = new btCollisionWorld( dispatcher, inter, collisionConfiguration );

    return( collisionWorld );
}


vsg::ref_ptr<vsg::Group> createScene( btCollisionWorld* cw, MoveManipulator* mm, vsg::CommandLine& arguments )
{
    vsg::ref_ptr< vsg::Group > root = vsg::Group::create();

    vsg::Builder builder;
    vsg::GeometryInfo geomInfo;
    vsg::StateInfo stateInfo;

    geomInfo.color = vsg::vec4{1, 1, 1, 1};

    geomInfo.dx*=.5*2.;
    geomInfo.dy*=.5*2.;
    geomInfo.dz*=.5*2.;

    auto node = builder.createBox(geomInfo, stateInfo);
    // Create a static box
   /* vsg::Geode* geode = new vsg::Geode;
    geode->addDrawable( new vsg::ShapeDrawable(new vsg::Box(vsg::vec3(),1.0)));// vsg::vec3( .5, .5, .5 ) ) );*/
    root->addChild( node );

    btCollisionObject* btBoxObject = new btCollisionObject;
    btBoxObject->setCollisionShape( vsgbCollision::btBoxCollisionShapeFromVSG( node ) );
    btBoxObject->setCollisionFlags( btCollisionObject::CF_STATIC_OBJECT );
    cw->addCollisionObject( btBoxObject );


    // Create a box we can drag around with the mouse
    node = builder.createBox(geomInfo, stateInfo);
    // Create a static box
    /* vsg::Geode* geode = new vsg::Geode;
    geode->addDrawable( new vsg::ShapeDrawable(new vsg::Box(vsg::vec3(),1.0)));// vsg::vec3( .5, .5, .5 ) ) );*/

    vsg::mat4 transMatrix = vsg::translate<float>( 4., 0., 0. );
    vsg::ref_ptr<vsg::MatrixTransform> mt = vsg::MatrixTransform::create();
    mt->matrix = transMatrix ;
    mt->addChild( node );
    root->addChild( mt );

    btBoxObject = new btCollisionObject;
    btBoxObject->setCollisionShape( vsgbCollision::btBoxCollisionShapeFromVSG( node ) );
    btBoxObject->setCollisionFlags( btCollisionObject::CF_KINEMATIC_OBJECT );
    btBoxObject->setWorldTransform( vsgbCollision::asBtTransform( transMatrix ) );
    cw->addCollisionObject( btBoxObject );
    mm->setCollisionObject( btBoxObject );
    mm->setMatrixTransform( mt );


    return( root );
}

void detectCollision( bool& lastColState, btCollisionWorld* cw )
{
    unsigned int numManifolds = cw->getDispatcher()->getNumManifolds();
    if( ( numManifolds == 0 ) && (lastColState == true ) )
    {
        std::cerr << "No collision." << std::endl;
        lastColState = false;
    }
    else {
        for( unsigned int i = 0; i < numManifolds; i++ )
        {
            btPersistentManifold* contactManifold = cw->getDispatcher()->getManifoldByIndexInternal(i);
            unsigned int numContacts = contactManifold->getNumContacts();
            for( unsigned int j=0; j<numContacts; j++ )
            {
                btManifoldPoint& pt = contactManifold->getContactPoint( j );
                if( ( pt.getDistance() <= 0.f ) && ( lastColState == false ) )
                {
                    // grab these values for the contact normal arrows:
                    vsg::vec3 pos = vsgbCollision::asVsgVec3( pt.getPositionWorldOnA() ); // position of the collision on object A
                    vsg::vec3 normal = vsgbCollision::asVsgVec3( pt.m_normalWorldOnB ); // returns a unit vector
                    float pen = pt.getDistance(); //penetration depth

                    vsg::quat q;
                    q=vsgbCollision::makeRotate( vsg::vec3( 0, 0, 1 ), normal );

                    std::cerr << "Collision detected." << std::endl;

                    std::cerr << "\tPosition: " << pos << std::endl;
                    std::cerr << "\tNormal: " << normal << std::endl;
                    std::cerr << "\tPenetration depth: " << pen << std::endl;
                    //std::cerr << q.w() <<","<< q.x() <<","<< q.y() <<","<< q.z() << std::endl;
                    lastColState = true;
                }
                else if( ( pt.getDistance() > 0.f ) && ( lastColState == true ) )
                {
                    std::cerr << "No collision." << std::endl;
                    lastColState = false;
                }
            }
        }
    }
}

int main( int argc,
         char * argv[] )
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
    windowTraits->windowTitle = "collision";
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

    btCollisionWorld * collisionWorld = initCollision();

        auto camera = vsg::Camera::create();//perspective, lookAt, vsg::ViewportState::create(window->extent2D()));
        auto mm=MoveManipulator::create(camera);
        vsg::ref_ptr< vsg::Group > root = createScene( collisionWorld, mm, arguments );

        vsg::ComputeBounds computeBounds;
        root->accept(computeBounds);
        vsg::dvec3 centre = (computeBounds.bounds.min + computeBounds.bounds.max);
        centre *= 0.5;
        float radius = vsg::length(computeBounds.bounds.max - computeBounds.bounds.min);// * 0.6;

        // set up the camera
        camera->viewMatrix = vsg::LookAt::create(centre + vsg::dvec3(0.0, -radius * 3.5, 0.0), centre, vsg::dvec3(0.0, 0.0, 1.0));
        camera->projectionMatrix = vsg::Perspective::create(30.0, static_cast<double>(window->extent2D().width) / static_cast<double>(window->extent2D().height), nearFarRatio * radius, radius * 100);
        camera->viewportState=vsg::ViewportState::create(window->extent2D());
        // add close handler to respond to the close window button and pressing escape
        viewer->addEventHandler(vsg::CloseHandler::create(viewer));
        //viewer->addEventHandler(vsgbInteraction::DragHandler::create(vsgbt_scene->getDynamicsWorld(), vsgbt_scene, camera, ellipsoidModel));
        //  viewer->addEventHandler(vsgbInteraction::LaunchHandler::create(vsgbt_scene->getDynamicsWorld(), vsgbt_scene, vsg::observer_ptr<vsg::Viewer>(viewer), camera, ellipsoidModel));
        viewer->addEventHandler( mm);
        auto commandGraph = vsg::createCommandGraphForView(window, camera, root);
        viewer->assignRecordAndSubmitTaskAndPresentation({commandGraph});
        viewer->compile();
        // viewer->addUpdateOperation(vsgbCollision::opMerge::create(vsg::observer_ptr<vsg::Viewer>(viewer),vsgbt_scene, newnode,viewer->compileManager->compile(newnode)));
        viewer->start_point() = vsg::clock::now();

        // rendering main loop
        bool lastColState = false;
        while (viewer->advanceToNextFrame() && (numFrames < 0 || (numFrames--) > 0) && (viewer->getFrameStamp()->simulationTime < maxTime))
        {
            collisionWorld->performDiscreteCollisionDetection();

            detectCollision( lastColState, collisionWorld );
           /* dbgDraw->BeginDraw();
            collisionWorld->debugDrawWorld();
            dbgDraw->EndDraw();*/
            // pass any events into EventHandlers assigned to the Viewer
            viewer->handleEvents();

            viewer->update();

            viewer->recordAndSubmit();

            viewer->present();
        }




    return( 0 );
}



/** \page collision Using osgBullet For Collision Detection
osgBullet consists of two libraries, vsgbCollision and vsgbDynamics. This
library split allows your application to use Bullet for collision detection
with no dependency on libBulletDynamics, and render your results with OSG.
osgBullet contains an example program, \c collision, to demonstrate this usage.

\c collision renders two boxes. You can view them from any angle using the
OSG TrackballManipulator, but the example behaves more intuitively if you
use the default home position.

Move the box on the right by holding down the control key and dragging
with your left mouse button. If you drag the right box so that it is in
collision with the left box, the following message appears on the console:

\code
Collision detected.
        Position: 0.5 0.5 0.5
        Normal: -1 -0 -0
        Penetration depth: -5.96046e-008
\endcode

The \c Position, \c normal, and \c Penetration \c depth values are taken from the
Bullet collision information.

Drag the right box away from the left box and the following message appears
on the console:

\code
No collision.
\endcode

Using osgBullet, your application interfaces directly with
the Bullet API to determine if a collision has occurred, and if so, which
collision objects are in collision. The \c collision
example detects collisions by examining the manifold count in the Bullet collision
dispatcher, but Bullet provides other ways to detect collisions, as
discussed in the Bullet documentation and online forum.
*/
