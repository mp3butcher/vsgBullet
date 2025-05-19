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

#include <vsg/all.h>

#ifdef vsgXchange_FOUND
#    include <vsgXchange/all.h>
#endif


#include <vsg/utils/SharedObjects.h>
#include <vsg/nodes/MatrixTransform.h>

#include <vsgbDynamics/GroundPlane.h>
#include <vsgbDynamics/MotionState.h>
#include <vsgbDynamics/RigidBody.h>
#include <vsgbCollision/RefBulletObject.h>
#include <vsgbCollision/Utils.h>
//#include <vsgbInteraction/SaveRestoreHandler.h>
#include <vsgbInteraction/DragHandler.h>

#include <btBulletDynamicsCommon.h>
#include <sstream>


btRigidBody* createObject( vsg::Group* parent, const vsg::mat4& m,
                        //  vsgbInteraction::SaveRestoreHandler* srh,
                          const vsg::vec3& com=vsg::vec3(0,0,0), bool setCom=false )
{
    //vsg::Node* node = osgDB::readNodeFile( "com.vsgt" );
    auto options = vsg::Options::create();
    options->sharedObjects = vsg::SharedObjects::create();
    options->fileCache = vsg::getEnv("VSG_FILE_CACHE");
    options->paths = vsg::getEnvPaths("VSG_FILE_PATH");
    vsg::ref_ptr<vsg::Node>node =vsg::read_cast<vsg::Node>("com.vsgt",options);

    if( node == nullptr )
    {
        vsg::warn( "Can't load file \"com.vsgt\". Make sure osgBullet data directory is in OSG_FILE_PATH.");
        return( nullptr );
    }

    auto mt = vsg::MatrixTransform::create();
    parent->addChild( mt );
    mt->addChild( node );

    // Begin_doxygen example code block
    vsg::ref_ptr< vsgbDynamics::CreationRecord > cr =  vsgbDynamics::CreationRecord::create();
    if( setCom )
        cr->setCenterOfMass( com );
    cr->_sceneGraph = mt;
    cr->_shapeType = BOX_SHAPE_PROXYTYPE;
    cr->_parentTransform = m;
    cr->_restitution = 1.f;
    btRigidBody* rb = vsgbDynamics::createRigidBody( cr.get() );
    // End_doxygen example code block

    rb->setAngularVelocity( btVector3( 0., .2, 0. ) );

    //mt->setUserData( new vsgbCollision::RefRigidBody( rb ) );
    std::ostringstream id;
    id << std::hex << mt;
   // srh->add( id.str(), rb );

    return( rb );
}


btDiscreteDynamicsWorld* initPhysics()
{
    btDefaultCollisionConfiguration * collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher * dispatcher = new btCollisionDispatcher( collisionConfiguration );
    btConstraintSolver * solver = new btSequentialImpulseConstraintSolver;

    btVector3 worldAabbMin( -10000, -10000, -10000 );
    btVector3 worldAabbMax( 10000, 10000, 10000 );
    btBroadphaseInterface * inter = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );

    btDiscreteDynamicsWorld * dynamicsWorld = new btDiscreteDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );

    dynamicsWorld->setGravity( btVector3( 0, 0, -9.8 ) );

    return( dynamicsWorld );
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
        vsg::info( "Enabled depth clamp." );
        auto deviceFeatures = windowTraits->deviceFeatures = vsg::DeviceFeatures::create();
        deviceFeatures->get().samplerAnisotropy = VK_TRUE;
        deviceFeatures->get().depthClamp = VK_TRUE;
    }

    // create the viewer and assign window(s) to it
    auto viewer = vsg::Viewer::create();
    auto window = vsg::Window::create(windowTraits);
    if (!window)
    {
        vsg::info( "Could not create window." );
        return 1;
    }

    viewer->addWindow(window);

    btDiscreteDynamicsWorld* bw = initPhysics();
    auto root = vsgbDynamics::World::create();
    root->setDynamicsWorld(bw);

   // vsg::ref_ptr< vsgbInteraction::SaveRestoreHandler > srh = new        vsgbInteraction::SaveRestoreHandler;

    vsg::mat4 m;

    // Left object:
    // Create the object on the left, which has center of mass at the model origin.
    // In this case, that's the lower-left front of the model. This is almost certainly
    // NOT what you want, but is what you would get with a naive conversion of OSG data
    // into a collision shape.
    m = vsg::translate( -24., 0., 10. )*vsg::rotate( .4, 0., 0., 1. )  ;
    bw->addRigidBody( createObject( root, m
    /*, srh.get()*/
   , vsg::vec3( 0., 0., 0. ), true ) );

    // Center object:
    // Specify the actual center of the mass of the model, in the model's own coordinate
    // space. In the case of this model, the COM is approx ( 2.15, 3., 2. ). This results
    // in much more realistic dynamics.
    m = vsg::translate( -4., 0., 10. )*vsg::rotate( .4, 0., 0., 1. )  ;
    bw->addRigidBody( createObject( root, m    /*, srh.get()*/, vsg::vec3( 2.15, 3., 2. ), true ) );

    // Right object:
    // If you don't specify the center of mass, osgBullet tries to do you a favor, and
    // uses the bounding volume center as the COM. This works well in a lot of cases, but
    // for a model such as com.vsgt, you really should specify the COM expliticly.
    m = vsg::translate( 16., 0., 10. )*vsg::rotate( .4, 0., 0., 1. ) ;
    bw->addRigidBody( createObject( root, m    /*, srh.get()*/ ) );

    root->addChild( vsgbDynamics::generateGroundPlane( vsg::vec4( 0.f, 0.f, 1.f, 0.f ), bw ) );


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
    viewer->addEventHandler(vsgbInteraction::DragHandler::create(root, camera, ellipsoidModel));
    //      viewer->addEventHandler(vsgbInteraction::LaunchHandler::create(vsgbt_scene, glider, vsg::observer_ptr<vsg::Viewer>(viewer), camera, ellipsoidModel));

    viewer->addUpdateOperation(vsgbDynamics::BulletOperation::create(root), vsg::UpdateOperations::ALL_FRAMES);
    auto commandGraph = vsg::createCommandGraphForView(window, camera, root);
    viewer->assignRecordAndSubmitTaskAndPresentation({commandGraph});
    viewer->compile();   viewer->start_point() = vsg::clock::now();

    // rendering main loop
    while (viewer->advanceToNextFrame() && (numFrames < 0 || (numFrames--) > 0) && (viewer->getFrameStamp()->simulationTime < maxTime))
    {
        // pass any events into EventHandlers assigned to the Viewer
        viewer->handleEvents();

        viewer->update();

        viewer->recordAndSubmit();

        viewer->present();
    }
    return( 0 );
}


/** \page examplecom Support For Non-Origin Center Of Mass
Bullet dynamics always assumes the collision shape origin is the center of mass (COM).
However, 3D model origins rarely coincide with the COM. A simple conversion of OSG
geometric data into a collision shape would almost certainly result in incorrect
dynamics due to this issue.

osgBullet supports 3D models with non-origin COM. The support is implemented in the
\ref vsgbDynamics::MotionState "MotionState" class and the
\link collisionshapes collision shape creation utilities. \endlink
If your application uses the
\link rigidbody rigid body creation utilities, \endlink
then you merely specify the COM in the
\ref vsgbDynamics::CreationRecord "CreationRecord".

\dontinclude centerofmass.cpp
\skipline Begin_doxygen
\until createRigidBody

Note that the call to setCenterOfMass() is conditional; if you
don't call it, osgBullet will use the bounding volume center
as the COM. However, osgBullet never uses the origin as the COM,
unless you specify it directly with a call to setCenterOfMass()
(or it just happens to coincide with the bounding volume center).

If your application doesn't use the
\link rigidbody rigid body creation utilities, \endlink
then you should emulate the RigidBody.cpp source code.

\section examdescrip The centerofmass Example

The \c centerofmass example demonstrates setting COM on
a 3D model. It uses the \c com.vsgt model file, which has
its origin at the lower-left front corner. To see the
model origin, use the osgWorks utility \c osgwbvv:

\code
osgwbvv com.vsgt --box --origin
\endcode

Clearly, using the model origin as the COM would be incorrect. You can
see this incorrect behavior when you run the example; look at the model
on the left. If osgBullet (or your application) were to do a naive
comversion of OSG geometric data to a Bullet collision shape, this is
what you'd get.

If you don't specify a COM at all (that is, your application doesn't call
vsgbDynamics::CreationRecord::setCenterOfMass() ), osgBullet uses the
subgraph bounding volume center as the center of mass. This works well for
many models, but not for \c com.vsgt. To see this incorrect behavior, run
the example and look at the model on the right.

The object in the center of the example has a correct COM, which produces
correct rigid body dynamics. This was accomplished using the code above.

\section comcontrols UI Controls

\li Delete: Reset the physics simulation to its initial state.
\li ctrl-leftmouse: Select and drag an object.

*/
