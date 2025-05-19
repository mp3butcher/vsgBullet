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

#include <vsgbDynamics/MotionState.h>
#include <vsgbDynamics/CreationRecord.h>
#include <vsgbDynamics/RigidBody.h>
#include <vsgbCollision/CollisionShapes.h>
#include <vsgbCollision/RefBulletObject.h>
#include <vsgbDynamics/GroundPlane.h>
#include <vsgbCollision/GLDebugDrawer.h>
#include <vsgbCollision/Utils.h>
#include <vsgbInteraction/DragHandler.h>
#include <vsgbInteraction/LaunchHandler.h>
/*#include <vsgbInteraction/SaveRestoreHandler.h>
*/
#include <vsgbCollision/GeometryOperation.h>
#include <vsgbCollision/GeometryModifier.h>
#include <btBulletDynamicsCommon.h>

#include <osg/io_utils>
#include <string>
#include <iostream>


// Filter out collisions between the gate and walls.
//
// Bullet collision filtering tutorial:
//   http://www.bulletphysics.com/mediawiki-1.5.8/index.php?title=Collision_Filtering
//
// Define filter groups

class FindNamedNode : public  vsg::Inherit<vsg::Intersector, FindNamedNode>
{
public:
    /**
    @param name Name of the Node to search for.
    */

    explicit FindNamedNode(const std::string &name, vsg::ref_ptr<vsg::ArrayState> initialArrayData = {}):
        vsg::Inherit<vsg::Intersector,FindNamedNode>(initialArrayData),
        _name( name ),
        _method( EXACT_MATCH ),
        _includeTargetNode( true ){

        localToWorldStack().push_back(vsg::dmat4());
        worldToLocalStack().push_back(vsg::dmat4());
    }
    ~FindNamedNode();

    bool intersectDraw(uint32_t firstVertex, uint32_t vertexCount, uint32_t firstInstance, uint32_t instanceCount) override{return true;}
    bool intersectDrawIndexed(uint32_t firstIndex, uint32_t indexCount, uint32_t firstInstance, uint32_t instanceCount) override{return true;}
    /// check for intersection with sphere
    bool intersects(const vsg::dsphere& bs) override{return true;}

    void pushTransform(const vsg::Transform& transform) override
    {
        auto& l2wStack = localToWorldStack();
        auto& w2lStack = worldToLocalStack();

        vsg::dmat4 localToWorld = l2wStack.empty() ? transform.transform(vsg::dmat4{}) : transform.transform(l2wStack.back());
        vsg::dmat4 worldToLocal = inverse(localToWorld);

        l2wStack.push_back(localToWorld);
        w2lStack.push_back(worldToLocal);

        // const auto& worldLineSegment = _lineSegmentStack.front();
        // _lineSegmentStack.push_back(LineSegment{worldToLocal * worldLineSegment.start, worldToLocal * worldLineSegment.end});
    }

    void popTransform() override
    {
        localToWorldStack().pop_back();
        worldToLocalStack().pop_back();
    }



    typedef std::pair< vsg::Node*, vsg::mat4 > NodeAndPath;
    typedef std::vector< NodeAndPath > NodeAndPathList;
    NodeAndPathList _napl;

    void reset();

    /**
    Algorithm for matching the specified name. Possible future
    work: support for case-insensitive matching.
    */
    typedef enum {
        EXACT_MATCH,
        CONTAINS
    } MatchMethod;
    /**
    Specifies the match algorithm.
    @param method The match algorithm. The default is EXACT_MATCH
    */
    void setMatchMethod( MatchMethod method );
    /**
    Gets the match algorithm.
    */
    MatchMethod getMatchMethod() const;

    /**
    Controls whether the named Node is included at the end of
    the NodePaths in _napl.
    @param includeTargetNode If false, don't include the named Node
    in the returned NodePaths. The default is true (include the named Node
    in the paths).
    */
    void setPathsIncludeTargetNode( bool includeTargetNode );
    /**
    Gets the current setting for including the named Node in the
    returned NodePaths.
    */
    bool getPathsIncludeTargetNode() const;

    /**
    Overrides of base class apply() method.
    */
    void apply(const vsg::Node& node) override{

        checkname(node);
        node.traverse(*this);
    }
    //void apply(const vsg::Group& node) override;
    void apply(const vsg::Transform& node) override{
        pushTransform(node);
        checkname(node);
        node.traverse(*this);
        popTransform();
    }
    void checkname(const vsg::Node& node) ;

protected:
    std::string _name;

    MatchMethod _method;
    bool _includeTargetNode;
};



FindNamedNode::~FindNamedNode()
{
}

void
FindNamedNode::reset()
{
    _napl.clear();
}

void
FindNamedNode::setMatchMethod( MatchMethod method )
{
    _method = method;
}
FindNamedNode::MatchMethod
FindNamedNode::getMatchMethod() const
{
    return( _method );
}

void
FindNamedNode::setPathsIncludeTargetNode( bool includeTargetNode )
{
    _includeTargetNode = includeTargetNode;
}
bool
FindNamedNode::getPathsIncludeTargetNode() const
{
    return( _includeTargetNode );
}


void
FindNamedNode::checkname(const vsg::Node& node )
{
    //_nodePath.push_back(&node);
    std::string tname;
    node.getValue("name",tname);
    bool match = (
        ( ( _method == EXACT_MATCH ) &&
            ( tname == _name ) ) ||
        ( ( _method == CONTAINS ) &&
            (tname.find( _name ) != std::string::npos ) ) );

    if( match )
    {
        // Copy the NodePath, so we can alter it if necessary.
        vsg::Intersector::NodePath np = this->_nodePath;//getNodePath();

        if( !_includeTargetNode )
            // Calling code has requested that the target node
            // be removed from the node paths.
            np.pop_back();

        NodeAndPath nap( const_cast<vsg::Node*>(&node), localToWorldStack().back() );
        _napl.push_back( nap );
    }

   // node.traverse( *this );_nodePath.pop_back();
}


enum CollisionTypes {
    COL_GATE = 0x1 << 0,
    COL_WALL = 0x1 << 1,
    COL_DEFAULT = 0x1 << 2,
};
// Define filter masks
unsigned int gateCollidesWith( COL_DEFAULT );
unsigned int wallCollidesWith( COL_DEFAULT );
unsigned int defaultCollidesWith( COL_GATE | COL_WALL | COL_DEFAULT );

//
// BEGIN WALL FIX
//

// The input model consists of two separate walls. However, the OSG scene
// graph for this is a single Geode with a single Geometry and a single
// QUADS PrimitiveSet. Our app needs to make this into two separate static
// collision shapes. One way to handle this situation would be to parse the
// geometry data and code directly to the Bullet API.
//
// Howver, for this example, I have instead chosen to sacrifice a little bit
// of rendering efficiency by fixing the scene graph, which will allow the
// example to use vsgbDynamics::createRigidBody() to automatically generate
// the collision shapes. In order for this to work, the scene graph must contain
// two Geodes, each with its own Geometry and PrimitiveSet. This allows the
// osgBullet rigid body create code to simple create one rigid body for
// each branch of the graph.
//
// The FindGeomOp is a GeometryOperation that returns a reference to the last
// Geometry found, which is all we need to locate the Geometry in question in
// this branch of the scene graph. The FixWalls function makes a copy of this
// scene graph branch, then uses FindGeomOp to locate the Geometry, then
// modifies the PrimitiveSet on each to only reference the vertices needed for
// each wall segment.
//
// Obviously, this code is very model specific, and is not intended for
// re-use with other models. Therefore I have wrapped it with BEGIN WALL FIX
// and END WALL FIX.

/* \cond */

void makeStaticObject( btDiscreteDynamicsWorld* bw, vsg::ref_ptr<vsg::Group> node, const vsg::mat4& m )
{
    vsg::ref_ptr< vsgbDynamics::CreationRecord > cr = vsgbDynamics::CreationRecord::create();
    cr->_sceneGraph = node;
    cr->_parentTransform = m;
    cr->_shapeType = CONVEX_HULL_SHAPE_PROXYTYPE;
    cr->_mass = 0.f;
    btRigidBody* rb = vsgbDynamics::createRigidBody( cr.get() );

    bw->addRigidBody( rb, COL_WALL, wallCollidesWith );//avoid collision with gate
    vsg::ref_ptr<vsg::Node> debugNode  (vsgbCollision::vsgNodeFromBtCollisionShape( rb->getCollisionShape() ));
   // node->addChild( debugNode );
}


btRigidBody* gateBody;
vsg::ref_ptr<vsg::Transform> makeGate( btDiscreteDynamicsWorld* bw, vsgbInteraction::SaveRestoreHandler* srh, vsg::ref_ptr<vsg::Node> node,vsg::ref_ptr<vsg::Group> parent, const vsg::mat4& m )
{
    //vsgbCollision::AbsoluteModelTransform* amt = new vsgbCollision::AbsoluteModelTransform; amt->setDataVariance( vsg::Object::DYNAMIC );
    vsg::ref_ptr<vsg::MatrixTransform> amt = vsg::MatrixTransform::create();

    //insertAbove( node, amt );
    parent->children.clear();
    amt->addChild(node);


    vsg::ref_ptr< vsgbDynamics::CreationRecord > cr =  vsgbDynamics::CreationRecord::create();
    cr->_sceneGraph = amt;
    cr->_shapeType = CONVEX_HULL_SHAPE_PROXYTYPE;
    cr->_parentTransform = m;
    cr->_mass = 1.f;
    cr->_restitution = .5f;
    btRigidBody* rb = vsgbDynamics::createRigidBody( cr.get() );


    bw->addRigidBody( rb);//, COL_GATE, gateCollidesWith );
    rb->setActivationState( DISABLE_DEACTIVATION );

    // Save RB in global, as AMT UserData (for DragHandler), and in SaveRestoreHandler.
    gateBody = rb;
   // amt->setUserData( new vsgbCollision::RefRigidBody( rb ) );
    srh->add( "gate", rb );
    vsg::ref_ptr<vsg::Node> debugNode  (vsgbCollision::vsgNodeFromBtCollisionShape( rb->getCollisionShape() ));
    //amt->addChild( debugNode );
    return( amt );
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

    dynamicsWorld->setGravity( btVector3( 0, 0, -9.81 ) );

    return( dynamicsWorld );
}


vsg::ref_ptr<vsg::Node> findNamedNode( vsg::Node* model, const std::string& name, vsg::mat4& xform )
{
    FindNamedNode fnn( name );
    model->accept( fnn );
    if( fnn._napl.empty() )
    {
        std::cerr << "hinge: Can't find node names \"" << name << "\"." << std::endl;
        return( nullptr );
    }
    auto nodepath=fnn._napl[ 0 ].second;
    xform = nodepath;
    return vsg::ref_ptr<vsg::Node>( fnn._napl[ 0 ].first );
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

    btDiscreteDynamicsWorld* bulletWorld = initPhysics();

    auto root = vsgbDynamics::World::create();
    root->setDynamicsWorld(bulletWorld);

    auto launchHandlerAttachPoint =  vsg::Group::create();
    root->addChild( launchHandlerAttachPoint );

    auto rootModel =vsg::read_cast<vsg::Group>("GateWall.vsgt",options);

    if( !rootModel.valid() )
    {
        std::cerr << "hinge: Can't load data file \"GateWall.vsgt\"." << std::endl;
        return( 1 );
    }
    vsg::ref_ptr<vsg::Group> vsgwallsNode =vsg::read_cast<vsg::Group>("wallsNode.vsgt",options);
    vsg::ref_ptr<vsg::Group> vsgotherWalls =vsg::read_cast<vsg::Group>("otherWall.vsgt",options);

    if( !rootModel.valid() )
    {
        std::cerr << "hinge: Can't load data file \"GateWall.vsgt\"." << std::endl;
        return( 1 );
    }

    root->addChild( rootModel );

    // Get Node pointers and parent transforms for the wall and gate.
    // (Node names are taken from the osgWorks osgwnames utility.)
    vsg::mat4 wallXform, gateXform;
    vsg::ref_ptr<vsg::Group> mywallNode = rootModel;
    vsg::ref_ptr<vsg::Group> mywallparent = rootModel;//
    while(mywallNode->children.size()<3)mywallNode=mywallNode->children[0].cast<vsg::Group>();

    mywallparent=mywallNode;
    mywallNode->children[0]->setValue("name","Walls");
    mywallNode=mywallNode->children[2].cast<vsg::Group>();
   // while(mywallNode->children[0].cast<vsg::Group>()&&mywallNode->children.size()<2)        mywallNode=mywallNode->children[0].cast<vsg::Group>();

    mywallNode->children[0]->setValue("name","DOF_Gate");

    std::cerr<<mywallNode->className()<< mywallNode->children[0]->className()<<std::endl;
    auto wallsNode=findNamedNode( rootModel.get(), "Walls", wallXform );
    auto gateNode = findNamedNode( rootModel.get(), "DOF_Gate", gateXform );
    if( ( wallsNode == nullptr ) || ( gateNode == nullptr ) )
        return( 1 );


    vsg::mat4 otherWallXform = wallXform;
    //


    vsg::ref_ptr< vsgbInteraction::SaveRestoreHandler > srh =  vsgbInteraction::SaveRestoreHandler::create();

    // Make Bullet rigid bodies and collision shapes for the gate...
   root->addChild(makeGate( bulletWorld, srh.get() ,vsg::ref_ptr<vsg::Node>(gateNode),mywallparent.cast<vsg::Group>(), gateXform ));
    // ...and the two walls.

   root->addChild(vsg::ref_ptr<vsg::Node>(vsgwallsNode));
   root->addChild(vsg::ref_ptr<vsg::Node>(vsgotherWalls));
   makeStaticObject( bulletWorld, vsgwallsNode, wallXform );
   makeStaticObject( bulletWorld, vsgotherWalls, otherWallXform );

    // Add ground
    const vsg::vec4 plane( 0., 0., 1., 0. );
    root->addChild( vsgbDynamics::generateGroundPlane( plane, bulletWorld,
        nullptr, COL_DEFAULT, defaultCollidesWith ) );


    // Create the hinge constraint.
    {
        // Pivot point and pivot axis are both in the gate's object space.
        // Note that the gate is COM-adjusted, so the pivot point must also be
        // in the gate's COM-adjusted object space.
        // TBD extract this from hinge data fine.
        const btVector3 btPivot( -0.0/*498f*/, -0.019f, 0.146f );

        btVector3 btAxisA( 0., 0., 1. );
        btHingeConstraint* hinge = new btHingeConstraint( *gateBody, btPivot, btAxisA );
        hinge->setLimit( -1.5f, 1.5f );
        bulletWorld->addConstraint( hinge, true );
    }


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
    auto launcher =vsgbInteraction::LaunchHandler::create(root, root, vsg::observer_ptr<vsg::Viewer>(viewer), camera, ellipsoidModel);
    launcher->setInitialVelocity(10);
    viewer->addEventHandler(launcher);

    viewer->addUpdateOperation(vsgbDynamics::BulletOperation::create(root), vsg::UpdateOperations::ALL_FRAMES);
    auto commandGraph = vsg::createCommandGraphForView(window, camera, root);
    viewer->assignRecordAndSubmitTaskAndPresentation({commandGraph});
    viewer->compile();
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
   /* TODO vsgbCollision::GLDebugDrawer* dbgDraw( nullptr );
    if( debugDisplay )
    {
        dbgDraw = new vsgbCollision::GLDebugDrawer();
        dbgDraw->setDebugMode( ~btIDebugDraw::DBG_DrawText );
        bulletWorld->setDebugDrawer( dbgDraw );
        root->addChild( dbgDraw->getSceneGraph() );
    }
*/


 /*
 vsg::ref_ptr<vsgbDynamics::World> _world = new vsgbDynamics::World;
 _world->setDynamicsWorld(bulletWorld);
 lh->setWorld(_world);
        lh->setAttachPoint( launchHandlerAttachPoint );
    {
        // Use a custom launch model: Sphere with radius 0.2 (instead of default 1.0).
        vsg::Geode* geode = new vsg::Geode;
        const double radius( .2 );
        geode->addDrawable( new vsg::ShapeDrawable(new vsg::Sphere(vsg::vec3(),radius)));//osgwTools::makeGeodesicSphere( radius ) );
        lh->setLaunchModel( geode, new btSphereShape( radius ) );
        lh->setInitialVelocity( 20. );

        // Also add the proper collision masks
        lh->setCollisionFilters( COL_DEFAULT, defaultCollidesWith );

        viewer.addEventHandler( lh );
    }

   srh->setLaunchHandler( lh );
    srh->capture();
    viewer.addEventHandler( srh.get() );*/
    
  /*  viewer.realize();

    double prevSimTime = 0.;
    while( !viewer.done() )
    {
        if( dbgDraw != nullptr )
            dbgDraw->BeginDraw();

        const double currSimTime = viewer.getFrameStamp()->getSimulationTime();
        bulletWorld->stepSimulation( 0.025 );
        prevSimTime = currSimTime;

        if( dbgDraw != nullptr )
        {
            bulletWorld->debugDrawWorld();
            dbgDraw->EndDraw();
        }

        viewer.frame();
    }
*/
    return( 0 );
}


/** \page hingelowlevel Simple Hinge Constraint

Demonstrates coding directly to the Bullet API to create a hinge constraint.

Use the --debug command line option to enable debug collision object display.

\section hingecontrols UI Controls

\li Delete: Reset the physics simulation to its initial state.
\li ctrl-leftmouse: Select and drag an object.
\li shift-leftmouse: Launches a sphere into the scene.

*/
