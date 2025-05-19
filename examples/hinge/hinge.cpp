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

#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/Geode>

#include <osgbDynamics/MotionState.h>
#include <osgbDynamics/CreationRecord.h>
#include <osgbDynamics/RigidBody.h>
#include <osgbCollision/CollisionShapes.h>
#include <osgbCollision/RefBulletObject.h>
#include <osgbDynamics/GroundPlane.h>
#include <osgbCollision/GLDebugDrawer.h>
#include <osgbCollision/Utils.h>
#include <osgbInteraction/DragHandler.h>
#include <osgbInteraction/LaunchHandler.h>
/*#include <osgbInteraction/SaveRestoreHandler.h>

#include <osgwTools/InsertRemove.h>
#include <osgwTools/FindNamedNode.h>

#include <osgwTools/Shapes.h>*/
#include <osgbCollision/GeometryOperation.h>
#include <osgbCollision/GeometryModifier.h>
#include <btBulletDynamicsCommon.h>

#include <osg/io_utils>
#include <string>
#include <map>



// Filter out collisions between the gate and walls.
//
// Bullet collision filtering tutorial:
//   http://www.bulletphysics.com/mediawiki-1.5.8/index.php?title=Collision_Filtering
//
// Define filter groups

class FindNamedNode : public vsg::NodeVisitor
{
public:
    /**
    @param name Name of the Node to search for.
    */
    FindNamedNode( const std::string& name, const vsg::NodeVisitor::TraversalMode travMode=vsg::NodeVisitor::TRAVERSE_ACTIVE_CHILDREN );
    ~FindNamedNode();

    typedef std::pair< vsg::Node*, vsg::NodePath > NodeAndPath;
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
    void apply( vsg::Node& node );

protected:
    std::string _name;

    MatchMethod _method;
    bool _includeTargetNode;
};

FindNamedNode::FindNamedNode( const std::string& name, const vsg::NodeVisitor::TraversalMode travMode )
  : vsg::NodeVisitor( travMode ),
    _name( name ),
    _method( EXACT_MATCH ),
    _includeTargetNode( true )
{
}

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
FindNamedNode::apply( vsg::Node& node )
{
    bool match = (
        ( ( _method == EXACT_MATCH ) &&
            ( node.getName() == _name ) ) ||
        ( ( _method == CONTAINS ) &&
            ( node.getName().find( _name ) != std::string::npos ) ) );

    if( match )
    {
        // Copy the NodePath, so we can alter it if necessary.
        vsg::NodePath np = getNodePath();

        if( !_includeTargetNode )
            // Calling code has requested that the target node
            // be removed from the node paths.
            np.pop_back();

        NodeAndPath nap( &node, np );
        _napl.push_back( nap );
    }

    traverse( node );
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
// example to use osgbDynamics::createRigidBody() to automatically generate
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
class FindGeomOp : public osgbCollision::GeometryOperation
{
public:
    FindGeomOp() {}
    FindGeomOp( const FindGeomOp& rhs, const vsg::CopyOp& copyOp={} ) {}
    META_Object(osgBulletExamples,FindGeomOp);

    virtual vsg::Geometry* operator()( vsg::Geometry& geom )
    {
        _target = &geom;
        return( &geom );
    }

    vsg::ref_ptr< vsg::Geometry > _target;
};
/* \endcond */

vsg::Node* fixWalls( vsg::Node* wallsNode )
{
    vsg::ref_ptr< vsg::Node > otherWall;
    {
        vsg::ref_ptr< vsg::Group > srcTempGroup = new vsg::Group;
        srcTempGroup->addChild( wallsNode );
        vsg::ref_ptr< vsg::Group > otherWallTempGroup = new vsg::Group( *srcTempGroup,
            vsg::CopyOp::DEEP_COPY_NODES | vsg::CopyOp::DEEP_COPY_DRAWABLES | vsg::CopyOp::DEEP_COPY_PRIMITIVES );
        otherWall = otherWallTempGroup->getChild( 0 );
    }

    unsigned int count;
    {
        vsg::ref_ptr< FindGeomOp > findGeom = new FindGeomOp;
        osgbCollision::GeometryModifier modifier( findGeom.get() );
        wallsNode->accept( modifier );

        vsg::Geometry* geom = findGeom->_target.get();
        vsg::DrawArrays* da = dynamic_cast< vsg::DrawArrays* >( geom->getPrimitiveSet( 0 ) );
        count = da->getCount();
        da->setCount( count / 2 );
    }
    {
        vsg::ref_ptr< FindGeomOp > findGeom = new FindGeomOp;
        osgbCollision::GeometryModifier modifier( findGeom.get() );
        otherWall->accept( modifier );

        vsg::Geometry* geom = findGeom->_target.get();
        vsg::DrawArrays* da = dynamic_cast< vsg::DrawArrays* >( geom->getPrimitiveSet( 0 ) );
        da->setFirst( count / 2 );
        da->setCount( count / 2 );
    }

    return( otherWall.release() );
}

//
// END WALL FIX
//



void makeStaticObject( btDiscreteDynamicsWorld* bw, vsg::Node* node, const vsg::mat4& m )
{
    vsg::ref_ptr< osgbDynamics::CreationRecord > cr = new osgbDynamics::CreationRecord;
    cr->_sceneGraph = node;
    cr->_shapeType = CONVEX_HULL_SHAPE_PROXYTYPE;
    cr->_mass = 0.f;
    btRigidBody* rb = osgbDynamics::createRigidBody( cr.get() );

    bw->addRigidBody( rb, COL_WALL, wallCollidesWith );
}

void
insertAbove( vsg::Node* node, vsg::Group* newParent )
{
    // Don't let the node get deleted when we remove it from all its parents.
    // Equivalent to explicit call to node->ref(), then node->unref() at end of function.
    vsg::ref_ptr< vsg::Node > nodeHolder( node );

    vsg::Node::ParentList pl = node->getParents();
    vsg::Node::ParentList::iterator it;
    for( it = pl.begin(); it != pl.end(); it++ )
    {
        vsg::Group* oldParent( *it );
        oldParent->addChild( newParent );
        oldParent->removeChild( node );
    }
    newParent->addChild( node );
}

btRigidBody* gateBody;
vsg::Transform* makeGate( btDiscreteDynamicsWorld* bw/*, osgbInteraction::SaveRestoreHandler* srh*/, vsg::Node* node, const vsg::mat4& m )
{
    osgbCollision::AbsoluteModelTransform* amt = new osgbCollision::AbsoluteModelTransform;
    amt->setDataVariance( vsg::Object::DYNAMIC );
    insertAbove( node, amt );

    vsg::ref_ptr< osgbDynamics::CreationRecord > cr = new osgbDynamics::CreationRecord;
    cr->_sceneGraph = amt;
    cr->_shapeType = CONVEX_HULL_SHAPE_PROXYTYPE;
    cr->setCenterOfMass( node->getBound().center() );
    cr->_parentTransform = m;
    cr->_mass = 1.f;
    cr->_restitution = .5f;
    btRigidBody* rb = osgbDynamics::createRigidBody( cr.get() );


    bw->addRigidBody( rb, COL_GATE, gateCollidesWith );
    rb->setActivationState( DISABLE_DEACTIVATION );

    // Save RB in global, as AMT UserData (for DragHandler), and in SaveRestoreHandler.
    gateBody = rb;
    amt->setUserData( new osgbCollision::RefRigidBody( rb ) );
  //  srh->add( "gate", rb );

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


vsg::Node* findNamedNode( vsg::Node* model, const std::string& name, vsg::mat4& xform )
{
    FindNamedNode fnn( name );
    model->accept( fnn );
    if( fnn._napl.empty() )
    {
        std::cerr << "hinge: Can't find node names \"" << name << "\"." << std::endl;
        return( NULL );
    }
    xform = vsg::computeLocalToWorld( fnn._napl[ 0 ].second );
    return( fnn._napl[ 0 ].first );
}

int main( int argc, char** argv )
{
    vsg::ArgumentParser arguments( &argc, argv );
    const bool debugDisplay( arguments.find( "--debug" ) > 0 );

    btDiscreteDynamicsWorld* bulletWorld = initPhysics();
    vsg::Group* root = new vsg::Group;

    vsg::Group* launchHandlerAttachPoint = new vsg::Group;
    root->addChild( launchHandlerAttachPoint );


    vsg::ref_ptr< vsg::Node > rootModel = osgDB::readNodeFile( "GateWall.flt" );
    if( !rootModel.valid() )
    {
        std::cerr << "hinge: Can't load data file \"GateWall.flt\"." << std::endl;
        return( 1 );
    }

    root->addChild( rootModel.get() );

    // Get Node pointers and parent transforms for the wall and gate.
    // (Node names are taken from the osgWorks osgwnames utility.)
    vsg::mat4 wallXform, gateXform;
    vsg::Node* wallsNode = findNamedNode( rootModel.get(), "Walls", wallXform );
    vsg::Node* gateNode = findNamedNode( rootModel.get(), "DOF_Gate", gateXform );
    if( ( wallsNode == NULL ) || ( gateNode == NULL ) )
        return( 1 );

    // BEGIN WALL FIX
    //
    // Unfortunately, the two walls come to us as a single Geode with
    // a single Geometry and single PrimitiveSet. Break that into two Geodes, a
    // left wall and a right wall, so we can make a collision shape for each.
    vsg::Node* otherWall = fixWalls( wallsNode );
    wallsNode->getParent( 0 )->addChild( otherWall );
    otherWall->setName( "otherWall" );
    vsg::mat4 otherWallXform = wallXform;
    //
    // END WALL FIX


//    vsg::ref_ptr< osgbInteraction::SaveRestoreHandler > srh = new        osgbInteraction::SaveRestoreHandler;

    // Make Bullet rigid bodies and collision shapes for the gate...
    makeGate( bulletWorld/*, srh.get() */,gateNode, gateXform );
    // ...and the two walls.
    makeStaticObject( bulletWorld, wallsNode, wallXform );
    makeStaticObject( bulletWorld, otherWall, otherWallXform );

    // Add ground
    const vsg::Vec4 plane( 0., 0., 1., 0. );
    root->addChild( osgbDynamics::generateGroundPlane( plane, bulletWorld,
        NULL, COL_DEFAULT, defaultCollidesWith ) );


    // Create the hinge constraint.
    {
        // Pivot point and pivot axis are both in the gate's object space.
        // Note that the gate is COM-adjusted, so the pivot point must also be
        // in the gate's COM-adjusted object space.
        // TBD extract this from hinge data fine.
        const btVector3 btPivot( -0.498f, -0.019f, 0.146f );

        btVector3 btAxisA( 0., 0., 1. );
        btHingeConstraint* hinge = new btHingeConstraint( *gateBody, btPivot, btAxisA );
        hinge->setLimit( -1.5f, 1.5f );
        bulletWorld->addConstraint( hinge, true );
    }


    osgbCollision::GLDebugDrawer* dbgDraw( NULL );
    if( debugDisplay )
    {
        dbgDraw = new osgbCollision::GLDebugDrawer();
        dbgDraw->setDebugMode( ~btIDebugDraw::DBG_DrawText );
        bulletWorld->setDebugDrawer( dbgDraw );
        root->addChild( dbgDraw->getSceneGraph() );
    }


    osgViewer::Viewer viewer( arguments );
    viewer.setUpViewInWindow( 30, 30, 768, 480 );
    viewer.setSceneData( root );

    osgGA::TrackballManipulator* tb = new osgGA::TrackballManipulator;
    tb->setHomePosition( vsg::vec3( 0., -8., 2. ), vsg::vec3( 0., 0., 1. ), vsg::vec3( 0., 0., 1. ) );
    viewer.setCameraManipulator( tb );
    viewer.getCamera()->setClearColor( vsg::Vec4( .5, .5, .5, 1. ) );


    // Create the launch handler.
 osgbInteraction::LaunchHandler* lh = new osgbInteraction::LaunchHandler();

 vsg::ref_ptr<osgbDynamics::World> _world = new osgbDynamics::World;
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

  /*  srh->setLaunchHandler( lh );
    srh->capture();
    viewer.addEventHandler( srh.get() );*/
    
    viewer.realize();

    double prevSimTime = 0.;
    while( !viewer.done() )
    {
        if( dbgDraw != NULL )
            dbgDraw->BeginDraw();

        const double currSimTime = viewer.getFrameStamp()->getSimulationTime();
        bulletWorld->stepSimulation( 0.025 );
        prevSimTime = currSimTime;

        if( dbgDraw != NULL )
        {
            bulletWorld->debugDrawWorld();
            dbgDraw->EndDraw();
        }

        viewer.frame();
    }

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
