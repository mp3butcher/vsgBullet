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
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/TrackballManipulator>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/Geode>

#include <osgbDynamics/RigidBody.h>
#include <osgbDynamics/MotionState.h>
#include <osgbDynamics/GroundPlane.h>
#include <osgbCollision/CollisionShapes.h>
#include <osgbCollision/RefBulletObject.h>
#include <osgbCollision/Utils.h>
#include <osgbDynamics/TripleBuffer.h>
#include <osgbDynamics/PhysicsThread.h>
#include <osgbInteraction/DragHandler.h>
#include <osgbInteraction/LaunchHandler.h>
//#include <osgbInteraction/SaveRestoreHandler.h>

//#include <osgwTools/Shapes.h>

#include <btBulletDynamicsCommon.h>

#include <sstream>
#include <osg/io_utils>
#include <string>
#include <map>



osgbDynamics::TripleBuffer tBuf;
osgbDynamics::MotionStateList msl;


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


vsg::ref_ptr< vsg::Node > modelNode( NULL );

vsg::Transform*
makeModel( const std::string& fileName, const int index, btDynamicsWorld* bw, vsg::vec3 pos/*, osgbInteraction::SaveRestoreHandler* srh */)
{
    vsg::mat4 m( vsg::mat4::translate( pos ) );
    vsg::ref_ptr< osgbCollision::AbsoluteModelTransform > amt = new osgbCollision::AbsoluteModelTransform;
    amt->setDataVariance( vsg::Object::DYNAMIC );

    if( !modelNode.valid() )
	{
        modelNode = osgDB::readNodeFile( fileName );
		if( !modelNode.valid() )
		{
			std::cerr << "Can't find \"" << fileName << "\". Make sure OSG_FILE_PATH is set correctly." << std::endl;
			exit( 0 );
		}
	}
    amt->addChild( modelNode.get() );


    vsg::ref_ptr< osgbDynamics::CreationRecord > cr = new osgbDynamics::CreationRecord;
    cr->_sceneGraph = amt.get();
    cr->_shapeType = BOX_SHAPE_PROXYTYPE;
    cr->_mass = .2f;
    cr->_restitution = 0.3f;
    cr->_parentTransform = m;
    btRigidBody* rb = osgbDynamics::createRigidBody( cr.get() );

    rb->setActivationState( DISABLE_DEACTIVATION );


    // Set up for multithreading and triple buffering.
    osgbDynamics::MotionState* motion = static_cast< osgbDynamics::MotionState* >( rb->getMotionState() );
    motion->registerTripleBuffer( &tBuf );
    msl.insert( motion );

    std::ostringstream ostr;
    ostr << fileName << index;
   // srh->add( ostr.str(), rb );

    amt->setUserData( new osgbCollision::RefRigidBody( rb ) );
    bw->addRigidBody( rb );

    return( amt.release() );
}

vsg::MatrixTransform*
makeCow( btDynamicsWorld* bw, vsg::vec3 pos /*,osgbInteraction::SaveRestoreHandler* srh*/ )
{
    vsg::mat4 m( vsg::mat4::rotate( 1.5, vsg::vec3( 0., 0., 1. ) ) *
        vsg::mat4::translate( pos ) );
    vsg::MatrixTransform* root = new vsg::MatrixTransform( m );
    osgbCollision::AbsoluteModelTransform* amt = new osgbCollision::AbsoluteModelTransform;
    amt->setDataVariance( vsg::Object::DYNAMIC );
    root->addChild( amt );

	const std::string fileName( "cow.osg" );
    vsg::Node* node = osgDB::readNodeFile( fileName );
	if( node == NULL )
	{
		std::cerr << "Can't find \"" << fileName << "\". Make sure OSG_FILE_PATH includes the OSG sample data directory." << std::endl;
		exit( 0 );
	}
    amt->addChild( node );

    btCollisionShape* cs = osgbCollision::btConvexTriMeshCollisionShapeFromOSG( node );
    osgbDynamics::MotionState* motion = new osgbDynamics::MotionState();
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

   // srh->add( "cow", body );
    amt->setUserData( new osgbCollision::RefRigidBody( body ) );

    return( root );
}


int main( int argc, char** argv )
{
    // Increase triple buffer size to hold lots of transform data.
    tBuf.resize( 16384 );

    btDiscreteDynamicsWorld* bulletWorld = initPhysics();
    osgbDynamics::PhysicsThread pt( bulletWorld, &tBuf );
    vsg::Group* root = new vsg::Group;

    vsg::Group* launchHandlerAttachPoint = new vsg::Group;
    root->addChild( launchHandlerAttachPoint );

//    vsg::ref_ptr< osgbInteraction::SaveRestoreHandler > srh = new        osgbInteraction::SaveRestoreHandler;

    std::string fileName( "dice.osg" );
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
                root->addChild( makeModel( fileName, index++, bulletWorld, pos/*, srh.get()*/ ) );
            }
        }
        xStart += 1.25;
        yStart += 1.25;
        xCount--;
        yCount--;
        z += zInc;
    }

    // Add a cow
    root->addChild( makeCow( bulletWorld, vsg::vec3( -11., 6., 4. )/*, srh.get() */) );

    // Make ground.
    {
        vsg::Vec4 gp( 0, 0, 1, 0 );
        root->addChild( osgbDynamics::generateGroundPlane( gp, bulletWorld ) );
    }



    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow( 10, 30, 800, 600 );
    viewer.setSceneData( root );

    osgGA::TrackballManipulator* tb = new osgGA::TrackballManipulator;
    tb->setHomePosition( vsg::vec3( 0., -26., 12. ), vsg::vec3( 0., 0., 2. ), vsg::vec3( 0., 0., 1. ) );
    viewer.setCameraManipulator( tb );

    viewer.addEventHandler( new osgViewer::StatsHandler );

    // Create the launch handler.
 osgbInteraction::LaunchHandler* lh = new osgbInteraction::LaunchHandler();
     //lh->setWorld(root);
        lh->setAttachPoint( launchHandlerAttachPoint );
    {
        // Use a custom launch model: Sphere with radius 0.5 (instead of default 1.0).
        vsg::Geode* geode = new vsg::Geode;
        const double radius( .5 );
        geode->addDrawable( new vsg::ShapeDrawable(new vsg::Sphere( vsg::vec3(),radius)));//osgwTools::makeGeodesicSphere( radius ) );
        lh->setLaunchModel( geode, new btSphereShape( radius ) );
        lh->setInitialVelocity( 50. );

        viewer.addEventHandler( lh );
    }

  /*  srh->setLaunchHandler( lh );
    srh->capture();
    viewer.addEventHandler( srh.get() );*/
    osgbInteraction::DragHandler* dh = new osgbInteraction::DragHandler(
       /* bulletWorld, viewer.getCamera() */);
    viewer.addEventHandler( dh );

    lh->setThreadedPhysicsSupport( &pt, &tBuf, &msl );
   // srh->setThreadedPhysicsSupport( &pt );
    dh->setThreadedPhysicsSupport( &pt );


    viewer.realize();
    pt.setProcessorAffinity( 0 );
    pt.start();

    while( !viewer.done() )
    {
        // Get the latest transform information from the
        // Bullet simulation.
        TripleBufferMotionStateUpdate( msl, &tBuf );

        viewer.frame();
    }

    pt.stopPhysics();
    pt.join();

    return( 0 );
}



/** \page multithreaded The multithreaded Example
This examples demonstrates running the Bullet physics simultation in a separate thread.

\section multithreadedcontrols UI Controls

\li Delete: Reset the physics simulation to its initial state.
\li ctrl-leftmouse: Select and drag an object.
\li shift-leftmouse: Launches a sphere into the scene.

*/
