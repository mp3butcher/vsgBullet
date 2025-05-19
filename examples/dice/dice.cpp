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
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/Geode>

#include <vsgbDynamics/MotionState.h>
#include <vsgbCollision/CollisionShapes.h>
#include <vsgbDynamics/RigidBody.h>
#include <vsgbCollision/Utils.h>

#include <btBulletDynamicsCommon.h>

#include <string>
#include <osg/io_utils>



vsg::MatrixTransform*
makeDie( btDynamicsWorld* bw )
{
    vsg::MatrixTransform* root = new vsg::MatrixTransform;
	const std::string fileName( "dice.osg" );
    vsg::Node* node = osgDB::readNodeFile( fileName );
	if( node == nullptr )
	{
		std::cerr << "Can't find \"" << fileName << "\". Make sure OSG_FILE_PATH includes the osgBullet data directory." << std::endl;
		exit( 0 );
	}
    root->addChild( node );

    btCollisionShape* cs = vsgbCollision::btBoxCollisionShapeFromOSG( node );
    
    vsg::ref_ptr< vsgbDynamics::CreationRecord > cr = new vsgbDynamics::CreationRecord;
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


/* \cond */
class ShakeManipulator : public osgGA::GUIEventHandler
{
public:
    ShakeManipulator( vsgbDynamics::MotionState* motion )
      : _motion( motion )
    {}

    bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& )
    {
        switch( ea.getEventType() )
        {
            case osgGA::GUIEventAdapter::KEYUP:
            {
                if (ea.getKey()==osgGA::GUIEventAdapter::KEY_Space)
                {
                    btTransform trans; trans.setIdentity();
                    _motion->setWorldTransform( trans );

                    return true;
                }

                return false;
            }

            case osgGA::GUIEventAdapter::PUSH:
            {
                _lastX = ea.getXnormalized();
                _lastY = ea.getYnormalized();

                btTransform world;
                _motion->getWorldTransform( world );
                btVector3 o = world.getOrigin();
                o[ 2 ] = 0.25;
                world.setOrigin( o );
                _motion->setWorldTransform( world );

                return true;
            }
            case osgGA::GUIEventAdapter::DRAG:
            {
                btVector3 move;
                move[ 0 ] = _lastX - ea.getXnormalized();
                move[ 1 ] = ea.getYnormalized() - _lastY;
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

                _lastX = ea.getXnormalized();
                _lastY = ea.getYnormalized();

                return true;
            }
            default:
            break;
        }
        return false;
    }

protected:
    vsgbDynamics::MotionState* _motion;
    float _lastX, _lastY;
};
/* \endcond */




vsg::Geode* osgBox( const vsg::vec3& center, const vsg::vec3& halfLengths )
{
    vsg::vec3 l( halfLengths * 2. );
    vsg::Box* box = new vsg::Box( center, l.x(), l.y(), l.z() );
    vsg::ShapeDrawable* shape = new vsg::ShapeDrawable( box );
    shape->setColor( vsg::Vec4( 1., 1., 1., 1. ) );
    vsg::Geode* geode = new vsg::Geode();
    geode->addDrawable( shape );
    return( geode );
}



int
main( int argc,
      char ** argv )
{
    btDynamicsWorld* bulletWorld = initPhysics();
    vsg::Group* root = new vsg::Group;

    root->addChild( makeDie( bulletWorld ) );
    root->addChild( makeDie( bulletWorld ) );


    /* BEGIN: Create environment boxes */
    float xDim( 10. );
    float yDim( 10. );
    float zDim( 6. );
    float thick( .1 );

    vsg::MatrixTransform* shakeBox = new vsg::MatrixTransform;
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
    { // top +Z (invisible, to allow user to see through; no OSG analogue
        vsg::vec3 halfLengths( xDim*.5, yDim*.5, thick*.5 );
        vsg::vec3 center( 0., 0., -zDim*.5 );
        //shakeBox->addChild( osgBox( center, halfLengths ) );
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
    { // bottom of window -Y
        vsg::vec3 halfLengths( xDim*.5, thick*.5, zDim*.5 );
        vsg::vec3 center( 0., -yDim*.5, 0. );
        shakeBox->addChild( osgBox( center, halfLengths ) );
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

    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow( 150, 150, 400, 400 );
    viewer.setSceneData( root );
    viewer.getCamera()->setViewMatrixAsLookAt(
        vsg::vec3( 0, 0, -20 ), vsg::vec3( 0, 0, 0 ), vsg::vec3( 0, 1, 0 ) );
    viewer.getCamera()->setProjectionMatrixAsPerspective( 40., 1., 1., 50. );
    viewer.addEventHandler( new ShakeManipulator( shakeMotion ) );

    viewer.realize();
    double prevSimTime = 0.;
    while( !viewer.done() )
    {
        const double currSimTime = viewer.getFrameStamp()->getSimulationTime();
        double elapsed( currSimTime - prevSimTime );
        if( viewer.getFrameStamp()->getFrameNumber() < 3 )
            elapsed = 1./60.;
        //std::cerr << elapsed / 3. << ", " << 1./180. << std::endl;
        bulletWorld->stepSimulation( elapsed, 4, elapsed/4. );
        prevSimTime = currSimTime;
        viewer.frame();
    }

    return( 0 );
}


/** \page diceexample The Mandatory Dice Example
No physics-based project would be complete without a dice example. Use the
left mouse button to chake the dice shaker.
*/
