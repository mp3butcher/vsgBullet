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
#include <vsgbCollision/CollisionShapes.h>
#include <vsgbCollision/Utils.h>

#include <vsgbCollision/GLDebugDrawer.h>
//#include <osgwTools/Shapes.h>
//#include <osgwTools/Version.h

#include <iostream>


/* \cond
class MoveManipulator : public osgGA::GUIEventHandler
{
public:
    MoveManipulator() : _co( nullptr ), _mt( nullptr ) {}
    MoveManipulator( const MoveManipulator& mm, vsg::CopyOp copyop ) : _co( mm._co ), _mt( mm._mt ) {}
    ~MoveManipulator() {}
#if( OSGWORKS_OSG_VERSION > 20800 )
    META_Object(osgBulletExample,MoveManipulator);
#endif

    virtual bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
    {
        if( ( ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL ) == 0 )
        {
            return( false );
        }
        else if( ea.getEventType() == osgGA::GUIEventAdapter::PUSH )
        {
            _lastX = ea.getXnormalized();
            _lastY = ea.getYnormalized();
            return( true );
        }
        else if( ea.getEventType() == osgGA::GUIEventAdapter::DRAG )
        {
            double deltaX = ea.getXnormalized() - _lastX;
            double deltaY = ea.getYnormalized() - _lastY;
            _lastX = ea.getXnormalized();
            _lastY = ea.getYnormalized();

            deltaX *= 6.;
            deltaY *= 6.;
            vsg::mat4 trans = vsgbCollision::asOsgMatrix( _co->getWorldTransform() );
            trans = trans * vsg::mat4::translate( deltaX, 0., deltaY );
            _mt->setMatrix( trans );
            _co->setWorldTransform( vsgbCollision::asBtTransform( trans ) );
            return( true );
        }
        return( false );
    }

    void setCollisionObject( btCollisionObject* co ) { _co = co; }
    void setMatrixTransform( vsg::MatrixTransform* mt ) { _mt = mt; }

protected:
    btCollisionObject* _co;
    vsg::MatrixTransform* _mt;
    double _lastX, _lastY;
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


vsg::Node* createScene( btCollisionWorld* cw, MoveManipulator* mm, vsg::ArgumentParser& arguments )
{
    vsg::ref_ptr< vsg::Group > root = new vsg::Group;

    // Create a static box
    vsg::Geode* geode = new vsg::Geode;
    geode->addDrawable( /*osgwTools::makeBox*/new vsg::ShapeDrawable(new vsg::Box(vsg::vec3(),1.0)));// vsg::vec3( .5, .5, .5 ) ) );
    root->addChild( geode );

    btCollisionObject* btBoxObject = new btCollisionObject;
    btBoxObject->setCollisionShape( vsgbCollision::btBoxCollisionShapeFromOSG( geode ) );
    btBoxObject->setCollisionFlags( btCollisionObject::CF_STATIC_OBJECT );
    cw->addCollisionObject( btBoxObject );


    // Create a box we can drag around with the mouse
    geode = new vsg::Geode;
    geode->addDrawable( /*osgwTools::makeBox*/new vsg::ShapeDrawable(new vsg::Box(vsg::vec3(),1.0)));// vsg::vec3( .5, .5, .5 ) ) );

    vsg::mat4 transMatrix = vsg::mat4::translate( 4., 0., 0. );
    vsg::MatrixTransform* mt = new vsg::MatrixTransform( transMatrix );
    mt->addChild( geode );
    root->addChild( mt );

    btBoxObject = new btCollisionObject;
    btBoxObject->setCollisionShape( vsgbCollision::btBoxCollisionShapeFromOSG( geode ) );
    btBoxObject->setCollisionFlags( btCollisionObject::CF_KINEMATIC_OBJECT );
    btBoxObject->setWorldTransform( vsgbCollision::asBtTransform( transMatrix ) );
    cw->addCollisionObject( btBoxObject );
    mm->setCollisionObject( btBoxObject );
    mm->setMatrixTransform( mt );


    return( root.release() );
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
                    vsg::vec3 pos = vsgbCollision::asOsgVec3( pt.getPositionWorldOnA() ); // position of the collision on object A
                    vsg::vec3 normal = vsgbCollision::asOsgVec3( pt.m_normalWorldOnB ); // returns a unit vector
                    float pen = pt.getDistance(); //penetration depth

                    vsg::Quat q;
                    q.makeRotate( vsg::vec3( 0, 0, 1 ), normal );

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
    btCollisionWorld* collisionWorld = initCollision();

    vsg::ArgumentParser arguments( &argc, argv );
    MoveManipulator* mm = new MoveManipulator;
    vsg::ref_ptr< vsg::Group > root =(vsg::Group*) createScene( collisionWorld, mm, arguments );

    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow( 10, 30, 800, 600 );
    viewer.setCameraManipulator( new osgGA::TrackballManipulator() );
    viewer.addEventHandler( mm );

           vsgbCollision::GLDebugDrawer* dbgDraw = new vsgbCollision::GLDebugDrawer();
        dbgDraw->setDebugMode( ~btIDebugDraw::DBG_DrawText );
        collisionWorld->setDebugDrawer( dbgDraw );
        root->addChild( dbgDraw->getSceneGraph() );

    viewer.setSceneData( root.get() );



    bool lastColState = false;
    while( !viewer.done() )
    {
        collisionWorld->performDiscreteCollisionDetection();

        detectCollision( lastColState, collisionWorld );
          dbgDraw->BeginDraw();

        collisionWorld->debugDrawWorld();
            dbgDraw->EndDraw();
        viewer.frame();
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
