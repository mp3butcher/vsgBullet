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

#include <vsgbInteraction/LaunchHandler.h>
#include <btBulletDynamicsCommon.h>
#include <vsgbDynamics/RigidBody.h>
#include <vsgbCollision/CollisionShapes.h>
#include <vsgbCollision/RefBulletObject.h>
#include <vsgbCollision/Utils.h>
#include <vsgbDynamics/PhysicsThread.h>
#include <vsgbDynamics/TripleBuffer.h>
//#include <vsgbCollision/Shapes.h>


namespace vsgbInteraction
{

LaunchHandler::LaunchHandler(vsg::ref_ptr<vsgbDynamics::World> w, vsg::ref_ptr<vsg::Group> attachPoint , vsg::observer_ptr<vsg::Viewer> refviewer ,vsg::ref_ptr<vsg::Camera> camera, vsg::ref_ptr<vsg::EllipsoidModel> ellipsoidModel)
    : vsg::Inherit<vsg::Trackball,LaunchHandler>(camera, ellipsoidModel),
    _refviewer(refviewer), _world( w ), _attachPoint( attachPoint ),
    _launchCollisionShape( nullptr ),
    _initialVelocity( 100. ),
    _launchedMass( 1. ),
    _group( 0 ),
    _mask( 0 ),
    _pt( nullptr ),
    _tb( nullptr ),
    _msl( nullptr )
{
    // Make the default launch model: Sphere with radius 1.0.
    const double radius( 0.5 );
    _launchCollisionShape = new btSphereShape(radius);
    _launchModel = vsgbCollision::vsgDrawableFromBtCollisionShape((btSphereShape*)_launchCollisionShape);
    _ownsCollisionShape = true;
}


LaunchHandler::~LaunchHandler()
{
    reset();
    if( ( _launchCollisionShape != nullptr ) && _ownsCollisionShape )
        delete _launchCollisionShape;
}

void LaunchHandler::setThreadedPhysicsSupport( vsgbDynamics::PhysicsThread* pt, vsgbDynamics::TripleBuffer* tb, vsgbDynamics::MotionStateList* msl )
{
    _pt = pt;
    _tb = tb;
    _msl = msl;
}

void LaunchHandler::setLaunchModel( vsg::Node* model, btCollisionShape* shape )
{
    _launchModel = model;

    if( ( _launchCollisionShape != nullptr ) && _ownsCollisionShape )
        delete _launchCollisionShape;

    if( shape != nullptr )
    {
        _launchCollisionShape = shape;
        _ownsCollisionShape = false;
    }
    else
    {
        btConvexHullShape* ch = vsgbCollision::btConvexHullCollisionShapeFromVSG( model );
        ch->setMargin( 0.0 );
        _launchCollisionShape = ch;
        _ownsCollisionShape = true;
    }
}



void LaunchHandler::apply(vsg::ButtonPressEvent& buttonPress)
{
    if (buttonPress.handled || !eventRelevant(buttonPress))
    {
        _hasKeyboardFocus = false;
        return;
    }
    if(_ctrlpressed)
    {
        auto lookat = _camera->viewMatrix.cast<vsg::LookAt>();
        // vsg::vec3 look, at, up;

        auto proj = _camera->projectionMatrix.cast<vsg::Perspective>();
        // double fovy, aspect, zNear, zFar;
        // proj.getPerspective( fovy, aspect, zNear, zFar );
        //view.invert( view );
        //proj.invert( proj );
        //vsg::vec4 clip( proj->farDistance, proj->farDistance, proj->farDistance, proj->farDistance);

        vsg::vec3 launchPos =vsg::vec3(lookat->up);
        launchPos*= ( 1. * 2. ) ;
        launchPos+= vsg::vec3(lookat->eye) ;


        vsg::mat4 parentTrans = vsg::translate( launchPos );
        vsg::vec3 launchDir =vsg::vec3( lookat->center-lookat->eye);
        launchDir = normalize(launchDir);

        //vsg::ref_ptr< vsgbDynamics::RigidBody > rig =  vsgbDynamics::RigidBody::create();
        vsg::ref_ptr< vsg::MatrixTransform > rig =  vsg::MatrixTransform::create();
        rig->addChild( _launchModel );

        vsg::ref_ptr<vsg::Viewer> viewer(_refviewer);
        viewer->addUpdateOperation(vsgbCollision::opMerge::create(_refviewer, _attachPoint, rig, viewer->compileManager->compile(rig)));
        _nodeList.push_back( rig );

        vsg::ref_ptr< vsgbDynamics::CreationRecord > cr = vsgbDynamics::CreationRecord::create();
        cr->_sceneGraph = rig.get();
        cr->_mass = getLaunchedMass();
        cr->_parentTransform = parentTrans;
        btRigidBody* rb = vsgbDynamics::createRigidBody( cr.get(), _launchCollisionShape );
        launchDir *= _initialVelocity;
        rb->setLinearVelocity( vsgbCollision::asBtVector3( launchDir ) );
        rb->setAngularVelocity( btVector3( .2, .3, 1.5 ) );

        vsgbDynamics::MotionState* motion = static_cast< vsgbDynamics::MotionState* >( rb->getMotionState() );
        if( _tb != nullptr )
            motion->registerTripleBuffer( _tb );
        if( _msl != nullptr )
            _msl->insert( motion );

        if( _pt != nullptr )
            _pt->pause( true );

        bool added( false );
       // rig->setRigidBody(rb);
        rig->setValue("btRigidBody", new vsgbCollision::RefRigidBody( rb ) );
          btDiscreteDynamicsWorld* ddw =0;
        if(_world.valid() &&   ( ddw=_world->getDynamicsWorld()) )
            if( (_group != 0) || (_mask != 0) )
            {
                // Collision filters were specified. Get the discrete dynamics world

                // btDiscreteDynamicsWorld* ddw = dynamic_cast< btDiscreteDynamicsWorld* >( _dw );
                if( ddw != nullptr )
                {
                    ddw->addRigidBody( rb, _group, _mask );
                    added = true;
                }
            }
        if( !added )
        {
            // This is both the main path for not using collision filters, and
            // also the fallback if the btDiscreteDynamicsWorld* dynamic cast fails.
            ddw->addRigidBody( rb );
        }

        if( _pt != nullptr )
            _pt->pause( false );

    } else vsg::Inherit<vsg::Trackball,LaunchHandler>::apply(buttonPress);
}
void LaunchHandler::apply(vsg::ButtonReleaseEvent& buttonRelease)
{
    vsg::Inherit<vsg::Trackball,LaunchHandler>::apply(buttonRelease);

}
void LaunchHandler::apply(vsg::MoveEvent& moveEvent)
{
    vsg::Inherit<vsg::Trackball,LaunchHandler>::apply(moveEvent);
    if (moveEvent.handled || !eventRelevant(moveEvent))
    {
        _hasKeyboardFocus = false;
        return;
    }
}
void LaunchHandler::apply(vsg::KeyPressEvent& buttonPress)
{
    std::cerr<<buttonPress.keyBase<<std::endl;
    if (buttonPress.keyBase == 65507)//ctrl == 65507
        _ctrlpressed=true;
    vsg::Inherit<vsg::Trackball,LaunchHandler>::apply(buttonPress);
}
void LaunchHandler::apply(vsg::KeyReleaseEvent& buttonRelease) {
    if (buttonRelease.keyBase == 65507)//ctrl == 65507
        _ctrlpressed=false;
    std::cerr<<buttonRelease.keyBase<<std::endl;
    vsg::Inherit<vsg::Trackball,LaunchHandler>::apply(buttonRelease);
}

/*
bool LaunchHandler::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&aa )
{

    if(!_world->getDynamicsWorld())
        return false;
    // We want a shift-leftmouse event. Return false if we don't have it.
    if(!_attachPoint.valid() || ( ea.getEventType() != osgGA::GUIEventAdapter::PUSH ) ||
        ( ea.getButton() != osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON ) ||
        ( ( ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_SHIFT ) == 0 ) )
        return( false );

    vsg::mat4 view = aa.asView()->getCamera()->getViewMatrix();
    vsg::vec3 look, at, up;
    view.getLookAt( look, at, up );

    vsg::mat4 proj = aa.asView()->getCamera()->getProjectionMatrix();
    double fovy, aspect, zNear, zFar;
    proj.getPerspective( fovy, aspect, zNear, zFar );

    view.invert( view );
    proj.invert( proj );
    vsg::Vec4 clip( ea.getXnormalized() * zFar, ea.getYnormalized() * zFar, zFar, zFar );
    vsg::Vec4 wc = clip * proj * view;

    const vsg::vec3 launchPos = look + ( up * ( _launchModel->getBound()._radius * 2. ) );

    vsg::mat4 parentTrans = vsg::mat4::translate( launchPos );
    vsg::vec3 launchDir = vsg::vec3( wc[0], wc[1], wc[2] ) - launchPos;
    launchDir.normalize();

    vsg::ref_ptr< vsgbCollision::AbsoluteTransform > amt = new vsgbCollision::AbsoluteTransform;
    amt->setDataVariance( vsg::Object::DYNAMIC );
    amt->addChild( _launchModel.get() );

    _attachPoint->addChild( amt.get() );
    _nodeList.push_back( amt.get() );

    vsg::ref_ptr< vsgbDynamics::CreationRecord > cr = new vsgbDynamics::CreationRecord;
    cr->_sceneGraph = amt.get();
    cr->_mass = getLaunchedMass();
    cr->_parentTransform = parentTrans;
    btRigidBody* rb = vsgbDynamics::createRigidBody( cr.get(), _launchCollisionShape );
    rb->setLinearVelocity( vsgbCollision::asBtVector3( launchDir * _initialVelocity ) );
    rb->setAngularVelocity( btVector3( .2, .3, 1.5 ) );

    vsgbDynamics::MotionState* motion = static_cast< vsgbDynamics::MotionState* >( rb->getMotionState() );
    if( _tb != nullptr )
        motion->registerTripleBuffer( _tb );
    if( _msl != nullptr )
        _msl->insert( motion );

    if( _pt != nullptr )
        _pt->pause( true );

    bool added( false );
    amt->setUserData( new vsgbCollision::RefRigidBody( rb ) );
    btDiscreteDynamicsWorld* ddw =0;
    if(_world.valid() &&   ( ddw=_world->getDynamicsWorld()) )
    if( (_group != 0) || (_mask != 0) )
    {
        // Collision filters were specified. Get the discrete dynamics world

       // btDiscreteDynamicsWorld* ddw = dynamic_cast< btDiscreteDynamicsWorld* >( _dw );
        if( ddw != nullptr )
        {
            ddw->addRigidBody( rb, _group, _mask );
            added = true;
        }
    }
    if( !added )
    {
        // This is both the main path for not using collision filters, and
        // also the fallback if the btDiscreteDynamicsWorld* dynamic cast fails.
        ddw->addRigidBody( rb );
    }

    if( _pt != nullptr )
        _pt->pause( false );

    return( true );
}
*/
void LaunchHandler::reset()
{
    if( _pt != nullptr )
        _pt->pause( true );

    NodeList::iterator it;    btDiscreteDynamicsWorld* ddw =0;
    if(_world.valid() &&   ( ddw=_world->getDynamicsWorld()) )
        for( it=_nodeList.begin(); it != _nodeList.end(); ++it )
        {
            vsg::ref_ptr< vsg::Node > node = *it;
            vsgbCollision::RefRigidBody* rrb = nullptr;// dynamic_cast< vsgbCollision::RefRigidBody* >( node->getUserData() );
            if( rrb == nullptr )
            {
                std::cerr << "LaunchHandler::reset: Node has no RefRigidBody in UserData." << std::endl;
                continue;
            }

            btRigidBody* rb = rrb->get();
            if( rb->getMotionState() )
            {
                vsgbDynamics::MotionState* motion = static_cast< vsgbDynamics::MotionState* >( rb->getMotionState() );
                if( _msl != nullptr )
                {
                    vsgbDynamics::MotionStateList::iterator it = _msl->find( motion );
                    if( it != _msl->end() )
                        _msl->erase( it );
                }
                delete motion;
            }
            ddw->removeRigidBody( rb );
            delete rb;
            //TODO _attachPoint->removeChild( node.get() );
            for(auto d=_attachPoint->children.begin();d!=_attachPoint->children.end();d++)
            {
                if((*d).get()==node.get()){
                    _attachPoint->children.erase(d); break;
                }
            }
        }

    if( _pt != nullptr )
        _pt->pause( false );

    _nodeList.clear();
}


// vsgbInteraction
}
