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

#include <iostream>
#include <vsgbInteraction/DragHandler.h>
#include <vsgbCollision/RefBulletObject.h>
#include <vsgbCollision/Utils.h>
#include <vsgbDynamics/MotionState.h>
#include <vsgbDynamics/PhysicsThread.h>
#include <vsgbDynamics/World.h>

#include <btBulletDynamicsCommon.h>


namespace vsgbInteraction
{


DragHandler::DragHandler(vsg::ref_ptr<vsgbDynamics::World> w, vsg::ref_ptr<vsg::Camera> camera, vsg::ref_ptr<vsg::EllipsoidModel> ellipsoidModel)
    : vsg::Inherit<vsg::Trackball,DragHandler>(camera, ellipsoidModel),
    _dw( w ),

    _constraint( nullptr ),
    _constrainedMotionState( nullptr ),
    _pt( nullptr )
{
}
DragHandler::~DragHandler()
{
}


void DragHandler::apply(vsg::ButtonPressEvent& buttonPress)
{

    lastPointerEvent = &buttonPress;
    if (buttonPress.handled || !eventRelevant(buttonPress))
    {
        _hasKeyboardFocus = false;
        return;
    }
    if(_ctrlpressed){

        const bool picked = pick(*lastPointerEvent);//_dw, 1,1 );

        if( picked )
            _constraint->getRigidBodyA().activate( true );

        return;


    }
    vsg::Inherit<vsg::Trackball,DragHandler>::apply(buttonPress);
}
void DragHandler::apply(vsg::ButtonReleaseEvent& buttonRelease)
{
    lastPointerEvent = &buttonRelease;
    if( _constraint == nullptr ) return;

    if( _pt != nullptr )
        _pt->pause( true );

    _dw->getDynamicsWorld()->removeConstraint( _constraint );

    if( _pt != nullptr )
        _pt->pause( false );

    delete _constraint;
    _constraint = nullptr;
    _constrainedMotionState = nullptr;

    vsg::Inherit<vsg::Trackball,DragHandler>::apply(buttonRelease);

}

void DragHandler::apply(vsg::MoveEvent& moveEvent)
{
    lastPointerEvent = &moveEvent;
    if (moveEvent.handled || !eventRelevant(moveEvent))
    {
        _hasKeyboardFocus = false;
        return;
    }

    if(_ctrlpressed && _constraint != nullptr)
    {
        auto lookat = _camera->viewMatrix.cast<vsg::LookAt>();
        auto proj = _camera->projectionMatrix.cast<vsg::Perspective>();

        vsg::vec2 cndc(ndc(moveEvent));
        //vsg::vec4 clip( proj->farDistance, proj->farDistance, proj->farDistance, proj->farDistance);
        // Intersect ray with plane.
        vsg::vec4 farPointCC = vsg::vec4( cndc.x,cndc.y, 1., 1. );
        farPointCC*= proj->farDistance;
        vsg::vec4 farPointWC =   vsg::mat4(lookat->inverse()*proj->inverse())*farPointCC;
        vsg::dvec3 planeNormal = vsg::dvec3( _dragPlane[ 0 ], _dragPlane[ 1 ], _dragPlane[ 2 ] );
      /*  vsg::dvec3 planeNormal = vsg::dvec3( _dragPlane[ 0 ], _dragPlane[ 1 ], _dragPlane[ 2 ] );
        const vsg::dvec3 vDir = vsg::vec3( farPointWC[ 0 ], farPointWC[ 1 ], farPointWC[ 2 ] ) - look;
        const double dotVd = vDir * planeNormal;
        if( dotVd == 0. )
        {
            std::cerr << "DragHandler: No plane intersection." << std::endl;
            return( false );
        }
        double length = -( planeNormal * look + _dragPlane[ 3 ] ) / dotVd;
        vsg::vec3 pointOnPlane = look + ( vDir * length );*/
        vsg::dvec3 vDir = vsg::dvec3( farPointWC[ 0 ], farPointWC[ 1 ], farPointWC[ 2 ] );
        vDir -= lookat->eye;
        const double dotVd = vsg::dot(vDir, planeNormal);
        if( dotVd == 0. )
        {
            vsg::warn( "DragHandler: No plane intersection.");
            return;
        }
        double length = -( vsg::dot(planeNormal, lookat->eye) + _dragPlane[ 3 ] ) / dotVd;
        vsg::vec3 pointOnPlane =  vsg::vec3( vDir );
        pointOnPlane *= length;
        pointOnPlane += vsg::vec3(lookat->eye);
        vsg::info( "  VSG point " , pointOnPlane );

        if( _pt != nullptr )
            _pt->pause( true );

        vsg::mat4 ow2bw;
        if( _constrainedMotionState != nullptr )
            ow2bw = _constrainedMotionState->computeVsgWorldToBulletWorld();
        vsg::vec3 bulletPoint = ow2bw * pointOnPlane  ;
        vsg::info( "    bullet point " , bulletPoint );

        _constraint->setPivotB( vsgbCollision::asBtVector3( bulletPoint ) );

        if( _pt != nullptr )
            _pt->pause( false );
    }else
    vsg::Inherit<vsg::Trackball,DragHandler>::apply(moveEvent);
}
void DragHandler::apply(vsg::KeyPressEvent& buttonPress)
{
    std::cerr<<buttonPress.keyBase<<std::endl;
    if (buttonPress.keyBase == 65507)//ctrl == 65507
        _ctrlpressed=true;
    vsg::Inherit<vsg::Trackball,DragHandler>::apply(buttonPress);
}
void DragHandler::apply(vsg::KeyReleaseEvent& buttonRelease) {
    if (buttonRelease.keyBase == 65507)//ctrl == 65507
        _ctrlpressed=false;
    std::cerr<<buttonRelease.keyBase<<std::endl;
    vsg::Inherit<vsg::Trackball,DragHandler>::apply(buttonRelease);
}

/*
bool DragHandler::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
{

    const bool ctrl( ( ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL ) != 0 );

    if( ea.getEventType() == osgGA::GUIEventAdapter::PUSH )
    {
        if( !ctrl ||
            ( ( ea.getButtonMask() & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON ) == 0 ) )
            return( false );

        const bool picked = pick(_dw, ea.getXnormalized(), ea.getYnormalized() );

        if( picked )
            _constraint->getRigidBodyA().activate( true );

        return( picked );
    }
    else if( ea.getEventType() == osgGA::GUIEventAdapter::DRAG )
    {
        if( ( !ctrl ) || ( _constraint == nullptr ) ||
            ( ( ea.getButtonMask() & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON ) == 0 ) )
            return( false );

        vsg::Vec4d farPointNDC = vsg::Vec4d( ea.getXnormalized(), ea.getYnormalized(), 1., 1. );
        vsg::mat4 p = cam->getProjectionMatrix();
        double zNear, zFar, fovy, aspect;
        p.getPerspective( fovy, aspect, zNear, zFar );
        vsg::Vec4d farPointCC = farPointNDC * zFar;
        p.invert( p );
        vsg::mat4 v = cam->getViewMatrix();
        v.invert( v );
        vsg::Vec4d farPointWC = farPointCC * p * v;

        vsg::dvec3 look, at, up;
        cam->getViewMatrixAsLookAt( look, at, up );


        // Intersect ray with plane.
        // TBD. Stolen from osgWorks' MxCore::intersectPlaneRay(), which really should be in some math library somewhere.
        vsg::dvec3 planeNormal = vsg::dvec3( _dragPlane[ 0 ], _dragPlane[ 1 ], _dragPlane[ 2 ] );
        const vsg::dvec3 vDir = vsg::vec3( farPointWC[ 0 ], farPointWC[ 1 ], farPointWC[ 2 ] ) - look;
        const double dotVd = vDir * planeNormal;
        if( dotVd == 0. )
        {
            std::cerr << "DragHandler: No plane intersection." << std::endl;
            return( false );
        }
        double length = -( planeNormal * look + _dragPlane[ 3 ] ) / dotVd;
        vsg::vec3 pointOnPlane = look + ( vDir * length );
        vsg::notify( vsg::DEBUG_FP ) << "  VSG point " << pointOnPlane << std::endl;

        if( _pt != nullptr )
            _pt->pause( true );

        vsg::mat4 ow2bw;
        if( _constrainedMotionState != nullptr )
            ow2bw = _constrainedMotionState->computeVsgWorldToBulletWorld();
        vsg::dvec3 bulletPoint = pointOnPlane * ow2bw;
        vsg::notify( vsg::DEBUG_FP ) << "    bullet point " << bulletPoint << std::endl;

        _constraint->setPivotB( vsgbCollision::asBtVector3( bulletPoint ) );

        if( _pt != nullptr )
            _pt->pause( false );

        return( true );
    }
    else if( ea.getEventType() == osgGA::GUIEventAdapter::RELEASE )
    {
        if( _constraint == nullptr )
            return( false );

        if( _pt != nullptr )
            _pt->pause( true );

        _dw->getDynamicsWorld()->removeConstraint( _constraint );

        if( _pt != nullptr )
            _pt->pause( false );

        delete _constraint;
        _constraint = nullptr;
        _constrainedMotionState = nullptr;
        return( true );
    }

    return( false );
}
*/
void DragHandler::setThreadedPhysicsSupport( vsgbDynamics::PhysicsThread* pt )
{
    _pt = pt;
}
bool DragHandler::pick(vsg::PointerEvent& pointerEvent)
{
    auto intersector = vsg::LineSegmentIntersector::create(*_camera, pointerEvent.x, pointerEvent.y);
    _dw->accept(*intersector);

    vsg::info( "intersection_LineSegmentIntersector(" , pointerEvent.x , ", " , pointerEvent.y,") " , intersector->intersections.size() , ")" );

    if (intersector->intersections.empty()) return false;

    // sort the intersections front to back
    std::sort(intersector->intersections.begin(), intersector->intersections.end(), [](auto& lhs, auto& rhs) { return lhs->ratio < rhs->ratio; });

    for (auto& intersection : intersector->intersections)
    {
        vsg::info( "intersection = world(" , intersection->worldIntersection , "), instanceIndex " , intersection->instanceIndex);

        /*if (ellipsoidModel)
        {
            std::cout.precision(10);
            auto location = ellipsoidModel->convertECEFToLatLongAltitude(intersection->worldIntersection);
            vsg::info( " lat = " ,location[0] , ", long = " , location[1] , ", height = " , location[2]);
        }*/

        if (lastIntersection)
        {
            vsg::info( ", distance from previous intersection = " , vsg::length(intersection->worldIntersection - lastIntersection->worldIntersection));
        }

        vsg::ref_ptr<const vsgbDynamics::RigidBody> rb;
        for (auto& node : intersection->nodePath)
        {
            rb=node->cast<const vsgbDynamics::RigidBody>();
            if( rb.valid())
            {
                vsg::dvec3 pickPointWC = intersection->worldIntersection;// getWorldIntersectPoint();
                // Save the MotionState for this rigid body. We'll use it during the DRAG events.
                _constrainedMotionState = dynamic_cast<const vsgbDynamics::MotionState* >( rb->getRigidBody()->getMotionState() );
                vsg::mat4 ow2col;
                if( _constrainedMotionState != nullptr )
                    ow2col = _constrainedMotionState->computeVsgWorldToCOLocal();
                vsg::vec3 pickPointBulletOCLocal = ow2col * vsg::vec3(pickPointWC);
                vsg::info( "pickPointWC: " , pickPointWC);
                vsg::info( "pickPointBulletOCLocal: " , pickPointBulletOCLocal );
                _constraint = new btPoint2PointConstraint( *(btRigidBody*)(rb->getRigidBody()),
                                                          vsgbCollision::asBtVector3( pickPointBulletOCLocal ) );
                if( _pt != nullptr )
                    _pt->pause( true );
                _dw->getDynamicsWorld()->addConstraint( _constraint );
                if( _pt != nullptr )
                    _pt->pause( false );
                // Also make a drag plane.
                vsg::dvec3 look, at, up;
                //_scene->getViewMatrixAsLookAt( look, at, up );
                vsg::dvec3 viewDir = _lookAt->center - _lookAt->eye;// at - look;
                viewDir = normalize(viewDir);
                _dragPlane = vsg::vec4(viewDir.x, viewDir.y, viewDir.z, -vsg::dot( pickPointWC, viewDir ) );
                vsg::info( "dragplane: " , _dragPlane );
                return true;
            }
        }
        if (false)
        {
            std::string name;
            for (auto& node : intersection->nodePath)
            {
                std::cout << ", " << node->className();
                if (node->getValue("name", name)) std::cout << ":name=" << name;
            }

            std::cout << ", Arrays[ ";
            for (auto& array : intersection->arrays)
            {
                std::cout << array << " ";
            }
            std::cout << "] [";
            for (auto& ir : intersection->indexRatios)
            {
                std::cout << "{" << ir.index << ", " << ir.ratio << "} ";
            }
            std::cout << "]";

            std::cout << std::endl;
        }
    }

    lastIntersection = intersector->intersections.front();
    return false;
}

// vsgbInteraction
}
