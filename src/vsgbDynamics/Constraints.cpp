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
#include <vsgbDynamics/Constraints.h>
#include <vsgbDynamics/MotionState.h>
#include <vsgbCollision/Utils.h>
#include <ostream>
#include <vsg/core/Object.h>
#include <vsg/maths/mat4.h>
#include <vsg/maths/transform.h>

#include <btBulletDynamicsCommon.h>

using namespace vsg;

namespace vsgbDynamics
{



Constraint::Constraint():
    _constraint( nullptr ),
    _dirty( true ),
    _rbA( nullptr ),
    _rbB( nullptr )
{
}

Constraint::Constraint( btRigidBody* rbA, btRigidBody* rbB )
  :
    _constraint( nullptr ),
    _dirty( true ),
    _rbA( rbA ),
    _rbB( rbB )
{
}

Constraint::Constraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
        btRigidBody* rbB, const vsg::mat4& rbBXform )
  : _constraint( nullptr ),
    _dirty( true ),
    _rbA( rbA ),
    _rbB( rbB ),
    _rbAXform( rbAXform ),
    _rbBXform( rbBXform )
{
}

Constraint::Constraint( const Constraint& rhs, const vsg::CopyOp& copyop )
    : vsg::Inherit<vsg::Object, Constraint>( rhs, copyop ),
    _constraint( rhs._constraint ),
    _dirty( rhs._dirty ),
    _rbA( rhs._rbA ),
    _rbB( rhs._rbB ),
    _rbAXform( rhs._rbAXform ),
    _rbBXform( rhs._rbBXform )
{
}

Constraint::~Constraint()
{
    // Deleting the constraint is up to the calling code. Something like this:
    //delete vsgbDynamics::Constraint::getConstraint();
}

btTypedConstraint* Constraint::getConstraint() const
{
    if( getDirty() || ( _constraint == nullptr ) )
    {
        Constraint* nonConst = const_cast< Constraint* >( this );
        nonConst->createConstraint();
    }

    return( _constraint );
}

void Constraint::setRigidBodies( btRigidBody* rbA, btRigidBody* rbB )
{
    _rbA = rbA;
    _rbB = rbB;
    setDirty();
}
void Constraint::setAXform( const vsg::mat4& rbAXform )
{
    _rbAXform = rbAXform;
    setDirty();
}
void Constraint::setBXform( const vsg::mat4& rbBXform )
{
    _rbBXform = rbBXform;
    setDirty();
}

bool Constraint::operator==( const Constraint& rhs ) const
{
    return( !( operator!=( rhs ) ) );
}
bool Constraint::operator!=( const Constraint& rhs ) const
{
    return(
        ( _rbAXform != rhs._rbAXform ) ||
        ( _rbBXform != rhs._rbBXform )
    );
}

vsg::mat4 Constraint::orthonormalize( const vsg::mat4& in )
{
    vsg::dvec3 a( in( 0, 0 ), in( 0, 1 ), in( 0, 2 ) );
    vsg::dvec3 b( in( 1, 0 ), in( 1, 1 ), in( 1, 2 ) );
    vsg::dvec3 c(cross( a , b ));
    c = normalize(c);
    b = cross(c , a);
    b = normalize(b);
    a = cross(b , c);
    a = normalize(a);

    vsg::mat4 m( a[0], a[1], a[2], in(0,3),
        b[0], b[1], b[2], in(1,3),
        c[0], c[1], c[2], in(2,3),
        in(3,0), in(3,1), in(3,2), in(3,3) );
    return( m );
}




SliderConstraint::SliderConstraint()
    : vsg::Inherit<Constraint,SliderConstraint>(),
    _axis( 1., 0., 0. ),
    _slideLimit( -1., 1. )
{
}
SliderConstraint::SliderConstraint( btRigidBody* rbA, btRigidBody* rbB )
  : vsg::Inherit<Constraint,SliderConstraint>( rbA, rbB ),
    _axis( 1., 0., 0. ),
    _slideLimit( -1., 1. )
{
    setDirty();
}
SliderConstraint::SliderConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
        const vsg::vec3& axis, const vsg::vec2& slideLimit )
  : vsg::Inherit<Constraint,SliderConstraint>( rbA, rbAXform ),
    _axis( axis ),
    _slideLimit( slideLimit )
{
    setDirty();
}
SliderConstraint::SliderConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
        btRigidBody* rbB, const vsg::mat4& rbBXform,
        const vsg::vec3& axis, const vsg::vec2& slideLimit )
  : vsg::Inherit<Constraint,SliderConstraint>( rbA, rbAXform, rbB, rbBXform ),
    _axis( axis ),
    _slideLimit( slideLimit )
{
    setDirty();
}
SliderConstraint::SliderConstraint( const SliderConstraint& rhs, const vsg::CopyOp& copyop )
  : vsg::Inherit<Constraint,SliderConstraint>( rhs, copyop ),
    _axis( rhs._axis),
    _slideLimit( rhs._slideLimit )
{
}
SliderConstraint::~SliderConstraint()
{
    // Deleting the constraint is up to the calling code.
}

btSliderConstraint* SliderConstraint::getAsBtSlider() const
{
    return( static_cast< btSliderConstraint* >( getConstraint() ) );
}

void SliderConstraint::setAxis( const vsg::vec3& axis )
{
    _axis = axis;
    setDirty();
}
void SliderConstraint::setAxis( const double x, const double y, const double z )
{
    setAxis( vsg::vec3( x, y, z ) );
}
void SliderConstraint::setLimit( const vsg::vec2& limit )
{
    _slideLimit = limit;

    if( !getDirty() && ( _constraint != nullptr ) )
    {
        // Dynamically modify the existing constraint.
        btSliderConstraint* cons = getAsBtSlider();
        cons->setLowerLinLimit( _slideLimit[ 0 ] );
        cons->setUpperLinLimit( _slideLimit[ 1 ] );
    }
    else
        setDirty();
}
void SliderConstraint::setLimit( const double lo, const double hi )
{
    setLimit( vsg::vec2( lo, hi ) );
}

bool SliderConstraint::operator==( const SliderConstraint& rhs ) const
{
    return( !( operator!=( rhs ) ) );
}
bool SliderConstraint::operator!=( const SliderConstraint& rhs ) const
{
    return(
        ( _axis != rhs._axis ) ||
        ( _slideLimit != rhs._slideLimit ) ||
        ( Constraint::operator!=( static_cast< const Constraint& >( rhs ) ) )
    );
}

vsg::mat4 on( const vsg::mat4& in )
{
    vsg::dvec3 a( in( 0, 0 ), in( 0, 1 ), in( 0, 2 ) );
    vsg::dvec3 b( in( 1, 0 ), in( 1, 1 ), in( 1, 2 ) );
    vsg::dvec3 c( cross(a, b) );
    c = normalize(c);
    b = cross(c , a);
    b = normalize(b);
    a = cross(b , c);
    a = normalize(a);

    vsg::mat4 m( a[0], a[1], a[2], in(0,3),
        b[0], b[1], b[2], in(1,3),
        c[0], c[1], c[2], in(2,3),
        in(3,0), in(3,1), in(3,2), in(3,3) );
    return( m );
}

void SliderConstraint::createConstraint()
{
    if( _rbA == nullptr )
    {
        std::cerr << "createConstraint: _rbA == nullptr." << std::endl;
        return;
    }

    if( _constraint != nullptr )
    {
        delete _constraint;
        _constraint = nullptr;
    }


    // Orientation matrix for the slider x-axis.
    vsg::vec3 localAxis( _axis );
    localAxis = normalize(localAxis);
    const vsg::mat4 orientation = vsg::rotate( vsgbCollision::makeRotate( vsg::vec3( 1., 0., 0. ), localAxis ));


    // Create a matrix that returns A to the origin.
    //
    //   1. Inverse A center of mass offset.
    vsgbDynamics::MotionState* motion = dynamic_cast< vsgbDynamics::MotionState* >( _rbA->getMotionState() );
    if( motion == nullptr )
    {
        std::cerr << "createConstraint: Invalid MotionState." << std::endl;
        return;
    }
    vsg::vec3 invCom = -( motion->getCenterOfMass() );
    vsg::vec3 scale = motion->getScale();
    vsg::vec3 scaledInvCom = vsg::vec3( invCom[0]*scale[0], invCom[1]*scale[1], invCom[2]*scale[2] );
    const vsg::mat4 invACOM( vsg::translate( scaledInvCom ) );
    //
    //   2. Transform A back to the origin.
    const vsg::mat4 invAXform( vsg::inverse( orthonormalize( _rbAXform ) ) );
    //
    //   3. The final rbA frame matrix.
    btTransform rbAFrame = vsgbCollision::asBtTransform(
        orientation * invAXform * invACOM );


    btTransform rbBFrame;
    if( _rbB != nullptr )
    {
        // Create a matrix that orients the spring axis/point in B's coordinate space.
        //
        //   1. Inverse B center of mass offset.
        motion = dynamic_cast< vsgbDynamics::MotionState* >( _rbB->getMotionState() );
        if( motion == nullptr )
        {
            std::cout << "InternalCreateSpring: Invalid MotionState." << std::endl;
            return;
        }
        invCom = -( motion->getCenterOfMass() );
        scale = motion->getScale();
        scaledInvCom = vsg::vec3( invCom[0]*scale[0], invCom[1]*scale[1], invCom[2]*scale[2] );
        const vsg::mat4 invBCOM( vsg::translate( scaledInvCom ) );
        //
        //   2. Transform B back to the origin.
        const vsg::mat4 invBXform( vsg::inverse( orthonormalize( _rbBXform ) ) );
        //
        //   3. The final rbB frame matrix.
        rbBFrame = vsgbCollision::asBtTransform( 
            orientation * invBXform * invBCOM );
    }


    btSliderConstraint* cons;
    if( _rbB != nullptr )
        cons = new btSliderConstraint( *_rbA, *_rbB, rbAFrame, rbBFrame, false );
    else
        cons = new btSliderConstraint( *_rbA, rbAFrame, true );

    cons->setLowerLinLimit( btScalar( _slideLimit[ 0 ] ) );
    cons->setUpperLinLimit( btScalar( _slideLimit[ 1 ] ) );

    _constraint = cons;

    setDirty( false );
}




TwistSliderConstraint::TwistSliderConstraint()
  : vsg::Inherit<Constraint, TwistSliderConstraint>(),
    _axis( 1., 0., 0. ),
    _point( 0., 0., 0. ),
    _slideLimit( -1., 1. ),
    _twistLimit( -vsg::PIf*.5f, vsg::PIf*.5f )
{
}
TwistSliderConstraint::TwistSliderConstraint( btRigidBody* rbA, btRigidBody* rbB )
  : vsg::Inherit<Constraint, TwistSliderConstraint>( rbA, rbB ),
    _axis( 1., 0., 0. ),
    _point( 0., 0., 0. ),
    _slideLimit( -1., 1. ),
    _twistLimit(-vsg::PIf*.5f, vsg::PIf*.5f )
{
}
TwistSliderConstraint::TwistSliderConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
            btRigidBody* rbB, const vsg::mat4& rbBXform,
            const vsg::vec3& axis, const vsg::vec3& wcPoint,
            const vsg::vec2& slideLimit, const vsg::vec2& twistLimit )
  : vsg::Inherit<Constraint, TwistSliderConstraint>( rbA, rbAXform, rbB, rbBXform ),
    _axis( axis ),
    _point( wcPoint ),
    _slideLimit( slideLimit ),
    _twistLimit( twistLimit )
{
}
TwistSliderConstraint::TwistSliderConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
            const vsg::vec3& axis, const vsg::vec3& wcPoint,
            const vsg::vec2& slideLimit, const vsg::vec2& twistLimit )
  : vsg::Inherit<Constraint, TwistSliderConstraint>( rbA, rbAXform ),
    _axis( axis ),
    _point( wcPoint ),
    _slideLimit( slideLimit ),
    _twistLimit( twistLimit )
{
}
TwistSliderConstraint::TwistSliderConstraint( const TwistSliderConstraint& rhs, const vsg::CopyOp& copyop )
  : vsg::Inherit<Constraint, TwistSliderConstraint>( rhs, copyop ),
    _axis( rhs._axis ),
    _point( rhs._point ),
    _slideLimit( rhs._slideLimit ),
    _twistLimit( rhs._twistLimit )
{
}
TwistSliderConstraint::~TwistSliderConstraint()
{
}

btSliderConstraint* TwistSliderConstraint::getAsBtSlider() const
{
    return( static_cast< btSliderConstraint* >( getConstraint() ) );
}

void TwistSliderConstraint::setAxis( const vsg::vec3& axis )
{
    _axis = axis;
    setDirty();
}
void TwistSliderConstraint::setAxis( const double x, const double y, const double z )
{
    setAxis( vsg::vec3( x, y, z ) );
}
void TwistSliderConstraint::setPoint( const vsg::vec3& wcPoint )
{
    _point = wcPoint;
    setDirty();
}
void TwistSliderConstraint::setPoint( const double x, const double y, const double z )
{
    setPoint( vsg::vec3( x, y, z ) );
}
void TwistSliderConstraint::setSlideLimit( const vsg::vec2& limit )
{
    _slideLimit = limit;

    if( !getDirty() && ( _constraint != nullptr ) )
    {
        // Dynamically modify the existing constraint.
        btSliderConstraint* cons = getAsBtSlider();
        cons->setLowerLinLimit( _slideLimit[ 0 ] );
        cons->setUpperLinLimit( _slideLimit[ 1 ] );
    }
    else
        setDirty();
}
void TwistSliderConstraint::setSlideLimit( const double lo, const double hi )
{
    setSlideLimit( vsg::vec2( lo, hi ) );
}
void TwistSliderConstraint::setTwistLimit( const vsg::vec2& limitRadians )
{
    _twistLimit = limitRadians;

    if( !getDirty() && ( _constraint != nullptr ) )
    {
        // Dynamically modify the existing constraint.
        btSliderConstraint* cons = getAsBtSlider();
        cons->setLowerAngLimit( _twistLimit[ 0 ] );
        cons->setUpperAngLimit( _twistLimit[ 1 ] );
    }
    else
        setDirty();
}
void TwistSliderConstraint::setTwistLimit( const double lo, const double hi )
{
    setTwistLimit( vsg::vec2( lo, hi ) );
}

bool TwistSliderConstraint::operator==( const TwistSliderConstraint& rhs ) const
{
    return( !( operator!=( rhs ) ) );
}
bool TwistSliderConstraint::operator!=( const TwistSliderConstraint& rhs ) const
{
    return(
        ( _axis != rhs._axis ) ||
        ( _point != rhs._point ) ||
        ( _slideLimit != rhs._slideLimit ) ||
        ( _twistLimit != rhs._twistLimit ) ||
        ( Constraint::operator!=( static_cast< const Constraint& >( rhs ) ) )
    );
}


void TwistSliderConstraint::createConstraint()
{
    if( _rbA == nullptr )
    {
        std::cerr << "createConstraint: _rbA == nullptr." << std::endl;
        return;
    }

    if( _constraint != nullptr )
    {
        delete _constraint;
        _constraint = nullptr;
    }


    // Orientation matrix for the slider x-axis / point.
    vsg::vec3 localAxis( _axis );
    localAxis = normalize(localAxis);
    const vsg::mat4 orientation =
        vsg::rotate( vsgbCollision::makeRotate( vsg::vec3( 1., 0., 0. ), localAxis ) )*
        vsg::translate( _point );


    // Create a matrix that returns A to the origin.
    //
    //   1. Inverse A center of mass offset.
    vsgbDynamics::MotionState* motion = dynamic_cast< vsgbDynamics::MotionState* >( _rbA->getMotionState() );
    if( motion == nullptr )
    {
        std::cerr << "createConstraint: Invalid MotionState." << std::endl;
        return;
    }
    vsg::vec3 invCom = -( motion->getCenterOfMass() );
    vsg::vec3 scale = motion->getScale();
    vsg::vec3 scaledInvCom = vsg::vec3( invCom[0]*scale[0], invCom[1]*scale[1], invCom[2]*scale[2] );
    const vsg::mat4 invACOM( vsg::translate( scaledInvCom ) );
    //
    //   2. Transform A back to the origin.
    const vsg::mat4 invAXform( vsg::inverse( orthonormalize( _rbAXform ) ) );
    //
    //   3. The final rbA frame matrix.
    btTransform rbAFrame = vsgbCollision::asBtTransform( 
        orientation * invAXform * invACOM );


    btTransform rbBFrame;
    if( _rbB != nullptr )
    {
        // Create a matrix that orients the spring axis/point in B's coordinate space.
        //
        //   1. Inverse B center of mass offset.
        motion = dynamic_cast< vsgbDynamics::MotionState* >( _rbB->getMotionState() );
        if( motion == nullptr )
        {
            std::cerr<< "InternalCreateSpring: Invalid MotionState." << std::endl;
            return;
        }
        invCom = -( motion->getCenterOfMass() );
        scale = motion->getScale();
        scaledInvCom = vsg::vec3( invCom[0]*scale[0], invCom[1]*scale[1], invCom[2]*scale[2] );
        const vsg::mat4 invBCOM( vsg::translate( scaledInvCom ) );
        //
        //   2. Transform B back to the origin.
        const vsg::mat4 invBXform( vsg::inverse( orthonormalize( _rbBXform ) ) );
        //
        //   3. The final rbB frame matrix.
        rbBFrame = vsgbCollision::asBtTransform( 
            orientation * invBXform * invBCOM );
    }


    btSliderConstraint* cons;
    if( _rbB != nullptr )
        cons = new btSliderConstraint( *_rbA, *_rbB, rbAFrame, rbBFrame, false );
    else
        cons = new btSliderConstraint( *_rbA, rbAFrame, true );

    cons->setLowerLinLimit( btScalar( _slideLimit[ 0 ] ) );
    cons->setUpperLinLimit( btScalar( _slideLimit[ 1 ] ) );
    cons->setLowerAngLimit( _twistLimit[ 0 ] );
    cons->setUpperAngLimit( _twistLimit[ 1 ] );

    _constraint = cons;

    setDirty( false );
}




InternalSpringData::InternalSpringData()
  : _linearLowerLimits( 0., 0., 0. ),
    _linearUpperLimits( 0., 0., 0. ),
    _angularLowerLimits( 0., 0., 0. ),
    _angularUpperLimits( 0., 0., 0. )
{
    for( int idx=0; idx<6; idx++ )
    {
        _enable[ idx ] = false;
        _stiffness[ idx ] = _damping[ idx ] = 0.;
    }
}
InternalSpringData::InternalSpringData( const InternalSpringData& rhs, const vsg::CopyOp& copyop )
  : _linearLowerLimits( rhs._linearLowerLimits ),
    _linearUpperLimits( rhs._linearUpperLimits ),
    _angularLowerLimits( rhs._angularLowerLimits ),
    _angularUpperLimits( rhs._angularUpperLimits )
{
    memcpy( _enable, rhs._enable, sizeof( _enable ) );
    memcpy( _stiffness, rhs._stiffness, sizeof( _stiffness ) );
    memcpy( _damping, rhs._damping, sizeof( _damping ) );
}

void InternalSpringData::apply( btGeneric6DofSpringConstraint* cons ) const
{
    cons->setLinearLowerLimit( vsgbCollision::asBtVector3( _linearLowerLimits ) );
    cons->setLinearUpperLimit( vsgbCollision::asBtVector3( _linearUpperLimits ) );
    cons->setAngularLowerLimit( vsgbCollision::asBtVector3( _angularLowerLimits ) );
    cons->setAngularUpperLimit( vsgbCollision::asBtVector3( _angularUpperLimits ) );

    int idx;
    for( idx=0; idx<6; idx++ )
    {
        cons->enableSpring( idx, _enable[ idx ] );
        cons->setStiffness( idx, _stiffness[ idx ] );
        cons->setDamping( idx, _damping[ idx ] );
    }
}

bool InternalSpringData::operator==( const InternalSpringData& rhs ) const
{
    return( !( operator!=( rhs ) ) );
}
bool InternalSpringData::operator!=( const InternalSpringData& rhs ) const
{
    if( ( _linearLowerLimits != rhs._linearLowerLimits ) ||
        ( _linearUpperLimits != rhs._linearUpperLimits ) ||
        ( _angularLowerLimits != rhs._angularLowerLimits ) ||
        ( _angularUpperLimits != rhs._angularUpperLimits ) )
        return( true );

    int idx;
    for( idx=0; idx<6; idx++ )
    {
        if( ( _enable[ idx ] != rhs._enable[ idx ] ) ||
            ( _stiffness[ idx ] != rhs._stiffness[ idx ] ) ||
            ( _damping[ idx ] != rhs._damping[ idx ] ) )
            return( true );
    }

    return( false );
}




LinearSpringConstraint::LinearSpringConstraint()
  : vsg::Inherit<Constraint, LinearSpringConstraint>(),
    _data( new InternalSpringData )
{
    _data->_enable[ 0 ] = true;
    _data->_linearLowerLimits[ 0 ] = -1.;
    _data->_linearUpperLimits[ 0 ] = 1.;
    _data->_stiffness[ 0 ] = 10.f;
    _data->_damping[ 0 ] = .1f;
}
LinearSpringConstraint::LinearSpringConstraint( btRigidBody* rbA, btRigidBody* rbB )
  : vsg::Inherit<Constraint, LinearSpringConstraint>( rbA, rbB ),
    _axis( 1., 0., 0. ),
    _data( new InternalSpringData )
{
    _data->_enable[ 0 ] = true;
    _data->_linearLowerLimits[ 0 ] = -1.;
    _data->_linearUpperLimits[ 0 ] = 1.;
    _data->_stiffness[ 0 ] = 10.f;
    _data->_damping[ 0 ] = .1f;
}
LinearSpringConstraint::LinearSpringConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
        btRigidBody* rbB, const vsg::mat4& rbBXform, const vsg::vec3& axis )
  : vsg::Inherit<Constraint, LinearSpringConstraint>( rbA, rbAXform, rbB, rbBXform ),
    _axis( axis ),
    _data( new InternalSpringData )
{
    _data->_enable[ 0 ] = true;
    _data->_linearLowerLimits[ 0 ] = -1.;
    _data->_linearUpperLimits[ 0 ] = 1.;
    _data->_stiffness[ 0 ] = 10.f;
    _data->_damping[ 0 ] = .1f;
}
LinearSpringConstraint::LinearSpringConstraint( const LinearSpringConstraint& rhs, const vsg::CopyOp& copyop )
  : vsg::Inherit<Constraint, LinearSpringConstraint>( rhs, copyop ),
    _axis( rhs._axis ),
    _data( rhs._data )
{
}
LinearSpringConstraint::~LinearSpringConstraint()
{
}

btGeneric6DofSpringConstraint* LinearSpringConstraint::getAsBtGeneric6DofSpring() const
{
    return( static_cast< btGeneric6DofSpringConstraint* >( getConstraint() ) );
}

void LinearSpringConstraint::setSpringData( InternalSpringData* data )
{
    _data = data;

    if( !getDirty() && ( _constraint != nullptr ) )
    {
        // Dynamically modify the existing constraint.
        btGeneric6DofSpringConstraint* cons = getAsBtGeneric6DofSpring();
        _data->apply( cons );
    }
    else
        setDirty();
}
void LinearSpringConstraint::setAxis( const vsg::vec3& axis )
{
    _axis = axis;
    setDirty();
}
void LinearSpringConstraint::setAxis( const double x, const double y, const double z )
{
    setAxis( vsg::vec3( x, y, z ) );
}
void LinearSpringConstraint::setLimit( const vsg::vec2& limit )
{
    _data->_linearLowerLimits[ 0 ] = limit[ 0 ];
    _data->_linearUpperLimits[ 0 ] = limit[ 1 ];
    setSpringData( _data.get() );
}
void LinearSpringConstraint::setLimit( const double lo, const double hi )
{
    LinearSpringConstraint::setLimit( vsg::vec2( lo, hi ) );
}
void LinearSpringConstraint::setStiffness( float stiffness )
{
    _data->_stiffness[ 0 ] = btScalar( stiffness );
    setSpringData( _data.get() );
}
void LinearSpringConstraint::setDamping( float damping )
{
    _data->_damping[ 0 ] = btScalar( damping );
    setSpringData( _data.get() );
}

bool LinearSpringConstraint::operator==( const LinearSpringConstraint& rhs ) const
{
    return( !( operator!=( rhs ) ) );
}
bool LinearSpringConstraint::operator!=( const LinearSpringConstraint& rhs ) const
{
    return(
        ( _axis != rhs._axis ) ||
        ( *_data != *( rhs._data ) ) ||
        ( Constraint::operator!=( static_cast< const Constraint& >( rhs ) ) )
    );
}

void LinearSpringConstraint::createConstraint()
{
    if( _constraint != nullptr )
    {
        delete _constraint;
        _constraint = nullptr;
    }

    // The internal construction/config code for Bullet spring constraints
    // is identical for our three rpring types, except that LinearSpringConstraint
    // does not have a pivot point.
    _constraint = internalCreateSpringConstraint( this, _data.get(), _axis );

    setDirty( _constraint == nullptr );
}

btGeneric6DofSpringConstraint* LinearSpringConstraint::internalCreateSpringConstraint(
    Constraint* cons, const InternalSpringData* isd,
    const vsg::vec3& axis, const vsg::vec3& point )
{
    btRigidBody* rbA, * rbB;
    cons->getRigidBodies( rbA, rbB );

    if( ( rbA == nullptr ) || ( rbB == nullptr ) )
    {
        std::cerr << "InternalSpringCreate: _rbA == nullptr or _rbB == nullptr." << std::endl;
        return( nullptr );
    }

    const vsg::mat4 aXform = cons->getAXform();
    const vsg::mat4 bXform = cons->getBXform();


    // Orientation matrix for the spring x-axis / point.
    vsg::vec3 localAxis( axis );
    localAxis = normalize(localAxis);
    const vsg::mat4 orientation =
        vsg::rotate( vsgbCollision::makeRotate(vsg::vec3( 1., 0., 0. ), localAxis )) *
        vsg::translate( point );


    // Create a matrix that puts A in B's coordinate space, accounting
    // for orientation of the constraint axes.
    //
    //   1. Inverse A center of mass offset.
    vsgbDynamics::MotionState* motion = dynamic_cast< vsgbDynamics::MotionState* >( rbA->getMotionState() );
    if( motion == nullptr )
    {
        std::cerr<< "InternalCreateSpring: Invalid MotionState." << std::endl;
        return( nullptr );
    }
    vsg::vec3 invCom = -( motion->getCenterOfMass() );
    vsg::vec3 scale = motion->getScale();
    vsg::vec3 scaledInvCom = vsg::vec3( invCom[0]*scale[0], invCom[1]*scale[1], invCom[2]*scale[2] );
    const vsg::mat4 invACOM( vsg::translate( scaledInvCom ) );
    //
    //   2. Transform A back to the origin.
    const vsg::mat4 invAXform( vsg::inverse( aXform ) );
    //
    //   3. The final rbA frame matrix.
    btTransform rbAFrame = vsgbCollision::asBtTransform( 
        orientation * invAXform * invACOM );


    // Create a matrix that orients the spring axis/point in B's coordinate space.
    //
    //   1. Inverse B center of mass offset.
    motion = dynamic_cast< vsgbDynamics::MotionState* >( rbB->getMotionState() );
    if( motion == nullptr )
    {
        std::cerr << "InternalCreateSpring: Invalid MotionState." << std::endl;
        return( nullptr );
    }
    invCom = -( motion->getCenterOfMass() );
    scale = motion->getScale();
    scaledInvCom = vsg::vec3( invCom[0]*scale[0], invCom[1]*scale[1], invCom[2]*scale[2] );
    const vsg::mat4 invBCOM( vsg::translate( scaledInvCom ) );
    //
    //   2. Transform B back to the origin.
    const vsg::mat4 invBXform( vsg::inverse( bXform ) );
    //
    //   3. The final rbB frame matrix.
    btTransform rbBFrame = vsgbCollision::asBtTransform( 
        orientation * invBXform * invBCOM );


    btGeneric6DofSpringConstraint* springCons = new btGeneric6DofSpringConstraint(
        *rbA, *rbB, rbAFrame, rbBFrame, false );
    isd->apply( springCons );
    springCons->setEquilibriumPoint();

    return( springCons );
}




AngleSpringConstraint::AngleSpringConstraint()
    : vsg::Inherit<Constraint, AngleSpringConstraint>(),
    _axis( 1., 0., 0. ),
    _pivotPoint( 0., 0., 0. ),
    _data( new InternalSpringData )
{
    _data->_enable[ 3 ] = true;
    _data->_angularLowerLimits[ 0 ] = -vsg::PIf*.5f;
    _data->_angularUpperLimits[ 0 ] = vsg::PIf*.5f;
    _data->_stiffness[ 3 ] = 10.f;
    _data->_damping[ 3 ] = .1f;
}
AngleSpringConstraint::AngleSpringConstraint( btRigidBody* rbA, btRigidBody* rbB )
  : vsg::Inherit<Constraint, AngleSpringConstraint>( rbA, rbB ),
    _axis( 1., 0., 0. ),
    _pivotPoint( 0., 0., 0. ),
    _data( new InternalSpringData )
{
    _data->_enable[ 3 ] = true;
    _data->_angularLowerLimits[ 0 ] = -vsg::PIf*.5f;
    _data->_angularUpperLimits[ 0 ] = vsg::PIf*.5f;
    _data->_stiffness[ 3 ] = 10.f;
    _data->_damping[ 3 ] = .1f;
}
AngleSpringConstraint::AngleSpringConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
        btRigidBody* rbB, const vsg::mat4& rbBXform,
        const vsg::vec3& axis, const vsg::vec3& point )
  : vsg::Inherit<Constraint, AngleSpringConstraint>( rbA, rbAXform, rbB, rbBXform ),
    _axis( axis ),
    _pivotPoint( point ),
    _data( new InternalSpringData )
{
    _data->_enable[ 3 ] = true;
    _data->_angularLowerLimits[ 0 ] = -vsg::PIf*.5f;
    _data->_angularUpperLimits[ 0 ] = vsg::PIf*.5f;
    _data->_stiffness[ 3 ] = 10.f;
    _data->_damping[ 3 ] = .1f;
}
AngleSpringConstraint::AngleSpringConstraint( const AngleSpringConstraint& rhs, const vsg::CopyOp& copyop )
  : vsg::Inherit<Constraint, AngleSpringConstraint>( rhs, copyop ),
    _axis( rhs._axis ),
    _pivotPoint( rhs._pivotPoint ),
    _data( rhs._data )
{
}
AngleSpringConstraint::~AngleSpringConstraint()
{
}

btGeneric6DofSpringConstraint* AngleSpringConstraint::getAsBtGeneric6DofSpring() const
{
    return( static_cast< btGeneric6DofSpringConstraint* >( getConstraint() ) );
}

void AngleSpringConstraint::setSpringData( InternalSpringData* data )
{
    _data = data;

    if( !getDirty() && ( _constraint != nullptr ) )
    {
        // Dynamically modify the existing constraint.
        btGeneric6DofSpringConstraint* cons = getAsBtGeneric6DofSpring();
        _data->apply( cons );
    }
    else
        setDirty();
}
void AngleSpringConstraint::setAxis( const vsg::vec3& axis )
{
    _axis = axis;
    setDirty();
}
void AngleSpringConstraint::setAxis( const double x, const double y, const double z )
{
    setAxis( vsg::vec3( x, y, z ) );
}
void AngleSpringConstraint::setPivotPoint( const vsg::vec3& wcPoint )
{
    _pivotPoint = wcPoint;
    setDirty();
}
void AngleSpringConstraint::setPivotPoint( const double x, const double y, const double z )
{
    setPivotPoint( vsg::vec3( x, y, z ) );
}
void AngleSpringConstraint::setLimit( const vsg::vec2& limit )
{
    _data->_angularLowerLimits[ 0 ] = limit[ 0 ];
    _data->_angularUpperLimits[ 0 ] = limit[ 1 ];
    setSpringData( _data.get() );
}
void AngleSpringConstraint::setLimit( const double lo, const double hi )
{
    setLimit( vsg::vec2( lo, hi ) );
}
void AngleSpringConstraint::setStiffness( float stiffness )
{
    _data->_stiffness[ 3 ] = btScalar( stiffness );
    setSpringData( _data.get() );
}
void AngleSpringConstraint::setDamping( float damping )
{
    _data->_damping[ 3 ] = btScalar( damping );
    setSpringData( _data.get() );
}

bool AngleSpringConstraint::operator==( const AngleSpringConstraint& rhs ) const
{
    return( !( operator!=( rhs ) ) );
}
bool AngleSpringConstraint::operator!=( const AngleSpringConstraint& rhs ) const
{
    return(
        ( _axis != rhs._axis ) ||
        ( *_data != *( rhs._data ) ) ||
        ( Constraint::operator!=( static_cast< const Constraint& >( rhs ) ) )
    );
}

void AngleSpringConstraint::createConstraint()
{
    if( _constraint != nullptr )
    {
        delete _constraint;
        _constraint = nullptr;
    }

    _constraint = LinearSpringConstraint::internalCreateSpringConstraint(
        this, _data.get(), _axis, _pivotPoint );

    setDirty( _constraint == nullptr );
}




LinearAngleSpringConstraint::LinearAngleSpringConstraint()
  : vsg::Inherit<Constraint, LinearAngleSpringConstraint>(),
    _axis( 1., 0., 0. ),
    _pivotPoint( 0., 0., 0. ),
    _data( new InternalSpringData )
{
    _data->_enable[ 0 ] = _data->_enable[ 3 ] = true;
    _data->_linearLowerLimits[ 0 ] = -1.;
    _data->_linearUpperLimits[ 0 ] = 1.;
    _data->_angularLowerLimits[ 0 ] = -vsg::PIf*.5f;
    _data->_angularUpperLimits[ 0 ] = vsg::PIf*.5f;
    _data->_stiffness[ 0 ] = _data->_stiffness[ 3 ] = 10.f;
    _data->_damping[ 0 ] = _data->_damping[ 3 ] = .1f;
}
LinearAngleSpringConstraint::LinearAngleSpringConstraint( btRigidBody* rbA, btRigidBody* rbB )
  : vsg::Inherit<Constraint, LinearAngleSpringConstraint>( rbA, rbB ),
    _axis( 1., 0., 0. ),
    _pivotPoint( 0., 0., 0. ),
    _data( new InternalSpringData )
{
    _data->_enable[ 0 ] = _data->_enable[ 3 ] = true;
    _data->_linearLowerLimits[ 0 ] = -1.;
    _data->_linearUpperLimits[ 0 ] = 1.;
    _data->_angularLowerLimits[ 0 ] = -vsg::PIf*.5f;
    _data->_angularUpperLimits[ 0 ] = vsg::PIf*.5f;
    _data->_stiffness[ 0 ] = _data->_stiffness[ 3 ] = 10.f;
    _data->_damping[ 0 ] = _data->_damping[ 3 ] = .1f;
}
LinearAngleSpringConstraint::LinearAngleSpringConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
        btRigidBody* rbB, const vsg::mat4& rbBXform,
        const vsg::vec3& axis, const vsg::vec3& point )
  : vsg::Inherit<Constraint, LinearAngleSpringConstraint>( rbA, rbAXform, rbB, rbBXform ),
    _axis( axis ),
    _pivotPoint( point ),
    _data( new InternalSpringData )
{
    _data->_enable[ 0 ] = _data->_enable[ 3 ] = true;
    _data->_linearLowerLimits[ 0 ] = -1.;
    _data->_linearUpperLimits[ 0 ] = 1.;
    _data->_angularLowerLimits[ 0 ] = -vsg::PIf*.5f;
    _data->_angularUpperLimits[ 0 ] = vsg::PIf*.5f;
    _data->_stiffness[ 0 ] = _data->_stiffness[ 3 ] = 10.f;
    _data->_damping[ 0 ] = _data->_damping[ 3 ] = .1f;
}
LinearAngleSpringConstraint::LinearAngleSpringConstraint( const LinearAngleSpringConstraint& rhs, const vsg::CopyOp& copyop )
  : vsg::Inherit<Constraint, LinearAngleSpringConstraint>( rhs, copyop ),
    _axis( rhs._axis ),
    _pivotPoint( rhs._pivotPoint ),
    _data( rhs._data )
{
}
LinearAngleSpringConstraint::~LinearAngleSpringConstraint()
{
}

btGeneric6DofSpringConstraint* LinearAngleSpringConstraint::getAsBtGeneric6DofSpring() const
{
    return( static_cast< btGeneric6DofSpringConstraint* >( getConstraint() ) );
}

void LinearAngleSpringConstraint::setSpringData( InternalSpringData* data )
{
    _data = data;

    if( !getDirty() && ( _constraint != nullptr ) )
    {
        // Dynamically modify the existing constraint.
        btGeneric6DofSpringConstraint* cons = getAsBtGeneric6DofSpring();
        _data->apply( cons );
    }
    else
        setDirty();
}
void LinearAngleSpringConstraint::setAxis( const vsg::vec3& axis )
{
    _axis = axis;
    setDirty();
}
void LinearAngleSpringConstraint::setAxis( const double x, const double y, const double z )
{
    setAxis( vsg::vec3( 2, y, z ) );
}
void LinearAngleSpringConstraint::setPivotPoint( const vsg::vec3& wcPoint )
{
    _pivotPoint = wcPoint;
    setDirty();
}
void LinearAngleSpringConstraint::setPivotPoint( const double x, const double y, const double z )
{
    setPivotPoint( vsg::vec3( x, y, z ) );
}
void LinearAngleSpringConstraint::setLinearLimit( const vsg::vec2& limit )
{
    _data->_linearLowerLimits[ 0 ] = limit[ 0 ];
    _data->_linearUpperLimits[ 0 ] = limit[ 1 ];
    setSpringData( _data.get() );
}
void LinearAngleSpringConstraint::setLinearLimit( const double lo, const double hi )
{
    setLinearLimit( vsg::vec2( lo, hi ) );
}
void LinearAngleSpringConstraint::setAngleLimit( const vsg::vec2& limit )
{
    _data->_angularLowerLimits[ 0 ] = limit[ 0 ];
    _data->_angularUpperLimits[ 0 ] = limit[ 1 ];
    setSpringData( _data.get() );
}
void LinearAngleSpringConstraint::setAngleLimit( const double lo, const double hi )
{
    setAngleLimit( vsg::vec2( lo, hi ) );
}
void LinearAngleSpringConstraint::setLinearStiffness( float stiffness )
{
    _data->_stiffness[ 0 ] = btScalar( stiffness );
    setSpringData( _data.get() );
}
void LinearAngleSpringConstraint::setAngleStiffness( float stiffness )
{
    _data->_stiffness[ 3 ] = btScalar( stiffness );
    setSpringData( _data.get() );
}
void LinearAngleSpringConstraint::setLinearDamping( float damping )
{
    _data->_damping[ 0 ] = btScalar( damping );
    setSpringData( _data.get() );
}
void LinearAngleSpringConstraint::setAngleDamping( float damping )
{
    _data->_damping[ 3 ] = btScalar( damping );
    setSpringData( _data.get() );
}

bool LinearAngleSpringConstraint::operator==( const LinearAngleSpringConstraint& rhs ) const
{
    return( !( operator!=( rhs ) ) );
}
bool LinearAngleSpringConstraint::operator!=( const LinearAngleSpringConstraint& rhs ) const
{
    return(
        ( _axis != rhs._axis ) ||
        ( *_data != *( rhs._data ) ) ||
        ( Constraint::operator!=( static_cast< const Constraint& >( rhs ) ) )
    );
}

void LinearAngleSpringConstraint::createConstraint()
{
    if( _constraint != nullptr )
    {
        delete _constraint;
        _constraint = nullptr;
    }

    _constraint = LinearSpringConstraint::internalCreateSpringConstraint(
        this, _data.get(), _axis, _pivotPoint );

    setDirty( _constraint == nullptr );
}




FixedConstraint::FixedConstraint()
  : vsg::Inherit<Constraint, FixedConstraint>()
{
}
FixedConstraint::FixedConstraint( btRigidBody* rbA, btRigidBody* rbB )
  : vsg::Inherit<Constraint, FixedConstraint>( rbA, rbB )
{
    setDirty();
}
FixedConstraint::FixedConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
        btRigidBody* rbB, const vsg::mat4& rbBXform )
  : vsg::Inherit<Constraint, FixedConstraint>( rbA, rbAXform, rbB, rbBXform )
{
    setDirty();
}
FixedConstraint::FixedConstraint( const FixedConstraint& rhs, const vsg::CopyOp& copyop )
  : vsg::Inherit<Constraint, FixedConstraint>( rhs, copyop )
{
}
FixedConstraint::~FixedConstraint()
{
}

btGeneric6DofConstraint* FixedConstraint::getAsBtGeneric6Dof() const
{
    return( static_cast< btGeneric6DofConstraint* >( getConstraint() ) );
}

void FixedConstraint::createConstraint()
{
    if( _rbA == nullptr )
    {
        std::cerr << "createConstraint: _rbA == nullptr." << std::endl;
        return;
    }

    if( _constraint != nullptr )
    {
        delete _constraint;
        _constraint = nullptr;
    }


    // A's reference frame is just the COM offset.
    vsgbDynamics::MotionState* motion = dynamic_cast< vsgbDynamics::MotionState* >( _rbA->getMotionState() );
    if( motion == nullptr )
    {
        std::cerr<< "SliderConstraint: Invalid MotionState." << std::endl;
        return;
    }
    vsg::vec3 invCom = -( motion->getCenterOfMass() );
    vsg::vec3 scale = motion->getScale();
    vsg::vec3 scaledInvCom = vsg::vec3( invCom[0]*scale[0], invCom[1]*scale[1], invCom[2]*scale[2] );
    const vsg::mat4 invACOM( vsg::translate( scaledInvCom ) );
    btTransform rbAFrame = vsgbCollision::asBtTransform( invACOM );


    btTransform rbBFrame; // OK to not initialize.
    if( _rbB != nullptr )
    {
        // Create a matrix that puts A in B's coordinate space.
        //
        //   1. Inverse B center of mass offset.
        motion = dynamic_cast< vsgbDynamics::MotionState* >( _rbB->getMotionState() );
        if( motion == nullptr )
        {
            std::cerr << "SliderConstraint: Invalid MotionState." << std::endl;
            return;
        }
        invCom = -( motion->getCenterOfMass() );
        scale = motion->getScale();
        scaledInvCom = vsg::vec3( invCom[0]*scale[0], invCom[1]*scale[1], invCom[2]*scale[2] );
        const vsg::mat4 invBCOM( vsg::translate( scaledInvCom ) );
        //
        //   3. Transform from B's origin to A's origin.
        const vsg::mat4 rbBToRbA( vsg::inverse( orthonormalize( _rbBXform ) ) *
            orthonormalize( _rbAXform ) );
        //
        //   4. The final rbB frame matrix.
        rbBFrame = vsgbCollision::asBtTransform(
            invBCOM * rbBToRbA );
    }


    btGeneric6DofConstraint* cons;
    if( _rbB != nullptr )
        cons = new btGeneric6DofConstraint( *_rbA, *_rbB, rbAFrame, rbBFrame, false );
    else
        cons = new btGeneric6DofConstraint( *_rbA, rbAFrame, true );
    cons->setAngularLowerLimit( btVector3( 0., 0., 0. ) );
    cons->setAngularUpperLimit( btVector3( 0., 0., 0. ) );
    _constraint = cons;

    setDirty( false );
}




PlanarConstraint::PlanarConstraint()
  : vsg::Inherit<Constraint, PlanarConstraint>()
{
}
PlanarConstraint::PlanarConstraint( btRigidBody* rbA, btRigidBody* rbB,
        const vsg::vec2& loLimit, const vsg::vec2& hiLimit, const vsg::mat4& orient )
  : vsg::Inherit<Constraint, PlanarConstraint>( rbA, rbB ),
    _loLimit( loLimit ),
    _hiLimit( hiLimit ),
    _orient( orient )
{
    setDirty( true );
}
PlanarConstraint::PlanarConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
        const vsg::vec2& loLimit, const vsg::vec2& hiLimit, const vsg::mat4& orient )
  : vsg::Inherit<Constraint, PlanarConstraint>( rbA, rbAXform ),
    _loLimit( loLimit ),
    _hiLimit( hiLimit ),
    _orient( orient )
{
    setDirty( true );
}
PlanarConstraint::PlanarConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
        btRigidBody* rbB, const vsg::mat4& rbBXform,
        const vsg::vec2& loLimit, const vsg::vec2& hiLimit, const vsg::mat4& orient )
  : vsg::Inherit<Constraint, PlanarConstraint>( rbA, rbAXform, rbB, rbBXform ),
    _loLimit( loLimit ),
    _hiLimit( hiLimit ),
    _orient( orient )
{
    setDirty( true );
}
PlanarConstraint::PlanarConstraint( const PlanarConstraint& rhs, const vsg::CopyOp& copyop )
  : vsg::Inherit<Constraint, PlanarConstraint>( rhs, copyop ),
    _loLimit( rhs._loLimit ),
    _hiLimit( rhs._hiLimit ),
    _orient( rhs._orient )
{
    setDirty( true );
}
PlanarConstraint::~PlanarConstraint()
{
}

btGeneric6DofConstraint* PlanarConstraint::getAsBtGeneric6Dof() const
{
    return( static_cast< btGeneric6DofConstraint* >( getConstraint() ) );
}

void PlanarConstraint::setLowLimit( const vsg::vec2& loLimit )
{
    _loLimit = loLimit;
    setDirty( true );
}
void PlanarConstraint::setLowLimit( const double x, const double y )
{
    setLowLimit( vsg::vec2( x, y ) );
}
void PlanarConstraint::setHighLimit( const vsg::vec2& hiLimit )
{
    _hiLimit = hiLimit;
    setDirty( true );
}
void PlanarConstraint::setHighLimit( const double x, const double y )
{
    setHighLimit( vsg::vec2( x, y ) );
}
void PlanarConstraint::setOrientation( const vsg::mat4& orient )
{
    _orient = orient;
    setDirty( true );
}

bool PlanarConstraint::operator==( const PlanarConstraint& rhs ) const
{
    return( !( operator!=( rhs ) ) );
}
bool PlanarConstraint::operator!=( const PlanarConstraint& rhs ) const
{
    return(
        ( _loLimit != rhs._loLimit ) ||
        ( _hiLimit != rhs._hiLimit ) ||
        ( _orient != rhs._orient ) ||
        ( Constraint::operator!=( static_cast< const Constraint& >( rhs ) ) )
    );
}

void PlanarConstraint::createConstraint()
{
    if( _rbA == nullptr )
    {
        std::cerr << "createConstraint: _rbA == nullptr." << std::endl;
        return;
    }

    if( _constraint != nullptr )
    {
        delete _constraint;
        _constraint = nullptr;
    }


    // Planar and Box share common code to compute the Bullet constraint
    // reference frames.
    btTransform rbAFrame, rbBFrame;
    BoxConstraint::internalPlanarBoxFrameComputation(
        rbAFrame, rbBFrame, this, _orient );


    btGeneric6DofConstraint* cons;
    if( _rbB != nullptr )
        cons = new btGeneric6DofConstraint( *_rbA, *_rbB, rbAFrame, rbBFrame, false );
    else
        cons = new btGeneric6DofConstraint( *_rbA, rbAFrame, true );
    cons->setAngularLowerLimit( btVector3( 0., 0., 0. ) );
    cons->setAngularUpperLimit( btVector3( 0., 0., 0. ) );

    const vsg::vec3 loLimit( _loLimit[ 0 ], _loLimit[ 1 ], 0. );
    const vsg::vec3 hiLimit( _hiLimit[ 0 ], _hiLimit[ 1 ], 0. );
    cons->setLinearLowerLimit( vsgbCollision::asBtVector3( loLimit ) );
    cons->setLinearUpperLimit( vsgbCollision::asBtVector3( hiLimit ) );

    _constraint = cons;

    setDirty( false );
}




BoxConstraint::BoxConstraint()
    :vsg::Inherit<Constraint, BoxConstraint>()
{
}
BoxConstraint::BoxConstraint( btRigidBody* rbA, btRigidBody* rbB,
        const vsg::vec3& loLimit, const vsg::vec3& hiLimit, const vsg::mat4& orient )
  : vsg::Inherit<Constraint, BoxConstraint>( rbA, rbB ),
    _loLimit( loLimit ),
    _hiLimit( hiLimit ),
    _orient( orient )
{
    setDirty( true );
}
BoxConstraint::BoxConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
        const vsg::vec3& loLimit, const vsg::vec3& hiLimit, const vsg::mat4& orient )
  : vsg::Inherit<Constraint, BoxConstraint>( rbA, rbAXform ),
    _loLimit( loLimit ),
    _hiLimit( hiLimit ),
    _orient( orient )
{
    setDirty( true );
}
BoxConstraint::BoxConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
        btRigidBody* rbB, const vsg::mat4& rbBXform,
        const vsg::vec3& loLimit, const vsg::vec3& hiLimit, const vsg::mat4& orient )
  : vsg::Inherit<Constraint, BoxConstraint>( rbA, rbAXform, rbB, rbBXform ),
    _loLimit( loLimit ),
    _hiLimit( hiLimit ),
    _orient( orient )
{
    setDirty( true );
}
BoxConstraint::BoxConstraint( const BoxConstraint& rhs, const vsg::CopyOp& copyop )
  : vsg::Inherit<Constraint, BoxConstraint>( rhs, copyop ),
    _loLimit( rhs._loLimit ),
    _hiLimit( rhs._hiLimit ),
    _orient( rhs._orient )
{
    setDirty( true );
}
BoxConstraint::~BoxConstraint()
{
}

btGeneric6DofConstraint* BoxConstraint::getAsBtGeneric6Dof() const
{
    return( static_cast< btGeneric6DofConstraint* >( getConstraint() ) );
}

void BoxConstraint::setLowLimit( const vsg::vec3& loLimit )
{
    _loLimit = loLimit;
    setDirty( true );
}
void BoxConstraint::setLowLimit( const double x, const double y, const double z )
{
    setLowLimit( vsg::vec3( x, y, z ) );
}
void BoxConstraint::setHighLimit( const vsg::vec3& hiLimit )
{
    _hiLimit = hiLimit;
    setDirty( true );
}
void BoxConstraint::setHighLimit( const double x, const double y, const double z )
{
    setHighLimit( vsg::vec3( x, y, z ) );
}
void BoxConstraint::setOrientation( const vsg::mat4& orient )
{
    _orient = orient;
    setDirty( true );
}

bool BoxConstraint::operator==( const BoxConstraint& rhs ) const
{
    return( !( operator!=( rhs ) ) );
}
bool BoxConstraint::operator!=( const BoxConstraint& rhs ) const
{
    return(
        ( _loLimit != rhs._loLimit ) ||
        ( _hiLimit != rhs._hiLimit ) ||
        ( _orient != rhs._orient ) ||
        ( Constraint::operator!=( static_cast< const Constraint& >( rhs ) ) )
    );
}

void BoxConstraint::createConstraint()
{
    if( _rbA == nullptr )
    {
        std::cerr<< "createConstraint: _rbA == nullptr." << std::endl;
        return;
    }

    if( _constraint != nullptr )
    {
        delete _constraint;
        _constraint = nullptr;
    }


    // Planar and Box share common code to compute the Bullet constraint
    // reference frames.
    btTransform rbAFrame, rbBFrame;
    BoxConstraint::internalPlanarBoxFrameComputation(
        rbAFrame, rbBFrame, this, _orient );


    btGeneric6DofConstraint* cons;
    if( _rbB != nullptr )
        cons = new btGeneric6DofConstraint( *_rbA, *_rbB, rbAFrame, rbBFrame, false );
    else
        cons = new btGeneric6DofConstraint( *_rbA, rbAFrame, true );
    cons->setAngularLowerLimit( btVector3( 0., 0., 0. ) );
    cons->setAngularUpperLimit( btVector3( 0., 0., 0. ) );

    cons->setLinearLowerLimit( vsgbCollision::asBtVector3( _loLimit ) );
    cons->setLinearUpperLimit( vsgbCollision::asBtVector3( _hiLimit ) );

    _constraint = cons;

    setDirty( false );
}

void BoxConstraint::internalPlanarBoxFrameComputation(
        btTransform& aFrame, btTransform& bFrame,
        Constraint* cons, const vsg::mat4& orient )
{
    // Remove any translation that might be in the orient matrix.
    vsg::mat4 orientation( orient );
    //orientation.setTrans( 0., 0., 0. );
    orientation[3][0] = 0.;
    orientation[3][1] = 0.;
    orientation[3][2] = 0.;
    btRigidBody* rbA, * rbB;
    cons->getRigidBodies( rbA, rbB );


    // Create a matrix that puts A in B's coordinate space, accounting
    // for orientation of the constraint axes.
    //
    //   1. Inverse A center of mass offset.
    vsgbDynamics::MotionState* motion = dynamic_cast< vsgbDynamics::MotionState* >( rbA->getMotionState() );
    if( motion == nullptr )
    {
        std::cerr<< "InternalCreateSpring: Invalid MotionState." << std::endl;
        return;
    }
    vsg::vec3 invCom = -( motion->getCenterOfMass() );
    vsg::vec3 scale = motion->getScale();
    vsg::vec3 scaledInvCom = vsg::vec3( invCom[0]*scale[0], invCom[1]*scale[1], invCom[2]*scale[2] );
    const vsg::mat4 invACOM( vsg::translate( scaledInvCom ) );
    //
    //   2. Transform A back to the origin.
    const vsg::mat4 invAXform( vsg::inverse( cons->getAXform() ) );
    //
    //   3. The final rbA frame matrix.
    aFrame = vsgbCollision::asBtTransform( 
        orientation * invAXform * invACOM );


    if( rbB != nullptr )
    {
        // Create a matrix that orients the spring axis/point in B's coordinate space.
        //
        //   1. Inverse B center of mass offset.
        motion = dynamic_cast< vsgbDynamics::MotionState* >( rbB->getMotionState() );
        if( motion == nullptr )
        {
            std::cerr<< "InternalCreateSpring: Invalid MotionState." << std::endl;
            return;
        }
        invCom = -( motion->getCenterOfMass() );
        scale = motion->getScale();
        scaledInvCom = vsg::vec3( invCom[0]*scale[0], invCom[1]*scale[1], invCom[2]*scale[2] );
        const vsg::mat4 invBCOM( vsg::translate( scaledInvCom ) );
        //
        //   2. Transform B back to the origin.
        const vsg::mat4 invBXform( vsg::inverse( cons->getBXform() ) );
        //
        //   3. The final rbB frame matrix.
        bFrame = vsgbCollision::asBtTransform( 
            orientation * invBXform * invBCOM );
    }
}




HingeConstraint::HingeConstraint()
  : vsg::Inherit<Constraint, HingeConstraint>(),
    _axis( 0., 0., 1. ),
    _pivotPoint( 0., 0., 0. ),
    _limit( vsg::PI, vsg::PI )
{
}
HingeConstraint::HingeConstraint( btRigidBody* rbA, btRigidBody* rbB,
        const vsg::vec3& axis, const vsg::vec3& pivotPoint,
        const vsg::vec2& limit )
  : vsg::Inherit<Constraint, HingeConstraint>( rbA, rbB ),
    _axis( axis ),
    _pivotPoint( pivotPoint ),
    _limit( limit )
{
}
HingeConstraint::HingeConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
        btRigidBody* rbB, const vsg::mat4& rbBXform,
        const vsg::vec3& axis, const vsg::vec3& pivotPoint,
        const vsg::vec2& limit )
  : vsg::Inherit<Constraint, HingeConstraint>( rbA, rbAXform, rbB, rbBXform ),
    _axis( axis ),
    _pivotPoint( pivotPoint ),
    _limit( limit )
{
}
HingeConstraint::HingeConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
        const vsg::vec3& axis, const vsg::vec3& pivotPoint,
        const vsg::vec2& limit )
  : vsg::Inherit<Constraint, HingeConstraint>( rbA, rbAXform ),
    _axis( axis ),
    _pivotPoint( pivotPoint ),
    _limit( limit )
{
}
HingeConstraint::HingeConstraint( const HingeConstraint& rhs, const vsg::CopyOp& copyop )
  : vsg::Inherit<Constraint, HingeConstraint>( rhs, copyop ),
    _axis( rhs._axis ),
    _pivotPoint( rhs._pivotPoint ),
    _limit( rhs._limit )
{
}
HingeConstraint::~HingeConstraint()
{
}

btHingeConstraint* HingeConstraint::getAsBtHinge() const
{
    return( static_cast< btHingeConstraint* >( getConstraint() ) );
}

void HingeConstraint::setAxis( const vsg::vec3& axis )
{
    _axis = axis;
    setDirty( true );
}
void HingeConstraint::setAxis( const double x, const double y, const double z )
{
    setAxis( vsg::vec3( x, y, z ) );
}
void HingeConstraint::setPivotPoint( const vsg::vec3& wcPoint )
{
    _pivotPoint = wcPoint;
    setDirty( true );
}
void HingeConstraint::setPivotPoint( const double x, const double y, const double z )
{
    setPivotPoint( vsg::vec3( x, y, z ) );
}
void HingeConstraint::setLimit( const vsg::vec2& limit )
{
    _limit = limit;

    if( !getDirty() && ( _constraint != nullptr ) )
    {
        // Dynamically modify the existing constraint.
        btHingeConstraint* cons = getAsBtHinge();
        cons->setLimit( _limit[ 0 ], _limit[ 1 ] );
    }
    else
        setDirty();
}
void HingeConstraint::setLimit( const double lo, const double hi )
{
    setLimit( vsg::vec2( lo, hi ) );
}

bool HingeConstraint::operator==( const HingeConstraint& rhs ) const
{
    return( !( operator!=( rhs ) ) );
}
bool HingeConstraint::operator!=( const HingeConstraint& rhs ) const
{
    return(
        ( _axis != rhs._axis ) ||
        ( _pivotPoint != rhs._pivotPoint ) ||
        ( _limit != rhs._limit ) ||
        ( Constraint::operator!=( static_cast< const Constraint& >( rhs ) ) )
    );
}


void HingeConstraint::createConstraint()
{
    if( _rbA == nullptr )
    {
        std::cerr << "createConstraint: _rbA == nullptr." << std::endl;
        return;
    }

    if( _constraint != nullptr )
    {
        delete _constraint;
        _constraint = nullptr;
    }


    // Put pivot point and axis into A's space.
    // 
    // 1. Inverse A center of mass:
    vsgbDynamics::MotionState* motion = dynamic_cast< vsgbDynamics::MotionState* >( _rbA->getMotionState() );
    if( motion == nullptr )
    {
        std::cerr<< "HingeConstraint: Invalid MotionState." << std::endl;
        return;
    }
    vsg::vec3 invCom = -( motion->getCenterOfMass() );
    vsg::vec3 scale = motion->getScale();
    vsg::vec3 scaledInvCom = vsg::vec3( invCom[0]*scale[0], invCom[1]*scale[1], invCom[2]*scale[2] );
    vsg::mat4 invACom = vsg::translate( scaledInvCom );
    //
    // 2. Inverse A transform.
    vsg::mat4 invAXform = vsg::inverse( orthonormalize( _rbAXform ) );
    //
    // 3. A's orientation.
    vsg::mat4 rbAOrient( orthonormalize( _rbAXform ) );
    //rbAOrient.setTrans( 0., 0., 0. );
    rbAOrient[3][0] = 0.;
    rbAOrient[3][1] = 0.;
    rbAOrient[3][2] = 0.;
    //
    // Transform the point and axis.
    btVector3 pivotInA( vsgbCollision::asBtVector3(
        _pivotPoint * invAXform * rbAOrient * invACom ) );
    btVector3 axisInA( vsgbCollision::asBtVector3( _axis * rbAOrient ) );


    btVector3 pivotInB, axisInB;
    if( _rbB != nullptr )
    {
        // Put pivot point and axis into B's space.
        // 
        // 1. Inverse B center of mass:
        vsgbDynamics::MotionState* motion = dynamic_cast< vsgbDynamics::MotionState* >( _rbB->getMotionState() );
        if( motion == nullptr )
        {
            std::cerr << "HingeConstraint: Invalid MotionState." << std::endl;
            return;
        }
        invCom = -( motion->getCenterOfMass() );
        scale = motion->getScale();
        scaledInvCom = vsg::vec3( invCom[0]*scale[0], invCom[1]*scale[1], invCom[2]*scale[2] );
        vsg::mat4 invBCom = vsg::translate( scaledInvCom );
        //
        // 2. Inverse A transform.
        vsg::mat4 invBXform = vsg::inverse( orthonormalize( _rbBXform ) );
        //
        // 3. B's orientation.
        vsg::mat4 rbBOrient( orthonormalize( _rbBXform ) );
        //rbBOrient.setTrans( 0., 0., 0. );
        rbBOrient[3][0] = 0.;
        rbBOrient[3][1] = 0.;
        rbBOrient[3][2] = 0.;
        // Transform the point and axis.
        pivotInB = vsgbCollision::asBtVector3(
            _pivotPoint * invBXform * rbBOrient * invBCom );
        axisInB = vsgbCollision::asBtVector3( _axis * rbBOrient );
    }


    btHingeConstraint* hinge;
    if( _rbB != nullptr )
        hinge = new btHingeConstraint( *_rbA, *_rbB,
                pivotInA, pivotInB, axisInA, axisInB, false );
    else
        hinge = new btHingeConstraint( *_rbA, pivotInA, axisInA, false );

    hinge->setLimit( _limit[ 0 ], _limit[ 1 ] );

    _constraint = hinge;
}




CardanConstraint::CardanConstraint()
  : vsg::Inherit<Constraint, CardanConstraint>(),
    _axisA( 0., 1., 0. ),
    _axisB( 1., 0., 0. ),
    _point( 0., 0., 0. )
{
}
CardanConstraint::CardanConstraint( btRigidBody* rbA, btRigidBody* rbB )
  : vsg::Inherit<Constraint, CardanConstraint>( rbA, rbB ),
    _axisA( 0., 1., 0. ),
    _axisB( 1., 0., 0. ),
    _point( 0., 0., 0. )
{
}
CardanConstraint::CardanConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
        btRigidBody* rbB, const vsg::mat4& rbBXform,
        const vsg::vec3& axisA, const vsg::vec3& axisB, const vsg::vec3& point )
  : vsg::Inherit<Constraint, CardanConstraint>( rbA, rbAXform, rbB, rbBXform ),
    _axisA( axisA ),
    _axisB( axisB ),
    _point( point )
{
}
CardanConstraint::CardanConstraint( const CardanConstraint& rhs, const vsg::CopyOp& copyop )
  : vsg::Inherit<Constraint, CardanConstraint>( rhs, copyop ),
    _axisA( rhs._axisA ),
    _axisB( rhs._axisB ),
    _point( rhs._point )
{
}
CardanConstraint::~CardanConstraint()
{
}

btUniversalConstraint* CardanConstraint::getAsBtUniversal() const
{
    return( static_cast< btUniversalConstraint* >( getConstraint() ) );
}

void CardanConstraint::setAxisA( const vsg::vec3& axisA )
{
    _axisA = axisA;
    setDirty( true );
}
void CardanConstraint::setAxisA( const double x, const double y, const double z )
{
    setAxisA( vsg::vec3( x, y, z ) );
}
void CardanConstraint::setAxisB( const vsg::vec3& axisB )
{
    _axisB = axisB;
    setDirty( true );
}
void CardanConstraint::setAxisB( const double x, const double y, const double z )
{
    setAxisB( vsg::vec3( x, y, z ) );
}
void CardanConstraint::setAnchorPoint( const vsg::vec3& wcPoint )
{
    _point = wcPoint;
    setDirty( true );
}
void CardanConstraint::setAnchorPoint( const double x, const double y, const double z )
{
    setAnchorPoint( vsg::vec3( x, y, z ) );
}

bool CardanConstraint::operator==( const CardanConstraint& rhs ) const
{
    return( !( operator!=( rhs ) ) );
}
bool CardanConstraint::operator!=( const CardanConstraint& rhs ) const
{
    return(
        ( _axisA != rhs._axisA ) ||
        ( _axisB != rhs._axisB ) ||
        ( _point != rhs._point ) ||
        ( Constraint::operator!=( static_cast< const Constraint& >( rhs ) ) )
    );
}

void CardanConstraint::createConstraint()
{
    if( ( _rbA == nullptr ) || ( _rbB == nullptr ) )
    {
        std::cerr << "createConstraint: _rbA == nullptr or _rbB == nullptr." << std::endl;
        return;
    }

    if( _constraint != nullptr )
    {
        delete _constraint;
        _constraint = nullptr;
    }


    // Transform the world coordinate _axisA into A's local coordinates.
    vsg::mat4 aOrient = orthonormalize( _rbAXform) ;
    //aOrient.setTrans( 0., 0., 0. );
    aOrient[3][0] = 0.;
    aOrient[3][1] = 0.;
    aOrient[3][2] = 0.;
    btVector3 localAxisA = vsgbCollision::asBtVector3(
        _axisA * vsg::inverse( aOrient ) );
    localAxisA.normalize();


    // Force _axisB to be orthogonal to _axisA.
    vsg::vec3 c = cross(_axisA, _axisB);
    vsg::vec3 axisB( cross(c, _axisA ));

    // Transform the world coordinate _axisB into B's local coordinates.
    vsg::mat4 bOrient = orthonormalize( _rbBXform );
    //bOrient.setTrans( 0., 0., 0. );
    bOrient[3][0] = 0.;
    bOrient[3][1] = 0.;
    bOrient[3][2] = 0.;
    btVector3 localAxisB = vsgbCollision::asBtVector3(
        axisB * vsg::inverse( bOrient ) );
    localAxisB.normalize();


    btVector3 btPt( vsgbCollision::asBtVector3( _point ) );
    btUniversalConstraint* cons = new btUniversalConstraint( *_rbA, *_rbB,
        btPt, localAxisA, localAxisB );

    _constraint = cons;

    setDirty( false );
}
    



BallAndSocketConstraint::BallAndSocketConstraint()
  : vsg::Inherit<Constraint, BallAndSocketConstraint>()
{
}
BallAndSocketConstraint::BallAndSocketConstraint( btRigidBody* rbA, btRigidBody* rbB )
  : vsg::Inherit<Constraint, BallAndSocketConstraint>( rbA, rbB )
{
}
BallAndSocketConstraint::BallAndSocketConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
            btRigidBody* rbB, const vsg::mat4& rbBXform, const vsg::vec3& wcPoint )
  : vsg::Inherit<Constraint, BallAndSocketConstraint>( rbA, rbAXform, rbB, rbBXform ),
    _point( wcPoint )
{
}
BallAndSocketConstraint::BallAndSocketConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
            const vsg::vec3& wcPoint )
  : vsg::Inherit<Constraint, BallAndSocketConstraint>( rbA, rbAXform ),
    _point( wcPoint )
{
}
BallAndSocketConstraint::BallAndSocketConstraint( const BallAndSocketConstraint& rhs, const vsg::CopyOp& copyop )
  : vsg::Inherit<Constraint, BallAndSocketConstraint>( rhs, copyop ),
    _point( rhs._point )
{
}
BallAndSocketConstraint::~BallAndSocketConstraint()
{
}

btPoint2PointConstraint* BallAndSocketConstraint::getAsBtPoint2Point() const
{
    return( static_cast< btPoint2PointConstraint* >( getConstraint() ) );
}

void BallAndSocketConstraint::setPoint( const vsg::vec3& wcPoint )
{
    _point = wcPoint;
    setDirty();
}
void BallAndSocketConstraint::setPoint( const double x, const double y, const double z )
{
    setPoint( vsg::vec3( x, y, z ) );
}

bool BallAndSocketConstraint::operator==( const BallAndSocketConstraint& rhs ) const
{
    return( !( operator!=( rhs ) ) );
}
bool BallAndSocketConstraint::operator!=( const BallAndSocketConstraint& rhs ) const
{
    return(
        ( _point != rhs._point ) ||
        ( Constraint::operator!=( static_cast< const Constraint& >( rhs ) ) )
    );
}

void BallAndSocketConstraint::createConstraint()
{
    if( _rbA == nullptr )
    {
        std::cerr << "createConstraint: _rbA == nullptr." << std::endl;
        return;
    }

    if( _constraint != nullptr )
    {
        delete _constraint;
        _constraint = nullptr;
    }


    // Create a matrix that returns A to the origin.
    //
    //   1. Inverse A center of mass offset.
    vsgbDynamics::MotionState* motion = dynamic_cast< vsgbDynamics::MotionState* >( _rbA->getMotionState() );
    if( motion == nullptr )
    {
        std::cerr << "InternalCreateSpring: Invalid MotionState." << std::endl;
        return;
    }
    vsg::vec3 invCom = -( motion->getCenterOfMass() );
    vsg::vec3 scale = motion->getScale();
    vsg::vec3 scaledInvCom = vsg::vec3( invCom[0]*scale[0], invCom[1]*scale[1], invCom[2]*scale[2] );
    const vsg::mat4 invACOM( vsg::translate( scaledInvCom ) );
    //
    //   2. Transform A back to the origin.
    const vsg::mat4 invAXform( vsg::inverse( orthonormalize( _rbAXform ) ) );
    //
    //   3. The final rbA frame matrix.
    const vsg::mat4 aXform( invAXform * invACOM );

    // And now compute the WC point in rbA space:
    const btVector3 aPt = vsgbCollision::asBtVector3( _point * aXform );


    btVector3 bPt;
    if( _rbB != nullptr )
    {
        // Create a matrix that returns B to the origin.
        //
        //   1. Inverse B center of mass offset.
        motion = dynamic_cast< vsgbDynamics::MotionState* >( _rbB->getMotionState() );
        if( motion == nullptr )
        {
            std::cerr << "InternalCreateSpring: Invalid MotionState." << std::endl;
            return;
        }
        invCom = -( motion->getCenterOfMass() );
        scale = motion->getScale();
        scaledInvCom = vsg::vec3( invCom[0]*scale[0], invCom[1]*scale[1], invCom[2]*scale[2] );
        const vsg::mat4 invBCOM( vsg::translate( scaledInvCom ) );
        //
        //   2. Transform B back to the origin.
        const vsg::mat4 invBXform( vsg::inverse( orthonormalize( _rbBXform ) ) );
        //
        //   3. The final rbB frame matrix.
        const vsg::mat4 bXform = invBXform * invBCOM;

        // And now compute the WC point in rbB space:
        bPt = vsgbCollision::asBtVector3( _point * bXform );
    }


    btPoint2PointConstraint* cons;
    if( _rbB != nullptr )
        cons = new btPoint2PointConstraint( *_rbA, *_rbB, aPt, bPt );
    else
        cons = new btPoint2PointConstraint( *_rbA, aPt );
    _constraint = cons;

    setDirty( false );
}




RagdollConstraint::RagdollConstraint()
  : vsg::Inherit<Constraint, RagdollConstraint>(),
    _axis( 1., 0., 0. ),
    _angle( vsg::PIf*.5f )
{
}
RagdollConstraint::RagdollConstraint( btRigidBody* rbA, btRigidBody* rbB )
  : vsg::Inherit<Constraint, RagdollConstraint>( rbA, rbB ),
    _axis( 1., 0., 0. ),
    _angle( vsg::PIf*.5f )
{
}
RagdollConstraint::RagdollConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
        const vsg::vec3& wcPoint, const vsg::vec3& wcAxis, const double angleRadians )
  : vsg::Inherit<Constraint, RagdollConstraint>( rbA, rbAXform ),
    _point( wcPoint ),
    _axis( wcAxis ),
    _angle( angleRadians )
{
}
RagdollConstraint::RagdollConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
        btRigidBody* rbB, const vsg::mat4& rbBXform,
        const vsg::vec3& wcPoint, const vsg::vec3& wcAxis, const double angleRadians )
  : vsg::Inherit<Constraint, RagdollConstraint>( rbA, rbAXform, rbB, rbBXform ),
    _point( wcPoint ),
    _axis( wcAxis ),
    _angle( angleRadians )
{
}
RagdollConstraint::RagdollConstraint( const RagdollConstraint& rhs, const vsg::CopyOp& copyop )
  : vsg::Inherit<Constraint, RagdollConstraint>( rhs, copyop ),
    _point( rhs._point ),
    _axis( rhs._axis ),
    _angle( rhs._angle )
{
}
RagdollConstraint::~RagdollConstraint()
{
}

btConeTwistConstraint* RagdollConstraint::getAsBtConeTwist() const
{
    return( static_cast< btConeTwistConstraint* >( getConstraint() ) );
}

void RagdollConstraint::setPoint( const vsg::vec3& wcPoint )
{
    _point = wcPoint;
    setDirty( true );
}
void RagdollConstraint::setPoint( const double x, const double y, const double z )
{
    setPoint( vsg::vec3( x, y, z ) );
}
void RagdollConstraint::setAxis( const vsg::vec3& wcAxis )
{
    _axis = wcAxis;
    setDirty( true );
}
void RagdollConstraint::setAxis( const double x, const double y, const double z )
{
    setAxis( vsg::vec3( x, y, z ) );
}
void RagdollConstraint::setAngle( const double angleRadians )
{
    _angle = angleRadians;

    if( !getDirty() && ( _constraint != nullptr ) )
    {
        // Dynamically modify the existing constraint.
        btConeTwistConstraint* cons = getAsBtConeTwist();
        cons->setLimit( 4, _angle );
        cons->setLimit( 5, _angle );
    }
    else
        setDirty();
}

bool RagdollConstraint::operator==( const RagdollConstraint& rhs ) const
{
    return( !( operator!=( rhs ) ) );
}
bool RagdollConstraint::operator!=( const RagdollConstraint& rhs ) const
{
    return(
        ( _point != rhs._point ) ||
        ( _axis != rhs._axis ) ||
        ( _angle != rhs._angle ) ||
        ( Constraint::operator!=( static_cast< const Constraint& >( rhs ) ) )
    );
}

void RagdollConstraint::createConstraint()
{
    if( _rbA == nullptr )
    {
        std::cerr << "createConstraint: _rbA == nullptr." << std::endl;
        return;
    }

    if( _constraint != nullptr )
    {
        delete _constraint;
        _constraint = nullptr;
    }


    // Orientation matrix for the ragdoll x-axis / point.
    vsg::vec3 localAxis( _axis );
    localAxis = normalize(localAxis);
    const vsg::mat4 orientation =
        vsg::rotate( vsgbCollision::makeRotate(vsg::vec3( 1., 0., 0. ), localAxis ) )*
        vsg::translate( _point );


    // Create a matrix that puts A in B's coordinate space, accounting
    // for orientation of the constraint axes.
    //
    //   1. Inverse A center of mass offset.
    vsgbDynamics::MotionState* motion = dynamic_cast< vsgbDynamics::MotionState* >( _rbA->getMotionState() );
    if( motion == nullptr )
    {
        std::cerr << "InternalCreateSpring: Invalid MotionState." << std::endl;
        return;
    }
    vsg::vec3 invCom = -( motion->getCenterOfMass() );
    vsg::vec3 scale = motion->getScale();
    vsg::vec3 scaledInvCom = vsg::vec3( invCom[0]*scale[0], invCom[1]*scale[1], invCom[2]*scale[2] );
    const vsg::mat4 invACOM( vsg::translate( scaledInvCom ) );
    //
    //   2. Transform A back to the origin.
    const vsg::mat4 invAXform( vsg::inverse( orthonormalize( _rbAXform ) ) );
    //
    //   3. The final rbA frame matrix.
    btTransform rbAFrame = vsgbCollision::asBtTransform( 
        orientation * invAXform * invACOM );


    btTransform rbBFrame;
    if( _rbB != nullptr )
    {
        // Create a matrix that orients the spring axis/point in B's coordinate space.
        //
        //   1. Inverse B center of mass offset.
        motion = dynamic_cast< vsgbDynamics::MotionState* >( _rbB->getMotionState() );
        if( motion == nullptr )
        {
            std::cerr << "InternalCreateSpring: Invalid MotionState." << std::endl;
            return;
        }
        invCom = -( motion->getCenterOfMass() );
        scale = motion->getScale();
        scaledInvCom = vsg::vec3( invCom[0]*scale[0], invCom[1]*scale[1], invCom[2]*scale[2] );
        const vsg::mat4 invBCOM( vsg::translate( scaledInvCom ) );
        //
        //   2. Transform B back to the origin.
        const vsg::mat4 invBXform( vsg::inverse( orthonormalize( _rbBXform ) ) );
        //
        //   3. The final rbB frame matrix.
        rbBFrame = vsgbCollision::asBtTransform( 
            orientation * invBXform * invBCOM );
    }


    btConeTwistConstraint* cons;
    if( _rbB != nullptr )
        cons = new btConeTwistConstraint( *_rbA, *_rbB, rbAFrame, rbBFrame );
    else
        cons = new btConeTwistConstraint( *_rbA, rbAFrame );

    // The btConeTwistConstraint cone is along axis x (index 3).
    // It allows an assymetrical cone spread in y and z (indices 4 and 5).
    // We set both y and z to the same angle for a symmetrical spread.
    cons->setLimit( 4, _angle );
    cons->setLimit( 5, _angle );

    _constraint = cons;

    setDirty( false );
}




WheelSuspensionConstraint::WheelSuspensionConstraint()
  : vsg::Inherit<Constraint, WheelSuspensionConstraint>(),
    _springAxis( 0., 0., 1. ),
    _axleAxis( 0., 1., 0. ),
    _linearLimit( -1., 1. ),
    _angleLimit( -vsg::PIf*.25f, vsg::PIf*.25f ),
    _point( 0., 0., 0. )
{
}
WheelSuspensionConstraint::WheelSuspensionConstraint( btRigidBody* rbA, btRigidBody* rbB,
        const vsg::vec3& springAxis, const vsg::vec3& axleAxis,
        const vsg::vec2& linearLimit, const vsg::vec2& angleLimit,
        const vsg::vec3& point )
  : vsg::Inherit<Constraint, WheelSuspensionConstraint>( rbA, rbB ),
    _springAxis( springAxis ),
    _axleAxis( axleAxis ),
    _linearLimit( linearLimit ),
    _angleLimit( angleLimit ),
    _point( point )
{
}
WheelSuspensionConstraint::WheelSuspensionConstraint( const WheelSuspensionConstraint& rhs, const vsg::CopyOp& copyop )
  : vsg::Inherit<Constraint, WheelSuspensionConstraint>( rhs, copyop ),
    _springAxis( rhs._springAxis ),
    _axleAxis( rhs._axleAxis ),
    _linearLimit( rhs._linearLimit ),
    _angleLimit( rhs._angleLimit ),
    _point( rhs._point )
{
}
WheelSuspensionConstraint::~WheelSuspensionConstraint()
{
}

btHinge2Constraint* WheelSuspensionConstraint::getAsBtHinge2() const
{
    return( static_cast< btHinge2Constraint* >( getConstraint() ) );
}

void WheelSuspensionConstraint::setSpringAxis( const vsg::vec3& springAxis )
{
    _springAxis = springAxis;
    setDirty( true );
}
void WheelSuspensionConstraint::setSpringAxis( const double x, const double y, const double z )
{
    setSpringAxis( vsg::vec3( x, y, z ) );
}
void WheelSuspensionConstraint::setAxleAxis( const vsg::vec3& axleAxis )
{
    _axleAxis = axleAxis;
    setDirty( true );
}
void WheelSuspensionConstraint::setAxleAxis( const double x, const double y, const double z )
{
    setAxleAxis( vsg::vec3( x, y, z ) );
}
void WheelSuspensionConstraint::setLinearLimit( const vsg::vec2& linearLimit )
{
    _linearLimit = linearLimit;

    if( !getDirty() && ( _constraint != nullptr ) )
    {
        // Dynamically modify the existing constraint.
        btHinge2Constraint* cons = getAsBtHinge2();
        cons->setLinearLowerLimit( btVector3( 0., 0., _linearLimit[ 0 ] ) );
        cons->setLinearUpperLimit( btVector3( 0., 0., _linearLimit[ 1 ] ) );
    }
    else
        setDirty();
}
void WheelSuspensionConstraint::setLinearLimit( const double lo, const double hi )
{
    setLinearLimit( vsg::vec2( lo, hi ) );
}
void WheelSuspensionConstraint::setAngleLimit( const vsg::vec2& limitRadians )
{
    _angleLimit = limitRadians;

    if( !getDirty() && ( _constraint != nullptr ) )
    {
        // Dynamically modify the existing constraint.
        btHinge2Constraint* cons = getAsBtHinge2();
        cons->setLowerLimit( _angleLimit[ 0 ] );
        cons->setUpperLimit( _angleLimit[ 1 ] );
    }
    else
        setDirty();
}
void WheelSuspensionConstraint::setAngleLimit( const double lo, const double hi )
{
    setAngleLimit( vsg::vec2( lo, hi ) );
}
void WheelSuspensionConstraint::setAnchorPoint( const vsg::vec3& wcPoint )
{
    _point = wcPoint;
    setDirty( true );
}
void WheelSuspensionConstraint::setAnchorPoint( const double x, const double y, const double z )
{
    setAnchorPoint( vsg::vec3( x, y, z ) );
}

bool WheelSuspensionConstraint::operator==( const WheelSuspensionConstraint& rhs ) const
{
    return( !( operator!=( rhs ) ) );
}
bool WheelSuspensionConstraint::operator!=( const WheelSuspensionConstraint& rhs ) const
{
    return(
        ( _springAxis != rhs._springAxis ) ||
        ( _axleAxis != rhs._axleAxis ) ||
        ( _linearLimit != rhs._linearLimit ) ||
        ( _angleLimit != rhs._angleLimit ) ||
        ( _point != rhs._point ) ||
        ( Constraint::operator!=( static_cast< const Constraint& >( rhs ) ) )
    );
}

void WheelSuspensionConstraint::createConstraint()
{
    if( ( _rbA == nullptr ) || ( _rbB == nullptr ) )
    {
        std::cerr << "createConstraint: _rbA == nullptr or _rbB == nullptr." << std::endl;
        return;
    }

    if( _constraint != nullptr )
    {
        delete _constraint;
        _constraint = nullptr;
    }


    // Force _axleAxis to be orthogonal to _springAxis.
    vsg::vec3 c = cross(_springAxis, _axleAxis);
    btVector3 axle = vsgbCollision::asBtVector3( cross(c, _springAxis ));

    btVector3 spring = vsgbCollision::asBtVector3( _springAxis );
    btVector3 anchor = vsgbCollision::asBtVector3( _point );

    // Everything is in world coords, just create the constraint.
    btHinge2Constraint* cons = new btHinge2Constraint( *_rbA, *_rbB, anchor, spring, axle );

    cons->setLinearLowerLimit( btVector3( 0., 0., _linearLimit[ 0 ] ) );
    cons->setLinearUpperLimit( btVector3( 0., 0., _linearLimit[ 1 ] ) );
    cons->setLowerLimit( _angleLimit[ 0 ] );
    cons->setUpperLimit( _angleLimit[ 1 ] );
    cons->setEquilibriumPoint();

    _constraint = cons;

    setDirty( false );
}



// vsgbDynamics
}
