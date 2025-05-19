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

#ifndef __VSGBDYNAMICS_CONSTRAINTS_H__
#define __VSGBDYNAMICS_CONSTRAINTS_H__ 1


#include <vsgbDynamics/Export.h>

#include <vsg/core/Object.h>
#include <vsg/core/Inherit.h>
#include <vsg/maths/mat4.h>
#include <vsg/maths/common.h>

#include <btBulletDynamicsCommon.h>
#include <BulletDynamics/ConstraintSolver/btContactConstraint.h>

#include <list>

namespace vsgbDynamics
{


/** \defgroup constraintsupport Constraint Support
\brief Convenience objects to support and enhance Bullet's constraint objects.

These objects are designed to make it easier for VSG developers to work with
Bullet constraints. The primary features are:

\li Each class is an vsg::Object with dot VSG file format support. An instance of
the constraint can be attached as UserData to a Node in your scene graph, and saved
and restored from a .vsg file.
\li Initial rigid body transforms are vsg::mat4 objects representing the initial
transform of the object (the "local to world" transform, as obtained from the Node's
NodePath).
\li The classes provide full support for collision shapes with non-origin center of
mass.

Note that the calling code is responsible for deleting the Bullet constraing.
See Constraint::~Constraint().

\test testconstraint
*/
/**@{*/

/** \class Constraint Constraints.h <vsgbDynamics/Constraints.h>
Base Constraint class with support for rigid bodies and transforms, lazy
Bullet constraint creation, constraint access, typecasting, and comparison.
*/
class VSGBDYNAMICS_EXPORT Constraint : public vsg::Inherit<vsg::Object,Constraint>
{
public:
    Constraint();
    Constraint( btRigidBody* rbA, btRigidBody* rbB=nullptr );
    Constraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
            btRigidBody* rbB=nullptr, const vsg::mat4& rbBXform=vsg::mat4() );
    Constraint( const Constraint& rhs, const vsg::CopyOp& copyop={} );

    /** \brief Get the constraint as a btTypedConstraint*.
    
    Note: This function will perform a const_cast if necessary to create the constraint
    (if it's dirty or doesn't yet exist). */
    virtual btTypedConstraint* getConstraint() const;
    /** \brief If the derived subclass uses a btConeTwistConstraint* internally, the subclass
    will override this function to return non-nullptr. The getConstraint() const_cast note also applies. */
    virtual btConeTwistConstraint* getAsBtConeTwist() const { return( nullptr ); }
    /** \brief If the derived subclass uses a btContactConstraint* internally, the subclass
    will override this function to return non-nullptr. The getConstraint() const_cast note also applies. */
    virtual btContactConstraint* getAsBtContact() const { return( nullptr ); }
    /** \brief If the derived subclass uses a btGeneric6DofConstraint* internally, the subclass
    will override this function to return non-nullptr. The getConstraint() const_cast note also applies. */
    virtual btGeneric6DofConstraint* getAsBtGeneric6Dof() const { return( nullptr ); }
    /** \brief If the derived subclass uses a btGeneric6DofSpringConstraint* internally, the subclass
    will override this function to return non-nullptr. The getConstraint() const_cast note also applies. */
    virtual btGeneric6DofSpringConstraint* getAsBtGeneric6DofSpring() const { return( nullptr ); }
    /** \brief If the derived subclass uses a btHingeConstraint* internally, the subclass
    will override this function to return non-nullptr. The getConstraint() const_cast note also applies. */
    virtual btHingeConstraint* getAsBtHinge() const { return( nullptr ); }
    /** \brief If the derived subclass uses a btHinge2Constraint* internally, the subclass
    will override this function to return non-nullptr. The getConstraint() const_cast note also applies. */
    virtual btHinge2Constraint* getAsBtHinge2() const { return( nullptr ); }
    /** \brief If the derived subclass uses a btPoint2PointConstraint* internally, the subclass
    will override this function to return non-nullptr. The getConstraint() const_cast note also applies. */
    virtual btPoint2PointConstraint* getAsBtPoint2Point() const { return( nullptr ); }
    /** \brief If the derived subclass uses a btSliderConstraint* internally, the subclass
    will override this function to return non-nullptr. The getConstraint() const_cast note also applies. */
    virtual btSliderConstraint* getAsBtSlider() const { return( nullptr ); }
    /** \brief If the derived subclass uses a btUniversalConstraint* internally, the subclass
    will override this function to return non-nullptr. The getConstraint() const_cast note also applies. */
    virtual btUniversalConstraint* getAsBtUniversal() const { return( nullptr ); }

    /** \brief Access the Bullet rigid body (or bodies).

    If only one rigid body is specified, it is constrained to the world.
    Otherwise, both rigid bodies are constrained together. */
    void setRigidBodies( btRigidBody* rbA, btRigidBody* rbB=nullptr );
    void getRigidBodies( btRigidBody*& rbA, btRigidBody*& rbB )
    {
        rbA = _rbA; rbB = _rbB;
    }

    /** \brief Specify the initial VSG transform of the subgraph corresponding to rigid body A.
    
    setAXform() dirties the Constraint, so the next call to getConstraint() will
    delete the current constraint (if it exists) and create a new one. */
    void setAXform( const vsg::mat4& rbAXform );
    vsg::mat4 getAXform() const
    {
        return( _rbAXform );
    }

    /** \brief Specify the initial VSG transform of the subgraph corresponding to rigid body B.

    setBXform() dirties the Constraint, so the next call to getConstraint() will
    delete the current constraint (if it exists) and create a new one. */
    void setBXform( const vsg::mat4& rbBXform );
    vsg::mat4 getBXform() const
    {
        return( _rbBXform );
    }

    /** \brief Set the dirty bit to indicate the constraint paramters have changed.

    This function implements a lazy constraint creation mechanism so that multiple
    parameters may be changed without deleting and re-creating the constraint multiple
    times. The getConstraint() function will delete (if necessary) and create the new
    Bullet constraint if \c _dirty is true. \c _dirty is initially true for all new
    Constraint and Constraing-derived objects. */
    void setDirty( bool dirty=true )
    {
        _dirty = dirty;
    }
    bool getDirty() const
    {
        return( _dirty );
    }

    /** Return true if both rigid body transform member variables are
    equal to the right-hand-side transforms. This function does not compare
    rigid body addresses. */
    virtual bool operator==( const Constraint& rhs ) const;
    /** Return true if either rigid body transform member variable differs
    from the right-hand-side transforms. This function does not compare
    rigid body addresses. */
    virtual bool operator!=( const Constraint& rhs ) const;

protected:
    /** \brief Destructor.
    Note that deleting the constraint is up to the calling code. For example:
    \code
    delete vsgbDynamics::Constraint::getConstraint();
    vsgbDynamics::Constraint::setDirty();
    \endcode
    */
    virtual ~Constraint();

    /** \brief Create the constraint, if it doesn't exist, or delete it and re-create it if
    the \c _dirty flag indicates that the parameters have changed.

    Note: Use of META_Object inhibits making this function pure virtual.
    Constraint::getConstraing() is a no-op, and all derived classes override this function
    to implement Bullet constraint creation. */
    virtual void createConstraint() {}

    /** Utility for supporting VSG local-to-world transforms
    that contain scales. */
    vsg::mat4 orthonormalize( const vsg::mat4& in );

    btTypedConstraint* _constraint;
    bool _dirty;

    btRigidBody* _rbA;
    btRigidBody* _rbB;
    vsg::mat4 _rbAXform;
    vsg::mat4 _rbBXform;
};
typedef std::list< vsg::ref_ptr< Constraint > > ConstraintList;


/** \class SliderConstraint Constraints.h <vsgbDynamics/Constraints.h>
\brief Creates a constraint from an axis and movement limits on that axis.

The axis is in world coordinates. The limit units are in world coordinates.
Position 0.0 along the axis refers to the initial transform of the constrained
bodies.

This class uses btSliderConstraint internally. Access the Bullet constraint
directly with getAsBtSlider(). */
class VSGBDYNAMICS_EXPORT SliderConstraint : public vsg::Inherit<Constraint, SliderConstraint>
{
public:
    SliderConstraint();
    SliderConstraint( btRigidBody* rbA, btRigidBody* rbB=nullptr );
    SliderConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
            const vsg::vec3& axis=vsg::vec3( 1., 0., 0. ),
            const vsg::vec2& slideLimit=vsg::vec2( -1., 1. ) );
    SliderConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
            btRigidBody* rbB, const vsg::mat4& rbBXform,
            const vsg::vec3& axis=vsg::vec3( 1., 0., 0. ),
            const vsg::vec2& slideLimit=vsg::vec2( -1., 1. ) );
    SliderConstraint( const SliderConstraint& rhs, const vsg::CopyOp& copyop={});

    virtual btSliderConstraint* getAsBtSlider() const;

    /** \brief Specify the slider axis in the world coordinate space.

    This is the axis along which the two constrained bodies are allowed to move
    relative to each other. In the common scenario where one rigid body is fixed,
    the other rigid body moves along this axis. If \c _rbB is nullptr, \c _rbA
    moves along this axis.
    
    The default axis is (1, 0, 0), the x axis. */
    void setAxis( const vsg::vec3& axis );
    /** \overload */
    void setAxis( const double x, const double y, const double z );
    vsg::vec3 getAxis() const
    {
        return( _axis );
    }

    /** \brief Specify movement limits along \c _axis.

    The limit values are in world coordinates and relative to the initial transforms
    \c _rbAXform and \c _rbBXform.
    
    The default limits are -1.0 to 1.0. */
    void setLimit( const vsg::vec2& limit );
    /** \overload */
    void setLimit( const double lo, const double hi );
    vsg::vec2 getLimit() const
    {
        return( _slideLimit );
    }

    /** Return true if the axis and limit member variables, and base class, are
    equal to the right-hand-side axis, limit, and base class. */
    virtual bool operator==( const SliderConstraint& rhs ) const;
    /** Return true if the axis or limit member variables, or base class, differ
    from the right-hand-side axis, limit, or base class. */
    virtual bool operator!=( const SliderConstraint& rhs ) const;

protected:
    virtual ~SliderConstraint();

    virtual void createConstraint();

    vsg::vec3 _axis;
    vsg::vec2 _slideLimit;
};


/** \class TwistSliderConstraint Constraints.h <vsgbDynamics/Constraints.h>
\brief Like SliderConstraint, but allows rotation around the axis.

The axis is in world coordinates. Calling code should also specify a world
coordinate point that the axis passes through.

There are two pairs of limit values:
\li The linear limit units are in world coordinates. Position 0.0 along the
axis refers to the initial transform of the constrained bodies.
\li The angle limit units are in radians. Radian value 0.0 refers to the initial
transform of the constrained bodies.

This class uses btSliderConstraint internally. Access the Bullet constraint
directly with getAsBtSlider(). */
class VSGBDYNAMICS_EXPORT TwistSliderConstraint : public vsg::Inherit<Constraint, TwistSliderConstraint>
{
public:
    TwistSliderConstraint();
    TwistSliderConstraint( btRigidBody* rbA, btRigidBody* rbB=nullptr );
    TwistSliderConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
            const vsg::vec3& axis=vsg::vec3( 1., 0., 0. ),
            const vsg::vec3& wcPoint=vsg::vec3( 0., 0., 0. ),
            const vsg::vec2& slideLimit=vsg::vec2( -1., 1. ),
            const vsg::vec2& twistLimit=vsg::vec2( -vsg::PIf*0.5, vsg::PIf*0.5 ) );
    TwistSliderConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
            btRigidBody* rbB, const vsg::mat4& rbBXform,
            const vsg::vec3& axis=vsg::vec3( 1., 0., 0. ),
            const vsg::vec3& wcPoint=vsg::vec3( 0., 0., 0. ),
            const vsg::vec2& slideLimit=vsg::vec2( -1., 1. ),
            const vsg::vec2& twistLimit=vsg::vec2( -vsg::PIf*.5f, vsg::PIf*.5f ) );
    TwistSliderConstraint( const TwistSliderConstraint& rhs, const vsg::CopyOp& copyop={});

    virtual btSliderConstraint* getAsBtSlider() const;

    /** \brief Specify the slider axis in the world coordinate space.

    This is the axis along which the two constrained bodies are allowed to move
    relative to each other. In the common scenario where one rigid body is fixed,
    the other rigid body moves along this axis. If \c _rbB is nullptr, \c _rbA
    moves along this axis.
    
    The default axis is (1, 0, 0), the x axis. */
    void setAxis( const vsg::vec3& axis );
    /** \overload */
    void setAxis( const double x, const double y, const double z );
    vsg::vec3 getAxis() const
    {
        return( _axis );
    }
    /** \brief Specify a world coordinate point on the _axis.

    Together, \c _axis and \c wcPoint define the axis in world coordinates that
    the constraint uses for rotation of the constrained bodies.
    
    The default point is (0, 0, 0), the origin. */
    void setPoint( const vsg::vec3& wcPoint );
    /** \overload */
    void setPoint( const double x, const double y, const double z );
    vsg::vec3 getPoint() const
    {
        return( _point );
    }

    /** \brief Specify sliding limits along \c _axis.

    The limit values are in world coordinates and relative to the initial transforms
    \c _rbAXform and \c _rbBXform.
    
    The default slide limits are -1.0 to 1.0. */
    void setSlideLimit( const vsg::vec2& limit );
    /** \overload */
    void setSlideLimit( const double lo, const double hi );
    vsg::vec2 getSlideLimit() const
    {
        return( _slideLimit );
    }
    /** \brief Specify rotational limits around \c _axis.

    The limit values are in radians and relative to the initial transforms
    \c _rbAXform and \c _rbBXform.
    
    The default twist limits are -PI/2 to PI/2. */
    void setTwistLimit( const vsg::vec2& limitRadians );
    /** \overload */
    void setTwistLimit( const double lo, const double hi );
    vsg::vec2 getTwistLimit() const
    {
        return( _twistLimit );
    }

    /** Return true if the axis, point, limits, and base class, are
    equal to the right-hand-side axis, point, limits, and base class. */
    virtual bool operator==( const TwistSliderConstraint& rhs ) const;
    /** Return true if the axis, point, limits, or base class, differ
    from the right-hand-side axis, point, limits, or base class. */
    virtual bool operator!=( const TwistSliderConstraint& rhs ) const;

protected:
    virtual ~TwistSliderConstraint();

    virtual void createConstraint();

    vsg::vec3 _axis;
    vsg::vec3 _point;
    vsg::vec2 _slideLimit;
    vsg::vec2 _twistLimit;
};


/** \class InternalSpringData Constraints.h <vsgbDynamics/Constraints.h>
\brief For internal storage of Bullet spring constraint parameters.

All three vsgBullet spring-type constraints (LinearSpringConstraint,
AngleSpringConstraint, and LinearAngleSpringConstraint) use the Bullet
btGeneric6DofSpringConstraint internally, but each configures the
Bullet constraint differently. InternalSpringData stores the parameters
for btGeneric6DofSpringConstraint, and each of the vsgBullet spring
constraints sets different parameter values to configure the Bullet
constraint accordingly. */
struct VSGBDYNAMICS_EXPORT InternalSpringData : public vsg::Inherit<vsg::Object,InternalSpringData>
{
    InternalSpringData();
    InternalSpringData( const InternalSpringData& rhs, const vsg::CopyOp& copyop={});

    /** Configure the Bullet constraint \c cons with this
    struct's stored parameter values. */
    void apply( btGeneric6DofSpringConstraint* cons ) const;

    /** Return true if all member variables are equal to their equivalent
    right-hand-side member variables. */
    bool operator==( const InternalSpringData& rhs ) const;
    /** Return true if any member variable differs
    from its equivalent right-hand-side member variable. */
    bool operator!=( const InternalSpringData& rhs ) const;

    vsg::vec3 _linearLowerLimits;
    vsg::vec3 _linearUpperLimits;
    vsg::vec3 _angularLowerLimits;
    vsg::vec3 _angularUpperLimits;
    bool _enable[ 6 ];
    btScalar _stiffness[ 6 ];
    btScalar _damping[ 6 ];
};


/** \class LinearSpringConstraint Constraints.h <vsgbDynamics/Constraints.h>
\brief An axis-constrained spring.

The axis is in world coordinates. The limit units are in world coordinates.
Position 0.0 along the axis refers to the initial transform of the constrained
bodies.

This class uses btGeneric6DofSpringConstraint internally. Access the Bullet constraint
directly with getAsBtGeneric6DofSpring().

The btGeneric6DofSpringConstraint parameters are stored in an InternalSpringData struct.
All vsgBullet spring-like constraints share a common function for configuring the
Bullet btGeneric6DofSpringConstraint,
LinearSpringConstraint::internalCreateSpringConstraint(). This is declared private,
but access to other spring constraints is allowed using the friend declarative.

As with all btGeneric6DofSpringConstraint constraints, and unlike most other
Bullet constraints, calling code must specify two rigid bodies. Constraining
one rigid body to a sping is not supported by Bullet. */
class VSGBDYNAMICS_EXPORT LinearSpringConstraint : public vsg::Inherit<Constraint,LinearSpringConstraint>
{
public:
    LinearSpringConstraint();
    LinearSpringConstraint( btRigidBody* rbA, btRigidBody* rbB );
    LinearSpringConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
            btRigidBody* rbB, const vsg::mat4& rbBXform,
            const vsg::vec3& axis=vsg::vec3(1., 0., 0.) );
    LinearSpringConstraint( const LinearSpringConstraint& rhs, const vsg::CopyOp& copyop={});

    virtual btGeneric6DofSpringConstraint* getAsBtGeneric6DofSpring() const;

    /** \brief Storage of spring constraint parameters.

    This is essentially for internal use by the three spring-type constraints,
    primarily for dot VSG support. The InternalSpringData struct is not intended
    to be shared by multiple instances of spring constraints. */
    void setSpringData( InternalSpringData* data );
    const InternalSpringData* getSpringData() const
    {
        return( _data.get() );
    }
    /** \vrief Specifies the spring's linear axis.
    
    The axis is in world coordinates. The default is (1,0,0), the x axis. */
    void setAxis( const vsg::vec3& axis );
    /** \overload */
    void setAxis( const double x, const double y, const double z );
    vsg::vec3 getAxis() const
    {
        return( _axis );
    }
    /** \brief Specify limits along the spring's axis.
    
    The default limits are -1 to 1. */
    void setLimit( const vsg::vec2& limit );
    /** \overload */
    void setLimit( const double lo, const double hi );
    vsg::vec2 getLimit() const
    {
        return( vsg::vec2( _data->_linearLowerLimits[ 0 ], _data->_linearUpperLimits[ 0 ] ) );
    }
    /** \brief Specify the spring stiffness.
    
    Larger values result in a stiffer spring. Default is 10. */
    void setStiffness( float stiffness );
    float getStiffness() const
    {
        return( (float)( _data->_stiffness[ 0 ] ) );
    }
    /** \brief Specify the spring damping.
    
    Larger values result in less damping. Default is .1 */
    void setDamping( float damping );
    float getDamping() const
    {
        return( (float)( _data->_damping[ 0 ] ) );
    }

    /** Return true if the axis and data member variables, and base class, are
    equal to the right-hand-side axis, data, and base class. */
    virtual bool operator==( const LinearSpringConstraint& rhs ) const;
    /** Return true if the axis and data member variables, or base class, differ
    from the right-hand-side axis, data, or base class. */
    virtual bool operator!=( const LinearSpringConstraint& rhs ) const;

protected:
    virtual ~LinearSpringConstraint();

    virtual void createConstraint();

    vsg::vec3 _axis;
    vsg::ref_ptr< InternalSpringData > _data;

private:
    /** \brief Shared Bullet constraint construction for Linear, Angle, and
    LinearAngle spring constraints. Access is allowed via "friend" declarative.
    
    Angle and LinearAngle pass the pivot point (rotation requires a point and
    an axis that passes through it). The Linear spring doesn't need a point (linear
    spring acts only on an axis and behaves the same regardless of the spatial
    location of the axis). The default value of the \c point parameter supports
    the Linear spring usage. */
    static btGeneric6DofSpringConstraint* internalCreateSpringConstraint(
        Constraint* cons, const InternalSpringData* isd,
        const vsg::vec3& axis, const vsg::vec3& point=vsg::vec3(0.,0.,0.) );

    friend class AngleSpringConstraint;
    friend class LinearAngleSpringConstraint;
};


/** \class AngleSpringConstraint Constraints.h <vsgbDynamics/Constraints.h>
\brief An angle spring for rotating around an axis.

The axis is in world coordinates. Calling code must also specify a point in
space that the axis passes through; this point is also in world coordinates.
The limit units are in radians. Radian value 0.0 refers to the initial transform
of the constrained bodies.

This class uses btGeneric6DofSpringConstraint internally. Access the Bullet constraint
directly with getAsBtGeneric6DofSpring().

The btGeneric6DofSpringConstraint parameters are stored in an InternalSpringData struct.
All vsgBullet spring-like constraints share a common function for configuring the
Bullet btGeneric6DofSpringConstraint,
LinearSpringConstraint::internalCreateSpringConstraint(). This is declared private,
but access to other spring constraints is allowed using the friend declarative.

As with all btGeneric6DofSpringConstraint constraints, and unlike most other
Bullet constraints, calling code must specify two rigid bodies. Constraining
one rigid body to a sping is not supported by Bullet. */
class VSGBDYNAMICS_EXPORT AngleSpringConstraint : public vsg::Inherit<Constraint,AngleSpringConstraint>
{
public:
    AngleSpringConstraint();
    AngleSpringConstraint( btRigidBody* rbA, btRigidBody* rbB );
    AngleSpringConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
            btRigidBody* rbB, const vsg::mat4& rbBXform,
            const vsg::vec3& axis=vsg::vec3( 1., 0., 0. ), const vsg::vec3& point=vsg::vec3( 0., 0., 0. ) );
    AngleSpringConstraint( const AngleSpringConstraint& rhs, const vsg::CopyOp& copyop={} );

    virtual btGeneric6DofSpringConstraint* getAsBtGeneric6DofSpring() const;

    /** \brief Storage of spring constraint parameters.

    This is essentially for internal use by the three spring-type constraints,
    primarily for dot VSG support. The InternalSpringData struct is not intended
    to be shared by multiple instances of spring constraints. */
    void setSpringData( InternalSpringData* data );
    const InternalSpringData* getSpringData() const
    {
        return( _data.get() );
    }
    /** \brief Specify the angle spring rotational axis.

    The axis is in world coordinates. The default is (1,0,0), the x axis. */
    void setAxis( const vsg::vec3& axis );
    /** \overload */
    void setAxis( const double x, const double y, const double z );
    vsg::vec3 getAxis() const
    {
        return( _axis );
    }
    /** \brief Specify the world coordinate point that the axis passes through.

    Default is (0., 0., 0.), the origin. */
    void setPivotPoint( const vsg::vec3& wcPoint );
    /** \overload */
    void setPivotPoint( const double x, const double y, const double z );
    vsg::vec3 getPivotPoint() const
    {
        return( _pivotPoint );
    }
    /** \brief Specify the angle spring's limits In radians.
    
    The default is -PI/2 to PI/2 (180 degrees). */
    void setLimit( const vsg::vec2& limit );
    /** \overload */
    void setLimit( const double lo, const double hi );
    vsg::vec2 getLimit() const
    {
        return( vsg::vec2( _data->_angularLowerLimits[ 0 ], _data->_angularUpperLimits[ 0 ] ) );
    }
    /** \brief Specify the spring stiffness.
    
    Larger values result in a stiffer spring. Default is 10. */
    void setStiffness( float stiffness );
    float getStiffness() const
    {
        return( (float)( _data->_stiffness[ 3 ] ) );
    }
    /** \brief Specify the spring damping.
    
    Larger values result in less damping. Default is .1 */
    void setDamping( float damping );
    float getDamping() const
    {
        return( (float)( _data->_damping[ 3 ] ) );
    }

    /** Return true if the axis and data member variables, and base class, are
    equal to the right-hand-side axis, data, and base class. */
    virtual bool operator==( const AngleSpringConstraint& rhs ) const;
    /** Return true if the axis and data member variables, or base class, differ
    from the right-hand-side axis, data, or base class. */
    virtual bool operator!=( const AngleSpringConstraint& rhs ) const;

protected:
    virtual ~AngleSpringConstraint();

    virtual void createConstraint();

    vsg::vec3 _axis;
    vsg::vec3 _pivotPoint;
    vsg::ref_ptr< InternalSpringData > _data;
};


/** \class LinearAngleSpringConstraint Constraints.h <vsgbDynamics/Constraints.h>
\brief TBD

The axis is in world coordinates. Calling code must also specify a point in
space that the axis passes through; this point is also in world coordinates.

There are two pairs of limit values:
\li The linear limit units are in world coordinates. Position 0.0 along the
axis refers to the initial transform of the constrained bodies.
\li The angle limit units are in radians. Radian value 0.0 refers to the initial
transform of the constrained bodies.

This class uses btGeneric6DofSpringConstraint internally. Access the Bullet constraint
directly with getAsBtGeneric6DofSpring().

The btGeneric6DofSpringConstraint parameters are stored in an InternalSpringData struct.
All vsgBullet spring-like constraints share a common function for configuring the
Bullet btGeneric6DofSpringConstraint,
LinearSpringConstraint::internalCreateSpringConstraint(). This is declared private,
but access to other spring constraints is allowed using the friend declarative.

As with all btGeneric6DofSpringConstraint constraints, and unlike most other
Bullet constraints, calling code must specify two rigid bodies. Constraining
one rigid body to a sping is not supported by Bullet. */
class VSGBDYNAMICS_EXPORT LinearAngleSpringConstraint : public vsg::Inherit<Constraint,LinearAngleSpringConstraint>
{
public:
    LinearAngleSpringConstraint();
    LinearAngleSpringConstraint( btRigidBody* rbA, btRigidBody* rbB );
    LinearAngleSpringConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
            btRigidBody* rbB, const vsg::mat4& rbBXform,
            const vsg::vec3& axis=vsg::vec3( 1., 0., 0. ), const vsg::vec3& point=vsg::vec3( 0., 0., 0. ) );
    LinearAngleSpringConstraint( const LinearAngleSpringConstraint& rhs, const vsg::CopyOp& copyop={} );

    virtual btGeneric6DofSpringConstraint* getAsBtGeneric6DofSpring() const;

    /** \brief Storage of spring constraint parameters.

    This is essentially for internal use by the three spring-type constraints,
    primarily for dot VSG support. The InternalSpringData struct is not intended
    to be shared by multiple instances of spring constraints. */
    void setSpringData( InternalSpringData* data );
    const InternalSpringData* getSpringData() const
    {
        return( _data.get() );
    }
    /** \brief Specify the axis for both the linear and angle components.

    The axis is in world coordinates. The default is (1,0,0), the x axis. */
    void setAxis( const vsg::vec3& axis );
    /** \overload */
    void setAxis( const double x, const double y, const double z );
    vsg::vec3 getAxis() const
    {
        return( _axis );
    }
    /** \brief Specify the world coordinate point that the axis passes through.

    Default is (0., 0., 0.), the origin. */
    void setPivotPoint( const vsg::vec3& wcPoint );
    /** \overload */
    void setPivotPoint( const double x, const double y, const double z );
    vsg::vec3 getPivotPoint() const
    {
        return( _pivotPoint );
    }
    /** \brief Specify linear limits along the spring's axis.
    
    The default limits are -1 to 1. */
    void setLinearLimit( const vsg::vec2& limit );
    /** \overload */
    void setLinearLimit( const double lo, const double hi );
    vsg::vec2 getLinearLimit() const
    {
        return( vsg::vec2( _data->_linearLowerLimits[ 0 ], _data->_linearUpperLimits[ 0 ] ) );
    }
    /** \brief Specify rotational limits around the axis.
    
    The default is -PI/2 to PI/2 (180 degrees). */
    void setAngleLimit( const vsg::vec2& limit );
    /** \overload */
    void setAngleLimit( const double lo, const double hi );
    vsg::vec2 getAngleLimit() const
    {
        return( vsg::vec2( _data->_angularLowerLimits[ 0 ], _data->_angularUpperLimits[ 0 ] ) );
    }
    /** \brief Specify the stiffness of the linear spring component.
    
    Larger values result in a stiffer spring. Default is 10. */
    void setLinearStiffness( float stiffness );
    float getLinearStiffness() const
    {
        return( (float)( _data->_stiffness[ 0 ] ) );
    }
    /** \brief Specify the stiffness of the angle spring component.
    
    Larger values result in a stiffer spring. Default is 10. */
    void setAngleStiffness( float stiffness );
    float getAngleStiffness() const
    {
        return( (float)( _data->_stiffness[ 3 ] ) );
    }
    /** \brief Specify the damping of the linear spring component.

    Larger values result in less damping. Default is .1 */
    void setLinearDamping( float damping );
    float getLinearDamping() const
    {
        return( (float)( _data->_damping[ 0 ] ) );
    }
    /** \brief Specify the damping of the angle spring component.

    Larger values result in less damping. Default is .1 */
    void setAngleDamping( float damping );
    float getAngleDamping() const
    {
        return( (float)( _data->_damping[ 3 ] ) );
    }

    /** Return true if the axis and data member variables, and base class, are
    equal to the right-hand-side axis, data, and base class. */
    virtual bool operator==( const LinearAngleSpringConstraint& rhs ) const;
    /** Return true if the axis and data member variables, or base class, differ
    from the right-hand-side axis, data, or base class. */
    virtual bool operator!=( const LinearAngleSpringConstraint& rhs ) const;

protected:
    virtual ~LinearAngleSpringConstraint();

    virtual void createConstraint();

    vsg::vec3 _axis;
    vsg::vec3 _pivotPoint;
    vsg::ref_ptr< InternalSpringData > _data;
};


/** \class FixedConstraint Constraints.h <vsgbDynamics/Constraints.h>
\brief A constraint that prohibits all relative transformation.

This class uses btGeneric6DofConstraint internally. Access the Bullet constraint
directly with getAsBtGeneric6Dof().
*/
class VSGBDYNAMICS_EXPORT FixedConstraint : public vsg::Inherit<Constraint,FixedConstraint>
{
public:
    FixedConstraint();
    FixedConstraint( btRigidBody* rbA, btRigidBody* rbB=nullptr );
    FixedConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
            btRigidBody* rbB=nullptr, const vsg::mat4& rbBXform=vsg::mat4() );
    FixedConstraint( const FixedConstraint& rhs, const vsg::CopyOp& copyop={} );

    virtual btGeneric6DofConstraint* getAsBtGeneric6Dof() const;

protected:
    virtual ~FixedConstraint();

    virtual void createConstraint();
};


/** \class PlanarConstraint Constraints.h <vsgbDynamics/Constraints.h>
\brief Allows bodies to move relative to each other in a plane.

The user can specify the orientation (\c _orient) of the plane using setOrient(). The \c _orient
matrix determines the plane orientation based on the following pseudocode:

\code
  if _orient is identity
    the plane is in B's coord space.
    if B is nullptr
      the plane is in the world coord space.
  else
    the plane coord space is B's coord space, multiplied
            by _orient.
    if B is nullptr
      the plane coord space is defined solely by _orient.
\endcode
"Coord space" above refers to orientation only. The origin is always based
on the initial local-to-world transform(s).

This class uses btGeneric6DofConstraint internally. Access the Bullet constraint
directly with getAsBtGeneric6Dof().

Configuration of the internal btGeneric6DofConstraint is nearly identical between
PlanarConstraint and BoxConstraint, so the two classes use a common function,
BoxConstraint::internalPlanarBoxFrameComputation(). Access to PlanarConstraint
is allowed via the friend declarative. */
class VSGBDYNAMICS_EXPORT PlanarConstraint : public vsg::Inherit<Constraint,PlanarConstraint>
{
public:
    PlanarConstraint();
    PlanarConstraint( btRigidBody* rbA, btRigidBody* rbB=nullptr,
            const vsg::vec2& loLimit=vsg::vec2(0.,0.),
            const vsg::vec2& hiLimit=vsg::vec2(0.,0.),
            const vsg::mat4& orient=vsg::mat4() );
    PlanarConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
            const vsg::vec2& loLimit=vsg::vec2(0.,0.),
            const vsg::vec2& hiLimit=vsg::vec2(0.,0.),
            const vsg::mat4& orient=vsg::mat4() );
    PlanarConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
            btRigidBody* rbB=nullptr, const vsg::mat4& rbBXform=vsg::mat4(),
            const vsg::vec2& loLimit=vsg::vec2(0.,0.),
            const vsg::vec2& hiLimit=vsg::vec2(0.,0.),
            const vsg::mat4& orient=vsg::mat4() );
    PlanarConstraint( const PlanarConstraint& rhs, const vsg::CopyOp& copyop={} );

    virtual btGeneric6DofConstraint* getAsBtGeneric6Dof() const;

    /** \brief Specify the lower limits for the plane constraint.

    Default is 0 in x and 0 in y. */
    void setLowLimit( const vsg::vec2& loLimit );
    /** \overload */
    void setLowLimit( const double x, const double y );
    vsg::vec2 getLowLimit() const
    {
        return( _loLimit );
    }

    /** \brief Specify the upper limits for the plane constraint.

    Default is 0 in x and 0 in y. */
    void setHighLimit( const vsg::vec2& hiLimit );
    /** \overload */
    void setHighLimit( const double x, const double y );
    vsg::vec2 getHighLimit() const
    {
        return( _hiLimit );
    }

    /** \brief Specify the orienation of the constrained axes.

    Note that this class ignores any translation component in \c orient.
    */
    void setOrientation( const vsg::mat4& orient );
    vsg::mat4 getOrientation() const
    {
        return( _orient );
    }

    /** Return true if the lower and upper limit member variables, and base class, are
    equal to the right-hand-side lower and upper limits and base class. */
    virtual bool operator==( const PlanarConstraint& rhs ) const;
    /** Return true if the lower and upper limit member variables, or base class, differ
    from the right-hand-side lower and upper limits or base class. */
    virtual bool operator!=( const PlanarConstraint& rhs ) const;

protected:
    virtual ~PlanarConstraint();

    virtual void createConstraint();

    vsg::vec2 _loLimit, _hiLimit;
    vsg::mat4 _orient;
};


/** \class BoxConstraint Constraints.h <vsgbDynamics/Constraints.h>
\brief Allows translation along three axes, but doesn't allow any rotation.

Translation is allowed withing a "box" defined by setLowLimit() and
setHighLimit().

Orient the box axes using setOrient(). The resulting orientation is described
in the documentation for PlanarConstraint.

This class uses btGeneric6DofConstraint internally. Access the Bullet constraint
directly with getAsBtGeneric6Dof().

Configuration of the internal btGeneric6DofConstraint is nearly identical between
PlanarConstraint and BoxConstraint, so the two classes use a common function,
BoxConstraint::internalPlanarBoxFrameComputation(). Access to PlanarConstraint
is allowed via the friend declarative. */
class VSGBDYNAMICS_EXPORT BoxConstraint : public vsg::Inherit<Constraint,BoxConstraint>
{
public:
    BoxConstraint();
    BoxConstraint( btRigidBody* rbA, btRigidBody* rbB=nullptr,
            const vsg::vec3& loLimit=vsg::vec3(0.,0.,0.),
            const vsg::vec3& hiLimit=vsg::vec3(0.,0.,0.),
            const vsg::mat4& orient=vsg::mat4() );
    BoxConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
            btRigidBody* rbB=nullptr, const vsg::mat4& rbBXform=vsg::mat4(),
            const vsg::vec3& loLimit=vsg::vec3(0.,0.,0.),
            const vsg::vec3& hiLimit=vsg::vec3(0.,0.,0.),
            const vsg::mat4& orient=vsg::mat4() );
    BoxConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
            const vsg::vec3& loLimit=vsg::vec3(0.,0.,0.),
            const vsg::vec3& hiLimit=vsg::vec3(0.,0.,0.),
            const vsg::mat4& orient=vsg::mat4() );
    BoxConstraint( const BoxConstraint& rhs, const vsg::CopyOp& copyop={} );

    virtual btGeneric6DofConstraint* getAsBtGeneric6Dof() const;

    /** \brief Specify the lower limits for the box constraint.

    Default is 0 in x, y, and z. */
    void setLowLimit( const vsg::vec3& loLimit );
    /** \overload */
    void setLowLimit( const double x, const double y, const double z );
    vsg::vec3 getLowLimit() const
    {
        return( _loLimit );
    }
    /** \brief Specify the upper limits for the box constraint.

    Default is 0 in x, y, and z. */
    void setHighLimit( const vsg::vec3& hiLimit );
    /** \overload */
    void setHighLimit( const double x, const double y, const double z );
    vsg::vec3 getHighLimit() const
    {
        return( _hiLimit );
    }

    /** \brief Specify the orienation of the constrained axes.

    Note that this class ignores any translation component in \c orient.
    */
    void setOrientation( const vsg::mat4& orient );
    vsg::mat4 getOrientation() const
    {
        return( _orient );
    }

    /** Return true if the lower and upper limit member variables, and base class, are
    equal to the right-hand-side lower and upper limits and base class. */
    virtual bool operator==( const BoxConstraint& rhs ) const;
    /** Return true if the lower and upper limit member variables, or base class, differ
    from the right-hand-side lower and upper limits or base class. */
    virtual bool operator!=( const BoxConstraint& rhs ) const;

protected:
    virtual ~BoxConstraint();

    virtual void createConstraint();

    vsg::vec3 _loLimit, _hiLimit;
    vsg::mat4 _orient;

private:
    /** \brief Shared reference frame computation code for both Planar and
    Box constraints. Access to Planar is allowed via "friend" declarative. */
    static void internalPlanarBoxFrameComputation(
        btTransform& aFrame, btTransform& bFrame,
        Constraint* cons, const vsg::mat4& orient );
    friend class PlanarConstraint;
};


/** \class HingeConstraint Constraints.h <vsgbDynamics/Constraints.h>
\brief Limits rotation around a single axis. Allows no other rotation, and
no translation.

The axis and pivot points are in world coordinates. The hinge limits
are in radians, with rotation limit 0.0 corresponding to the initial transform
of the constrained body or bodies.

This class uses btHingeConstraint internally. Access the Bullet constraint
directly with getAsBtHinge().
*/
class VSGBDYNAMICS_EXPORT HingeConstraint : public vsg::Inherit<Constraint,HingeConstraint>
{
public:
    HingeConstraint();
    HingeConstraint( btRigidBody* rbA, btRigidBody* rbB=nullptr,
            const vsg::vec3& axis=vsg::vec3(0.,0.,1.),
            const vsg::vec3& pivotPoint=vsg::vec3(0.,0.,0.),
            const vsg::vec2& limit=vsg::vec2(-vsg::PI,vsg::PI) );
    HingeConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
            btRigidBody* rbB=nullptr, const vsg::mat4& rbBXform=vsg::mat4(),
            const vsg::vec3& axis=vsg::vec3(0.,0.,1.),
            const vsg::vec3& pivotPoint=vsg::vec3(0.,0.,0.),
            const vsg::vec2& limit=vsg::vec2(-vsg::PI,vsg::PI) );
    HingeConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
            const vsg::vec3& axis=vsg::vec3(0.,0.,1.),
            const vsg::vec3& pivotPoint=vsg::vec3(0.,0.,0.),
            const vsg::vec2& limit=vsg::vec2(-vsg::PI,vsg::PI) );
    HingeConstraint( const HingeConstraint& rhs, const vsg::CopyOp& copyop={});

    virtual btHingeConstraint* getAsBtHinge() const;

    /** \brief Hinge axis in world coordinates.
    
    The default axis is (0,0,1), the z axis. */
    void setAxis( const vsg::vec3& axis );
    /** \overload */
    void setAxis( const double x, const double y, const double z );
    vsg::vec3 getAxis() const
    {
        return( _axis );
    }

    /** \brief Hinge pivot point in world coordinates.

    The default pivot point is (0,0,0), the origin. */
    void setPivotPoint( const vsg::vec3& wcPoint );
    /** \overload */
    void setPivotPoint( const double x, const double y, const double z );
    vsg::vec3 getPivotPoint() const
    {
        return( _pivotPoint );
    }

    /** \brief Hinge rotation limits, with 0.0 corresponding to the
    initial position of the constrained body/bodies.

    Pass (-vsg::PI_2, vsg::PI_2) to allow free rotation. */
    void setLimit( const vsg::vec2& limit );
    /** \overload */
    void setLimit( const double lo, const double hi );
    vsg::vec2 getLimit() const
    {
        return( _limit );
    }

    /** Return true if the axis, pivot point, and limit member variables, and base class, are
    equal to the right-hand-side axis, pivot point, limit, and base class. */
    virtual bool operator==( const HingeConstraint& rhs ) const;
    /** Return true if the axis, pivot point, or limit member variables, or base class, differ
    from the right-hand-side axis, pivot point, limit, or base class. */
    virtual bool operator!=( const HingeConstraint& rhs ) const;

protected:
    virtual ~HingeConstraint();

    virtual void createConstraint();

    vsg::vec3 _axis;
    vsg::vec3 _pivotPoint;
    vsg::vec2 _limit;
};


/** \class CardanConstraint Constraints.h <vsgbDynamics/Constraints.h>
\brief Implements a universal or Cardan constraint, useful for transferring torque
from one body to another.

Like the spring constraints, and unlike most Bullet constraints,
CardanConstraint requires two rigid bodies. This is a requirement imposed
by Bullet.

The axes and anchor point are in world coordinates. \c _axisA is the
rotational axis for \c _rbA, and \c _axisB is the rotational axis for
\c _rbB. The anchor point (\c _point) is the world coordinate point common
to both axes.

This class uses btUniversalConstraint internally. Access the Bullet constraint
directly with getAsBtUniversal().
*/
class VSGBDYNAMICS_EXPORT CardanConstraint : public vsg::Inherit<Constraint,CardanConstraint>
{
public:
    CardanConstraint();
    CardanConstraint( btRigidBody* rbA, btRigidBody* rbB );
    CardanConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
            btRigidBody* rbB, const vsg::mat4& rbBXform,
            const vsg::vec3& axisA=vsg::vec3(0.,1.,0.),
            const vsg::vec3& axisB=vsg::vec3(1.,0.,0.),
            const vsg::vec3& point=vsg::vec3(0.,0.,0.) );
    CardanConstraint( const CardanConstraint& rhs, const vsg::CopyOp& copyop={} );

    virtual btUniversalConstraint* getAsBtUniversal() const;

    /** \brief Specify rigid body A's rotational axis in world coords.

    Default is ( 0., 1., 0. ), the y axis. */
    void setAxisA( const vsg::vec3& axisA );
    /** \overload */
    void setAxisA( const double x, const double y, const double z );
    vsg::vec3 getAxisA() const
    {
        return( _axisA );
    }
    /** \brief Specify rigid body B's rotational axis in world coords.

    Default is ( 1., 0., 0. ), the x axis. */
    void setAxisB( const vsg::vec3& axisB );
    /** \overload */
    void setAxisB( const double x, const double y, const double z );
    vsg::vec3 getAxisB() const
    {
        return( _axisB );
    }
    /** \brief Specify the world coordinate anchor point.

    Default is ( 0., 0., 0. ), the origin. */
    void setAnchorPoint( const vsg::vec3& wcPoint );
    /** \overload */
    void setAnchorPoint( const double x, const double y, const double z );
    vsg::vec3 getAnchorPoint() const
    {
        return( _point );
    }

    /** Return true if the axes and point member variables, and base class, are
    equal to the right-hand-side axes and base class. */
    virtual bool operator==( const CardanConstraint& rhs ) const;
    /** Return true if the axes or point member variables, or base class, differ
    from the right-hand-side axes or base class. */
    virtual bool operator!=( const CardanConstraint& rhs ) const;

protected:
    virtual ~CardanConstraint();

    virtual void createConstraint();

    vsg::vec3 _axisA;
    vsg::vec3 _axisB;
    vsg::vec3 _point;
};


/** \class BallAndSocketConstraint Constraints.h <vsgbDynamics/Constraints.h>
\brief Constrains two rigid bodies at the same world coordinate point.

This class uses btPoint2PointConstraint internally. Access the Bullet constraint
directly with getAsBtPoint2Point().
*/
class VSGBDYNAMICS_EXPORT BallAndSocketConstraint : public vsg::Inherit<Constraint,BallAndSocketConstraint>
{
public:
    BallAndSocketConstraint();
    BallAndSocketConstraint( btRigidBody* rbA, btRigidBody* rbB=nullptr );
    BallAndSocketConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
            const vsg::vec3& wcPoint=vsg::vec3(0.,0.,0.) );
    BallAndSocketConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
            btRigidBody* rbB, const vsg::mat4& rbBXform,
            const vsg::vec3& wcPoint=vsg::vec3(0.,0.,0.) );
    BallAndSocketConstraint( const BallAndSocketConstraint& rhs, const vsg::CopyOp& copyop={} );

    virtual btPoint2PointConstraint* getAsBtPoint2Point() const;

    /** \brief Specify the common point in world coordinates.

    The rbA and rbB transforms are used to convert this point into each body's local
    coordinates, which are then passed into the btPoint2PointConstraint constructor. */
    void setPoint( const vsg::vec3& wcPoint );
    /** \overload */
    void setPoint( const double x, const double y, const double z );
    vsg::vec3 getPoint() const
    {
        return( _point );
    }

    /** Return true if the point member variables and base class are
    equal to the right-hand-side point and base class. */
    virtual bool operator==( const BallAndSocketConstraint& rhs ) const;
    /** Return true if the point member variable or base class differ
    from the right-hand-side point or base class. */
    virtual bool operator!=( const BallAndSocketConstraint& rhs ) const;

protected:
    virtual ~BallAndSocketConstraint();

    virtual void createConstraint();

    vsg::vec3 _point;
};


/** \class RagdollConstraint Constraints.h <vsgbDynamics/Constraints.h>
\brief Like BallAndSocketConstraint, but limits movement to within a cone around
a specified axis.

Like BallAndSocketConstraint, RagdollConstraint takes a world coordinate point
that is common between the two constrained bodies. Use RagdollConstraint::setAngle()
to specify the spread angle of the cone in radians. RagdollConstraint::setAxis()
specifies the center of the cone.

This class uses btConeTwistConstraint internally. Access the Bullet constraint
directly with getAsBtConeTwist().

NOTE: Currently there appears to be an issue if only one rigid body is specified.
This could be an vsgBullet issue, or an issue with Bullet btConeTwistConstraint.
Need to investigate. */
class VSGBDYNAMICS_EXPORT RagdollConstraint : public vsg::Inherit<Constraint,RagdollConstraint>
{
public:
    RagdollConstraint();
    RagdollConstraint( btRigidBody* rbA, btRigidBody* rbB=nullptr );
    RagdollConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
            const vsg::vec3& wcPoint=vsg::vec3(0.,0.,0),
            const vsg::vec3& wcAxis=vsg::vec3(1.,0.,0),
            const double angleRadians=0. );
    RagdollConstraint( btRigidBody* rbA, const vsg::mat4& rbAXform,
            btRigidBody* rbB, const vsg::mat4& rbBXform,
            const vsg::vec3& wcPoint=vsg::vec3(0.,0.,0),
            const vsg::vec3& wcAxis=vsg::vec3(1.,0.,0),
            const double angleRadians=0. );
    RagdollConstraint( const RagdollConstraint& rhs, const vsg::CopyOp& copyop={} );

    virtual btConeTwistConstraint* getAsBtConeTwist() const;

    /** \brief Specify the common point in world coordinates.

    The default is (0,0,0), the origin. */
    void setPoint( const vsg::vec3& wcPoint );
    /** \overload */
    void setPoint( const double x, const double y, const double z );
    vsg::vec3 getPoint() const
    {
        return( _point );
    }
    /** \brief Specify an axis corresponding to the center of the cone.

    The default is (1,0,0), the x axis. */
    void setAxis( const vsg::vec3& wcAxis );
    /** \overload */
    void setAxis( const double x, const double y, const double z );
    vsg::vec3 getAxis() const
    {
        return( _axis );
    }
    /** \brief Specify the cone spread angle in radians.

    The default is PI/2 (90 degrees). */
    void setAngle( const double angleRadians );
    double getAngle() const
    {
        return( _angle );
    }

    /** Return true if the point, axis, and angle member variables, and base class, are
    equal to the right-hand-side point, axis, angle, and base class. */
    virtual bool operator==( const RagdollConstraint& rhs ) const;
    /** Return true if the point, axis, or angle member variables, or base class, differ
    from the right-hand-side point, axis, angle, or base class. */
    virtual bool operator!=( const RagdollConstraint& rhs ) const;

protected:
    virtual ~RagdollConstraint();

    virtual void createConstraint();

    vsg::vec3 _point;
    vsg::vec3 _axis;
    double _angle;
};


/** \class WheelSuspensionConstraint Constraints.h <vsgbDynamics/Constraints.h>
\brief Model of a wheel constrained to a vehicle.

The constraint allows up and down spring-like motion along the \x _springAxis.
Rotation around that axis is controlled with setLimit(), and the default is the
range -pi/4 to pi/4 radians. The constraint also allows rotation around the
\c _axleAxis. Both axes pass through \c _point, the center of rotation.

This constraint class, along with the spring constraints, require two rigid
bodies. This restriction is imposed by the underlying Bullet support classes.

This class uses btHinge2Constraint internally. Access the Bullet constraint
directly with getAsBtHinge2().
*/
class VSGBDYNAMICS_EXPORT WheelSuspensionConstraint : public vsg::Inherit<Constraint,WheelSuspensionConstraint>
{
public:
    WheelSuspensionConstraint();
    WheelSuspensionConstraint( btRigidBody* rbA, btRigidBody* rbB,
            const vsg::vec3& springAxis=vsg::vec3(0.,1.,0),
            const vsg::vec3& axleAxis=vsg::vec3(1.,0.,0),
            const vsg::vec2& linearLimit=vsg::vec2( -1., 1. ),
            const vsg::vec2& angleLimit=vsg::vec2( -vsg::PIf*0.25f, vsg::PIf*0.25f ),
            const vsg::vec3& point=vsg::vec3(0.,0.,0) );
    WheelSuspensionConstraint( const WheelSuspensionConstraint& rhs, const vsg::CopyOp& copyop={} );

    virtual btHinge2Constraint* getAsBtHinge2() const;

    /** \brief Specify the spring axis in world space.

    The spring axis allows +/- motion along the axis, as well
    as rotation (see setLimit()).

    The spring and axle axes must be orthogonal. This class resolves
    any non-orthogonality issues when it creates the Bullet constraint
    (in createConstraing()).
    */
    void setSpringAxis( const vsg::vec3& springAxis );
    /** \overload */
    void setSpringAxis( const double x, const double y, const double z );
    vsg::vec3 getSpringAxis() const
    {
        return( _springAxis );
    }
    /** \brief Specify the axle axis in world space.

    Rotation around the axle axis is unrestricted.

    The spring and axle axes must be orthogonal. This class resolves
    any non-orthogonality issues when it creates the Bullet constraing
    (in createConstraing()).
    */
    void setAxleAxis( const vsg::vec3& axleAxis );
    /** \overload */
    void setAxleAxis( const double x, const double y, const double z );
    vsg::vec3 getAxleAxis() const
    {
        return( _axleAxis );
    }
    /** \brief Set the \c _springAxis linear spring limits.

    The spring along the \c _springAxis acts like a LinearSpringConstraint.
    Use setLinearLimit() to specify the allowable translation along this axis.
    Default is -1.0 to 1.0 in world units, with limit position 0.0 corresponding
    to the initial transformation of the rigid bodies. */
    void setLinearLimit( const vsg::vec2& linearLimit );
    /** \overload */
    void setLinearLimit( const double lo, const double hi );
    vsg::vec2 getLinearLimit() const
    {
        return( _linearLimit );
    }
    /** \brief Set the \c _springAxis rotation limits in radians.

    In the typical use case of modeling a vehicle wheel, \c limitRadians indicates
    the maximum extent the wheel can be turned to the right or left. Limit angle
    extents are in radians around \c _springAxis. Default is the range -pi/4 to pi/4.
    */
    void setAngleLimit( const vsg::vec2& limitRadians );
    /** \overload */
    void setAngleLimit( const double lo, const double hi );
    vsg::vec2 getAngleLimit() const
    {
        return( _angleLimit );
    }
    /** \brief Set the center of rotation, which both axes pass through.

    The default is (0,0,0), the origin. */
    void setAnchorPoint( const vsg::vec3& wcPoint );
    /** \overload */
    void setAnchorPoint( const double x, const double y, const double z );
    vsg::vec3 getAnchorPoint() const
    {
        return( _point );
    }

    /** Return true if the spring and axle axes member variables, and base class, are
    equal to the right-hand-side spring and axle axes and base class. */
    virtual bool operator==( const WheelSuspensionConstraint& rhs ) const;
    /** Return true if the spring and axle axes member variables, or base class, differ
    from the right-hand-side spring and axle axes, or base class. */
    virtual bool operator!=( const WheelSuspensionConstraint& rhs ) const;

protected:
    virtual ~WheelSuspensionConstraint();

    virtual void createConstraint();

    vsg::vec3 _springAxis;
    vsg::vec3 _axleAxis;
    vsg::vec2 _linearLimit;
    vsg::vec2 _angleLimit;
    vsg::vec3 _point;
};


/**@}*/


// vsgbDynamics
}


// __VSGBDYNAMICS_CONSTRAINTS_H__
#endif
