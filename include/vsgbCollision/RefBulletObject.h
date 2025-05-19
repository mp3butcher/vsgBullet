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

#ifndef __VSGBCOLLISION_REF_COLLISION_OBJECT_H__
#define __VSGBCOLLISION_REF_COLLISION_OBJECT_H__ 1

#include <vsgbCollision/Export.h>
#include <vsg/core/Object.h>
#include <vsg/core/Inherit.h>
#include <BulletCollision/CollisionDispatch/btCollisionObject.h>


// Forwaed declarations
class btCollisionShape;
class btCollisionObject;
class btRigidBody;


namespace vsgbCollision {


/** \class RefBulletObject RefBulletObject.h <vsgbCollision/RefBulletObject.h>
\brief A template class for maintaining reference-counted Bullet objects.

This class overlays VSG's reference-counted memory system on top of Bullet objects.
Use it to store a reference to a Bullet object in a scene graph Node's UserData pointer,
for example, or in any VSG interactions that require a reference-counted entity.

This class replaces RefCollisionShape, RefCollisionObject, and RefRigidBody.

This class derives from Referenced. When its reference count reaches zero, the
base class invokes the destructor. By default, RefBulletObject doesn't delete the
managed object. Change the default behavior with the \c doDelete constuctor
parameter.

If you change the default behavior so that this class deletes the managed object,
then calling setBulletObject() while an object is already managed will cause that
object to be deleted.
*/
template< class T >
class RefBulletObject : public vsg::Inherit<vsg::Object,RefBulletObject<T>>
{
public:
    RefBulletObject( bool doDelete=false )
      : _doDelete( doDelete ),
        _tPtr( nullptr )
    {}
    RefBulletObject( T* tPtr, bool doDelete=false )
      : _doDelete( doDelete ),
        _tPtr( tPtr )
    {}

    void set( const T* tPtr )
    {
        if( _doDelete && ( _tPtr != nullptr ) )
            delete _tPtr;
        _tPtr = tPtr;
    }

    T* get()
    {
        return( _tPtr );
    }
    const T* get() const
    {
        return( _tPtr );
    }

protected:
    virtual ~RefBulletObject()
    {
        if( _doDelete )
            delete _tPtr;
    }

    bool _doDelete;
    T* _tPtr;
};



/** For backwards compatibility with vsgBullet v1.x.
\relates RefBulletObject */
typedef RefBulletObject< btCollisionShape > RefCollisionShape;

/** For backwards compatibility with vsgBullet v1.x.
\relates RefBulletObject */
typedef RefBulletObject< btCollisionObject > RefCollisionObject;

/** For backwards compatibility with vsgBullet v1.x.
\relates RefBulletObject */
typedef RefBulletObject< btRigidBody > RefRigidBody;



// namespace vsgbCollision
}


//__VSGBCOLLISION_REF_COLLISION_OBJECT_H__
#endif
