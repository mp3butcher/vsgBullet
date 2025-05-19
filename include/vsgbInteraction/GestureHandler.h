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

#ifndef __VSGBINTERACTION_GESTURE_HANDLER_H__
#define __VSGBINTERACTION_GESTURE_HANDLER_H__


#include <vsgbInteraction/Export.h>
#include <vsg/app/Trackball.h>
#include <functional>
#include <vector>


// Forward
class btGeneric6DofConstraint;


namespace vsgbInteraction
{


// Forward
class HandNode;


/** \class GestureHandler GestureHandler.h <vsgbInteraction/GestureHandler.h>
\brief Allows data gloves to trigger interations via gesture codes.

*/
class VSGBINTERACTION_EXPORT GestureHandler : public vsg::Inherit<vsg::Trackball,GestureHandler>
{
public:
    virtual bool operator()( const unsigned int gestureCode, HandNode& handNode ) = 0;

    static const unsigned int Unknown;
    static const unsigned int Default;
    static const unsigned int Point;
    static const unsigned int Fist;
};

typedef std::vector< vsg::ref_ptr< GestureHandler > > GestureHandlerVector;


class VSGBINTERACTION_EXPORT GripRelease : public GestureHandler
{
public:
    GripRelease();

    virtual bool operator()( const unsigned int gestureCode, HandNode& handNode );

protected:
    ~GripRelease();

    btGeneric6DofConstraint* _constraint;
};


// vsgbInteraction
}

// __VSGBINTERACTION_GESTURE_HANDLER_H__
#endif
