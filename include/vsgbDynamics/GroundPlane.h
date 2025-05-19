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

#ifndef __VSGBULLET_GROUND_PLANE_H__
#define __VSGBULLET_GROUND_PLANE_H__ 1


#include <vsgbDynamics/Export.h>
#include <btBulletDynamicsCommon.h>
#include <vsg/core/ref_ptr.h>
#include <vsg/maths/vec4.h>


namespace vsg {
    class Node;
}

namespace vsgbDynamics
{


/** \brief Add a plane rigid body to the dynamics world and return an VSG subgraph to render the plane.
*/
VSGBDYNAMICS_EXPORT vsg::ref_ptr<vsg::Node> generateGroundPlane( const vsg::vec4& plane, btDynamicsWorld* bulletWorld, btRigidBody** rb=nullptr, short group=0, short mask=0 );


// vsgbDynamics
}


// __VSGBULLET_GROUND_PLANE_H__
#endif
