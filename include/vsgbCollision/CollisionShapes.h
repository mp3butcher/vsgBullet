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

#ifndef __VSGBCOLLISION_COLLISION_SHAPES_H__
#define __VSGBCOLLISION_COLLISION_SHAPES_H__ 1


#include <vsgbCollision/Export.h>
#include <vsg/maths/box.h>
#include <vsg/core/ref_ptr.h>
#include <btBulletCollisionCommon.h>

// Forward
namespace vsg {
    class Node;
    //class Drawable;
}


namespace vsgbCollision
{


/** \defgroup collisionshapes Collision Shape Conversion
\brief Routines for converting between VSG geometric data and Bullet collision shapes.

*/
/**@{*/

/** \brief Enumerant to denote the major axis of a cylinder.

These are the values that btCylinderShape::getUpAxis() returns
for the three different cylinder major axes.
*/
enum AXIS
{
    X=0, Y=1, Z=2
};


/** \brief Return a Bullet box collision shape that approximates the specified VSG geometry.

The box is untransformed. Bullet's box collision shape uses the box extents only.
(btBoxShape is always centered on the origin. See btCompoundShapeFromBounds() to create
the box as a transformed child shape within a btCompoundShape.)

If the calling code has already computed the bounding box extents, pass this information
as the \c bb parameter, and this function will use that information to create the collision shape.
If you do not pass a \c bb parameter, this function uses the \c vsg::ComputeBoundsVisitor to
determine the bounding box extents. */
VSGBCOLLISION_EXPORT btBoxShape* btBoxCollisionShapeFromVSG( vsg::Node* node, const vsg::dbox* bb=nullptr );

/** \brief Return a Bullet sphere collision shape that approximates the specified VSG geometry.

The sphere is untransformed. Bullet's sphere collision shape uses the radius only.
(btSphereShape is always centered on the origin. See btCompoundShapeFromBounds() to create
the sphere as a transformed child shape within a btCompoundShape.) */
VSGBCOLLISION_EXPORT btSphereShape* btSphereCollisionShapeFromVSG( vsg::Node* node );

/** \brief Return a Bullet cylinder collision shape that approximates the specified VSG geometry.

The cylinder is untransformed. Bullet's cylinder collision shape uses the specified axis and computed radius only.
(btCylinderShape is always centered on the origin. See btCompoundShapeFromBounds() to create
the cylinder as a transformed child shape within a btCompoundShape.) */
VSGBCOLLISION_EXPORT btCylinderShape* btCylinderCollisionShapeFromVSG( vsg::Node* node, AXIS axis=Y );

/** \brief Return a Bullet triangle mesh collision shape that approximates the specified VSG geometry.

This function collects all triangles and transforms them by any Transforms in the subgraph rootes at \c node.
*/
VSGBCOLLISION_EXPORT btTriangleMeshShape* btTriMeshCollisionShapeFromVSG( vsg::Node* node );

/** \brief Return a Bullet convex triangle mesh collision shape that approximates the specified VSG geometry.

This function collects all triangles and transforms them by any Transforms in the subgraph rootes at \c node.
*/
VSGBCOLLISION_EXPORT btConvexTriangleMeshShape* btConvexTriMeshCollisionShapeFromVSG( vsg::Node* node );

/** \brief Return a Bullet convex hull collision shape that approximates the specified VSG geometry.

This function collects all vertices and transforms them by any Transforms in the subgraph rootes at \c node.
*/
VSGBCOLLISION_EXPORT btConvexHullShape* btConvexHullCollisionShapeFromVSG( vsg::Node* node );

/** \brief Creates a collision shape for each Geode in the scene graph,
and assembles them into a single btCompoundShape.

If \c shapeType is CYLINDER_SHAPE_PROXYTYPE, \c axis indicates the cylinder axis.

\param reductionLevel If \c shapeType is TRIANGLE_MESH_SHAPE_PROXYTYPE or
CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE, a copy of the input geometry is reduced based on
this value, which is in the range 0 (to reduction) to 3 (aggressive reduction). See
vsgbDynamics::CreationRecord::ReductionLevel. Default is 0 (no reduction). */
VSGBCOLLISION_EXPORT btCompoundShape* btCompoundShapeFromVSGGeodes( vsg::Node* node,
    const BroadphaseNativeTypes shapeType, const vsgbCollision::AXIS axis=Y,
    const unsigned int reductionLevel = 0 );

/** \brief Currently not implemented. Creates a collision shape for each Geode in the scene graph,
and assembles them into a single btCompoundShape. */
VSGBCOLLISION_EXPORT btCompoundShape* btCompoundShapeFromVSGGeometry( vsg::Node* node );

/** \brief Creates a transformed box, sphere, or cylinder collision shape.

Bullet supports transformed box, sphere, and cylinder shapes with the btCompoundShape.
This function creates a box, sphere, or cylinder collision shape, and adds it as a child
shape to a btCompoundShape. The child shape transform is the bounding volume translation.

Currently, this function supports only translation. In the future, rotation and
scale could also be supported. */
VSGBCOLLISION_EXPORT btCompoundShape* btCompoundShapeFromBounds( vsg::Node* node,
    const BroadphaseNativeTypes shapeType, const vsgbCollision::AXIS axis=Y );



/*@{*/
/** \brief Return an VSG representation of the given bullet collision shape.

This is useful for Bullet applications being ported to visualize the physics simulation with VSG. */
VSGBCOLLISION_EXPORT vsg::ref_ptr<vsg::Node> vsgNodeFromBtCollisionShape( const btCollisionShape* btShape, const btTransform& trans = btTransform::getIdentity() );
/** \overload */
VSGBCOLLISION_EXPORT vsg::ref_ptr<vsg::Node> vsgNodeFromBtCollisionShape( const btBoxShape* btShape, const btTransform& trans = btTransform::getIdentity() );
/** \overload */
VSGBCOLLISION_EXPORT vsg::ref_ptr<vsg::Node> vsgNodeFromBtCollisionShape( const btSphereShape* btShape, const btTransform& trans = btTransform::getIdentity() );
/** \overload */
VSGBCOLLISION_EXPORT vsg::ref_ptr<vsg::Node> vsgNodeFromBtCollisionShape( const btCylinderShape* btShape, const btTransform& trans = btTransform::getIdentity() );
/** \overload */
VSGBCOLLISION_EXPORT vsg::ref_ptr<vsg::Node> vsgNodeFromBtCollisionShape( const btTriangleMeshShape* btShape, const btTransform& trans = btTransform::getIdentity() );
/** \overload */
VSGBCOLLISION_EXPORT vsg::ref_ptr<vsg::Node> vsgNodeFromBtCollisionShape( const btConvexTriangleMeshShape* btShape, const btTransform& trans = btTransform::getIdentity() );
/** \overload */
VSGBCOLLISION_EXPORT vsg::ref_ptr<vsg::Node> vsgNodeFromBtCollisionShape( const btConvexHullShape* btShape, const btTransform& trans = btTransform::getIdentity() );
/*@}*/


/*@{*/
/** \brief Returns an VSG Geometry to render the specified collision shape.
VSGBCOLLISION_EXPORT vsg::Geometry* vsgGeometryFromBtCollisionShape( const btBoxShape* btShape );
/** \brief Returns an VSG Geometry to render the specified collision shape. * /
VSGBCOLLISION_EXPORT vsg::Geometry* vsgGeometryFromBtCollisionShape( const btSphereShape* btSphere );
/**  \brief Returns an VSG Geometry to render the specified collision shape.
Uses the VSG ShapeDrawable utilities to create the cylinder Geometry.
 * /
VSGBCOLLISION_EXPORT vsg::Geometry* vsgGeometryFromBtCollisionShape( const btCylinderShape* btCylinder ); */


VSGBCOLLISION_EXPORT vsg::ref_ptr<vsg::Node> vsgDrawableFromBtCollisionShape( const btBoxShape* btShape );

VSGBCOLLISION_EXPORT vsg::ref_ptr<vsg::Node> vsgDrawableFromBtCollisionShape( const btSphereShape* btSphere );

VSGBCOLLISION_EXPORT vsg::ref_ptr<vsg::Node> vsgDrawableFromBtCollisionShape( const btCylinderShape* btCylinder );

/*@}*/

/**@}*/


// vsgbCollision
}


// __VSGBCOLLISION_COLLISION_SHAPES_H__
#endif
