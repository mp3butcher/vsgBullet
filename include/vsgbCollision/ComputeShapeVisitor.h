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

#ifndef __VSGCOLLISION_COMPUTE_SHAPE_VISITOR_H__
#define __VSGCOLLISION_COMPUTE_SHAPE_VISITOR_H__ 1

#include <vsgbCollision/Export.h>
#include <vsgbCollision/CollisionShapes.h>
#include <btBulletCollisionCommon.h>
#include <vsg/core/Visitor.h>
#include <vsg/nodes/VertexIndexDraw.h>

#include <string>


namespace vsgbCollision
{

/** \class ComputeShapeVisitor ComputeShapeVisitor.h <vsgbCollision/ComputeShapeVisitor.h>
\brief A NodeVisitor that creates Bullet collision shapes for each Geode and assembles them
into a single btCompoundShape.

This visitor is designed to work in conjuction with the
\link rigidbody rigid body creation utilities \endlink and is invoked indirectly from
vsgbDynamics::createRigidBody() via btCompoundShapeFromVSGGeodes().

To correctly support center of mass and scaling, the source scene graph should
be temporarily multiparented to a MatrixTransform containing the concatenation of
the negative center of mass translation and the scale matrix. See the implementation
of vsgbDynamics::createRigidBody(), which does this correctly.

Bullet's box, sphere, and cylinder shapes are always at the origin, so this visitor
creates them as child shapes of a btCompoundShape (which supports a btTransform per child).
Triangle mesh, convex tri mesh, and convex hull are generated from transformed geometric
data and don't require this additional level of indirection.

<b>Work To Be Done</b>

The created collision shape is a btCompoundShape. However, the compound shape
is entirely flat (collision shapes created from each Geode are added directly as
children to the btCompoundShape). Bullet might benefit from a more hierarchical
arrangement. For example, when the visitor encounters a Group during traversal,
it could add a btCompoundShape child, which becomes the new parent for any Geodes
under that Group. We should investigate this before proceeding with this work.

A collision shape per Geode is not necessarily the most accurate way to represent
all VSG scene graphs. Perhaps a collision shape per Geometry, or per PrimitiveSet,
or some combination of all three, would be best. For now, only per-Geode is supported.
*/
class VSGBCOLLISION_EXPORT ComputeShapeVisitor : public vsg::Visitor
{
public:
    /** \brief Specifies the shape to create per Geode, axis (if \c shapeType is a cylinder),
    and traversal mode. */
    ComputeShapeVisitor( const BroadphaseNativeTypes shapeType, const vsgbCollision::AXIS axis=Y,
        const unsigned int reductionLevel=0
                        //, vsg::NodeVisitor::TraversalMode traversalMode=vsg::NodeVisitor::TRAVERSE_ALL_CHILDREN
                        );


    /** \brief Computes overall bound of input scene graph for use in geometry reduction. */
    void apply( vsg::Group& node );
    /** \brief Computes overall bound of input scene graph for use in geometry reduction. */
    void apply( vsg::Node& node );
    /** \brief Builds ComputeShapeVisitor::_localNodePath (a NodePath) from all Transforms, excluding AbsoluteTransform.

    This visitor must transform all geometry by Transform nodes in the scene graph before using
    that geometry to create the collision shape. However, in order to be compatible with the
    \link rigidbody rigid body creation utilities, \endlink the visitor can't consider
    AbsoluteTransforms in such a transformation, as they ignore all parent transforms.

    To support this, we override NodeVisitor::apply(vsg::Transform&) to build our own
    NodePath (ComputeShapeVisitor::_localNodePath) that contains all Transform nodes encountered during traversal
    except AbsoluteTransform nodes. */
    void apply( vsg::Transform& node );

    void apply( vsg::StateGroup& node );
    vsg::StateGroup *curparrentstategr;

    /** After the visitor has traversed the scene graph, call this function to
    obtain a pointer to the created collision shape. The calling code is responsible
    for deleting the btCollisionShape pointer to avoid memory leaks. */
    btCollisionShape* getShape();
    /** \overload */
    const btCollisionShape* getShape() const;

protected:
    /** \brief Calls createShape() and adds the result to the master btCompoundShape.
    */
    void createAndAddShape( vsg::StateGroup& node, const vsg::mat4& m );

    /** \brief Creates a btCollisionShape for the specified Node.

    Creates a deep copy of the Geode, transforms the copy, and creates a
    collision shape from the transformed data.

    If ComputeShapeVisitor::_shapeType speciies a box, sphere, or cylinder, this function adds the
    created shape as a child to a btCompoundShape, with a btTransform that accounts
    for the bounding volume center. btCompoundShape is not used if ComputeShapeVisitor::_shapeType is a
    triangle mesh, convex tri mesh, or convex hull, as the shape is already in
    the appropriate coordinate space.

    \param node Currently, this must be a Geode.
    \param m Accumulated transformation of the \c node. This function creates
    a deep copy of \c node, transforms the copy by \c m, creates the collision
    shape from the transformed copy, then discards the copy.
    \return nullptr if \c node is not a Geode, or if ComputeShapeVisitor::_shapeType is unsupported.
    Otherwise, returns the created collision shape.
    */
    btCollisionShape* createShape( vsg::StateGroup& node, const vsg::mat4& m );

    /** Called when _shapeType indicates a triangle mesh or convex triangle mesh.
    Uses _reductionLevel to reduce triangle count in the input subgraph. */
    void reduce( vsg::Node& node );

    /** Shape to create per Geode. Set in ComputeShapeVisitor(). */
    const BroadphaseNativeTypes _shapeType;
    /** If _shapeType is a cylinder, created shapes use this axis. Set in ComputeShapeVisitor(). */
    const vsgbCollision::AXIS _axis;

    /** If _shapeType is a triangle mesh or convex triangle mesh, geometry is reduced prior to creating
    the collision shape. Range is 0 (no reduction) to 3 (aggressive reduction). */
    const unsigned int _reductionLevel;
    /** Computed in the first invoked apply() method to compute the overall bounding volume.
    Used in geometry reduction if _reductionLevel is greater than zero and _shapeType indicates
    a triangle mesh or convex triangle mesh. */
    vsg::dbox _bs;

    /** This is the created collision shape representing the traversed scene graph.
    Obtain it by calling getShape(). */
    btCollisionShape* _shape;

    /** NodePath containing only Transform nodes, but excluding AbsoluteTransform. */
    std::vector<vsg::mat4> _localNodePath;
};


// vsgbCollision
}


// __VSGCOLLISION_COMPUTE_SHAPE_VISITOR_H__
#endif
