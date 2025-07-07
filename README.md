# vsgBullet

vsgBullet is a set of software tools for applications that use both VulkanSceneGraph (VSG) and Bullet. The vsgBullet library is the result of collaborative work between Paul Martz (Skew Matrix Software), Ames Lab, and ARDEC. It's used as part of the VE-Suite project. vsgBullet is open source and available under the GNU LGPL v2.1 software license.

## Dependencies
-  VulkanSceneGraph ( https://github.com/vsg-dev/VulkanSceneGraph )
-  Bullet Physics Engine ( https://github.com/bulletphysics/bullet3 )

## Documentation
Doxygen documentation.

## Features
### Applications
osgbpp: The vsgBullet physics previewer. Loads a model file, creates a Bullet rigid body to match the geometry, and drops it on a surface. View a graphical representation of the collision shape with the --debug option. Use the --help option for more info.

### Libraries
The vsgbCollision library Includes the following features:
-  Supports using Bullet for collision detection, with no dependency on libBulletDynamics.
-  Utility functions for converting between Bullet collision shapes and OSG scene graphs (create a scene graph to render a collision shape, for example, or create a collision shape from a scene graph).
-  A NodeVisitor to compute collision shapes per Geode and assemble them into a single btCompoundShape.
-  Utility functions for converting between Bullet and OSG matrix and vector data types.
-  A template class to incorporate Bullet objects as OSG Objects to support reference counting and association as UserData. Allows you to reference count a btRigidBody, for example, or store a btCollisionShape as UserData.

The vsgbDynamics library Includes the following features:
-  Supports interleaved/serial physics step and render, and a separate physics simulation thread. Threaded physics simulation features an efficient triple buffering mechanism for shared transform data.
-  A MotionState class to keep your OSG subgraph visual representation in sync with the Bullet rigid body / collision shape. Bullet effectively owns the OSG transformation matrix, and the MotionState transparently accounts for center of mass and scaling.
-  Convenience routines to create rigid bodies from scene graphs, which handles all of the details of creating a MotionState, setting the initial transformation, handling center of mass offset, and creating scaled collision shapes.
-  A VulkanDebugDrawer debugging aid class that renders information regarding Bullet collision shapes and intersection points, both within the OSG scene and graphically as a HUD.

### Examples

The project contains a small collection of example programs to demonstrate use of many of vsgBullet's features.

### Tests

The project contains several test programs to ensure correct functionality.

### Building

CMake is your friend

## Contribute
All contributions are welcome and will be considered for inclusion in the project. Please contribute any enhancements or bug fixes by opening an issue and use the Contribution from user issue template. Create and attach a compressed patch file containing your changes.
