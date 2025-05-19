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

#ifndef __VSGBCOLLISION_VERSION_H__
#define __VSGBCOLLISION_VERSION_H__ 1

#include <vsgbCollision/Export.h>
#include <string>


namespace vsgbCollision {

// Please keep in sync with top-level CMakeLists.txt VSGBULLET_VERSION variable.
#define VSGBCOLLISION_MAJOR_VERSION (3)
#define VSGBCOLLISION_MINOR_VERSION (0)
#define VSGBCOLLISION_SUB_VERSION (0)

// C preprocessor integrated version number.
// The form is Mmmss, where:
//   M is the major version
//   mm is the minor version (zero-padded)
//   ss is the sub version (zero padded)
// Use this in version-specific code, for example:
//   #if( VSGBCOLLISION_VERSION < 10500 )
//      ... code specific to releases before v1.05
//   #endif
#define VSGBCOLLISION_VERSION ( \
        ( VSGBCOLLISION_MAJOR_VERSION * 10000 ) + \
        ( VSGBCOLLISION_MINOR_VERSION * 100 ) + \
          VSGBCOLLISION_SUB_VERSION )

// Returns VSGBCOLLISION_VERSION.
unsigned int VSGBCOLLISION_EXPORT getVersionNumber();

// Pretty string.
std::string VSGBCOLLISION_EXPORT getVersionString();


// Backwards compatibility
#define VSGBBULLET_MAJOR_VERSION VSGBCOLLISION_MAJOR_VERSION
#define VSGBBULLET_MINOR_VERSION VSGBCOLLISION_MINOR_VERSION
#define VSGBBULLET_SUB_VERSION VSGBCOLLISION_SUB_VERSION
#define VSGBBULLET_VERSION VSGBCOLLISION_VERSION


// namespace vsgbCollision
}


// __VSGBCOLLISION_VERSION_H__
#endif


/** \mainpage vsgBullet Documentation

\section Introduction Introduction

vsgBullet is a set of software tools for applications that use both
<a href="https://vsg-dev.github.io/vsg-dev.io/">VulkanSceneGraph (VSG)</a> and
<a href="https://github.com/bulletphysics/bullet3">Bullet</a>.

The vsgBullet library is brought to you by Julien Valentin (mp3butcher@hotmail.com).
It's an adaptation of works done on osgBullet by Paul Martz
(<a href="http://www.skew-matrix.com/">Skew Matrix Software</a>),
<a href="http://www.ameslab.gov/">Ames Lab</a>, and
<a href="http://www.pica.army.mil/PicatinnyPublic/index.asp">ARDEC</a>.
It was used as part of the
<a href="http://www.ve-suite.org/">VE-Suite</a> project, as well as
other applications and software projects.

vsgBullet is open source and
available under the GNU LGPL v2.1 software license. 

vsgBullet <a href="https://github.com/mp3butcher/vsgBullet">source and
issue tracking</a> are on Github.

\subsection phil Philosophy

vsgBullet is a set of tools to facilitate developing software that uses Bullet
for physics simultation and collision detection, and uses VSG for rendering.
vsgBullet doesn't attempt to wrap either VSG or Bullet. Instead, it allows you
(the programmer) direct access to both APIs. vsgBullet gets out of your way
so that your application can use the full feature set of both VSG and Bullet.

vsgBullet plays a key role in this scenario by providing useful tools for applications
that use both APIs.

\subsection feat Features

vsgBullet's most useful feature is its ability to accomodate Bullet's strict coordinate
system and transformation requirements and still support the arbitrary coordinate systems
and transformations often encountered in VSG (and other 3D) applications. Bullet collision
shapes must be created so that the center of mass corresponds to the coordinate origin,
but this is not how most 3D models are built. Bullet doesn't support scaling, but 3D models
are often scaled up or down to match a world coordinate system. VSG transformations are
hierarchical, but Bullet wants to completely own the local-to-world transformation. All
of these issues are handled by the
\link collisionshapes collision shape creation utilities \endlink
in combination with the \ref vsgbDynamics::MotionState "MotionState" class. vsgBullet also provides
\link rigidbody rigid body creation utilities \endlink
that wrap collision shape and \ref vsgbDynamics::MotionState "MotionState" creation.

Other features include the following.

\li Support for running physics and rendering in separate threads.
\li Supports using Bullet for both rigid body dynamics as well as collision detection only.
\li A set of vsg::Visitor for interacting with the physics simulation, including
the vsgbInteraction::HandNode class to support data glove usage.
\li Routines for creating Bullet collision shapes from VSG geometry.
\li TODO An VSG-based implementation of \c \c btIDebugDraw.
\li VSG reference counting for Bullet objects.
\li Functions to convert between VSG and Bullet matrices and vectors.


\section appsexamples Applications and Examples

\li The \ref vsgbpp "vsgbpp" application allows you to preview a physics simultation on a model.
\li The \ref examplecom "centerofmass" example demonstrates application-specified center of mass.
\li The \ref collision "collision" example demonstrates using vsgBullet and Bullet for collision detection only.
\li The \ref diceexample "dice" example, just for fun.
\li The \ref handphysicsexample "hand physics" example demonstrates using the \c HandNode to interact with the scene.
\li The \ref hingelowlevel "hinge" example demonstrates creating a hinge constraint.
\li The \ref multithreaded "multithreaded" example demonstrates running Bullet in a separate thread.
\li The \ref saverestoreexample "saverestore" example demonstrates saving/restoring a physics simultation to/from disk.
\li The \ref sliderlowlevel "slider" example demonstrates creating a slider constraint.

\section libraries Libraries
TODO Dot VSG file support for classes and objects in the vsgbDynamics library.
TODO Support for .SGB, an vsgBullet file format for storing physics state.
\subsection vsgbcollision vsgbCollision

Collision detection and collision shape support. Facilities for
creating Bullet collision shapes from VSG scene graphs, and vice
versa.

\subsection vsgbdynamics vsgbDynamics

Rigid body dynamics and constraints support.

\subsection vsgbinteraction vsgbInteraction

Support for user interaction with the physics simulation, such as dragging
objects, resetting the simulation to a save point, and launching models into
the scene.
TODO An articulatable hand model supports an immersive simultation
experience, and includes aupport for the P5 data glove.

*/
