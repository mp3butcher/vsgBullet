/*************** <auto-copyright.pl BEGIN do not edit this line> **************
 *
 * osgWorks is (C) Copyright 2025 by Julien Valentin
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

#ifndef __vsgbCollision_ABSOLUTE_MODEL_TRANSFORM_H__
#define __vsgbCollision_ABSOLUTE_MODEL_TRANSFORM_H__


#include <vsg/core/Visitor.h>
#include <vsg/nodes/Transform.h>
#include <vsgbCollision/Export.h>

#include <string>


namespace vsgbCollision {


/** \mainpage osgWorks Documentation

\section IntroSection Introduction

osgWorks is an open source SDK intended for OSG developers.
The project contains developer tools that are useful to developers
of most OSG software, such as standalone applications,
transformation utilities, visitors, custom nodes, scene graph
editing facilities, basic geometric shapes, plugins, test models,
and example programs.

\section LibrariesSection Libraries

\subsection osgwTools osgwTools

The primary osgWorks library, osgwTools, contains a large collection of
OSG-based NodeVisitors, custom Nodes, utility functions, and other useful tools.

\subsection osgwControls osgwControls

osgwControls is a partially implemented 3D UI toolkit.

\subsection osgwMx osgwMx

osgwMx contains mat4 support routines, including a flexible viewing manipulator
and event handler that supports variable FOV and toggling between perspective and
orthographic projections.

\subsection osgwQuery osgwQuery

osgwQuery contains OpenGL occlusion query support utilities, including an implementation
of the Guthe optimal occlusion query algorithm.

\section PluginsSection Plugins

\subsection PluginosgwTools osgdb_osgwTools
\subsection Pluginskeletom osgdb_skeleton

See the \ref Plugins module description.

\section AppsSection Applications
\ref osgwbvv visualizes OSG bounding volumes. \n
\ref osgwcaps displays OpenGL / graphics card capabilities. \n
\ref osgwcomp compares the structure of two scene graphs. \n
\ref osgwinfo displays scene graph contents and statistical information. \n
\ref osgwnames displays the structure of a scene graph. \n
\ref osgwnormalmap maps the specified normal map to the loaded file. \n
\ref osgwwhich locates OSG data files and shared libraries.
*/




/** \class AbsoluteModelTransform AbsoluteModelTransform.h <osgwTools/AbsoluteModelTransform.h>
\brief Sets the model part of the OpenGL modelview mat4.

AbsoluteModelTransform (AMT) is an OSG Transform that overloads ABSOLUTE_RF to
preserve the view. Non-Camera transforms above this Node in the hierarchy are
ignored, and the mat4 stored in this Node is the only transform concatenated
with the view.

This is useful, for example, when allowing a 3rd party physics simultator to
transform parts of your scene graph without reparenting those parts to the
scene graph root. In fact, this class was created to support the osgBullet
project (osgBullet.googlecode.com).

You can disable this behavior by setting the reference frame to RELATIVE_RF,
which causes this Transform to behave like a regular mat4Transform. The
default reference frame is ABSOLUTE_RF.

For the AbsoluteModelTransform to operate, it needs the current view mat4.
The code supports obtaining the view mat4 in two ways. Select the algorithm
using the OSGWORKS_SCENEVIEW_ANAGLYPHIC_STEREO_SUPPORT CMake variable. By default,
OSGWORKS_SCENEVIEW_ANAGLYPHIC_STEREO_SUPPORT is disabled, and AbsoluteModelTransform
assumes there is a Camera in the CullVisitor NodePath that contains the view mat4.
This is the most efficient method, but it doesn't work in some usages, such
as when the user has enabled anaglyphic stereo via the OSG_STEREO environment variable.
Set OSGWORKS_SCENEVIEW_ANAGLYPHIC_STEREO_SUPPORT on in CMake to support this usage. When
enabled, AbsoluteModelTransform obtains the view mat4 by accumulating the model transform
from the NodePath, inverting it, and multiplying it by the CullVisitor current modelview
mat4. The result is the view transform. This is a more general purpose, but less efficient solution.

\test amt

*/
class VSGBCOLLISION_EXPORT AbsoluteModelTransform : public vsg::Transform
{
public:
    AbsoluteModelTransform();
    AbsoluteModelTransform( const vsg::mat4& m );
    AbsoluteModelTransform( const AbsoluteModelTransform& rhs );

    //META_Node( vsgbCollision, AbsoluteModelTransform );

    virtual bool computeLocalToWorldmat4( vsg::mat4& mat4, vsg::Visitor* nv ) const;
    virtual bool computeWorldToLocalmat4( vsg::mat4& mat4, vsg::Visitor* nv ) const;

    inline void setmat4( const vsg::mat4& m ) { _mat4 = m; }// dirtyBound(); }
    inline const vsg::mat4& getmat4() const { return _mat4; }


protected:
    virtual ~AbsoluteModelTransform();

    vsg::mat4 _mat4;
};

// namespace osgwTools
}

// __OSGWORKS_ABSOLUTE_MODEL_TRANSFORM_H__
#endif
