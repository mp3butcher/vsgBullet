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

#include <vsgbCollision/AbsoluteModelTransform.h>
//#include <osgUtil/CullVisitor>

#include <vsg/maths/mat4.h>

#include <string>
#include <osg/io_utils>


namespace vsgbCollision
{


AbsoluteModelTransform::AbsoluteModelTransform()
{
    setReferenceFrame( vsg::Transform::ABSOLUTE_RF );
}
AbsoluteModelTransform::AbsoluteModelTransform( const vsg::mat4& m )
  : _mat4( m )
{
    setReferenceFrame( vsg::Transform::ABSOLUTE_RF );
}
AbsoluteModelTransform::AbsoluteModelTransform( const AbsoluteModelTransform& rhs, const vsg::CopyOp& copyop )
  : vsg::Transform( rhs, copyop ),
    _mat4( rhs._mat4 )
{
    setReferenceFrame( vsg::Transform::ABSOLUTE_RF );
}
AbsoluteModelTransform::~AbsoluteModelTransform()
{
}


bool
AbsoluteModelTransform::computeLocalToWorldmat4( vsg::mat4& mat4, vsg::Visitor* nv ) const
{
    if( getReferenceFrame() == vsg::Transform::ABSOLUTE_RF )
    {
        vsg::mat4 view;
        if( !nv )
            vsg::notify( vsg::INFO ) << "AbsoluteModelTransform: nullptr NodeVisitor; can't get view." << std::endl;
        else if( nv->getVisitorType() != vsg::NodeVisitor::CULL_VISITOR )
            vsg::notify( vsg::INFO ) << "AbsoluteModelTransform: NodeVisitor is not CullVisitor; can't get view." << std::endl;
        else
        {
            osgUtil::CullVisitor* cv = static_cast< osgUtil::CullVisitor* >( nv );
#ifdef SCENEVIEW_ANAGLYPHIC_STEREO_SUPPORT
            // If OSG_STEREO=ON is in the environment, SceneView hides the view mat4
            // in a stack rather than placing it in a Camera node. Enable this code
            // (using CMake) to use a less-efficient way to compute the view mat4 that
            // is compatible with SceneView's usage.
            vsg::NodePath np = nv->getNodePath();
            np.pop_back();
            vsg::mat4 l2w = vsg::computeLocalToWorld( np );
            vsg::mat4 invL2w = vsg::mat4::inverse( l2w );
            view = invL2w * *( cv->getModelViewmat4() );
#else
            // Faster way to determine the view mat4, but not compatible with
            // SceneView anaglyphic stereo.
            vsg::Camera* cam = cv->getCurrentCamera();
            cam->computeLocalToWorldmat4( view, cv );
#endif
        }
        mat4 = ( _mat4 * view );
    }
    else
        // RELATIVE_RF
        mat4.preMult(_mat4);

    return( true );
}

bool
AbsoluteModelTransform::computeWorldToLocalmat4( vsg::mat4& mat4, vsg::Visitor* nv ) const
{
    vsg::mat4 inv( vsg::mat4::inverse( _mat4 ) );
    if( getReferenceFrame() == vsg::Transform::ABSOLUTE_RF )
    {
        vsg::mat4 invView;
        if( !nv )
            vsg::notify( vsg::NOTICE ) << "AbsoluteModelTransform: nullptr NodeVisitor; can't get invView." << std::endl;
        else if( nv->getVisitorType() != vsg::NodeVisitor::CULL_VISITOR )
            vsg::notify( vsg::NOTICE ) << "AbsoluteModelTransform: NodeVisitor is not CullVisitor; can't get invView." << std::endl;
        else
        {
            osgUtil::CullVisitor* cv = static_cast< osgUtil::CullVisitor* >( nv );
#ifdef SCENEVIEW_ANAGLYPHIC_STEREO_SUPPORT
            // If OSG_STEREO=ON is in the environment, SceneView hides the view mat4
            // in a stack rather than placing it in a Camera node. Enable this code
            // (using CMake) to use a less-efficient way to compute the view mat4 that
            // is compatible with SceneView's usage.
            vsg::NodePath np = nv->getNodePath();
            np.pop_back();
            vsg::mat4 l2w = vsg::computeLocalToWorld( np );
            invView = *( cv->getModelViewmat4() ) * l2w;
#else
            vsg::Camera* cam = cv->getCurrentCamera();
            cam->computeWorldToLocalmat4( invView, cv );
#endif
        }
        mat4 = ( invView * inv );
    }
    else
        // RELATIVE_RF
        mat4.postMult( inv );

    return( true );
}


// namespace osgwTools
}
