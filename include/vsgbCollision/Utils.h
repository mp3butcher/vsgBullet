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

#include "vsg/app/Viewer.h"
#ifndef __VSGBCOLLISION_UTILS_H__
#define __VSGBCOLLISION_UTILS_H__ 1


#include <vsgbCollision/Export.h>

#include <vsg/maths/mat4.h>
#include <vsg/maths/vec3.h>
#include <vsg/maths/vec3.h>
#include <vsg/core/Array.h>
#include <vsg/threading/OperationQueue.h>
#include <vsg/core/Inherit.h>
#include <vsg/core/observer_ptr.h>
#include <vsg/app/CompileManager.h>
#include <LinearMath/btTransform.h>


namespace vsgbCollision
{


/** \defgroup conversionutils Vector and Matrix Data Conversion Utilities
\brief Convenience functions for converting between VSG's and Bullet's vector and matrix classes.

*/
/** Maths utilities **/
template <typename value_type>
inline bool equivalent(value_type lhs, value_type rhs, value_type epsilon=1e-6)
{ float delta = rhs-lhs; return delta < 0.0 ? delta >= -epsilon : delta <= epsilon; }

VSGBCOLLISION_EXPORT template <typename value_type>
void  orthoNormalize(vsg::t_mat4<value_type>& _mat,const vsg::t_mat4<value_type>& rhs)
{
    value_type x_colMag = (rhs[0][0] * rhs[0][0]) + (rhs[1][0] * rhs[1][0]) + (rhs[2][0] * rhs[2][0]);
    value_type y_colMag = (rhs[0][1] * rhs[0][1]) + (rhs[1][1] * rhs[1][1]) + (rhs[2][1] * rhs[2][1]);
    value_type z_colMag = (rhs[0][2] * rhs[0][2]) + (rhs[1][2] * rhs[1][2]) + (rhs[2][2] * rhs[2][2]);

    if(!equivalent((double)x_colMag, 1.0) && !equivalent((double)x_colMag, 0.0))
    {
        x_colMag = sqrt(x_colMag);
        _mat[0][0] = rhs[0][0] / x_colMag;
        _mat[1][0] = rhs[1][0] / x_colMag;
        _mat[2][0] = rhs[2][0] / x_colMag;
    }
    else
    {
        _mat[0][0] = rhs[0][0];
        _mat[1][0] = rhs[1][0];
        _mat[2][0] = rhs[2][0];
    }

    if(!equivalent((double)y_colMag, 1.0) && !equivalent((double)y_colMag, 0.0))
    {
        y_colMag = sqrt(y_colMag);
        _mat[0][1] = rhs[0][1] / y_colMag;
        _mat[1][1] = rhs[1][1] / y_colMag;
        _mat[2][1] = rhs[2][1] / y_colMag;
    }
    else
    {
        _mat[0][1] = rhs[0][1];
        _mat[1][1] = rhs[1][1];
        _mat[2][1] = rhs[2][1];
    }

    if(!equivalent((double)z_colMag, 1.0) && !equivalent((double)z_colMag, 0.0))
    {
        z_colMag = sqrt(z_colMag);
        _mat[0][2] = rhs[0][2] / z_colMag;
        _mat[1][2] = rhs[1][2] / z_colMag;
        _mat[2][2] = rhs[2][2] / z_colMag;
    }
    else
    {
        _mat[0][2] = rhs[0][2];
        _mat[1][2] = rhs[1][2];
        _mat[2][2] = rhs[2][2];
    }

    _mat[3][0] = rhs[3][0];
    _mat[3][1] = rhs[3][1];
    _mat[3][2] = rhs[3][2];

    _mat[0][3] = rhs[0][3];
    _mat[1][3] = rhs[1][3];
    _mat[2][3] = rhs[2][3];
    _mat[3][3] = rhs[3][3];

}

/**  vsg::Viewer utilities **/
struct opDeferredDelete : public vsg::Inherit<vsg::Operation, opDeferredDelete>
{
    opDeferredDelete(vsg::observer_ptr<vsg::Viewer> viewer,vsg::ref_ptr<vsg::Node> node, int frameDelay = 4):
        _node(node),
        _frameDelay(frameDelay),
        _viewer(viewer)
    {}

    vsg::ref_ptr<vsg::Node> _node;
    int _frameDelay;
    vsg::observer_ptr<vsg::Viewer>  _viewer;

    void run() override
    {
        if(_frameDelay <= 0)
        {
            _node = nullptr;
            vsg::ref_ptr<vsg::Viewer> ref_viewer = _viewer;

            if (ref_viewer) ref_viewer->updateOperations->remove(vsg::ref_ptr<opDeferredDelete>(this));
        }
        _frameDelay--;
    }
};
struct opMerge : public vsg::Inherit<vsg::Operation, opMerge>
{
    opMerge(vsg::observer_ptr<vsg::Viewer> viewer, vsg::ref_ptr<vsg::Group> attachmentPoint, vsg::ref_ptr<vsg::Node> node, const vsg::CompileResult& compileResult):
        _viewer(viewer),
        _attachmentPoint(attachmentPoint),
        _node(node),
        _compileResult(compileResult)
    {}

    vsg::observer_ptr<vsg::Viewer> _viewer;
    vsg::ref_ptr<vsg::Group> _attachmentPoint;
    vsg::ref_ptr<vsg::Node> _node;
    vsg::CompileResult _compileResult;

    void run() override
    {
        vsg::ref_ptr<vsg::Viewer> ref_viewer = _viewer;

        if (ref_viewer)
        {
            updateViewer(*ref_viewer, _compileResult);
        }

        _attachmentPoint->addChild(_node);
    }
};

struct opDetach : public vsg::Inherit<vsg::Operation, opDetach>
{
    opDetach(vsg::observer_ptr<vsg::Viewer> viewer,vsg::ref_ptr<vsg::Group> detachmentPoint, vsg::ref_ptr<vsg::Node> node):
        _viewer(viewer),
        _detachmentPoint(detachmentPoint),
        _node(node)
    {}

    vsg::observer_ptr<vsg::Viewer> _viewer;
    vsg::ref_ptr<vsg::Group> _detachmentPoint;
    vsg::ref_ptr<vsg::Node> _node;

    void run() override
    {
        if (_detachmentPoint)
        {
            auto it = std::find(_detachmentPoint->children.begin(), _detachmentPoint->children.end(), _node);
            if(it != _detachmentPoint->children.end())
            {
                _detachmentPoint->children.erase(it);
                vsg::ref_ptr<vsg::Viewer> ref_viewer = _viewer;

                if (ref_viewer)
                   ref_viewer->addUpdateOperation(opDeferredDelete::create(_viewer, _node), vsg::UpdateOperations::ALL_FRAMES);
            }
        }
    }
};

/** Type Conversion utilities **/

VSGBCOLLISION_EXPORT vsg::quat  makeRotate( const vsg::vec3& from, const vsg::vec3& to );
VSGBCOLLISION_EXPORT void makeRotate_original( const vsg::vec3& from, const vsg::vec3& to );

VSGBCOLLISION_EXPORT vsg::mat4 asVsgMatrix( const btTransform& t );
VSGBCOLLISION_EXPORT btTransform asBtTransform( const vsg::mat4& m );

VSGBCOLLISION_EXPORT vsg::mat4 asVsgMatrix( const btMatrix3x3& m );
VSGBCOLLISION_EXPORT btMatrix3x3 asBtMatrix3x3( const vsg::mat4& m );

VSGBCOLLISION_EXPORT vsg::vec3 asVsgVec3( const btVector3& v );
VSGBCOLLISION_EXPORT btVector3 asBtVector3( const vsg::vec3& v );

VSGBCOLLISION_EXPORT vsg::vec4 asVsgVec4( const btVector3& v, const double w );
VSGBCOLLISION_EXPORT vsg::vec4 asVsgVec4( const btVector4& v );
VSGBCOLLISION_EXPORT btVector4 asBtVector4( const vsg::vec4& v );

/** Note: Return value allocated with new. Call disposeBtVector3Array() to
ensure the array is deleted within the vsgbCollision library.
\see LocalBtVector3Array */
VSGBCOLLISION_EXPORT btVector3* asBtVector3Array( const vsg::vec3Array* v );
VSGBCOLLISION_EXPORT bool disposeBtVector3Array( btVector3* array );

VSGBCOLLISION_EXPORT vsg::vec3Array* asVsgVec3Array( const btVector3* v, const unsigned int size );

/** Create a btVector3 array from an VSG vec3Array as a local variable
(deleted the btVector3 array when the class instance goes out of scope). */
class VSGBCOLLISION_EXPORT LocalBtVector3Array
{
public:
    LocalBtVector3Array( const vsg::vec3Array* v );
    virtual ~LocalBtVector3Array();

    btVector3* get();
    const btVector3* get() const;

protected:
    btVector3* _btVector3;
};

// vsgbCollision
}

// __VSGBCOLLISION_UTILS_H__
#endif
