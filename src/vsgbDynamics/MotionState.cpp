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

#include <vsgbDynamics/MotionState.h>
#include <vsgbCollision/Utils.h>
#include <vsgbDynamics/TripleBuffer.h>

#include <vsg/nodes/MatrixTransform.h>
#include <vsg/maths/transform.h>


namespace vsgbDynamics
{

MotionState::MotionState( const vsg::mat4& parentTransform,
                          const vsg::vec3& centerOfMass )
  : _parentTransform( parentTransform ),
    _com( centerOfMass ),
    _scale( vsg::vec3( 1., 1., 1. ) ),
    _tb( nullptr ),
    _tbIndex( 0 )
{
    _transform.setIdentity();
}

// Sets both the transformation of the collision shape / rigid body in the
// physics simulation, as well as the matrix of the subgraph's parent
// Transform node for the VSG visual representation.
//
// Called by resetTransform() with the transform (C/S P) -- center of mass
// over scale, concatenated with the initial parent transform.
// Apps typically call setCenterOfMass(), setScale(), and
// setParentTransform() during initialization; these routines then call
// resetTransform(), which calls here. This results in setting the initial
// position of both the rigid body and the visual representation.
//
// Bullet calls this method during the physics simulation to position
// collision shapes / rigid bodies.
//
// Note that the transformation of the collision shape is not the same as the
// transformation for the visual representation. MotionState supports
// off-origin and scaled visual representations, and thus compensates for
// differences between scaled and COM-translated collision shapes and
// unscaled and COM-untranslated VSG subgraphs.
void MotionState::setWorldTransform(const btTransform& worldTrans)
{
    // Call the callback, if registered.
    if( _mscl.size() > 0 )
    {
        // Call only if position changed.
        const btVector3 delta( worldTrans.getOrigin() - _transform.getOrigin() );
        const btScalar eps( (btScalar)( 1e-5 ) );
        const bool quiescent( vsgbCollision::equivalent( delta[ 0 ], btScalar(0.), eps ) &&
            vsgbCollision::equivalent( delta[ 1 ], btScalar(0.), eps ) &&
            vsgbCollision::equivalent( delta[ 2 ], btScalar(0.), eps ) );
        if( !quiescent )
        {
            MotionStateCallbackList::iterator it;
            for( it = _mscl.begin(); it != _mscl.end(); ++it )
                (**it)( worldTrans );
        }
    }

    // _transform is the model-to-world transformation used to place collision shapes
    // in the physics simulation. Bullet queries this with getWorldTransform().
    _transform = worldTrans;

    if( _tb == nullptr )
        setWorldTransformInternal( worldTrans );
    else
    {
        char* addr( _tb->writeAddress() );
        if( addr == nullptr )
        {
           std::cerr<< "MotionState: No TripleBuffer write address." << std::endl;
            return;
        }
        btScalar* fAddr = reinterpret_cast< btScalar* >( addr + _tbIndex );
        worldTrans.getOpenGLMatrix( fAddr );
    }
}

void MotionState::getWorldTransform(btTransform& worldTrans ) const
{
    worldTrans = _transform;
}

void MotionState::setWorldTransformInternal( const btTransform& worldTrans )
{
    // Compute the transformation of the VSG visual representation.
    const vsg::mat4 dt = vsgbCollision::asVsgMatrix( worldTrans );
    const vsg::mat4 col2ol = computeCOLocalToVsgLocal();
    const vsg::mat4 t = col2ol * dt;
    vsg::warn(t);
    if( _mt.valid() )
         _mt->matrix=( t );
    else if( _amt.valid() )
        _amt->matrix=( t );
}

vsg::mat4 MotionState::computeCOLocalToVsgLocal() const
{
    // Accound for center of mass and scale.
    const vsg::vec3 cs( _com[0]*_scale[0], _com[1]*_scale[1], _com[2]*_scale[2] );
    const vsg::mat4 csMat = vsg::translate( -cs );

    const vsg::mat4 scale = vsg::scale( _scale );

    // Return the concatenation of these.
    return( scale * csMat );
}
vsg::mat4 MotionState::computeVsgLocalToCOLocal() const
{
    // Accound for center of mass and scale.
    const vsg::vec3 cs( _com[0]*_scale[0], _com[1]*_scale[1], _com[2]*_scale[2] );
    const vsg::mat4 csMat = vsg::translate( -cs );

    return( csMat );
}
vsg::mat4 MotionState::computeVsgWorldToCOLocal() const
{
    // Convert to VSG local coords...
    vsg::mat4 l2w;
    if( _mt.valid() )
        l2w = _mt->matrix;
    else if( _amt.valid() )
        l2w = _amt->matrix;
    vsg::mat4 w2l = vsg::inverse(l2w);
    //w2l.invert( l2w );

    // ...and accound for center of mass and scale.
    vsg::mat4 ol2col = computeVsgLocalToCOLocal();

    vsg::mat4 scale = vsg::scale( _scale );

    // Return the concatenation of these.
    //return( w2l * ol2col * scale );
    return(  scale* ol2col * w2l );
}
vsg::mat4 MotionState::computeVsgWorldToBulletWorld() const
{
    // Compute VSG world coords to collision object local coords matrix...
    vsg::mat4 ow2col = computeVsgWorldToCOLocal();

    // ...and convert out to Bullet world coords.
    btTransform bulletl2w;
    getWorldTransform( bulletl2w );
    vsg::mat4 bl2w = vsgbCollision::asVsgMatrix( bulletl2w );
    // Return the concatenation of these.
    //return( ow2col * bl2w );
    return(   bl2w * ow2col );
}


void MotionState::setTransform( vsg::Transform* transform )
{
    auto mt=transform->cast<vsg::MatrixTransform>();
    vsg::ref_ptr<vsg::AbsoluteTransform> amt;
    if( mt != nullptr )
        _mt = mt;
    else if( (amt = transform->cast<vsg::AbsoluteTransform>()) != nullptr )
        _amt = amt;
    else
        std::cerr << "MotionState: Unsupported transform type: " << transform->className() << std::endl;
}

vsg::Transform* MotionState::getTransform()
{
    if( _mt.valid() )
        return( _mt.get() );
    else if( _amt.valid() )
        return( _amt.get() );
    else
        return nullptr;
}

const vsg::Transform* MotionState::getTransform() const
{
    if( _mt.valid() )
        return( _mt.get() );
    else if( _amt.valid() )
        return( _amt.get() );
    else
        return nullptr;
}

void MotionState::setParentTransform( const vsg::dmat4 m )
{
    vsg::info ( "setParent" , m );
    vsgbCollision::orthoNormalize<double>(_parentTransform, m);
    //_parentTransform = vsg::mat4::orthoNormal( m );
    resetTransform();
}

vsg::dmat4 MotionState::getParentTransform() const
{
    return( _parentTransform );
}

void MotionState::setCenterOfMass( const vsg::vec3& com )
{
    _com = com;
    resetTransform();
}

vsg::vec3 MotionState::getCenterOfMass() const
{
    return( _com );
}

void MotionState::setScale( const vsg::vec3& scale )
{
    _scale = scale;
    resetTransform();
}

vsg::vec3 MotionState::getScale() const
{
    return( _scale );
}

MotionStateCallbackList& MotionState::getCallbackList()
{
    return( _mscl );
}



void MotionState::resetTransform()
{
    // Divide the center of mass by the scale, concatenate it with the parent transform,
    // then call setWorldTransform.
    // This creates the initial model-to-world transform for both the collision shape /
    // rigid body and the VSG visual representation.
    const vsg::vec3 cs( _com[0]*_scale[0], _com[1]*_scale[1], _com[2]*_scale[2] );
    vsg::mat4 csMat = vsg::translate( cs );
    setWorldTransform( vsgbCollision::asBtTransform( csMat * vsg::mat4(_parentTransform) ) );
}


void MotionState::registerTripleBuffer( vsgbDynamics::TripleBuffer* tb )
{
    _tb = tb;
    _tbIndex = tb->reserve( sizeof( btScalar ) * 16 );
}

void MotionState::updateTripleBuffer( const char* addr )
{
    const btScalar* fAddr = reinterpret_cast< const btScalar* >( addr + _tbIndex );
    btTransform trans;
    trans.setFromOpenGLMatrix( fAddr );
    setWorldTransformInternal( trans );
}

bool TripleBufferMotionStateUpdate( vsgbDynamics::MotionStateList& msl, vsgbDynamics::TripleBuffer* tb )
{
    const char* addr = tb->beginRead();
    if( addr == nullptr )
        // No updated buffer is available. No valid data.
        return( false );

    MotionStateList::const_iterator it;
    for( it = msl.begin(); it != msl.end(); ++it )
        (*it)->updateTripleBuffer( addr );

    tb->endRead();
    return( true );
}


// vsgbDynamics
}
