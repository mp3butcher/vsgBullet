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

#include <vsgbCollision/Utils.h>
#include <vsg/maths/mat4.h>
#include <vsg/maths/quat.h>
#include <vsg/core/Array.h>
#include <LinearMath/btTransform.h>

using namespace vsgbCollision;

vsg::quat vsgbCollision::makeRotate(const vsg::vec3& from, const vsg::vec3& to)
{
    // This routine takes any vector as argument but normalized
    // vectors are necessary, if only for computing the dot product.
    // Too bad the API is that generic, it leads to performance loss.
    // Even in the case the 2 vectors are not normalized but same length,
    // the sqrt could be shared, but we have no way to know beforehand
    // at this point, while the caller may know.
    // So, we have to test... in the hope of saving at least a sqrt
    vsg::vec3 sourceVector = from;
    vsg::vec3 targetVector = to;

    float fromLen2 = length2(from);
    float fromLen;
    // normalize only when necessary, epsilon test
    if ((fromLen2 < 1.0-1e-7) || (fromLen2 > 1.0+1e-7)) {
        fromLen = sqrt(fromLen2);
        sourceVector /= fromLen;
    } else fromLen = 1.0;

    float toLen2 = length2(to);
    // normalize only when necessary, epsilon test
    if ((toLen2 < 1.0-1e-7) || (toLen2 > 1.0+1e-7)) {
        float toLen;
        // re-use fromLen for case of mapping 2 vectors of the same length
        if ((toLen2 > fromLen2-1e-7) && (toLen2 < fromLen2+1e-7)) {
            toLen = fromLen;
        }
        else toLen = sqrt(toLen2);
        targetVector /= toLen;
    }

    // Now let's get into the real stuff
    // Use "dot product plus one" as test as it can be re-used later on
    double dotProdPlus1 = 1.0 + vsg::dot(sourceVector , targetVector);

    // Check for degenerate case of full u-turn. Use epsilon for detection
    if (dotProdPlus1 < 1e-7) {
        // Get an orthogonal vector of the given vector
        // in a plane with maximum vector coordinates.
        // Then use it as quaternion axis with pi angle
        // Trick is to realize one value at least is >0.6 for a normalized vector.
        if (fabs(sourceVector.x) < 0.6f) {
            const double norm = sqrt(1.0 - sourceVector.x * sourceVector.x);
            return vsg::quat(0.0, sourceVector.z / norm,-sourceVector.y / norm,0.0);
        } else if (fabs(sourceVector.y) < 0.6f) {
            const double norm = sqrt(1.0 - sourceVector.y * sourceVector.y);
            return vsg::quat( -sourceVector.z / norm, 0.0, sourceVector.x / norm, 0.0);
        } else {
            const double norm = sqrt(1.0 - sourceVector.z * sourceVector.z);
            return vsg::quat( sourceVector.y / norm, -sourceVector.x / norm, 0.0, 0.0);
        }
    }
    else {
        // Find the shortest angle quaternion that transforms normalized vectors
        // into one other. Formula is still valid when vectors are colinear
        const double s = sqrt(0.5 * dotProdPlus1);
        vsg::vec3 tmp = vsg::cross(sourceVector, targetVector);
        tmp /= (2.0*s);
        return vsg::quat(tmp.x,tmp.y,tmp.z,s);
    }
}

// Make a rotation Quat which will rotate vec1 to vec2
// Generally take a dot product to get the angle between these
// and then use a cross product to get the rotation axis
// Watch out for the two special cases of when the vectors
// are co-incident or opposite in direction.
void makeRotate_original(const vsg::vec3& from, const vsg::vec3& to)
{
    const float epsilon = 0.0000001;

    float length1  = length(from);
    float length2  = length(to);

    // dot product vec1*vec2
  /*  float cosangle = from*to/(length1*length2);

    if ( fabs(cosangle - 1) < epsilon )
    {
        std::cerr<<"*** Quat::makeRotate(from,to) with near co-linear vectors, epsilon= "<<fabs(cosangle-1)<<std::endl;

        // cosangle is close to 1, so the vectors are close to being coincident
        // Need to generate an angle of zero with any vector we like
        // We'll choose (1,0,0)
        makeRotate( 0.0, 0.0, 0.0, 1.0 );
    }
    else
        if ( fabs(cosangle + 1.0) < epsilon )
        {
            // vectors are close to being opposite, so will need to find a
            // vector orthongonal to from to rotate about.
            Vec3d tmp;
            if (fabs(from.x())<fabs(from.y()))
                if (fabs(from.x())<fabs(from.z())) tmp.set(1.0,0.0,0.0); // use x axis.
                else tmp.set(0.0,0.0,1.0);
            else if (fabs(from.y())<fabs(from.z())) tmp.set(0.0,1.0,0.0);
            else tmp.set(0.0,0.0,1.0);

            Vec3d fromd(from.x(),from.y(),from.z());

            // find orthogonal axis.
            Vec3d axis(fromd^tmp);
            axis.normalize();

            _v[0] = axis[0]; // sin of half angle of PI is 1.0.
            _v[1] = axis[1]; // sin of half angle of PI is 1.0.
            _v[2] = axis[2]; // sin of half angle of PI is 1.0.
            _v[3] = 0; // cos of half angle of PI is zero.

        }
        else
        {
            // This is the usual situation - take a cross-product of vec1 and vec2
            // and that is the axis around which to rotate.
            Vec3d axis(from^to);
            float angle = acos( cosangle );
            makeRotate( angle, axis );
        }*/
}
// Convert a btTransform to a VSG Matrix
vsg::mat4 vsgbCollision::asVsgMatrix( const btTransform& t )
{
    btScalar ogl[ 16 ];
    t.getOpenGLMatrix( ogl );
    vsg::mat4 m( ogl );
    return m;
}
btTransform vsgbCollision::asBtTransform( const vsg::mat4& m )
{
    const vsg::mat4::value_type* oPtr = m.data();
    btScalar bPtr[ 16 ];
    int idx;
    for (idx=0; idx<16; idx++)
        bPtr[ idx ] = oPtr[ idx ];
    btTransform t;
    t.setFromOpenGLMatrix( bPtr );
    return t;
}
vsg::mat4 vsgbCollision::asVsgMatrix( const btMatrix3x3& m )
{
    btScalar f[ 9 ];
    m.getOpenGLSubMatrix( f );
    return( vsg::mat4(
        f[0], f[1], f[2], 0.,
        f[3], f[4], f[5], 0.,
        f[6], f[7], f[8], 0.,
        0., 0., 0., 1. ) );
}
btMatrix3x3 vsgbCollision::asBtMatrix3x3( const vsg::mat4& m )
{
    return( btMatrix3x3(
        m(0,0), m(0,1), m(0,2),
        m(1,0), m(1,1), m(1,2),
        m(2,0), m(2,1), m(2,2) ) );
}
vsg::vec3 vsgbCollision::asVsgVec3( const btVector3& v )
{
    return vsg::vec3( v.x(), v.y(), v.z() );
}
btVector3 vsgbCollision::asBtVector3( const vsg::vec3& v )
{
    return btVector3( v.x, v.y, v.z);
}
vsg::vec4 vsgbCollision::asVsgVec4( const btVector3& v, const double w )
{
    return vsg::vec4( v.x(), v.y(), v.z(), w );
}
vsg::vec4 vsgbCollision::asVsgVec4( const btVector4& v )
{
    return vsg::vec4( v.x(), v.y(), v.z(), v.w() );
}
btVector4 vsgbCollision::asBtVector4( const vsg::vec4& v )
{
    return btVector4( v.x, v.y, v.z, v.w );
}
btVector3* vsgbCollision::asBtVector3Array( const vsg::vec3Array* v )
{
    btVector3* out( new btVector3[ v->size() ] );

    btVector3* outPtr( out );
    vsg::vec3Array::const_iterator inPtr;
    for( inPtr=v->begin(); inPtr != v->end(); ++inPtr )
    {
        *outPtr++ = asBtVector3( *inPtr );
    }

    return( out );
}
bool vsgbCollision::disposeBtVector3Array( btVector3* array )
{
    if( array == nullptr )
        return( false );
    delete[] array;
    return( true );
}
vsg::vec3Array* asVsgVec3Array( const btVector3* v, const unsigned int size )
{
    vsg::ref_ptr< vsg::vec3Array > out = vsg::vec3Array::create(size);

    vsg::vec3Array::iterator outPtr;
    btVector3 const* inPtr = v;
    for( outPtr=out->begin(); outPtr != out->end(); ++outPtr )
    {
        *outPtr = asVsgVec3( *inPtr++ );
    }

    return( out.get() );
}
LocalBtVector3Array::LocalBtVector3Array( const vsg::vec3Array* v )
  : _btVector3( asBtVector3Array( v ) )
{
}
LocalBtVector3Array::~LocalBtVector3Array()
{
    disposeBtVector3Array( _btVector3 );
}
btVector3* LocalBtVector3Array::get()
{
    return( _btVector3 );
}
const btVector3* LocalBtVector3Array::get() const
{
    return( _btVector3 );
}
