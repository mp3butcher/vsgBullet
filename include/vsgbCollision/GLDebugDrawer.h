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

#ifndef __VSGBCOLLISION_GL_DEBUG_DRAWER_H__
#define __VSGBCOLLISION_GL_DEBUG_DRAWER_H__ 1

#include <vsgbCollision/Export.h>
#include <LinearMath/btIDebugDraw.h>
#include <vsgbCollision/Chart.h>



namespace vsgbCollision
{


/** \class GLDebugDrawer GLDebugDrawer.h <vsgbCollision/GLDebugDrawer.h>
\brief Debug utility to render Bullet collision shapes.

The class visualizes Bullet collision shapes. Use it as a debugging aid
to ensure that Bullet collision shapes and VSG geometry are synchronized.
*/
class VSGBCOLLISION_EXPORT GLDebugDrawer : public btIDebugDraw
{
public:
    GLDebugDrawer();
    virtual ~GLDebugDrawer();

    vsg::Node* getSceneGraph();

    void setEnabled( bool enable );
    bool getEnabled() const;

    virtual void	drawLine( const btVector3& from,const btVector3& to,const btVector3& color );
   // virtual void	drawSphere( const btVector3& p, btScalar radius, const btVector3& color );
	virtual void	drawSphere(btScalar radius, const btTransform& transform, const btVector3& color);

    virtual void	drawTriangle(const btVector3& a,const btVector3& b,const btVector3& c,const btVector3& color,btScalar alpha);
    virtual void	drawContactPoint(const btVector3& PointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color);
    virtual void	reportErrorWarning(const char* warningString);
    virtual void	draw3dText(const btVector3& location,const char* textString);

    virtual void	setDebugMode(int debugMode);
    virtual int		getDebugMode() const;

    void EndDraw();
    void BeginDraw();

    void setTextSize( const float size );
    float getTextSize() const;

private:
    vsg::ref_ptr<vsg::Text> initText();

	int _debugMode;
    bool _enabled;
    bool _active;
    float _textSize;

    vsg::ref_ptr< vsg::Group > _group;

    // Points, lines, triangles, and text
    vsg::ref_ptr< vsg::Group > _geode;
    vsg::ref_ptr< vsg::Geometry > _ptGeom;
    vsg::ref_ptr< vsg::Geometry > _lnGeom;
    vsg::ref_ptr< vsg::Geometry > _triGeom;

    typedef std::vector< vsg::ref_ptr< vsg::Text > > TextVec;
    TextVec _textVec;
    unsigned int _textStrings;

    vsg::vec3Array* _ptVerts;
    vsg::vec4Array* _ptColors;

    vsg::vec3Array* _lnVerts;
    vsg::vec4Array* _lnColors;

    vsg::vec3Array* _triVerts;
    vsg::vec4Array* _triColors;

    // HUD display
    vsg::ref_ptr< vsg::Camera > _hudCam;

    vsg::ref_ptr< vsgbCollision::Chart > _chart;
    int _frame;
    int _contacts;
};


// vsgbCollision
}


// __VSGBCOLLISION_GL_DEBUG_DRAWER_H__
#endif
