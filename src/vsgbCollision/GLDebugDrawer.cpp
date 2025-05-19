/*************** <auto-copyright.pl BEGIN do not edit this line> **************
*
* osgBullet is (C) Copyright 2025 by Julien Valentin
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

#include <vsgbCollision/GLDebugDrawer.h>
#include <vsgbCollision/Chart.h>
#include <vsgbCollision/Utils.h>

#include <osg/Camera>
#include <osg/Geometry>
#include <osg/Point>
#include <osgText/Text>

#include <iostream>

#include <stdio.h> //printf debugging



namespace vsgbCollision
{


	////////////////////////////////////////////////////////////////////////////////
	GLDebugDrawer::GLDebugDrawer()
		: _enabled(true),
		_active(false),
		_textSize(1.f),
		_textStrings(0),
		_frame(0),
		_contacts(0)
	{
		setDebugMode(~0u);

		_group = new vsg::Group();
		_group->setName("Bullet Debug");

		_geode = new vsg::Geode();
		_geode->setName("Bullet pts, lns, tris, and text");
		_geode->setDataVariance(vsg::Object::DYNAMIC);
		{
			vsg::StateSet* ss = _geode->getOrCreateStateSet();
			ss->setMode(GL_LIGHTING, vsg::StateAttribute::OFF);
		}
		_group->addChild(_geode.get());


		_ptGeom = new vsg::Geometry;
		_ptGeom->setDataVariance(vsg::Object::DYNAMIC);
		_ptGeom->setUseDisplayList(false);
		_ptGeom->setUseVertexBufferObjects(false);
		{
			vsg::StateSet* ss = _geode->getOrCreateStateSet();
			ss->setMode(GL_POINT_SMOOTH, vsg::StateAttribute::ON);
			vsg::Point* point = new vsg::Point(20.);
			ss->setAttributeAndModes(point, vsg::StateAttribute::ON);
		}
		_geode->addDrawable(_ptGeom.get());

        _ptVerts = new vsg::vec3Array();
		_ptGeom->setVertexArray(_ptVerts);
		_ptColors = new vsg::Vec4Array();
		_ptGeom->setColorArray(_ptColors);
		_ptGeom->setColorBinding(vsg::Geometry::BIND_PER_VERTEX);


		_lnGeom = new vsg::Geometry;
		_lnGeom->setDataVariance(vsg::Object::DYNAMIC);
		_lnGeom->setUseDisplayList(false);
		_lnGeom->setUseVertexBufferObjects(false);
		_geode->addDrawable(_lnGeom.get());

        _lnVerts = new vsg::vec3Array();
		_lnGeom->setVertexArray(_lnVerts);
		_lnColors = new vsg::Vec4Array();
		_lnGeom->setColorArray(_lnColors);
		_lnGeom->setColorBinding(vsg::Geometry::BIND_PER_VERTEX);


		_triGeom = new vsg::Geometry;
		_triGeom->setDataVariance(vsg::Object::DYNAMIC);
		_triGeom->setUseDisplayList(false);
		_triGeom->setUseVertexBufferObjects(false);
		_geode->addDrawable(_triGeom.get());

        _triVerts = new vsg::vec3Array();
		_triGeom->setVertexArray(_triVerts);
		_triColors = new vsg::Vec4Array();
		_triGeom->setColorArray(_triColors);
		_triGeom->setColorBinding(vsg::Geometry::BIND_PER_VERTEX);


		// Initialize _textVec to display 10 text strings; resize later if necessary
		_textVec.resize(10);
		int idx;
		for (idx = 0; idx<10; idx++)
			_textVec[idx] = initText();


		// Set up for HUD
		_hudCam = new vsg::Camera;
		_hudCam->setRenderOrder(vsg::Camera::POST_RENDER);
		_hudCam->setClearMask(GL_DEPTH_BUFFER_BIT);
		_hudCam->setReferenceFrame(vsg::Transform::ABSOLUTE_RF);
		_hudCam->setViewMatrix(vsg::mat4::identity());
		_hudCam->setProjectionMatrixAsOrtho(0., 1., 0., 1., -1., 1.);
		_group->addChild(_hudCam.get());

		_chart = new vsgbCollision::Chart;
		_chart->createChart();
		_hudCam->addChild(_chart->get());
	}
	GLDebugDrawer::~GLDebugDrawer()
	{
		while (_group->getNumParents() > 0)
			_group->getParent(0)->removeChild(_group.get());
	}


	vsg::Node*
		GLDebugDrawer::getSceneGraph()
	{
		return(_group.get());
	}

	void GLDebugDrawer::setEnabled(bool enable)
	{
		if (!enable)
		{
			// Clear all geometry.
			_enabled = true;
			BeginDraw();
			// Disable any other rendering (like the chart)
			_group->setNodeMask(0x0);
		}
		else
		{
			_group->setNodeMask(0xffffffff);
		}
		_enabled = enable;
	}
	bool GLDebugDrawer::getEnabled() const
	{
		return(_enabled);
	}

	////////////////////////////////////////////////////////////////////////////////
	void GLDebugDrawer::drawLine(const btVector3& from, const btVector3& to, const btVector3& color)
	{
		if (!getEnabled())
			return;

		if (!_active)
		{
			vsg::notify(vsg::WARN) << "GLDebugDrawer: BeginDraw was not called." << std::endl;
			return;
		}

		// If the physics sim contains a plane, the AABB is rendered with
		// astronomical values. When this occurs in combination with OSG's
		// auto-compute near/far feature, the resulting far plane is very
		// distant, and consequently the near plane is pulled back to maintain
		// the default near/far ratio. As a result, the entire scene is clipped.
		// In this case, don't draw this line.
        vsg::vec3 osgFrom = vsgbCollision::asOsgvec3(from);
        vsg::vec3 osgTo = vsgbCollision::asOsgvec3(to);
		const double bigValue(10000.);
    /*	if ((vsg::absolute< double >(osgFrom[0]) > bigValue) ||
			(vsg::absolute< double >(osgFrom[1]) > bigValue) ||
			(vsg::absolute< double >(osgFrom[2]) > bigValue) ||
			(vsg::absolute< double >(osgTo[0]) > bigValue) ||
			(vsg::absolute< double >(osgTo[1]) > bigValue) ||
			(vsg::absolute< double >(osgTo[2]) > bigValue))
            return;*/
		_lnVerts->push_back(osgFrom);
		_lnVerts->push_back(osgTo);

		vsg::Vec4 c = vsgbCollision::asOsgVec4(color, 1.);
		_lnColors->push_back(c);
		_lnColors->push_back(c);
	}

	void GLDebugDrawer::drawSphere/*(const btVector3& p, btScalar radius, const btVector3& color)*/(btScalar radius, const btTransform& transform, const btVector3& color)
	{
		if (!getEnabled())
			return;

		if (!_active)
		{
			vsg::notify(vsg::WARN) << "GLDebugDrawer: BeginDraw was not called." << std::endl;
			return;
		}

		vsg::notify(vsg::ALWAYS) << "GLDebugDrawer::drawASphere NYI" << std::endl;
	}


	////////////////////////////////////////////////////////////////////////////////
	void GLDebugDrawer::drawTriangle(const btVector3& a, const btVector3& b, const btVector3& c, const btVector3& color, btScalar alpha)
	{
		if (!getEnabled())
			return;

		if (!_active)
		{
			vsg::notify(vsg::WARN) << "GLDebugDrawer: BeginDraw was not called." << std::endl;
			return;
		}

        _triVerts->push_back(vsgbCollision::asOsgvec3(a));
        _triVerts->push_back(vsgbCollision::asOsgvec3(b));
        _triVerts->push_back(vsgbCollision::asOsgvec3(c));

		vsg::Vec4 c4 = vsgbCollision::asOsgVec4(color, alpha);
		_triColors->push_back(c4);
		_triColors->push_back(c4);
		_triColors->push_back(c4);
	}
	////////////////////////////////////////////////////////////////////////////////
	void GLDebugDrawer::draw3dText(const btVector3& location, const char* textString)
	{
		if (!getEnabled())
			return;

		if ((_debugMode & btIDebugDraw::DBG_DrawText) == 0)
			return;

		if (!_active)
		{
			vsg::notify(vsg::WARN) << "GLDebugDrawer: BeginDraw was not called." << std::endl;
			return;
		}

		if (_textStrings == _textVec.size())
		{
			int oldSize(_textVec.size());
			int newSize(oldSize * 2);
			_textVec.resize(newSize);
			int idx;
			for (idx = oldSize; idx<newSize; idx++)
				_textVec[idx] = initText();
		}
		osgText::Text* text = _textVec[_textStrings].get();
		_textStrings++;

        text->setPosition(vsgbCollision::asOsgvec3(location));
		text->setText(std::string(textString));

		_geode->addDrawable(text);
	}
	////////////////////////////////////////////////////////////////////////////////
	void GLDebugDrawer::reportErrorWarning(const char* warningString)
	{
		if (!getEnabled())
			return;

		vsg::notify(vsg::WARN) << warningString << std::endl;
	}
	////////////////////////////////////////////////////////////////////////////////
	void GLDebugDrawer::drawContactPoint(const btVector3& pointOnB,
		const btVector3& normalOnB, btScalar distance,
		int lifeTime, const btVector3& color)
	{
		if (!getEnabled())
			return;

		if (!_active)
		{
			vsg::notify(vsg::WARN) << "GLDebugDrawer: BeginDraw was not called." << std::endl;
			return;
		}

		_contacts++;

        _ptVerts->push_back(vsgbCollision::asOsgvec3(pointOnB));
		_ptColors->push_back(vsgbCollision::asOsgVec4(color, 1.));

		btVector3 to = pointOnB + normalOnB*distance;
		const btVector3&from = pointOnB;

		drawLine(from, to, color);

		char buf[12];
		sprintf(buf, " %d", lifeTime);

		draw3dText(from, buf);
	}
	////////////////////////////////////////////////////////////////////////////////
	void GLDebugDrawer::BeginDraw()
	{
		if (!getEnabled())
			return;

		if (_ptVerts->size() > 0)
		{
			_ptGeom->removePrimitiveSet(0);
			_ptVerts->clear();
			_ptColors->clear();
		}

		if (_lnVerts->size() > 0)
		{
			_lnGeom->removePrimitiveSet(0);
			_lnVerts->clear();
			_lnColors->clear();
		}

		if (_triVerts->size() > 0)
		{
			_triGeom->removePrimitiveSet(0);
			_triVerts->clear();
			_triColors->clear();
		}

		if (_geode->getNumDrawables() > 3)
			_geode->removeDrawables(3, _geode->getNumChildren());
		_textStrings = 0;

		_contacts = 0;

		_active = true;
	}
	////////////////////////////////////////////////////////////////////////////////
	void GLDebugDrawer::EndDraw()
	{
		if (!getEnabled())
			return;

		_active = false;

		if (_ptVerts->size())
			_ptGeom->addPrimitiveSet(new vsg::DrawArrays(GL_POINTS, 0, _ptVerts->size()));
		if (_lnVerts->size())
			_lnGeom->addPrimitiveSet(new vsg::DrawArrays(GL_LINES, 0, _lnVerts->size() ));
		if (_triVerts->size())
			_triGeom->addPrimitiveSet(new vsg::DrawArrays(GL_TRIANGLES, 0, _triVerts->size()));
		_ptGeom->getVertexArray()->dirty();
		_lnGeom->getVertexArray()->dirty();
		_triGeom->getVertexArray()->dirty();
		_chart->setValue(_frame, _contacts);
		_frame++;
	}
	////////////////////////////////////////////////////////////////////////////////

	void GLDebugDrawer::setDebugMode(int debugMode)
	{
		_debugMode = debugMode;
	}
	int GLDebugDrawer::getDebugMode() const
	{
		return(_debugMode);
	}

	void GLDebugDrawer::setTextSize(const float size)
	{
		_textSize = size;
	}
	float GLDebugDrawer::getTextSize() const
	{
		return(_textSize);
	}

	osgText::Text*
		GLDebugDrawer::initText()
	{
		//vsg::ref_ptr<osgText::Text> text = new osgText::Text;
		osgText::Text* text = new osgText::Text;
		text->setDataVariance(vsg::Object::DYNAMIC);
		text->setFont("fonts/arial.ttf");
		text->setColor(vsg::Vec4(1., 1., 1., 1.));
		text->setCharacterSize(_textSize);
		text->setAxisAlignment(osgText::Text::SCREEN);

		//return( text.release() );
		return text;
	}


	// vsgbCollision
}
