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

#ifndef __VSGCOLLISION_CHART_H__
#define __VSGCOLLISION_CHART_H__ 1


#include <vsgbCollision/Export.h>

#include <vsg/state/DescriptorBuffer.h>
#include <vsg/nodes/Geometry.h>
#include <vsg/state/Image.h>



namespace vsgbCollision
{


/** \class Chart Chart.h <vsgbCollision/Chart.h>
\brief Used by GLDebugDrawer to render a 2D HUD graph of intersection points.

*/
class VSGBCOLLISION_EXPORT Chart : public vsg::Inherit<vsg::Object, Chart>
{
public:
    ///Constructor
    Chart();
    ///Destructor
    ~Chart();

    ///Set the value to be plotted
    ///\param idx The index into the data array for the value storage
    ///\param value The value to be plotted
    void setValue( int idx, float value );
    ///Set the background color
    ///\param bgColor The background color
    void setBackgroundColor( vsg::vec4& bgColor );
    ///Set the foreground color
    ///\param fgColor The foreground color
    void setForegroundColor( vsg::vec4& fgColor );
    ///Set the starting location and size values
    ///\param x The x starting location
    ///\param y The y starting location
    ///\param w The width of the chart
    ///\param h The hieght of the chart
    void setChartLocationAndSize( float x, float y, float w, float h );
    ///Get the chart
    ///\return The vsg::Geode for the chart
    inline vsg::Group* get() const { return _chart.get();}
    ///Create the chart for the data display
    void createChart();

protected:
    float _x, _y, _w, _h;
    float _yScale;
    int _texW;

    float* _xValues;
    vsg::ref_ptr< vsg::Image > _image;

    vsg::ref_ptr< vsg::Geometry > _geom;
    vsg::ref_ptr< vsg::Group > _chart;

    vsg::ref_ptr< vsg::vec3Array > _verts;
    vsg::ref_ptr< vsg::vec2Array > _tc;
    vsg::vec4 _bg, _fg, _overrun;

    vsg::ref_ptr< vsg::DescriptorBuffer > _fgUniform, _bgUniform;
};


// vsgbDynamics
}


// __VSGCOLLISION_CHART_H__
#endif
