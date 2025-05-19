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

#include <vsgbCollision/Chart.h>
#include <vsg/state/DescriptorBuffer.h>
#include <vsg/nodes/Geometry.h>
#include <vsg/nodes/Group.h>


namespace vsgbCollision
{

////////////////////////////////////////////////////////////////////////////////
Chart::Chart()
  : _x( .05f ),
    _y( .05f ),
    _w( .25f ),
    _h( .1f ),
    _yScale( 30.f ),
    _texW( 256 ),
    _bg( vsg::vec4( 0.f, 0.f, 0.f, .33f ) ),
    _fg( vsg::vec4( 1.f, 1.f, 1.f, .5f ) ),
    _overrun( vsg::vec4( 1.f, 0.1f, 0.1f, .5f ) )
{
    auto foreground=vsg::vec4Array::create(1);
    foreground->at(0)=_fg;
    auto background=vsg::vec4Array::create(1);
    background->at(0)=_bg;
    //uint32_t in_dstBinding, uint32_t in_dstArrayElement, VkDescriptorType in_descriptorType
    _fgUniform = vsg::DescriptorBuffer::create( foreground,0);//"foreground", _fg,VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER );
    _bgUniform = vsg::DescriptorBuffer::create( background,1 );
}
////////////////////////////////////////////////////////////////////////////////
Chart::~Chart()
{
    // We create the Image with USE_NEW_DELETE so we do *not* need
    // to explitly delete this image data.
    //delete[] _xValues;
}
////////////////////////////////////////////////////////////////////////////////
void
Chart::setValue( int idx, float value )
{
    if( idx >= _texW )
    {
        const int n( idx / _texW );
        idx = idx - n * _texW;
    }
    _xValues[ idx ] = value / _yScale;
    // Unfortunately, VSG doesn't support just updating a single value.
    // Dirty the whole image.
    _image->syncModifiedCount(0);
}
////////////////////////////////////////////////////////////////////////////////
void Chart::setBackgroundColor( vsg::vec4& bgColor )
{
    _bg = bgColor;
    auto f=_bgUniform->bufferInfoList.at(0).cast<vsg::vec4Array>();
    f->at(0)= _bg ;
    f->dirty();

}
////////////////////////////////////////////////////////////////////////////////
void Chart::setForegroundColor( vsg::vec4& fgColor )
{
    _fg = fgColor;
    auto f=_fgUniform->bufferInfoList.at(0).cast<vsg::vec4Array>();
    f->at(0)= _fg ;
    f->dirty();
}
////////////////////////////////////////////////////////////////////////////////
void Chart::setChartLocationAndSize( float x, float y, float w, float h )
{
    _x = x;
    _y = y;
    _w = w;
    _h = h;

    if( _verts.valid() )
    {
        (*_verts)[ 0 ] = vsg::vec3( _x, _y, 0. );
        (*_verts)[ 1 ] = vsg::vec3( _x+_w, _y, 0. );
        (*_verts)[ 2 ] = vsg::vec3( _x+_w, _y+_h, 0. );
        (*_verts)[ 3 ] = vsg::vec3( _x, _y+_h, 0. );
    }
}
////////////////////////////////////////////////////////////////////////////////
void Chart::createChart()
{
    // Init the 1D array of texture values. This will be the 1D array sampled in the fragment shader.
    _xValues = new float[ _texW ];
    int idx;
    for( idx=0; idx<_texW; idx++ )
        _xValues[ idx ] = 0.f;
    
    _chart = new vsg::Group;
    _geom = new vsg::Geometry;

    //_geom->setDataVariance( vsg::Object::DYNAMIC );
  /*  {
        // Create stateset to draw the chart quad. Just a single texture mapped quad with blending and no lighting, depth test ALWAYS.
        vsg::StateSet* ss = _geom->getOrCreateStateSet();
        ss->setMode( GL_LIGHTING, vsg::StateAttribute::OFF );
        ss->setAttributeAndModes( new vsg::BlendFunc );
        ss->setAttributeAndModes( new vsg::Depth( vsg::Depth::ALWAYS ) );
        
        // Load vert and frag shaders.
        vsg::ref_ptr< vsg::Program > program = new vsg::Program();
        ss->setAttribute( program.get(),
                         vsg::StateAttribute::ON | vsg::StateAttribute::PROTECTED );
        
        std::string shaderName = osgDB::findDataFile( "hud.vs" );
        if( !shaderName.empty() )
        {
            vsg::ref_ptr< vsg::Shader > vertShader = vsg::Shader::readShaderFile( vsg::Shader::VERTEX, shaderName );
            program->addShader( vertShader.get() );
        }
        else
        {
            std::cerr << "Chart::createChart(): Cannot find hud.vs." << std::endl;
        }
        
        shaderName = osgDB::findDataFile( "hud.fs" );
        if( !shaderName.empty() )
        {
            vsg::ref_ptr< vsg::Shader > fragShader = vsg::Shader::readShaderFile( vsg::Shader::FRAGMENT, shaderName );
            program->addShader( fragShader.get() );
        }
        else
        {
            std::cerr << "Chart::createChart(): Cannot find hud.fs." << std::endl;
        }
        
        // Uniforms for color values and 1D texture width.
        ss->addUniform( _fgUniform.get() );
        ss->addUniform( _bgUniform.get() );
        ss->addUniform( new vsg::Uniform( "overrun", _overrun ) );
        ss->addUniform( new vsg::Uniform( "texwidth", _texW ) );
        
        // Create a 1D texture object for the values array and create a corresponding sampler uniform.
        _image = new vsg::Image;
        _image->setImage( _texW, 1, 1, GL_INTENSITY32F_ARB, GL_RED, GL_FLOAT,
                         (unsigned char*) _xValues, vsg::Image::USE_NEW_DELETE );
        vsg::Texture1D* texVal = new vsg::Texture1D;
        texVal->setImage( _image.get() );
        texVal->setFilter( vsg::Texture::MIN_FILTER, vsg::Texture::NEAREST );
        texVal->setFilter( vsg::Texture::MAG_FILTER, vsg::Texture::NEAREST );
        texVal->setWrap( vsg::Texture::WRAP_S, vsg::Texture::REPEAT );
        ss->setTextureAttributeAndModes( 0, texVal );
        
        ss->addUniform( new vsg::Uniform( "texVal", 0 ) );
    }
    _geode->addDrawable( _geom.get() );
    
    _verts = new vsg::vec3Array;
    _verts->resize( 4 );
    _geom->setVertexArray( _verts.get() );
    _tc = new vsg::vec2Array;
    _tc->resize( 4 );
    _geom->setTexCoordArray( 0, _tc.get() );
    
    
    (*_verts)[ 0 ] = vsg::vec3( _x, _y, 0. );
    (*_verts)[ 1 ] = vsg::vec3( _x+_w, _y, 0. );
    (*_verts)[ 2 ] = vsg::vec3( _x+_w, _y+_h, 0. );
    (*_verts)[ 3 ] = vsg::vec3( _x, _y+_h, 0. );
    (*_tc)[ 0 ] = vsg::vec2( 0., 0. );
    (*_tc)[ 1 ] = vsg::vec2( 1., 0. );
    (*_tc)[ 2 ] = vsg::vec2( 1., 1. );
    (*_tc)[ 3 ] = vsg::vec2( 0., 1. );
    
    _geom->addPrimitiveSet( new vsg::DrawArrays( GL_QUADS, 0, 4 ) );*/
}
////////////////////////////////////////////////////////////////////////////////


// vsgbCollision
}
