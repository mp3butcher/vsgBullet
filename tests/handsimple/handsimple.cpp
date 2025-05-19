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

#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osgGA/StateSetManipulator>
#include <osgGA/TrackballManipulator>

#include <vsgbInteraction/HandNode.h>
#include <vsgbInteraction/HandTestEventHandler.h>
#include <vsgbCollision/Version.h>



int main( int argc, char** argv )
{
    osg::notify( osg::ALWAYS ) << vsgbCollision::getVersionString() << std::endl;

    osg::Group* root = new osg::Group;

    if( argc > 1 )
        root->addChild( osgDB::readNodeFile( argv[ 1 ] ) );

    osg::ref_ptr< vsgbInteraction::HandNode > hn = new vsgbInteraction::HandNode( nullptr, vsgbInteraction::HandNode::LEFT, 18. );
    root->addChild( hn.get() );

    hn->setPosition( osg::Vec3( 5., 0., 0. ) );

    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow( 10, 30, 960, 600 );
    viewer.setSceneData( root );
    viewer.setCameraManipulator( new osgGA::TrackballManipulator );
    viewer.addEventHandler( new vsgbInteraction::VirtualHandTestEventHandler( hn.get() ) );
    viewer.addEventHandler( new osgGA::StateSetManipulator( viewer.getCamera()->getOrCreateStateSet() ) );

    viewer.run();

    return 0;
}
