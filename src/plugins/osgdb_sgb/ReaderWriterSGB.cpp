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

#include "ReaderWriterSGB.h"
/*
#include <vsgbDynamics/PhysicsState.h>
#include <osgwTools/RefID.h>*/

#include <osgDB/ReaderWriter>
#include <osgDB/FileUtils>
#include <osgDB/FileNameUtils>
#include <osgDB/Registry>
#include <osgDB/Input>
#include <osgDB/Output>

#include <iostream>
#include <string>


ReaderWriterSGB::ReaderWriterSGB()
{
    supportsExtension( "sgb", "osgbBullet physics simulation state." );

    supportsOption( "TBD", "To Be Determined." );
}
ReaderWriterSGB::~ReaderWriterSGB()
{
}

const char*
ReaderWriterSGB::className() const
{
    return "osgbBullet physics simulation state";
}

osgDB::ReaderWriter::ReadResult
ReaderWriterSGB::readObject( const std::string& fileName, const Options* options ) const
{
    const std::string ext = osgDB::getFileExtension( fileName );
    if( !acceptsExtension( ext ) )
    {
        std::cerr << "OSGB: Unsupported extension " << fileName << std::endl;
        return( osgDB::ReaderWriter::ReadResult::FILE_NOT_HANDLED );
    }

    const std::string fullName = osgDB::findDataFile( fileName );
    if( fullName.empty() )
        return( osgDB::ReaderWriter::ReadResult::FILE_NOT_FOUND );

    std::ifstream ifs( fullName.c_str() );
    if( !ifs.good() )
        return( osgDB::ReaderWriter::ReadResult::ERROR_IN_READING_FILE );

    osgDB::Input fr;
    fr.attach( &ifs );


    unsigned int numEntries( 0 );
    if( fr.matchSequence( "Physics data entries %i" ) )
    {
        fr[3].getUInt( numEntries );
        fr+=4;
    }
    if( numEntries == 0 )
        return( osgDB::ReaderWriter::ReadResult::ERROR_IN_READING_FILE );
    std::cerr << "OSGB: " << numEntries << " entries." << std::endl;

   /* vsg::ref_ptr< vsgbDynamics::PhysicsState > ps = new vsgbDynamics::PhysicsState;
    unsigned int idx;
    for( idx = 0; idx < numEntries; idx++ )
    {
        std::cerr << "OSGB: Reading entry " << idx << std::endl;

        vsg::ref_ptr< osgwTools::RefID > rid = static_cast< osgwTools::RefID* >( fr.readObject() );
        if( rid == nullptr )
        {
            std::cerr << "OSGB: Failed, rid " << rid.get() << std::endl;
            return( osgDB::ReaderWriter::ReadResult::ERROR_IN_READING_FILE );
        }
        vsg::ref_ptr< vsgbDynamics::PhysicsData > pd = static_cast< vsgbDynamics::PhysicsData* >( fr.readObject() );
        if( pd == nullptr )
        {
            std::cerr << "OSGB: Failed, pd " << pd.get() << std::endl;
            return( osgDB::ReaderWriter::ReadResult::ERROR_IN_READING_FILE );
        }
        ps->addPhysicsData( rid.get(), pd.get() );

        std::cerr << "OSGB: Finished reading entry " << idx << std::endl;
    }


    return( ps.release() );*/
    return( osgDB::ReaderWriter::ReadResult::FILE_LOADED );
}

osgDB::ReaderWriter::WriteResult
ReaderWriterSGB::writeObject( const vsg::Object& obj, const std::string& fileName, const Options* options ) const
{
    osgDB::Output fw( fileName.c_str() );
    if( !fw.good() )
        return( osgDB::ReaderWriter::WriteResult::ERROR_IN_WRITING_FILE );

/*    const vsgbDynamics::PhysicsState* ps = dynamic_cast< const vsgbDynamics::PhysicsState* > ( &obj );
    if( ps == nullptr )
        return( osgDB::ReaderWriter::WriteResult::ERROR_IN_WRITING_FILE );


    fw << "Physics data entries " << ps->getNumEntries() << std::endl;
    ps->exportEntired( fw );*/

    return( osgDB::ReaderWriter::WriteResult::FILE_SAVED );
}


REGISTER_OSGPLUGIN( osgb, ReaderWriterSGB )
