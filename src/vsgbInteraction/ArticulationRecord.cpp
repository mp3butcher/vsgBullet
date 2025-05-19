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

#include <vsgbInteraction/ArticulationRecord.h>


namespace vsgbInteraction
{


ArticulationRecord::ArticulationRecord()
    :vsg::Inherit<vsg::Object,ArticulationRecord>(),
    _version( 2 )
{
}
ArticulationRecord::ArticulationRecord( const vsg::dvec3& axis, const vsg::dvec3& pivotPoint )
    :vsg::Inherit<vsg::Object,ArticulationRecord>(),
    _axis( axis ),
    _pivotPoint( pivotPoint ),
    _version( 2 )
{
}

ArticulationRecord::ArticulationRecord( const ArticulationRecord& rhs, const vsg::CopyOp& copyop )
    : vsg::Inherit<vsg::Object,ArticulationRecord>(rhs,copyop),
    _axis( rhs._axis ),
    _pivotPoint( rhs._pivotPoint ),
    _version( rhs._version )
{
}


ArticulationRecord::~ArticulationRecord()
{
}


// vsgbInteraction
}
