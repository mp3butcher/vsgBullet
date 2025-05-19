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

#ifndef __VSGBINTERACTION_ARTICULATION_RECORD_H__
#define __VSGBINTERACTION_ARTICULATION_RECORD_H__


#include <vsgbInteraction/Export.h>
#include <vsg/core/Object.h>
#include <vsg/core/Inherit.h>

#include <vsg/maths/vec3.h>


namespace vsgbInteraction
{


/** \class ArticulationRecord ArticulationRecord.h <vsgbInteraction/ArticulationRecord.h>
\brief Support for HandNode articulations.

handpreprocess2 stores transformation axis and pivot point
data in this record, and attaches it as UserData to each
MatrixTransform node. HandNode loads this data and stores it
in ArticulationInfo to control how each articulation
transforms its subgraph.
*/
class VSGBINTERACTION_EXPORT ArticulationRecord : public vsg::Inherit<vsg::Object,ArticulationRecord>
{
public:
    ArticulationRecord();
    ArticulationRecord( const vsg::dvec3& axis, const vsg::dvec3& pivotPoint );

    ArticulationRecord( const ArticulationRecord& rhs, const vsg::CopyOp& copyop={} );

    vsg::dvec3 _axis;
    vsg::dvec3 _pivotPoint;

    unsigned int _version;

protected:
    ~ArticulationRecord();
};


// vsgbInteraction
}


// __VSGBINTERACTION_ARTICULATION_RECORD_H__
#endif
