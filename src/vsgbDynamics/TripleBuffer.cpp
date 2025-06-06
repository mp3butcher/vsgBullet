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

#include <vsgbDynamics/TripleBuffer.h>
#include <iostream>
#include <string>
#include <cstring>


namespace vsgbDynamics
{



TripleBuffer::TripleBuffer( unsigned int initialSize )
  : _writeAddress( nullptr ),
    _readAddress( nullptr )
{
    _currentSize = 0;
    resize( initialSize );
    _nextFree = 0;

    _status[ 0 ] = INVALID;
    _status[ 1 ] = INVALID;
    _status[ 2 ] = INVALID;
}
TripleBuffer::~TripleBuffer()
{
    std::scoped_lock< std::mutex > lock( _lock );

    // Warn if read or write is active.
    if( get( READ ) != -1 )
    {
        std::cerr << "~TripleBuffer: READ is active." << std::endl;
    }
    if( get( WRITE) != -1 )
    {
        std::cerr << "~TripleBuffer: WRITE is active." << std::endl;
    }

    _currentSize = 0;
    _nextFree = 0;
    delete[] _buf[ 0 ];
    delete[] _buf[ 1 ];
    delete[] _buf[ 2 ];
}

void
TripleBuffer::resize( unsigned int size )
{
    std::scoped_lock< std::mutex > lock( _lock );

    if( size > _currentSize )
    {
        reallocate( 0, size );
        reallocate( 1, size );
        reallocate( 2, size );
        _currentSize = size;
    }
}

unsigned int
TripleBuffer::reserve( unsigned int size, char* data )
{
    std::scoped_lock< std::mutex > lock( _lock );

    unsigned int newSize( _nextFree + size );
    if( newSize > _currentSize )
    {
        std::cerr << "TripleBuffer: Reallocating, current size " <<
            _currentSize << ", to new size " << newSize << std::endl;
        reallocate( 0, newSize );
        reallocate( 1, newSize );
        reallocate( 2, newSize );
        _currentSize = newSize;
    }

    unsigned int address = _nextFree;
    _nextFree += size;

    if( data != nullptr )
    {
        memcpy( _buf[ 0 ] + address, data, size );
        memcpy( _buf[ 1 ] + address, data, size );
        memcpy( _buf[ 2 ] + address, data, size );
    }

    return( address );
}

char*
TripleBuffer::beginWrite()
{
    std::scoped_lock< std::mutex > lock( _lock );
    debugDump( "beginWrite", std::cerr );

    // Get an INVALID buffer, change status to WRITE,
    // and return its address.
    const int invalidIdx( get( INVALID ) );
    if( invalidIdx == -1 )
    {
        std::cerr << "ERROR: beginWrite: No available INVALID buffer." << std::endl;
        return( nullptr );
    }
    _status[ invalidIdx ] = WRITE;
    _writeAddress = _buf[ invalidIdx ];
    return( _writeAddress );
}
char*
TripleBuffer::writeAddress()
{
    return( _writeAddress );
}

void
TripleBuffer::endWrite()
{
    std::scoped_lock< std::mutex > lock( _lock );
    debugDump( "endWrite", std::cerr );

    // If there is an UPDATED buffer, change its status to INVALID.
    const int updatedIdx( get( UPDATED ) );
    if( updatedIdx != -1 )
        _status[ updatedIdx ] = INVALID;

    // Get the WRITE buffer and change its status to UPDATED.
    const int writeIdx( get( WRITE ) );
    if( writeIdx == -1 )
    {
        std::cerr << "ERROR: endWrite: No available WRITE buffer." << std::endl;
        return;
    }
    _status[ writeIdx ] = UPDATED;
    _writeAddress = nullptr;
}


char*
TripleBuffer::beginRead()
{
    std::scoped_lock< std::mutex > lock( _lock );
    debugDump( "beginRead", std::cerr );

    // Get the UPDATED buffer, change its status to READ,
    // and return its address.
    // If no UPDATED buffer, return nullptr.
    const int updatedIdx( get( UPDATED ) );
    if( updatedIdx == -1 )
    {
        // Not an error if physics sim hasn't completed one update yet.
        std::cerr << "Warning: beginRead: No available UPDATED buffer." << std::endl;
        return( nullptr );
    }
    _status[ updatedIdx ] = READ;
    _readAddress = _buf[ updatedIdx ];
    return( _readAddress );
}
char*
TripleBuffer::readAddress()
{
    return( _readAddress );
}

void
TripleBuffer::endRead()
{
    std::scoped_lock< std::mutex > lock( _lock );
    debugDump( "endRead", std::cerr );

    // Get the READ buffer.
    // If there are no UPDATED buffers, change its status to UPDATED.
    // otherwise change its status to INVALID.
    const int readIdx( get( READ ) );
    if( readIdx == -1 )
    {
        std::cerr << "ERROR: endRead: No available READ buffer." << std::endl;
        return;
    }
    if( get( UPDATED ) == -1 )
        // Read thread is running faster than the write thread, so there
        // is no new data yet. Mark our current READ buffer as UPDATED,
        // so that we will read from it again (rather than toggling back
        // to last frame's data, or getting nullptr).
        _status[ readIdx ] = UPDATED;
    else
        _status[ readIdx ] = INVALID;
    _readAddress = nullptr;
}


void
TripleBuffer::debugDump( const std::string& msg, std::ostream& oStr ) const
{
    oStr << msg << ": " << _status[ 0 ];
    oStr << ", " << _status[ 1 ];
    oStr << ", " << _status[ 2 ] << std::endl;
}

int
TripleBuffer::get( BufferStatus status )
{
    // Must be locked.
    // (TBD test _lock and display error if not locked.)

    int idx;
    for( idx=0; idx<3; idx++ )
    {
        if( _status[ idx ] == status )
            return( idx );
    }
    return( -1 );
}

void
TripleBuffer::reallocate( unsigned int index, unsigned int size )
{
    char* newBuf = new char[ size ];
    if( newBuf == nullptr )
    {
        std::cerr << "TripleBuffer: reallocate failed to allocate memory." << std::endl;
        return;
    }
    if( _currentSize > 0 )
    {
        memcpy( newBuf, _buf[ index ], _currentSize );
        delete[] _buf[ index ];
    }
    _buf[ index ] = newBuf;
}


// vsgbDynamics
}
