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

#include <vsgbDynamics/TripleBuffer.h>
#include <OpenThreads/Thread>
#include <osg/Timer>
#include <iostream>


/* \cond */
class WriteThread : public OpenThreads::Thread
{
public:
    WriteThread( vsgbDynamics::TripleBuffer* tb )
      : _tb( tb ),
        _count( 0 )
    {}

    virtual void run()
    {
        while( !testCancel() )
        {
            char* addr = _tb->beginWrite();
            OpenThreads::Thread::microSleep( 10000 );
            unsigned int* iAddr = reinterpret_cast< unsigned int* >( addr );
            *iAddr = _count++;
            _tb->endWrite();
        }
    }

protected:
    vsgbDynamics::TripleBuffer* _tb;
    unsigned int _count;
};

class ReadThread : public OpenThreads::Thread
{
public:
    ReadThread( vsgbDynamics::TripleBuffer* tb )
      : _tb( tb )
    {}

    virtual void run()
    {
        while( !testCancel() )
        {
            char* addr = _tb->beginRead();
            if( addr == nullptr )
                continue;

            unsigned int* iAddr = reinterpret_cast< unsigned int* >( addr );
            std::cout << _timer.time_s() << ": " << *iAddr << std::endl;
            _tb->endRead();
        }
    }

protected:
    vsgbDynamics::TripleBuffer* _tb;
    osg::Timer _timer;
};
/* \endcond */



int
main( int argc, char * argv[] )
{
    vsgbDynamics::TripleBuffer tb;
    WriteThread wt( &tb );
    ReadThread rt( &tb );

    wt.start();
    rt.start();

    osg::Timer timer;
    while( timer.time_s() < 2.0 )
    {
        OpenThreads::Thread::microSleep( 100 );
    }

    rt.cancel();
    wt.cancel();
    rt.join();
    wt.join();
}
