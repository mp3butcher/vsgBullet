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

#include <vsgbDynamics/PhysicsThread.h>
#include <vsgbDynamics/TripleBuffer.h>

#include <btBulletDynamicsCommon.h>

namespace vsgbDynamics
{

PhysicsThread::PhysicsThread( btDynamicsWorld* bw, vsgbDynamics::TripleBuffer* tb )
    : _timeStep( btScalar( 0.0 ) ),
    _bw( bw ),
    _stopped( true ),
    _pauseCount( 0 ),
    _tb( tb )
{
}
PhysicsThread::~PhysicsThread()
{
}

void
PhysicsThread::setTimeStep( btScalar timeStep )
{
    _timeStep = timeStep;
    //_lastTime = _timer.tick();
}
btScalar
PhysicsThread::getTimeStep() const
{
    return( _timeStep );
}

void
PhysicsThread::run()
{
    _stopped = false;

    vsg::clock::time_point currentTime;

    //_timer.setStartTick();
    _lastTime = vsg::clock::now();//timer.tick();

    std::cerr << "PhysicsThread: Starting" << std::endl;

    while( !isStopping() )
    {
        currentTime =vsg::clock::now();
        const btScalar deltaTime =
            ( ( _timeStep > 0.0 ) ?
                 _timeStep :
                 ( btScalar )( std::chrono::duration_cast<std::chrono::milliseconds>(currentTime-_lastTime).count() * 0.001f ) );

        bool localPause;
        {
            std::scoped_lock< std::mutex > lock( _pauseMutex );
            localPause = (_pauseCount > 0);
        }
        if( localPause )
        {
            std::cerr << "PT: Pausing..." << std::endl;
            // Wait to be released.
            // _pauseGate.block();
            std::cerr << "PT: Released." << std::endl;

            // We were just released. Reset the block.
            // _pauseGate.release();

            // Yawn! That was a nice nap. What time is it?
            currentTime = vsg::clock::now();
        }

        if( _tb != nullptr )
        {
            // Run with triple buffering.
            _tb->beginWrite();
            _bw->stepSimulation( deltaTime );
            _tb->endWrite();
        }
        else
        {
            // Run normally. (Not sure if this is useful.)
            _bw->stepSimulation( deltaTime );
        }

        _lastTime = currentTime;
    }

    std::cerr << "PhysicsThread: Stopping" << std::endl;
}


void PhysicsThread::start()
{
    _delegate = new std::thread( [&](){run();});
}

void PhysicsThread::setProcessorAffinity(int i)
{
    // Create a cpu_set_t object representing a set of CPUs. Clear it and mark
    // only CPU i as set.
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(i, &cpuset);
    int rc = pthread_setaffinity_np(_delegate->native_handle(),
                                    sizeof(cpu_set_t), &cpuset);
    if (rc != 0) {
        std::cerr << "Error calling pthread_setaffinity_np: " << rc << "\n";
    }
}
void PhysicsThread::stopPhysics()
{
    std::scoped_lock< std::mutex > lock( _stopMutex );
    _stopped = true;
}

bool PhysicsThread::isStopping() const
{
    std::scoped_lock< std::mutex > lock( _stopMutex );
    return( _stopped );
}

void PhysicsThread::pause( bool pause )
{
    if( _stopped )
        return;

    bool block( false );
    bool unblock( false );
    {
        std::scoped_lock< std::mutex >  lock( _pauseMutex );
        if( pause )
        {
            _pauseCount++;
            block = ( _pauseCount == 1 );
        }
        else
        {
            _pauseCount--;
            unblock = ( _pauseCount == 0 );
        }
    }

    /* if( block )
    {
        // Give physics thread a change to hit the gate.
        YieldCurrentThread();

        // Block until physics thread hits the gate.
        while( !( isPaused() ) )

        using namespace std::chrono_literals;
        std::this_thread::sleep_for(10ms);
    }
    else if( unblock )
        _pauseGate.release();*/
}
bool
PhysicsThread::isPaused() const
{
    std::scoped_lock< std::mutex > lock( _pauseMutex );
    return( true);// _pauseGate.numThreadsCurrentlyBlocked() > 0 );
}
// vsgbDynamics
}
