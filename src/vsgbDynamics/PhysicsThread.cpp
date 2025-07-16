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


#include <condition_variable>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace vsgbDynamics
{

class Barrier {
public:
    // Construct barrier for use with num threads.
    Barrier(std::size_t num)
        : num_threads(num),
        wait_count(0),
        instance(0),
        mut(),
        cv()
    {
        if (num == 0) {
            throw std::invalid_argument("Barrier thread count cannot be 0");
        }
    }

    // disable copying of barrier
    Barrier(const Barrier&) = delete;
    Barrier& operator =(const Barrier&) = delete;

    // This function blocks the calling thread until
    // all threads (specified by num_threads) have
    // called it. Blocking is achieved using a
    // call to condition_variable.wait().
    void wait() {
        std::unique_lock<std::mutex> lock(mut); // acquire lock
        std::size_t inst = instance; // store current instance for comparison
            // in predicate

        if (++wait_count == num_threads) { // all threads reached barrier
            wait_count = 0; // reset wait_count
            instance++; // increment instance for next use of barrier and to
                // pass condition variable predicate
            cv.notify_all();
        } else { // not all threads have reached barrier
            cv.wait(lock, [this, &inst]() { return instance != inst; });
            // NOTE: The predicate lambda here protects against spurious
            //       wakeups of the thread. As long as this->instance is
            //       equal to inst, the thread will not wake.
            //       this->instance will only increment when all threads
            //       have reached the barrier and are ready to be unblocked.
        }
    }
private:
    std::size_t num_threads; // number of threads using barrier
    std::size_t wait_count; // counter to keep track of waiting threads
    std::size_t instance; // counter to keep track of barrier use count
    std::mutex mut; // mutex used to protect resources
    std::condition_variable cv; // condition variable used to block threads
};


PhysicsThread::PhysicsThread( btDynamicsWorld* bw, vsgbDynamics::TripleBuffer* tb )
    : _timeStep( btScalar( 0.0 ) ),
    _bw( bw ),
    _stopped( true ),
    _pauseCount( 0 ),
    _tb( tb )//,
    //_pauseGate(new Barrier(10))
{
}
PhysicsThread::~PhysicsThread()
{
    if(_delegate)
        delete _delegate;
}

void
PhysicsThread::setTimeStep( btScalar timeStep )
{
    _timeStep = timeStep;
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

    _lastTime = vsg::clock::now();

    std::cerr << "PhysicsThread: Starting" << std::endl;

    while( !isStopping() )
    {
        currentTime = vsg::clock::now();
        const btScalar deltaTime =
            ( ( _timeStep > 0.0 ) ?
                 _timeStep :
                 ( btScalar )( std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - _lastTime).count() * 0.001f ) );

        bool localPause;
        {
            std::scoped_lock< std::mutex > lock( _pauseMutex );
            localPause = (_pauseCount > 0);
        }
        if( localPause )
        {
            std::cerr << "PT: Pausing..." << std::endl;
            // Wait to be released.

            std::unique_lock lk(_pauseGate);
            cv.wait(lk, [this, &currentTime, deltaTime]{
                //  _pauseGate.lock();
                std::cerr << "PT: Released." << std::endl;

                // We were just released. Reset the block.
                //_pauseGate->release();

                // Yawn! That was a nice nap. What time is it?
                currentTime = vsg::clock::now();
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
                return true;
            });
        }else

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

    if( block )
    {
        // Give physics thread a change to hit the gate.
        /* YieldCurrentThread();
        using namespace std::chrono_literals;
        // Block until physics thread hits the gate.
        while( !( isPaused() ) )
        std::this_thread::sleep_for(std::chrono::milliseconds(10));*/
    }
    else if( unblock )
        //    _pauseGate->release();
        cv.notify_all();
}

bool
PhysicsThread::isPaused() const
{
    std::scoped_lock< std::mutex > lock( _pauseMutex );
    return( _pauseCount == 1);// _pauseGate.numThreadsCurrentlyBlocked() > 0 );
}
// vsgbDynamics
}
