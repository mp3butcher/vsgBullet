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

#ifndef __VSGBDYNAMICS_PHYSICSTHREAD_H__
#define __VSGBDYNAMICS_PHYSICSTHREAD_H__ 1

#include <vsgbDynamics/Export.h>
#include <barrier>
#include <vsg/ui/UIEvent.h>
#include <btBulletDynamicsCommon.h>
#include <mutex>
#include <thread>


// Forward declaraction
class btDynamicsWorld;

namespace vsgbDynamics {


// Forward declaraction
class TripleBuffer;


/** \class PhysicsThread PhysicsThread.h <vsgbDynamics/PhysicsThread.h>
\brief An VSG / OpenThreads class for asynchronous physics simulation.

*/
class VSGBDYNAMICS_EXPORT PhysicsThread //: public std::thread
{
public:
    PhysicsThread( btDynamicsWorld* bw, vsgbDynamics::TripleBuffer* tb=nullptr );
    ~PhysicsThread();
    std::thread * _delegate;
    /** Specify the elapsed time parameter, used in call to stepSimulation.
    If value is <= 0.0, PhysicsThread uses the elapsed time.
    Default is 0.0 (use elapsed time). */
    void setTimeStep( btScalar timeStep );
    btScalar getTimeStep() const;

    /** Call start() to launch the thread. */
    virtual void run();

    /** Cause the thread to exit. Call Thread::isRunning() to verify that the
    thread has actually exited. */
    void stopPhysics();

    void start();
    void join(){_delegate->join();};
    void setProcessorAffinity(int i);


    /** Temporarily pause the physics thread (to add a new rigid body,
    or move a static body, for example). You could also call
    stopPhysics then restart the thread, but this would incur the
    overhead of stopping and starting a thread. Use pause to temporarily
    halt the running thread. */
    void pause( bool pause );

    /** After telling the thread to pause, call isPaused() to ensure the
    thread has reached the pause gate and is idle. */
    bool isPaused() const;


    /** Allows access to the Bullet dynamics world. If the calling code
    intends to modify the dynamics world, it
    is responsible for ensuring that the thread is not running (and
    therefore not asynchronously modifying) the physics sim). */
    btDynamicsWorld* getDynamicsWorld() const { return( _bw ); }

protected:
    bool isStopping() const;

    btScalar _timeStep;

    btDynamicsWorld* _bw;
  //  vsg::Timer _timer;
    bool _stopped;
    int _pauseCount;
    vsg::clock::time_point _lastTime;

    vsgbDynamics::TripleBuffer* _tb;

    mutable std::mutex _stopMutex;
    mutable std::mutex _pauseMutex;
   // c++20 required mutable std::barrier _pauseGate;
};


// vsgbDynamics
}

// __VSGBDYNAMICS_PHYSICSTHREAD_H__
#endif
