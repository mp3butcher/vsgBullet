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

#ifndef VSGBCOLLISION_EXPORT_
#define VSGBCOLLISION_EXPORT_ 1


#if defined( _MSC_VER ) || defined( __CYGWIN__ ) || defined( __MINGW32__ ) || defined( __BCPLUSPLUS__ ) || defined( __MWERKS__ )
    #if defined( VSGBULLET_STATIC )
        #define VSGBCOLLISION_EXPORT
    #elif defined( VSGBULLET_SHARED ) && defined( VSGBCOLLISION_LIBRARY )
        #define VSGBCOLLISION_EXPORT __declspec( dllexport )
    #else
        #define VSGBCOLLISION_EXPORT __declspec( dllimport )
    #endif
#else
    #define VSGBCOLLISION_EXPORT
#endif


// VSGBCOLLISION_EXPORT_
#endif
