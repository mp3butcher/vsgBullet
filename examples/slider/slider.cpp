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
#include <osgDB/FileUtils>
#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osgUtil/Optimizer>
#include <osg/ComputeBoundsVisitor>

#include <osg/Light>
#include <osg/LightSource>
#include <osg/Material>
#include <osg/LightModel>
#include <osgShadow/ShadowedScene>
#include <osgShadow/StandardShadowMap>

#include <vsgbDynamics/MotionState.h>
#include <vsgbCollision/CollisionShapes.h>
#include <vsgbCollision/RefBulletObject.h>
#include <vsgbDynamics/RigidBody.h>
#include <vsgbDynamics/GroundPlane.h>
#include <vsgbCollision/GLDebugDrawer.h>
#include <vsgbCollision/Utils.h>
#include <vsgbInteraction/DragHandler.h>
#include <vsgbInteraction/LaunchHandler.h>
//#include <vsgbInteraction/SaveRestoreHandler.h>
/*
#include <osgwTools/InsertRemove.h>
#include <osgwTools/FindNamedNode.h>
#include <osgwTools/Version.h>*/

#include <btBulletDynamicsCommon.h>

#include <osg/io_utils>
#include <string>
#include <map>



class FindNamedNode : public vsg::NodeVisitor
{
public:
    /**
    @param name Name of the Node to search for.
    */
    FindNamedNode( const std::string& name, const vsg::NodeVisitor::TraversalMode travMode=vsg::NodeVisitor::TRAVERSE_ACTIVE_CHILDREN );
    ~FindNamedNode();

    typedef std::pair< vsg::Node*, vsg::NodePath > NodeAndPath;
    typedef std::vector< NodeAndPath > NodeAndPathList;
    NodeAndPathList _napl;

    void reset();

    /**
    Algorithm for matching the specified name. Possible future
    work: support for case-insensitive matching.
    */
    typedef enum {
        EXACT_MATCH,
        CONTAINS
    } MatchMethod;
    /**
    Specifies the match algorithm.
    @param method The match algorithm. The default is EXACT_MATCH
    */
    void setMatchMethod( MatchMethod method );
    /**
    Gets the match algorithm.
    */
    MatchMethod getMatchMethod() const;

    /**
    Controls whether the named Node is included at the end of
    the NodePaths in _napl.
    @param includeTargetNode If false, don't include the named Node
    in the returned NodePaths. The default is true (include the named Node
    in the paths).
    */
    void setPathsIncludeTargetNode( bool includeTargetNode );
    /**
    Gets the current setting for including the named Node in the
    returned NodePaths.
    */
    bool getPathsIncludeTargetNode() const;

    /**
    Overrides of base class apply() method.
    */
    void apply( vsg::Node& node );

protected:
    std::string _name;

    MatchMethod _method;
    bool _includeTargetNode;
};

FindNamedNode::FindNamedNode( const std::string& name, const vsg::NodeVisitor::TraversalMode travMode )
  : vsg::NodeVisitor( travMode ),
    _name( name ),
    _method( EXACT_MATCH ),
    _includeTargetNode( true )
{
}

FindNamedNode::~FindNamedNode()
{
}

void
FindNamedNode::reset()
{
    _napl.clear();
}

void
FindNamedNode::setMatchMethod( MatchMethod method )
{
    _method = method;
}
FindNamedNode::MatchMethod
FindNamedNode::getMatchMethod() const
{
    return( _method );
}

void
FindNamedNode::setPathsIncludeTargetNode( bool includeTargetNode )
{
    _includeTargetNode = includeTargetNode;
}
bool
FindNamedNode::getPathsIncludeTargetNode() const
{
    return( _includeTargetNode );
}


void
FindNamedNode::apply( vsg::Node& node )
{
    bool match = (
        ( ( _method == EXACT_MATCH ) &&
            ( node.getName() == _name ) ) ||
        ( ( _method == CONTAINS ) &&
            ( node.getName().find( _name ) != std::string::npos ) ) );

    if( match )
    {
        // Copy the NodePath, so we can alter it if necessary.
        vsg::NodePath np = getNodePath();

        if( !_includeTargetNode )
            // Calling code has requested that the target node
            // be removed from the node paths.
            np.pop_back();

        NodeAndPath nap( &node, np );
        _napl.push_back( nap );
    }

    traverse( node );
}

/** \cond */

// Derive a class from StandardShadowMap so we have better control over
// the direction of the depth map generation Camera.
class ControlledShadowMap : public osgShadow::StandardShadowMap
{
public:
    ControlledShadowMap()
      : osgShadow::StandardShadowMap()
    {
        // Improve on the defaults.
        _textureSize = vsg::Vec2s( 2048, 2048 ),
        // StandardShadowMap doesn't appear to provide accessors for these values...
        _polygonOffsetFactor = 2.f;
        _polygonOffsetUnits = 2.f;
    }

protected:
    struct ViewData : public osgShadow::StandardShadowMap::ViewData
    {
        virtual void aimShadowCastingCamera(
            const vsg::BoundingSphere& bounds,
            const vsg::Light* light,
            const vsg::Vec4& worldLightPos,
            const vsg::vec3& worldLightDir,
            const vsg::vec3& worldLightUp = vsg::vec3( 0, 1,0 ) )
        {
            // For out case, we have a point light source, but we always want
            // the center of the depth map at the center of the scene. We don't
            // care of objects away from the center don't cast a shadow. So we
            // override aimShadowCastingCamera() to compute the depth map generation
            // Camera the way we want it.

            vsg::Matrixd& view = _camera->getViewMatrix();
            vsg::Matrixd& projection = _camera->getProjectionMatrix();

            vsg::vec3 up = worldLightUp;
            if( up.length2() <= 0 )
                up.set( 0, 1, 0 );

            vsg::vec3 position( worldLightPos.x(), worldLightPos.y(), worldLightPos.z() );
            view.makeLookAt( position, vsg::vec3( 0., 0., 0. ), up );

            const double distance( ( bounds.center() - position ).length() );
            const double zFar = distance + bounds.radius();
            const double zNear = vsg::maximum< double >( (zFar * 0.001), (distance-bounds.radius()) );
            const float fovy = 120.f;
            projection.makePerspective( fovy, 1.0, zNear, zFar );
            _camera->setComputeNearFarMode( vsg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR );
        }
    };

    friend struct ViewData;

    META_ViewDependentShadowTechniqueData( ControlledShadowMap, ControlledShadowMap::ViewData )
};

/** \endcond */


// Filter out collisions between the drawer and nightstand.
//
// Bullet collision filtering tutorial:
//   http://www.bulletphysics.com/mediawiki-1.5.8/index.php?title=Collision_Filtering
//
// Define filter groups
enum CollisionTypes {
    COL_DRAWER = 0x1 << 0,
    COL_STAND = 0x1 << 1,
    COL_DEFAULT = 0x1 << 2,
};
// Define filter masks
unsigned int drawerCollidesWith( COL_DEFAULT );
unsigned int standCollidesWith( COL_DEFAULT );
unsigned int defaultCollidesWith( COL_DRAWER | COL_STAND | COL_DEFAULT );



btRigidBody* standBody;
void makeStaticObject( btDiscreteDynamicsWorld* bw, vsg::Node* node, const vsg::mat4& m )
{
    vsg::ref_ptr< vsgbDynamics::CreationRecord > cr = new vsgbDynamics::CreationRecord;
    cr->_sceneGraph = node;
    cr->_shapeType = BOX_SHAPE_PROXYTYPE;
    cr->_mass = 0.f;
    btRigidBody* rb = vsgbDynamics::createRigidBody( cr.get() );


    bw->addRigidBody( rb, COL_STAND, standCollidesWith );

    // Save RB in global
    standBody = rb;
}

void
insertAbove( vsg::Node* node, vsg::Group* newParent )
{
    // Don't let the node get deleted when we remove it from all its parents.
    // Equivalent to explicit call to node->ref(), then node->unref() at end of function.
    vsg::ref_ptr< vsg::Node > nodeHolder( node );

    vsg::Node::ParentList pl = node->getParents();
    vsg::Node::ParentList::iterator it;
    for( it = pl.begin(); it != pl.end(); it++ )
    {
        vsg::Group* oldParent( *it );
        oldParent->addChild( newParent );
        oldParent->removeChild( node );
    }
    newParent->addChild( node );
}

btRigidBody* drawerBody;
vsg::Transform* makeDrawer( btDiscreteDynamicsWorld* bw/*, vsgbInteraction::SaveRestoreHandler* srh*/, vsg::Node* node, const vsg::mat4& m )
{
    vsgbCollision::AbsoluteModelTransform* amt = new vsgbCollision::AbsoluteModelTransform;
    amt->setDataVariance( vsg::Object::DYNAMIC );
     insertAbove( node, amt );

    vsg::ref_ptr< vsgbDynamics::CreationRecord > cr = new vsgbDynamics::CreationRecord;
    cr->_sceneGraph = amt;
    cr->_shapeType = BOX_SHAPE_PROXYTYPE;
    cr->setCenterOfMass( node->getBound().center() );
    cr->_parentTransform = m;
    cr->_mass = .75f;
    cr->_restitution = .5f;
    btRigidBody* rb = vsgbDynamics::createRigidBody( cr.get() );


    bw->addRigidBody( rb, COL_DRAWER, drawerCollidesWith );
    rb->setActivationState( DISABLE_DEACTIVATION );

    // Save RB in global, as AMT UserData (for DragHandler), and in SaveRestoreHandler.
    drawerBody = rb;
    amt->setUserData( new vsgbCollision::RefRigidBody( rb ) );
    //srh->add( "gate", rb );

    return( amt );
}


btDiscreteDynamicsWorld* initPhysics()
{
    btDefaultCollisionConfiguration * collisionConfiguration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher * dispatcher = new btCollisionDispatcher( collisionConfiguration );
    btConstraintSolver * solver = new btSequentialImpulseConstraintSolver;

    btVector3 worldAabbMin( -10000, -10000, -10000 );
    btVector3 worldAabbMax( 10000, 10000, 10000 );
    btBroadphaseInterface * inter = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );

    btDiscreteDynamicsWorld * dynamicsWorld = new btDiscreteDynamicsWorld( dispatcher, inter, solver, collisionConfiguration );

    dynamicsWorld->setGravity( btVector3( 0, 0, -9.81 ) );

    return( dynamicsWorld );
}


vsg::Node* findNamedNode( vsg::Node* model, const std::string& name, vsg::mat4& xform )
{
    FindNamedNode fnn( name );
    model->accept( fnn );
    if( fnn._napl.empty() )
    {
        std::cerr << "hinge: Can't find node names \"" << name << "\"." << std::endl;
        return( nullptr );
    }
    xform = vsg::computeLocalToWorld( fnn._napl[ 0 ].second );
    return( fnn._napl[ 0 ].first );
}

void simpleLighting( vsg::Group* root )
{
    vsg::StateSet* rootState = root->getOrCreateStateSet();
    rootState->setMode( GL_LIGHT0, vsg::StateAttribute::ON );

    vsg::LightSource* ls = new vsg::LightSource();
    ls->setReferenceFrame( vsg::LightSource::RELATIVE_RF );
    root->addChild( ls );

    vsg::Light* light = new vsg::Light;
    light->setLightNum( 0 );
    light->setAmbient( vsg::Vec4( 1., 1., 1., 1. ) );
    light->setDiffuse( vsg::Vec4( 1., 1., 1., 1. ) );
    light->setSpecular( vsg::Vec4( 1., 1., 1., 1. ) );

    vsg::vec3 pos( -.5, -.4, 2. );
    light->setPosition( vsg::Vec4( pos, 1. ) );
    ls->setLight( light );

    vsg::LightModel* lm = new vsg::LightModel;
    lm->setAmbientIntensity( vsg::Vec4( 0., 0., 0., 1. ) );
    lm->setColorControl( vsg::LightModel::SEPARATE_SPECULAR_COLOR );
    rootState->setAttribute( lm, vsg::StateAttribute::ON );
}

void nightstandMaterial( vsg::Node* root )
{
    vsg::StateSet* rootState = root->getOrCreateStateSet();

    vsg::Material* mat = new vsg::Material;
    mat->setAmbient( vsg::Material::FRONT, vsg::Vec4( .1, .1, .1, 1. ) );
    mat->setDiffuse( vsg::Material::FRONT, vsg::Vec4( 1., 1., 1., 1. ) );
    mat->setSpecular( vsg::Material::FRONT, vsg::Vec4( 0.6, 0.6, 0.5, 1. ) );
    mat->setShininess( vsg::Material::FRONT, 16.f );
    rootState->setAttribute( mat, vsg::StateAttribute::ON | vsg::StateAttribute::OVERRIDE );
}

void groundMaterial( vsg::Node* root )
{
    vsg::StateSet* rootState = root->getOrCreateStateSet();

    vsg::Material* mat = new vsg::Material;
    mat->setAmbient( vsg::Material::FRONT, vsg::Vec4( .1, .1, .1, 1. ) );
    mat->setDiffuse( vsg::Material::FRONT, vsg::Vec4( .75, .75, .75, 1. ) );
    mat->setSpecular( vsg::Material::FRONT, vsg::Vec4( 0., 0., 0., 1. ) );
    rootState->setAttribute( mat, vsg::StateAttribute::ON | vsg::StateAttribute::OVERRIDE );
}

void launchMaterial( vsg::Node* root )
{
    vsg::StateSet* rootState = root->getOrCreateStateSet();

    vsg::Material* mat = new vsg::Material;
    mat->setAmbient( vsg::Material::FRONT, vsg::Vec4( .2, .2, .3, 1. ) );
    mat->setDiffuse( vsg::Material::FRONT, vsg::Vec4( .4, .4, .5, 1. ) );
    mat->setSpecular( vsg::Material::FRONT, vsg::Vec4( .4, .4, .4, 1. ) );
    mat->setShininess( vsg::Material::FRONT, 30.f );
    rootState->setAttribute( mat, vsg::StateAttribute::ON | vsg::StateAttribute::OVERRIDE );
}

#define SHADOW_CASTER 0x1
#define SHADOW_RECEIVER 0x2
#define SHADOW_BOTH (SHADOW_CASTER|SHADOW_RECEIVER)

int main( int argc, char** argv )
{
    vsg::ArgumentParser arguments( &argc, argv );
    const bool debugDisplay( arguments.find( "--debug" ) > 0 );

    btDiscreteDynamicsWorld* bulletWorld = initPhysics();
    vsg::Group* root = new vsg::Group;

    simpleLighting( root );

    vsg::Group* launchHandlerAttachPoint = new vsg::Group;
    launchHandlerAttachPoint->setNodeMask( SHADOW_BOTH );
    root->addChild( launchHandlerAttachPoint );
    launchMaterial( launchHandlerAttachPoint );


    vsg::ref_ptr< vsg::Node > rootModel = osgDB::readNodeFile( "NightStand.flt" );
    if( !rootModel.valid() )
    {
        std::cerr << "hinge: Can't load data file \"NightStand.flt\"." << std::endl;
        return( 1 );
    }
    rootModel->setNodeMask( SHADOW_BOTH );
    nightstandMaterial( rootModel.get() );

    root->addChild( rootModel.get() );

    // Get Node pointers and parent transforms for the night stand and drawer.
    // (Node names are taken from the osgWorks osgwnames utility.)
    vsg::mat4 standXform, drawerXform;
    vsg::Node* standNode = findNamedNode( rootModel.get(), "NightStand_Body", standXform );
    vsg::Node* drawerNode = findNamedNode( rootModel.get(), "DOF_Drawer", drawerXform );
    if( ( standNode == nullptr ) || ( drawerNode == nullptr ) )
        return( 1 );

//    vsg::ref_ptr< vsgbInteraction::SaveRestoreHandler > srh = new        vsgbInteraction::SaveRestoreHandler;

    // Make Bullet rigid bodies and collision shapes for the drawer...
    makeDrawer( bulletWorld/*, srh.get()*/, drawerNode, drawerXform );
    // ...and the stand.
    makeStaticObject( bulletWorld, standNode, standXform );


    // Add ground
    const vsg::Vec4 plane( 0., 0., 1., 0. );
    vsg::Node* groundRoot = vsgbDynamics::generateGroundPlane( plane,
        bulletWorld, nullptr, COL_DEFAULT, defaultCollidesWith );
    groundRoot->setNodeMask( SHADOW_RECEIVER );
    groundMaterial( groundRoot );
    root->addChild( groundRoot );


    // create slider constraint between drawer and stand, and add it to world.
    // Note: Bullet slider is always along x axis. Alter this behavior with reference frames.
    btSliderConstraint* slider;
    float drawerMinLimit;
    btVector3 startPos;
    {
        // Model-specific constants.
        // TBD Should obtain these from model metadata or user input:
        const vsg::vec3 drawerAxis( 0., 1., 0. );
        const float drawerMaxLimit( 0.f );

        vsg::ComputeBoundsVisitor cbv;
        drawerNode->accept( cbv );
        const vsg::BoundingBox& bb = cbv.getBoundingBox();
        drawerMinLimit = -( bb.yMax() - bb.yMin() );

        // Compute a matrix that transforms the stand's collision shape origin and x axis
        // to the drawer's origin and drawerAxis.
        //   1. Matrix to align the (slider constraint) x axis with the drawer axis.
        const vsg::vec3 bulletSliderAxis( 1., 0., 0. );
        const vsg::mat4 axisRotate( vsg::mat4::rotate( bulletSliderAxis, drawerAxis ) );
        //
        //   2. Inverse stand center of mass offset.
        vsgbDynamics::MotionState* motion = dynamic_cast< vsgbDynamics::MotionState* >( standBody->getMotionState() );
        const vsg::mat4 invStandCOM( vsg::mat4::translate( -( motion->getCenterOfMass() ) ) );
        //
        //   3. Transform from the stand's origin to the drawer's origin.
        const vsg::mat4 standToDrawer( vsg::mat4::inverse( standXform ) * drawerXform );
        //
        //   4. The final stand frame matrix.
        btTransform standFrame = vsgbCollision::asBtTransform(
            axisRotate * invStandCOM * standToDrawer );


        // Compute a matrix that transforms the drawer's collision shape origin and x axis
        // to the drawer's origin and drawerAxis.
        //   1. Drawer center of mass offset.
        motion = dynamic_cast< vsgbDynamics::MotionState* >( drawerBody->getMotionState() );
        const vsg::mat4 invDrawerCOM( vsg::mat4::translate( -( motion->getCenterOfMass() ) ) );
        //
        //   2. The final drawer frame matrix.
        btTransform drawerFrame = vsgbCollision::asBtTransform(
            axisRotate * invDrawerCOM );


        slider = new btSliderConstraint( *drawerBody, *standBody, drawerFrame, standFrame, false );
        slider->setLowerLinLimit( drawerMinLimit );
	    slider->setUpperLinLimit( drawerMaxLimit );
        bulletWorld->addConstraint( slider, true );

        btTransform m = drawerBody->getWorldTransform();
        startPos = m.getOrigin();
    }


    vsgbCollision::GLDebugDrawer* dbgDraw( nullptr );
    if( debugDisplay )
    {
        dbgDraw = new vsgbCollision::GLDebugDrawer();
        dbgDraw->setDebugMode( ~btIDebugDraw::DBG_DrawText );
        bulletWorld->setDebugDrawer( dbgDraw );
        vsg::Node* dbgRoot = dbgDraw->getSceneGraph();
        dbgRoot->setNodeMask( ~SHADOW_BOTH );
        root->addChild( dbgRoot );
    }


    osgShadow::ShadowedScene* sceneRoot = new osgShadow::ShadowedScene;
    sceneRoot->setCastsShadowTraversalMask( SHADOW_CASTER );
    sceneRoot->setReceivesShadowTraversalMask( SHADOW_RECEIVER );
    sceneRoot->addChild( root );
    {
        ControlledShadowMap* sTex = new ControlledShadowMap;

        // Workaround the fact that OSG StandardShadowMap fragment shader
        // doesn't add in the specular color.
#if( OSGWORKS_OSG_VERSION < 20912 )
        // Prior to Feb 23 2011 (2.9.12 release), StandardShadowMap used vertex
        // shaders that employed a varying to convey ambient color.
        std::string shaderName( "ShadowMap-Main.fs" );
#else
        // From 2.9.12 onward, StandardShadowMap uses only a fragment shader,
        // so ambient color is conveyed in the FFP built-in varying.
        std::string shaderName( "ShadowMap-Main-3x.fs" );
#endif
        vsg::Shader* shader = new vsg::Shader( vsg::Shader::FRAGMENT );
        shader->setName( shaderName );
        shader->loadShaderSourceFromFile( osgDB::findDataFile( shaderName ) );
        if( shader->getShaderSource().empty() )
            std::cerr << "Warning: Unable to load shader file: \"" << shaderName << "\"." << std::endl;
        else
            sTex->setMainFragmentShader( shader );

        sceneRoot->setShadowTechnique( sTex );
    }


    osgViewer::Viewer viewer( arguments );
    viewer.setUpViewInWindow( 30, 30, 800, 450 );
    viewer.setSceneData( sceneRoot );

    osgGA::TrackballManipulator* tb = new osgGA::TrackballManipulator;
    tb->setHomePosition( vsg::vec3( .8, -5., 1.6 ), vsg::vec3( 0., 0., .5 ), vsg::vec3( 0., 0., 1. ) );
    viewer.setCameraManipulator( tb );
    viewer.getCamera()->setClearColor( vsg::Vec4( .5, .5, .5, 1. ) );

    // Create the launch handler.
 vsgbInteraction::LaunchHandler* lh = new vsgbInteraction::LaunchHandler();
     //lh->setWorld(root);
        lh->setAttachPoint( launchHandlerAttachPoint );
    {
        // Use a custom launch model: A scaled-down teapot.
        vsg::ref_ptr< vsg::MatrixTransform > mt = new vsg::MatrixTransform(
            vsg::mat4::scale( 0.2, 0.2, 0.2 ) );
        mt->addChild( osgDB::readNodeFile( "teapot.osg" ) );
        osgUtil::Optimizer opt;
        opt.optimize( mt.get(), osgUtil::Optimizer::FLATTEN_STATIC_TRANSFORMS );
        mt->getOrCreateStateSet()->setMode( GL_NORMALIZE, vsg::StateAttribute::ON );

        lh->setLaunchModel( mt.get() );
        lh->setInitialVelocity( 10. );

        // Also add the proper collision masks
        lh->setCollisionFilters( COL_DEFAULT, defaultCollidesWith );

        viewer.addEventHandler( lh );
    }

/*    srh->setLaunchHandler( lh );
    srh->capture();
    viewer.addEventHandler( srh.get() );*/
    viewer.addEventHandler( new vsgbInteraction::DragHandler(
        /*bulletWorld, viewer.getCamera() */) );

    viewer.realize();
    double prevSimTime = 0.;
    while( !viewer.done() )
    {
        if( dbgDraw != nullptr )
            dbgDraw->BeginDraw();

        const double currSimTime = viewer.getFrameStamp()->getSimulationTime();
        bulletWorld->stepSimulation( currSimTime - prevSimTime );
        prevSimTime = currSimTime;

        if( dbgDraw != nullptr )
        {
            bulletWorld->debugDrawWorld();
            dbgDraw->EndDraw();
        }

        viewer.frame();

        if( slider != nullptr )
        {
            btTransform m = drawerBody->getWorldTransform();
            btVector3 v = m.getOrigin();
            if( ( v[ 1 ] - startPos[ 1 ] ) < ( drawerMinLimit * 1.02 ) )
            {
                bulletWorld->removeConstraint( slider );
                delete slider;
                slider = nullptr;
                // Advice from Bullet forum is that the correct way to change the collision filters
                // is to remove the body and add it back:
                // http://bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=9&t=7538
                bulletWorld->removeRigidBody( drawerBody );
                bulletWorld->addRigidBody( drawerBody, COL_DEFAULT, defaultCollidesWith );
            }
        }
    }

    return( 0 );
}


/** \page sliderlowlevel Simple Slider Constraint

Demonstrates coding directly to the Bullet API to create a slider constraint.

Use the --debug command line option to enable debug collision object display.

\section slidercontrols UI Controls

\li Delete: Reset the physics simulation to its initial state.
\li ctrl-leftmouse: Select and drag an object.
\li shift-leftmouse: Launches a sphere into the scene.

*/
