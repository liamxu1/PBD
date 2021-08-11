//
// Created by Jack Purvis
//

#ifndef POSITIONBASEDDYNAMICS_SIMULATION_HPP
#define POSITIONBASEDDYNAMICS_SIMULATION_HPP

#include <GL/glew.h>
#include <mesh.hpp>
#include <scene.hpp>
#include <constraint.hpp>

class Simulation {

    float COLLISION_THRESHOLD = 0.1f;

public:
    Simulation();
    ~Simulation();

    void reset();

    void update();
    void renderGUI();

    // Variables
    int solverIterations = 20;
    float timeStep = 0.03f;
    float gravity = 0.981f;
    float windSpeed = 1.5f;
    float compliance = 1e-7;
    float velocityDamping = 0.999f;
    float stretchFactor = 0.999f;
    float bendFactor = 0.3f;
    float dampFactor = 0.01f;
    float poisonRatio = 0.3f;
    float YongModulus = 20;

    bool wireframe = true;
    
    /// <summary>
    /// type: PBD type
    /// 0: Normal PBD
    /// 1: XPBD
    /// </summary>
    int type = 0;

    /// <summary>
    /// dampType
    /// 0: No damp
    /// 1: Simple mutiplier
    /// 2: Combine with rotating velocity
    /// 3: In XPBD
    /// </summary>
    int dampType = 0;

    // Scene
    Scene* scene;

private:

    void simulate(Configuration *configuration);
    void generateCollisionConstraints(Configuration *configuration, Mesh *mesh, int index);
    bool planeIntersection(Vector3f rayOrigin, Vector3f rayDirection, float &t, Vector3f &normal);
    void updateCollisionVelocities(CollisionConstraint* constraint);

    // Forces
    float windOscillation = 0.0f;

    bool showStatus = SHOWALLINFO;
};

#endif //POSITIONBASEDDYNAMICS_SIMULATION_HPP
