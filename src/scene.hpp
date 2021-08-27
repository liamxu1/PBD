//
// Created by Jack Purvis
//

#ifndef POSITIONBASEDDYNAMICS_SCENE_HPP
#define POSITIONBASEDDYNAMICS_SCENE_HPP

#include <GL/glew.h>
#include <camera.hpp>
#include <constraint.hpp>
#include <mesh.hpp>

const int INITIAL_SCENE_INDEX = 2;

class Constraint;
class CollisionConstraint;

struct Configuration {
    vector<Mesh*> staticObjects;
    vector<Mesh*> simulatedObjects;
    vector<Vector3f> estimatePositions;
    vector<Vector3f> currentPositions;
    vector<float> inverseMasses;
    vector<Constraint*> constraints;
    vector<CollisionConstraint*> collisionConstraints;

    vector<float> lambda; //for xpbd

    ~Configuration() {
        for (Mesh* mesh : staticObjects) delete mesh;
        for (Mesh* mesh : simulatedObjects) delete mesh;
        for (Constraint* constraint : constraints) delete constraint;
        for (CollisionConstraint* constraint : collisionConstraints) delete constraint;
    }
};

class Scene {

public:
    Scene();
    ~Scene();

    void reset();
    void setConfiguration(int index);

    // axis: 0-x, 1-y, 2-z
    void translateInteraction(Vector3f translate, size_t axis = 0);
    void render(bool wireframe);

    int sceneNum();

    // Camera
    Camera* camera;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float roll = 0.0f;

    Vector3f translation = Vector3f(0.0f, 0.0f, 0.0f);

    Configuration* currentConfiguration;

    bool isClothSimulation(int index = -1);

private:

    // Scene configuration setup
    Configuration* setupConfigurationA();
    Configuration* setupConfigurationB();
    Configuration* setupConfigurationC();
    Configuration* setupConfigurationD();
    Configuration* setupConfigurationE();
    Configuration* setupConfigurationF();
    Configuration* setupConfigurationG();
    Configuration* setupConfigurationH();
    Configuration* setupConfigurationI();
    Configuration* setupConfigurationJ();
    Configuration* setupConfigurationK();
    Configuration* setupConfigurationL();
    void addPlaneToConfiguration(Configuration* configuration);
    void setupEstimatePositionOffsets(Configuration* configuration);

    vector<Configuration*> configurations;
};

#endif //POSITIONBASEDDYNAMICS_SCENE_HPP
