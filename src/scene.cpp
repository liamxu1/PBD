//
// Created by Jack Purvis
//

#include <iostream>
#include <main.hpp>
#include <constraint.hpp>
#include <scene.hpp>

Scene::Scene() {

    // Setup camera
    camera = new Camera();

    // Setup scene configurations
    configurations.push_back(setupConfigurationA());
    configurations.push_back(setupConfigurationB());
    configurations.push_back(setupConfigurationC());
    configurations.push_back(setupConfigurationD());
    configurations.push_back(setupConfigurationE());
    currentConfiguration = configurations[2];
}

Scene::~Scene() {
    delete camera;
    for (auto configuation : configurations)
        delete configuation;
}

void Scene::reset() {
    for (Mesh* mesh : currentConfiguration->simulatedObjects) {
        mesh->reset();
    }

    size_t numPositions = currentConfiguration->estimatePositions.size();
    currentConfiguration->estimatePositions.clear();
    currentConfiguration->currentPositions.clear();
    currentConfiguration->estimatePositions.resize(numPositions, Vector3f::Zero());
    currentConfiguration->currentPositions.resize(numPositions, Vector3f::Zero());
}

void Scene::setConfiguration(int index) {
    assert(index >= 0 && index < configurations.size());
    currentConfiguration = configurations[index];
}

void Scene::translateInteraction(Vector3f translate) {

    // Translate the attachment points in scene 3
    if (currentConfiguration == configurations[2]) {
        currentConfiguration->simulatedObjects[0]->translate(translate);
    }
}

void Scene::render(bool wireframe) {

    // Setup camera
    camera->setPerspective(45.0f, (float) SCREEN_WIDTH / (float) SCREEN_HEIGHT, 0.1f, 100.0f);
    camera->lookAt(Vector3f(0, 0, 20) + translation, Vector3f(0, 0, 0), Vector3f(0, 1, 0));

    // Rotate the model matrix based on the camera rotation
    AngleAxisf pitchAngle(pitch, Vector3f::UnitX());
    AngleAxisf yawAngle(yaw, Vector3f::UnitY());
    AngleAxisf rollAngle(roll, Vector3f::UnitZ());
    Quaternion<float> q = rollAngle * yawAngle * pitchAngle;
    Matrix4f r = Matrix4f::Identity();
    r.block(0, 0, 3, 3) = q.toRotationMatrix();
    Matrix4f modelMatrix = Matrix4f::Identity() * r;

    // Set draw mode
    if (wireframe) glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    else glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    // Render configuration objects
    for (Mesh* mesh : currentConfiguration->staticObjects) {
        mesh->render(camera, modelMatrix);
    }

    for (Mesh* mesh : currentConfiguration->simulatedObjects) {
        mesh->render(camera, modelMatrix);
    }
}

int Scene::sceneNum()
{
    return configurations.size();
}

Configuration* Scene::setupConfigurationA() {
    Configuration* configurationA = new Configuration();

    addPlaneToConfiguration(configurationA);

    Vector3f flagPoleColour = { 0.337f, 0.184f, 0.054f };
    TriangularMesh* flagPole = new TriangularMesh("../resources/models/sceneA/flagPole.obj", flagPoleColour);
    TriangularMesh* flagPole2 = new TriangularMesh("../resources/models/sceneA/flagPole2.obj", flagPoleColour);

    Vector3f flagColour = { 0.6f, 0.0f, 0.0f };
    TriangularMesh* flag = new TriangularMesh("../resources/models/sceneA/flag.obj", flagColour);
    flag->gravityAffected = true;
    flag->windAffected = true;
    TriangularMesh* flagHigh = new TriangularMesh("../resources/models/sceneA/flagHigh.obj", flagColour);
    flagHigh->gravityAffected = true;
    flagHigh->windAffected = true;

    configurationA->staticObjects.push_back(flagPole);
    configurationA->staticObjects.push_back(flagPole2);
    configurationA->simulatedObjects.push_back(flag);
    configurationA->simulatedObjects.push_back(flagHigh);

    setupEstimatePositionOffsets(configurationA);

    for (int i = 0; i < 7; i++) buildFixedConstraint(configurationA, flag, i, flag->initialVertices[i]);
    buildEdgeConstraints(configurationA, flag);
    buildBendConstraints(configurationA, flag);

    for (int i = 0; i < 14; i++) buildFixedConstraint(configurationA, flagHigh, i, flagHigh->initialVertices[i]);
    buildEdgeConstraints(configurationA, flagHigh);
    buildBendConstraints(configurationA, flagHigh);

    return configurationA;
}

Configuration* Scene::setupConfigurationB() {
    Configuration* configurationB = new Configuration();

    addPlaneToConfiguration(configurationB);

    Vector3f clothColour = { 0.0f, 0.6f, 0.0f };
    TriangularMesh* cloth = new TriangularMesh("../resources/models/sceneB/cloth.obj", clothColour);
    cloth->gravityAffected = true;

    Vector3f resetObjectColour = { 0.5f, 0.5f, 0.5f };
    TriangularMesh* restObject = new TriangularMesh("../resources/models/sceneB/sphere.obj", resetObjectColour);
    restObject->isRigidBody = true;

    configurationB->simulatedObjects.push_back(cloth);
    configurationB->staticObjects.push_back(restObject);

    setupEstimatePositionOffsets(configurationB);

    buildEdgeConstraints(configurationB, cloth);
    buildBendConstraints(configurationB, cloth);

    return configurationB;
}

Configuration* Scene::setupConfigurationC() {
    Configuration* configurationC = new Configuration();

    addPlaneToConfiguration(configurationC);

    Vector3f solidColour = { 1.0f, 1.0f, 1.0f };
    TriangularMesh* attachPoints = new TriangularMesh("../resources/models/sceneC/attachPoints.obj", solidColour, 0.0f);
    attachPoints->isRigidBody = true;

    Vector3f clothColour = { 0.8f, 0.4f, 0.1f };
    TriangularMesh* cloth = new TriangularMesh("../resources/models/sceneC/cloth.obj", clothColour);
    cloth->gravityAffected = true;
    cloth->windAffected = true;

    TriangularMesh* bar = new TriangularMesh("../resources/models/sceneC/bar.obj", solidColour, 0.5f);
    bar->isRigidBody = true;
    bar->gravityAffected = true;
    bar->windAffected = true;

    configurationC->simulatedObjects.push_back(attachPoints);
    configurationC->simulatedObjects.push_back(cloth);
    configurationC->simulatedObjects.push_back(bar);

    setupEstimatePositionOffsets(configurationC);

    buildEdgeConstraints(configurationC, cloth);
    buildBendConstraints(configurationC, cloth);

    buildRigidBodyConstraints(configurationC, bar);

    buildTwoWayCouplingConstraints(configurationC, cloth);

    return configurationC;
}

Configuration* Scene::setupConfigurationD() {
    Configuration* configurationD = new Configuration();

    addPlaneToConfiguration(configurationD);

    Vector3f colourA = { 1.0f, 1.0f, 0.0f };
    Vector3f colourB = { 1.0f, 0.0f, 1.0f };
    Vector3f colourC = { 0.0f, 1.0f, 1.0f };
    Vector3f colourD = { 0.4f, 0.4f, 0.4f };

    TriangularMesh* cube = new TriangularMesh("../resources/models/sceneD/cube.obj", colourA);
    cube->isRigidBody = true;
    cube->gravityAffected = true;
    cube->windAffected = true;

    TriangularMesh* pyramid = new TriangularMesh("../resources/models/sceneD/pyramid.obj", colourB);
    pyramid->isRigidBody = true;
    pyramid->gravityAffected = true;
    pyramid->windAffected = true;

    TriangularMesh* cylinder = new TriangularMesh("../resources/models/sceneD/cylinder.obj", colourC);
    cylinder->isRigidBody = true;
    cylinder->gravityAffected = true;
    cylinder->windAffected = true;

    TriangularMesh* sphere = new TriangularMesh("../resources/models/sceneD/sphere.obj", colourD);
    sphere->isRigidBody = true;
    sphere->gravityAffected = true;
    sphere->windAffected = true;

    configurationD->simulatedObjects.push_back(cube);
    configurationD->simulatedObjects.push_back(pyramid);
    configurationD->simulatedObjects.push_back(cylinder);
    configurationD->simulatedObjects.push_back(sphere);

    setupEstimatePositionOffsets(configurationD);

    buildRigidBodyConstraints(configurationD, cube);
    buildRigidBodyConstraints(configurationD, pyramid);
    buildRigidBodyConstraints(configurationD, cylinder);
    buildRigidBodyConstraints(configurationD, sphere);

    return configurationD;
}

Configuration* Scene::setupConfigurationE()
{
    Configuration* configurationE = new Configuration();
    
    addPlaneToConfiguration(configurationE);

    Vector3f colourA = { 0.0f,1.0f,0.0f };
    TetrahedralMesh* cube = new TetrahedralMesh("../resources/models/sceneE/cube.tet", colourA);

    cube->isRigidBody = true;
    cube->gravityAffected = true;

    configurationE->simulatedObjects.push_back(cube);

    setupEstimatePositionOffsets(configurationE);

    buildRigidBodyConstraints(configurationE, cube);

    return configurationE;
}

void Scene::addPlaneToConfiguration(Configuration* configuration) {
    Vector3f planeColour = { 1.0f, 1.0f, 1.0f };
    TriangularMesh* plane = new TriangularMesh("../resources/models/plane.obj", planeColour);
    plane->isRigidBody = true;

    configuration->staticObjects.push_back(plane);
}

void Scene::setupEstimatePositionOffsets(Configuration* configuration) {
    int totalNumVertices = 0;

    // Compute the index offset for each object in the shared estimate positions list
    for (Mesh* mesh : configuration->simulatedObjects) {
        mesh->estimatePositionsOffset = totalNumVertices;
        totalNumVertices += mesh->numVertices;

        for (int i = 0; i < mesh->numVertices; i++) configuration->inverseMasses.push_back(mesh->inverseMass[i]);
    }

    configuration->estimatePositions.resize((size_t) totalNumVertices, Vector3f::Zero());
    configuration->currentPositions.resize((size_t)totalNumVertices, Vector3f::Zero());
    configuration->lambda.resize((size_t)totalNumVertices, 0.0f);
}
