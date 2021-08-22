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
    configurations.push_back(setupConfigurationF());
    configurations.push_back(setupConfigurationG());
    configurations.push_back(setupConfigurationH());
    configurations.push_back(setupConfigurationI());
    currentConfiguration = configurations[INITIAL_SCENE_INDEX];
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

void Scene::translateInteraction(Vector3f translate, size_t axis) {

    // Translate the attachment points in scene 3
    if (currentConfiguration == configurations[2]) {
        currentConfiguration->simulatedObjects[0]->translate(translate);
    }

    // Translate the controlling points in scene 8
    else if (currentConfiguration == configurations[7])
    {
        size_t n = currentConfiguration->simulatedObjects.size();
        for (size_t i = 0; i < n - 1; i++)
        {
            auto mesh = currentConfiguration->simulatedObjects[i];
            assert(mesh->meshType == MeshType::singlePoint);
            bool direction = true;
            switch (axis)
            {
            case 0:
                direction = dynamic_cast<SinglePointMesh*>(mesh)->x;
                break;
            case 1:
                direction = dynamic_cast<SinglePointMesh*>(mesh)->y;
                break;
            case 2:
                direction = dynamic_cast<SinglePointMesh*>(mesh)->z;
                break;
            default:
                cout << "Error: wrong axis used in function Scene::translateInteraction()\n\n";
                break;
            }

            if (direction)
            {
                mesh->vertices[0] += translate;
            }
            else
            {
                mesh->vertices[0] -= translate;
            }
        }
    }

    // Translate the controlling points in scene 9
    else if (currentConfiguration == configurations[8])
    {
        if (axis == 0)
        {
            size_t n = currentConfiguration->simulatedObjects.size();
            for (size_t i = 0; i < n - 1; i++)
            {
                auto mesh = currentConfiguration->simulatedObjects[i];
                assert(mesh->meshType == MeshType::singlePoint);
                bool direction = dynamic_cast<SinglePointMesh*>(mesh)->x;
                if (direction)
                {
                    mesh->vertices[0] += translate;
                }
                else
                {
                    mesh->vertices[0] -= translate;
                }
            }
        }
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

bool Scene::isClothSimulation(int index)
{
    Configuration* configuration = nullptr;
    if (index == -1)  configuration = currentConfiguration;
    else if (index < 0 || index >= sceneNum())
    {
        cout << "Incorrectly use function Scene::isTriangularMesh()\nIndex should be within [0, sceneNum - 1]\n\n";
        return false;
    }
    else configuration = configurations[index];
    int count = 0;
    for (auto mesh : configuration->simulatedObjects)
    {
        if (mesh->meshType != MeshType::triangular) return false;
        if (mesh->needCoef) count++;
    }
    if (count > 0)
        return true;
    return false;
}

Configuration* Scene::setupConfigurationA() {
    Configuration* configurationA = new Configuration();

    addPlaneToConfiguration(configurationA);

    Vector3f flagPoleColour = { 0.337f, 0.184f, 0.054f };
    TriangularMesh* flagPole = new TriangularMesh("", "../resources/models/sceneA/flagPole.obj", flagPoleColour);
    TriangularMesh* flagPole2 = new TriangularMesh("", "../resources/models/sceneA/flagPole2.obj", flagPoleColour);

    Vector3f flagColour = { 0.6f, 0.0f, 0.0f };
    TriangularMesh* flag = new TriangularMesh("FlagA", "../resources/models/sceneA/flag.obj", flagColour);
    flag->gravityAffected = true;
    flag->windAffected = true;
    flag->needCoef = true;
    TriangularMesh* flagHigh = new TriangularMesh("FlagB", "../resources/models/sceneA/flagHigh.obj", flagColour);
    flagHigh->gravityAffected = true;
    flagHigh->windAffected = true;
    flagHigh->needCoef = true;

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
    TriangularMesh* cloth = new TriangularMesh("Cloth", "../resources/models/sceneB/cloth.obj", clothColour);
    cloth->gravityAffected = true;
    cloth->needCoef = true;

    Vector3f resetObjectColour = { 0.5f, 0.5f, 0.5f };
    TriangularMesh* restObject = new TriangularMesh("", "../resources/models/sceneB/sphere.obj", resetObjectColour);
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
    TriangularMesh* attachPoints = new TriangularMesh("", "../resources/models/sceneC/attachPoints.obj", solidColour, 0.0f);
    attachPoints->isRigidBody = true;
    attachPoints->dynamicCollisionTest = false;

    Vector3f clothColour = { 0.8f, 0.4f, 0.1f };
    TriangularMesh* cloth = new TriangularMesh("Cloth", "../resources/models/sceneC/cloth.obj", clothColour);
    cloth->gravityAffected = true;
    cloth->windAffected = true;
    cloth->needCoef = true;

    TriangularMesh* bar = new TriangularMesh("Bar", "../resources/models/sceneC/bar.obj", solidColour, 0.5f);
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

    TriangularMesh* cube = new TriangularMesh("Cube", "../resources/models/sceneD/cube.obj", colourA);
    cube->isRigidBody = true;
    cube->gravityAffected = true;
    cube->windAffected = true;

    TriangularMesh* pyramid = new TriangularMesh("Pyramid", "../resources/models/sceneD/pyramid.obj", colourB);
    pyramid->isRigidBody = true;
    pyramid->gravityAffected = true;
    pyramid->windAffected = true;

    TriangularMesh* cylinder = new TriangularMesh("Cylinder", "../resources/models/sceneD/cylinder.obj", colourC);
    cylinder->isRigidBody = true;
    cylinder->gravityAffected = true;
    cylinder->windAffected = true;

    TriangularMesh* sphere = new TriangularMesh("Sphere", "../resources/models/sceneD/sphere.obj", colourD);
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
    TetrahedralMesh* cube = new TetrahedralMesh("Cube", "../resources/models/sceneE/cube.tet", colourA);

    cube->isRigidBody = true;
    cube->gravityAffected = true;

    configurationE->simulatedObjects.push_back(cube);

    setupEstimatePositionOffsets(configurationE);

    buildRigidBodyConstraints(configurationE, cube);

    return configurationE;
}

Configuration* Scene::setupConfigurationF()
{
    Configuration* configurationF = new Configuration();
    addPlaneToConfiguration(configurationF);

    Vector3f colourA = { 0.0f,1.0f,0.f };

    TetrahedralMesh* cubeA = new TetrahedralMesh("Cube", "../resources/models/sceneF/cubeA.tet", colourA);
    cubeA->gravityAffected = true;
    cubeA->needCoef = true;

    configurationF->simulatedObjects.push_back(cubeA);

    setupEstimatePositionOffsets(configurationF);

    buildTetrahedralConstraints(configurationF, cubeA);

    return configurationF;
}

Configuration* Scene::setupConfigurationG()
{
    Configuration* configurationG = new Configuration();
    addPlaneToConfiguration(configurationG);

    Vector3f colourA = { 0.0f,1.0f,0.0f };
    Vector3f colourB = { 1.0f,0.0f,0.0f };

    TetrahedralMesh* cubeA = new TetrahedralMesh("CubeA", "../resources/models/sceneG/cubeA.tet", colourA);
    cubeA->gravityAffected = true;
    cubeA->needCoef = true;
    
    TetrahedralMesh* cubeB = new TetrahedralMesh("CubeB", "../resources/models/sceneG/cubeB.tet", colourB);
    cubeB->gravityAffected = true;
    cubeB->needCoef = true;
    
    configurationG->simulatedObjects.push_back(cubeA);
    configurationG->simulatedObjects.push_back(cubeB);

    setupEstimatePositionOffsets(configurationG);

    buildTetrahedralConstraints(configurationG, cubeA);
    buildTetrahedralConstraints(configurationG, cubeB);

    return configurationG;
}

Configuration* Scene::setupConfigurationH()
{
    Configuration* configurationH = new Configuration();

    Vector3f colour = { 0.0f,1.0f,0.f };

    TetrahedralMesh* cube = new TetrahedralMesh("Cube", "../resources/models/sceneH/cube.tet", colour);
    cube->gravityAffected = true;
    cube->needCoef = true;

    vector<Vector3f> fixedPoints = { Vector3f(0,0,0),Vector3f(0,0,3) ,Vector3f(0,3,0) ,Vector3f(0,3,3) ,Vector3f(3,0,0) ,Vector3f(3,0,3) ,Vector3f(3,3,0) ,Vector3f(3,3,3) };
    size_t numFixedPoints = fixedPoints.size();
    vector<SinglePointMesh*> vertices(numFixedPoints, nullptr);
    for (size_t i = 0; i < numFixedPoints; i++)
    {
        vertices[i] = new SinglePointMesh("", fixedPoints[i], i);
    }

    for (auto vertex : vertices)
    {
        vertex->dynamicCollisionTest = false;
        configurationH->simulatedObjects.push_back(vertex);
    }
    configurationH->simulatedObjects.push_back(cube);

    setupEstimatePositionOffsets(configurationH);

    buildTetrahedralConstraints(configurationH, cube);

    buildTwoWayCouplingConstraints(configurationH, cube);

    return configurationH;
}

Configuration* Scene::setupConfigurationI()
{
    Configuration* configurationI = new Configuration();

    Vector3f colour = { 0.0f,1.0f,0.f };

    TetrahedralMesh* cuboid = new TetrahedralMesh("Cuboid", "../resources/models/sceneI/cube.tet", colour);
    cuboid->gravityAffected = false;
    cuboid->needCoef = true;
    
    for (auto &pos : cuboid->vertices)
    {
        if (fabs(pos[0]) < 1e-5)
        {
            SinglePointMesh* vertex = new SinglePointMesh("", pos, 4);
            vertex->dynamicCollisionTest = false;
            configurationI->simulatedObjects.push_back(vertex);
        }
        else if (fabs(pos[0] - 5.f) < 1e-5)
        {
            SinglePointMesh* vertex = new SinglePointMesh("", pos, 0);
            vertex->dynamicCollisionTest = false;
            configurationI->simulatedObjects.push_back(vertex);
        }
    }

    configurationI->simulatedObjects.push_back(cuboid);

    setupEstimatePositionOffsets(configurationI);

    buildTetrahedralConstraints(configurationI, cuboid);

    buildTwoWayCouplingConstraints(configurationI, cuboid);

    return configurationI;
}

void Scene::addPlaneToConfiguration(Configuration* configuration) {
    Vector3f planeColour = { 1.0f, 1.0f, 1.0f };
    TriangularMesh* plane = new TriangularMesh("", "../resources/models/plane.obj", planeColour);
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

    for (Mesh* mesh : configuration->staticObjects) {
        mesh->estimatePositionsOffset = -totalNumVertices;
    }
}
