//
// Created by Jack Purvis
//

#include <iostream>
#include <Eigen>
#include <imgui.h>
#include <omp.h>
#include <main.hpp>
#include <simulation.hpp>

using namespace Eigen;

Simulation::Simulation() {

    // Setup scene
    scene = new Scene();

    omp_set_num_threads(4);

    reset();
}

Simulation::~Simulation() {
    delete scene;
}

void Simulation::reset() {
    scene->reset();
    windOscillation = 0.0f;
}
void Simulation::update() {

    simulate(scene->currentConfiguration);

    windOscillation += 0.05f;

    // Render scene
    scene->render(wireframe);
}

void Simulation::simulate(Configuration *configuration) {

    if (showStatus)
    {
        cout << "\n-------------------------------\n";
    }

    configuration->lambda.clear();
    configuration->lambda.resize(configuration->estimatePositions.size(), 0.0f);

    float dampfactor = dampFactor;
    int damptype = dampType;

    for (auto mesh : configuration->simulatedObjects) {
        if (mesh->needCoef) mesh->updateCoefs();
    }

    // Apply external forces
    for (Mesh* mesh : configuration->simulatedObjects) {
        if (mesh->gravityAffected) mesh->applyImpulse(2.0f * timeStep * Vector3f(0, -gravity, 0));
        if (mesh->windAffected) mesh->applyImpulse(2.0f * timeStep * Vector3f(0, 0, -windSpeed + (sinf(windOscillation) * windSpeed / 2.0f)));
    }

    // Dampen velocities
    for (Mesh* mesh : configuration->simulatedObjects) {
        mesh->dampVelocity(dampfactor, damptype);
    }

    // Initialise estimate positions
    for (Mesh* mesh : configuration->simulatedObjects) {
        #pragma omp parallel for
        for (int i = 0; i < mesh->numVertices; i++) {
            configuration->estimatePositions[i + mesh->estimatePositionsOffset] = mesh->vertices[i] + timeStep * mesh->velocities[i];
            configuration->currentPositions[i + mesh->estimatePositionsOffset] = mesh->vertices[i];
        }
    }

    // Clear existing collision constraints
    for (int c = 0; c < configuration->collisionConstraints.size(); c++) delete configuration->collisionConstraints[c];
    configuration->collisionConstraints.clear();

    // Generate collision constraints
    for (Mesh* mesh : configuration->simulatedObjects) {
        for (int i = 0; i < mesh->numVertices; i++) {
            generateCollisionConstraints(configuration, mesh, i);
        }
    }

    for (Mesh* meshA : configuration->simulatedObjects) {
        if (meshA->meshType == MeshType::SPH)
        {
            for (Mesh* meshB : configuration->simulatedObjects)
            {
                if (meshA == meshB) buildSPHSelfCollisionConstraints(configuration, dynamic_cast<SPHMesh*>(meshA));
                else if (meshB->meshType == MeshType::SPH)
                {
                    buildSPHCollisionConstraints(configuration, dynamic_cast<SPHMesh*>(meshA), dynamic_cast<SPHMesh*>(meshB));
                }
            }
        }
    }

    // Setup constraint parameters
    Params params;
    params.solverIterations = solverIterations;
    switch (type)
    {
    case 0:
        params.type = PBDType::normalPBD;
        break;
    case 1:
        params.type = PBDType::XPBD;
        break;
    default:
        break;
    }
    params.timeStep = timeStep;
    params.useXPBDDamp = (dampType == 3);

    switch (constitutiveModelType)
    {
    case 0:
        params.modelType = ConstitutiveMaterialModel::StVKModel;
        break;
    case 1:
        params.modelType = ConstitutiveMaterialModel::NeoHookeanModel;
        break;
    default:
        break;
    }

    // Project constraints iteratively
    for (int iteration = 0; iteration < solverIterations; iteration++) {
        //#pragma omp parallel for // Improves performance but constraint solving order is not deterministic
        for (int c = 0; c < configuration->constraints.size(); c++) {
            configuration->constraints[c]->project(configuration, params);
        }

        //#pragma omp parallel for // Improves performance but constraint solving order is not deterministic
        for (int c = 0; c < configuration->collisionConstraints.size(); c++) {
            configuration->collisionConstraints[c]->project(configuration, params);
        }
    }

    // Update positions and velocities
    if (showStatus) cout << "Overall:\n";
    for (Mesh* mesh : configuration->simulatedObjects) {
        #pragma omp parallel for
        for (int i = 0; i < mesh->numVertices; i++) {
            if (showStatus) cout << i + mesh->estimatePositionsOffset << ":\nVelocity:\t";
            mesh->velocities[i] = (configuration->estimatePositions[mesh->estimatePositionsOffset + i] - mesh->vertices[i]) / timeStep;
            if (showStatus) cout << mesh->velocities[i][0] << ' ' << mesh->velocities[i][1] << ' ' << mesh->velocities[i][2] << "\nPosition:\t";
            mesh->vertices[i] = configuration->estimatePositions[mesh->estimatePositionsOffset + i];
            if (showStatus) cout << mesh->vertices[i][0] << ' ' << mesh->vertices[i][1] << ' ' << mesh->vertices[i][2] << "\n\n";
        }
    }

    // Update velocities of colliding vertices
    /*
    #pragma omp parallel for
    for (int c = 0; c < configuration->collisionConstraints.size(); c++) {
        updateCollisionVelocities(configuration->collisionConstraints[c]);
    }
    */
}

void Simulation::generateCollisionConstraints(Configuration* configuration, Mesh *mesh, int index) {

    // Setup ray
    Vector3f rayOrigin = mesh->vertices[index];
    Vector3f vertexToEstimate = configuration->estimatePositions[mesh->estimatePositionsOffset + index] - mesh->vertices[index];
    Vector3f rayDirection = Vector3f(vertexToEstimate);
    rayDirection.normalize();

    // Setup intersection variables
    float t = INFINITY;
    Vector3f normal;
    int triangleIndex;

    // Dynamic object collision
    if (mesh->dynamicCollisionTest)
    {
        for (Mesh* dynamicMesh : configuration->simulatedObjects)
        {
            if (!dynamicMesh->isLineBased()) continue;
            auto dynamicMesh_ = dynamic_cast<LineBasedMesh*>(dynamicMesh);
            if (dynamicMesh == mesh && !mesh->selfCollisionTest) continue;
            if (!dynamicMesh->dynamicCollisionTest) continue;

            bool meshCollision = dynamicMesh_->intersect(rayOrigin, rayDirection, t, normal, index + mesh->estimatePositionsOffset, triangleIndex);

            t = fabs(t);

            if (meshCollision && 0 < t && t * 0.5 <= COLLISION_THRESHOLD) {
                SimpleTriangle triangle = dynamicMesh_->triangles[triangleIndex];

                if ((dynamicMesh->vertices[triangle.v[0].p] - mesh->vertices[index]).dot(normal) > 0.0f) {
                    configuration->collisionConstraints.push_back(buildTriangleCollisionConstraint(mesh, index, normal, COLLISION_THRESHOLD, triangle.v[0].p, triangle.v[1].p, triangle.v[2].p, dynamicMesh));
                }
                else {
                    configuration->collisionConstraints.push_back(buildTriangleCollisionConstraint(mesh, index, normal, COLLISION_THRESHOLD, triangle.v[0].p, triangle.v[2].p, triangle.v[1].p, dynamicMesh));
                }
            }
        }
    }

    // Static mesh collision
    for (Mesh* staticMesh : configuration->staticObjects) {
        if (!staticMesh->isRigidBody || !staticMesh->isLineBased()) continue;

        bool meshCollision = dynamic_cast<LineBasedMesh*>(staticMesh)->intersect(rayOrigin, rayDirection, t, normal, index + mesh->estimatePositionsOffset, triangleIndex);

        // If a collision occured
        if (meshCollision && fabs(t) * 0.5f <= (vertexToEstimate).norm() + COLLISION_THRESHOLD) {

            // Fix weird negative 0 issue
            if (normal[0] == -0) normal[0] = 0;
            if (normal[1] == -0) normal[1] = 0;
            if (normal[2] == -0) normal[2] = 0;

            // Account for the offset threshold
            if (t >= 0.0f) t -= COLLISION_THRESHOLD;
            else t += COLLISION_THRESHOLD;

            Vector3f intersectionPoint = (rayOrigin + t * rayDirection);

            // Build the collision constraint
            CollisionConstraint* constraint = buildStaticCollisionConstraint(mesh, index, normal, intersectionPoint);
            configuration->collisionConstraints.push_back(constraint);
        }
    }
}

// Parametric plane collision
bool Simulation::planeIntersection(Vector3f rayOrigin, Vector3f rayDirection, float &t, Vector3f &normal) {

    // Hardcoded plane position for now
    Vector3f position = Vector3f(0.0f, -3.0f, 0.0f);
    normal = Vector3f(0.0f, 1.0f, 0.0f);

    float num = (position - rayOrigin).dot(normal);
    float denom = normal.dot(rayDirection);

    // Ray is parallel with plane
    if (fabs(denom) < EPSILONTHRESHOLD) return false;

    t = num / denom;

    return true;
}

void Simulation::updateCollisionVelocities(CollisionConstraint* constraint) {
    Mesh* mesh = constraint->mesh;
    int index = constraint->indices[0] - mesh->estimatePositionsOffset;

    Vector3f updatedVelocity = mesh->velocities[index];

    // Reflect the velocity vector around the collision normal
    updatedVelocity = updatedVelocity - 2 * updatedVelocity.dot(constraint->normal) * constraint->normal;

    mesh->velocities[index] = updatedVelocity;

    // TODO Friction / restitution
}

void HelpMarker(const char* desc)
{
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered())
    {
        ImGui::BeginTooltip();
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
        ImGui::TextUnformatted(desc);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

void Simulation::renderGUI() {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    ImGuiStyle& style = ImGui::GetStyle();
    style.FrameRounding = 6.0f;
    style.GrabRounding = 6.0f;

    ImGui::Begin("Simulator");

    ImGui::Text("Scene Selection");

    int sceneNum = scene->sceneNum();
    char info[] = "Show Scene A";
    ImGuiWindowFlags window_flags = ImGuiWindowFlags_HorizontalScrollbar;
    ImGui::BeginChild("Child", ImVec2(0, 150), true, window_flags);
    for (int i = 0; i < sceneNum; i++)
    {
        const bool isSelected = (currentSceneIndex == i);
        info[11] = 'A' + i;
        if (ImGui::Selectable(info, isSelected))
        {
            currentSceneIndex = i;
        }
        switch (scene->allowMoveStatus(i))
        {
        case KeyBoardControlling::NOT_ALLOW_ANY_MOVE:
            break;

        case KeyBoardControlling::ALLOW_ALL_MOVE:
            ImGui::SameLine();
            HelpMarker("In this scene, you can use keys UP, DOWN, LEFT, RIGHT, COMMA and PERIOD to move all points.");
            break;

        case KeyBoardControlling::CONTROLLING_POINT_ALL_MOVE:
            ImGui::SameLine();
            HelpMarker("In this scene, you can use keys UP, DOWN, LEFT, RIGHT, COMMA and PERIOD to move controlling points.");
            break;

        case KeyBoardControlling::CONTROLLING_POINT_LEFT_RIGHT_MOVE:
            ImGui::SameLine();
            HelpMarker("In this scene, you can use keys LEFT and RIGHT to move controlling points.");
            break;

        case KeyBoardControlling::CONTROLLING_POINT_LEFT_RIGHT_UP_DOWN_MOVE:
            ImGui::SameLine();
            HelpMarker("In this scene, you can use keys UP, DOWN, LEFT and RIGHT to move controlling points.");
            break;

        case KeyBoardControlling::FIRST_OBJECT_ALL_MOVE:
            ImGui::SameLine();
            HelpMarker("In this scene, you can use keys UP, DOWN, LEFT, RIGHT, COMMA and PERIOD to move a object.");
            break;

        default:
            break;
        }
    }
    scene->setConfiguration(currentSceneIndex);
    ImGui::EndChild();

    ImGui::Text("Edit coefficients"); ImGui::SameLine();
    ImGui::Checkbox("##Editcoefficients", &adjustCoefficientWindow);

    ImGui::Text("Solver Iterations");
    ImGui::SliderInt("##solverIterations", &solverIterations, 1, 50, "%.0f");

    ImGui::Text("Timestep");
    ImGui::SliderFloat("##timeStep", &timeStep, 0.001f, 0.1f, "%.3f");

    ImGui::Text("Gravity");
    ImGui::SliderFloat("##gravity", &gravity, -10.0f, 10.0f, "%.2f");

    ImGui::Text("WindSpeed");
    ImGui::SliderFloat("##windSpeed", &windSpeed, 0.01f, 10.0f, "%.2f");

    ImGui::Text("Wireframe");
    ImGui::Checkbox("##wireframe", &wireframe);

    ImGui::Text("PBDType");
    ImGui::Combo("##PBDType", &type, "Normal\0XPBD\0\0");

    ImGui::Text("Damp Factor");
    ImGui::SliderFloat("##dampFactor", &dampFactor, 0.0f, 0.2f, "%.3f");

    ImGui::Text("\nDampType");
    ImGui::Combo("##dampType", &dampType, "None\0Simple\0Rotate\0Extended(in XPBD only)\0\0");

    ImGui::Text("ModelType");
    ImGui::Combo("##modelType", &constitutiveModelType, "StVK model\0Neo Hookean model\0\0");

    ImGui::End();

    if (adjustCoefficientWindow)
    {
        ImGui::Begin("Coefficient Editor");
        for (auto &mesh : scene->currentConfiguration->simulatedObjects)
        {
            if (!mesh->needCoef) continue;
            ImGui::Text(mesh->meshName.c_str());
            if (type == 0 && mesh->meshType == MeshType::triangular)
            {
                ImGui::Text("Stretch Factor");
                ImGui::SliderFloat((string("##stretchFactor") + mesh->meshName).c_str(), &(mesh->backupCoefData[0]), 0.0f, 1.0f);

                ImGui::Text("Bend Factor");
                ImGui::SliderFloat((string("##bendFactor") + mesh->meshName).c_str(), &(mesh->backupCoefData[1]), 0.0f, 1.0f);
            }
            else if (type == 1 && mesh->meshType == MeshType::triangular)
            {
                ImGui::Text("Stretch Compliance * 10^(-5)");
                ImGui::SliderFloat((string("##stretchFactor") + mesh->meshName).c_str(), &(mesh->backupCoefData[2]), 0.001f, 1.0f);

                ImGui::Text("Bend Compliance * 10^(-5)");
                ImGui::SliderFloat((string("##bendFactor") + mesh->meshName).c_str(), &(mesh->backupCoefData[3]), 0.001f, 1.0f);

                ImGui::Text("Damp Compliance * 10^(-5)");
                ImGui::SliderFloat((string("##dampFactor") + mesh->meshName).c_str(), &(mesh->backupCoefData[4]), 0.001f, 1.0f);
            }
            else if (mesh->meshType == MeshType::tetrahedral)
            {
                ImGui::Text("Poison's Ratio");
                ImGui::SliderFloat((string("##PoisonRatio") + mesh->meshName).c_str(), &(mesh->backupCoefData[0]), 0.001f, 0.499f);

                ImGui::Text("Young's Modulus");
                ImGui::SliderFloat((string("##YoungModulus") + mesh->meshName).c_str(), &(mesh->backupCoefData[1]), 1.0f, 100.f);
            }
            else if (mesh->meshType == MeshType::SPH)
            {
                ImGui::Text("Poison's Ratio");
                ImGui::SliderFloat((string("##PoisonRatio") + mesh->meshName).c_str(), &(mesh->backupCoefData[0]), 0.001f, 0.499f);

                ImGui::Text("Young's Modulus");
                ImGui::SliderFloat((string("##YoungModulus") + mesh->meshName).c_str(), &(mesh->backupCoefData[1]), 1.0f, 100.f);
            }

            ImGui::Text("Gravity"); ImGui::SameLine();
            ImGui::Checkbox((string("##Gravity") + mesh->meshName).c_str(), &(mesh->gravityAffected));
            ImGui::SameLine();
            ImGui::Text("Self Collision"); ImGui::SameLine();
            ImGui::Checkbox((string("##SelfCollision") + mesh->meshName).c_str(), &(mesh->selfCollisionTest));
            ImGui::SameLine();
            ImGui::Text("Dynamic Collision"); ImGui::SameLine();
            ImGui::Checkbox((string("##DynamicCollision") + mesh->meshName).c_str(), &(mesh->dynamicCollisionTest));
        }
        ImGui::End();
    }

    if (dampType == 3 && type != 1)
    {
        dampType = 0;
        warningLogWindow = true;
        warningMessage = "Extended damping can only be applied in XPBD mode.";
    }

    if (warningLogWindow)
    {
        ImGui::Begin("Warning");

        ImGui::Text(warningMessage.c_str());

        if (ImGui::Button("Close"))
        {
            warningLogWindow = false;
            warningMessage.clear();
        }
        ImGui::End();
    }

}
