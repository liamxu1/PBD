//
// Created by Jack Purvis
//

#include <iostream>
#include <main.hpp>
#include <constraint.hpp>

const float SPH_SELF_COLLISION_DETECTION_THRESHOLD = 5.0f;

void Constraint::preCompute(Configuration* configuration) {
    if (inverseMasses.empty())
        inverseMasses.resize(cardinality);
    updateInversedMasses();
}

void Constraint::updateInversedMasses()
{
    if (relatedMesh == nullptr)
    {
        for (int i = 0; i < cardinality; i++)
        {
            inverseMasses[i] = mesh->inverseMass[indices[i] - mesh->estimatePositionsOffset];
        }
    }
    else
    {
        // only collision / coupling

        inverseMasses[0] = mesh->inverseMass[indices[0] - mesh->estimatePositionsOffset];
        for (int i = 1; i < cardinality; i++)
        {
            inverseMasses[i] = relatedMesh->inverseMass[indices[i] - relatedMesh->estimatePositionsOffset];
        }
    }
}

void Constraint::commonOnProject(Configuration* configuration, Params params, float C, vector<Vector3f>& partialDerivatives, vector<float>& coeffs)
{
    switch (params.type)
    {
    case PBDType::normalPBD:
    {
        if (coeffs.size() == 0)
        {
            commonOnProjectNormal(configuration, params.solverIterations, C, partialDerivatives);
        }
        else
        {
            assert(coeffs.size() == 1);
            commonOnProjectNormal(configuration, params.solverIterations, C, partialDerivatives, coeffs[0]);
        }
        break;
    }

    case PBDType::XPBD:
    {
        if (coeffs.size() == 0) {
            commonOnProjectExtended(configuration, params.timeStep, C, partialDerivatives);
        }
        else
        {
            assert(coeffs.size() == 2);
            commonOnProjectExtended(configuration, params.timeStep, C, partialDerivatives, coeffs[0], coeffs[1]);
        }
        break;
    }

    default:
    {
        string currentConstraintType = typeNameString.at(type);
        cout << currentConstraintType + static_cast<string>(" projection not finished in this PBD type!\nYou may have faced incorrect simulation.\n\n");
        break;
    }

    }
}

void Constraint::commonOnProjectNormal(Configuration* configuration, int iteration, float C, vector<Vector3f>& partialDerivatives, float factor)
{
    float wSum = 0;
    for (int i = 0; i < cardinality; i++)
    {
        wSum += inverseMasses[i] * partialDerivatives[i].squaredNorm();
    }

    if (wSum < EPSILONTHRESHOLD) return;

    if (showStatus) cout << typeNameString.at(type) << ":\n";
    float s = C / wSum;
    float kNew = 1.0f - pow(1.0f - factor, 1.0f / iteration);
    for (int i = 0; i < cardinality; i++)
    {
        Vector3f& vertex = configuration->estimatePositions[indices[i]];

        if (showStatus) cout << indices[i] << ":\t" << vertex[0] << ' ' << vertex[1] << ' ' << vertex[2] << "\t->\t";
        vertex -= kNew * s * inverseMasses[i] * partialDerivatives[i];
        if (showStatus) cout << vertex[0] << ' ' << vertex[1] << ' ' << vertex[2] << '\n';
    }
    if (showStatus) cout << '\n';
}

void Constraint::commonOnProjectExtended(Configuration* configuration, float timeStep, float C, vector<Vector3f>& partialDerivatives, float compliance, float dampCompliance)
{
    float wSum = 0;
    for (int i = 0; i < cardinality; i++)
    {
        wSum += inverseMasses[i] * partialDerivatives[i].squaredNorm();
    }

    if (wSum < EPSILONTHRESHOLD) return;

    if (showStatus) cout << typeNameString.at(type) << ":\n";
    vector<float> dlambda(cardinality, 0.f);
    float alpha = compliance / powf(timeStep, 2);
    float gamma = alpha * dampCompliance * timeStep;
    for (int i = 0; i < cardinality; i++)
    {
        dlambda[i] = (-C - alpha * configuration->lambda[indices[i]]
            - gamma * partialDerivatives[i].dot(configuration->estimatePositions[indices[i]] - configuration->currentPositions[indices[i]]))
            / ((1 + gamma) * wSum + alpha);
    }
    for (int i = 0; i < cardinality; i++)
    {
        configuration->lambda[indices[i]] += dlambda[i];
        Vector3f& vertex = configuration->estimatePositions[indices[i]];

        if (showStatus) cout << indices[i] << ":\t" << vertex[0] << ' ' << vertex[1] << ' ' << vertex[2] << "\t->\t";
        vertex += dlambda[i] * inverseMasses[i] * partialDerivatives[i];
        if (showStatus) cout << vertex[0] << ' ' << vertex[1] << ' ' << vertex[2] << '\n';
    }
    if (showStatus) cout << '\n';
}

void buildEdgeConstraints(Configuration* configuration, TriangularMesh* mesh) {

    // Build a distance constraint along each edge
    for (Edge edge : mesh->edges) {
        int v0 = edge.v[0].p;
        int v1 = edge.v[1].p;

        buildDistanceConstraint(configuration, mesh, v0, v1, (mesh->vertices[v0] - mesh->vertices[v1]).norm(), true);
    }
}

void buildRigidBodyConstraints(Configuration* configuration, Mesh* mesh) {

    // Build a distance constraint between each vertex
    for (int i = 0; i < mesh->numVertices; i++) {
        for (int j = i + 1; j < mesh->numVertices; j++) {
            buildDistanceConstraint(configuration, mesh, i, j, (mesh->vertices[i] - mesh->vertices[j]).norm());
        }
    }
}

void buildBendConstraints(Configuration* configuration, TriangularMesh* mesh) {

    // Build a bend constraint between each pair of triangles that share an edge
    for (auto edge : mesh->edges) {

        // Skip edges with only one adjacent triangle
        if (mesh->adjacentTriangles[edge].size() != 2) continue;

        auto t1 = mesh->adjacentTriangles[edge][0];
        auto t2 = mesh->adjacentTriangles[edge][1];

        int p1 = edge.v[0].p; // Shared vertex 1
        int p2 = edge.v[1].p; // Shared vertex 2

        // Determine vertex 3
        int p3;
        for (auto v : t1.v) {
            int p = v.p;
            if (p1 != p && p2 != p) p3 = p;
        }

        // Determine vertex 4
        int p4;
        for (auto v : t2.v) {
            int p = v.p;
            if (p1 != p && p2 != p) p4 = p;
        }

        Vector3f p2_ = mesh->vertices[p2] - mesh->vertices[p1], p3_ = mesh->vertices[p3] - mesh->vertices[p1], p4_ = mesh->vertices[p4] - mesh->vertices[p1];

        // Compute the initial dot product between the two triangles
        Vector3f n1 = p2_.cross(p3_);
        Vector3f n2 = p2_.cross(p4_);
        n1.normalize();
        n2.normalize();
        float d = n1.dot(n2);

        buildBendConstraint(configuration, mesh, p1, p2, p3, p4, acosf(d));
    }
}

void buildTwoWayCouplingConstraints(Configuration* configuration, Mesh* meshA) {

    // Build a distance constraints between objects if they have vertices that overlap
    for (Mesh* meshB : configuration->simulatedObjects) {
        if (meshA == meshB) continue;

        for (int i = 0; i < meshA->numVertices; i++) {
            for (int j = 0; j < meshB->numVertices; j++) {
                if (meshA->vertices[i] == meshB->vertices[j]) {
                    buildDistanceConstraint(configuration, meshA, i, j, 0.0f, false, meshB);
                }
            }
        }
    }
}

void buildFixedConstraint(Configuration* configuration, Mesh* mesh, int index, Vector3f target) {

    mesh->inverseMass[index] = EPSILON;

    Constraint* constraint = new FixedConstraint(mesh, 1, target);
    constraint->indices.push_back(index + mesh->estimatePositionsOffset);

    configuration->constraints.push_back(constraint);
}

void buildDistanceConstraint(Configuration* configuration, Mesh* mesh, int indexA, int indexB, float distance, bool useMeshCoef, Mesh* secondMesh) {
    Constraint* constraint = new DistanceConstraint(mesh, 2, distance, useMeshCoef);
    constraint->indices.push_back(indexA + mesh->estimatePositionsOffset);

    // If a second mesh is provided we are building a constraint between two dynamic objects
    if (secondMesh != nullptr) {
        constraint->indices.push_back(indexB + secondMesh->estimatePositionsOffset);
    } else {
        constraint->indices.push_back(indexB + mesh->estimatePositionsOffset);
    }

    if (secondMesh)
        constraint->relatedMesh = secondMesh;
    constraint->preCompute(configuration);

    configuration->constraints.push_back(constraint);
}

void buildBendConstraint(Configuration* configuration, Mesh* mesh, int indexA, int indexB, int indexC, int indexD, float angle) {
    Constraint* constraint = new BendConstraint(mesh, 4, angle, true);
    constraint->indices.push_back(indexA + mesh->estimatePositionsOffset);
    constraint->indices.push_back(indexB + mesh->estimatePositionsOffset);
    constraint->indices.push_back(indexC + mesh->estimatePositionsOffset);
    constraint->indices.push_back(indexD + mesh->estimatePositionsOffset);

    constraint->preCompute(configuration);
    configuration->constraints.push_back(constraint);
}

CollisionConstraint* buildStaticCollisionConstraint(Mesh* mesh, int index, Vector3f normal, Vector3f position) {
    CollisionConstraint* constraint = new StaticCollisionConstraint(mesh, 1, normal, position);
    constraint->indices.push_back(index + mesh->estimatePositionsOffset);

    return constraint;
}

CollisionConstraint* buildTriangleCollisionConstraint(Mesh *mesh, int vertexIndex, Vector3f normal, float height, int indexA, int indexB, int indexC, Mesh* secondMesh) {
    CollisionConstraint* constraint = new TriangleCollisionConstraint(mesh, 4, normal, height);
    constraint->indices.push_back(vertexIndex + mesh->estimatePositionsOffset);
    constraint->indices.push_back(indexA + secondMesh->estimatePositionsOffset);
    constraint->indices.push_back(indexB + secondMesh->estimatePositionsOffset);
    constraint->indices.push_back(indexC + secondMesh->estimatePositionsOffset);

    constraint->relatedMesh = secondMesh;
    return constraint;
}

void buildSPHDeformationConstraints(Configuration* configuration, SPHMesh* mesh)
{
    int numVertices = mesh->numVertices;
    for (int i = 0; i < numVertices; i++)
    {
        vector<int> indices;
        indices.clear();
        auto node = mesh->kernelInfos[i];
        while (node != nullptr)
        {
            indices.push_back(node->index + mesh->estimatePositionsOffset);
            node = node->next;
        }
        SPHDeformationConstraint* constraint = new SPHDeformationConstraint(mesh, indices.size(), true);
        constraint->indices = indices;
        constraint->preCompute(configuration);

        configuration->constraints.push_back(constraint);
    }
}

void buildSPHCollisionConstraintsOnOneList(Configuration* configuration, SPHMesh* meshA, SPHMesh* meshB, vector<pair<unsigned, float>>& listA, vector<pair<unsigned, float>>& listB, bool threshold)
{
    for (auto& vertexA : listA)
    {
        unsigned indexA = vertexA.first;
        float radiusA = vertexA.second;
        for (auto& vertexB : listB)
        {
            unsigned indexB = vertexB.first;
            if (threshold && (configuration->estimatePositions[indexA + meshA->estimatePositionsOffset] - configuration->estimatePositions[indexB + meshB->estimatePositionsOffset]).norm() < SPH_SELF_COLLISION_DETECTION_THRESHOLD)
                continue;
            float radiusB = vertexB.second;
            if ((configuration->estimatePositions[indexA + meshA->estimatePositionsOffset] - configuration->estimatePositions[indexB + meshB->estimatePositionsOffset]).norm() < radiusA + radiusB)
            {
                PointCollisionConstraint* constraint = new PointCollisionConstraint(meshA, 2, radiusA + radiusB);
                constraint->relatedMesh = meshB;
                constraint->indices.push_back(indexA + meshA->estimatePositionsOffset);
                constraint->indices.push_back(indexB + meshB->estimatePositionsOffset);

                constraint->preCompute(configuration);
                configuration->collisionConstraints.push_back(constraint);
            }
        }
    }
}

void buildSPHSelfCollisionConstraints(Configuration* configuration, SPHMesh* mesh)
{
    if (!mesh->selfCollisionTest) return;

    buildSPHCollisionConstraintsOnOneList(configuration, mesh, mesh, mesh->firstCollidingInfos, mesh->firstCollidingInfos, true);
    buildSPHCollisionConstraintsOnOneList(configuration, mesh, mesh, mesh->firstCollidingInfos, mesh->secondCollidingInfos, true);
}

void buildSPHCollisionConstraints(Configuration* configuration, SPHMesh* meshA, SPHMesh* meshB)
{
    if (!meshA->dynamicCollisionTest || !meshB->dynamicCollisionTest) return;
    if (meshA->estimatePositionsOffset == meshB->estimatePositionsOffset) return;

    buildSPHCollisionConstraintsOnOneList(configuration, meshA, meshB, meshA->firstCollidingInfos, meshB->firstCollidingInfos, false);
    buildSPHCollisionConstraintsOnOneList(configuration, meshA, meshB, meshA->firstCollidingInfos, meshB->secondCollidingInfos, false);
    buildSPHCollisionConstraintsOnOneList(configuration, meshA, meshB, meshA->secondCollidingInfos, meshB->firstCollidingInfos, false);
    buildSPHCollisionConstraintsOnOneList(configuration, meshA, meshB, meshA->secondCollidingInfos, meshB->secondCollidingInfos, false);
}

void buildTetrahedralConstraints(Configuration* configuration, TetrahedralMesh* mesh)
{
    for (auto &tetra : mesh->tetrahedrons)
    {
        int indexA = tetra.v[0].p;
        int indexB = tetra.v[1].p;
        int indexC = tetra.v[2].p;
        int indexD = tetra.v[3].p;

        Matrix3f originalShape = Matrix3f::Zero();
        originalShape << mesh->vertices[indexA] - mesh->vertices[indexD], mesh->vertices[indexB] - mesh->vertices[indexD], mesh->vertices[indexC] - mesh->vertices[indexD];

        auto constraint = buildTetrahedralConstraint(mesh, indexA, indexB, indexC, indexD, originalShape, true);
        constraint->preCompute(configuration);

        configuration->constraints.push_back(constraint);
    }
}

TetrahedralConstraint* buildTetrahedralConstraint(Mesh* mesh, int indexA, int indexB, int indexC, int indexD, Matrix3f originalShape, bool useMeshCoef)
{
    TetrahedralConstraint* constraint = new TetrahedralConstraint(mesh, 4, originalShape, useMeshCoef);
    constraint->indices.push_back(indexA + mesh->estimatePositionsOffset);
    constraint->indices.push_back(indexB + mesh->estimatePositionsOffset);
    constraint->indices.push_back(indexC + mesh->estimatePositionsOffset);
    constraint->indices.push_back(indexD + mesh->estimatePositionsOffset);

    return constraint;
}

void FixedConstraint::project(Configuration* configuration, Params params) {
    if (params.type == PBDType::none)
    {
        cout << "Fixed constraint projection not finished in this PBD type!\nYou may have faced incorrect simulation.\n\n";
        return;
    }
    Vector3f& vertex = configuration->estimatePositions[indices[0]];

    if (showStatus) cout << "Static collision:\n";
    if (showStatus) cout << indices[0] << ":\t" << vertex[0] << ' ' << vertex[1] << ' ' << vertex[2] << "\t->\t";
    vertex = target;
    if (showStatus) cout << vertex[0] << ' ' << vertex[1] << ' ' << vertex[2] << "\n\n";
}

void DistanceConstraint::project(Configuration* configuration, Params params) {
    
    assert(cardinality == 2);

    float w1 = inverseMasses[0], w2 = inverseMasses[1];
    if (w1 + w2 < EPSILONTHRESHOLD) return;

    Vector3f p1 = configuration->estimatePositions[indices[0]];
    Vector3f p2 = configuration->estimatePositions[indices[1]];

    float a = (p1 - p2).norm() - distance;
    Vector3f n = (p1 - p2) / ((p1 - p2).norm() + EPSILON);
    Vector3f b = n * a;

    vector<Vector3f> partialDerivatives = { n, -n };

    vector<float> coeffs;
    if (mesh->meshType == MeshType::triangular)
    {
        if (params.type == PBDType::normalPBD)
        {
            if (useMeshCoef)
                commonOnProjectNormal(configuration, params.solverIterations, a, partialDerivatives, dynamic_cast<TriangularMesh*>(mesh)->stretchFactor);
            else
                commonOnProjectNormal(configuration, params.solverIterations, a, partialDerivatives);
        }
        else if (params.type == PBDType::XPBD)
        {
            if (useMeshCoef)
            {
                if (params.useXPBDDamp)
                    commonOnProjectExtended(configuration, params.timeStep, a, partialDerivatives, dynamic_cast<TriangularMesh*>(mesh)->stretchCompliance, dynamic_cast<TriangularMesh*>(mesh)->dampCompliance);
                else
                    commonOnProjectExtended(configuration, params.timeStep, a, partialDerivatives, dynamic_cast<TriangularMesh*>(mesh)->stretchCompliance);
            }
            else
            {
                commonOnProjectExtended(configuration, params.timeStep, a, partialDerivatives);
            }
        }
    }
    else
        commonOnProjectNormal(configuration, params.solverIterations, a, partialDerivatives);
}

void BendConstraint::project(Configuration* configuration, Params params) {
    assert(cardinality == 4);

    Vector3f p1 = configuration->estimatePositions[indices[0]];
    Vector3f p2 = configuration->estimatePositions[indices[1]];
    Vector3f p3 = configuration->estimatePositions[indices[2]];
    Vector3f p4 = configuration->estimatePositions[indices[3]];
    p2 -= p1, p3 -= p1, p4 -= p1;

    Vector3f p2Xp3 = p2.cross(p3);
    Vector3f p2Xp4 = p2.cross(p4);

    float p2Xp3Norm = p2Xp3.norm(), p2Xp4Norm = p2Xp4.norm();
    if (p2Xp3Norm < EPSILONTHRESHOLD || p2Xp4Norm < EPSILONTHRESHOLD)
    {
        //cout << "p2\n" << p2 << '\n' << "p3\n" << p3 << '\n' << "p4\n" << p4 << '\n';
        return;
    }

    // Compute normals
    Vector3f n1 = p2Xp3 / p2Xp3Norm;
    Vector3f n2 = p2Xp4 / p2Xp4Norm;
    float d = n1.dot(n2);

    // Prevent issue where d falls out of range -1 to 1
    d = fmax(fmin(d, 1.0f), -1.0f);

    Vector3f q3 = (p2.cross(n2) + d * n1.cross(p2)) / p2Xp3Norm;
    Vector3f q4 = (p2.cross(n1) + d * n2.cross(p2)) / p2Xp4Norm;
    Vector3f q2 = -(p3.cross(n2) + d * n1.cross(p3)) / p2Xp3Norm - (p4.cross(n1) + d * n2.cross(p4)) / p2Xp4Norm;
    Vector3f q1 = -q2 - q3 - q4;

    float coef = sqrtf(1.0f - d * d + EPSILON);
    vector<Vector3f> partialDerivatives = { q1 / coef,q2 / coef,q3 / coef,q4 / coef };

    vector<float> coeffs;
    if (mesh->meshType == MeshType::triangular)
    {
        if (params.type == PBDType::normalPBD)
        {
            if (useMeshCoef)
                commonOnProjectNormal(configuration, params.solverIterations, acosf(d) - angle, partialDerivatives, dynamic_cast<TriangularMesh*>(mesh)->bendFactor);
            else
                commonOnProjectNormal(configuration, params.solverIterations, acosf(d) - angle, partialDerivatives);
        }
        else if (params.type == PBDType::XPBD)
        {
            if (useMeshCoef)
            {
                if (params.useXPBDDamp)
                    commonOnProjectExtended(configuration, params.timeStep, acosf(d) - angle, partialDerivatives, dynamic_cast<TriangularMesh*>(mesh)->bendCompliance, dynamic_cast<TriangularMesh*>(mesh)->dampCompliance);
                else
                    commonOnProjectExtended(configuration, params.timeStep, acosf(d) - angle, partialDerivatives, dynamic_cast<TriangularMesh*>(mesh)->bendCompliance);
            }
            else
            {
                commonOnProjectExtended(configuration, params.timeStep, acosf(d) - angle, partialDerivatives);
            }
        }
    }
    else
        commonOnProjectNormal(configuration, params.solverIterations, acosf(d) - angle, partialDerivatives);

}

void StaticCollisionConstraint::project(Configuration* configuration, Params params) {
    Vector3f p = configuration->estimatePositions[indices[0]];

    Vector3f pointToPosition = p - position;
    pointToPosition.normalize();

    // Check if constraint is already satisfied
    if (pointToPosition.dot(normal) >= 0.0f) return;

    if (showStatus) cout << "Static collision:\n";

    float a = (p - position).dot(normal);
    Vector3f b = (p - position) / ((p - position).norm());

    Vector3f displacement = a * b;

    Vector3f& vertex = configuration->estimatePositions[indices[0]];

    if (showStatus) cout << indices[0] << ":\t" << vertex[0] << ' ' << vertex[1] << ' ' << vertex[2] << "\t->\t";
    vertex += displacement;
    if (showStatus) cout << vertex[0] << ' ' << vertex[1] << ' ' << vertex[2] << "\n\n";

    isCollisionHappened = true;
    if(params.useFriction)
        commonFrictionProjecting(configuration, -pointToPosition.dot(normal), params.staticFrictionCoef, params.kineticFrictionCoef);
}

void TriangleCollisionConstraint::project(Configuration* configuration, Params params) {
    preCompute(configuration);

    Vector3f q = configuration->estimatePositions[indices[0]];
    Vector3f p1 = configuration->estimatePositions[indices[1]];
    Vector3f p2 = configuration->estimatePositions[indices[2]];
    Vector3f p3 = configuration->estimatePositions[indices[3]];

    // Check if constraint is already satisfied
    Vector3f q_ = q - p1, p2_ = p2 - p1, p3_ = p3 - p1;
    Vector3f n = p2_.cross(p3_);
    float normalLength = n.norm();
    if (normalLength < EPSILONTHRESHOLD)
        return;

    normal = n / normalLength;

    Matrix3f cordMat = Matrix3f::Zero();
    cordMat << p2_, p3_, normal;
    Vector3f cord = cordMat.inverse() * q_;

    if (cord[0] < 0.f || cord[1] < 0.f || cord[0] + cord[1]>1.f) return;

    float d = q_.dot(normal);
    float c = d - height;

    if (c > 0) return;

    Vector3f dc_p2 = (p3_.cross(q_) + d * normal.cross(p3_)) / normalLength;
    Vector3f dc_p3 = -(p2_.cross(q_) + d * normal.cross(p2_)) / normalLength;
    Vector3f dc_p1 = -dc_p2 - dc_p3 - normal;
    vector<Vector3f> partialDerivatives = { normal,dc_p1,dc_p2,dc_p3 };

    commonOnProjectNormal(configuration, params.solverIterations, c, partialDerivatives);

    isCollisionHappened = true;
    if (params.useFriction)
        commonFrictionProjecting(configuration, -c, params.staticFrictionCoef, params.kineticFrictionCoef);
}

void PointCollisionConstraint::project(Configuration* configuration, Params params) {

    assert(cardinality == 2);

    float w1 = inverseMasses[0], w2 = inverseMasses[1];
    if (w1 + w2 < EPSILONTHRESHOLD) return;

    Vector3f p1 = configuration->estimatePositions[indices[0]];
    Vector3f p2 = configuration->estimatePositions[indices[1]];

    float a = (p1 - p2).norm() - distance;

    if (a > 0) return;

    Vector3f n = (p1 - p2) / ((p1 - p2).norm() + EPSILON);
    Vector3f b = n * a;

    normal = n;

    vector<Vector3f> partialDerivatives = { n, -n };
    
    commonOnProjectNormal(configuration, params.solverIterations, a, partialDerivatives);

    isCollisionHappened = true;
    if (params.useFriction)
        commonFrictionProjecting(configuration, -a, params.staticFrictionCoef, params.kineticFrictionCoef);
}

pair<Matrix3f, float> calculateStressTensorAndStressEnergyDensity(Matrix3f F, ConstitutiveMaterialModel modelType, float PoisonRatio, float YoungModulus)
{
    float nu = PoisonRatio, k = YoungModulus;
    // lame coefficients
    float mu = k / (2.f * (1.f + nu)), lambda = k * nu / ((1.f + nu) * (1.f - 2.f * nu));
    float phi = 0.f;

    JacobiSVD<MatrixXf> svd(F, ComputeFullU | ComputeFullV);
    Matrix3f U = svd.matrixU();
    Matrix3f V = svd.matrixV();
    Vector3f F_ = svd.singularValues();
    
    if (F.determinant() < 0)
    {
        F_[2] = -F_[2];
        assert(U.determinant() * V.determinant() < 0);
        if (U.determinant() > 0)
        {
            V(0, 2) = -V(0, 2);
            V(1, 2) = -V(1, 2);
            V(2, 2) = -V(2, 2);
        }
        else
        {
            U(0, 2) = -U(0, 2);
            U(1, 2) = -U(1, 2);
            U(2, 2) = -U(2, 2);
        }
    }
    
    Vector3f P = Vector3f::Zero();

    switch (modelType)
    {
    case ConstitutiveMaterialModel::StVKModel:
    {
        Vector3f epsilon = Vector3f::Zero();
        float e = 0;
        for (size_t i = 0; i < 3; i++)
        {
            epsilon[i] = (powf(F_[i], 2) - 1) / 2;
            e += epsilon[i];
            if (F_[i] < 0.58) {
                F_[i] = 0.58;
                epsilon[i] = (powf(F_[i], 2) - 1) / 2;
            }
        }
        for (size_t i = 0; i < 3; i++)
        {
            float s = 2 * mu * epsilon[i] + lambda * e;
            P[i] = s * F_[i];
            phi += 0.5 * epsilon[i] * s;
        }
        break;
    }

    case ConstitutiveMaterialModel::NeoHookeanModel:
    {
        float I1 = F_.squaredNorm(), I3 = powf(F_[0] * F_[1] * F_[2], 2);
        assert(fabs(I3) > EPSILONTHRESHOLD);

        for (size_t i = 0; i < 3; i++)
        {
            if (F_[i] < 0.5) {
                F_[i] = 0.5;
            }
            P[i] = mu * F_[i] - mu / F_[i] + lambda * logf(I3) * 0.5f / F_[i];
        }

        phi = mu / 2 * (I1 - logf(I3) - 3) + lambda / 8 * powf(logf(I3), 2);
        break;
    }

    default:
    {
        cout << "Wrong model type faced when calculating stress tensor!\n";
        break;
    }

    }

    return pair<Matrix3f, float>(U * P.asDiagonal() * V.transpose(), phi);
}

void TetrahedralConstraint::project(Configuration* configuration, Params params)
{
    // calculate new shape
    Matrix3f newShape = Matrix3f::Zero();
    vector<Vector3f> currentPositions(4);
    for (int i = 0; i < 4; i++)
    {
        currentPositions[i] = configuration->estimatePositions[indices[i]];
    }
    newShape << currentPositions[0] - currentPositions[3], currentPositions[1] - currentPositions[3], currentPositions[2] - currentPositions[3];

    // deformation gradient
    Matrix3f F = newShape * inversedOriginalShape;

    float PoisonRatio = dynamic_cast<TetrahedralMesh*>(mesh)->poisonRatio, YoungModulus = dynamic_cast<TetrahedralMesh*>(mesh)->YoungModulus;

    auto temp = calculateStressTensorAndStressEnergyDensity(F, params.modelType, PoisonRatio, YoungModulus);

    // stress tensor
    Matrix3f P = temp.first;

    // energy density
    float phi = temp.second;

    float energy = phi * initialVolume;
    Matrix3f partialDerivativesInMatrix = initialVolume * P * inversedOriginalShape.transpose();

    vector<Vector3f> partialDerivatives(4, Vector3f::Zero());
    for (int i = 0; i < 3; i++)
    {
        partialDerivatives[i] = partialDerivativesInMatrix.col(i);
        partialDerivatives[3] -= partialDerivatives[i];
    }

    if (params.type == PBDType::normalPBD)
        commonOnProjectNormal(configuration, params.solverIterations, energy, partialDerivatives);
    else if (params.type == PBDType::XPBD)
        commonOnProjectExtended(configuration, params.timeStep, energy, partialDerivatives, 1e-9);

}

void SPHDeformationConstraint::project(Configuration* configuration, Params params)
{
    /* first index: current vertex
    * following indices: adjacent vertices
    * (in the same order of that restored in mesh's kernel information)
    */

    assert(mesh->meshType == MeshType::SPH);
    int index = indices[0];
    int indexInMesh = index - mesh->estimatePositionsOffset;

    SPHMesh* sphMesh = dynamic_cast<SPHMesh*>(mesh);

    float PoisonRatio = sphMesh->poisonRatio, YoungModulus = sphMesh->YoungModulus;

    // Deformation gradient
    Matrix3f F = Matrix3f::Zero();

    auto node = sphMesh->kernelInfos[indexInMesh];
    vector<Vector3f> correctedGradients;
    correctedGradients.reserve(cardinality);
    while (node != nullptr)
    {
        F += sphMesh->volumes[node->index] *
            (configuration->estimatePositions[node->index + mesh->estimatePositionsOffset]\
                - configuration->estimatePositions[index]) *
            node->correctedGradient.transpose();
        correctedGradients.push_back(node->correctedGradient * sphMesh->volumes[node->index]);
        node = node->next;
    }

    auto temp = calculateStressTensorAndStressEnergyDensity(F, params.modelType, PoisonRatio, YoungModulus);

    // stress tensor
    Matrix3f P = temp.first;

    // energy density
    float phi = temp.second;

    vector<Vector3f> partialDerivatives(cardinality, Vector3f::Zero());

    for (int i = 1; i < cardinality; i++)
    {
        partialDerivatives[i] = P * correctedGradients[i];
        partialDerivatives[0] -= partialDerivatives[i];
    }

    if (params.type == PBDType::normalPBD)
        commonOnProjectNormal(configuration, params.solverIterations, phi, partialDerivatives);
    else if (params.type == PBDType::XPBD)
        commonOnProjectExtended(configuration, params.timeStep, phi, partialDerivatives, 1e-5);
}

void CollisionConstraint::commonFrictionProjecting(Configuration* configuration, float penetrationDepth, float staticFrictionCoef, float kineticFrictionCoef)
{
    auto info = this->CollisionVertexIndices();
    for (int infoIndex = 0; infoIndex < info.size(); infoIndex++)
    {
        auto pair = info[infoIndex];
        int i = pair.first, j = pair.second;
        if (j > 0)
        {
            Vector3f relativeDisplacement = (configuration->estimatePositions[i] - configuration->currentPositions[i]) - (configuration->estimatePositions[j] - configuration->currentPositions[j]);
            Vector3f tangentialRelativeDisplacement = relativeDisplacement - relativeDisplacement.dot(normal) * normal;
            float tangentialNorm = tangentialRelativeDisplacement.norm();

            float wi = inverseMasses[0], wj = inverseMasses[infoIndex + 1];
            Vector3f dxi = Vector3f::Zero();
            if (tangentialNorm < staticFrictionCoef * penetrationDepth)
            {
                dxi = wi / (wi + wj) * tangentialRelativeDisplacement;
            }
            else
                dxi = wi / (wi + wj) * tangentialRelativeDisplacement * min<float>(1.f, kineticFrictionCoef * penetrationDepth / tangentialNorm);

            configuration->estimatePositions[i] -= dxi;
            configuration->estimatePositions[j] += wj / (wi + wj) * dxi;
        }
        else
        {
            Vector3f relativeDisplacement = configuration->estimatePositions[i] - configuration->currentPositions[i];
            Vector3f tangentialRelativeDisplacement = relativeDisplacement - relativeDisplacement.dot(normal) * normal;
            float tangentialNorm = tangentialRelativeDisplacement.norm();

            Vector3f dxi = Vector3f::Zero();
            if (tangentialNorm < staticFrictionCoef * penetrationDepth)
            {
                dxi = tangentialRelativeDisplacement;
            }
            else
                dxi = tangentialRelativeDisplacement * min<float>(1.f, kineticFrictionCoef * penetrationDepth / tangentialNorm);
            configuration->estimatePositions[i] -= dxi;
        }
    }
}
