//
// Created by Jack Purvis
//

#include <iostream>
#include <main.hpp>
#include <constraint.hpp>

void Constraint::preCompute(Configuration* configuration) {
    inverseMasses.resize(cardinality);
    for (int i = 0; i < cardinality; i++)
        inverseMasses[i] = configuration->inverseMasses[indices[i]];
}

void Constraint::commonOnProject(Configuration* configuration, Params params, float C, vector<Vector3f>& partialDerivatives, vector<float>& coeffs)
{
    float wSum = 0;
    for (int i = 0; i < cardinality; i++)
    {
        wSum += inverseMasses[i] * partialDerivatives[i].squaredNorm();
    }
    
    if (wSum < EPSILONTHRESHOLD) return;

    if (showStatus) cout << typeNameString.at(type) << ":\n";
    switch (params.type)
    {
    case PBDType::normalPBD:
    {
        assert(coeffs.size() == 1);
        float k = coeffs[0];
        float s = C / wSum;
        float kNew = 1.0f - pow(1.0f - k, 1.0f / params.solverIterations);
        for (int i = 0; i < cardinality; i++)
        {
            Vector3f& vertex = configuration->estimatePositions[indices[i]];

            if (showStatus) cout << indices[i] << ":\t" << vertex[0] << ' ' << vertex[1] << ' ' << vertex[2] << "\t->\t";
            vertex -= kNew * s * inverseMasses[i] * partialDerivatives[i];
            if (showStatus) cout << vertex[0] << ' ' << vertex[1] << ' ' << vertex[2] << '\n';
        }
        if (showStatus) cout << '\n';
        break;
    }

    case PBDType::XPBD:
    {
        assert(coeffs.size() == 2);
        float compliance = coeffs[0], dampFactor = coeffs[1];
        vector<float> dlambda(cardinality, 0.f);
        float alpha = compliance / powf(params.timeStep, 2);
        float gamma = alpha * dampFactor * params.timeStep;
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
    configuration->inverseMasses[index + mesh->estimatePositionsOffset] = EPSILON;
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

CollisionConstraint* buildTriangleCollisionConstraint(Mesh *mesh, int vertexIndex, Vector3f normal, float height, int indexA, int indexB, int indexC) {
    CollisionConstraint* constraint = new TriangleCollisionConstraint(mesh, 1, normal, height);
    constraint->indices.push_back(vertexIndex + mesh->estimatePositionsOffset);
    constraint->indices.push_back(indexA + mesh->estimatePositionsOffset);
    constraint->indices.push_back(indexB + mesh->estimatePositionsOffset);
    constraint->indices.push_back(indexC + mesh->estimatePositionsOffset);

    return constraint;
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
            float k = useMeshCoef ? dynamic_cast<TriangularMesh*>(mesh)->stretchFactor : 1.0f;
            coeffs.push_back(k);
        }
        else if (params.type == PBDType::XPBD)
        {
            coeffs.push_back(dynamic_cast<TriangularMesh*>(mesh)->stretchCompliance);
            if (params.useXPBDDamp)
                coeffs.push_back(dynamic_cast<TriangularMesh*>(mesh)->dampCompliance);
            else coeffs.push_back(0.f);
        }
    }
    else coeffs.push_back(1.0);
    commonOnProject(configuration, params, a, partialDerivatives, coeffs);
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
            coeffs.push_back(dynamic_cast<TriangularMesh*>(mesh)->bendFactor);
        }
        else if (params.type == PBDType::XPBD)
        {
            coeffs.push_back(dynamic_cast<TriangularMesh*>(mesh)->bendCompliance);
            if (params.useXPBDDamp)
                coeffs.push_back(dynamic_cast<TriangularMesh*>(mesh)->dampCompliance);
            else coeffs.push_back(0.f);
        }
    }
    else coeffs.push_back(1.0);
    commonOnProject(configuration, params, acosf(d) - angle, partialDerivatives, coeffs);

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
}

void TriangleCollisionConstraint::project(Configuration* configuration, Params params) {
    Vector3f q = configuration->estimatePositions[indices[0]];
    Vector3f p1 = configuration->estimatePositions[indices[1]];
    Vector3f p2 = configuration->estimatePositions[indices[2]];
    Vector3f p3 = configuration->estimatePositions[indices[3]];

    // Check if constraint is already satisfied
    Vector3f n = (p2 - p1).cross(p3 - p1);
    n /= n.norm();

    normal = n;

    Vector3f qToP1 = q - p1;
    qToP1.normalize();

    if (qToP1.dot(n) - height >= 0.0f) return;

    if (showStatus) cout << "Triangle collision:\n";

    float a = (q - p1).dot(n) - height;
    Vector3f b = n;

    Vector3f displacement = a * b;

    Vector3f& vertex = configuration->estimatePositions[indices[0]];

    if (showStatus) cout << indices[0] << ":\t" << vertex[0] << ' ' << vertex[1] << ' ' << vertex[2] << "\t->\t";
    vertex -= displacement;
    if (showStatus) cout << vertex[0] << ' ' << vertex[1] << ' ' << vertex[2] << "\n\n";
}

pair<Matrix3f, float> calculateStressTensorAndStressEnergyDensity(Matrix3f F, Params params, float PoisonRatio, float YoungModulus)
{
    float nu = PoisonRatio, k = YoungModulus;
    // lame coefficients
    float mu = k / (2.f * (1.f + nu)), lambda = k * nu / ((1.f + nu) * (1.f - 2.f * nu));

    Matrix3f P = Matrix3f::Zero();
    float phi = 0.f;

    switch (params.modelType)
    {
    case ConstitutiveMaterialModel::StVKModel:
    {
        float e = F.trace();
        Matrix3f epsilon = (F.transpose() * F - Matrix3f::Identity()) / 2;
        Matrix3f S = 2 * mu * epsilon + lambda * e * Matrix3f::Identity();
        P = F * S;
        phi = 0.5 * (epsilon.transpose() * S).trace();
        break;
    }

    case ConstitutiveMaterialModel::NeoHookeanModel:
    {
        Matrix3f FT = F.transpose();
        Matrix3f FTF = FT * F;
        float I1 = FTF.trace(), I3 = FTF.determinant();
        assert(fabs(I3) > EPSILONTHRESHOLD);
        Matrix3f FT_ = FT.inverse();

        P = mu * F - mu * FT_ + lambda * logf(I3) * 0.5f * FT_;
        phi = mu / 2 * (I1 - logf(I3) - 3) + lambda / 8 * powf(logf(I3), 2);
        break;
    }

    default:
    {
        cout << "Wrong model type faced when calculating stress tensor!\n";
        break;
    }

    }

    return pair<Matrix3f, float>(P, phi);
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

    float PoisonRatio = dynamic_cast<TetrahedralMesh*>(mesh)->poisonRatio, YoungModulus = dynamic_cast<TetrahedralMesh*>(mesh)->YongModulus;

    auto temp = calculateStressTensorAndStressEnergyDensity(F, params, PoisonRatio, YoungModulus);

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

    commonOnProject(configuration, params, energy, partialDerivatives);

}
