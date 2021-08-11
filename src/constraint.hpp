//
// Created by Jack Purvis
//

#ifndef POSITIONBASEDDYNAMICS_CONSTRAINT_HPP
#define POSITIONBASEDDYNAMICS_CONSTRAINT_HPP

#include <vector>
#include <Eigen>
#include <mesh.hpp>
#include <scene.hpp>

using namespace std;
using namespace Eigen;

// Forward declaration
class Mesh;
struct Configuration;

const bool SHOWALLINFO = false;

enum class PBDType
{
    normalPBD,
    XPBD,
    none
};

enum class ConstitutiveMaterialModel
{
    StVKModel,
    NeoHookeanModel
};

enum class ConstraintType
{
    NoneType,
    FixedConstraint,
    DistanceConstraint,
    BendConstraint,
    TetrahedralConstraint,
    StaticCollisionConstraint,
    TriangleCollisionConstraint
};

const static map<ConstraintType, string> typeNameString = {
    {ConstraintType::NoneType,"Undefined constraint"},
    {ConstraintType::FixedConstraint,"Fixed constraint"},
    {ConstraintType::DistanceConstraint,"Distance constraint"},
    {ConstraintType::BendConstraint,"Bend constraint"},
    {ConstraintType::TetrahedralConstraint,"Tetrahedral constraint"},
    {ConstraintType::StaticCollisionConstraint,"Static collision constraint"},
    {ConstraintType::TriangleCollisionConstraint,"Triangle collision constraint"}
};

struct Params {
    int solverIterations;
    float stretchFactor;
    float bendFactor;
    float timeStep;
    
    // for xpbd

    float compliance;
    float dampStiffness;

    // for continuous material

    float poisonRatio;
    float YoungModulus;
    
    PBDType type = PBDType::none;
    ConstitutiveMaterialModel modelType = ConstitutiveMaterialModel::NeoHookeanModel;

};

class Constraint {

public:
    Constraint(Mesh* mesh, int cardinality, bool showStatus = SHOWALLINFO) :
        mesh(mesh), cardinality(cardinality), showStatus(showStatus){}
    void preCompute(Configuration* configuration);
    virtual void project(Configuration* configuration, Params params) {}

    int cardinality;
    vector<int> indices;
    Mesh* mesh;
    vector<float> inverseMasses;

    ConstraintType type = ConstraintType::NoneType;

    bool showStatus;

protected:
    bool commonOnProject(Configuration* configuration, Params params, float C, vector<Vector3f>& partialDerivatives, float k = 1.0f);

};

class FixedConstraint : public Constraint {

public:
    FixedConstraint(Mesh* mesh, int cardinality, Vector3f target) :
        Constraint(mesh, cardinality), target(target) {
        type = ConstraintType::FixedConstraint;
    }
    void project(Configuration* configuration, Params params);

    Vector3f target;

};

class DistanceConstraint : public Constraint {

public:
    DistanceConstraint(Mesh* mesh, int cardinality, float distance) :
        Constraint(mesh, cardinality), distance(distance) {
        type = ConstraintType::DistanceConstraint;
    }
    void project(Configuration* configuration, Params params);

    float distance;
};

class BendConstraint : public Constraint {

public:
    BendConstraint(Mesh* mesh, int cardinality, float angle) :
        Constraint(mesh, cardinality), angle(angle) {
        type = ConstraintType::BendConstraint;
    }
    void project(Configuration* configuration, Params params);

    float angle;

};

class CollisionConstraint : public Constraint {

public:
    CollisionConstraint(Mesh* mesh, int cardinality, Vector3f normal) :
            Constraint(mesh, cardinality), normal(normal) {}
    virtual void project(Configuration* configuration, Params params) {}

    Vector3f normal;

};

class TetrahedralConstraint : public Constraint {

public:
    TetrahedralConstraint(Mesh* mesh, int cardinality, Matrix3f originalShape) :
        Constraint(mesh, cardinality), inversedOriginalShape(originalShape.inverse()), initialVolume(fabs(originalShape.determinant()) / 6.f) {
        type = ConstraintType::TetrahedralConstraint;
    }
    void project(Configuration* configuration, Params params);

    Matrix3f inversedOriginalShape;
    float initialVolume;
};

class StaticCollisionConstraint : public CollisionConstraint {

public:
    StaticCollisionConstraint(Mesh* mesh, int cardinality, Vector3f normal, Vector3f position) :
        CollisionConstraint(mesh, cardinality, normal), position(position) {
        type = ConstraintType::StaticCollisionConstraint;
    }
    void project(Configuration* configuration, Params params);

    Vector3f position;
};

class TriangleCollisionConstraint : public CollisionConstraint {

public:
    TriangleCollisionConstraint(Mesh* mesh, int cardinality, Vector3f normal, float height) :
        CollisionConstraint(mesh, cardinality, normal), height(height) {
        type = ConstraintType::TriangleCollisionConstraint;
    }
    void project(Configuration* configuration, Params params);

    float height;

};

// calculate stress tensor and stress energy density according to deformation gradient F
pair<Matrix3f, float> calculateStressTensorAndStressEnergyDensity(Matrix3f F, Params params);

// Constraint building
void buildEdgeConstraints(Configuration* configuration, TriangularMesh* mesh);
void buildRigidBodyConstraints(Configuration* configuration, Mesh* mesh);
void buildBendConstraints(Configuration* configuration, TriangularMesh* mesh);
void buildTwoWayCouplingConstraints(Configuration* configuration, Mesh* meshA);
void buildFixedConstraint(Configuration* configuration, Mesh* mesh, int index, Vector3f target);
void buildDistanceConstraint(Configuration* configuration, Mesh* mesh, int indexA, int indexB, float distance, Mesh* secondMesh = nullptr);
void buildBendConstraint(Configuration* configuration, Mesh* mesh, int indexA, int indexB, int indexC, int indexD, float angle);
void buildTetrahedralConstraints(Configuration* configuration, TetrahedralMesh* mesh);
TetrahedralConstraint* buildTetrahedralConstraint(Mesh* mesh, int indexA, int indexB, int indexC, int indexD, Matrix3f originalShape);
CollisionConstraint* buildStaticCollisionConstraint(Mesh* mesh, int index, Vector3f normal, Vector3f position);
CollisionConstraint* buildTriangleCollisionConstraint(Mesh *mesh, int vertexIndex, Vector3f normal, float height, int indexA, int indexB, int indexC);

#endif //POSITIONBASEDDYNAMICS_CONSTRAINT_HPP
