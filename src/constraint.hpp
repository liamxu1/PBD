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
    TriangleCollisionConstraint,
    PointCollisionConstraint,
    SPHDeformationConstraint
};

const static map<ConstraintType, string> typeNameString = {
    {ConstraintType::NoneType,"Undefined constraint"},
    {ConstraintType::FixedConstraint,"Fixed constraint"},
    {ConstraintType::DistanceConstraint,"Distance constraint"},
    {ConstraintType::BendConstraint,"Bend constraint"},
    {ConstraintType::TetrahedralConstraint,"Tetrahedral constraint"},
    {ConstraintType::StaticCollisionConstraint,"Static collision constraint"},
    {ConstraintType::TriangleCollisionConstraint,"Triangle collision constraint"},
    {ConstraintType::PointCollisionConstraint, "Point collision constraint"},
    {ConstraintType::SPHDeformationConstraint,"SPH deformation constraint"}
};

struct Params {
    int solverIterations;
    float timeStep;
    
    bool useXPBDDamp;

    PBDType type = PBDType::none;
    ConstitutiveMaterialModel modelType = ConstitutiveMaterialModel::NeoHookeanModel;

    bool useFriction;
    float staticFrictionCoef, kineticFrictionCoef;
};

class Constraint {

public:
    Constraint(Mesh* mesh, int cardinality, bool useMeshCoef, bool showStatus = SHOW_PROCEDURE_INFO) :
        mesh(mesh), cardinality(cardinality), showStatus(showStatus), useMeshCoef(useMeshCoef){}
    void preCompute(Configuration* configuration);
    virtual void project(Configuration* configuration, Params params) {}

    int cardinality;
    vector<int> indices;
    Mesh* mesh;
    Mesh* relatedMesh = nullptr;
    vector<float> inverseMasses;

    ConstraintType type = ConstraintType::NoneType;

    bool showStatus;

protected:
    // Put in needed parameters by coeffs
    // e.g.
    // In normal PBD condition, put in {k}
    // In XPBD condition, put in {compliance, damp factor}
    void commonOnProject(Configuration* configuration, Params params, float C, vector<Vector3f>& partialDerivatives, vector<float> &coeffs = vector<float>());
    void commonOnProjectNormal(Configuration* configuration, int iteration, float C, vector<Vector3f>& partialDerivatives, float factor = 1.f);
    void commonOnProjectExtended(Configuration* configuration, float timeStep, float C, vector<Vector3f>& partialDerivatives, float compliance = 1e-7, float dampCompliance = 0.f);

    bool useMeshCoef;
};

class FixedConstraint : public Constraint {

public:
    FixedConstraint(Mesh* mesh, int cardinality, Vector3f target, bool useMeshCoef = false) :
        Constraint(mesh, cardinality, useMeshCoef), target(target) {
        type = ConstraintType::FixedConstraint;
    }
    void project(Configuration* configuration, Params params);

    Vector3f target;

};

class DistanceConstraint : public Constraint {

public:
    DistanceConstraint(Mesh* mesh, int cardinality, float distance, bool useMeshCoef = false) :
        Constraint(mesh, cardinality, useMeshCoef), distance(distance) {
        type = ConstraintType::DistanceConstraint;
    }
    void project(Configuration* configuration, Params params);

    float distance;
};

class BendConstraint : public Constraint {

public:
    BendConstraint(Mesh* mesh, int cardinality, float angle, bool useMeshCoef = false) :
        Constraint(mesh, cardinality, useMeshCoef), angle(angle) {
        type = ConstraintType::BendConstraint;
    }
    void project(Configuration* configuration, Params params);

    float angle;

};

class CollisionConstraint : public Constraint {

public:
    CollisionConstraint(Mesh* mesh, int cardinality, Vector3f normal, bool useMeshCoef) :
            Constraint(mesh, cardinality, useMeshCoef), normal(normal) {}
    virtual void project(Configuration* configuration, Params params) {}

    Vector3f normal;

    // return pairs of collision vertices, make the index on the second position less than 0 if that should be unmovable
    virtual vector<pair<int, int>> CollisionVertexIndices() = 0;
    bool isCollisionHappened = false;

    void commonFrictionProjecting(Configuration* configuration, float penetrationDepth, float staticFrictionCoef, float kineticFrictionCoef);
};

class TetrahedralConstraint : public Constraint {

public:
    TetrahedralConstraint(Mesh* mesh, int cardinality, Matrix3f originalShape, bool useMeshCoef = false) :
        Constraint(mesh, cardinality, useMeshCoef), inversedOriginalShape(originalShape.inverse()), initialVolume(fabs(originalShape.determinant()) / 6.f) {
        type = ConstraintType::TetrahedralConstraint;
    }
    void project(Configuration* configuration, Params params);

    Matrix3f inversedOriginalShape;
    float initialVolume;
};

class StaticCollisionConstraint : public CollisionConstraint {

public:
    StaticCollisionConstraint(Mesh* mesh, int cardinality, Vector3f normal, Vector3f position, bool useMeshCoef = false) :
        CollisionConstraint(mesh, cardinality, normal, useMeshCoef), position(position) {
        type = ConstraintType::StaticCollisionConstraint;
    }
    void project(Configuration* configuration, Params params);

    Vector3f position;

    vector<pair<int, int>> CollisionVertexIndices() { return { pair<int,int>(indices[0],-1) }; }
};

class TriangleCollisionConstraint : public CollisionConstraint {

public:
    TriangleCollisionConstraint(Mesh* mesh, int cardinality, Vector3f normal, float height, bool useMeshCoef = false) :
        CollisionConstraint(mesh, cardinality, normal, useMeshCoef), height(height) {
        type = ConstraintType::TriangleCollisionConstraint;
    }
    void project(Configuration* configuration, Params params);

    float height;

    vector<pair<int, int>> CollisionVertexIndices() { return { pair<int,int>(indices[0],-1),pair<int,int>(indices[0],-1),pair<int,int>(indices[0],-1) }; }
};

class PointCollisionConstraint : public CollisionConstraint {

public:
    PointCollisionConstraint(Mesh* mesh, int cardinality, float distance, bool useMeshCoef = false)
        : CollisionConstraint(mesh, cardinality, Vector3f::Zero(), useMeshCoef), distance(distance){
        type = ConstraintType::PointCollisionConstraint;
    }
    void project(Configuration* configuration, Params params);

    float distance;

    vector<pair<int, int>> CollisionVertexIndices() { return { pair<int,int>(indices[0],indices[1]) }; }
};

class SPHDeformationConstraint : public Constraint
{

public:
    SPHDeformationConstraint(Mesh* mesh, int cardinality, bool useMeshCoef = false)
        : Constraint(mesh, cardinality, useMeshCoef){
        type = ConstraintType::SPHDeformationConstraint;
    }

    void project(Configuration* configuration, Params params);


};

// calculate stress tensor and stress energy density according to deformation gradient F
pair<Matrix3f, float> calculateStressTensorAndStressEnergyDensity(Matrix3f F, ConstitutiveMaterialModel modelType, float PoisonRatio, float YoungModulus);

// Constraint building
void buildEdgeConstraints(Configuration* configuration, TriangularMesh* mesh);
void buildRigidBodyConstraints(Configuration* configuration, Mesh* mesh);
void buildBendConstraints(Configuration* configuration, TriangularMesh* mesh);
void buildTwoWayCouplingConstraints(Configuration* configuration, Mesh* meshA);
void buildFixedConstraint(Configuration* configuration, Mesh* mesh, int index, Vector3f target);
void buildDistanceConstraint(Configuration* configuration, Mesh* mesh, int indexA, int indexB, float distance, bool useMeshCoef = false, Mesh* secondMesh = nullptr);
void buildBendConstraint(Configuration* configuration, Mesh* mesh, int indexA, int indexB, int indexC, int indexD, float angle);
void buildTetrahedralConstraints(Configuration* configuration, TetrahedralMesh* mesh);
void buildSPHDeformationConstraints(Configuration* configuration, SPHMesh* mesh);
void buildSPHCollisionConstraints(Configuration* configuration, SPHMesh* meshA, SPHMesh* meshB);
void buildSPHSelfCollisionConstraints(Configuration* configuration, SPHMesh* mesh);
TetrahedralConstraint* buildTetrahedralConstraint(Mesh* mesh, int indexA, int indexB, int indexC, int indexD, Matrix3f originalShape, bool useMeshCoef = false);
CollisionConstraint* buildStaticCollisionConstraint(Mesh* mesh, int index, Vector3f normal, Vector3f position);
CollisionConstraint* buildTriangleCollisionConstraint(Mesh* mesh, int vertexIndex, Vector3f normal, float height, int indexA, int indexB, int indexC, Mesh* secondMesh);

#endif //POSITIONBASEDDYNAMICS_CONSTRAINT_HPP
