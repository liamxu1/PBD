//
// Created by Jack Purvis
//

#ifndef POSITIONBASEDDYNAMICS_MESH_HPP
#define POSITIONBASEDDYNAMICS_MESH_HPP

#include <map>
#include <set>
#include <string>
#include <vector>
#include <Eigen>
#include <GL/glew.h>
#include <camera.hpp>

const int XMOVE = 4, YMOVE = 2, ZMOVE = 1;

using namespace std;
using namespace Eigen;

const float KERNEL_FUNCTION_THRESHOLD = 0.1f;
const bool SHOW_PROCEDURE_INFO = false;
const bool SHOW_MESH_INFO = false;

struct SimpleVertex {
    int p;
};

struct Vertex {
    int p;
    int t;
    int n;
};

struct Edge {
    Edge(Vertex vertexA, Vertex vertexB) {
        v[0] = vertexA;
        v[1] = vertexB;
    }
    Vertex v[2];
};

struct EdgeCompare {
    std::less<std::pair<int, int>> lessComparator;

    // Determine if two edges are equal
    bool operator() (const Edge &l, const Edge &r) const {
        return lessComparator(minmax(l.v[0].p, l.v[1].p), minmax(r.v[0].p, r.v[1].p));
    }
};

struct Triangle {
    Vertex v[3];
};

struct SimpleTriangle {
    SimpleTriangle(){}
    SimpleTriangle(int p1, int p2, int p3)
    {
        v[0].p = p1;
        v[1].p = p2;
        v[2].p = p3;
    }
    SimpleVertex v[3];
};

struct SimpleTetrahedron{
    SimpleTetrahedron(){}
    SimpleTetrahedron(vector<int> p)
    {
        if (p.size() == 4)
        {
            for (int i = 0; i < 4; i++)
                v[i].p = p[i];
        }
    }
    SimpleVertex v[4];
};

enum class MeshType
{
    triangular,
    tetrahedral,
    singlePoint,
    SPH
};

enum class MeshBaseType
{
    point,
    line
};

class Mesh {

public:
    Mesh(const char* name, MeshType type, MeshBaseType baseType) :meshName(name), meshType(type),meshBaseType(baseType) {}
    virtual ~Mesh() = 0 {}

    void reset();
    void applyImpulse(Vector3f force);
    void translate(Vector3f translate);
    void dampVelocity(float kDamp = 0.1f, int type = 1);
    virtual void updateCoefs() {}

    string meshName;
    MeshType meshType;
    MeshBaseType meshBaseType;
    bool isLineBased() { return meshBaseType == MeshBaseType::line; }
    bool isPointBased() { return meshBaseType == MeshBaseType::point; }

    virtual void render(Camera* camera, Matrix4f transform){}

    int numVertices = -1;
    vector<Vector3f> initialVertices;
    vector<Vector3f> vertices;
    vector<Vector3f> velocities;
    vector<float> inverseMass;
    int estimatePositionsOffset = -1;

    // Simulation fields
    bool isRigidBody = false;
    bool gravityAffected = false;
    bool windAffected = false;

    bool needCoef = false;

    bool selfCollisionTest = false;
    bool dynamicCollisionTest = true;

    Vector3f position = Vector3f(0.0f, 0.0f, 0.0f);
    vector<float> backupCoefData;

protected:

    // VBOs
    GLuint positionVBO = 0;
    GLuint normalVBO = 0;

    // Rendering
    GLuint shader = 0;
    Vector3f colour;

};

class LineBasedMesh : public Mesh{

public:
    LineBasedMesh(const char* name, MeshType type) :Mesh(name, type, MeshBaseType::line) {}
    virtual ~LineBasedMesh() = 0 {}
    virtual void updateCoefs(){}
    
    bool intersect(Vector3f rayOrigin, Vector3f rayDirection, float &t, Vector3f &normal, int vertexIndex, int &triangleIndex);
    virtual void render(Camera* camera, Matrix4f transform);

    int numFaces = -1;

    // Mesh fields
    vector<SimpleTriangle> triangles;
    std::vector<Vector3f> surfaceNormals;


protected:
    bool rayTriangleIntersect(Vector3f rayOrigin, Vector3f rayDirection, float &t, int triangleIndex, int vertexIndex);
    void generateSurfaceNormals();

};

class TriangularMesh : public LineBasedMesh
{
public:
    TriangularMesh(const char* name, string filename, Vector3f colour, float inverseMass = 1.0f);
    ~TriangularMesh(){}

    void updateCoefs();

    vector<Vector2f> uvs;
    vector<Vector3f> normals;
    set<Edge, EdgeCompare> edges;
    map<Edge, vector<SimpleTriangle>, EdgeCompare> adjacentTriangles;

    float stretchFactor = 0.999f;
    float bendFactor = 0.3f;

    // for xpbd
    float stretchCompliance = 1e-7;
    float bendCompliance = 1e-5;

    float dampCompliance = 0;

private:
    void parseObjFile(string filename);
};

class TetrahedralMesh : public LineBasedMesh
{
public:
    TetrahedralMesh(const char* name, string filename, Vector3f colour, float inverseMass = 1.0f);
    ~TetrahedralMesh(){}

    void updateCoefs();
    vector<SimpleTetrahedron> tetrahedrons;
    int numBodies;

    float poisonRatio = 0.3f;
    float YoungModulus = 20;

private:

    // my .tet file format:
    // First line: (total vertices number) (total tetrahedrons number)
    // Next part: (vertices' positions)
    // ----each line with 3 floats representing the position of one vertex
    // Final part: (tetrahedrons)
    // ----each line with 4 ints representing the vertex indices (start with 0) in a tetrahedron
    void parseTetFile(string filename);

    void insertTriangle(SimpleTriangle& triangle);
};

class PointBasedMesh : public Mesh
{
public:
    PointBasedMesh(const char* name, MeshType type) : Mesh(name, type, MeshBaseType::point) {}
    virtual ~PointBasedMesh() = 0 {}

    virtual void render(Camera* camera, Matrix4f transform);

};

// For fixed point controlling
class SinglePointMesh : public PointBasedMesh
{
public:
    SinglePointMesh(const char* name, Vector3f position, Vector3f colour, size_t pos = 0);
    ~SinglePointMesh() {}

    // indicate whether the outward direction is the same with positive direction in three axes
    bool x = false, y = false, z = false;
};

class SPHMesh : public PointBasedMesh
{
public:
    SPHMesh(const char* name, string filename, Vector3f colour, float inverseMass = 1.0f);
    ~SPHMesh();

    void updateCoefs();

    struct Node
    {
        Node(int index, float kernel, Vector3f gradientKernel, Vector3f correctedKernel = Vector3f::Zero(), Vector3f correctedGradient = Vector3f::Zero())
            :index(index), kernel(kernel), gradientKernel(gradientKernel), correctedKernel(correctedKernel), correctedGradient(correctedGradient), next(nullptr){}
        int index;
        float kernel;
        Vector3f gradientKernel;
        Vector3f correctedKernel;
        Vector3f correctedGradient;
        Node* next;
    };

    vector<float> volumes;
    vector<Node*> kernelInfos;

    int numFirstCollidingParticles = -1;
    int numSecondCollidingParticles = -1;
    vector<pair<unsigned, float>> firstCollidingInfos;
    vector<pair<unsigned, float>> secondCollidingInfos;

    float poisonRatio = 0.3f;
    float YoungModulus = 20;

private:
    // my .sph file format:
    // First line: (total vertices number) (first border vertices number)(second border vertices number)
    // Next part: (vertices' positions)
    // ----each line with 3 floats representing the position of one vertex
    // Final part: (borders)
    // ----each line with a int and a float representing the index and collision radius
    void parseSphFile(string filename);

    bool createKernelInfo();
    void adjustKernelThreshold();

    float h = KERNEL_FUNCTION_THRESHOLD;
    static float kernelFunction(Vector3f r, float h);
    static Vector3f gradientKernelFunction(Vector3f r, float h);

};
#endif //POSITIONBASEDDYNAMICS_MESH_HPP