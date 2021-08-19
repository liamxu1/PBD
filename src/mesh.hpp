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

using namespace std;
using namespace Eigen;

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
    singlePoint
};

class Mesh {

public:
    Mesh(const char* name, MeshType type) :meshName(name), meshType(type) {}
    virtual ~Mesh() = 0;
    virtual void updateCoefs(){}
    void reset();
    void applyImpulse(Vector3f force);
    void translate(Vector3f translate);
    bool intersect(Vector3f rayOrigin, Vector3f rayDirection, float &t, Vector3f &normal, int vertexIndex, int &triangleIndex);
    void render(Camera* camera, Matrix4f transform);

    void dampVelocity(float kDamp = 0.1f, int type = 1);

    int numVertices = -1;
    int numFaces = -1;

    Vector3f position = Vector3f(0.0f, 0.0f, 0.0f);

    // Mesh fields
    vector<Vector3f> initialVertices;
    vector<Vector3f> vertices;
    vector<SimpleTriangle> triangles;
    std::vector<Vector3f> surfaceNormals;

    // Simulation fields
    vector<Vector3f> velocities;
    vector<float> inverseMass;
    int estimatePositionsOffset = -1;
    bool isRigidBody = false;
    bool gravityAffected = false;
    bool windAffected = false;

    string meshName;
    MeshType meshType;
    bool needCoef = false;

    vector<float> backupCoefData;

protected:
    bool rayTriangleIntersect(Vector3f rayOrigin, Vector3f rayDirection, float &t, int triangleIndex, int vertexIndex);
    void generateSurfaceNormals();

    // VBOs
    GLuint positionVBO = 0;
    GLuint normalVBO = 0;

    // Rendering
    GLuint shader = 0;
    Vector3f colour;

};

class TriangularMesh : public Mesh
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

class TetrahedralMesh : public Mesh
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

// For fixed point controlling
class SinglePointMesh : public Mesh
{
public:
    SinglePointMesh(const char* name, Vector3f position);
    ~SinglePointMesh() {}
};
#endif //POSITIONBASEDDYNAMICS_MESH_HPP