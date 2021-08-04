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

struct Tetrahedron
{
    Vertex v[4];
};

class Mesh {

public:
    Mesh(){}
    virtual ~Mesh() = 0;
    void reset();
    void applyImpulse(Vector3f force);
    void translate(Vector3f translate);
    bool intersect(Vector3f rayOrigin, Vector3f rayDirection, float &t, Vector3f &normal, int vertexIndex, int &triangleIndex);
    void render(Camera* camera, Matrix4f transform);

    void dampVelocity(float kDamp = 0.1f, int type = 1);

    int numVertices;
    int numFaces;

    Vector3f position = Vector3f(0.0f, 0.0f, 0.0f);

    // Mesh fields
    vector<Vector3f> initialVertices;
    vector<Vector3f> vertices;
    vector<Triangle> triangles;
    std::vector<Vector3f> surfaceNormals;

    // Simulation fields
    vector<Vector3f> velocities;
    vector<float> inverseMass;
    int estimatePositionsOffset;
    bool isRigidBody = false;
    bool gravityAffected = false;
    bool windAffected = false;

protected:
    bool rayTriangleIntersect(Vector3f rayOrigin, Vector3f rayDirection, float &t, int triangleIndex, int vertexIndex);
    void generateSurfaceNormals();

    // VBOs
    GLuint positionVBO;
    GLuint normalVBO;

    // Rendering
    GLuint shader;
    Vector3f colour;

};

class TriangularMesh : public Mesh
{
public:
    TriangularMesh(string filename, Vector3f colour, float inverseMass = 1.0f);
    ~TriangularMesh(){}

    vector<Vector2f> uvs;
    vector<Vector3f> normals;
    set<Edge, EdgeCompare> edges;
    map<Edge, vector<Triangle>, EdgeCompare> adjacentTriangles;

private:
    void parseObjFile(string filename);
};

#endif //POSITIONBASEDDYNAMICS_MESH_HPP