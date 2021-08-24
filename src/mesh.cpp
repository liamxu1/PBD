//
// Created by Jack Purvis
//

#include <iostream>
#include <fstream>
#include <sstream>
#include <shaderLoader.hpp>
#include <mesh.hpp>
#include <main.hpp>

const float h = KERNEL_FUNCTION_THRESHOLD;

Matrix3f CrossMat(Vector3f vec)
{
    Matrix3f mat;
    mat(0, 0) = 0;
    mat(0, 1) = -vec[2];
    mat(0, 2) = vec[1];
    mat(1, 0) = vec[2];
    mat(1, 1) = 0;
    mat(1, 2) = -vec[0];
    mat(2, 0) = -vec[1];
    mat(2, 1) = vec[0];
    mat(2, 2) = 0;
    return mat;
}

TriangularMesh::TriangularMesh(const char* name, string filename, Vector3f colour, float inverseMass)
    :LineBasedMesh(name, MeshType::triangular)
{

    this->colour = colour;
    parseObjFile(filename);

    initialVertices = vertices;

    // Setup VBO
    glGenBuffers(1, &positionVBO);
    glGenBuffers(1, &normalVBO);

    // Setup shader
    shader = loadShaders("SimpleVertexShader", "SimpleFragmentShader");

    // Setup simulation
    reset();
    this->inverseMass.resize((size_t)numVertices, inverseMass);

    backupCoefData = { stretchFactor,bendFactor,stretchCompliance * (float)1e5,bendCompliance * (float)1e5,dampCompliance };
}

void TriangularMesh::updateCoefs()
{
    stretchFactor = backupCoefData[0];
    bendFactor = backupCoefData[1];
    stretchCompliance = backupCoefData[2] * float(1e-5);
    bendCompliance = backupCoefData[3] * float(1e-5);
    dampCompliance = backupCoefData[4];
}

void LineBasedMesh::generateSurfaceNormals() {
    surfaceNormals.clear();
    for (int i = 0; i < numFaces; i++) {
        Vector3f v1 = vertices[triangles[i].v[1].p] - vertices[triangles[i].v[0].p];
        Vector3f v2 = vertices[triangles[i].v[2].p] - vertices[triangles[i].v[0].p];
        Vector3f surfaceNormal = v1.cross(v2);
        surfaceNormal.normalize();
        surfaceNormals.push_back(surfaceNormal);
    }
}

void Mesh::reset() {
    vertices = initialVertices;

    velocities.clear();
    velocities.resize((size_t) numVertices, Vector3f::Zero());
}

void Mesh::applyImpulse(Vector3f force) {
    for (int i = 0; i < numVertices; i++) {
        velocities[i] += force;
    }
}

void Mesh::translate(Vector3f translate) {
    for (int i = 0; i < numVertices; i++) {
        vertices[i] += translate;
    }
}

bool LineBasedMesh::intersect(Vector3f rayOrigin, Vector3f rayDirection, float &t, Vector3f &normal, int vertexIndex, int &triHitIndex) {

    // Ensure the ray intersects the bounding box before testing each triangle

    bool hit = false;
    t = INFINITY;
    int closestIndex = 0;

    // Check each triangle to try and find an intersection
    for (int triangleIndex = 0; triangleIndex < numFaces; triangleIndex++) {
        float tTri = INFINITY;
        if (rayTriangleIntersect(rayOrigin, rayDirection, tTri, triangleIndex, vertexIndex) && fabs(tTri) < fabs(t)) {
            hit = true;
            t = tTri;
            closestIndex = triangleIndex;
        }
    }

    if (hit) {
        triHitIndex = closestIndex;

        // Compute the normal at the intersected triangle
        Vector3f v1 = vertices[triangles[closestIndex].v[1].p] - vertices[triangles[closestIndex].v[0].p];
        Vector3f v2 = vertices[triangles[closestIndex].v[2].p] - vertices[triangles[closestIndex].v[0].p];
        normal = v1.cross(v2);
        normal.normalize();
    }

    return hit;
}

bool LineBasedMesh::rayTriangleIntersect(Vector3f rayOrigin, Vector3f rayDirection, float &t, int triangleIndex, int vertexIndex) {
    int i0 = triangles[triangleIndex].v[0].p;
    int i1 = triangles[triangleIndex].v[1].p;
    int i2 = triangles[triangleIndex].v[2].p;

    if (i0 + estimatePositionsOffset == vertexIndex || i1 + estimatePositionsOffset == vertexIndex || i2 + estimatePositionsOffset == vertexIndex) return false;

    // Get the triangle properties
    Vector3f v0 = vertices[i0];
    Vector3f v1 = vertices[i1];
    Vector3f v2 = vertices[i2];

    Vector3f v0v1 = v1 - v0;
    Vector3f v0v2 = v2 - v0;
    Vector3f pvec = rayDirection.cross(v0v2);
    float det = v0v1.dot(pvec);

    // Check if ray and triangle are parallel
    if (fabs(det) < 0.0001f) return false;

    float invDet = 1.0f / det;

    Vector3f tvec = rayOrigin - v0;
    float u = tvec.dot(pvec) * invDet;
    if (u < 0 || u > 1) return false;

    Vector3f qvec = tvec.cross(v0v1);
    float v = rayDirection.dot(qvec) * invDet;
    if (v < 0 || u + v > 1) return false;

    t = v0v2.dot(qvec) * invDet;

    return true;
}

void LineBasedMesh::render(Camera* camera, Matrix4f transform) {

    // Setup transform
    Affine3f t(Translation3f(position[0], position[1], position[2]));
    Matrix4f modelMatrix = transform * t.matrix();

    // Compute vertex normals
    vector<Vector3f> tempNormals;
    tempNormals.resize((size_t) numVertices, Vector3f::Zero());
    generateSurfaceNormals();
    for (unsigned int i = 0; i < numFaces; i++) {
        SimpleTriangle tri = triangles[i];

        for (int j = 0; j < 3; j++) {
            tempNormals[tri.v[j].p] += surfaceNormals[i];
        }
    }

    // Build vertex positions and normals
    vector<Vector3f> outVertices;
    vector<Vector3f> outNormals;
    for (unsigned int i = 0; i < numFaces; i++) {
        SimpleTriangle tri = triangles[i];

        for (int j = 0; j < 3; j++) {
            Vector3f position = vertices[tri.v[j].p];
            Vector3f normal = tempNormals[tri.v[j].p];

            outVertices.push_back(position);
            normal.normalize();
            outNormals.push_back(normal);
        }
    }

    glBindBuffer(GL_ARRAY_BUFFER, positionVBO);
    glBufferData(GL_ARRAY_BUFFER, outVertices.size() * sizeof(Vector3f), outVertices[0].data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, normalVBO);
    glBufferData(GL_ARRAY_BUFFER, outNormals.size() * sizeof(Vector3f), outNormals[0].data(), GL_STATIC_DRAW);

    glUseProgram(shader);

    glUniform3fv(glGetUniformLocation(shader, "materialColour"), 1, colour.data());
    Vector4f lightPosition = Vector4f(8, 10, 0, 0);
    lightPosition = modelMatrix * lightPosition;
    glUniform3fv(glGetUniformLocation(shader, "lightPosition"), 1, lightPosition.data());

    // Bind matrices
    glUniformMatrix4fv(glGetUniformLocation(3, "projection"), 1, GL_FALSE, camera->projectionMatrix.data());
    glUniformMatrix4fv(glGetUniformLocation(3, "view"), 1, GL_FALSE, camera->viewMatrix.data());
    glUniformMatrix4fv(glGetUniformLocation(3, "model"), 1, GL_FALSE, modelMatrix.data());

    // Bind vertex positions
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, positionVBO);
    glVertexAttribPointer(
            0,         // shader layout attribute
            3,         // size
            GL_FLOAT,  // type
            GL_FALSE,  // normalized?
            0,         // stride
            (void*)0   // array buffer offset
    );

    // Bind vertex normals
    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, normalVBO);
    glVertexAttribPointer(
            1,         // shader layout attribute
            3,         // size
            GL_FLOAT,  // type
            GL_FALSE,  // normalized?
            0,         // stride
            (void*)0   // array buffer offset
    );

    glDrawArrays(GL_TRIANGLES, 0, outVertices.size());
    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
}

void Mesh::dampVelocity(float kDamp, int type)
{
    switch (type)
    {
    case 0:
        break;
    case 1:
    {
        #pragma omp parallel for
        for (int i = 0; i < numVertices; i++) {
            velocities[i] *= 1 - kDamp;
        }
        break;
    }
    
    case 2:
    {
        size_t n = numVertices;
        Vector3f pos = Vector3f::Zero();
        Vector3f vel = Vector3f::Zero();
        float total_mass = 0;
        for (size_t i = 0; i < n; i++)
        {
            if (inverseMass[i] < EPSILONTHRESHOLD)
                continue;
            pos += vertices[i] / inverseMass[i];
            vel += velocities[i] / inverseMass[i];
            total_mass += 1.f / inverseMass[i];
        }
        Matrix3f I = Matrix3f::Zero();
        Vector3f L = Vector3f::Zero();
        vel = vel / total_mass;
        pos = pos / total_mass;
        for (size_t i = 0; i < n; i++)
        {
            if (inverseMass[i] < EPSILONTHRESHOLD)
                continue;
            Vector3f posi = vertices[i] - pos;
            Matrix3f posi_mat = CrossMat(posi);
            L += posi_mat * velocities[i] / inverseMass[i];
            I += posi_mat * posi_mat.transpose() / inverseMass[i];
        }
        Vector3f angular_velocity = I.inverse() * L;
        for (size_t i = 0; i < n; i++)
        {
            if (inverseMass[i] < EPSILONTHRESHOLD)
                continue;
            Vector3f dv = vel + angular_velocity.cross(vertices[i] - pos) - velocities[i];
            velocities[i] = velocities[i] + kDamp * dv;
        }

        break;
    }

    default:
        break;
    }
}

void TriangularMesh::parseObjFile(string filename) {

    // Attempt to open an input stream to the file
    ifstream objFile(filename);
    if (!objFile.is_open()) {
        cout << "Error reading " << filename << endl;
        return;
    }

    // While the file has lines remaining
    while (objFile.good()) {

        // Pull out line from file
        string line;
        getline(objFile, line);
        istringstream objLine(line);

        // Pull out mode from line
        string mode;
        objLine >> mode;

        // Reading like this means whitespace at the start of the line is fine
        // attempting to read from an empty string/line will set the failbit
        if (!objLine.fail()) {

            if (mode == "v") { // Vertex
                Vector3f v;
                objLine >> v[0] >> v[1] >> v[2];
                vertices.push_back(v);
            } else if (mode == "vn") { // Vertex normal
                Vector3f vn;
                objLine >> vn[0] >> vn[1] >> vn[2];
                normals.push_back(vn);
            } else if (mode == "vt") { // UV
                Vector2f vt;
                objLine >> vt[0] >> vt[1];
                uvs.push_back(vt);
            } else if (mode == "f") { // Face
                vector<Vertex> verts;
                while (objLine.good()) {
                    Vertex v;

                    // OBJ face is formatted as v/vt/vn
                    objLine >> v.p; // Scan in position index
                    objLine.ignore(1); // Ignore the '/' character
                    objLine >> v.t; // Scan in uv (texture coord) index
                    objLine.ignore(1); // Ignore the '/' character
                    objLine >> v.n; // Scan in normal index

                    // Correct the indices
                    v.p -= 1;
                    v.n -= 1;
                    v.t -= 1;

                    verts.push_back(v);
                }

                // If we have 3 vertices, construct a triangle
                if (verts.size() >= 3) {
                    SimpleTriangle triangle(verts[0].p, verts[1].p, verts[2].p);
                    triangles.push_back(triangle);

                    // Construct edges
                    Edge e1 = Edge(verts[0], verts[1]);
                    Edge e2 = Edge(verts[0], verts[2]);
                    Edge e3 = Edge(verts[1], verts[2]);
                    edges.insert(e1);
                    edges.insert(e2);
                    edges.insert(e3);

                    // Add to adjacent triangles
                    adjacentTriangles[e1].push_back(triangle);
                    adjacentTriangles[e2].push_back(triangle);
                    adjacentTriangles[e3].push_back(triangle);
                }
            }
        }
    }

    this->numVertices = (int) vertices.size();
    this->numFaces = (int) triangles.size();

    generateSurfaceNormals();
}

TetrahedralMesh::TetrahedralMesh(const char* name, string filename, Vector3f colour, float inverseMass)
    : LineBasedMesh(name, MeshType::tetrahedral)
{
    this->colour = colour;
    parseTetFile(filename);

    initialVertices = vertices;

    // Setup VBO
    glGenBuffers(1, &positionVBO);
    glGenBuffers(1, &normalVBO);

    // Setup shader
    shader = loadShaders("SimpleVertexShader", "SimpleFragmentShader");

    // Setup simulation
    reset();
    this->inverseMass.resize((size_t)numVertices, inverseMass);

    backupCoefData = { poisonRatio,YoungModulus };
}

void TetrahedralMesh::updateCoefs()
{
    poisonRatio = backupCoefData[0];
    YoungModulus = backupCoefData[1];
}

void TetrahedralMesh::parseTetFile(string filename)
{
    // Attempt to open an input stream to the file
    ifstream tetFile(filename);
    if (!tetFile.is_open()) {
        cout << "Error reading " << filename << endl;
        return;
    }

    string line;
    getline(tetFile, line);
    istringstream tetline(line);
    tetline >> numVertices >> numBodies;

    for (int i = 0; i < numVertices; i++)
    {
        Vector3f v;
        getline(tetFile, line);
        istringstream tetline(line);
        tetline >> v[0] >> v[1] >> v[2];
        vertices.push_back(v);
    }
    for (int i = 0; i < numBodies; i++)
    {
        getline(tetFile, line);
        istringstream tetline(line);
        
        vector<int> p(4);
        tetline >> p[0] >> p[1] >> p[2] >> p[3];

        tetrahedrons.push_back(SimpleTetrahedron(p));

        insertTriangle(SimpleTriangle(p[0], p[1], p[2]));
        insertTriangle(SimpleTriangle(p[1], p[0], p[3]));
        insertTriangle(SimpleTriangle(p[0], p[2], p[3]));
        insertTriangle(SimpleTriangle(p[2], p[1], p[3]));
    }

    this->numFaces = (int)triangles.size();

    generateSurfaceNormals();
}

void TetrahedralMesh::insertTriangle(SimpleTriangle& triangle)
{
    for (auto iter = triangles.begin(); iter != triangles.end(); iter++)
    {
        if (iter->v[0].p == triangle.v[0].p)
        {
            if ((iter->v[1].p == triangle.v[1].p && iter->v[2].p == triangle.v[2].p) || (iter->v[1].p == triangle.v[2].p && iter->v[2].p == triangle.v[1].p))
            {
                triangles.erase(iter);
                return;
            }
        }
        else if (iter->v[0].p == triangle.v[1].p)
        {
            if ((iter->v[1].p == triangle.v[0].p && iter->v[2].p == triangle.v[2].p) || (iter->v[1].p == triangle.v[2].p && iter->v[2].p == triangle.v[0].p))
            {
                triangles.erase(iter);
                return;
            }
        }
        else if (iter->v[0].p == triangle.v[2].p)
        {
            if ((iter->v[1].p == triangle.v[0].p && iter->v[2].p == triangle.v[1].p) || (iter->v[1].p == triangle.v[1].p && iter->v[2].p == triangle.v[0].p))
            {
                triangles.erase(iter);
                return;
            }
        }
    }

    triangles.push_back(triangle);
}

SinglePointMesh::SinglePointMesh(const char* name, Vector3f position, Vector3f colour, size_t pos) : PointBasedMesh(name, MeshType::singlePoint)
{
    this->colour = colour;

    this->numVertices = 1;
    this->initialVertices.push_back(position);
    this->inverseMass.push_back(0.f);
    this->reset();

    // Setup VBO
    glGenBuffers(1, &positionVBO);

    // Setup shader
    shader = loadShaders("SimpleVertexShader", "SimpleFragmentShader");

    x = (pos / 4 == 1);
    y = (pos / 2 % 2 == 1);
    z = (pos % 2 == 1);
}

void PointBasedMesh::render(Camera* camera, Matrix4f transform)
{
    // Setup transform
    Affine3f t(Translation3f(position[0], position[1], position[2]));
    Matrix4f modelMatrix = transform * t.matrix();

    glBindBuffer(GL_ARRAY_BUFFER, positionVBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vector3f), vertices[0].data(), GL_STATIC_DRAW);

    glUseProgram(shader);

    glUniform3fv(glGetUniformLocation(shader, "materialColour"), 1, colour.data());
    Vector4f lightPosition = Vector4f(8, 10, 0, 0);
    lightPosition = modelMatrix * lightPosition;
    glUniform3fv(glGetUniformLocation(shader, "lightPosition"), 1, lightPosition.data());

    // Bind matrices
    glUniformMatrix4fv(glGetUniformLocation(3, "projection"), 1, GL_FALSE, camera->projectionMatrix.data());
    glUniformMatrix4fv(glGetUniformLocation(3, "view"), 1, GL_FALSE, camera->viewMatrix.data());
    glUniformMatrix4fv(glGetUniformLocation(3, "model"), 1, GL_FALSE, modelMatrix.data());

    // Bind vertex positions
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, positionVBO);
    glVertexAttribPointer(
            0,         // shader layout attribute
            3,         // size
            GL_FLOAT,  // type
            GL_FALSE,  // normalized?
            0,         // stride
            (void*)0   // array buffer offset
    );

    glDrawArrays(GL_POINTS, 0, vertices.size());
    glDisableVertexAttribArray(0);
}

SPHMesh::SPHMesh(const char* name, string filename, Vector3f colour, float inverseMass)
    : PointBasedMesh(name, MeshType::SPH)
{
    this->colour = colour;

    parseSphFile(filename);
    initialVertices = vertices;

    // Setup VBO
    glGenBuffers(1, &positionVBO);

    // Setup shader
    shader = loadShaders("SimpleVertexShader", "SimpleFragmentShader");

    // Setup simulation
    reset();
    this->inverseMass.resize((size_t)numVertices, inverseMass);

    float volume = 4.f / 3.f * PI * powf(h, 3);
    volumes.resize(numVertices, volume);
    kernelInfos.resize(numVertices, nullptr);

    createKernelInfo();

    backupCoefData = { poisonRatio,YoungModulus };
}

SPHMesh::~SPHMesh()
{
    for (auto info : kernelInfos)
    {
        auto nextInfo = info->next;
        delete info;
        while (nextInfo)
        {
            info = nextInfo;
            nextInfo = info->next;
            delete info;
        }
    }
}

void SPHMesh::updateCoefs()
{
    poisonRatio = backupCoefData[0];
    YoungModulus = backupCoefData[1];
}

void SPHMesh::parseSphFile(string filename)
{
    // Attempt to open an input stream to the file
    ifstream sphFile(filename);
    if (!sphFile.is_open()) {
        cout << "Error reading " << filename << endl;
        return;
    }

    string line;
    getline(sphFile, line);
    istringstream tetline(line);
    tetline >> numVertices >> numFirstCollidingParticles >> numSecondCollidingParticles;

    for (int i = 0; i < numVertices; i++)
    {
        Vector3f v;
        getline(sphFile, line);
        istringstream sphline(line);
        sphline >> v[0] >> v[1] >> v[2];
        vertices.push_back(v);
    }
    for (int i = 0; i < numFirstCollidingParticles; i++)
    {
        pair<unsigned, float> temp;
        getline(sphFile, line);
        istringstream sphline(line);
        sphline >> temp.first >> temp.second;
        firstCollidingInfos.push_back(temp);
    }
    for (int i = 0; i < numSecondCollidingParticles; i++)
    {
        pair<unsigned, float> temp;
        getline(sphFile, line);
        istringstream sphline(line);
        sphline >> temp.first >> temp.second;
        secondCollidingInfos.push_back(temp);
    }
}

void SPHMesh::createKernelInfo()
{
    for (int i = 0; i < numVertices; i++)
    {
        for (int j = 0; j < numVertices; j++)
        {
            if (j == i) continue;

            Vector3f r = vertices[i] - vertices[j];
            float kernel = kernelFunction(r);
            if (kernel == 0) continue;

            Vector3f gradient = gradientKernelFunction(r);
            Node* node = new Node(j, kernel, gradient);
            node->next = kernelInfos[i];
            kernelInfos[i] = node;
        }

        float kernel = kernelFunction(Vector3f::Zero());
        Node* node = new Node(i, kernel, Vector3f::Zero());
        node->next = kernelInfos[i];
        kernelInfos[i] = node;
    }

    for (int i = 0; i < numVertices; i++)
    {
        Vector3f gradientSum = Vector3f::Zero();
        float kernelSum = 0;

        Node* current = kernelInfos[i];
        while (current != nullptr)
        {
            gradientSum += volumes[current->index] * current->gradientKernel;
            kernelSum += volumes[current->index] * current->kernel;
            current = current->next;
        }

        Vector3f gamma = gradientSum / kernelSum;

        current = kernelInfos[i];
        while (current != nullptr)
        {
            current->correctedKernel = (current->gradientKernel - gamma) / kernelSum;
            current = current->next;
        }
        
        Matrix3f L = Matrix3f::Zero();
        current = kernelInfos[i];
        while (current != nullptr)
        {
            L += volumes[current->index] * current->correctedKernel * (vertices[current->index] - vertices[i]).transpose();
            current = current->next;
        }
        Matrix3f inversedL = L.inverse();

        current = kernelInfos[i];
        while (current != nullptr)
        {
            current->correctedGradient = inversedL * current->correctedKernel;
            current = current->next;
        }

        if (SHOWALLINFO)
        {
            cout << "-------------------------------------\n";
            cout << i << '\n';
            cout << "-------------------------------------\n";
            current = kernelInfos[i];
            while (current != nullptr)
            {
                cout << current->index << '\n';
                cout << current->kernel << '\n';
                cout << current->gradientKernel[0] << '\t' << current->gradientKernel[1] << '\t' << current->gradientKernel[2] << '\n';
                cout << current->correctedKernel[0] << '\t' << current->correctedKernel[1] << '\t' << current->correctedKernel[2] << '\n';
                cout << current->correctedGradient[0] << '\t' << current->correctedGradient[1] << '\t' << current->correctedGradient[2] << '\n';
                current = current->next;
                cout << '\n';
            }
        }
    }

}

float SPHMesh::kernelFunction(Vector3f r)
{
    static float coeff = 315.f / (64.f * PI * powf(h, 9));
    float distance = r.norm();
    if (distance > h) return 0.f;

    return  coeff * powf(h * h - distance * distance, 3);
}

Vector3f SPHMesh::gradientKernelFunction(Vector3f r)
{
    static float coeff = -945.f / (32.f * PI * powf(h, 9));
    float distance = r.norm();
    if (distance > h) return Vector3f::Zero();

    return  coeff * powf(h * h - distance * distance, 2) * r;
}
