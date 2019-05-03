#include <boost/function_output_iterator.hpp>
#include <boost/geometry/geometry.hpp>
#include <chrono>
#include <iostream>
#include <random>
#include <vector>
#include <string>
#include <fstream>

using namespace std;

// using cxxopts for CLI argument parsing
#include "cxxopts.hpp"

// using prakhar1989's ProgressBar for CLI progress indicator
#include "ProgressBar.hpp"

// using tinyobjloader for OBJ file parsing
#define TINYOBJLOADER_IMPLEMENTATION
#define EPSILON 0.00000001
#include "tiny_obj_loader.h"

using namespace std;

// full 3D model to inhibit growth
string fullModelFilename = "";
tinyobj::attrib_t fullModelAttrib;
vector<tinyobj::shape_t> fullModelShapes;
vector<tinyobj::material_t> fullModelMaterials;

// 3D model of selected faces to seed growth
string seedModelFilename = "";
tinyobj::attrib_t seedModelAttrib;
vector<tinyobj::shape_t> seedModelShapes;
vector<tinyobj::material_t> seedModelMaterials;

// number of particles
const int DefaultNumberOfParticles = 1000000;
int numParticles = DefaultNumberOfParticles;

// progress bar
ProgressBar progressBar(numParticles, 40);

// output file
string filename = "points";
ofstream file;

// interval (in iterations) that point data is outputted
int lastOutputIteration = 0;
int interval = numParticles;

// number of dimensions (must be 2 or 3)
const int D = 3;

// default parameters (documented below)
const double DefaultParticleSpacing = 1;
const double DefaultAttractionDistance = 3;
const double DefaultMinMoveDistance = 1;
const int DefaultStubbornness = 0;
const double DefaultStickiness = 1;
const double DefaultBoundingRadius = 0;

// boost is used for its spatial index
using BoostPoint = boost::geometry::model::point<double, D, boost::geometry::cs::cartesian>;
using IndexValue = pair<BoostPoint, int>;
using Index = boost::geometry::index::rtree<IndexValue, boost::geometry::index::linear<4>>;

// Vector represents a point or a vector
class Vector {
public:
    Vector() :
        m_X(0), m_Y(0), m_Z(0) {}

    Vector(double x, double y) :
        m_X(x), m_Y(y), m_Z(0) {}

    Vector(double x, double y, double z) :
        m_X(x), m_Y(y), m_Z(z) {}

    double X() const {
        return m_X;
    }

    double Y() const {
        return m_Y;
    }

    double Z() const {
        return m_Z;
    }

    BoostPoint ToBoost() const {
        return BoostPoint(m_X, m_Y, m_Z);
    }

    double Length() const {
        return sqrt(m_X * m_X + m_Y * m_Y + m_Z * m_Z);
    }

    double LengthSquared() const {
        return m_X * m_X + m_Y * m_Y + m_Z * m_Z;
    }

    double Distance(const Vector &v) const {
        const double dx = m_X - v.m_X;
        const double dy = m_Y - v.m_Y;
        const double dz = m_Z - v.m_Z;
        return sqrt(dx * dx + dy * dy + dz * dz);
    }

    Vector Normalized() const {
        const double m = 1 / Length();
        return Vector(m_X * m, m_Y * m, m_Z * m);
    }

    Vector GetCrossed(const Vector &v) const {
        return Vector(m_Y * v.m_Z - m_Z * v.m_Y, 
                      m_Z * v.m_X - m_X * v.m_Z,
                      m_X * v.m_Y - m_Y * v.m_X);
    }

    double GetDot(const Vector &v) const {
        return m_X * v.m_X +
               m_Y * v.m_Y +
               m_Z * v.m_Z;
    }

    Vector operator+(const Vector &v) const {
        return Vector(m_X + v.m_X, m_Y + v.m_Y, m_Z + v.m_Z);
    }

    Vector operator-(const Vector &v) const {
        return Vector(m_X - v.m_X, m_Y - v.m_Y, m_Z - v.m_Z);
    }

    Vector operator*(const double a) const {
        return Vector(m_X * a, m_Y * a, m_Z * a);
    }

    Vector &operator+=(const Vector &v) {
        m_X += v.m_X; m_Y += v.m_Y; m_Z += v.m_Z;
        return *this;
    }

    bool operator==(const Vector &v) const {
        return m_X == v.m_X && m_Y == v.m_Y && m_Z == v.m_Z;
    }

private:
    double m_X;
    double m_Y;
    double m_Z;
};

// Lerp linearly interpolates from a to b by distance.
Vector Lerp(const Vector &a, const Vector &b, const double d) {
    return a + (b - a).Normalized() * d;
}

// Random returns a uniformly distributed random number between lo and hi
double Random(const double lo = 0, const double hi = 1) {
    static thread_local mt19937 gen(chrono::high_resolution_clock::now().time_since_epoch().count());
    uniform_real_distribution<double> dist(lo, hi);
    return dist(gen);
}

// RandomInUnitSphere returns a random, uniformly distributed point inside the
// unit sphere (radius = 1)
Vector RandomInUnitSphere() {
    while (true) {
        const Vector p = Vector(
            Random(-1, 1),
            Random(-1, 1),
            D == 2 ? 0 : Random(-1, 1));
        if (p.LengthSquared() < 1) {
            return p;
        }
    }
}

// IsIntersectingFace checks for intersection of a ray (O) with random direction (D) and a triangle face defined by vertices V1, V2, and V3
// Adapted from Amnon Owed's ofxPointInMesh::triangleIntersection: https://github.com/AmnonOwed/ofxPointInMesh
// Uses Möller–Trumbore: http://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
bool IsIntersectingFace(const Vector &V1, const Vector &V2, const Vector &V3, const Vector &O, const Vector &D, Vector &R) {
	Vector e1, e2; // Edge1, Edge2
	Vector P, Q, T;
	float det, inv_det, u, v;
	float t;

	// Find vectors for two edges sharing V1
	e1 = V2 - V1;
	e2 = V3 - V1;

	// Begin calculating determinant - also used to calculate u parameter
	P = D.GetCrossed(e2);

	// if determinant is near zero, ray lies in plane of triangle
	det = e1.GetDot(P);

	// NOT CULLING
	if(det > -EPSILON && det < EPSILON){
		return false;
	}

	inv_det = 1.f / det;

	// calculate distance from V1 to ray origin
	T = O - V1;

	// calculate u parameter and test bound
	u = T.GetDot(P) * inv_det;

	// the intersection lies outside of the triangle
	if(u < 0.f || u > 1.f){
		return false;
	}

	// prepare to test v parameter
	Q = T.GetCrossed(e1);

	// calculate V parameter and test bound
	v = D.GetDot(Q) * inv_det;

	// the intersection lies outside of the triangle
	if(v < 0.f || u + v  > 1.f){
		return false;
	}

	t = e2.GetDot(Q) * inv_det;

	if(t > EPSILON){ // ray intersection
		R = O + D * t; // store intersection point
		return true;
	}

	// no hit, no win
	return false;
}

// IsInsideMesh determines if a given point is inside of the base model mesh
// Adapted from Amnon Owed's ofxPointInMesh::isInside: https://github.com/AmnonOwed/ofxPointInMesh
bool IsInsideMesh(Vector &p) {
    // if no mesh was provided, skip this test
    if(fullModelFilename.empty()) {
        return false;
    }

    Vector foundIntersection; // variable to store a single found intersection
	vector<Vector> results;  // vector to store all found intersections
	Vector randomDirection = Vector(0.1, 0.2, 0.3); // a random direction

    // go over all the shapes in the mesh
    for (size_t s = 0; s < fullModelShapes.size(); s++) {
        size_t index_offset = 0;

        // go over all the faces in the shape
        for (size_t f = 0; f < fullModelShapes[s].mesh.num_face_vertices.size(); f++) {
            unsigned int fv = fullModelShapes[s].mesh.num_face_vertices[f];
            vector<Vector> vertices;

            // collect the vertices
            for (size_t v = 0; v < fv; v++) {
                tinyobj::index_t idx = fullModelShapes[s].mesh.indices[index_offset + v];
                tinyobj::real_t vx = fullModelAttrib.vertices[3*idx.vertex_index+0];
                tinyobj::real_t vy = fullModelAttrib.vertices[3*idx.vertex_index+1];
                tinyobj::real_t vz = fullModelAttrib.vertices[3*idx.vertex_index+2];
                vertices.push_back(Vector(vx, vy, vz));
            }

            index_offset += fv;

            // do a triangle-ray intersection on each face in the mesh
            // store the intersection (if any) in the variable foundIntersection
            if(IsIntersectingFace(vertices[0], vertices[1], vertices[2], p, randomDirection, foundIntersection)) {
                // store all found intersections
                results.push_back(foundIntersection);
            }
        }
    }

	// handle multiple mesh intersections at the same point (by removing duplicates)
	vector<Vector> unique_results;
	unique_copy(results.begin(), results.end(), back_inserter(unique_results));

	// // determine if the point is inside or outside the mesh, based on the number of unique intersections
	if(unique_results.size() % 2 == 1) {
		return true;
	} else {
		return false;
	}
}

// GetDot2 calculates the dot product of a vector with itself
double GetDot2(const Vector &p) {
    return p.GetDot(p);
}

// GetSign returns -1 for negative numbers, 1 for positive numbers, and 0 for 0
int GetSign(const double &v) {
    int result = 0;
    if(v < 0) {
        result = -1;
    } else if(v > 0) {
        result = 1;
    }
    return result;
}

// clamp ensures that a value (x) is within the range defined by [lower] and [upper]
double clamp(double x, double lower, double upper) {
    return min(upper, max(x, lower));
}

// DistanceToTriangle calculates the shortest distance between a point (p) and a 3D triangle defined by vertices v1, v2, and v3
// Adapted from: https://iquilezles.org/www/articles/triangledistance/triangledistance.htm
double DistanceToTriangle(const Vector &v1, const Vector &v2, const Vector &v3, const Vector &p) {
    Vector v21 = v2 - v1;
    Vector v32 = v3 - v2;
    Vector v13 = v1 - v3;
    Vector p1 = p - v1;
    Vector p2 = p - v2;
    Vector p3 = p - p3;
    Vector nor = v21.GetCrossed(v13);

    return sqrt(
        // inside/outside test
        (GetSign(p1.GetDot(v21.GetCrossed(nor))) +
         GetSign(p2.GetDot(v32.GetCrossed(nor))) +
         GetSign(p3.GetDot(v13.GetCrossed(nor))) < 2.0)
        ?
        // 3 edges
        min( 
            min( 
                GetDot2(v21 * clamp(p1.GetDot(v21) / GetDot2(v21), 0.0, 1.0) - p1),
                GetDot2(v32 * clamp(p2.GetDot(v32) / GetDot2(v32), 0.0, 1.0) - p2)
            ),
            GetDot2(v13 * clamp(p3.GetDot(v13) / GetDot2(v13), 0.0, 1.0) - p3)
        )
        :
        // 1 face
        p1.GetDot(nor) * p1.GetDot(nor) / GetDot2(nor)
    );
}

// Model holds all of the particles and defines their behavior.
class Model {
public:
    Model() :
        m_ParticleSpacing(DefaultParticleSpacing),
        m_AttractionDistance(DefaultAttractionDistance),
        m_MinMoveDistance(DefaultMinMoveDistance),
        m_Stubbornness(DefaultStubbornness),
        m_Stickiness(DefaultStickiness),
        m_BoundingRadius(DefaultBoundingRadius) {}

    void SetParticleSpacing(const double a) {
        m_ParticleSpacing = a;
    }

    void SetAttractionDistance(const double a) {
        m_AttractionDistance = a;
    }

    void SetMinMoveDistance(const double a) {
        m_MinMoveDistance = a;
    }

    void SetStubbornness(const int a) {
        m_Stubbornness = a;
    }

    void SetStickiness(const double a) {
        m_Stickiness = a;
    }

    void SetBoundingRadius(const double a) {
        m_BoundingRadius = a;
    }

    // Add adds a new particle with the specified parent particle
    void Add(const Vector &p, const int parent = -1) {
        const int id = m_Points.size();
        m_Index.insert(make_pair(p.ToBoost(), id));
        m_Points.push_back(p);
        m_Parents.push_back(parent);
        m_JoinAttempts.push_back(0);
        m_BoundingRadius = max(m_BoundingRadius, p.Length() + m_AttractionDistance);

        // update and display progress bar
        ++progressBar;
        progressBar.display();

        // wrap up the progress bar when last particle is placed
        if(id == numParticles) {
            progressBar.done();
        }
    }

    // Nearest returns the index of the particle nearest the specified point
    int Nearest(const Vector &point) const {
        int result = -1;
        m_Index.query(
            boost::geometry::index::nearest(point.ToBoost(), 1),
            boost::make_function_output_iterator([&result](const auto &value) {
                result = value.second;
            }));
        return result;
    }

    // DistanceToNearestFace calculates the shortest distance from a particle (p) and 
    // TODO: if needed, consider putting mesh vertices in spatial index, then doing knn search search within radius defined by largest edge found in mesh (precomputed)
    double DistanceToNearestFace(const Vector &p) {
        double distance = m_AttractionDistance;

        // go over all the shapes in the mesh
        for (size_t s = 0; s < seedModelShapes.size(); s++) {
            size_t index_offset = 0;

            // go over all the faces in the shape
            for (size_t f = 0; f < seedModelShapes[s].mesh.num_face_vertices.size(); f++) {
                unsigned int fv = seedModelShapes[s].mesh.num_face_vertices[f];
                vector<Vector> vertices;

                // collect the vertices
                for (size_t v = 0; v < fv; v++) {
                    tinyobj::index_t idx = seedModelShapes[s].mesh.indices[index_offset + v];
                    tinyobj::real_t vx = seedModelAttrib.vertices[3*idx.vertex_index+0];
                    tinyobj::real_t vy = seedModelAttrib.vertices[3*idx.vertex_index+1];
                    tinyobj::real_t vz = seedModelAttrib.vertices[3*idx.vertex_index+2];
                    vertices.push_back(Vector(vx, vy, vz));
                }

                index_offset += fv;

                // calculate distance from point to face
                distance = min(distance, DistanceToTriangle(vertices[0], vertices[1], vertices[2], p));
            }
        }

        return distance;
    }

    // RandomStartingPosition returns a random point to start a new particle
    Vector RandomStartingPosition() const {
        const double d = m_BoundingRadius;
        return RandomInUnitSphere().Normalized() * d;
    }

    // ShouldReset returns true if the particle has gone too far away and
    // should be reset to a new random starting position
    bool ShouldReset(const Vector &p) const {
        return p.Length() > m_BoundingRadius * 2;
    }

    // ShouldJoin returns true if the point should attach to the specified
    // parent particle. This is only called when the point is already within
    // the required attraction distance.
    bool ShouldJoin(const Vector &p, const int parent) {
        m_JoinAttempts[parent]++;
        if (m_JoinAttempts[parent] < m_Stubbornness) {
            return false;
        }
        return Random() <= m_Stickiness;
    }

    // PlaceParticle computes the final placement of the particle.
    Vector PlaceParticle(const Vector &p, const int parent) const {
        return Lerp(m_Points[parent], p, m_ParticleSpacing);
    }

    // MotionVector returns a vector specifying the direction that the
    // particle should move for one iteration. The distance that it will move
    // is determined by the algorithm.
    Vector MotionVector(const Vector &p) const {
        return RandomInUnitSphere();
    }

    // AddParticle diffuses one new particle and adds it to the model
    void AddParticle() {
        // compute particle starting location
        Vector p = RandomStartingPosition();

        // do the random walk
        while (true) {
            // get distance to nearest other particle
            const int parent = Nearest(p);
            const double d = p.Distance(m_Points[parent]);

            // do not allow particle to stick when its inside of the base model mesh
            if(!IsInsideMesh(p)) {
                // check for particle-particle collisions
                if (d < m_AttractionDistance) {
                    if (!ShouldJoin(p, parent)) {
                        // push particle away a bit
                        p = Lerp(m_Points[parent], p, m_AttractionDistance + m_MinMoveDistance);
                        continue;
                    }

                    // adjust particle position in relation to its parent
                    p = PlaceParticle(p, parent);

                    // add the point
                    Add(p, parent);
                    return;
                }

                // get distance to the nearest seed face
                const double df = DistanceToNearestFace(p);

                // check for particle-face collisions
                if (df < m_AttractionDistance) {
                    Add(p, -1);
                    return;
                }
            }

            // move randomly
            const double m = max(m_MinMoveDistance, d - m_AttractionDistance);
            p += MotionVector(p).Normalized() * m;

            // check if particle is too far away, reset if so
            if (ShouldReset(p)) {
                p = RandomStartingPosition();
            }
        }
    }

    // OutputPointData creates a new CSV file and fills it with most current point data
    void OutputPointData(const int iteration) const {
        file.open("data/" + filename + "-" + to_string(iteration) + ".csv");

        for(unsigned int id = 0; id < m_Points.size(); id++) {
            file << id << "," << m_Parents[id] << "," << m_Points[id].X() << "," << m_Points[id].Y() << "," << m_Points[id].Z() << endl;
        }

        file.close();
    }

private:
    // m_ParticleSpacing defines the distance between particles that are
    // joined together
    double m_ParticleSpacing;

    // m_AttractionDistance defines how close together particles must be in
    // order to join together
    double m_AttractionDistance;

    // m_MinMoveDistance defines the minimum distance that a particle will move
    // during its random walk
    double m_MinMoveDistance;

    // m_Stubbornness defines how many interactions must occur before a
    // particle will allow another particle to join to it.
    int m_Stubbornness;

    // m_Stickiness defines the probability that a particle will allow another
    // particle to join to it.
    double m_Stickiness;

    // m_BoundingRadius defines the radius of the bounding sphere that bounds
    // all of the particles
    double m_BoundingRadius;

    // m_Points stores the final particle positions
    vector<Vector> m_Points;

    // m_Parents stores the parent IDs of each clustered particle
    vector<int> m_Parents;

    // m_JoinAttempts tracks how many times other particles have attempted to
    // join with each finalized particle
    vector<int> m_JoinAttempts;

    // m_Index is the spatial index used to accelerate nearest neighbor queries
    Index m_Index;
};


// create the model in global scope so that parseArgs can configure it
Model model;


// Parses CLI arguments and configures simulation with what is passed
void ParseArgs(int argc, char* argv[]) {
    try {
        cxxopts::Options options(argv[0]);

        options
        .allow_unrecognised_options()
        .add_options()
            ("p,particles", "Number of walker particles", cxxopts::value<int>())
            ("i,input", "Full 3D model filename (.obj)", cxxopts::value<string>())
            ("f,seed", "Seed faces 3D model filename (.obj)", cxxopts::value<string>())
            ("o,output", "Output filename", cxxopts::value<string>())
            ("n,interval", "Point data capture interval", cxxopts::value<int>())
            ("s,spacing", "Particle spacing", cxxopts::value<double>())
            ("a,attraction", "Attraction distance", cxxopts::value<double>())
            ("m,move", "Minimum move distance", cxxopts::value<double>())
            ("b,stubbornness", "Stubbornness", cxxopts::value<int>())
            ("k,stickiness", "Stickiness", cxxopts::value<double>())
            ("r,radius", "Initial bounding radius", cxxopts::value<double>())
        ;

        auto result = options.parse(argc, argv);

        if(result.count("particles")) {
            numParticles = result["particles"].as<int>();
            interval = numParticles;
            progressBar.setTotal(numParticles);
        }

        if(result.count("input")) {
            fullModelFilename = result["input"].as<string>();
            
            // throw error when filename doesn't end in `.obj`
            if(fullModelFilename.substr(fullModelFilename.length() - 4, fullModelFilename.length() - 1).compare(".obj") != 0) {
                cerr << "Base model file must be an OBJ file" << endl;
                exit(1);
            }
        }

        if(result.count("seed")) {
            seedModelFilename = result["seed"].as<string>();
            
            // throw error when filename doesn't end in `.obj`
            if(seedModelFilename.substr(seedModelFilename.length() - 4, seedModelFilename.length() - 1).compare(".obj") != 0) {
                cerr << "Seed model file must be an OBJ file" << endl;
                exit(1);
            }
        }

        if(result.count("output")) {
            filename = result["output"].as<string>();

            // throw error when filename doesn't end in `.csv`
            if(filename.substr(filename.length() - 4, filename.length() - 1).compare(".csv") != 0) {
                cerr << "Output file must be a CSV file" << endl;
                exit(1);
            }

            // trim the `.csv` extension (it'll be added by OutputPointData later)
            for(int i = 0; i < 4; i++) {
                filename.pop_back();
            }
        }

        if(result.count("interval")) {
            interval = result["interval"].as<int>();
        }
        
        if(result.count("spacing")) {
            model.SetParticleSpacing(result["spacing"].as<double>());
        }

        if(result.count("attraction")) {
            model.SetAttractionDistance(result["attraction"].as<double>());
        }

        if(result.count("move")) {
            model.SetMinMoveDistance(result["move"].as<double>());
        }

        if(result.count("stubbornness")) {
            model.SetStubbornness(result["stubbornness"].as<int>());
        }

        if(result.count("stickiness")) {
            model.SetStickiness(result["stickiness"].as<double>());
        }

        if(result.count("radius")) {
            model.SetBoundingRadius(result["radius"].as<double>());
        }
    } catch(const cxxopts::OptionException& e) {
        cout << "Error parsing options: " << e.what() << endl;
        exit(1);
    }
}

// LoadFullModel loads the 3D model passed with the `-i` option
void LoadFullModel() {
    string warn;
    string err;

    bool ret = tinyobj::LoadObj(&fullModelAttrib, &fullModelShapes, &fullModelMaterials, &warn, &err, fullModelFilename.c_str());

    if (!warn.empty()) {
        cout << warn << endl;
    }

    if (!err.empty()) {
        cerr << err << endl;
    }

    if (!ret) {
        exit(1);
    }
}

// LoadSeedModel loads the 3D model passed with the `-f` option
void LoadSeedModel() {
    string warn;
    string err;

    bool ret = tinyobj::LoadObj(&seedModelAttrib, &seedModelShapes, &seedModelMaterials, &warn, &err, seedModelFilename.c_str());

    if (!warn.empty()) {
        cout << warn << endl;
    }

    if (!err.empty()) {
        cerr << err << endl;
    }

    if (!ret) {
        exit(1);
    }
}

int main(int argc, char* argv[]) {
    // parse the CLI arguments
    ParseArgs(argc, argv);

    // load the full 3D model for inhibiting growth
    if(!fullModelFilename.empty()) {
        LoadFullModel();
    }

    // load the seed faces 3D model
    if(!seedModelFilename.empty()) {
        LoadSeedModel();
    }

    // add seed point at origin (a single point is necessary for now)
    model.Add(Vector());

    // {
    //     const int n = 3600;
    //     const double r = 1000;
    //     for (int i = 0; i < n; i++) {
    //         const double t = (double)i / n;
    //         const double a = t * 2 * M_PI;
    //         const double x = cos(a) * r;
    //         const double y = sin(a) * r;
    //         model.Add(Vector(x, y, 0));
    //     }
    // }

    // run diffusion-limited aggregation
    for (int i = 1; i <= numParticles; i++) {
        model.AddParticle();
        
        // output current point data based on interval
        if(i - lastOutputIteration >= interval) {
            model.OutputPointData(i);
            lastOutputIteration = i;
        }
    }

    return 0;
}
