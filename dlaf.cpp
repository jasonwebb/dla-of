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

// number of particles
const int DefaultNumberOfParticles = 1000000;
int numParticles = DefaultNumberOfParticles;

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

// boost is used for its spatial index
using BoostPoint = boost::geometry::model::point<
    double, D, boost::geometry::cs::cartesian>;

using IndexValue = pair<BoostPoint, int>;

using Index = boost::geometry::index::rtree<
    IndexValue, boost::geometry::index::linear<4>>;

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
    static thread_local mt19937 gen(
        chrono::high_resolution_clock::now().time_since_epoch().count());
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

// Model holds all of the particles and defines their behavior.
class Model {
public:
    Model() :
        m_ParticleSpacing(DefaultParticleSpacing),
        m_AttractionDistance(DefaultAttractionDistance),
        m_MinMoveDistance(DefaultMinMoveDistance),
        m_Stubbornness(DefaultStubbornness),
        m_Stickiness(DefaultStickiness),
        m_BoundingRadius(0) {}

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

    // Add adds a new particle with the specified parent particle
    void Add(const Vector &p, const int parent = -1) {
        const int id = m_Points.size();
        m_Index.insert(make_pair(p.ToBoost(), id));
        m_Points.push_back(p);
        m_Parents.push_back(parent);
        m_JoinAttempts.push_back(0);
        m_BoundingRadius = max(
            m_BoundingRadius, p.Length() + m_AttractionDistance);
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

            // check if close enough to join
            if (d < m_AttractionDistance) {
                if (!ShouldJoin(p, parent)) {
                    // push particle away a bit
                    p = Lerp(m_Points[parent], p,
                        m_AttractionDistance + m_MinMoveDistance);
                    continue;
                }

                // adjust particle position in relation to its parent
                p = PlaceParticle(p, parent);

                // add the point
                Add(p, parent);
                return;
            }

            // move randomly
            const double m = max(
                m_MinMoveDistance, d - m_AttractionDistance);
            p += MotionVector(p).Normalized() * m;

            // check if particle is too far away, reset if so
            if (ShouldReset(p)) {
                p = RandomStartingPosition();
            }
        }
    }

    // OutputPointData creates a new CSV file and fills it with most current point data
    void OutputPointData(const int iteration) const {
        file.open(filename + "-" + to_string(iteration) + ".csv");

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


// create the model
Model model;


// Parses CLI arguments and configures simulation with what is passed
void parseArgs(int argc, char* argv[]) {
    try {
        cxxopts::Options options(argv[0]);

        options
        .allow_unrecognised_options()
        .add_options()
            ("p,particles", "Number of walker particles", cxxopts::value<int>())
            ("o,output", "Output filename", cxxopts::value<string>())
            ("i,interval", "Point data capture interval", cxxopts::value<int>())
            ("s,spacing", "Particle spacing", cxxopts::value<double>())
            ("a,attraction", "Attraction distance", cxxopts::value<double>())
            ("m,move", "Minimum move distance", cxxopts::value<double>())
            ("b,stubbornness", "Stubbornness", cxxopts::value<int>())
            ("k,stickiness", "Stickiness", cxxopts::value<double>())
        ;

        auto result = options.parse(argc, argv);

        if(result.count("particles")) {
            numParticles = result["particles"].as<int>();
            interval = numParticles;
        }

        if(result.count("output")) {
            // TODO: trim `.csv` from end of string
            filename = result["output"].as<string>();
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
    } catch(const cxxopts::OptionException& e) {
        cout << "Error parsing options: " << e.what() << endl;
        exit(1);
    }
}

int main(int argc, char* argv[]) {
    // parse the CLI arguments
    parseArgs(argc, argv);

    // add seed point(s)
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
