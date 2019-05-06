# dlaf with models

Extension of Michael Fogleman's [dlaf](https://github.com/fogleman/dlaf) that enables complex, large-scale DLA growth on 3D models.

## Features
* Fully configurable via command line interface
* Periodic output of point cloud data so that growth animations can be created
* Live updating progress bar so you know how far along the simulation is when running
* Supports import of a manifold 3D model to inhibit growth of particles
* Supports import of 3D model containing faces that particle clusters can grow on. You can have this model contain only some of the faces from the full 3D model to influence where clusters get started.

## Algorithm overview
1. Load decimated 3D model that walkers will not be allowed to cluster inside of.
   1. Do not "reset" particles that travel inside model. Instead, allow them to wander until they exit again. Will this make the growth seem to "crawl" along the surface of the model?
2. Load 3D model containing face patches to act as seeds for clustering.
3. For each walker ...
   1. Check if inside of 3D model. If NO, continue to next steps.
   2. For each face of the cluster face patches ...
      1. Calculate minimal distance between walker and face. If distance is less than or equal to attraction distance, continue.
      2. Project walker coordinates onto plane of the face.
      3. Check if projected point is within face triangle. If YES, stick.
   3. Run all other particle-particle collision checks as normal.

## Install

**Win10**
1. Enable the Windows Subsystem for Linux (WSL) and install a distro (I used Ubuntu): https://docs.microsoft.com/en-us/windows/wsl/install-win10
2. Use the Linux bash to run the install commands for Unix below.

**Unix/Linux**
1. Install make, boost and g++
```
sudo apt install make libboost-dev g++
```
2. Compile the project using the Makefile
```
make
```
3. Run the compiled application
```
./dlaf
```

**MacOS**
```
Can you provide install notes for Mac? Please let me know or open a PR with the README changes!
```

## Usage
### Base 3D model
A 3D model can be provided to the application to inhibit the growth of particles. This may allow particle clusters to spread across the surface of the model given the right parameters.

* Model must be in the `.obj` format
* Model should be manifold (no holes). Use Meshmixer or similar to repair it, if needed.
* Keep triangle count low to improve performance. A ray is cast from every walker to every face to determine if it's inside or outside, so fewer triangles means fewer computations.

### Selected faces 3D model
Another 3D model containing only some of the faces of the previous 3D model can be provided to serve as "seed" points for particle clusters. Collision detection is run for each walker against each face in this model so that clusters can be spawn when particles collide with them, even if no other clustered particles are there. 

* Model must be in the `.obj` format
* Model does not need to be manifold.

### CLI options
The following arguments are available for configuring the simulation to your liking:

| Flag | Long form        | Description                            | Example         | Default           |
|---   |---               |---                                     |---              |---                |
| `-p` | `--particles`    | Number of walkers                      | `-p 10000`      | 1000000           |
| `-i` | `--input`        | Filename of base 3D model              | `-i cube.obj`   | _None_            |
| `-f` | `--faces`        | Filename of 3D model with seed faces   | `-f faces.obj`  | _None_            |
| `-o` | `--output`       | Output file name                       | `-o output`     | `points`          |
| `-n` | `--interval`     | Output point data every _n_ iterations | `-n 100`        | End of simulation |
| `-s` | `--spacing`      | Particle spacing                       | `-s 2.0`        | 1                 |
| `-a` | `--attraction`   | Attraction distance                    | `-a 5.0`        | 3                 |
| `-m` | `--move`         | Minimum move distance                  | `-m 2.0`        | 1                 |
| `-b` | `--stubbornness` | Stubbornness                           | `-b 5`          | 0                 |
| `-k` | `--stickiness`   | Stickiness                             | `-k 3`          | 1                 |
| `-r` | `--radius`       | Initial bounding radius                | `-r 10.0`       | 0                 |

### Examples
Use 1000 particles and output a point data file every 10 iterations.
```
./dlaf -p 1000 -n 10
```

Use default particle count and output to point data file with custom name
```
./dlaf -o output
```

Use custom particle count and tweak several simulation parameters in various forms
```
./dlaf -p 50000 -s 5.0 --stickiness 4 -a 10.0
```

Use one 3D model to inhibit growth (`-i`) and another containing faces to seed cluster growth (`-f`). Note that you will probably want to increase the initial size of the bounding radius (`-r`) to be at least as large as these models.
```
./dlaf -i base.obj -f faces.obj -r 400.0
```

## Acknowledgements
* [dlaf](https://github.com/fogleman/dlaf) by Michael Fogleman for blazingly-fast 2D/3D DLA implementation
* [tinyobjloader](https://github.com/syoyo/tinyobjloader) by syoyo for parsing of OBJ files
* [cxxopts](https://github.com/jarro2783/cxxopts) by jaroo2783 for easy cross-platform CLI option handling
* [progress-cpp](https://github.com/prakhar1989/progress-cpp) by prakhar1989 for a CLI progress bar
* Method for determining if a point is within a 3D mesh adapted from Amnon Owed's [ofxPointInMesh](https://github.com/AmnonOwed/ofxPointInMesh)
* Method for computing point-triangle distance in 3D adapted from Íñigo Quílez's [distance to triangle](https://iquilezles.org/www/articles/triangledistance/triangledistance.htm) article

## TODO
- [X] Rename repo (dlaf-with-models)
- [X] Add parser for 3D models ([tinyobjloader](https://github.com/syoyo/tinyobjloader))
- [X] Add collision detection between walkers and faces from 3D model
- [X] Add ability to check if walker is inside a 3D model
- [X] Create 3D model of cube
- [ ] Create 3D model of faces selected from sub-divided cube to serve as seed points for clusters
  - Use Meshmixer
- [ ] Work out rendering pipeline.
  - Look into Embree, Mitsuba, and OSPRay
  - Look into higher-level interfaces like Houdini
- [ ] Output color values for points based on mesh
  - [ ] "Fade" colors out based on distance from model surface
- [ ] Measure performance of updated algorithm (below).
  - Based on results, look into "Dockerizing" the app and running it on AWS.
- [X] Add DefaultBoundingRadius value, with CLI option
- [X] Add progress bar, if feasible
- [X] Implement periodic output of timestamped point data in order to create animations of growth process.
- [X] Implement CLI flags
  - [X] Particle count
  - [X] Output file name
  - [X] Frequency of point data output (in iterations)
  - [X] Attraction distance
  - [X] Particle spacing
  - [X] Minimum move distance
  - [X] Stubbornness
  - [X] Stickiness
  - [X] Bounding radius