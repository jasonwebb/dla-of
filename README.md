# dlaf with models

Based on [dlaf](https://github.com/fogleman/dlaf) by Michael Fogleman

## Goal
To grow large-scale (10M+ particle) DLA clusters on 3D models.

## TODO
- [ ] Rename repo (dlaf-on-models, dlaf-with-models, dlaf-for-models)
- [X] Add parser for 3D models ([tinyobjloader](https://github.com/syoyo/tinyobjloader))
- [ ] Add collision detection between walkers and faces from 3D model
- [X] Add ability to check if walker is inside a 3D model
- [ ] Create 3D model of cube, but with lots of sub-faces
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
- [ ] Add progress bar, if feasible
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

**Unix-based (Mac, Linux)**
1. Install make, boost and g++
```
sudo apt-get install make libboost-dev g++
```
2. Compile the project using the Makefile
```
make
```
3. Run the compiled application
```
./dlaf
```

## Usage
The following arguments are available for configuring the simulation to your liking:

| Flag | Long form        | Description                            | Example         | Default           |
|---   |---               |---                                     |---              |---                |
| `-p` | `--particles`    | Number of walkers                      | `-p 10000`      | 1000000           |
| `-i` | `--input`        | Input 3D model file name               | `-i cube.obj`   | _None_            |
| `-o` | `--output`       | Output file name                       | `-o output.csv` | `points.csv`.     |
| `-n` | `--interval`     | Output point data every _n_ iterations | `-n 100`        | End of simulation |
| `-s` | `--spacing`      | Particle spacing                       | `-s 2.0`        | 1                 |
| `-a` | `--attraction`   | Attraction distance                    | `-a 5.0`        | 3                 |
| `-m` | `--move`         | Minimum move distance                  | `-m 2.0`        | 1                 |
| `-b` | `--stubbornness` | Stubbornness                           | `-b 5`          | 0                 |
| `-k` | `--stickiness`   | Stickiness                             | `-k 3`          | 1                 |
| `-r` | `--radius`       | Initial bounding radius                | `-r 10.0`      | 0                 |kv

### Examples
Use 1000 particles and output a CSV file every 10 iterations.
```
./dlaf -p 1000 -n 10
```

Use default particle count and output to custom CSV file
```
./dlaf -o output.csv
```

Use custom particle count and tweak several simulation parameters in various forms
```
./dlaf -p 50000 -s 5.0 --stickiness 4 -a 10.0
```
