Based on [dlaf](https://github.com/fogleman/dlaf) by Michael Fogleman

## Goal
To grow large-scale (10M+ particle) DLA clusters on 3D models.

## TODO
- [ ] Port to openFrameworks, or add 3D model import to original implementation (_IN PROGRESS_)
  * oF has [oFMesh](https://openframeworks.cc/documentation/3d/ofMesh/) built in, but doesn't play nicely with boost. Also, the toolchain kind of sucks (Visual Studio).
  * Will be broken down more based on findings of research into oF vs vanilla C++ approach
- [ ] Create 3D model of cube, but with lots of sub-faces
- [ ] Create 3D model of faces selected from sub-divided cube to serve as seed points for clusters
  * Use Meshmixer
- [ ] Work out rendering pipeline.
  * Look into Embree, Mitsuba, and OSPRay
- [ ] Measure performance of updated algorithm (below).
  * Based on results, look into "Dockerizing" the app and running it on AWS.
- [ ] Implement periodic output of timestamped point data in order to create animations of growth process.

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
