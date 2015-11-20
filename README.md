# easyBA
C++ wrapper around [SBA](http://users.ics.forth.gr/~lourakis/sba/) that uses [Bundler](http://www.cs.cornell.edu/~snavely/bundler/) output file format as input.

# File formats
## Bundler files
Bundler files follow the same file format as Bundler's output ".out" files.
Camera parameters:
- f : focal length
- k1, k2 : radial distortion
- r0, r1, ..., r8 : rotation matrix
- t0, t1, t2 : translation vector

## Extended Bundler files
Extends Bundler's output file format. Uses a more detailed camera model. The differences are : 
- different horizontal and vertical focal lengths
- addition of the principal point position
- addition of a third radial distortion parameter

Camera parameters:
- fx, fy : horizontal and vertical focal length
- cx, cy : principal point
- k1, k2, k3 : radial distortion
- r0, r1, ..., r8 : rotation matrix
- t0, t1, t2 : translation vector