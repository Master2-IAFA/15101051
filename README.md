# MLoDscope
MLoDscope is a software that implements the "Moving Level-of-Details surfaces", which is a surface approximation technique for point clouds, check it's reference [here](https://perso.telecom-paristech.fr/boubek/papers/MLoDSurfaces/). The software offers an interactive graphical user interface that enables users to test the MLoD technique on .ply point clouds. 

## Dependencies
The software is self contained and requires no external packages. All dependent libraries can be found in the external folder and are comprised of:

- [Polyscope](http://polyscope.run/), which is the core of the GUI
- [OpenMesh](https://www.graphics.rwth-aachen.de/software/openmesh/), used to load point cloud files


## Installation
Run the following commands:

```bash
$ git clone --recurse-submodules git@github.com:Master2-IAFA/15101051.git
$ cd 150101051
$ mkdir build && cd build
$ cmake .. 
$ make -j 8
```
The software has been compiled and tested on Ubuntu 22.04 and Windows 11.

## Manual

The following image shows the customized polyscope gui.

![MLoDscope GUI](doc/gui.jpg)

the 3D - 2D radio buttons allow the user to change the rendering mode.

### File Selection
All files contained in the asset directory are listed in the scroll-menu. A file can be loaded
by clicking its name in the menu and pressing the load button. 

It is worth noting that files are loaded as 3D point clouds no matter the rendering mode, 
the 2D mode simply ignores the Z coordinate. It is up to the user to check consistency for fitting purposes.

### Octree parameters
#### Octree fit
- Depth slider : allows to display the nodes of the current octree at a given depth
- Max Depth + fit : pressing the fit button generates a new octree on the current point
cloud with a given max depth and blends all the nodes' statistics
#### Algebraic sphere display
The user can display the sphere fit from the statistics of any single tree node. To do so, the max depth slider must be set to its correct value. Then, the user can choose a specific node through specifying its depth and idx and pressing the "show sphere" button.
### Fitting 
- sample random points : generates as many points as indicated in the corresponding slider value
inside the bounding box of the current point cloud
- show root's sphere stat : displays the bounding box's protection sphere (The lambda parameter is multiplied by the bounding sphere's radius to give the protection sphere's radius)
- fit : runs one fitting iteration on all sampeled points using the lambda and kernel given parameters
- iterative fit : runs the given number of succesive fitting iteration

A slider appears after fitting. It allows to see the projection of the sampeled points from their original positions.

Warning : running a fit requires sampeled points!

- Clear Random Pc : removes the sampeled points
- Randomize and fit : generates one point sample inside the bounding box, fits its algebraic sphere and projects it on it. The point, the sphere and the traversed octree nodes are, then, displayed
#### Fitting parameters

Two kernels are available. Kernels are use to determine the influence of the points ( or nodes ) from the point cloud on the point *q* being projected, based on their respective distances.

The first kernel is a Gaussian Mixture: $\sum_i \sigma_{i}^{-3} e^{-||q-p||^2\over 2 \sigma_{i}^2}$ which has 3 parameters
 - *k*, the number of Gaussian
 - *sigma*, the sigma value of the first Gaussian
 - *a*, a scale factor for sigma apply on each gaussian ( $\sigma_{i} = a^i * sigma$ )
 
 The second one is a rational kernel: $(||q-p||^2+\epsilon)^{-k \over 2 }$ which has 2 parameters
  - $\epsilon$, if epsilon equals 0, it will result in an interpolating surface
  - *k*
