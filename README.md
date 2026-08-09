# ZweiCFD (Formerly ZweiFoil)

ZweiCFD is an interactive and fun Computational Fluid Dynamics (CFD) solver and visualization tool built using C++, Qt6, and VTK. It is designed to run real-time aerodynamics simulations while providing a highly interactive graphical user interface for dynamically tweaking simulation parameters.



## Core Features

### 1. Active Solver: Lattice Boltzmann Method (LBM)
ZweiCFD is powered by a 3D Lattice Boltzmann volumetric solver.
- Uses a **D3Q19** discrete velocity model (19 directions in 3D).
- Implicit geometry representation via Signed Distance Fields (SDF) allows for arbitrary boundaries and interactive "drawing" inside the domain in real-time.
- Highly optimized volumetric step simulation with VTK integration for live 3D flow rendering (streamlines, velocity slices).

### 2. Experimental Features
- **2D Inviscid LVPM Solver**: The codebase includes a 2D Lumped Vortex Point Method solver for potential flow, but it is currently unhooked from the main simulation pipeline. This was a thing of the past, but has been discontinued in the favor of 3d support. as it kept messing up the 3d 2d logic
- **Flap Airfoil**: The flap airfoil implementation is currently not working well. Due to low compute resources because of my laptop being low end, I wasn't able to run the simulation smoothly enough to properly debug and resolve the underlying issues.

### 3. Geometry & Airfoil Support
- **Procedural Generation**: Dynamically generate NACA 4-digit series airfoils by adjusting sliders for camber and thickness in real-time.
- **Custom Airfoils**: Loads custom airfoil coordinates from standard `.dat` files.
- The 2D coordinates are extruded into the 3D domain for the volumetric LBM solver

### 4. Interactive UI & Visualization (Qt6 + VTK)
- **Real-Time Tweaking**: You Are Able To adjust the AOA, The speed,  thickness, camber shape (only for default airfoil) on the go, and the lbm solver reinitiates to calculate it again
- **Interactive Domain Drawing**: Use different brush shapes (spheres, cubes, cones) to draw custom 3D obstacles directly into the flow domain using raycasting. (although pretty crude yet. will work on it in the future)
- **Rich Visualization**: Render the flow field with VTK, featuring customizable colormaps, variable streamline densities, and velocity heatmaps.

## Dependencies

- C++17 Compiler
- CMake 3.10+
- Qt6 (Widgets)
- VTK (Common, Rendering, Filters, etc.)
- Eigen 3.4.0 (Fetched automatically via CMake)
- OpenMP

## Building from Source

```bash
mkdir build
cd build
cmake ..
cmake --build .
```

## Downloads

You Can Download For Windows From The Releases Page. As This Depends On VTK, You Will Not be Able To Run the Exe Standalone So I Have Zipped the Necessary DLLs With The EXE So It Will Run Properly!

## NOTE
There might be some issues with the code. as i am working solo and managing my studies, i simply havent had the time and motivation to fix them all as i have been getting burned out by this project. i would appreciate your support and reviews so that i may get the motivation to push it to great heights!!!! :3

