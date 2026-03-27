# RTBase

## Current Image of Cornell Box

<img width="1538" height="1584" alt="Screenshot 2026-03-27 205218" src="https://github.com/user-attachments/assets/bb3e0792-bd11-4492-ba49-af12405470c8" />

## Overview

This project provides a foundational ray tracing renderer for students studying Advanced Computer Graphics. The codebase is structured to facilitate learning by including sections where students need to complete missing implementations.

## Project Structure

```
Renderer/
│── Core.h               # Core mathematical and utility functions
│── Renderer.h           # Main ray tracing logic
│── Sampling.h           # Monte Carlo sampling utilities
│── Scene.h              # Scene representation and camera logic
│── SceneLoader.h        # Loads scenes and configurations
│── Lights.h             # Light source definitions
│── Geometry.h           # Geometric structures and operations
│── Imaging.h            # Image generation and storage
│── Materials.h          # Materials and shading models
│── GEMLoader.h          # External loader for scene assets
│── GamesEngineeringBase.h  # Base utilities for integration
```

## Scenes

Scenes can be found on the Moodle page. Please download them and place them in the working directory.

## Tasks for Students

Several functions and algorithms are left incomplete and require implementation. Look for comments such as:

```cpp
// Add code here
```

## Notes

- Ensure your implementations are efficient and well-commented.
- Test incremental changes using appropriate scenes.
  
