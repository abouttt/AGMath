# AGM - Abouttt Game Math

AGM (About Game Math) is a C++20 math library primarily designed for game development. It provides core functionalities like Vectors, Quaternions, and Matrices, drawing inspiration from established game engines like **Unreal Engine** and **Unity Engine**. This library is developed and primarily tested with MSVC (Microsoft Visual C++).

## 🌟 Features

* **Core Mathematical Constructs**:
    * `Vector2`, `Vector3`, `Vector4` classes for 2D, 3D, and 4D operations.
    * `Quaternion` class for handling rotations and orientation.
    * `Matrix4x4` class for 4x4 matrix operations, suitable for transformations in 3D space.
* **Inspired by Industry Standards**: The API design and functionalities are influenced by common practices found in leading game engines such as Unreal Engine and Unity, aiming for a familiar and intuitive experience for game developers.
* **Coordinate System**: Primarily targets a **Left-Handed Coordinate System**. This is common in some game engines and graphics APIs (e.g., DirectX).
    * Projection matrices (`Perspective`, `Orthographic`, `Frustum`) are set up for a Z-range of **\[-1, 1]** (OpenGL-style depth convention, though the view matrix itself is LHS).
* **C++20 Standard**: Leverages modern C++20 features for a more robust and efficient codebase.
* **MSVC Focused**: Developed and primarily tested with the Microsoft Visual C++ compiler.
* **Header-Only (Implied)**: Structured as a header-only library for straightforward integration into projects.
* **Utility Functions**: Includes a `utilities.h` header with a collection of mathematical helper functions (e.g., `Lerp`, `Clamp`, `WrapAngle`, `IsNearlyZero`, trigonometric functions).

## ⚠️ Important Notes

* **Work in Progress / Experimental**: Not all functions and features have been exhaustively tested. This library is an ongoing project. Use with consideration and please report any issues or bugs you encounter!
* **MSVC `constexpr` Limitations**: Due to MSVC's default handling of `constexpr` with standard C math library functions (like `std::sqrt`, `std::sin`) for floating-point types in C++17/20, `constexpr` has been removed from functions that call these to ensure wider compatibility. Static constants for math types are typically defined using `inline const`.
* **Clear Role Separation**:
    * **Vectors** (`Vector2`, `Vector3`, `Vector4`) are primarily for representing **positions, directions, offsets, and magnitudes**.
    * **Quaternions** (`Quaternion`) are dedicated to handling **rotations and orientations**.
    * Vector rotation and transformation are achieved by applying quaternion or matrix operations to vectors.

## 🛠️ Usage

To use AGM in your project, simply include the necessary header files.

```cpp
#include "agm/agmath.h"       // For all include Headers
#include "agm/vector3.h"      // For 3D vectors
#include "agm/quaternion.h"   // For rotations
#include "agm/matrix4x4.h"    // For transformation matrices
#include "agm/utilities.h"    // For helper functions

// ... your game logic ...

agm::Vector3 currentPosition(10.0f, 0.0f, 5.0f);
agm::Quaternion ninetyDegAroundY = agm::Quaternion::AngleAxis(90.0f, agm::Vector3::UP);
agm::Vector3 newPosition = ninetyDegAroundY.RotateVector3(currentPosition);

agm::Matrix4x4 modelMatrix = agm::Matrix4x4::TRS(newPosition, ninetyDegAroundY, agm::Vector3::ONE);
