## ElasticBody2D

![](https://github.com/cp3-ws1920/elastic_body_2d/raw/master/preview.gif)

A module for the open source game engine [Godot](https://github.com/godotengine/godot) that adds a new Node, *ElasticBody2D*. Attached to a *Polygon2D* or *CollisionPolygon2D* it will triangulate the polygon and enable real-time elastic deformations of the shape using a finite element method.

This was done as a student project for the course Computational Physics III at the FSU Jena in the winter term 19/20.

It uses a [delaunay triangulator](https://github.com/cp3-ws1920/triangulator) and a [finite element solver](https://github.com/cp3-ws1920/fem_solver) submodule which I also wrote for my project.

## Installation of the module
To get the source of Godot 3.2.1 stable and add the module, run

```
git clone  --single-branch --branch 3.2.1-stable https://github.com/godotengine/godot
cd godot
git submodule add https://github.com/cp3-ws1920/elastic_body_2d modules/elastic_body_2d
git submodule update --init --recursive
```

To then compile a custom build of Godot which contains the new Node, simply follow the instructions for [compiling Godot from source](http://docs.godotengine.org/en/3.2/development/compiling/). (Normally, this should boil down to execute `scons platform=<windows/x11/osx> target=<release/debug>` with SCons and a compiler installed.)

An executable will then be located inside `godot/bin`. A precompiled binary can also be downloaded from the [release page](https://github.com/cp3-ws1920/elastic_body_2d/releases).

## How to use

Using a freshly compiled or [precompiled](https://github.com/cp3-ws1920/elastic_body_2d/releases) binary of the engine wih the module, the new Node ca be found under `Node/CanvasItem/Node2D/ElasticBody2D` from the node creation dialog. It can either be attached to a `Polygon2D` or a `CollisionPolygon2D` that you want to deform. Any `Area2D` that is attached to a `ElasticBody2D` node will be used to constrain nodes lying within the area.

### Enumerations

`enum`**`Method`**:

* **`METHOD_EXPLICIT_EULER = 0`**
* **`METHOD_MODIFIED_EXPLICIT_EULER = 1`**
* **`METHOD_IMPROVED_EULER = 2`**
* **`METHOD_RUNGE_KUTTA_3 = 3`**
* **`METHOD_RUNGE_KUTTA_4 = 4`**
* **`METHOD_IMPLICIT_EULER = 5`**

### Property Descriptions

* **`float`**`poisson_ratio`

|||
|----------|----------------------------|
| *Setter* | `set_poisson_ratio(value)` |
| *Getter* | `get_poisson_ratio()`      |

[Poisson ratio](https://en.wikipedia.org/wiki/Poisson%27s_ratio) of the body.


* **`float`**`young_modulus`

|||
|----------|----------------------------|
| *Setter* | `set_young_modulus(value)` |
| *Getter* | `get_young_modulus()`      |

[Young modulus](https://en.wikipedia.org/wiki/Young%27s_modulus) of the body.

* **`float`**`friction`

|||
|----------|----------------------------|
| *Setter* | `set_friction(value)` |
| *Getter* | `get_friction()`      |

Friction or dampening of the body when in free motion.

* **`float`**`min_angle`

|||
|----------|----------------------------|
| *Setter* | `set_min_angle(value)` |
| *Getter* | `get_min_angle()`      |

Minimum angle for triangles in the triangulation of the polygon.

* **`PoolVector2Array`**`forces`

|||
|----------|----------------------------|
| *Setter* | `set_forces(value)` |
| *Getter* | `get_forces()`      |

Array of forces applied to each node when calling `deform()`.

* **`Method`**`method`

|||
|----------|----------------------------|
| *Setter* | `set_method(value)` |
| *Getter* | `get_method()`      |

Method used for integration of the motion equations when calling `free_motion()`.

* **`bool`**`fixed_delta`

|||
|----------|----------------------------|
| *Setter* | `is_fixed_delta(value)` |
| *Getter* | `get_fixed_delta()`      |

If `true` a fixed delta is used when calling `free_motion()`. This allows preprocessing of the inverse matrix when using the Implicit Euler Method and thus improves performance for this method. The internal delta time is set to Godot's phyisics delta time on node initialization. (Godot's physics delta thus shouldn't be changed at runtime when using this option.)

### Method Descriptions

* **`void`**`deform()`

Instantly deforms the parent `Polygon2D` or `CollisionPolygon2D` from the forces in the `forces` property.

* **`void`**`free_motion(delta)`

Performs a single free motion step acting as if there were no forces applied on the body. Uses the method set in the `method` property. When the `fixed_delta` step is enabled, `delta` will be ignored. `free_motion()` should then only be called from within `_phyisics_process` as the internal delta is set to Godot's phyiscs delta at node initialization.

* **`PoolVector2Array`**`get_velocities()`

Returns the velocities of the vertices.

* **`PoolVector2Array`**`get_displacements()`

Returns the displacements of the vertices (the offsets to their original positions).

## Credits

This repository as well as its submodules use [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page), a free C++ template library for linear algebra.
