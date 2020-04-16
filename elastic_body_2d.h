#ifndef ELASTIC_BODY_2D_H
#define ELASTIC_BODY_2D_H

#include "scene/2d/node_2d.h"
#include "scene/2d/area_2d.h"
#include "core/variant.h"
#include "fem.hpp"
#include "triangulator.hpp"

class ElasticBody2D : public Node2D {
    GDCLASS(ElasticBody2D, Node2D);

public:
    enum Method {
		METHOD_EXPLICIT_EULER,
		METHOD_MODIFIED_EXPLICIT_EULER,
		METHOD_IMPROVED_EULER,
        METHOD_RUNGE_KUTTA_3,
        METHOD_RUNGE_KUTTA_4,
        METHOD_IMPLICIT_EULER
	};

private:
    float poisson_ratio;
    float young_modulus;
    float friction;
    float min_angle;

    Method method;
    bool fixed_delta;

    Vector<Area2D *> pinned_areas;
    PoolVector2Array forces;

    PoolVector2Array nodes;
    PoolIntArray map;

    FEM::DeformableMesh2D *solver;
    Triangulator::Delaunay2D *delaunay;

protected:
    static void _bind_methods();

    void _notification(int p_what);

public:
    ElasticBody2D();
    ~ElasticBody2D();

    virtual String get_configuration_warning() const;

    void deform();
    void free_motion();

    void set_poisson_ratio(float p_poisson_ratio);
    float get_poisson_ratio() const;
    void set_young_modulus(float p_young_modulus);
    float get_young_modulus() const;
    void set_friction(float p_friction);
    float get_friction() const;
    void set_min_angle(float p_angle);
    float get_min_angle() const;
    void set_forces(PoolVector2Array f_forces);
    Method get_method() const;
    void set_method(Method p_method);
    bool is_fixed_delta() const;
    void set_fixed_delta(bool p_fixed);
    PoolVector2Array get_forces() const;
    PoolVector2Array get_velocities() const;
    PoolVector2Array get_displacements() const;
};

VARIANT_ENUM_CAST(ElasticBody2D::Method);

#endif // ELASTIC_BODY_2D_H