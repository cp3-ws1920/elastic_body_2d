#include "elastic_body_2d.h"

#include "triangulator.hpp"
#include "fem.hpp"

#include "core/engine.h"
#include "servers/physics_2d_server.h"
#include "scene/2d/polygon_2d.h"
#include "scene/2d/collision_polygon_2d.h"
#include "scene/2d/area_2d.h"

#include <vector>

ElasticBody2D::ElasticBody2D() {
    poisson_ratio = 0.3f;
    young_modulus = 2000.0f;
    friction  = 0.1f;
    min_angle = 20.7f;
    method = ElasticBody2D::Method::METHOD_MODIFIED_EXPLICIT_EULER;

    forces = PoolVector2Array();

    solver = new FEM::DeformableMesh2D();
    delaunay = new Triangulator::Delaunay2D();
}

ElasticBody2D::~ElasticBody2D() {
    delete solver;
    delete delaunay;
}

void ElasticBody2D::deform() {
    std::vector<float> disp;
    for (int i = 0; i < forces.size(); ++i) {
        solver->setForce(i, forces[i].x, forces[i].y);
    }
    disp = solver->calculateDisplacements();
    solver->resetVelocity();

    PoolVector2Array new_pos;

    new_pos.resize(nodes.size());
    for (int i =0; i < nodes.size(); ++i) {
        new_pos.set(i, Vector2(nodes[i].x + disp[2*map[i]+0], nodes[i].y + disp[2*map[i]+1]));
    }
    if (Object::cast_to<CollisionPolygon2D>(get_parent())) {
        Vector<Vector2> poly;
        poly.resize(new_pos.size());
        for (int i = 0; i < poly.size(); ++i) {
            poly.set(i, new_pos[i]);
        }
        Object::cast_to<CollisionPolygon2D>(get_parent())->set_polygon(poly);
    } else if (Object::cast_to<Polygon2D>(get_parent())) {
        Object::cast_to<Polygon2D>(get_parent())->set_polygon(new_pos);
    }    
}

void ElasticBody2D::free_motion() {
    if (fixed_delta && !Engine::get_singleton()->is_in_physics_frame()) {
        WARN_PRINT("free_motion() should only be called from a physics frame if fixed_delta is enabled!");
    }
    std::vector<float> disp;
    float delta = Engine::get_singleton()->is_in_physics_frame() ? get_physics_process_delta_time() : get_process_delta_time();
    disp = solver->freeOscillationStep(delta);

    PoolVector2Array new_pos;

    new_pos.resize(nodes.size());
    for (int i =0; i < nodes.size(); ++i) {
        new_pos.set(i, Vector2(nodes[i].x + disp[2*map[i]+0], nodes[i].y + disp[2*map[i]+1]));
    }
    if (Object::cast_to<CollisionPolygon2D>(get_parent())) {
        Vector<Vector2> poly;
        poly.resize(new_pos.size());
        for (int i = 0; i < poly.size(); ++i) {
            poly.set(i, new_pos[i]);
        }
        Object::cast_to<CollisionPolygon2D>(get_parent())->set_polygon(poly);
    } else if (Object::cast_to<Polygon2D>(get_parent())) {
        Object::cast_to<Polygon2D>(get_parent())->set_polygon(new_pos);
    }   
}

void ElasticBody2D::preprocess() {
    for (int i = 0; i < get_child_count(); ++i) {
        if (Object::cast_to<Area2D>(get_child(i))) {
            pinned_areas.push_back(Object::cast_to<Area2D>(get_child(i)));
        }
    }

    if (Object::cast_to<Polygon2D>(get_parent())) {
        nodes = Object::cast_to<Polygon2D>(get_parent())->get_polygon();
    } else if (Object::cast_to<CollisionPolygon2D>(get_parent())) {
        Vector<Vector2> poly = Object::cast_to<CollisionPolygon2D>(get_parent())->get_polygon();
        nodes.resize(poly.size());
        for (int i = 0; i < nodes.size(); ++i) {
            nodes.set(i, poly[i]);
        }
    } else {
        ERR_PRINT("Parent is neither Polygon2D nor CollisionPolygon2D!");
        return;
    }

    forces.resize(nodes.size());

    std::vector<Triangulator::Vertex *> poly;
    std::vector<Triangulator::Edge> segments;
    poly.resize(nodes.size());
    segments.resize(nodes.size());

    for (int i = 0; i < nodes.size(); ++i) {
        poly[i] = new Triangulator::Vertex(nodes[i].x, nodes[i].y);
    }
    for (int i = 0; i < nodes.size(); ++i) {
        segments[i] = Triangulator::Edge(*poly[i], *poly[(i + 1) % nodes.size()]);
    }

    delaunay->vertices = poly;
    delaunay->segments = segments;
    delaunay->RefineRupperts(min_angle);

    map.resize(nodes.size());
    for (int i = 0; i < delaunay->vertices.size(); ++i) {
        if (delaunay->vertices[i]->maps_to != -1) {
            map.set(delaunay->vertices[i]->maps_to, i);
        }
    }

    std::vector<float> nodes_x;
    std::vector<float> nodes_y;
    std::vector<FEM::Element2D> fem_elements;
    nodes_x.resize(delaunay->vertices.size());
    nodes_y.resize(delaunay->vertices.size());
    fem_elements.resize(delaunay->triangles.size());

    for (int i = 0; i < delaunay->vertices.size(); ++i)
    {
        nodes_x[i] = delaunay->vertices[i]->coordinates.x();
        nodes_y[i] = delaunay->vertices[i]->coordinates.y();
    }

    for (int i = 0; i < fem_elements.size(); ++i)
    {
        FEM::Element2D element;
        // TODO: Why is this in reversed order?
        element.node_ids_[0] = std::distance(delaunay->vertices.begin(), std::find(delaunay->vertices.begin(), delaunay->vertices.end(), delaunay->triangles[i]->v[2]));
        element.node_ids_[1] = std::distance(delaunay->vertices.begin(), std::find(delaunay->vertices.begin(), delaunay->vertices.end(), delaunay->triangles[i]->v[1]));
        element.node_ids_[2] = std::distance(delaunay->vertices.begin(), std::find(delaunay->vertices.begin(), delaunay->vertices.end(), delaunay->triangles[i]->v[0]));
        fem_elements[i] = element;
    }

    solver->setMethod((FEM::DeformableMesh2D::Method)(int(method)));
    solver->setNodesX(nodes_x);
    solver->setNodesY(nodes_y);
    solver->setElements(fem_elements);
    solver->setPoissonRatio(poisson_ratio);
    solver->setYoungModulus(young_modulus);
    solver->setFriction(friction);
    if (fixed_delta) {
        float delta = 1.0f / (float)Engine::get_singleton()->get_iterations_per_second();
        solver->setFixedDelta(delta);
        print_line(String::num_real(delta));
        solver->setFixedDeltaEnabled(true);
    }

    for (int i = 0; i < solver->getNodesX().size(); ++i) {
        Vector2 pos = Vector2(solver->getNodesX()[i], solver->getNodesY()[i]) + get_global_position();
        Physics2DDirectSpaceState::ShapeResult *res = (Physics2DDirectSpaceState::ShapeResult *)malloc(32 * sizeof(Physics2DDirectSpaceState::ShapeResult));
        int count = get_world_2d()->get_direct_space_state()->intersect_point(pos, res, 32, Set<RID>(), 4294967295U, false, true);
        bool flag = false;
        for (int j = 0; j < pinned_areas.size(); ++j) {
            for (int k = 0; k < count; ++k) {
                if (res[k].collider == pinned_areas[j]) {
                    FEM::Constraint constraint;
                    constraint.node = i;
                    constraint.type = static_cast<FEM::Constraint::Type>(3);
                    solver->setConstraint(constraint);
                    flag = true;
                    break;
                }
            }
            if (flag) break;
        }
    }

    solver->preprocess();  
}

void ElasticBody2D::_notification(int p_what) {
    switch (p_what) {
        case NOTIFICATION_READY: {
            if (Engine::get_singleton()->is_editor_hint()) break;

            preprocess();
        }
        case NOTIFICATION_DRAW: {
            if (!Object::cast_to<SceneTree>(get_tree())->is_debugging_collisions_hint()) {
                break;
            }
            std::vector<FEM::Element2D> elements = solver->getElements();
            std::vector<float> disp = solver->getDisplacements();
            std::vector<float> nodes_x = solver->getNodesX();
            std::vector<float> nodes_y = solver->getNodesY();

            for (int i = 0; i < elements.size(); ++i) {
                int i1 = elements[i].node_ids_[0];
                int i2 = elements[i].node_ids_[1];
                int i3 = elements[i].node_ids_[2];

                draw_line(Vector2(nodes_x[i1], nodes_y[i1]) - get_position(), Vector2(nodes_x[i2], nodes_y[i2]) - get_position(), Color(0.9, 0.3, 0.0, 0.8), 1);
                draw_line(Vector2(nodes_x[i2], nodes_y[i2]) - get_position(), Vector2(nodes_x[i3], nodes_y[i3]) - get_position(), Color(0.9, 0.3, 0.0, 0.8), 1);
                draw_line(Vector2(nodes_x[i3] , nodes_y[i3]) - get_position(), Vector2(nodes_x[i1], nodes_y[i1]) - get_position(), Color(0.9, 0.3, 0.0, 0.8), 1);
            }
        }
    }
}

void ElasticBody2D::set_poisson_ratio(float p_poisson_ratio) {
    poisson_ratio = p_poisson_ratio;
}

float ElasticBody2D::get_poisson_ratio() const {
    return poisson_ratio;
}

void ElasticBody2D::set_young_modulus(float p_young_modulus) {
    young_modulus = p_young_modulus;
}

float ElasticBody2D::get_young_modulus() const {
    return young_modulus;
}

void ElasticBody2D::set_friction(float p_friction) {
    friction = p_friction;
}

float ElasticBody2D::get_friction() const {
    return friction;
}

void ElasticBody2D::set_min_angle(float p_angle) {
    min_angle = p_angle;
}

float ElasticBody2D::get_min_angle() const {
    return min_angle;
}

void ElasticBody2D::set_forces(PoolVector2Array p_forces) {
    forces = p_forces;
}

PoolVector2Array ElasticBody2D::get_forces() const {
    return forces;
}

void ElasticBody2D::set_method(ElasticBody2D::Method p_method) { 
    method = p_method;
}

ElasticBody2D::Method ElasticBody2D::get_method() const {
    return method;
}

bool ElasticBody2D::is_fixed_delta() const {
    return fixed_delta;
}

void ElasticBody2D::set_fixed_delta(bool p_fixed) {
    fixed_delta = p_fixed;
}

PoolVector2Array ElasticBody2D::get_velocities() const {
    PoolVector2Array v;
    v.resize(nodes.size());
    std::vector<float> d2 = solver->getVelocities();
    for (int i = 0; i < v.size(); ++i) {
        v.set(i, Vector2(d2[2*map[i]+0], d2[2*map[i]+1]));
    }
    return v;
}

PoolVector2Array ElasticBody2D::get_displacements() const {
    PoolVector2Array d;
    d.resize(nodes.size());
    std::vector<float> d1 = solver->getDisplacements();
    for (int i = 0; i < d.size(); ++i) {
        d.set(i, Vector2(d1[2*map[i]+0], d1[2*map[i]+1]));
    }
    return d;
}

String ElasticBody2D::get_configuration_warning() const {
    if (!Object::cast_to<Polygon2D>(get_parent()) && !Object::cast_to<CollisionPolygon2D>(get_parent())) {
        return TTR("Parent is neither Polygon2D nor CollisionPolygon2D!");
    }

	return String();
}

void ElasticBody2D::_bind_methods() {
    ClassDB::bind_method(D_METHOD("preprocess"), &ElasticBody2D::preprocess);
    ClassDB::bind_method(D_METHOD("deform"), &ElasticBody2D::deform);
    ClassDB::bind_method(D_METHOD("free_motion"), &ElasticBody2D::free_motion);
    ClassDB::bind_method(D_METHOD("set_poisson_ratio", "poisson_ratio"), &ElasticBody2D::set_poisson_ratio);
    ClassDB::bind_method(D_METHOD("get_poisson_ratio"), &ElasticBody2D::get_poisson_ratio);
    ClassDB::bind_method(D_METHOD("set_young_modulus", "young_modulus"), &ElasticBody2D::set_young_modulus);
    ClassDB::bind_method(D_METHOD("get_young_modulus"), &ElasticBody2D::get_young_modulus);
    ClassDB::bind_method(D_METHOD("set_friction", "friction"), &ElasticBody2D::set_friction);
    ClassDB::bind_method(D_METHOD("get_friction"), &ElasticBody2D::get_friction);
    ClassDB::bind_method(D_METHOD("set_min_angle", "angle"), &ElasticBody2D::set_min_angle);
    ClassDB::bind_method(D_METHOD("get_min_angle"), &ElasticBody2D::get_min_angle);
    ClassDB::bind_method(D_METHOD("set_method", "method"), &ElasticBody2D::set_method);
    ClassDB::bind_method(D_METHOD("get_method"), &ElasticBody2D::get_method);
    ClassDB::bind_method(D_METHOD("set_fixed_delta", "fixed"), &ElasticBody2D::set_fixed_delta);
    ClassDB::bind_method(D_METHOD("is_fixed_delta"), &ElasticBody2D::is_fixed_delta);
    ClassDB::bind_method(D_METHOD("set_forces", "forces"), &ElasticBody2D::set_forces);
    ClassDB::bind_method(D_METHOD("get_forces"), &ElasticBody2D::get_forces);
    ClassDB::bind_method(D_METHOD("get_velocities"), &ElasticBody2D::get_velocities);
    ClassDB::bind_method(D_METHOD("get_displacements"), &ElasticBody2D::get_displacements);

    ADD_PROPERTY(PropertyInfo(Variant::REAL, "poisson_ratio"), "set_poisson_ratio", "get_poisson_ratio");
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "young_modulus"), "set_young_modulus", "get_young_modulus");
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "friction"), "set_friction", "get_friction");
    ADD_PROPERTY(PropertyInfo(Variant::REAL, "min_angle"), "set_min_angle", "get_min_angle");
    ADD_PROPERTY(PropertyInfo(Variant::POOL_VECTOR2_ARRAY, "forces"), "set_forces", "get_forces");
    ADD_PROPERTY(PropertyInfo(Variant::INT, "method", PROPERTY_HINT_ENUM, "Explcit Euler,Modified Explicit Euler,Improved Euler,Runge Kutta 3,Runge Kutta 4,Implicit Euler"), "set_method", "get_method");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "fixed_delta"), "set_fixed_delta", "is_fixed_delta");

    BIND_ENUM_CONSTANT(METHOD_EXPLICIT_EULER);
	BIND_ENUM_CONSTANT(METHOD_MODIFIED_EXPLICIT_EULER);
	BIND_ENUM_CONSTANT(METHOD_IMPROVED_EULER);
    BIND_ENUM_CONSTANT(METHOD_RUNGE_KUTTA_3);
	BIND_ENUM_CONSTANT(METHOD_RUNGE_KUTTA_4);
	BIND_ENUM_CONSTANT(METHOD_IMPLICIT_EULER);
}