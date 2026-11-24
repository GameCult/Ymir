// Particles and the constraints between them: Ymir's own position-based solver, stepped inside Box3D's substeps
// (see ymir_box3d_soa.c, the only Box3D caller). This module knows nothing of Box3D. soa.c fills one proxy per body
// (pose, velocity, shape, inverse mass and inertia) before each substep, the solver moves those proxies by the
// constraints it solves, and the impulse each body accumulated comes back in the proxy for soa.c to apply.
// See docs/soa-world.md.
//
// There is no object above the lanes: particles are one lane set with a phase word, and every constraint type
// (distance, bend, attachment) is a lane set of its own. A thing made of particles is whatever constraints a
// caller emitted. Tethers and components are derived from the distance graph by rederive_topology, which every
// topology command ends in and which is their only writer.
//
// Determinism: single thread; constraints are Gauss-Seidel in lane order; the one Jacobi pass (particle-particle
// contact) is gather-form (each particle writes only its own correction, neighbours read in cell then index order
// from a stable counting-sort grid, a pair weighted the same from both ends); proxy impulses are summed per body
// in lane order; heap and breadth-first orders break ties by lane. Compiled with -ffp-contract=off (/fp:precise
// on MSVC) so no fused multiply-add changes a byte.
//
// Licence note: the formulas come from the papers (Kelager 2010 bend, Kim 2012 tethers, Macklin 2014, 2016, 2019).
// No code of PhysX 5.6.0 or any other solver is adapted here.

#ifndef YMIR_PARTICLES_H
#define YMIR_PARTICLES_H

#include <stdint.h>

enum
{
    YMIR_PARTICLES_OK = 0,
    YMIR_PARTICLES_INVALID_ARGUMENT = 1,
    YMIR_PARTICLES_DUPLICATE_ID = 2,
    YMIR_PARTICLES_FULL = 3,
    YMIR_PARTICLES_OUT_OF_MEMORY = 4
};

// Phase word: bits 0-19 are the group; the two flags below are the only other bits.
enum
{
    YMIR_PHASE_GROUP_MASK = 0xFFFFFu,
    YMIR_PHASE_SELF_COLLIDE = 1u << 20, // particles of one group collide (both must set it)
    YMIR_PHASE_SELF_FILTER = 1u << 21,  // ... except pairs closer than 1.5 (ri + rj) in the rest pose (either sets it)
    YMIR_PHASE_VALID = YMIR_PHASE_GROUP_MASK | YMIR_PHASE_SELF_COLLIDE | YMIR_PHASE_SELF_FILTER
};

// Lane sets a command names (a type for remove, a set for free_slots).
enum
{
    YMIR_LANES_PARTICLE = 0,
    YMIR_LANES_DISTANCE = 1,
    YMIR_LANES_BEND = 2,
    YMIR_LANES_ATTACHMENT = 3
};

enum
{
    YMIR_ATTACH_BODY = 0,   // particle to a point fixed in a body (its lane)
    YMIR_ATTACH_STATIC = 1, // particle to a world point
    YMIR_ATTACH_REMOVED = 2
};

enum
{
    YMIR_ATTACH_FLAG_TETHER = 1u // the attachment bounds every particle its component reaches by the rest path
};

enum
{
    YMIR_PROXY_NONE = 0, // no shape (an anchor): attachments reach it, particles never touch it
    YMIR_PROXY_SPHERE = 1,
    YMIR_PROXY_CAPSULE = 2, // segment along local Y
    YMIR_PROXY_BOX = 3
};

// A lane and the generation it had when the handle was made.
typedef struct ymir_handle
{
    int32_t lane;
    uint32_t generation;
} ymir_handle;

typedef struct ymir_particle_capacities
{
    int32_t particles, distances, bends, attachments, tethers, materials;
} ymir_particle_capacities;

// A particle to create. `mass` kg (0 = immovable); `velocity` m/s; `rest` is its place in the rest pose (rest lengths, bends and
// the self-collision filter read it); `material` indexes the material table.
typedef struct ymir_particle_desc
{
    float position[3];
    float rest[3];
    float velocity[3];
    float mass;
    float radius;
    uint32_t phase;
    uint32_t material;
} ymir_particle_desc;

// A distance constraint. `rest` < 0 takes the rest-pose length. `compliance` m/N, >= 0.
typedef struct ymir_distance_desc
{
    ymir_handle a, b;
    float rest;
    float compliance;
} ymir_distance_desc;

// A bend over two distance constraints that share one particle. `compliance` < 0 switches the soft pass off;
// `max_angle` (radians, the deflection from straight) < 0 switches the hard limit off.
typedef struct ymir_bend_desc
{
    ymir_handle e1, e2;
    float compliance;
    float max_angle;
} ymir_bend_desc;

// An attachment of `particle`. `body` is the body lane (body kind) or -1. `point` is in the body's frame (body)
// or the world (static).
typedef struct ymir_attachment_desc
{
    uint32_t kind;
    uint32_t flags;
    ymir_handle particle;
    int32_t body;
    float point[3];
    float compliance; // m/N, >= 0
    uint32_t reserved;
} ymir_attachment_desc;

typedef struct ymir_material_desc
{
    float friction; // >= 0
    uint32_t reserved;
    uint64_t material_id;
} ymir_material_desc;

// A body as the particle phase sees it. Filled by the caller before a step; `impulse` and `angular_impulse`
// (N s, world, about the body position) are the totals the particle phase delivered, written by it.
typedef struct ymir_particle_proxy
{
    uint32_t shape;
    float size[3];
    float position[3];
    float rotation[4]; // x y z w
    float velocity[3];
    float angular_velocity[3];
    float inv_mass;       // 0 for static, kinematic and anchor bodies
    float inv_inertia[6]; // world inverse inertia, symmetric: xx xy xz yy yz zz
    float impulse[3];         // N s, world: the tick's total, the sum of every substep's
    float angular_impulse[3];
    float sub_impulse[3];     // N s: this substep's, applied by the caller before the body's positions integrate
    float sub_angular[3];
} ymir_particle_proxy;

typedef struct ymir_particles
{
    int32_t particle_capacity, distance_capacity, bend_capacity, attachment_capacity, tether_capacity, material_capacity;
    int32_t body_capacity;
    int32_t particle_count, distance_count, bend_count, attachment_count; // high-water marks
    int32_t tether_count;                                                 // derived: the live tethers
    int32_t body_count;                                                   // bodies filled in `proxies` this step
    int32_t live_particles;
    ymir_particle_proxy* proxies;

    // particle lanes; a free lane reads live 0 and every other lane 0 except generation and removed_at
    float *pos_x, *pos_y, *pos_z, *vel_x, *vel_y, *vel_z, *radius, *inv_mass;
    float *rest_x, *rest_y, *rest_z;
    uint32_t *phase, *material, *particle_live, *particle_generation;
    uint64_t* particle_removed_at;
    int32_t* component; // derived: the lowest lane of the particle's distance-connected component

    // distance lanes
    int32_t *dist_a, *dist_b;
    float *dist_rest, *dist_compliance;
    float* dist_tension; // N, the load the constraint carries, mean over the substeps: a sensor
    uint32_t *dist_live, *dist_generation;
    uint64_t* dist_removed_at;

    // bend lanes: e1 and e2 are distance lanes; a, b, c the particles (b shared)
    int32_t *bend_e1, *bend_e2, *bend_a, *bend_b, *bend_c;
    float *bend_rest, *bend_limit, *bend_compliance; // offsets of b from the centroid; limit < 0 is off
    uint32_t *bend_live, *bend_generation;
    uint64_t* bend_removed_at;

    // attachment lanes
    uint32_t *att_kind, *att_flags, *att_generation;
    int32_t *att_particle, *att_body;
    float *att_point_x, *att_point_y, *att_point_z, *att_compliance;
    float *att_force_x, *att_force_y, *att_force_z; // N, the force the attachment exerts on its particles (mean): a sensor
    uint64_t* att_removed_at;

    // tether lanes: derived, per attachment its settled particles in order and then its pairs, parents before children;
    // other >= 0 makes the tether a bound between two attachment points (edge -1) instead of a particle and a point
    int32_t *tether_att, *tether_particle, *tether_parent, *tether_edge, *tether_other;
    float* tether_bound;

    // material table
    float* mat_friction;
    uint64_t* mat_id;

    // solver scratch, allocated once at create
    float *pp_x, *pp_y, *pp_z, *dp_x, *dp_y, *dp_z, *dist_acc, *tacc_x, *tacc_y, *tacc_z;
    int32_t *lane_list, *skip_body, *cell_x, *cell_y, *cell_z, *cell_start, *cell_fill, *sorted, *contacts;
    int32_t *adj_start, *adj_edge, *heap_node, *heap_edge, *prev_edge, *tether_of, *queue, *comp_size;
    float *heap_key, *path;
    uint8_t* done;
    uint32_t table_mask;
    void* arena;
} ymir_particles;

// Creates the state for the capacities (all >= 0) and `body_capacity` proxies; NULL on failure.
ymir_particles* ymir_particles_create(const ymir_particle_capacities* capacities, int32_t body_capacity);
void ymir_particles_free(ymir_particles* p);

// Commands. `step_count` is the world's completed steps: a removed lane is reused once a step has passed, lowest
// lane first. Every refusal happens before anything changes; a command that would need more tethers than the
// capacity is refused whole. Every topology command ends in the one rederive_topology.
int32_t ymir_particles_create_particles(ymir_particles* p, int32_t n, const ymir_particle_desc* d, uint64_t step_count, ymir_handle* out);
int32_t ymir_particles_destroy_particles(ymir_particles* p, int32_t n, const ymir_handle* h, uint64_t step_count);
int32_t ymir_particles_create_distances(ymir_particles* p, int32_t n, const ymir_distance_desc* d, uint64_t step_count, ymir_handle* out);
int32_t ymir_particles_create_bends(ymir_particles* p, int32_t n, const ymir_bend_desc* d, uint64_t step_count, ymir_handle* out);
int32_t ymir_particles_attach(ymir_particles* p, const ymir_attachment_desc* d, int32_t body_count, uint64_t step_count, ymir_handle* out);
int32_t ymir_particles_remove(ymir_particles* p, uint32_t type, int32_t n, const ymir_handle* h, uint64_t step_count);
int32_t ymir_particles_set_material(ymir_particles* p, int32_t index, const ymir_material_desc* d);
int32_t ymir_particles_free_slots(const ymir_particles* p, uint32_t lane_set, uint64_t step_count);

// The particle phase runs inside the rigid solver's substeps: begin once per tick (after the caller filled the
// proxies' shape, size, inverse mass and inertia), substep once per rigid substep (after the caller refreshed each
// proxy's pose and velocity; it then applies each proxy's sub_impulse before positions integrate), end once.
int32_t ymir_particles_begin(ymir_particles* p);
void ymir_particles_substep(ymir_particles* p, float h, float gx, float gy, float gz);
void ymir_particles_end(ymir_particles* p, int32_t substeps);

#endif
