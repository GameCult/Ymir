// Particle chains and their attachments: Ymir's own position-based solver, stepped by ymir_box3d_soa_step
// after Box3D has stepped the rigid bodies. This module knows nothing of Box3D: ymir_box3d_soa.c is the only
// Box3D caller. It fills one proxy per body (pose, velocity, shape, inverse mass and inertia) before the step,
// the particle phase moves those proxies by the constraints it solves, and the impulse each body accumulated
// comes back in the proxy for soa.c to apply. See docs/soa-world.md.
//
// Determinism: single thread; chains are Gauss-Seidel in link order; the one Jacobi pass (particle-particle
// contact) is gather-form (each particle writes only its own correction, neighbours read in cell then index
// order from a stable counting-sort grid); proxy impulses are summed per body in constraint order. Compiled
// with -ffp-contract=off (/fp:precise on MSVC) so no fused multiply-add changes a byte.

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

enum
{
    YMIR_CHAIN_LIVE = 0,
    YMIR_CHAIN_REMOVED = 1 // a destroyed chain's lane: reusable only once a step has passed
};

enum
{
    YMIR_ATTACH_PARTICLE_BODY = 0,     // particle to a point fixed in a body (its lane)
    YMIR_ATTACH_PARTICLE_STATIC = 1,   // particle to a world point
    YMIR_ATTACH_PARTICLE_PARTICLE = 2, // particle to another particle (its lane in `body`)
    YMIR_ATTACH_REMOVED = 3
};

enum
{
    YMIR_PROXY_NONE = 0, // no shape (an anchor): attachments reach it, particles never touch it
    YMIR_PROXY_SPHERE = 1,
    YMIR_PROXY_CAPSULE = 2, // segment along local Y
    YMIR_PROXY_BOX = 3
};

// A chain to create: `count` >= 2 particles at `positions` (xyz interleaved) with `masses` (kg, 0 = immovable).
// Link and bend rest lengths are the creation distances.
typedef struct ymir_chain_desc
{
    uint64_t stable_id;
    int32_t count;
    uint32_t reserved;
    const float* positions;
    const float* masses;
    float radius;
    float stretch_compliance; // m/N, >= 0
    float bend_compliance;    // m/N, >= 0
    float friction;           // >= 0
    uint64_t material_id;
} ymir_chain_desc;

// An attachment of `particle`. `body` is the body lane (particle-body), the other particle's lane
// (particle-particle) or -1. `point` is in the body's frame (particle-body) or the world (particle-static).
typedef struct ymir_attachment_desc
{
    uint32_t kind;
    int32_t particle;
    int32_t body;
    float point[3];
    float compliance; // m/N, >= 0
    uint32_t reserved;
} ymir_attachment_desc;

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
    int32_t particle_capacity, chain_capacity, attachment_capacity, body_capacity;
    int32_t particle_count, chain_count, attachment_count; // high-water marks
    int32_t body_count;                                    // bodies filled in `proxies` this step
    int32_t live_chains;
    ymir_particle_proxy* proxies;

    // particle lanes
    float *pos_x, *pos_y, *pos_z, *vel_x, *vel_y, *vel_z, *radius, *inv_mass;
    float* link_tension; // N, link (i, i+1) on particle i: mean over the substeps; a sensor
    int32_t* chain;      // chain lane, -1 for a free slot

    // chain lanes
    uint64_t* chain_stable_id;
    uint32_t* chain_kind;
    uint32_t* chain_generation;
    int32_t* chain_first;
    int32_t* chain_size;
    uint64_t* chain_removed_at;
    uint64_t* chain_material_id;
    float *chain_stretch, *chain_bend, *chain_friction;

    // attachment lanes
    uint32_t* att_kind;
    uint32_t* att_generation;
    int32_t* att_particle;
    int32_t* att_body;
    float *att_point_x, *att_point_y, *att_point_z, *att_compliance;
    float *att_force_x, *att_force_y, *att_force_z; // N, the force the attachment exerts on the chain (mean); a sensor
    uint64_t* att_removed_at;

    // solver scratch, allocated once at create
    float *pp_x, *pp_y, *pp_z, *dp_x, *dp_y, *dp_z;
    float *rest_link, *rest_bend, *rest_pos, *tension_acc, *struct_x, *struct_y, *struct_z;
    int32_t *skip_body, *cell_x, *cell_y, *cell_z, *cell_start, *cell_fill, *sorted;
    uint32_t table_mask;
    uint8_t* mark;
    void* arena;
} ymir_particles;

// Creates the state for the given capacities (all >= 0) and `body_capacity` proxies; NULL on failure.
ymir_particles* ymir_particles_create(int32_t particle_capacity, int32_t chain_capacity, int32_t attachment_capacity, int32_t body_capacity);
void ymir_particles_free(ymir_particles* p);

// Commands. `step_count` is the world's completed steps: a removed lane is reused once a step has passed.
// Every refusal happens before anything changes.
int32_t ymir_particles_create_chain(ymir_particles* p, const ymir_chain_desc* d, uint64_t step_count, int32_t* chain, uint32_t* generation);
int32_t ymir_particles_split_chain(ymir_particles* p, int32_t chain, uint32_t generation, int32_t link, uint64_t tail_stable_id, uint64_t step_count, int32_t* tail, uint32_t* tail_generation);
int32_t ymir_particles_destroy_chain(ymir_particles* p, int32_t chain, uint32_t generation, uint64_t step_count);
int32_t ymir_particles_attach(ymir_particles* p, const ymir_attachment_desc* d, int32_t body_count, uint64_t step_count, int32_t* attachment, uint32_t* generation);
int32_t ymir_particles_detach(ymir_particles* p, int32_t attachment, uint32_t generation, uint64_t step_count);
int32_t ymir_particles_free_particle_slots(const ymir_particles* p, uint64_t step_count); // the largest free run
int32_t ymir_particles_free_chain_slots(const ymir_particles* p, uint64_t step_count);
int32_t ymir_particles_free_attachment_slots(const ymir_particles* p, uint64_t step_count);

// The particle phase runs inside the rigid solver's substeps: begin once per tick (after the caller filled the
// proxies' shape, size, inverse mass and inertia), substep once per rigid substep (after the caller refreshed each
// proxy's pose and velocity; it then applies each proxy's sub_impulse before positions integrate), end once.
int32_t ymir_particles_begin(ymir_particles* p);
void ymir_particles_substep(ymir_particles* p, float h, float gx, float gy, float gz);
void ymir_particles_end(ymir_particles* p, int32_t substeps);

#endif
