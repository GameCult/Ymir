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
// contact) accumulates and applies after the walk (each pair once, from the lower of its two cells, neighbours read in cell then index order
// from a stable counting-sort grid, a pair weighted the same at both ends); proxy impulses are summed per body
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
    YMIR_PARTICLES_OUT_OF_MEMORY = 4,
    YMIR_PARTICLES_REFERENCED = 6 // a body that a live attachment or sensor names cannot be released
};

// Phase word: bits 0-19 are the group; the three flags below are the only other bits.
enum
{
    YMIR_PHASE_GROUP_MASK = 0xFFFFFu,
    YMIR_PHASE_SELF_COLLIDE = 1u << 20, // particles of one group collide (both must set it)
    YMIR_PHASE_SELF_FILTER = 1u << 21,  // ... except pairs closer than 1.5 (ri + rj) in the rest pose (either sets it)
    YMIR_PHASE_FLUID = 1u << 22,        // a fluid particle: number density, not contact, with other fluid; contact with solids. Group and
                                        // the SELF_* flags are ignored between two fluid particles (all fluid shares one density field)
    YMIR_PHASE_VALID = YMIR_PHASE_GROUP_MASK | YMIR_PHASE_SELF_COLLIDE | YMIR_PHASE_SELF_FILTER | YMIR_PHASE_FLUID
};

// Lane sets a command names (a type for remove, a set for free_slots).
enum
{
    YMIR_LANES_PARTICLE = 0,
    YMIR_LANES_DISTANCE = 1,
    YMIR_LANES_BEND = 2,
    YMIR_LANES_ATTACHMENT = 3,
    YMIR_LANES_WET = 4,   // a wet patch: settled liquid on a body or a solid particle
    YMIR_LANES_SENSOR = 5 // a liquid sensor: a sphere fixed in a body that reads the liquid volume inside it
};

enum
{
    YMIR_ATTACH_BODY = 0,   // particle to a point fixed in a body (its lane)
    YMIR_ATTACH_STATIC = 1, // particle to a world point
    YMIR_ATTACH_REMOVED = 2
};

enum
{
    YMIR_ATTACH_FLAG_TETHER = 1u // bounds each particle by the rest path to the nearest point of its two nearest islands of tether attachments
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
    int32_t particles, distances, bends, attachments, tethers, materials, fluid, wets, sensors; // fluid: the most live particles with the FLUID phase; wets: the most wet patches; sensors: the most liquid sensors
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

// A liquid sensor to create: a sphere of `radius` m (> 0) centred on `point` in the frame of body lane `body`.
typedef struct ymir_sensor_desc
{
    int32_t body;
    float point[3];
    float radius;
} ymir_sensor_desc;

typedef struct ymir_material_desc
{
    float friction;  // >= 0
    float viscosity; // >= 0, XSPH c (dimensionless): a fluid pair's velocities relax toward each other by c W / n0 per substep; mean of the pair's two
    float wet_friction; // >= 0: what settled liquid of this material does to friction where it lands; stored for the friction cut, read by nothing here
    uint32_t pad;
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
    int32_t body_capacity, fluid_capacity; // fluid_capacity: the most live fluid particles; it sizes the neighbour arena
    int32_t particle_count, distance_count, bend_count, attachment_count; // high-water marks
    int32_t tether_count;                                                 // derived: the live tethers
    int32_t body_count;                                                   // bodies filled in `proxies` this step
    int32_t live_particles;
    int32_t fluid_rings;    // derived by fluid_density: rings of kernel cells (4 rfmax) that hold a kernel pair, 0 without fluid
    int32_t fluid_overflow; // kernel neighbours past a fluid particle's list in the last step, summed over substeps: a sensor
    int32_t wet_capacity, wet_count; // wet patches: the capacity and the high-water mark
    int32_t sensor_capacity, sensor_count; // liquid sensors: the capacity and the high-water mark
    int32_t live_fluid; // derived: the live particles with the FLUID phase bit; the create and destroy commands are its only writers
    ymir_particle_proxy* proxies;

    // particle lanes; a free lane reads live 0 and every other lane 0 except generation and removed_at
    float *pos_x, *pos_y, *pos_z, *vel_x, *vel_y, *vel_z, *radius, *inv_mass;
    float *rest_x, *rest_y, *rest_z;
    uint32_t *phase, *material, *particle_live, *particle_generation;
    uint64_t* particle_removed_at;
    int32_t* component; // derived: the lowest lane of the particle's distance-connected component
    int32_t *nbr, *nbr_start, *nbr_count; // scratch: the fluid rows' kernel neighbours in walk order, packed (CSR) in an arena of fluid_capacity * YMIR_FLUID_NEIGHBOURS; row i holds nbr_count[i] from nbr_start[i], this substep's
    int32_t *support_body, *support_particle; // sensor: what a fluid particle touched in the last substep, the highest body lane / the first solid lane its row listed touching it; -1 for none, and always -1 for a solid or free lane
    float* density;     // sensor: a fluid particle's number density over its rest density n_i / n0 in the last substep it ran, else 0

    // wet lanes: a patch of settled liquid on one support, a body (wet_body, its point in the body frame) or a solid particle (wet_particle, point 0);
    // one of the two is -1, and a free lane holds -1 in both. Written by settle, remove and destroy_particles only; the step never touches them.
    int32_t *wet_body, *wet_particle;
    float *wet_x, *wet_y, *wet_z, *wet_radius, *wet_volume; // wet_volume m^3; wet_radius = sqrt(volume / (pi YMIR_WET_FILM))
    uint32_t *wet_material, *wet_live, *wet_generation;
    uint64_t* wet_removed_at;
    int32_t *wet_order, *wet_body_start, *wet_body_count; // derived lookup index (rederive_wet): live body patches by body then centre x; each body's slice
    float* wet_body_box; // derived, 7 per body: min xyz and max xyz of its discs' boxes, then its largest radius
    float* particle_wet_mu; // derived, per particle lane: the wet friction of the lowest live patch on that solid particle, +inf when dry (rederive_wet)
    float* wet_volume_body; // derived, per body lane: the sum of the live patches on it in wet lane order (rederive_wet, at command time only)
    int32_t* wu_code; // settle's undo log, one entry per distinct drop: a patch lane (merged: its old lanes below) or -(lane + 1) (created)
    float *wu_x, *wu_y, *wu_z, *wu_volume;

    // sensor lanes: a sphere fixed in a body; a free lane reads live 0, body -1 and 0 elsewhere except generation and removed_at. Written by create_sensors and
    // remove; sensor_liquid_volume and sensor_wet_volume are written by end alone, at the step's end (m^3): the sum of (2 r)^3 over the live fluid particles whose centre is inside,
    // and the sum of the volumes of the live wet patches whose centre is inside, on any body or solid particle
    int32_t* sensor_body;
    float *sensor_x, *sensor_y, *sensor_z, *sensor_radius, *sensor_liquid_volume, *sensor_wet_volume;
    uint32_t *sensor_live, *sensor_generation;
    uint64_t* sensor_removed_at;

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

    // tether lanes: derived: grouped by island in lane order (an island: tether particles joined by live distances, named by its
    // lowest tether attachment), settle order within; per island one for each particle that is not a tether particle and holds the island
    // among its two nearest, to the attachment point it is nearest; then per attachment pair with a body among them a route of
    // booking lanes (attachment -1, edge only) along their rest path and the pair tether; parents come first;
    // other >= 0 makes the tether a bound between two attachment points (edge -1) instead of a particle and a point
    int32_t *tether_att, *tether_particle, *tether_parent, *tether_edge, *tether_other;
    float* tether_bound;

    // material table
    float *mat_friction, *mat_viscosity, *mat_wet_friction;
    uint64_t* mat_id;

    // solver scratch, allocated once at create
    float *pp_x, *pp_y, *pp_z, *dp_x, *dp_y, *dp_z, *dist_acc, *tacc_x, *tacc_y, *tacc_z, *lambda;
    float fluid_k; // n0(h) = fluid_k / h^3: the Poly6 sum over the simple-cubic lattice of spacing h / 2, a create-time constant
    int32_t *lane_list, *cell_x, *cell_y, *cell_z, *cell_start, *cell_fill, *sorted, *contacts;
    float *sx, *sy, *sz, *sw; // the fluid grid's lanes in sorted order (predicted position, inverse mass): scratch, only fluid_density reads them
    uint32_t* sphase;         // the sorted lanes' phase
    int32_t *adj_start, *adj_edge, *heap_node, *heap_edge, *via, *owner, *tether_of, *queue, *comp_size, *isle, *near_isle;
    float *heap_key, *path, *near_path;
    uint8_t* done;
    uint32_t table_mask;
    void* arena;
} ymir_particles;

// Creates the state for the capacities (all >= 0) and `body_capacity` proxies; NULL on failure.
ymir_particles* ymir_particles_create(const ymir_particle_capacities* capacities, int32_t body_capacity);
void ymir_particles_free(ymir_particles* p);

// The friction on body lane `body` at body-frame point (x, y, z): the wet friction of the lowest live patch whose disc holds it, never above mu.
float ymir_particles_wet_mu(const ymir_particles* p, int32_t body, float x, float y, float z, float mu);

// Commands. `step_count` is the world's completed steps: a removed lane is reused once a step has passed, lowest
// lane first. Every refusal happens before anything changes; a command that would need more tethers than the
// capacity is refused whole (tether capacity: up to two per particle reached, plus the pair routes). Every topology command ends in the one rederive_topology.
int32_t ymir_particles_create_particles(ymir_particles* p, int32_t n, const ymir_particle_desc* d, uint64_t step_count, ymir_handle* out);
int32_t ymir_particles_destroy_particles(ymir_particles* p, int32_t n, const ymir_handle* h, uint64_t step_count);
int32_t ymir_particles_create_distances(ymir_particles* p, int32_t n, const ymir_distance_desc* d, uint64_t step_count, ymir_handle* out);
int32_t ymir_particles_create_bends(ymir_particles* p, int32_t n, const ymir_bend_desc* d, uint64_t step_count, ymir_handle* out);
int32_t ymir_particles_attach(ymir_particles* p, const ymir_attachment_desc* d, int32_t body_count, uint64_t step_count, ymir_handle* out);
int32_t ymir_particles_remove(ymir_particles* p, uint32_t type, int32_t n, const ymir_handle* h, uint64_t step_count);
// Settles the named fluid particles into wet patches and destroys them. Each reads its support from the last step (support_body, else
// support_particle) and its contact point from the proxies the caller has just filled; a drop's volume is (2 r)^3. Refuses whole
// (INVALID_ARGUMENT) a dead or non-fluid particle or one with no support, FULL when a drop finds no lane and no patch to merge into, before any
// change. `out` (NULL or n handles) receives the patch each drop landed in.
int32_t ymir_particles_settle(ymir_particles* p, int32_t n, const ymir_handle* h, uint64_t step_count, ymir_handle* out);
// Creates liquid sensors in the lowest free lanes. Refuses whole (INVALID_ARGUMENT) a body outside [0, body_count), a radius that is not finite and > 0, or a
// non-finite point; FULL when there are not n free lanes; nothing changes. A sensor only reads: its volume moves nothing.
int32_t ymir_particles_create_sensors(ymir_particles* p, int32_t n, const ymir_sensor_desc* d, int32_t body_count, uint64_t step_count, ymir_handle* out);
// Frees what the particle layer holds on body lane `body` so the body can be removed: REFERENCED, changing nothing, while a live body
// attachment or a live sensor names it (the caller removes those first); otherwise every live wet patch on it is removed (one rederive_wet) and
// `patches` and `volume` (either may be NULL) receive their count and summed volume (m^3). No rigid-body call.
int32_t ymir_particles_release_body(ymir_particles* p, int32_t body, uint64_t step_count, uint32_t* patches, float* volume);
int32_t ymir_particles_set_material(ymir_particles* p, int32_t index, const ymir_material_desc* d);
int32_t ymir_particles_free_slots(const ymir_particles* p, uint32_t lane_set, uint64_t step_count);

// The particle phase runs inside the rigid solver's substeps: begin once per tick (after the caller filled the
// proxies' shape, size, inverse mass and inertia), substep once per rigid substep (after the caller refreshed each
// proxy's pose and velocity; it then applies each proxy's sub_impulse before positions integrate), end once.
int32_t ymir_particles_begin(ymir_particles* p);
void ymir_particles_substep(ymir_particles* p, float h, float gx, float gy, float gz);
void ymir_particles_end(ymir_particles* p, int32_t substeps);

#endif
