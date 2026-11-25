// Particles and the constraint graph between them: XPBD with small steps (Macklin 2016, 2019), bending by the
// triangle-centroid form (Kelager 2010), long-range tethers derived from the graph (Kim 2012), contact by
// pair-symmetric position projection (Macklin 2014), and two-way coupling to rigid bodies through proxies with
// real mass (Mueller 2007, Detailed Rigid XPBD 2020). The file calls no Box3D and spawns nothing: one thread,
// every sum in a fixed order. See ymir_particles.h.

#include "ymir_particles.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

typedef struct v3
{
    float x, y, z;
} v3;

static v3 vadd(v3 a, v3 b) { return (v3){a.x + b.x, a.y + b.y, a.z + b.z}; }
static v3 vsub(v3 a, v3 b) { return (v3){a.x - b.x, a.y - b.y, a.z - b.z}; }
static v3 vmul(v3 a, float s) { return (v3){a.x * s, a.y * s, a.z * s}; }
static float vdot(v3 a, v3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static v3 vcross(v3 a, v3 b) { return (v3){a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
static float vlen(v3 a) { return sqrtf(vdot(a, a)); }
static float vmin(float a, float b) { return a < b ? a : b; }
static float vmax(float a, float b) { return a > b ? a : b; }
static float vclamp(float a, float lo, float hi) { return a < lo ? lo : a > hi ? hi : a; }

static v3 qrot(const float* q, v3 v)
{
    v3 u = {q[0], q[1], q[2]};
    v3 t = vmul(vcross(u, v), 2.0f);
    return vadd(vadd(v, vmul(t, q[3])), vcross(u, t));
}

static v3 qrot_inv(const float* q, v3 v)
{
    float c[4] = {-q[0], -q[1], -q[2], q[3]};
    return qrot(c, v);
}

static v3 mat_mul(const float* m, v3 v) // symmetric xx xy xz yy yz zz
{
    return (v3){m[0] * v.x + m[1] * v.y + m[2] * v.z, m[1] * v.x + m[3] * v.y + m[4] * v.z, m[2] * v.x + m[4] * v.y + m[5] * v.z};
}

static int finite3(const float* v)
{
    return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

static v3 P(const ymir_particles* p, const float* x, const float* y, const float* z, int i)
{
    (void)p;
    return (v3){x[i], y[i], z[i]};
}

static void put(float* x, float* y, float* z, int i, v3 v)
{
    x[i] = v.x;
    y[i] = v.y;
    z[i] = v.z;
}


// A particle is held to the nearest point of each of its two nearest tether islands (Kim, Chentanez, Mueller 2012 sec. 3.4);
// Kim's typical four were measured on the 20-pin 32x32 sheet: 1.66% worst edge against 1.72%, for twice the lanes.
#define YMIR_TETHER_ISLANDS 2

// Kernels (Mueller, Charypar, Gross 2003 as used by Macklin and Mueller 2013): Poly6 W(r, h) = 315 / (64 pi h^9) (h^2 - r^2)^3 for the
// density, Spiky grad W = -45 / (pi h^6) (h - r)^2 r_hat for the gradient.
#define YMIR_POLY6 (315.0f / (64.0f * 3.14159265358979f))
#define YMIR_SPIKY (-45.0f / 3.14159265358979f)

// Regularisation of the density constraint (Macklin and Mueller 2013 eq. 11), added to sum |grad C|^2 with the gradients over n0
// in 1/m^2.
#define YMIR_PBF_EPS 1.0e-4f

// Neighbour-list entries a fluid lane adds to the arena (CSR, packed): about 33 kernel neighbours at rest (4 r reach), about 45 at the column's 1.34 compression.
#define YMIR_FLUID_NEIGHBOURS 96

// The thickness of a settled film in metres: a patch of volume V is a disc of radius sqrt(V / (pi film)). One constant for every liquid.
#define YMIR_WET_FILM 1.0e-3f

// Capacity of the stack buffer a fluid row gathers its candidates into before the pair arithmetic; hits past it count in fluid_overflow.
#define YMIR_FLUID_CANDIDATES 128

// ---- memory: one block, laid out once at create -------------------------------------------------------------

static size_t layout(ymir_particles* p, unsigned char* base)
{
    size_t off = 0;
    int64_t np = p->particle_capacity, nd = p->distance_capacity, nb = p->bend_capacity, na = p->attachment_capacity,
            nt = p->tether_capacity, nm = p->material_capacity, nx = p->body_capacity, nw = p->wet_capacity, nf = p->fluid_capacity, ns = p->sensor_capacity;
    int64_t heap = 2 * nd + na + 1; // every directed edge relaxes once, and every attachment seeds one
    size_t table = (size_t)p->table_mask + 1;
#define CARVE(field, type, count)                                                                                      \
    do                                                                                                                 \
    {                                                                                                                  \
        off = (off + 15u) & ~(size_t)15u;                                                                              \
        if (base != NULL)                                                                                              \
            p->field = (type*)(base + off);                                                                            \
        off += sizeof(type) * (size_t)(count);                                                                         \
    } while (0)
    CARVE(proxies, ymir_particle_proxy, nx);
    CARVE(pos_x, float, np); CARVE(pos_y, float, np); CARVE(pos_z, float, np);
    CARVE(vel_x, float, np); CARVE(vel_y, float, np); CARVE(vel_z, float, np);
    CARVE(radius, float, np); CARVE(inv_mass, float, np);
    CARVE(rest_x, float, np); CARVE(rest_y, float, np); CARVE(rest_z, float, np);
    CARVE(phase, uint32_t, np); CARVE(material, uint32_t, np); CARVE(particle_live, uint32_t, np);
    CARVE(particle_generation, uint32_t, np); CARVE(particle_removed_at, uint64_t, np); CARVE(component, int32_t, np);
    CARVE(held_start, int32_t, np); CARVE(held_count, int32_t, np); CARVE(limb_body, int32_t, np); CARVE(held_body, int32_t, na);
    CARVE(density, float, np); CARVE(support_body, int32_t, np); CARVE(support_particle, int32_t, np);
    CARVE(nbr, int32_t, (int64_t)p->fluid_capacity * YMIR_FLUID_NEIGHBOURS); CARVE(nbr_start, int32_t, np); CARVE(nbr_count, int32_t, np);
    CARVE(wet_body, int32_t, nw); CARVE(wet_particle, int32_t, nw); CARVE(wet_x, float, nw); CARVE(wet_y, float, nw); CARVE(wet_z, float, nw);
    CARVE(wet_radius, float, nw); CARVE(wet_volume, float, nw); CARVE(wet_material, uint32_t, nw); CARVE(wet_live, uint32_t, nw);
    CARVE(wet_generation, uint32_t, nw); CARVE(wet_removed_at, uint64_t, nw); CARVE(wet_volume_body, float, nx); CARVE(particle_wet_mu, float, np);
    CARVE(wet_order, int32_t, nw); CARVE(wet_body_start, int32_t, nx); CARVE(wet_body_count, int32_t, nx); CARVE(wet_body_box, float, (int64_t)nx * 7);
    CARVE(sensor_body, int32_t, ns); CARVE(sensor_x, float, ns); CARVE(sensor_y, float, ns); CARVE(sensor_z, float, ns); CARVE(sensor_radius, float, ns);
    CARVE(sensor_liquid_volume, float, ns); CARVE(sensor_wet_volume, float, ns); CARVE(sensor_live, uint32_t, ns); CARVE(sensor_generation, uint32_t, ns); CARVE(sensor_removed_at, uint64_t, ns);
    CARVE(wu_code, int32_t, nf); CARVE(wu_x, float, nf); CARVE(wu_y, float, nf); CARVE(wu_z, float, nf); CARVE(wu_volume, float, nf);
    CARVE(dist_a, int32_t, nd); CARVE(dist_b, int32_t, nd); CARVE(dist_rest, float, nd); CARVE(dist_compliance, float, nd);
    CARVE(dist_tension, float, nd); CARVE(dist_live, uint32_t, nd); CARVE(dist_generation, uint32_t, nd);
    CARVE(dist_removed_at, uint64_t, nd);
    CARVE(bend_e1, int32_t, nb); CARVE(bend_e2, int32_t, nb); CARVE(bend_a, int32_t, nb); CARVE(bend_b, int32_t, nb);
    CARVE(bend_c, int32_t, nb); CARVE(bend_rest, float, nb); CARVE(bend_limit, float, nb); CARVE(bend_compliance, float, nb);
    CARVE(bend_live, uint32_t, nb); CARVE(bend_generation, uint32_t, nb); CARVE(bend_removed_at, uint64_t, nb);
    CARVE(att_kind, uint32_t, na); CARVE(att_flags, uint32_t, na); CARVE(att_generation, uint32_t, na);
    CARVE(att_particle, int32_t, na); CARVE(att_body, int32_t, na);
    CARVE(att_point_x, float, na); CARVE(att_point_y, float, na); CARVE(att_point_z, float, na);
    CARVE(att_compliance, float, na);
    CARVE(att_force_x, float, na); CARVE(att_force_y, float, na); CARVE(att_force_z, float, na);
    CARVE(att_removed_at, uint64_t, na);
    CARVE(tether_att, int32_t, nt); CARVE(tether_particle, int32_t, nt); CARVE(tether_parent, int32_t, nt);
    CARVE(tether_edge, int32_t, nt); CARVE(tether_other, int32_t, nt); CARVE(tether_bound, float, nt);
    CARVE(mat_friction, float, nm); CARVE(mat_viscosity, float, nm); CARVE(mat_wet_friction, float, nm); CARVE(mat_id, uint64_t, nm);
    CARVE(pp_x, float, np); CARVE(pp_y, float, np); CARVE(pp_z, float, np);
    CARVE(dp_x, float, np); CARVE(dp_y, float, np); CARVE(dp_z, float, np);
    CARVE(dist_acc, float, nd); CARVE(lambda, float, np);
    CARVE(tacc_x, float, nt); CARVE(tacc_y, float, nt); CARVE(tacc_z, float, nt);
    CARVE(lane_list, int32_t, nd); CARVE(cell_x, int32_t, np); CARVE(cell_y, int32_t, np); CARVE(cell_z, int32_t, np);
    CARVE(cell_start, int32_t, table + 1); CARVE(cell_fill, int32_t, table); CARVE(sorted, int32_t, np);
    CARVE(contacts, int32_t, np);
    CARVE(sx, float, np); CARVE(sy, float, np); CARVE(sz, float, np); CARVE(sw, float, np); CARVE(sphase, uint32_t, np);
    CARVE(adj_start, int32_t, np + 1); CARVE(adj_edge, int32_t, 2 * nd);
    CARVE(heap_key, float, heap); CARVE(heap_node, int32_t, heap); CARVE(heap_edge, int32_t, heap);
    CARVE(path, float, np); CARVE(via, int32_t, np); CARVE(owner, int32_t, np); CARVE(tether_of, int32_t, np);
    CARVE(queue, int32_t, np); CARVE(comp_size, int32_t, np); CARVE(done, uint8_t, np);
    CARVE(isle, int32_t, np); CARVE(near_isle, int32_t, YMIR_TETHER_ISLANDS * np); CARVE(near_path, float, YMIR_TETHER_ISLANDS * np);
#undef CARVE
    return off + 16;
}

ymir_particles* ymir_particles_create(const ymir_particle_capacities* c, int32_t body_capacity)
{
    if (c == NULL || c->particles < 0 || c->distances < 0 || c->bends < 0 || c->attachments < 0 || c->tethers < 0 ||
        c->materials < 0 || c->fluid < 0 || c->wets < 0 || c->sensors < 0 || body_capacity < 0)
    {
        return NULL;
    }
    ymir_particles* p = (ymir_particles*)calloc(1, sizeof(ymir_particles));
    if (p == NULL)
    {
        return NULL;
    }
    p->particle_capacity = c->particles;
    p->distance_capacity = c->distances;
    p->bend_capacity = c->bends;
    p->attachment_capacity = c->attachments;
    p->tether_capacity = c->tethers;
    p->material_capacity = c->materials;
    p->body_capacity = body_capacity;
    p->fluid_capacity = c->fluid;
    p->wet_capacity = c->wets;
    p->sensor_capacity = c->sensors;
    uint32_t table = 16;
    while (table < 2u * (uint32_t)c->particles)
    {
        table <<= 1;
    }
    p->table_mask = table - 1;
    size_t bytes = layout(p, NULL);
    p->arena = calloc(1, bytes);
    if (p->arena == NULL)
    {
        free(p);
        return NULL;
    }
    layout(p, (unsigned char*)p->arena);
    for (int32_t w = 0; w < p->wet_capacity; ++w)
    {
        p->wet_body[w] = p->wet_particle[w] = -1; // a free patch lane holds -1 on both supports
    }
    for (int32_t s = 0; s < p->sensor_capacity; ++s)
    {
        p->sensor_body[s] = -1; // a free sensor lane holds -1 on its body
    }
    for (int32_t i = 0; i < p->particle_capacity; ++i)
    {
        p->particle_wet_mu[i] = INFINITY; // dry
    }
    // n0 for h = 1: the Poly6 kernel summed over the simple-cubic lattice of spacing 1/2 within reach (self included); at h it is k / h^3
    float k = 0.0f;
    for (int a = -2; a <= 2; ++a)
    {
        for (int b = -2; b <= 2; ++b)
        {
            for (int e = -2; e <= 2; ++e)
            {
                float t = 1.0f - 0.25f * (float)(a * a + b * b + e * e);
                k += t > 0.0f ? t * t * t : 0.0f;
            }
        }
    }
    p->fluid_k = k * YMIR_POLY6;
    return p;
}

void ymir_particles_free(ymir_particles* p)
{
    if (p != NULL)
    {
        free(p->arena);
        free(p);
    }
}

// ---- lanes ---------------------------------------------------------------------------------------------------

static int lane_reusable(uint64_t step_count, uint64_t removed_at)
{
    return step_count > removed_at;
}

// The lowest free lane at or after `from`: a lane never used, or a dead one a step has passed since its removal.
// -1 when there is none. `state` is the lane set's live (or kind) lane and `dead` the value a dead lane holds.
static int32_t next_lane(const uint32_t* state, uint32_t dead, const uint64_t* removed_at, int32_t count, int32_t capacity,
                         int32_t from, uint64_t step_count)
{
    for (int32_t i = from; i < capacity; ++i)
    {
        if (i >= count || (state[i] == dead && lane_reusable(step_count, removed_at[i])))
        {
            return i;
        }
    }
    return -1;
}

static int32_t free_count(const uint32_t* state, uint32_t dead, const uint64_t* removed_at, int32_t count, int32_t capacity,
                          uint64_t step_count)
{
    int32_t n = capacity - count;
    for (int32_t i = 0; i < count; ++i)
    {
        n += state[i] == dead && lane_reusable(step_count, removed_at[i]);
    }
    return n;
}

int32_t ymir_particles_free_slots(const ymir_particles* p, uint32_t lane_set, uint64_t step_count)
{
    if (p == NULL)
    {
        return 0;
    }
    switch (lane_set)
    {
        case YMIR_LANES_PARTICLE:
            return free_count(p->particle_live, 0, p->particle_removed_at, p->particle_count, p->particle_capacity, step_count);
        case YMIR_LANES_DISTANCE:
            return free_count(p->dist_live, 0, p->dist_removed_at, p->distance_count, p->distance_capacity, step_count);
        case YMIR_LANES_BEND:
            return free_count(p->bend_live, 0, p->bend_removed_at, p->bend_count, p->bend_capacity, step_count);
        case YMIR_LANES_ATTACHMENT:
            return free_count(p->att_kind, YMIR_ATTACH_REMOVED, p->att_removed_at, p->attachment_count, p->attachment_capacity, step_count);
        case YMIR_LANES_WET:
            return free_count(p->wet_live, 0, p->wet_removed_at, p->wet_count, p->wet_capacity, step_count);
        case YMIR_LANES_SENSOR:
            return free_count(p->sensor_live, 0, p->sensor_removed_at, p->sensor_count, p->sensor_capacity, step_count);
        default:
            return 0;
    }
}

static int particle_ok(const ymir_particles* p, int32_t lane, uint32_t generation)
{
    return lane >= 0 && lane < p->particle_count && p->particle_live[lane] && p->particle_generation[lane] == generation;
}

static int distance_ok(const ymir_particles* p, int32_t lane, uint32_t generation)
{
    return lane >= 0 && lane < p->distance_count && p->dist_live[lane] && p->dist_generation[lane] == generation;
}

static int bend_ok(const ymir_particles* p, int32_t lane, uint32_t generation)
{
    return lane >= 0 && lane < p->bend_count && p->bend_live[lane] && p->bend_generation[lane] == generation;
}

static int attachment_ok(const ymir_particles* p, int32_t lane, uint32_t generation)
{
    return lane >= 0 && lane < p->attachment_count && p->att_kind[lane] != YMIR_ATTACH_REMOVED && p->att_generation[lane] == generation;
}

// Tombstones: the lane keeps its generation (advanced) and removed_at; every other lane reads 0.
static void kill_bend(ymir_particles* p, int32_t k, uint64_t step_count)
{
    p->bend_e1[k] = p->bend_e2[k] = p->bend_a[k] = p->bend_b[k] = p->bend_c[k] = 0;
    p->bend_rest[k] = p->bend_limit[k] = p->bend_compliance[k] = 0.0f;
    p->bend_live[k] = 0;
    p->bend_generation[k] += 1;
    p->bend_removed_at[k] = step_count;
}

// A distance takes the bends over it with it.
static void kill_distance(ymir_particles* p, int32_t e, uint64_t step_count)
{
    for (int32_t k = 0; k < p->bend_count; ++k)
    {
        if (p->bend_live[k] && (p->bend_e1[k] == e || p->bend_e2[k] == e))
        {
            kill_bend(p, k, step_count);
        }
    }
    p->dist_a[e] = p->dist_b[e] = 0;
    p->dist_rest[e] = p->dist_compliance[e] = p->dist_tension[e] = 0.0f;
    p->dist_live[e] = 0;
    p->dist_generation[e] += 1;
    p->dist_removed_at[e] = step_count;
}

static void kill_attachment(ymir_particles* p, int32_t a, uint64_t step_count)
{
    p->att_kind[a] = YMIR_ATTACH_REMOVED;
    p->att_flags[a] = 0;
    p->att_removed_at[a] = step_count;
    p->att_generation[a] += 1;
    p->att_particle[a] = -1;
    p->att_body[a] = -1;
    p->att_point_x[a] = p->att_point_y[a] = p->att_point_z[a] = 0.0f;
    p->att_compliance[a] = 0.0f;
    p->att_force_x[a] = p->att_force_y[a] = p->att_force_z[a] = 0.0f;
}

static int wet_ok(const ymir_particles* p, int32_t lane, uint32_t generation)
{
    return lane >= 0 && lane < p->wet_count && p->wet_live[lane] && p->wet_generation[lane] == generation;
}

static int sensor_ok(const ymir_particles* p, int32_t lane, uint32_t generation)
{
    return lane >= 0 && lane < p->sensor_count && p->sensor_live[lane] && p->sensor_generation[lane] == generation;
}

static void kill_sensor(ymir_particles* p, int32_t s, uint64_t step_count)
{
    p->sensor_live[s] = 0;
    p->sensor_body[s] = -1;
    p->sensor_x[s] = p->sensor_y[s] = p->sensor_z[s] = 0.0f;
    p->sensor_radius[s] = p->sensor_liquid_volume[s] = p->sensor_wet_volume[s] = 0.0f;
    p->sensor_removed_at[s] = step_count;
    p->sensor_generation[s] += 1;
}

static void wet_reset(ymir_particles* p, int32_t w)
{
    p->wet_live[w] = 0;
    p->wet_body[w] = p->wet_particle[w] = -1;
    p->wet_x[w] = p->wet_y[w] = p->wet_z[w] = 0.0f;
    p->wet_radius[w] = p->wet_volume[w] = 0.0f;
    p->wet_material[w] = 0;
}

static void kill_wet(ymir_particles* p, int32_t w, uint64_t step_count)
{
    wet_reset(p, w);
    p->wet_removed_at[w] = step_count;
    p->wet_generation[w] += 1;
}

// wet_volume_body[b] = the sum of the live patches on body b, in wet lane order; particle_wet_mu[i] = the wet friction of the lowest live
// patch on solid particle i, +inf when dry. The one writer; settle, remove(WET) and destroy call it.
static void rederive_wet(ymir_particles* p)
{
    for (int32_t i = 0; i < p->particle_capacity; ++i)
    {
        p->particle_wet_mu[i] = INFINITY;
    }
    for (int32_t b = 0; b < p->body_capacity; ++b)
    {
        p->wet_volume_body[b] = 0.0f;
    }
    for (int32_t w = 0; w < p->wet_count; ++w)
    {
        if (p->wet_live[w] && p->wet_body[w] >= 0 && p->wet_body[w] < p->body_capacity)
        {
            p->wet_volume_body[p->wet_body[w]] += p->wet_volume[w];
        }
        if (p->wet_live[w] && p->wet_particle[w] >= 0 && p->wet_particle[w] < p->particle_capacity &&
            p->particle_wet_mu[p->wet_particle[w]] == INFINITY)
        {
            p->particle_wet_mu[p->wet_particle[w]] = p->mat_wet_friction[p->wet_material[w]];
        }
    }
    // The lookup index: the live body patches grouped by body and sorted by centre x (then lane) in wet_order, with each body's slice, the box
    // its discs lie in and its largest radius (wet_body_box: min xyz, max xyz, rmax). Insertion sort: it runs at command time over wet_count lanes.
    int32_t n = 0;
    for (int32_t w = 0; w < p->wet_count; ++w)
    {
        if (p->wet_live[w] && p->wet_body[w] >= 0 && p->wet_body[w] < p->body_capacity)
        {
            int32_t k = n++;
            while (k > 0 && (p->wet_body[p->wet_order[k - 1]] > p->wet_body[w] ||
                             (p->wet_body[p->wet_order[k - 1]] == p->wet_body[w] && p->wet_x[p->wet_order[k - 1]] > p->wet_x[w])))
            {
                p->wet_order[k] = p->wet_order[k - 1];
                --k;
            }
            p->wet_order[k] = w;
        }
    }
    for (int32_t b = 0; b < p->body_capacity; ++b)
    {
        p->wet_body_start[b] = p->wet_body_count[b] = 0;
    }
    for (int32_t k = 0; k < n; ++k)
    {
        int32_t w = p->wet_order[k];
        int32_t b = p->wet_body[w];
        float* box = p->wet_body_box + 7 * (int64_t)b;
        float c[3] = {p->wet_x[w], p->wet_y[w], p->wet_z[w]};
        if (p->wet_body_count[b]++ == 0)
        {
            p->wet_body_start[b] = k;
            for (int a = 0; a < 3; ++a)
            {
                box[a] = c[a] - p->wet_radius[w];
                box[3 + a] = c[a] + p->wet_radius[w];
            }
            box[6] = p->wet_radius[w];
            continue;
        }
        for (int a = 0; a < 3; ++a)
        {
            box[a] = vmin(box[a], c[a] - p->wet_radius[w]);
            box[3 + a] = vmax(box[3 + a], c[a] + p->wet_radius[w]);
        }
        box[6] = vmax(box[6], p->wet_radius[w]);
    }
}

// ---- derivation: components and tethers ----------------------------------------------------------------------

static int heap_less(const ymir_particles* p, int32_t i, int32_t j)
{
    if (p->heap_key[i] != p->heap_key[j])
    {
        return p->heap_key[i] < p->heap_key[j];
    }
    if (p->heap_node[i] != p->heap_node[j])
    {
        return p->heap_node[i] < p->heap_node[j];
    }
    return p->heap_edge[i] < p->heap_edge[j];
}

static void heap_swap(ymir_particles* p, int32_t i, int32_t j)
{
    float k = p->heap_key[i]; p->heap_key[i] = p->heap_key[j]; p->heap_key[j] = k;
    int32_t n = p->heap_node[i]; p->heap_node[i] = p->heap_node[j]; p->heap_node[j] = n;
    int32_t e = p->heap_edge[i]; p->heap_edge[i] = p->heap_edge[j]; p->heap_edge[j] = e;
}

static void heap_push(ymir_particles* p, int32_t* size, float key, int32_t node, int32_t edge)
{
    int32_t i = (*size)++;
    p->heap_key[i] = key;
    p->heap_node[i] = node;
    p->heap_edge[i] = edge;
    while (i > 0 && heap_less(p, i, (i - 1) / 2))
    {
        heap_swap(p, i, (i - 1) / 2);
        i = (i - 1) / 2;
    }
}

// Moves the smallest entry to slot `size` (just past the shrunk heap), where the caller reads it.
static void heap_pop(ymir_particles* p, int32_t* size)
{
    int32_t n = --(*size);
    heap_swap(p, 0, n);
    int32_t i = 0;
    for (;;)
    {
        int32_t l = 2 * i + 1, r = l + 1, m = i;
        if (l < n && heap_less(p, l, m))
        {
            m = l;
        }
        if (r < n && heap_less(p, r, m))
        {
            m = r;
        }
        if (m == i)
        {
            break;
        }
        heap_swap(p, i, m);
        i = m;
    }
}


static int is_tether(const ymir_particles* p, int32_t a)
{
    return p->att_kind[a] != YMIR_ATTACH_REMOVED && (p->att_flags[a] & YMIR_ATTACH_FLAG_TETHER);
}

// 1 when attachments a < b are both tethers on different particles of one component (`label` names components) and at
// least one is on a body, so the rest path between their particles bounds the distance between their points. Two world
// points never move: their pair has no end to pull (end_w is 0 at both).
static int pairs_with(const ymir_particles* p, int32_t a, int32_t b, const int32_t* label)
{
    int32_t pa = p->att_particle[a], pb = p->att_particle[b];
    return is_tether(p, b) && pa != pb && label[pa] == label[pb] &&
           (p->att_kind[a] == YMIR_ATTACH_BODY || p->att_kind[b] == YMIR_ATTACH_BODY);
}

// Dijkstra (binary heap, ties by node then edge) over the live distances' rest lengths, from one attachment's particle
// (`only` >= 0) or from every particle of the tether island named `island` (>= 0; `isle` labels them). Writes for each
// settled particle its rest path `path`, the edge it was reached by `via` (-1 at a source), the attachment it is nearest
// `owner` (the lowest TETHER attachment on a source particle), and its place in settle order `queue`: a parent settles
// before its children. In island mode a tether particle of another island is settled but not expanded: the path through
// it is implied by its own tethers and the pair tethers. Returns the settled count. Writes no lane.
static int32_t settle(ymir_particles* p, int32_t only, int32_t island)
{
    int32_t size = 0, n = 0;
    for (int32_t i = 0; i < p->particle_count; ++i)
    {
        p->path[i] = 1.0e30f;
        p->done[i] = 0;
        p->via[i] = -1;
        p->owner[i] = -1;
    }
    for (int32_t a = 0; a < p->attachment_count; ++a)
    {
        int32_t s = p->att_particle[a];
        if (only >= 0 ? a != only : !is_tether(p, a) || p->isle[s] != island)
        {
            continue;
        }
        if (p->path[s] != 0.0f)
        {
            p->path[s] = 0.0f;
            p->owner[s] = a;
            heap_push(p, &size, 0.0f, s, -1);
        }
    }
    while (size > 0)
    {
        heap_pop(p, &size);
        float key = p->heap_key[size];
        int32_t u = p->heap_node[size];
        if (p->done[u])
        {
            continue;
        }
        p->done[u] = 1;
        p->queue[n++] = u;
        if (only < 0 && p->isle[u] >= 0 && p->via[u] >= 0)
        {
            continue; // another island's tether particle: settled, not expanded
        }
        for (int32_t k = p->adj_start[u]; k < p->adj_start[u + 1]; ++k)
        {
            int32_t e = p->adj_edge[k], v = p->dist_a[e] == u ? p->dist_b[e] : p->dist_a[e];
            float nd = key + p->dist_rest[e];
            if (!p->done[v] && nd < p->path[v])
            {
                p->path[v] = nd;
                p->via[v] = e;
                p->owner[v] = p->owner[u];
                heap_push(p, &size, nd, v, e);
            }
        }
    }
    return n;
}

// The edge count of the settled path from a source to particle q, and the particle one edge nearer the source.
static int32_t route_length(const ymir_particles* p, int32_t q)
{
    int32_t k = 0;
    for (int32_t u = q; p->via[u] >= 0; ++k)
    {
        int32_t e = p->via[u];
        u = p->dist_a[e] == u ? p->dist_b[e] : p->dist_a[e];
    }
    return k;
}

// 1 when attachment a (a tether) has a later tether on another particle of its component: a pair to derive.
static int has_pair(const ymir_particles* p, int32_t a, const int32_t* label)
{
    for (int32_t b = a + 1; b < p->attachment_count; ++b)
    {
        if (pairs_with(p, a, b, label))
        {
            return 1;
        }
    }
    return 0;
}

// The only writer of the tether lanes and the component lane. Called at the end of every topology command. The
// tether count is known from the component sizes before anything is written: a derivation that would not fit is
// refused with every lane as it was.
// ISOLATED is for a command that changed no distance and no attachment, only which lanes are live: the graph, so every
// tether and every other lane's component, is as it was. It writes the component of the `n` lanes in `lanes` (each
// just created, which is its own component, or just destroyed, which reads 0), their held list (empty) and limb body (-1),
// and returns.
enum
{
    TOPO_FULL = 0,
    TOPO_ISOLATED = 1
};

static int32_t rederive_topology(ymir_particles* p, int mode, const int32_t* lanes, int32_t n)
{
    if (mode == TOPO_ISOLATED)
    {
        for (int32_t k = 0; k < n; ++k)
        {
            p->component[lanes[k]] = p->particle_live[lanes[k]] ? lanes[k] : 0;
            p->held_count[lanes[k]] = 0;
            p->limb_body[lanes[k]] = -1;
        }
        return YMIR_PARTICLES_OK;
    }
    int32_t np = p->particle_count;
    memset(p->adj_start, 0, sizeof(int32_t) * ((size_t)np + 1));
    for (int32_t e = 0; e < p->distance_count; ++e)
    {
        if (p->dist_live[e])
        {
            p->adj_start[p->dist_a[e] + 1]++;
            p->adj_start[p->dist_b[e] + 1]++;
        }
    }
    for (int32_t i = 0; i < np; ++i)
    {
        p->adj_start[i + 1] += p->adj_start[i];
        p->comp_size[i] = p->adj_start[i]; // fill cursors
    }
    for (int32_t e = 0; e < p->distance_count; ++e)
    {
        if (p->dist_live[e])
        {
            p->adj_edge[p->comp_size[p->dist_a[e]]++] = e;
            p->adj_edge[p->comp_size[p->dist_b[e]]++] = e;
        }
    }
    // components by breadth-first labelling from the lowest lane, labels held in tether_of until the count is settled
    for (int32_t i = 0; i < np; ++i)
    {
        p->tether_of[i] = -1;
    }
    for (int32_t i = 0; i < np; ++i)
    {
        if (!p->particle_live[i] || p->tether_of[i] >= 0)
        {
            continue;
        }
        int32_t head = 0, tail = 0;
        p->tether_of[i] = i;
        p->queue[tail++] = i;
        while (head < tail)
        {
            int32_t u = p->queue[head++];
            for (int32_t k = p->adj_start[u]; k < p->adj_start[u + 1]; ++k)
            {
                int32_t e = p->adj_edge[k], v = p->dist_a[e] == u ? p->dist_b[e] : p->dist_a[e];
                if (p->tether_of[v] < 0)
                {
                    p->tether_of[v] = i;
                    p->queue[tail++] = v;
                }
            }
        }
        p->comp_size[i] = tail;
    }
    // Islands (Kim 2012 sec. 3.4): a particle holding a live TETHER attachment is a tether particle; an island is a maximal
    // set of them joined by live distances, named by its lowest TETHER attachment. isle labels tether particles, -1 elsewhere
    // (-2 while a tether particle awaits its island).
    for (int32_t i = 0; i < np; ++i)
    {
        p->isle[i] = -1;
    }
    for (int32_t a = 0; a < p->attachment_count; ++a)
    {
        if (is_tether(p, a))
        {
            p->isle[p->att_particle[a]] = -2;
        }
    }
    for (int32_t a = 0; a < p->attachment_count; ++a)
    {
        int32_t s = p->att_particle[a];
        if (!is_tether(p, a) || p->isle[s] != -2)
        {
            continue;
        }
        int32_t head = 0, tail = 0;
        p->isle[s] = a;
        p->queue[tail++] = s;
        while (head < tail)
        {
            int32_t u = p->queue[head++];
            for (int32_t k = p->adj_start[u]; k < p->adj_start[u + 1]; ++k)
            {
                int32_t e = p->adj_edge[k], v = p->dist_a[e] == u ? p->dist_b[e] : p->dist_a[e];
                if (p->isle[v] == -2)
                {
                    p->isle[v] = a;
                    p->queue[tail++] = v;
                }
            }
        }
    }
    // Each particle that is not a tether particle is held to the nearest point of each of its YMIR_TETHER_ISLANDS nearest
    // islands: pass 1 keeps, per particle, the nearest islands by rest path (ties to the lower island); pass 2 writes them.
    // Plus for each pair of tether attachments with a body among them one pair tether behind a route of booking lanes along
    // their rest path. Counted before written.
    int64_t need = 0;
    for (int32_t i = 0; i < YMIR_TETHER_ISLANDS * np; ++i)
    {
        p->near_isle[i] = -1;
    }
    for (int32_t a = 0; a < p->attachment_count; ++a)
    {
        if (!is_tether(p, a) || p->isle[p->att_particle[a]] != a)
        {
            continue;
        }
        int32_t n = settle(p, -1, a);
        for (int32_t k = 0; k < n; ++k)
        {
            int32_t u = p->queue[k];
            if (p->via[u] < 0 || p->isle[u] >= 0)
            {
                continue;
            }
            int32_t* isl = p->near_isle + YMIR_TETHER_ISLANDS * u;
            float* dist = p->near_path + YMIR_TETHER_ISLANDS * u;
            float d = p->path[u];
            int32_t m = 0;
            while (m < YMIR_TETHER_ISLANDS && isl[m] >= 0 && !(d < dist[m])) // an equal path keeps the earlier island
            {
                ++m;
            }
            if (m == YMIR_TETHER_ISLANDS)
            {
                continue;
            }
            for (int32_t j = YMIR_TETHER_ISLANDS - 1; j > m; --j)
            {
                isl[j] = isl[j - 1];
                dist[j] = dist[j - 1];
            }
            isl[m] = a;
            dist[m] = d;
        }
    }
    for (int32_t i = 0; i < YMIR_TETHER_ISLANDS * np; ++i)
    {
        need += p->near_isle[i] >= 0;
    }
    for (int32_t a = 0; a < p->attachment_count; ++a)
    {
        if (!is_tether(p, a) || !has_pair(p, a, p->tether_of))
        {
            continue;
        }
        settle(p, a, -1);
        for (int32_t b = a + 1; b < p->attachment_count; ++b)
        {
            if (pairs_with(p, a, b, p->tether_of))
            {
                need += 1 + route_length(p, p->att_particle[b]);
            }
        }
    }
    if (need > p->tether_capacity)
    {
        return YMIR_PARTICLES_FULL;
    }
    for (int32_t i = 0; i < np; ++i)
    {
        p->component[i] = p->tether_of[i] >= 0 ? p->tether_of[i] : 0;
        p->held_count[i] = 0;
        p->limb_body[i] = -1;
    }
    // The held lists: held_start first holds each particle's slice end, then the attachments are walked from the highest
    // lane down, each taking the slot below, so a slice ends up in ascending attachment lane order and starts at held_start.
    for (int32_t a = 0; a < p->attachment_count; ++a)
    {
        if (p->att_kind[a] == YMIR_ATTACH_BODY)
        {
            p->held_count[p->att_particle[a]]++;
        }
    }
    for (int32_t i = 0, end = 0; i < np; ++i)
    {
        end += p->held_count[i];
        p->held_start[i] = end;
    }
    for (int32_t a = p->attachment_count - 1; a >= 0; --a)
    {
        if (p->att_kind[a] == YMIR_ATTACH_BODY)
        {
            int32_t i = p->att_particle[a];
            p->held_body[--p->held_start[i]] = p->att_body[a];
            if (p->att_flags[a] & YMIR_ATTACH_FLAG_LIMB)
            {
                p->limb_body[i] = p->att_body[a]; // descending: the lowest lane's body is the last written
            }
        }
    }
    int32_t nt = 0;
    for (int32_t a = 0; a < p->attachment_count; ++a)
    {
        if (!is_tether(p, a) || p->isle[p->att_particle[a]] != a)
        {
            continue;
        }
        int32_t n = settle(p, -1, a);
        for (int32_t i = 0; i < np; ++i)
        {
            p->tether_of[i] = -1;
        }
        for (int32_t k = 0; k < n; ++k)
        {
            int32_t u = p->queue[k], e = p->via[u], held = 0;
            if (e < 0 || p->isle[u] >= 0)
            {
                continue;
            }
            for (int32_t m = 0; m < YMIR_TETHER_ISLANDS; ++m)
            {
                held |= p->near_isle[YMIR_TETHER_ISLANDS * u + m] == a;
            }
            int32_t other = p->dist_a[e] == u ? p->dist_b[e] : p->dist_a[e];
            if (!held || (p->via[other] >= 0 && p->tether_of[other] < 0))
            {
                continue; // not one of u's nearest islands; or its parent is not held to this island (a float tie at rank 2)
            }
            int32_t t = nt++;
            p->tether_att[t] = p->owner[u];
            p->tether_particle[t] = u;
            p->tether_bound[t] = p->path[u];
            p->tether_edge[t] = e;
            p->tether_other[t] = -1;
            p->tether_parent[t] = p->tether_of[other];
            p->tether_of[u] = t;
        }
    }
    for (int32_t a = 0; a < p->attachment_count; ++a)
    {
        if (!is_tether(p, a) || !has_pair(p, a, p->component))
        {
            continue;
        }
        settle(p, a, -1);
        for (int32_t b = a + 1; b < p->attachment_count; ++b)
        {
            if (!pairs_with(p, a, b, p->component))
            {
                continue;
            }
            int32_t q = p->att_particle[b], len = route_length(p, q), first = nt, u = q;
            for (int32_t m = len - 1; m >= 0; --m) // the route, written from the source end: parents first
            {
                int32_t e = p->via[u], t = first + m;
                p->tether_att[t] = -1;
                p->tether_particle[t] = u;
                p->tether_bound[t] = p->path[u];
                p->tether_edge[t] = e;
                p->tether_other[t] = -1;
                p->tether_parent[t] = m == 0 ? -1 : t - 1;
                u = p->dist_a[e] == u ? p->dist_b[e] : p->dist_a[e];
            }
            int32_t t = first + len;
            p->tether_att[t] = a;
            p->tether_particle[t] = q;
            p->tether_bound[t] = p->path[q];
            p->tether_edge[t] = -1;
            p->tether_other[t] = b;
            p->tether_parent[t] = t - 1;
            nt = t + 1;
        }
    }
    p->tether_count = nt;
    return YMIR_PARTICLES_OK;
}

// ---- commands ------------------------------------------------------------------------------------------------

int32_t ymir_particles_create_particles(ymir_particles* p, int32_t n, const ymir_particle_desc* d, uint64_t step_count, ymir_handle* out)
{
    if (p == NULL || n < 0 || (n > 0 && d == NULL))
    {
        return YMIR_PARTICLES_INVALID_ARGUMENT;
    }
    for (int32_t k = 0; k < n; ++k)
    {
        if (!finite3(d[k].position) || !finite3(d[k].rest) || !finite3(d[k].velocity) || !isfinite(d[k].mass) || d[k].mass < 0.0f ||
            (d[k].mass > 0.0f && !isfinite(1.0f / d[k].mass)) || !isfinite(d[k].radius) || d[k].radius <= 0.0f ||
            (d[k].phase & ~(uint32_t)YMIR_PHASE_VALID) != 0 || d[k].material >= (uint32_t)p->material_capacity)
        {
            return YMIR_PARTICLES_INVALID_ARGUMENT;
        }
    }
    int32_t fluid = 0; // the fluid capacity sizes the neighbour arena: a batch past it is refused whole
    for (int32_t k = 0; k < n; ++k)
    {
        fluid += (d[k].phase & YMIR_PHASE_FLUID) != 0;
    }
    if (p->live_fluid + fluid > p->fluid_capacity)
    {
        return YMIR_PARTICLES_FULL;
    }
    if (free_count(p->particle_live, 0, p->particle_removed_at, p->particle_count, p->particle_capacity, step_count) < n)
    {
        return YMIR_PARTICLES_FULL;
    }
    int32_t cursor = 0;
    for (int32_t k = 0; k < n; ++k)
    {
        int32_t i = next_lane(p->particle_live, 0, p->particle_removed_at, p->particle_count, p->particle_capacity, cursor, step_count);
        cursor = i + 1;
        p->particle_count = i + 1 > p->particle_count ? i + 1 : p->particle_count;
        p->pos_x[i] = d[k].position[0]; p->pos_y[i] = d[k].position[1]; p->pos_z[i] = d[k].position[2];
        p->vel_x[i] = d[k].velocity[0]; p->vel_y[i] = d[k].velocity[1]; p->vel_z[i] = d[k].velocity[2];
        p->rest_x[i] = d[k].rest[0]; p->rest_y[i] = d[k].rest[1]; p->rest_z[i] = d[k].rest[2];
        p->radius[i] = d[k].radius;
        p->inv_mass[i] = d[k].mass > 0.0f ? 1.0f / d[k].mass : 0.0f;
        p->phase[i] = d[k].phase;
        p->material[i] = d[k].material;
        p->support_body[i] = p->support_particle[i] = -1;
        p->particle_live[i] = 1;
        p->live_particles++;
        p->live_fluid += (d[k].phase & YMIR_PHASE_FLUID) != 0;
        p->queue[k] = i;
        if (out != NULL)
        {
            out[k] = (ymir_handle){i, p->particle_generation[i]};
        }
    }
    return rederive_topology(p, TOPO_ISOLATED, p->queue, n); // a new particle holds no constraint
}

int32_t ymir_particles_destroy_particles(ymir_particles* p, int32_t n, const ymir_handle* h, uint64_t step_count)
{
    if (p == NULL || n < 0 || (n > 0 && h == NULL))
    {
        return YMIR_PARTICLES_INVALID_ARGUMENT;
    }
    for (int32_t k = 0; k < n; ++k)
    {
        if (!particle_ok(p, h[k].lane, h[k].generation))
        {
            return YMIR_PARTICLES_INVALID_ARGUMENT;
        }
    }
    int held = 0; // whether a named particle holds a live distance: then the graph changes (an attachment alone makes no tether lane)
    for (int32_t k = 0; k < n && !held; ++k)
    {
        int32_t i = h[k].lane;
        for (int32_t e = 0; e < p->distance_count && !held; ++e)
        {
            held = p->dist_live[e] && (p->dist_a[e] == i || p->dist_b[e] == i);
        }
    }
    int32_t killed = 0;
    for (int32_t k = 0; k < n; ++k)
    {
        int32_t i = h[k].lane;
        if (!p->particle_live[i])
        {
            continue; // named twice
        }
        p->queue[killed++] = i;
        p->live_fluid -= (p->phase[i] & YMIR_PHASE_FLUID) != 0;
        for (int32_t e = 0; e < p->distance_count; ++e)
        {
            if (p->dist_live[e] && (p->dist_a[e] == i || p->dist_b[e] == i))
            {
                kill_distance(p, e, step_count);
            }
        }
        for (int32_t a = 0; a < p->attachment_count; ++a)
        {
            if (p->att_kind[a] != YMIR_ATTACH_REMOVED && p->att_particle[a] == i)
            {
                kill_attachment(p, a, step_count);
            }
        }
        p->pos_x[i] = p->pos_y[i] = p->pos_z[i] = 0.0f;
        p->vel_x[i] = p->vel_y[i] = p->vel_z[i] = 0.0f;
        p->rest_x[i] = p->rest_y[i] = p->rest_z[i] = 0.0f;
        p->radius[i] = p->inv_mass[i] = 0.0f;
        p->phase[i] = p->material[i] = 0;
        p->density[i] = p->lambda[i] = 0.0f;
        p->support_body[i] = p->support_particle[i] = -1;
        p->particle_live[i] = 0;
        p->particle_generation[i] += 1;
        p->particle_removed_at[i] = step_count;
        p->live_particles--;
    }
    for (int32_t i = 0; i < p->particle_count; ++i)
    {
        if (p->particle_live[i] && p->support_particle[i] >= 0 && !p->particle_live[p->support_particle[i]])
        {
            p->support_particle[i] = -1; // a drop does not keep resting on a solid destroyed under it until the next step
        }
    }
    int wet_killed = 0;
    for (int32_t w = 0; w < p->wet_count; ++w)
    {
        if (p->wet_live[w] && p->wet_particle[w] >= 0 && !p->particle_live[p->wet_particle[w]])
        {
            kill_wet(p, w, step_count); // the liquid on a destroyed solid goes with it; a particle patch is on no body, so wet_volume_body does not change
            wet_killed = 1;
        }
    }
    if (wet_killed)
    {
        rederive_wet(p); // the dry lane reads +inf again
    }
    return rederive_topology(p, held ? TOPO_FULL : TOPO_ISOLATED, p->queue, killed);
}

int32_t ymir_particles_create_distances(ymir_particles* p, int32_t n, const ymir_distance_desc* d, uint64_t step_count, ymir_handle* out)
{
    if (p == NULL || n < 0 || (n > 0 && d == NULL))
    {
        return YMIR_PARTICLES_INVALID_ARGUMENT;
    }
    for (int32_t k = 0; k < n; ++k)
    {
        if (!particle_ok(p, d[k].a.lane, d[k].a.generation) || !particle_ok(p, d[k].b.lane, d[k].b.generation) ||
            d[k].a.lane == d[k].b.lane || !isfinite(d[k].rest) || !isfinite(d[k].compliance) || d[k].compliance < 0.0f ||
            ((p->phase[d[k].a.lane] | p->phase[d[k].b.lane]) & YMIR_PHASE_FLUID) != 0) // fluid holds no constraint
        {
            return YMIR_PARTICLES_INVALID_ARGUMENT;
        }
    }
    if (free_count(p->dist_live, 0, p->dist_removed_at, p->distance_count, p->distance_capacity, step_count) < n)
    {
        return YMIR_PARTICLES_FULL;
    }
    int32_t old_count = p->distance_count, cursor = 0;
    for (int32_t k = 0; k < n; ++k)
    {
        int32_t e = next_lane(p->dist_live, 0, p->dist_removed_at, p->distance_count, p->distance_capacity, cursor, step_count);
        cursor = e + 1;
        int32_t a = d[k].a.lane, b = d[k].b.lane;
        p->lane_list[k] = e;
        p->distance_count = e + 1 > p->distance_count ? e + 1 : p->distance_count;
        p->dist_a[e] = a;
        p->dist_b[e] = b;
        p->dist_rest[e] = d[k].rest >= 0.0f
                              ? d[k].rest
                              : vlen(vsub((v3){p->rest_x[a], p->rest_y[a], p->rest_z[a]}, (v3){p->rest_x[b], p->rest_y[b], p->rest_z[b]}));
        p->dist_compliance[e] = d[k].compliance;
        p->dist_tension[e] = 0.0f;
        p->dist_live[e] = 1;
    }
    int32_t status = rederive_topology(p, TOPO_FULL, NULL, 0);
    if (status != YMIR_PARTICLES_OK)
    {
        for (int32_t k = 0; k < n; ++k) // back out: the lanes read as they did before the command
        {
            int32_t e = p->lane_list[k];
            p->dist_a[e] = p->dist_b[e] = 0;
            p->dist_rest[e] = p->dist_compliance[e] = 0.0f;
            p->dist_live[e] = 0;
        }
        p->distance_count = old_count;
        return status;
    }
    if (out != NULL)
    {
        for (int32_t k = 0; k < n; ++k)
        {
            out[k] = (ymir_handle){p->lane_list[k], p->dist_generation[p->lane_list[k]]};
        }
    }
    return YMIR_PARTICLES_OK;
}

int32_t ymir_particles_create_bends(ymir_particles* p, int32_t n, const ymir_bend_desc* d, uint64_t step_count, ymir_handle* out)
{
    if (p == NULL || n < 0 || (n > 0 && d == NULL))
    {
        return YMIR_PARTICLES_INVALID_ARGUMENT;
    }
    for (int32_t k = 0; k < n; ++k)
    {
        int32_t e1 = d[k].e1.lane, e2 = d[k].e2.lane;
        if (!distance_ok(p, e1, d[k].e1.generation) || !distance_ok(p, e2, d[k].e2.generation) || e1 == e2 ||
            !isfinite(d[k].compliance) || !isfinite(d[k].max_angle) || d[k].max_angle > 3.14159265f ||
            (d[k].compliance < 0.0f && d[k].max_angle < 0.0f) || !(p->dist_rest[e1] > 0.0f) || !(p->dist_rest[e2] > 0.0f))
        {
            return YMIR_PARTICLES_INVALID_ARGUMENT; // a bend that does nothing, or over links of no length, or not over two links
        }
        int32_t a1 = p->dist_a[e1], b1 = p->dist_b[e1], a2 = p->dist_a[e2], b2 = p->dist_b[e2];
        int shared = (a1 == a2) + (a1 == b2) + (b1 == a2) + (b1 == b2);
        if (shared != 1)
        {
            return YMIR_PARTICLES_INVALID_ARGUMENT; // the two distances must share exactly one particle
        }
    }
    if (free_count(p->bend_live, 0, p->bend_removed_at, p->bend_count, p->bend_capacity, step_count) < n)
    {
        return YMIR_PARTICLES_FULL;
    }
    int32_t cursor = 0;
    for (int32_t k = 0; k < n; ++k)
    {
        int32_t i = next_lane(p->bend_live, 0, p->bend_removed_at, p->bend_count, p->bend_capacity, cursor, step_count);
        cursor = i + 1;
        int32_t e1 = d[k].e1.lane, e2 = d[k].e2.lane;
        int32_t a1 = p->dist_a[e1], b1 = p->dist_b[e1], a2 = p->dist_a[e2], b2 = p->dist_b[e2];
        int32_t a, b, c;
        if (b1 == a2) { a = a1; b = b1; c = b2; }
        else if (a1 == a2) { a = b1; b = a1; c = b2; }
        else if (b1 == b2) { a = a1; b = b1; c = a2; }
        else { a = b1; b = a1; c = a2; }
        p->bend_count = i + 1 > p->bend_count ? i + 1 : p->bend_count;
        p->bend_e1[i] = e1; p->bend_e2[i] = e2;
        p->bend_a[i] = a; p->bend_b[i] = b; p->bend_c[i] = c;
        v3 ra = {p->rest_x[a], p->rest_y[a], p->rest_z[a]}, rb = {p->rest_x[b], p->rest_y[b], p->rest_z[b]},
           rc = {p->rest_x[c], p->rest_y[c], p->rest_z[c]};
        v3 centroid = vmul(vadd(vadd(ra, rb), rc), 1.0f / 3.0f);
        p->bend_rest[i] = vlen(vsub(rb, centroid));
        float l1 = p->dist_rest[e1], l2 = p->dist_rest[e2];
        // |b - centroid| = |(a - b) + (c - b)| / 3, and at deflection phi from straight the links' sum has
        // squared length l1^2 + l2^2 - 2 l1 l2 cos(phi): the offset at the limit.
        p->bend_limit[i] = d[k].max_angle < 0.0f ? -1.0f : sqrtf(vmax(0.0f, l1 * l1 + l2 * l2 - 2.0f * l1 * l2 * cosf(d[k].max_angle))) / 3.0f;
        p->bend_compliance[i] = d[k].compliance;
        p->bend_live[i] = 1;
        if (out != NULL)
        {
            out[k] = (ymir_handle){i, p->bend_generation[i]};
        }
    }
    return rederive_topology(p, TOPO_FULL, NULL, 0);
}

int32_t ymir_particles_attach(ymir_particles* p, const ymir_attachment_desc* d, int32_t body_count, uint64_t step_count, ymir_handle* out)
{
    if (p == NULL || d == NULL || (d->kind != YMIR_ATTACH_BODY && d->kind != YMIR_ATTACH_STATIC) ||
        (d->flags & ~(uint32_t)(YMIR_ATTACH_FLAG_TETHER | YMIR_ATTACH_FLAG_LIMB)) != 0 ||
        (d->kind == YMIR_ATTACH_STATIC && (d->flags & YMIR_ATTACH_FLAG_LIMB) != 0) || !particle_ok(p, d->particle.lane, d->particle.generation) ||
        !finite3(d->point) || !isfinite(d->compliance) || d->compliance < 0.0f ||
        (p->phase[d->particle.lane] & YMIR_PHASE_FLUID) != 0) // fluid holds no constraint
    {
        return YMIR_PARTICLES_INVALID_ARGUMENT;
    }
    if (d->kind == YMIR_ATTACH_BODY && (d->body < 0 || d->body >= body_count))
    {
        return YMIR_PARTICLES_INVALID_ARGUMENT;
    }
    int32_t a = next_lane(p->att_kind, YMIR_ATTACH_REMOVED, p->att_removed_at, p->attachment_count, p->attachment_capacity, 0, step_count);
    if (a < 0)
    {
        return YMIR_PARTICLES_FULL;
    }
    int32_t old_count = p->attachment_count;
    p->attachment_count = a + 1 > p->attachment_count ? a + 1 : p->attachment_count;
    p->att_kind[a] = d->kind;
    p->att_flags[a] = d->flags;
    p->att_particle[a] = d->particle.lane;
    p->att_body[a] = d->kind == YMIR_ATTACH_BODY ? d->body : -1;
    p->att_point_x[a] = d->point[0];
    p->att_point_y[a] = d->point[1];
    p->att_point_z[a] = d->point[2];
    p->att_compliance[a] = d->compliance;
    p->att_force_x[a] = p->att_force_y[a] = p->att_force_z[a] = 0.0f;
    int32_t status = rederive_topology(p, TOPO_FULL, NULL, 0);
    if (status != YMIR_PARTICLES_OK)
    {
        p->att_kind[a] = a < old_count ? YMIR_ATTACH_REMOVED : 0;
        p->att_flags[a] = 0;
        p->att_particle[a] = a < old_count ? -1 : 0;
        p->att_body[a] = a < old_count ? -1 : 0;
        p->att_point_x[a] = p->att_point_y[a] = p->att_point_z[a] = 0.0f;
        p->att_compliance[a] = 0.0f;
        p->attachment_count = old_count;
        return status;
    }
    if (out != NULL)
    {
        *out = (ymir_handle){a, p->att_generation[a]};
    }
    return YMIR_PARTICLES_OK;
}

int32_t ymir_particles_remove(ymir_particles* p, uint32_t type, int32_t n, const ymir_handle* h, uint64_t step_count)
{
    if (p == NULL || n < 0 || (n > 0 && h == NULL) ||
        (type != YMIR_LANES_DISTANCE && type != YMIR_LANES_BEND && type != YMIR_LANES_ATTACHMENT && type != YMIR_LANES_WET &&
         type != YMIR_LANES_SENSOR))
    {
        return YMIR_PARTICLES_INVALID_ARGUMENT;
    }
    for (int32_t k = 0; k < n; ++k)
    {
        int ok = type == YMIR_LANES_DISTANCE   ? distance_ok(p, h[k].lane, h[k].generation)
                 : type == YMIR_LANES_BEND     ? bend_ok(p, h[k].lane, h[k].generation)
                 : type == YMIR_LANES_WET      ? wet_ok(p, h[k].lane, h[k].generation)
                 : type == YMIR_LANES_SENSOR   ? sensor_ok(p, h[k].lane, h[k].generation)
                                               : attachment_ok(p, h[k].lane, h[k].generation);
        if (!ok)
        {
            return YMIR_PARTICLES_INVALID_ARGUMENT;
        }
    }
    if (type == YMIR_LANES_SENSOR)
    {
        for (int32_t k = 0; k < n; ++k)
        {
            if (p->sensor_live[h[k].lane])
            {
                kill_sensor(p, h[k].lane, step_count);
            }
        }
        return YMIR_PARTICLES_OK; // a sensor is in no graph: never rederive_topology
    }
    if (type == YMIR_LANES_WET)
    {
        for (int32_t k = 0; k < n; ++k)
        {
            if (p->wet_live[h[k].lane])
            {
                kill_wet(p, h[k].lane, step_count);
            }
        }
        rederive_wet(p); // patches change no graph: never through rederive_topology
        return YMIR_PARTICLES_OK;
    }
    for (int32_t k = 0; k < n; ++k)
    {
        int32_t lane = h[k].lane;
        if (type == YMIR_LANES_DISTANCE && p->dist_live[lane])
        {
            kill_distance(p, lane, step_count);
        }
        else if (type == YMIR_LANES_BEND && p->bend_live[lane])
        {
            kill_bend(p, lane, step_count);
        }
        else if (type == YMIR_LANES_ATTACHMENT && p->att_kind[lane] != YMIR_ATTACH_REMOVED)
        {
            kill_attachment(p, lane, step_count);
        }
    }
    return rederive_topology(p, TOPO_FULL, NULL, 0);
}

int32_t ymir_particles_set_material(ymir_particles* p, int32_t index, const ymir_material_desc* d)
{
    if (p == NULL || d == NULL || index < 0 || index >= p->material_capacity || !isfinite(d->friction) || d->friction < 0.0f ||
        !isfinite(d->viscosity) || d->viscosity < 0.0f || !isfinite(d->wet_friction) || d->wet_friction < 0.0f)
    {
        return YMIR_PARTICLES_INVALID_ARGUMENT;
    }
    p->mat_friction[index] = d->friction;
    p->mat_viscosity[index] = d->viscosity;
    p->mat_wet_friction[index] = d->wet_friction;
    p->mat_id[index] = d->material_id;
    return YMIR_PARTICLES_OK;
}

// ---- the particle phase --------------------------------------------------------------------------------------

typedef struct phase
{
    ymir_particles* p;
    float h, inv_h;
    float k; // force of a correction: lambda / h^2
} phase;

// Generalised inverse mass of a proxy along unit `n` at world point `at`.
static float proxy_w(const ymir_particle_proxy* b, v3 at, v3 n)
{
    if (b->inv_mass == 0.0f)
    {
        return 0.0f;
    }
    v3 r = vsub(at, (v3){b->position[0], b->position[1], b->position[2]});
    v3 rn = vcross(r, n);
    return b->inv_mass + vdot(rn, mat_mul(b->inv_inertia, rn));
}

// Applies a position-level impulse `imp` (kg m) at `at` to a proxy: moves its predicted pose and books the
// velocity-level impulse (imp / h) the caller applies to the body before Box3D integrates its position.
static void proxy_push(const phase* f, ymir_particle_proxy* b, v3 at, v3 imp)
{
    if (b->inv_mass == 0.0f)
    {
        return;
    }
    v3 r = vsub(at, (v3){b->position[0], b->position[1], b->position[2]});
    v3 torque = vcross(r, imp);
    v3 dtheta = mat_mul(b->inv_inertia, torque);
    b->position[0] += imp.x * b->inv_mass;
    b->position[1] += imp.y * b->inv_mass;
    b->position[2] += imp.z * b->inv_mass;
    float* q = b->rotation;
    float dq[4] = {
        0.5f * (dtheta.x * q[3] + dtheta.y * q[2] - dtheta.z * q[1]),
        0.5f * (dtheta.y * q[3] + dtheta.z * q[0] - dtheta.x * q[2]),
        0.5f * (dtheta.z * q[3] + dtheta.x * q[1] - dtheta.y * q[0]),
        0.5f * (-dtheta.x * q[0] - dtheta.y * q[1] - dtheta.z * q[2])};
    float n2 = 0.0f;
    for (int k = 0; k < 4; ++k)
    {
        q[k] += dq[k];
        n2 += q[k] * q[k];
    }
    float inv = 1.0f / sqrtf(n2);
    for (int k = 0; k < 4; ++k)
    {
        q[k] *= inv;
    }
    b->sub_impulse[0] += imp.x * f->inv_h;
    b->sub_impulse[1] += imp.y * f->inv_h;
    b->sub_impulse[2] += imp.z * f->inv_h;
    b->sub_angular[0] += torque.x * f->inv_h;
    b->sub_angular[1] += torque.y * f->inv_h;
    b->sub_angular[2] += torque.z * f->inv_h;
}

// The world point an attachment holds, and the inverse mass of what it holds it to along `n`.
static v3 end_point(const ymir_particles* p, int32_t a)
{
    if (p->att_kind[a] == YMIR_ATTACH_BODY)
    {
        const ymir_particle_proxy* b = p->proxies + p->att_body[a];
        v3 local = {p->att_point_x[a], p->att_point_y[a], p->att_point_z[a]};
        return vadd((v3){b->position[0], b->position[1], b->position[2]}, qrot(b->rotation, local));
    }
    return (v3){p->att_point_x[a], p->att_point_y[a], p->att_point_z[a]};
}

static float end_w(const ymir_particles* p, int32_t a, v3 at, v3 n)
{
    return p->att_kind[a] == YMIR_ATTACH_BODY ? proxy_w(p->proxies + p->att_body[a], at, n) : 0.0f;
}

static void end_push(const phase* f, int32_t a, v3 at, v3 imp)
{
    ymir_particles* p = f->p;
    if (p->att_kind[a] == YMIR_ATTACH_BODY)
    {
        proxy_push(f, p->proxies + p->att_body[a], at, imp);
    }
}

// Adds a force (N) the attachment `a` exerts on its particles to its sensor lane.
static void add_force(ymir_particles* p, int32_t a, v3 force)
{
    p->att_force_x[a] += force.x;
    p->att_force_y[a] += force.y;
    p->att_force_z[a] += force.z;
}

// ---- grid and particle-particle contact ----------------------------------------------------------------------

static uint32_t cell_hash(int32_t a, int32_t b, int32_t c, uint32_t mask)
{
    return (((uint32_t)a * 73856093u) ^ ((uint32_t)b * 19349663u) ^ ((uint32_t)c * 83492791u)) & mask;
}

static int32_t cell_of(float v, float inv_cell)
{
    return (int32_t)vclamp(floorf(v * inv_cell), -1.0e6f, 1.0e6f);
}

// Counting sort into a hash grid of cell 1 / inv_cell: the live particles' predicted positions, fluid lanes left out when
// `solids` is set. Within a bucket particles keep index order, so every neighbour walk is the same on every run.
static void grid_sort(ymir_particles* p, float inv_cell, int solids)
{
    int32_t buckets = (int32_t)p->table_mask + 1;
    memset(p->cell_start, 0, sizeof(int32_t) * (size_t)(buckets + 1));
    for (int32_t i = 0; i < p->particle_count; ++i)
    {
        if (!p->particle_live[i] || (solids && (p->phase[i] & YMIR_PHASE_FLUID)))
        {
            continue;
        }
        p->cell_x[i] = cell_of(p->pp_x[i], inv_cell);
        p->cell_y[i] = cell_of(p->pp_y[i], inv_cell);
        p->cell_z[i] = cell_of(p->pp_z[i], inv_cell);
        p->cell_start[cell_hash(p->cell_x[i], p->cell_y[i], p->cell_z[i], p->table_mask) + 1]++;
    }
    for (int32_t c = 0; c < buckets; ++c)
    {
        p->cell_start[c + 1] += p->cell_start[c];
        p->cell_fill[c] = 0;
    }
    for (int32_t i = 0; i < p->particle_count; ++i)
    {
        if (p->particle_live[i] && !(solids && (p->phase[i] & YMIR_PHASE_FLUID)))
        {
            uint32_t c = cell_hash(p->cell_x[i], p->cell_y[i], p->cell_z[i], p->table_mask);
            p->sorted[p->cell_start[c] + p->cell_fill[c]++] = i;
        }
    }
}

// The contact grid: the non-fluid lanes at a cell of 2 rmax, which holds every contact pair (reach 2 rmax). Fluid lanes meet
// the others in fluid_density, on a grid of their own.
static void build_grid(ymir_particles* p)
{
    float rmax = 0.0f;
    for (int32_t i = 0; i < p->particle_count; ++i)
    {
        if (p->particle_live[i] && !(p->phase[i] & YMIR_PHASE_FLUID))
        {
            rmax = vmax(rmax, p->radius[i]);
        }
    }
    grid_sort(p, rmax > 0.0f ? 1.0f / (2.0f * rmax) : 0.0f, 1);
}


// Do particles i and j collide by contact? Symmetric in i and j, which momentum conservation needs. Two fluid particles
// never do. Particles of different groups always do; of one group only when both set SELF_COLLIDE, and not, when either sets
// SELF_FILTER, if they touched in the rest pose.
static int collides(const ymir_particles* p, int32_t i, int32_t j)
{
    uint32_t a = p->phase[i], b = p->phase[j];
    if (a & b & YMIR_PHASE_FLUID)
    {
        return 0; // two fluid particles meet in the density, not by contact
    }
    if ((a & YMIR_PHASE_GROUP_MASK) != (b & YMIR_PHASE_GROUP_MASK))
    {
        return 1;
    }
    if (!(a & b & YMIR_PHASE_SELF_COLLIDE))
    {
        return 0;
    }
    if ((a | b) & YMIR_PHASE_SELF_FILTER)
    {
        v3 d = vsub((v3){p->rest_x[i], p->rest_y[i], p->rest_z[i]}, (v3){p->rest_x[j], p->rest_y[j], p->rest_z[j]});
        float reach = 1.5f * (p->radius[i] + p->radius[j]);
        if (vdot(d, d) < reach * reach)
        {
            return 0;
        }
    }
    return 1;
}

// The overlapping pair (i, j) found by the walk from i: the displacement from j to i, the contact distance, and the
// inverse-mass sum. 1 when the pair collides, overlaps and has a particle that moves, else 0.
static int touching(const ymir_particles* p, int32_t i, int32_t j, v3 xi, v3* d, float* r2, float* md, float* wsum)
{
    *d = vsub(xi, P(p, p->pp_x, p->pp_y, p->pp_z, j));
    *md = p->radius[i] + p->radius[j];
    *r2 = vdot(*d, *d);
    *wsum = p->inv_mass[i] + p->inv_mass[j];
    return *r2 < *md * *md && *r2 >= 1e-12f && *wsum != 0.0f && collides(p, i, j);
}

// The pairs of the live particles, each once: from every particle i, the partners j of its own cell with a higher index and
// every partner of the 13 cells that follow its own in lexicographic offset order, cells read in that order and a cell's
// particles in index order. `fn` sees each candidate pair (a bucket can hold other cells' particles; those are skipped here)
// and decides if it counts. Inlined at each call, so the contact pass pays nothing for sharing the walk.
#if defined(_MSC_VER)
#define YMIR_INLINE static __forceinline
#else
#define YMIR_INLINE static inline __attribute__((always_inline))
#endif

typedef int (*pair_fn)(ymir_particles* p, void* ctx, int32_t i, int32_t j);

YMIR_INLINE void walk_half(ymir_particles* p, pair_fn fn, void* ctx)
{
    for (int32_t i = 0; i < p->particle_count; ++i)
    {
        if (!p->particle_live[i] || (p->phase[i] & YMIR_PHASE_FLUID)) // the grid holds no fluid lane
        {
            continue;
        }
        for (int32_t cell = 0; cell < 14; ++cell) // 0 is the own cell; 1..13 are offsets 14..26 of the 27
        {
            int32_t o = cell == 0 ? 13 : 13 + cell, da = o / 9 - 1, db = o / 3 % 3 - 1, dc = o % 3 - 1;
            int32_t ca = p->cell_x[i] + da, cb = p->cell_y[i] + db, cc = p->cell_z[i] + dc;
            uint32_t bucket = cell_hash(ca, cb, cc, p->table_mask);
            for (int32_t q = p->cell_start[bucket]; q < p->cell_start[bucket + 1]; ++q)
            {
                int32_t j = p->sorted[q];
                if ((cell == 0 && j <= i) || p->cell_x[j] != ca || p->cell_y[j] != cb || p->cell_z[j] != cc)
                {
                    continue; // the other end of a pair, or another cell sharing the bucket
                }
                fn(p, ctx, i, j);
            }
        }
    }
}

// One pass of the contact walk: pass 0 counts every particle's overlaps, pass 1 moves both ends of each pair.
static int contact_pair(ymir_particles* p, void* ctx, int32_t i, int32_t j) // 1 when the pair touches
{
    v3 d;
    float r2, md, wsum;
    if (!touching(p, i, j, P(p, p->pp_x, p->pp_y, p->pp_z, i), &d, &r2, &md, &wsum))
    {
        return 0;
    }
    if (*(int*)ctx == 0)
    {
        p->contacts[i]++;
        p->contacts[j]++;
        return 1;
    }
    float rr = sqrtf(r2), depth = (md - rr) / rr;
    float weight = 1.0f / (float)(p->contacts[i] > p->contacts[j] ? p->contacts[i] : p->contacts[j]);
    v3 di = vmul(d, weight * (p->inv_mass[i] / wsum) * depth);
    v3 dj = vmul(d, weight * (p->inv_mass[j] / wsum) * depth);
    p->dp_x[i] += di.x; p->dp_y[i] += di.y; p->dp_z[i] += di.z;
    p->dp_x[j] -= dj.x; p->dp_y[j] -= dj.y; p->dp_z[j] -= dj.z;
    return 1;
}

// Overlapping pairs push each other apart, split by inverse mass (Macklin 2014). Each pair is found once, by walking
// half the 27-cell neighbourhood. Two walks: the first counts every particle's overlaps, the second moves both ends
// of each pair by one displacement weighted 1 / max(n_i, n_j). The pair's two corrections then hold equal and
// opposite momentum, so contact conserves it; a per-particle average would not. Pairs apply in walk order, so every
// sum is the same on every run. No pair list is kept: its size is not bounded by the particle count.
static void particle_contacts(ymir_particles* p)
{
    for (int32_t i = 0; i < p->particle_count; ++i)
    {
        p->contacts[i] = 0;
        p->dp_x[i] = p->dp_y[i] = p->dp_z[i] = 0.0f;
    }
    for (int pass = 0; pass < 2; ++pass)
    {
        walk_half(p, contact_pair, &pass);
    }
    for (int32_t i = 0; i < p->particle_count; ++i)
    {
        if (p->particle_live[i])
        {
            p->pp_x[i] += p->dp_x[i];
            p->pp_y[i] += p->dp_y[i];
            p->pp_z[i] += p->dp_z[i];
        }
    }
}

// ---- fluid: number density (Macklin and Mueller 2013, unilateral as Macklin et al. 2014 sec. 7) and XSPH ---------------

// The kernel reach of a pair: 4 r at equal radii. A kernel pair lies in the rings of cells fluid_grid sets.
static float kernel_h(const ymir_particles* p, int32_t i, int32_t j)
{
    return 2.0f * (p->radius[i] + p->radius[j]);
}

static float n0_of(const ymir_particles* p, float h)
{
    return p->fluid_k / (h * h * h);
}

static float poly6(float r2, float h)
{
    float d = h * h - r2, h3 = h * h * h;
    return d > 0.0f ? YMIR_POLY6 / (h3 * h3 * h3) * d * d * d : 0.0f;
}

// Gradient of W at displacement d (|d| = r) for reach h: Spiky, along d.
static v3 spiky(v3 d, float r, float h)
{
    float h3 = h * h * h, t = h - r;
    return vmul(d, YMIR_SPIKY / (h3 * h3) * t * t / r);
}

// Pass B: the pair's correction, (lambda_i + lambda_j) grad W / n0, with each end moving its own inverse mass times it (XPBD,
// Macklin et al. 2016): equal and opposite momentum at any mass ratio, equal masses unchanged, a static end takes nothing.
static void density_correct(ymir_particles* p, int32_t i, int32_t j)
{
    float wsum = p->inv_mass[i] + p->inv_mass[j];
    if (wsum == 0.0f)
    {
        return;
    }
    v3 d = vsub(P(p, p->pp_x, p->pp_y, p->pp_z, i), P(p, p->pp_x, p->pp_y, p->pp_z, j));
    float h = kernel_h(p, i, j), r2 = vdot(d, d), l = p->lambda[i] + p->lambda[j];
    if (r2 >= h * h || r2 < 1e-12f || l == 0.0f)
    {
        return;
    }
    v3 corr = vmul(spiky(d, sqrtf(r2), h), l / n0_of(p, h));
    v3 di = vmul(corr, p->inv_mass[i]), dj = vmul(corr, p->inv_mass[j]);
    p->dp_x[i] += di.x; p->dp_y[i] += di.y; p->dp_z[i] += di.z;
    p->dp_x[j] -= dj.x; p->dp_y[j] -= dj.y; p->dp_z[j] -= dj.z;
}

// The fluid grid: every live lane (a kernel pair can hold a solid) in the tables build_grid left, rebuilt at the kernel cell 4 rfmax.
// A kernel pair reaches h = 2 (ri + rj), at most 2 (rfmax + rmax): fluid_rings cells hold it (1 at equal radii). Then the lanes'
// predicted position, inverse mass and phase are copied into sorted order, so a bucket's candidates read contiguously; sorted[q] is
// the lane of sorted index q. Nothing after the fluid pass reads this grid. Returns rmax over the live lanes.
static float fluid_grid(ymir_particles* p)
{
    float rmax = 0.0f, rfmax = 0.0f;
    for (int32_t i = 0; i < p->particle_count; ++i)
    {
        if (p->particle_live[i])
        {
            rmax = vmax(rmax, p->radius[i]);
            rfmax = (p->phase[i] & YMIR_PHASE_FLUID) ? vmax(rfmax, p->radius[i]) : rfmax;
        }
    }
    float inv_cell = 1.0f / (4.0f * rfmax);
    p->fluid_rings = (int32_t)ceilf(2.0f * (rfmax + rmax) * inv_cell);
    grid_sort(p, inv_cell, 0);
    for (int32_t q = 0; q < p->live_particles; ++q)
    {
        int32_t j = p->sorted[q];
        p->sx[q] = p->pp_x[j];
        p->sy[q] = p->pp_y[j];
        p->sz[q] = p->pp_z[j];
        p->sw[q] = p->inv_mass[j];
        p->sphase[q] = p->phase[j];
    }
    memset(p->cell_fill, 0, sizeof(int32_t) * ((size_t)p->table_mask + 1)); // the buckets' visit stamps, pass A
    return rmax;
}

// Density constraint of every fluid particle, once per substep, after contact. Pass A gathers, fluid rows in sorted order: row i
// takes the buckets of the (2 rings + 1)^3 cells around its own, each bucket once (a bucket two offsets hash to is skipped the
// second time; a particle of a cell outside the rings is out of reach, so the distance test needs no cell compare), and
// compacts without a branch the candidates within 2 (ri + rmax) into a row buffer. Over the buffer: n_i = sum W (self included; the
// solids that touch i count too), C_i = max(n_i / n0 - 1, 0), lambda_i = -C_i / (sum_k w_k |grad_k C_i|^2 + eps w_i), 0 when that
// is 0; a solid's lambda is 0. Pass A also writes i's neighbour list: every kernel neighbour in walk order until
// YMIR_FLUID_NEIGHBOURS, each past that counted in fluid_overflow, as are candidates past the row buffer (n and lambda stay exact
// until the buffer overflows; the correction and XSPH lose the pairs past the list). Pass B reads the lists: each fluid row i in index
// order, each listed j in list order, a fluid j below i skipped (its row does the pair), a solid always done from i; corrections
// apply after (Jacobi), so the sums are the same on every run. Then the fluid-solid contacts, on the same lists.
static void fluid_density(ymir_particles* p)
{
    for (int32_t i = 0; i < p->particle_count; ++i)
    {
        p->lambda[i] = 0.0f;
        p->dp_x[i] = p->dp_y[i] = p->dp_z[i] = 0.0f;
    }
    float rmax = fluid_grid(p);
    int32_t rings = p->fluid_rings, used = 0, arena = p->fluid_capacity * YMIR_FLUID_NEIGHBOURS;
    for (int32_t qi = 0; qi < p->live_particles; ++qi)
    {
        if (!(p->sphase[qi] & YMIR_PHASE_FLUID))
        {
            continue;
        }
        int32_t i = p->sorted[qi];
        v3 xi = {p->sx[qi], p->sy[qi], p->sz[qi]}, gsum = {0.0f, 0.0f, 0.0f};
        float h0 = 4.0f * p->radius[i], n0 = n0_of(p, h0), n = poly6(0.0f, h0), ssum = 0.0f, wi = p->inv_mass[i];
        float reach = 2.0f * (p->radius[i] + rmax), reach2 = reach * reach;
        int32_t buf[YMIR_FLUID_CANDIDATES], m = 0, hits = 0, stamp = qi + 1;
        for (int32_t da = -rings; da <= rings; ++da)
        for (int32_t db = -rings; db <= rings; ++db)
        for (int32_t dc = -rings; dc <= rings; ++dc)
        {
            uint32_t bucket = cell_hash(p->cell_x[i] + da, p->cell_y[i] + db, p->cell_z[i] + dc, p->table_mask);
            if (p->cell_fill[bucket] == stamp)
            {
                continue;
            }
            p->cell_fill[bucket] = stamp;
            for (int32_t q = p->cell_start[bucket]; q < p->cell_start[bucket + 1]; ++q)
            {
                float dx = xi.x - p->sx[q], dy = xi.y - p->sy[q], dz = xi.z - p->sz[q];
                int32_t hit = (dx * dx + dy * dy + dz * dz < reach2) & (q != qi);
                buf[m] = q;
                m += hit & (m < YMIR_FLUID_CANDIDATES - 1);
                hits += hit;
            }
        }
        p->fluid_overflow += hits - m;
        int32_t* list = p->nbr + used;
        p->nbr_start[i] = used;
        int32_t count = 0;
        for (int32_t k = 0; k < m; ++k)
        {
            int32_t q = buf[k], j = p->sorted[q];
            if (!(p->sphase[q] & YMIR_PHASE_FLUID) && !collides(p, i, j)) // fluid shares the density with fluid, and with the solids it would touch by contact
            {
                continue;
            }
            v3 d = vsub(xi, (v3){p->sx[q], p->sy[q], p->sz[q]});
            float h = kernel_h(p, i, j), r2 = vdot(d, d);
            if (r2 >= h * h)
            {
                continue;
            }
            if (used + count < arena)
            {
                list[count++] = j;
            }
            else
            {
                p->fluid_overflow++;
            }
            n += poly6(r2, h);
            if (r2 >= 1e-12f)
            {
                v3 g = spiky(d, sqrtf(r2), h);
                gsum = vadd(gsum, g);
                ssum += p->sw[q] * vdot(g, g);
            }
        }
        p->nbr_count[i] = count;
        used += count;
        float c = vmax(n / n0 - 1.0f, 0.0f);
        p->density[i] = n / n0;
        float den = (wi * vdot(gsum, gsum) + ssum) / (n0 * n0) + YMIR_PBF_EPS * wi;
        p->lambda[i] = c == 0.0f || den == 0.0f ? 0.0f : -c / den;
    }
    for (int32_t i = 0; i < p->particle_count; ++i)
    {
        if (!p->particle_live[i] || !(p->phase[i] & YMIR_PHASE_FLUID))
        {
            continue;
        }
        const int32_t* list = p->nbr + p->nbr_start[i];
        for (int32_t k = 0; k < p->nbr_count[i]; ++k)
        {
            int32_t j = list[k];
            if (j < i && (p->phase[j] & YMIR_PHASE_FLUID))
            {
                continue;
            }
            density_correct(p, i, j);
        }
    }
    for (int32_t i = 0; i < p->particle_count; ++i)
    {
        if (p->particle_live[i])
        {
            p->pp_x[i] += p->dp_x[i];
            p->pp_y[i] += p->dp_y[i];
            p->pp_z[i] += p->dp_z[i];
        }
    }
    // Fluid-solid contact (the contact pass never sees a fluid lane): the listed solids of each fluid row that still overlap it,
    // counted then moved as contact_pair does, weight 1 / max(n_i, n_j) over these pairs only; the moves apply after (Jacobi).
    for (int32_t i = 0; i < p->particle_count; ++i)
    {
        p->contacts[i] = 0;
        p->dp_x[i] = p->dp_y[i] = p->dp_z[i] = 0.0f;
    }
    for (int pass = 0; pass < 2; ++pass)
    {
        for (int32_t i = 0; i < p->particle_count; ++i)
        {
            if (!p->particle_live[i] || !(p->phase[i] & YMIR_PHASE_FLUID))
            {
                continue;
            }
            const int32_t* list = p->nbr + p->nbr_start[i];
            for (int32_t k = 0; k < p->nbr_count[i]; ++k)
            {
                if (!(p->phase[list[k]] & YMIR_PHASE_FLUID) && contact_pair(p, &pass, i, list[k]) && pass == 1 && p->support_particle[i] < 0)
                {
                    p->support_particle[i] = list[k]; // the first listed solid that touches: what the particle rests on
                }
            }
        }
    }
    for (int32_t i = 0; i < p->particle_count; ++i)
    {
        if (p->particle_live[i])
        {
            p->pp_x[i] += p->dp_x[i];
            p->pp_y[i] += p->dp_y[i];
            p->pp_z[i] += p->dp_z[i];
        }
    }
}

// XSPH (Schechter and Bridson 2012, as Macklin and Mueller 2013 sec. 5): a fluid pair's velocities relax toward each other,
// dv = c (vj - vi) W / n0 with c the mean of the two materials', split by inverse mass like the density correction so the
// pair keeps its momentum. Applied after the pass. Pairs come from the lists pass A wrote this substep (fluid j above i only), so a
// pair that entered reach since is missed: smoothing, not a constraint.
static void viscosity_pair(ymir_particles* p, int32_t i, int32_t j)
{
    float wsum = p->inv_mass[i] + p->inv_mass[j];
    if (wsum == 0.0f)
    {
        return;
    }
    float c = 0.5f * (p->mat_viscosity[p->material[i]] + p->mat_viscosity[p->material[j]]);
    v3 d = vsub(P(p, p->pos_x, p->pos_y, p->pos_z, i), P(p, p->pos_x, p->pos_y, p->pos_z, j));
    float h = kernel_h(p, i, j), r2 = vdot(d, d);
    if (c == 0.0f || r2 >= h * h)
    {
        return;
    }
    v3 dv = vmul(vsub(P(p, p->vel_x, p->vel_y, p->vel_z, j), P(p, p->vel_x, p->vel_y, p->vel_z, i)), c * poly6(r2, h) / n0_of(p, h));
    v3 di = vmul(dv, 2.0f * p->inv_mass[i] / wsum), dj = vmul(dv, 2.0f * p->inv_mass[j] / wsum);
    p->dp_x[i] += di.x; p->dp_y[i] += di.y; p->dp_z[i] += di.z;
    p->dp_x[j] -= dj.x; p->dp_y[j] -= dj.y; p->dp_z[j] -= dj.z;
}

// Does any material have a viscosity? XSPH is a no-op when none does (a pair's coefficient is the mean of its two), so the walk is skipped.
static int viscous_material(const ymir_particles* p)
{
    for (int32_t m = 0; m < p->material_capacity; ++m)
    {
        if (p->mat_viscosity[m] > 0.0f)
        {
            return 1;
        }
    }
    return 0;
}

static void fluid_viscosity(ymir_particles* p)
{
    for (int32_t i = 0; i < p->particle_count; ++i)
    {
        p->dp_x[i] = p->dp_y[i] = p->dp_z[i] = 0.0f;
    }
    for (int32_t i = 0; i < p->particle_count; ++i)
    {
        if (!p->particle_live[i] || !(p->phase[i] & YMIR_PHASE_FLUID))
        {
            continue;
        }
        const int32_t* list = p->nbr + p->nbr_start[i];
        for (int32_t k = 0; k < p->nbr_count[i]; ++k)
        {
            int32_t j = list[k];
            if (j > i && (p->phase[j] & YMIR_PHASE_FLUID)) // a pair needs two fluid ends, and is done from the lower row
            {
                viscosity_pair(p, i, j);
            }
        }
    }
    for (int32_t i = 0; i < p->particle_count; ++i)
    {
        if (p->particle_live[i])
        {
            p->vel_x[i] += p->dp_x[i];
            p->vel_y[i] += p->dp_y[i];
            p->vel_z[i] += p->dp_z[i];
        }
    }
}

// ---- particle-body contact -----------------------------------------------------------------------------------

// Signed distance from `x` to the proxy's surface, the outward normal there and the nearest surface point.
static float proxy_distance(const ymir_particle_proxy* b, v3 x, v3* normal, v3* surface)
{
    v3 c = {b->position[0], b->position[1], b->position[2]};
    v3 n;
    float d;
    if (b->shape == YMIR_PROXY_BOX)
    {
        v3 l = qrot_inv(b->rotation, vsub(x, c));
        v3 half = {b->size[0], b->size[1], b->size[2]};
        v3 q = {fabsf(l.x) - half.x, fabsf(l.y) - half.y, fabsf(l.z) - half.z};
        v3 nl;
        if (q.x > 0.0f || q.y > 0.0f || q.z > 0.0f)
        {
            v3 out = {vmax(q.x, 0.0f), vmax(q.y, 0.0f), vmax(q.z, 0.0f)};
            d = vlen(out);
            nl = (v3){l.x < 0.0f ? -out.x : out.x, l.y < 0.0f ? -out.y : out.y, l.z < 0.0f ? -out.z : out.z};
            nl = vmul(nl, 1.0f / d);
        }
        else
        {
            d = vmax(q.x, vmax(q.y, q.z)); // inside: the nearest face
            nl = d == q.x ? (v3){l.x < 0.0f ? -1.0f : 1.0f, 0.0f, 0.0f}
                 : d == q.y ? (v3){0.0f, l.y < 0.0f ? -1.0f : 1.0f, 0.0f}
                            : (v3){0.0f, 0.0f, l.z < 0.0f ? -1.0f : 1.0f};
        }
        n = qrot(b->rotation, nl);
    }
    else
    {
        v3 centre = c;
        if (b->shape == YMIR_PROXY_CAPSULE)
        {
            v3 axis = qrot(b->rotation, (v3){0.0f, 1.0f, 0.0f});
            centre = vadd(c, vmul(axis, vclamp(vdot(vsub(x, c), axis), -b->size[1], b->size[1])));
        }
        v3 v = vsub(x, centre);
        float l = vlen(v);
        n = l > 1e-9f ? vmul(v, 1.0f / l) : (v3){0.0f, 1.0f, 0.0f};
        d = l - b->size[0];
    }
    *normal = n;
    *surface = vsub(x, vmul(n, d));
    return d;
}

// The friction on body `body` at body-frame point (x, y, z): the wet friction of the lowest live patch whose disc holds the point, never above mu.
// The one patch test, for particle-body contact here and body-body contact through the world's friction hook.
float ymir_particles_wet_mu(const ymir_particles* p, int32_t body, float x, float y, float z, float mu)
{
    if (p == NULL || body < 0 || body >= p->body_capacity || p->wet_body_count[body] == 0)
    {
        return mu;
    }
    const float* box = p->wet_body_box + 7 * (int64_t)body;
    if (x < box[0] || x > box[3] || y < box[1] || y > box[4] || z < box[2] || z > box[5])
    {
        return mu; // outside every disc's box: a constant check
    }
    // The slice is sorted by centre x: only centres within the body's largest radius of x can hold the point. The lowest lane among them wins.
    const int32_t* slice = p->wet_order + p->wet_body_start[body];
    int32_t lo = 0, hi = p->wet_body_count[body];
    while (lo < hi)
    {
        int32_t mid = (lo + hi) / 2;
        if (p->wet_x[slice[mid]] < x - box[6])
        {
            lo = mid + 1;
        }
        else
        {
            hi = mid;
        }
    }
    int32_t found = -1;
    for (int32_t k = lo; k < p->wet_body_count[body] && p->wet_x[slice[k]] <= x + box[6]; ++k)
    {
        int32_t w = slice[k];
        v3 d = {x - p->wet_x[w], y - p->wet_y[w], z - p->wet_z[w]};
        if ((found < 0 || w < found) && vlen(d) <= p->wet_radius[w])
        {
            found = w;
        }
    }
    return found < 0 ? mu : vmin(mu, p->mat_wet_friction[p->wet_material[found]]);
}

static float proxy_reach(const ymir_particle_proxy* b)
{
    if (b->shape == YMIR_PROXY_SPHERE)
    {
        return b->size[0];
    }
    if (b->shape == YMIR_PROXY_CAPSULE)
    {
        return b->size[0] + b->size[1];
    }
    return sqrtf(b->size[0] * b->size[0] + b->size[1] * b->size[1] + b->size[2] * b->size[2]);
}

// 1 when particle i is held to body k by an attachment: such a particle is the body's own point, so contact skips that body.
// Reads the list rederive_topology wrote at the last command.
static int held_by(const ymir_particles* p, int32_t i, int32_t k)
{
    for (int32_t s = p->held_start[i], e = s + p->held_count[i]; s < e; ++s)
    {
        if (p->held_body[s] == k)
        {
            return 1;
        }
    }
    return 0;
}

// Each particle against each shaped body in ascending lane order. The correction is split by inverse mass between
// the particle and the body at the contact point; Coulomb friction removes the tangential slip up to mu times the
// push. Every body the particle is attached to is skipped: the attachment already ties them. A LIMB attachment skips the
// attached body's whole limb tree: flesh does not meet the bones of its own limb.
static void body_contacts(phase* f)
{
    ymir_particles* p = f->p;
    for (int32_t i = 0; i < p->particle_count; ++i)
    {
        if (!p->particle_live[i] || p->inv_mass[i] == 0.0f)
        {
            continue;
        }
        int fluid = (p->phase[i] & YMIR_PHASE_FLUID) != 0;
        float mu = p->mat_friction[p->material[i]];
        float r = p->radius[i];
        float wi = p->inv_mass[i];
        for (int32_t k = 0; k < p->body_count; ++k)
        {
            ymir_particle_proxy* b = p->proxies + k;
            if (b->shape == YMIR_PROXY_NONE)
            {
                continue;
            }
            v3 x = P(p, p->pp_x, p->pp_y, p->pp_z, i);
            v3 c = {b->position[0], b->position[1], b->position[2]};
            float reach = proxy_reach(b) + r;
            v3 dc = vsub(x, c);
            if (vdot(dc, dc) > reach * reach)
            {
                continue;
            }
            if ((p->limb_body[i] >= 0 && b->root == p->proxies[p->limb_body[i]].root) || held_by(p, i, k))
            {
                continue; // the particle is flesh of this body's limb tree, or attached to this body
            }
            v3 n, s;
            float d = proxy_distance(b, x, &n, &s);
            if (d >= r)
            {
                continue;
            }
            if (p->phase[i] & YMIR_PHASE_FLUID)
            {
                p->support_body[i] = k; // bodies ascend: the highest lane touched wins
            }
            float depth = r - d;
            float wb = proxy_w(b, s, n);
            float w = wi + wb;
            x = vadd(x, vmul(n, depth * wi / w));
            proxy_push(f, b, s, vmul(n, -depth / w));
            float mu_k = mu;
            if (!fluid)
            {
                // A solid on a wet body or wetted itself slips: the lowest wet friction there; a fluid particle keeps its own.
                v3 l = qrot_inv(b->rotation, vsub(s, c));
                mu_k = vmin(ymir_particles_wet_mu(p, k, l.x, l.y, l.z, mu), p->particle_wet_mu[i]);
            }
            if (mu_k > 0.0f)
            {
                // Slip of the particle against the body point over this substep, tangential part.
                v3 rb = vsub(s, (v3){b->position[0], b->position[1], b->position[2]});
                v3 vb = vadd((v3){b->velocity[0], b->velocity[1], b->velocity[2]},
                             vcross((v3){b->angular_velocity[0], b->angular_velocity[1], b->angular_velocity[2]}, rb));
                v3 slip = vsub(vsub(x, P(p, p->pos_x, p->pos_y, p->pos_z, i)), vmul(vb, f->h)); // (x - x_prev) - v_body h
                slip = vsub(slip, vmul(n, vdot(slip, n)));
                float sl = vlen(slip);
                if (sl > 1e-9f)
                {
                    v3 t = vmul(slip, 1.0f / sl);
                    float move = vmin(sl, mu_k * depth);
                    float wt = wi + proxy_w(b, s, t);
                    x = vsub(x, vmul(t, move * wi / wt));
                    proxy_push(f, b, s, vmul(t, move / wt));
                }
            }
            put(p->pp_x, p->pp_y, p->pp_z, i, x);
        }
    }
}

// ---- attachments, distances, bends, tethers ------------------------------------------------------------------

// Books on edge `e` the load an impulse `imp` (kg m) delivered to the particle `far`, the side of the edge away
// from `near`, puts on it: along the edge, positive when the edge is being stretched. Every impulse the
// structure delivers has an equal and opposite one, so the sum past an edge is the load it carries however the
// solver routed it.
static void book(const phase* f, int32_t e, int32_t near_i, int32_t far_i, v3 imp)
{
    ymir_particles* p = f->p;
    v3 d = vsub(P(p, p->pp_x, p->pp_y, p->pp_z, far_i), P(p, p->pp_x, p->pp_y, p->pp_z, near_i));
    float len = vlen(d);
    if (len > 1e-9f)
    {
        p->dist_acc[e] += -vdot(imp, d) / len * f->k;
    }
}

// Pulls an attached particle onto the point it is attached to (rest length 0), by inverse mass.
static void solve_attachments(phase* f)
{
    ymir_particles* p = f->p;
    for (int32_t a = 0; a < p->attachment_count; ++a)
    {
        if (p->att_kind[a] == YMIR_ATTACH_REMOVED)
        {
            continue;
        }
        int32_t i = p->att_particle[a];
        v3 at = end_point(p, a);
        v3 d = vsub(P(p, p->pp_x, p->pp_y, p->pp_z, i), at);
        float len = vlen(d);
        if (len < 1e-9f)
        {
            continue;
        }
        v3 n = vmul(d, 1.0f / len);
        float den = p->inv_mass[i] + end_w(p, a, at, n) + p->att_compliance[a] / (f->h * f->h);
        if (den == 0.0f)
        {
            continue;
        }
        float dl = -len / den;
        put(p->pp_x, p->pp_y, p->pp_z, i, vadd(P(p, p->pp_x, p->pp_y, p->pp_z, i), vmul(n, p->inv_mass[i] * dl)));
        end_push(f, a, at, vmul(n, -dl));
        add_force(p, a, vmul(n, dl * f->k));
    }
}

// Each live distance in lane order (Gauss-Seidel), one iteration per substep: the small step does the converging.
// Its lambda is its tension (positive when stretched).
static void solve_distances(phase* f)
{
    ymir_particles* p = f->p;
    for (int32_t e = 0; e < p->distance_count; ++e)
    {
        if (!p->dist_live[e])
        {
            continue;
        }
        int32_t i = p->dist_a[e], j = p->dist_b[e];
        float wi = p->inv_mass[i], wj = p->inv_mass[j], alpha = p->dist_compliance[e] / (f->h * f->h);
        v3 d = vsub(P(p, p->pp_x, p->pp_y, p->pp_z, j), P(p, p->pp_x, p->pp_y, p->pp_z, i));
        float len = vlen(d);
        if (len < 1e-9f || wi + wj + alpha == 0.0f)
        {
            continue;
        }
        v3 n = vmul(d, 1.0f / len);
        float dl = -(len - p->dist_rest[e]) / (wi + wj + alpha);
        put(p->pp_x, p->pp_y, p->pp_z, i, vsub(P(p, p->pp_x, p->pp_y, p->pp_z, i), vmul(n, wi * dl)));
        put(p->pp_x, p->pp_y, p->pp_z, j, vadd(P(p, p->pp_x, p->pp_y, p->pp_z, j), vmul(n, wj * dl)));
        p->dist_acc[e] += -dl * f->k;
    }
}

// Each live bend in lane order, on the offset of the shared particle b from the centroid of a, b, c (Kelager
// 2010): a soft pass (XPBD, compliance / h^2) toward the rest offset, then a hard unilateral pass at the offset the
// maximum angle allows, free inside it and rigid at it. The three position impulses sum to zero; what crossed
// each of the two distances is booked on it.
static void solve_bends(phase* f)
{
    ymir_particles* p = f->p;
    for (int32_t k = 0; k < p->bend_count; ++k)
    {
        if (!p->bend_live[k])
        {
            continue;
        }
        int32_t a = p->bend_a[k], b = p->bend_b[k], c = p->bend_c[k];
        float wa = p->inv_mass[a], wb = p->inv_mass[b], wc = p->inv_mass[c];
        v3 ja = {0.0f, 0.0f, 0.0f}, jc = {0.0f, 0.0f, 0.0f};
        for (int pass = 0; pass < 2; ++pass)
        {
            if ((pass == 0 && p->bend_compliance[k] < 0.0f) || (pass == 1 && p->bend_limit[k] < 0.0f))
            {
                continue;
            }
            float target = pass == 0 ? p->bend_rest[k] : p->bend_limit[k];
            v3 centroid = vmul(vadd(vadd(P(p, p->pp_x, p->pp_y, p->pp_z, a), P(p, p->pp_x, p->pp_y, p->pp_z, b)),
                                    P(p, p->pp_x, p->pp_y, p->pp_z, c)), 1.0f / 3.0f);
            v3 d = vsub(P(p, p->pp_x, p->pp_y, p->pp_z, b), centroid);
            float len = vlen(d);
            if (len < 1e-9f)
            {
                continue;
            }
            float violation = len - target;
            if (pass == 1 && violation <= 0.0f)
            {
                continue;
            }
            float alpha = pass == 0 ? p->bend_compliance[k] / (f->h * f->h) : 0.0f;
            float den = (wa + 4.0f * wb + wc) / 9.0f + alpha;
            if (den == 0.0f)
            {
                continue;
            }
            v3 n = vmul(d, 1.0f / len);
            float dl = -violation / den;
            v3 ia = vmul(n, -dl / 3.0f), ib = vmul(n, 2.0f * dl / 3.0f), ic = vmul(n, -dl / 3.0f);
            put(p->pp_x, p->pp_y, p->pp_z, a, vadd(P(p, p->pp_x, p->pp_y, p->pp_z, a), vmul(ia, wa)));
            put(p->pp_x, p->pp_y, p->pp_z, b, vadd(P(p, p->pp_x, p->pp_y, p->pp_z, b), vmul(ib, wb)));
            put(p->pp_x, p->pp_y, p->pp_z, c, vadd(P(p, p->pp_x, p->pp_y, p->pp_z, c), vmul(ic, wc)));
            ja = vadd(ja, ia);
            jc = vadd(jc, ic);
        }
        book(f, p->bend_e1[k], a, b, vmul(ja, -1.0f)); // what crossed e1 toward b and c
        book(f, p->bend_e2[k], b, c, jc);
    }
}

// Long-range bounds derived from the graph, unilateral: a tether holds a particle within the rest path of an
// attachment point, a pair tether holds two attachment points within the rest path between their particles. They
// bear load past the distances, so each tether's impulse is routed leaves first up its path tree and booked on the
// edges it crossed: the tension sensors read the load, not the path.
static void solve_tethers(phase* f)
{
    ymir_particles* p = f->p;
    for (int32_t t = 0; t < p->tether_count; ++t)
    {
        p->tacc_x[t] = p->tacc_y[t] = p->tacc_z[t] = 0.0f;
        int32_t a = p->tether_att[t], b = p->tether_other[t];
        if (a < 0)
        {
            continue; // a booking lane: it carries load up its route and bounds nothing
        }
        float bound = p->tether_bound[t];
        if (b >= 0)
        {
            v3 pa = end_point(p, a), pb = end_point(p, b);
            v3 d = vsub(pa, pb);
            float len = vlen(d);
            if (len <= bound)
            {
                continue;
            }
            v3 n = vmul(d, 1.0f / len);
            float den = end_w(p, a, pa, n) + end_w(p, b, pb, n);
            if (den == 0.0f)
            {
                continue;
            }
            float dl = -(len - bound) / den;
            end_push(f, a, pa, vmul(n, dl));
            end_push(f, b, pb, vmul(n, -dl));
            add_force(p, a, vmul(n, -dl * f->k)); // each point pulls the particles away from the other
            add_force(p, b, vmul(n, dl * f->k));
            put(p->tacc_x, p->tacc_y, p->tacc_z, t, vmul(n, -dl)); // delivered to b's particle, toward a
            continue;
        }
        int32_t j = p->tether_particle[t];
        v3 at = end_point(p, a);
        v3 d = vsub(P(p, p->pp_x, p->pp_y, p->pp_z, j), at);
        float len = vlen(d);
        if (len <= bound)
        {
            continue;
        }
        v3 n = vmul(d, 1.0f / len);
        float den = p->inv_mass[j] + end_w(p, a, at, n);
        if (den == 0.0f)
        {
            continue;
        }
        float dl = -(len - bound) / den;
        put(p->pp_x, p->pp_y, p->pp_z, j, vadd(P(p, p->pp_x, p->pp_y, p->pp_z, j), vmul(n, p->inv_mass[j] * dl)));
        end_push(f, a, at, vmul(n, -dl));
        add_force(p, a, vmul(n, dl * f->k)); // the force on the particles, toward the attachment
        put(p->tacc_x, p->tacc_y, p->tacc_z, t, vmul(n, dl));
    }
    for (int32_t t = p->tether_count - 1; t >= 0; --t)
    {
        v3 load = P(p, p->tacc_x, p->tacc_y, p->tacc_z, t);
        int32_t e = p->tether_edge[t];
        if (e >= 0)
        {
            int32_t j = p->tether_particle[t], other = p->dist_a[e] == j ? p->dist_b[e] : p->dist_a[e];
            book(f, e, other, j, load);
        }
        if (p->tether_parent[t] >= 0)
        {
            int32_t q = p->tether_parent[t];
            put(p->tacc_x, p->tacc_y, p->tacc_z, q, vadd(P(p, p->tacc_x, p->tacc_y, p->tacc_z, q), load));
        }
    }
}

// ---- the step ------------------------------------------------------------------------------------------------

int32_t ymir_particles_begin(ymir_particles* p)
{
    if (p == NULL || p->live_particles == 0)
    {
        return 0;
    }
    for (int32_t k = 0; k < p->body_count; ++k)
    {
        ymir_particle_proxy* b = p->proxies + k;
        memset(b->impulse, 0, sizeof(b->impulse));
        memset(b->angular_impulse, 0, sizeof(b->angular_impulse));
    }
    p->fluid_overflow = 0;
    for (int32_t e = 0; e < p->distance_count; ++e)
    {
        p->dist_acc[e] = 0.0f;
    }
    for (int32_t a = 0; a < p->attachment_count; ++a)
    {
        p->att_force_x[a] = p->att_force_y[a] = p->att_force_z[a] = 0.0f;
    }
    return 1;
}

// One substep of `h` against the proxies' poses at its start and velocities after the rigid solve: proxies are
// predicted like particles, every constraint moves both sides by inverse mass, and each proxy's correction returns
// as sub_impulse (its correction over h) for the caller to apply before the rigid positions integrate. Position,
// velocity and momentum stay one state: XPBD's velocity update, done by the rigid integrator.
void ymir_particles_substep(ymir_particles* p, float h, float gx, float gy, float gz)
{
    if (p == NULL || p->live_particles == 0)
    {
        return;
    }
    phase f = {p, h, 1.0f / h, 1.0f / (h * h)};
    v3 g = {gx, gy, gz};
    for (int32_t k = 0; k < p->body_count; ++k)
    {
        ymir_particle_proxy* b = p->proxies + k;
        memset(b->sub_impulse, 0, sizeof(b->sub_impulse));
        memset(b->sub_angular, 0, sizeof(b->sub_angular));
        // predict the pose the rigid integrator reaches with this velocity
        b->position[0] += h * b->velocity[0];
        b->position[1] += h * b->velocity[1];
        b->position[2] += h * b->velocity[2];
        v3 w = vmul((v3){b->angular_velocity[0], b->angular_velocity[1], b->angular_velocity[2]}, h);
        float* q = b->rotation;
        float dq[4] = {
            0.5f * (w.x * q[3] + w.y * q[2] - w.z * q[1]),
            0.5f * (w.y * q[3] + w.z * q[0] - w.x * q[2]),
            0.5f * (w.z * q[3] + w.x * q[1] - w.y * q[0]),
            0.5f * (-w.x * q[0] - w.y * q[1] - w.z * q[2])};
        float n2 = 0.0f;
        for (int c = 0; c < 4; ++c)
        {
            q[c] += dq[c];
            n2 += q[c] * q[c];
        }
        float inv = 1.0f / sqrtf(n2);
        for (int c = 0; c < 4; ++c)
        {
            q[c] *= inv;
        }
    }
    for (int32_t i = 0; i < p->particle_count; ++i)
    {
        if (!p->particle_live[i])
        {
            continue;
        }
        if (p->phase[i] & YMIR_PHASE_FLUID)
        {
            p->support_body[i] = p->support_particle[i] = -1; // this substep's contacts write them again
        }
        v3 x = P(p, p->pos_x, p->pos_y, p->pos_z, i);
        if (p->inv_mass[i] > 0.0f)
        {
            v3 v = vadd(P(p, p->vel_x, p->vel_y, p->vel_z, i), vmul(g, h));
            put(p->vel_x, p->vel_y, p->vel_z, i, v);
            x = vadd(x, vmul(v, h));
        }
        put(p->pp_x, p->pp_y, p->pp_z, i, x);
    }
    build_grid(p);
    particle_contacts(p);
    if (p->live_fluid > 0)
    {
        fluid_density(p);
    }
    solve_attachments(&f);
    solve_distances(&f);
    solve_bends(&f);
    solve_tethers(&f);
    body_contacts(&f);
    for (int32_t i = 0; i < p->particle_count; ++i)
    {
        if (!p->particle_live[i])
        {
            continue;
        }
        v3 x = P(p, p->pos_x, p->pos_y, p->pos_z, i), y = P(p, p->pp_x, p->pp_y, p->pp_z, i);
        if (p->inv_mass[i] > 0.0f)
        {
            put(p->vel_x, p->vel_y, p->vel_z, i, vmul(vsub(y, x), f.inv_h));
        }
        put(p->pos_x, p->pos_y, p->pos_z, i, y);
    }
    if (p->live_fluid > 0 && viscous_material(p))
    {
        fluid_viscosity(p);
    }
    for (int32_t k = 0; k < p->body_count; ++k)
    {
        ymir_particle_proxy* b = p->proxies + k;
        for (int c = 0; c < 3; ++c)
        {
            b->impulse[c] += b->sub_impulse[c];
            b->angular_impulse[c] += b->sub_angular[c];
        }
    }
}

// The volume of the live wet patches whose centre lies within r of the world point c. A body's patches go through the wet cut's index: the sphere is taken into the
// body's frame (where its patches' centres are), culled against the box of the body's discs, and only centres within r of its x are walked, in the index's order
// (bodies ascending, then centre x). Patches on solid particles have no index and are summed after, in wet lane order, at their particle's position.
static float pooled_volume(const ymir_particles* p, v3 c, float r)
{
    float total = 0.0f;
    float r2 = r * r;
    for (int32_t b = 0; b < p->body_count && b < p->body_capacity; ++b)
    {
        if (p->wet_body_count[b] == 0)
        {
            continue;
        }
        const ymir_particle_proxy* body = p->proxies + b;
        v3 l = qrot_inv(body->rotation, vsub(c, (v3){body->position[0], body->position[1], body->position[2]}));
        const float* box = p->wet_body_box + 7 * (int64_t)b;
        if (l.x + r < box[0] || l.x - r > box[3] || l.y + r < box[1] || l.y - r > box[4] || l.z + r < box[2] || l.z - r > box[5])
        {
            continue;
        }
        const int32_t* slice = p->wet_order + p->wet_body_start[b];
        int32_t lo = 0, hi = p->wet_body_count[b];
        while (lo < hi)
        {
            int32_t mid = (lo + hi) / 2;
            if (p->wet_x[slice[mid]] < l.x - r)
            {
                lo = mid + 1;
            }
            else
            {
                hi = mid;
            }
        }
        for (int32_t k = lo; k < p->wet_body_count[b] && p->wet_x[slice[k]] <= l.x + r; ++k)
        {
            int32_t w = slice[k];
            v3 d = {l.x - p->wet_x[w], l.y - p->wet_y[w], l.z - p->wet_z[w]};
            if (vdot(d, d) <= r2)
            {
                total += p->wet_volume[w];
            }
        }
    }
    for (int32_t w = 0; w < p->wet_count; ++w)
    {
        if (p->wet_live[w] && p->wet_particle[w] >= 0 && p->wet_particle[w] < p->particle_count && p->particle_live[p->wet_particle[w]])
        {
            v3 d = vsub(P(p, p->pos_x, p->pos_y, p->pos_z, p->wet_particle[w]), c);
            if (vdot(d, d) <= r2)
            {
                total += p->wet_volume[w];
            }
        }
    }
    return total;
}

// Each live sensor reads the liquid inside it: the sum, in lane order, of (2 r)^3 over the live fluid particles whose centre lies within its sphere, against the
// pose the last substep left each proxy in. It runs whether or not any particle is live, so a sensor never keeps a reading once the liquid is gone.
static void read_sensors(ymir_particles* p)
{
    for (int32_t s = 0; s < p->sensor_count; ++s)
    {
        if (!p->sensor_live[s])
        {
            continue;
        }
        float volume = 0.0f;
        float pooled = 0.0f;
        if (p->sensor_body[s] < p->body_count)
        {
            const ymir_particle_proxy* b = p->proxies + p->sensor_body[s];
            v3 c = vadd((v3){b->position[0], b->position[1], b->position[2]}, qrot(b->rotation, (v3){p->sensor_x[s], p->sensor_y[s], p->sensor_z[s]}));
            float r2 = p->sensor_radius[s] * p->sensor_radius[s];
            pooled = pooled_volume(p, c, p->sensor_radius[s]);
            for (int32_t i = 0; p->live_fluid > 0 && i < p->particle_count; ++i)
            {
                if (!p->particle_live[i] || !(p->phase[i] & YMIR_PHASE_FLUID))
                {
                    continue;
                }
                v3 d = vsub(P(p, p->pos_x, p->pos_y, p->pos_z, i), c);
                if (vdot(d, d) <= r2)
                {
                    float side = 2.0f * p->radius[i];
                    volume += side * side * side;
                }
            }
        }
        p->sensor_liquid_volume[s] = volume;
        p->sensor_wet_volume[s] = pooled;
    }
}

void ymir_particles_end(ymir_particles* p, int32_t substeps)
{
    if (p == NULL)
    {
        return;
    }
    read_sensors(p);
    if (p->live_particles == 0)
    {
        return;
    }
    float mean = 1.0f / (float)substeps;
    for (int32_t e = 0; e < p->distance_count; ++e)
    {
        p->dist_tension[e] = p->dist_live[e] ? p->dist_acc[e] * mean : 0.0f;
    }
    for (int32_t a = 0; a < p->attachment_count; ++a)
    {
        p->att_force_x[a] *= mean;
        p->att_force_y[a] *= mean;
        p->att_force_z[a] *= mean;
    }
}

// ---- settle: liquid becomes data ----------------------------------------------------------------------------

static float wet_disc_radius(float volume)
{
    return sqrtf(volume / (3.14159265358979f * YMIR_WET_FILM));
}

// The contact point of fluid particle i on its support, in the support's frame: the nearest surface point of the body, or 0 on a solid particle.
// Returns 0 when the particle names no support the proxies can place.
static int wet_support_point(const ymir_particles* p, int32_t i, int32_t* body, int32_t* particle, v3* point)
{
    *body = p->support_body[i];
    *particle = p->support_particle[i];
    *point = (v3){0.0f, 0.0f, 0.0f};
    if (*body >= 0)
    {
        if (*body >= p->body_count || p->proxies[*body].shape == YMIR_PROXY_NONE)
        {
            return 0;
        }
        const ymir_particle_proxy* b = p->proxies + *body;
        v3 n, s;
        proxy_distance(b, P(p, p->pos_x, p->pos_y, p->pos_z, i), &n, &s);
        *point = qrot_inv(b->rotation, vsub(s, (v3){b->position[0], b->position[1], b->position[2]}));
        *particle = -1;
        return 1;
    }
    return *particle >= 0 && *particle < p->particle_count && p->particle_live[*particle];
}

int32_t ymir_particles_settle(ymir_particles* p, int32_t n, const ymir_handle* h, uint64_t step_count, ymir_handle* out)
{
    if (p == NULL || n < 0 || (n > 0 && h == NULL))
    {
        return YMIR_PARTICLES_INVALID_ARGUMENT;
    }
    memset(p->done, 0, (size_t)p->particle_count); // scratch with no baseline: marks the particles named
    for (int32_t k = 0; k < n; ++k)
    {
        int32_t body, particle;
        v3 pt;
        if (!particle_ok(p, h[k].lane, h[k].generation) || !(p->phase[h[k].lane] & YMIR_PHASE_FLUID) || p->done[h[k].lane] ||
            !wet_support_point(p, h[k].lane, &body, &particle, &pt))
        {
            return YMIR_PARTICLES_INVALID_ARGUMENT; // a particle named twice would settle its volume twice
        }
        p->done[h[k].lane] = 1;
    }
    int32_t saved_count = p->wet_count;
    int32_t logged = 0;
    int32_t status = YMIR_PARTICLES_OK;
    for (int32_t k = 0; k < n && status == YMIR_PARTICLES_OK; ++k)
    {
        int32_t i = h[k].lane;
        int32_t body, particle;
        v3 pt;
        wet_support_point(p, i, &body, &particle, &pt);
        uint32_t material = p->material[i];
        float r2 = 2.0f * p->radius[i];
        float volume = r2 * r2 * r2;
        int32_t into = -1;
        int32_t nearest = -1;
        float nearest_d = 0.0f;
        for (int32_t w = 0; w < p->wet_count; ++w)
        {
            if (!p->wet_live[w] || p->wet_body[w] != body || p->wet_particle[w] != particle || p->wet_material[w] != material)
            {
                continue;
            }
            v3 c = {p->wet_x[w], p->wet_y[w], p->wet_z[w]};
            float d = vlen(vsub(pt, c));
            if (d <= p->wet_radius[w] && into < 0)
            {
                into = w; // the lowest patch whose disc holds the point
            }
            if (nearest < 0 || d < nearest_d)
            {
                nearest = w;
                nearest_d = d;
            }
        }
        int created = 0;
        if (into < 0)
        {
            into = next_lane(p->wet_live, 0, p->wet_removed_at, p->wet_count, p->wet_capacity, 0, step_count);
            created = into >= 0;
            if (into < 0)
            {
                into = nearest; // no lane left: the nearest patch on this support takes it
            }
        }
        if (into < 0)
        {
            status = YMIR_PARTICLES_FULL;
            break;
        }
        p->wu_code[logged] = created ? -(into + 1) : into;
        p->wu_x[logged] = p->wet_x[into];
        p->wu_y[logged] = p->wet_y[into];
        p->wu_z[logged] = p->wet_z[into];
        p->wu_volume[logged] = p->wet_volume[into];
        logged++;
        if (created)
        {
            p->wet_live[into] = 1;
            p->wet_body[into] = body;
            p->wet_particle[into] = particle;
            p->wet_material[into] = material;
            p->wet_x[into] = pt.x;
            p->wet_y[into] = pt.y;
            p->wet_z[into] = pt.z;
            p->wet_volume[into] = volume;
            if (into >= p->wet_count)
            {
                p->wet_count = into + 1;
            }
        }
        else
        {
            float total = p->wet_volume[into] + volume;
            p->wet_x[into] = (p->wet_x[into] * p->wet_volume[into] + pt.x * volume) / total; // centre: volume-weighted
            p->wet_y[into] = (p->wet_y[into] * p->wet_volume[into] + pt.y * volume) / total;
            p->wet_z[into] = (p->wet_z[into] * p->wet_volume[into] + pt.z * volume) / total;
            p->wet_volume[into] = total;
        }
        p->wet_radius[into] = wet_disc_radius(p->wet_volume[into]);
        if (out != NULL)
        {
            out[k] = (ymir_handle){into, p->wet_generation[into]};
        }
    }
    if (status != YMIR_PARTICLES_OK)
    {
        for (int32_t u = logged - 1; u >= 0; --u)
        {
            if (p->wu_code[u] < 0)
            {
                wet_reset(p, -p->wu_code[u] - 1);
            }
            else
            {
                int32_t w = p->wu_code[u];
                p->wet_x[w] = p->wu_x[u];
                p->wet_y[w] = p->wu_y[u];
                p->wet_z[w] = p->wu_z[u];
                p->wet_volume[w] = p->wu_volume[u];
                p->wet_radius[w] = wet_disc_radius(p->wu_volume[u]);
            }
        }
        p->wet_count = saved_count;
        return status;
    }
    rederive_wet(p);
    return ymir_particles_destroy_particles(p, n, h, step_count);
}

int32_t ymir_particles_create_sensors(ymir_particles* p, int32_t n, const ymir_sensor_desc* d, int32_t body_count, uint64_t step_count, ymir_handle* out)
{
    if (p == NULL || n < 0 || (n > 0 && d == NULL))
    {
        return YMIR_PARTICLES_INVALID_ARGUMENT;
    }
    for (int32_t k = 0; k < n; ++k)
    {
        if (d[k].body < 0 || d[k].body >= body_count || !finite3(d[k].point) || !isfinite(d[k].radius) || d[k].radius <= 0.0f)
        {
            return YMIR_PARTICLES_INVALID_ARGUMENT;
        }
    }
    if (n > free_count(p->sensor_live, 0, p->sensor_removed_at, p->sensor_count, p->sensor_capacity, step_count))
    {
        return YMIR_PARTICLES_FULL;
    }
    int32_t from = 0;
    for (int32_t k = 0; k < n; ++k)
    {
        int32_t s = next_lane(p->sensor_live, 0, p->sensor_removed_at, p->sensor_count, p->sensor_capacity, from, step_count);
        from = s + 1;
        p->sensor_live[s] = 1;
        p->sensor_body[s] = d[k].body;
        p->sensor_x[s] = d[k].point[0];
        p->sensor_y[s] = d[k].point[1];
        p->sensor_z[s] = d[k].point[2];
        p->sensor_radius[s] = d[k].radius;
        p->sensor_liquid_volume[s] = p->sensor_wet_volume[s] = 0.0f; // the first step's end reads them
        if (s >= p->sensor_count)
        {
            p->sensor_count = s + 1;
        }
        if (out != NULL)
        {
            out[k] = (ymir_handle){s, p->sensor_generation[s]};
        }
    }
    return YMIR_PARTICLES_OK;
}
