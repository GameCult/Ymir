// Particle chains: XPBD with small steps (Macklin 2019), long-range tethers (Kim 2012) and two-way coupling
// to rigid bodies through proxies with real mass (Mueller 2007, Detailed Rigid XPBD 2020). The file calls no
// Box3D and spawns nothing: one thread, every sum in a fixed order. See ymir_particles.h.

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

// ---- memory: one block, laid out once at create -------------------------------------------------------------

static size_t layout(ymir_particles* p, unsigned char* base)
{
    size_t off = 0;
    int64_t np = p->particle_capacity, nc = p->chain_capacity, na = p->attachment_capacity, nb = p->body_capacity;
    size_t table = (size_t)p->table_mask + 1;
#define CARVE(field, type, count)                                                                                      \
    do                                                                                                                 \
    {                                                                                                                  \
        off = (off + 15u) & ~(size_t)15u;                                                                              \
        if (base != NULL)                                                                                              \
            p->field = (type*)(base + off);                                                                            \
        off += sizeof(type) * (size_t)(count);                                                                         \
    } while (0)
    CARVE(proxies, ymir_particle_proxy, nb);
    CARVE(pos_x, float, np); CARVE(pos_y, float, np); CARVE(pos_z, float, np);
    CARVE(vel_x, float, np); CARVE(vel_y, float, np); CARVE(vel_z, float, np);
    CARVE(radius, float, np); CARVE(inv_mass, float, np); CARVE(link_tension, float, np);
    CARVE(chain, int32_t, np);
    CARVE(chain_stable_id, uint64_t, nc); CARVE(chain_kind, uint32_t, nc); CARVE(chain_generation, uint32_t, nc);
    CARVE(chain_first, int32_t, nc); CARVE(chain_size, int32_t, nc); CARVE(chain_removed_at, uint64_t, nc);
    CARVE(chain_material_id, uint64_t, nc);
    CARVE(chain_stretch, float, nc); CARVE(chain_bend, float, nc); CARVE(chain_friction, float, nc);
    CARVE(att_kind, uint32_t, na); CARVE(att_generation, uint32_t, na); CARVE(att_particle, int32_t, na);
    CARVE(att_body, int32_t, na);
    CARVE(att_point_x, float, na); CARVE(att_point_y, float, na); CARVE(att_point_z, float, na);
    CARVE(att_compliance, float, na);
    CARVE(att_force_x, float, na); CARVE(att_force_y, float, na); CARVE(att_force_z, float, na);
    CARVE(att_removed_at, uint64_t, na);
    CARVE(pp_x, float, np); CARVE(pp_y, float, np); CARVE(pp_z, float, np);
    CARVE(dp_x, float, np); CARVE(dp_y, float, np); CARVE(dp_z, float, np);
    CARVE(rest_link, float, np); CARVE(rest_bend, float, np); CARVE(rest_pos, float, np);
    CARVE(tension_acc, float, np); CARVE(struct_x, float, np); CARVE(struct_y, float, np); CARVE(struct_z, float, np);
    CARVE(skip_body, int32_t, np); CARVE(cell_x, int32_t, np); CARVE(cell_y, int32_t, np); CARVE(cell_z, int32_t, np);
    CARVE(cell_start, int32_t, table + 1); CARVE(cell_fill, int32_t, table); CARVE(sorted, int32_t, np);
    CARVE(mark, uint8_t, np);
#undef CARVE
    return off + 16;
}

ymir_particles* ymir_particles_create(int32_t particle_capacity, int32_t chain_capacity, int32_t attachment_capacity, int32_t body_capacity)
{
    if (particle_capacity < 0 || chain_capacity < 0 || attachment_capacity < 0 || body_capacity < 0)
    {
        return NULL;
    }
    ymir_particles* p = (ymir_particles*)calloc(1, sizeof(ymir_particles));
    if (p == NULL)
    {
        return NULL;
    }
    p->particle_capacity = particle_capacity;
    p->chain_capacity = chain_capacity;
    p->attachment_capacity = attachment_capacity;
    p->body_capacity = body_capacity;
    uint32_t table = 16;
    while (table < 2u * (uint32_t)particle_capacity)
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
    for (int32_t i = 0; i < particle_capacity; ++i)
    {
        p->chain[i] = -1;
    }
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

// ---- slots ---------------------------------------------------------------------------------------------------

static int lane_reusable(uint64_t step_count, uint64_t removed_at)
{
    return step_count > removed_at;
}

// Marks the particle slots a new chain may not take: occupied ones, and the range of a chain destroyed in this
// very step (its lanes were read this step and stay meaningful until a step has passed).
static void mark_slots(const ymir_particles* p, uint64_t step_count)
{
    for (int32_t i = 0; i < p->particle_capacity; ++i)
    {
        p->mark[i] = (uint8_t)(i < p->particle_count && p->chain[i] >= 0);
    }
    for (int32_t c = 0; c < p->chain_count; ++c)
    {
        if (p->chain_kind[c] == YMIR_CHAIN_REMOVED && !lane_reusable(step_count, p->chain_removed_at[c]))
        {
            for (int32_t i = p->chain_first[c]; i < p->chain_first[c] + p->chain_size[c]; ++i)
            {
                p->mark[i] = 1;
            }
        }
    }
}

// First fit, lowest index. Returns -1 when no run of `n` slots is free.
static int32_t find_range(const ymir_particles* p, int32_t n, uint64_t step_count)
{
    mark_slots(p, step_count);
    int32_t run = 0;
    for (int32_t i = 0; i < p->particle_capacity; ++i)
    {
        run = p->mark[i] ? 0 : run + 1;
        if (run == n)
        {
            return i - n + 1;
        }
    }
    return -1;
}

int32_t ymir_particles_free_particle_slots(const ymir_particles* p, uint64_t step_count)
{
    mark_slots(p, step_count);
    int32_t best = 0, run = 0;
    for (int32_t i = 0; i < p->particle_capacity; ++i)
    {
        run = p->mark[i] ? 0 : run + 1;
        best = run > best ? run : best;
    }
    return best;
}

static int32_t chain_slot(const ymir_particles* p, uint64_t step_count)
{
    for (int32_t c = 0; c < p->chain_count; ++c)
    {
        if (p->chain_kind[c] == YMIR_CHAIN_REMOVED && lane_reusable(step_count, p->chain_removed_at[c]))
        {
            return c;
        }
    }
    return p->chain_count < p->chain_capacity ? p->chain_count : -1;
}

int32_t ymir_particles_free_chain_slots(const ymir_particles* p, uint64_t step_count)
{
    int32_t free_slots = p->chain_capacity - p->chain_count;
    for (int32_t c = 0; c < p->chain_count; ++c)
    {
        free_slots += p->chain_kind[c] == YMIR_CHAIN_REMOVED && lane_reusable(step_count, p->chain_removed_at[c]);
    }
    return free_slots;
}

static int32_t attachment_slot(const ymir_particles* p, uint64_t step_count)
{
    for (int32_t a = 0; a < p->attachment_count; ++a)
    {
        if (p->att_kind[a] == YMIR_ATTACH_REMOVED && lane_reusable(step_count, p->att_removed_at[a]))
        {
            return a;
        }
    }
    return p->attachment_count < p->attachment_capacity ? p->attachment_count : -1;
}

int32_t ymir_particles_free_attachment_slots(const ymir_particles* p, uint64_t step_count)
{
    int32_t free_slots = p->attachment_capacity - p->attachment_count;
    for (int32_t a = 0; a < p->attachment_count; ++a)
    {
        free_slots += p->att_kind[a] == YMIR_ATTACH_REMOVED && lane_reusable(step_count, p->att_removed_at[a]);
    }
    return free_slots;
}

static int chain_live(const ymir_particles* p, int32_t chain, uint32_t generation)
{
    return chain >= 0 && chain < p->chain_count && p->chain_kind[chain] == YMIR_CHAIN_LIVE && p->chain_generation[chain] == generation;
}

static int stable_id_taken(const ymir_particles* p, uint64_t id)
{
    for (int32_t c = 0; c < p->chain_count; ++c)
    {
        if (p->chain_kind[c] == YMIR_CHAIN_LIVE && p->chain_stable_id[c] == id)
        {
            return 1;
        }
    }
    return 0;
}

// ---- commands ------------------------------------------------------------------------------------------------

int32_t ymir_particles_create_chain(ymir_particles* p, const ymir_chain_desc* d, uint64_t step_count, int32_t* chain, uint32_t* generation)
{
    if (p == NULL || d == NULL || d->positions == NULL || d->masses == NULL || d->count < 2 || !isfinite(d->radius) ||
        d->radius <= 0.0f || !isfinite(d->stretch_compliance) || d->stretch_compliance < 0.0f ||
        !isfinite(d->bend_compliance) || d->bend_compliance < 0.0f || !isfinite(d->friction) || d->friction < 0.0f)
    {
        return YMIR_PARTICLES_INVALID_ARGUMENT;
    }
    for (int32_t i = 0; i < d->count; ++i)
    {
        if (!finite3(d->positions + 3 * i) || !isfinite(d->masses[i]) || d->masses[i] < 0.0f)
        {
            return YMIR_PARTICLES_INVALID_ARGUMENT;
        }
        if (i + 1 < d->count)
        {
            v3 a = {d->positions[3 * i], d->positions[3 * i + 1], d->positions[3 * i + 2]};
            v3 b = {d->positions[3 * i + 3], d->positions[3 * i + 4], d->positions[3 * i + 5]};
            if (!(vlen(vsub(b, a)) > 1e-6f))
            {
                return YMIR_PARTICLES_INVALID_ARGUMENT; // a link of no length has no direction
            }
        }
    }
    if (stable_id_taken(p, d->stable_id))
    {
        return YMIR_PARTICLES_DUPLICATE_ID;
    }
    int32_t lane = chain_slot(p, step_count);
    int32_t first = find_range(p, d->count, step_count);
    if (lane < 0 || first < 0)
    {
        return YMIR_PARTICLES_FULL;
    }

    if (lane == p->chain_count)
    {
        p->chain_count++;
    }
    p->live_chains++;
    p->chain_kind[lane] = YMIR_CHAIN_LIVE;
    p->chain_stable_id[lane] = d->stable_id;
    p->chain_first[lane] = first;
    p->chain_size[lane] = d->count;
    p->chain_removed_at[lane] = 0;
    p->chain_material_id[lane] = d->material_id;
    p->chain_stretch[lane] = d->stretch_compliance;
    p->chain_bend[lane] = d->bend_compliance;
    p->chain_friction[lane] = d->friction;
    if (first + d->count > p->particle_count)
    {
        p->particle_count = first + d->count;
    }
    float path = 0.0f;
    for (int32_t k = 0; k < d->count; ++k)
    {
        int32_t i = first + k;
        v3 x = {d->positions[3 * k], d->positions[3 * k + 1], d->positions[3 * k + 2]};
        put(p->pos_x, p->pos_y, p->pos_z, i, x);
        p->vel_x[i] = p->vel_y[i] = p->vel_z[i] = 0.0f;
        p->radius[i] = d->radius;
        p->inv_mass[i] = d->masses[k] > 0.0f ? 1.0f / d->masses[k] : 0.0f;
        p->link_tension[i] = 0.0f;
        p->chain[i] = lane;
        p->rest_pos[i] = path;
        p->rest_link[i] = 0.0f;
        p->rest_bend[i] = 0.0f;
        if (k + 1 < d->count)
        {
            v3 y = {d->positions[3 * k + 3], d->positions[3 * k + 4], d->positions[3 * k + 5]};
            p->rest_link[i] = vlen(vsub(y, x));
            path += p->rest_link[i];
        }
        if (k + 2 < d->count)
        {
            v3 z = {d->positions[3 * k + 6], d->positions[3 * k + 7], d->positions[3 * k + 8]};
            p->rest_bend[i] = vlen(vsub(z, x));
        }
    }
    if (chain != NULL)
    {
        *chain = lane;
    }
    if (generation != NULL)
    {
        *generation = p->chain_generation[lane];
    }
    return YMIR_PARTICLES_OK;
}

// Cuts the link between particles `link` and `link + 1`. The head keeps its lane and stable id; the tail takes a
// new chain lane and `tail_stable_id`. No particle moves lane or address, and attachments stay where they are.
int32_t ymir_particles_split_chain(
    ymir_particles* p, int32_t chain, uint32_t generation, int32_t link, uint64_t tail_stable_id, uint64_t step_count,
    int32_t* tail, uint32_t* tail_generation)
{
    if (p == NULL || !chain_live(p, chain, generation) || link < 0 || link + 2 > p->chain_size[chain])
    {
        return YMIR_PARTICLES_INVALID_ARGUMENT;
    }
    if (stable_id_taken(p, tail_stable_id))
    {
        return YMIR_PARTICLES_DUPLICATE_ID;
    }
    int32_t lane = chain_slot(p, step_count);
    if (lane < 0)
    {
        return YMIR_PARTICLES_FULL;
    }
    if (lane == p->chain_count)
    {
        p->chain_count++;
    }
    p->live_chains++;
    int32_t first = p->chain_first[chain], size = p->chain_size[chain];
    int32_t tail_first = first + link + 1;
    p->chain_kind[lane] = YMIR_CHAIN_LIVE;
    p->chain_stable_id[lane] = tail_stable_id;
    p->chain_first[lane] = tail_first;
    p->chain_size[lane] = size - link - 1;
    p->chain_removed_at[lane] = 0;
    p->chain_material_id[lane] = p->chain_material_id[chain];
    p->chain_stretch[lane] = p->chain_stretch[chain];
    p->chain_bend[lane] = p->chain_bend[chain];
    p->chain_friction[lane] = p->chain_friction[chain];
    p->chain_size[chain] = link + 1;
    for (int32_t i = tail_first; i < first + size; ++i)
    {
        p->chain[i] = lane;
    }
    p->link_tension[tail_first - 1] = 0.0f; // the cut link carries nothing
    if (tail != NULL)
    {
        *tail = lane;
    }
    if (tail_generation != NULL)
    {
        *tail_generation = p->chain_generation[lane];
    }
    return YMIR_PARTICLES_OK;
}

static void tombstone_attachment(ymir_particles* p, int32_t a, uint64_t step_count)
{
    p->att_kind[a] = YMIR_ATTACH_REMOVED;
    p->att_removed_at[a] = step_count;
    p->att_generation[a] += 1;
    p->att_particle[a] = -1;
    p->att_body[a] = -1;
    p->att_point_x[a] = p->att_point_y[a] = p->att_point_z[a] = 0.0f;
    p->att_compliance[a] = 0.0f;
    p->att_force_x[a] = p->att_force_y[a] = p->att_force_z[a] = 0.0f;
}

// Removes the chain and, with it, every attachment that reaches one of its particles: they would dangle. A
// consequence of the command, not a decision of the solver. Particle lanes read 0 until the range is reused.
int32_t ymir_particles_destroy_chain(ymir_particles* p, int32_t chain, uint32_t generation, uint64_t step_count)
{
    if (p == NULL || !chain_live(p, chain, generation))
    {
        return YMIR_PARTICLES_INVALID_ARGUMENT;
    }
    int32_t lo = p->chain_first[chain], hi = lo + p->chain_size[chain];
    for (int32_t a = 0; a < p->attachment_count; ++a)
    {
        if (p->att_kind[a] == YMIR_ATTACH_REMOVED)
        {
            continue;
        }
        int32_t q = p->att_particle[a];
        int32_t other = p->att_kind[a] == YMIR_ATTACH_PARTICLE_PARTICLE ? p->att_body[a] : -1;
        if ((q >= lo && q < hi) || (other >= lo && other < hi))
        {
            tombstone_attachment(p, a, step_count);
        }
    }
    for (int32_t i = lo; i < hi; ++i)
    {
        p->pos_x[i] = p->pos_y[i] = p->pos_z[i] = 0.0f;
        p->vel_x[i] = p->vel_y[i] = p->vel_z[i] = 0.0f;
        p->radius[i] = p->inv_mass[i] = p->link_tension[i] = 0.0f;
        p->rest_link[i] = p->rest_bend[i] = p->rest_pos[i] = 0.0f;
        p->chain[i] = -1;
    }
    p->chain_kind[chain] = YMIR_CHAIN_REMOVED; // first and size stay: the range is held until a step has passed
    p->chain_stable_id[chain] = 0;
    p->chain_removed_at[chain] = step_count;
    p->chain_generation[chain] += 1;
    p->chain_material_id[chain] = 0;
    p->chain_stretch[chain] = p->chain_bend[chain] = p->chain_friction[chain] = 0.0f;
    p->live_chains--;
    return YMIR_PARTICLES_OK;
}

int32_t ymir_particles_attach(ymir_particles* p, const ymir_attachment_desc* d, int32_t body_count, uint64_t step_count, int32_t* attachment, uint32_t* generation)
{
    if (p == NULL || d == NULL || d->kind > YMIR_ATTACH_PARTICLE_PARTICLE || d->particle < 0 || d->particle >= p->particle_count ||
        p->chain[d->particle] < 0 || !finite3(d->point) || !isfinite(d->compliance) || d->compliance < 0.0f)
    {
        return YMIR_PARTICLES_INVALID_ARGUMENT;
    }
    if (d->kind == YMIR_ATTACH_PARTICLE_BODY && (d->body < 0 || d->body >= body_count))
    {
        return YMIR_PARTICLES_INVALID_ARGUMENT;
    }
    if (d->kind == YMIR_ATTACH_PARTICLE_PARTICLE &&
        (d->body < 0 || d->body >= p->particle_count || d->body == d->particle || p->chain[d->body] < 0))
    {
        return YMIR_PARTICLES_INVALID_ARGUMENT;
    }
    int32_t a = attachment_slot(p, step_count);
    if (a < 0)
    {
        return YMIR_PARTICLES_FULL;
    }
    if (a == p->attachment_count)
    {
        p->attachment_count++;
    }
    p->att_kind[a] = d->kind;
    p->att_particle[a] = d->particle;
    p->att_body[a] = d->kind == YMIR_ATTACH_PARTICLE_STATIC ? -1 : d->body;
    p->att_point_x[a] = d->point[0];
    p->att_point_y[a] = d->point[1];
    p->att_point_z[a] = d->point[2];
    p->att_compliance[a] = d->compliance;
    p->att_force_x[a] = p->att_force_y[a] = p->att_force_z[a] = 0.0f;
    if (attachment != NULL)
    {
        *attachment = a;
    }
    if (generation != NULL)
    {
        *generation = p->att_generation[a];
    }
    return YMIR_PARTICLES_OK;
}

int32_t ymir_particles_detach(ymir_particles* p, int32_t attachment, uint32_t generation, uint64_t step_count)
{
    if (p == NULL || attachment < 0 || attachment >= p->attachment_count || p->att_kind[attachment] == YMIR_ATTACH_REMOVED ||
        p->att_generation[attachment] != generation)
    {
        return YMIR_PARTICLES_INVALID_ARGUMENT;
    }
    tombstone_attachment(p, attachment, step_count);
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
    if (p->att_kind[a] == YMIR_ATTACH_PARTICLE_BODY)
    {
        const ymir_particle_proxy* b = p->proxies + p->att_body[a];
        v3 local = {p->att_point_x[a], p->att_point_y[a], p->att_point_z[a]};
        return vadd((v3){b->position[0], b->position[1], b->position[2]}, qrot(b->rotation, local));
    }
    if (p->att_kind[a] == YMIR_ATTACH_PARTICLE_PARTICLE)
    {
        return P(p, p->pp_x, p->pp_y, p->pp_z, p->att_body[a]);
    }
    return (v3){p->att_point_x[a], p->att_point_y[a], p->att_point_z[a]};
}

static float end_w(const ymir_particles* p, int32_t a, v3 at, v3 n)
{
    if (p->att_kind[a] == YMIR_ATTACH_PARTICLE_BODY)
    {
        return proxy_w(p->proxies + p->att_body[a], at, n);
    }
    if (p->att_kind[a] == YMIR_ATTACH_PARTICLE_PARTICLE)
    {
        return p->inv_mass[p->att_body[a]];
    }
    return 0.0f;
}

static void end_push(const phase* f, int32_t a, v3 at, v3 imp)
{
    ymir_particles* p = f->p;
    if (p->att_kind[a] == YMIR_ATTACH_PARTICLE_BODY)
    {
        proxy_push(f, p->proxies + p->att_body[a], at, imp);
    }
    else if (p->att_kind[a] == YMIR_ATTACH_PARTICLE_PARTICLE)
    {
        int32_t q = p->att_body[a];
        put(p->pp_x, p->pp_y, p->pp_z, q, vadd(P(p, p->pp_x, p->pp_y, p->pp_z, q), vmul(imp, p->inv_mass[q])));
    }
}

// Adds a force (N) the attachment `a` exerts on the chain to its sensor lane.
static void add_force(ymir_particles* p, int32_t a, v3 force)
{
    p->att_force_x[a] += force.x;
    p->att_force_y[a] += force.y;
    p->att_force_z[a] += force.z;
}

// Books a structural position impulse (kg m) the chain delivers to its particle `i`: links, bends and tethers.
// Every booking has an equal and opposite one, so the sum over the particles past a link is the load the link
// carries (momentum balance), however the solver routed it.
static void add_struct(ymir_particles* p, int32_t i, v3 imp)
{
    p->struct_x[i] += imp.x;
    p->struct_y[i] += imp.y;
    p->struct_z[i] += imp.z;
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

// Counting sort of the live particles' predicted positions into a hash grid. Within a bucket particles keep
// index order, so every neighbour walk is the same on every run.
static float build_grid(ymir_particles* p)
{
    float rmax = 0.0f;
    for (int32_t i = 0; i < p->particle_count; ++i)
    {
        if (p->chain[i] >= 0)
        {
            rmax = vmax(rmax, p->radius[i]);
        }
    }
    float inv_cell = 1.0f / (2.0f * rmax);
    int32_t buckets = (int32_t)p->table_mask + 1;
    memset(p->cell_start, 0, sizeof(int32_t) * (size_t)(buckets + 1));
    for (int32_t i = 0; i < p->particle_count; ++i)
    {
        if (p->chain[i] < 0)
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
        if (p->chain[i] >= 0)
        {
            uint32_t c = cell_hash(p->cell_x[i], p->cell_y[i], p->cell_z[i], p->table_mask);
            p->sorted[p->cell_start[c] + p->cell_fill[c]++] = i;
        }
    }
    return rmax;
}

// Particles of different chains, or of one chain more than two links apart, push each other apart. Gather
// form: particle i reads its neighbours' predicted positions and writes only its own correction.
static void particle_contacts(ymir_particles* p)
{
    for (int32_t i = 0; i < p->particle_count; ++i)
    {
        p->dp_x[i] = p->dp_y[i] = p->dp_z[i] = 0.0f;
        if (p->chain[i] < 0 || p->inv_mass[i] == 0.0f)
        {
            continue;
        }
        v3 xi = P(p, p->pp_x, p->pp_y, p->pp_z, i);
        v3 sum = {0.0f, 0.0f, 0.0f};
        int32_t contacts = 0;
        for (int32_t da = -1; da <= 1; ++da)
        {
            for (int32_t db = -1; db <= 1; ++db)
            {
                for (int32_t dc = -1; dc <= 1; ++dc)
                {
                    int32_t ca = p->cell_x[i] + da, cb = p->cell_y[i] + db, cc = p->cell_z[i] + dc;
                    uint32_t bucket = cell_hash(ca, cb, cc, p->table_mask);
                    for (int32_t q = p->cell_start[bucket]; q < p->cell_start[bucket + 1]; ++q)
                    {
                        int32_t j = p->sorted[q];
                        if (j == i || p->cell_x[j] != ca || p->cell_y[j] != cb || p->cell_z[j] != cc)
                        {
                            continue; // another cell sharing the bucket
                        }
                        if (p->chain[j] == p->chain[i] && (j - i <= 2 && i - j <= 2))
                        {
                            continue;
                        }
                        v3 d = vsub(xi, P(p, p->pp_x, p->pp_y, p->pp_z, j));
                        float md = p->radius[i] + p->radius[j];
                        float r2 = vdot(d, d);
                        float wsum = p->inv_mass[i] + p->inv_mass[j];
                        if (r2 >= md * md || r2 < 1e-12f || wsum == 0.0f)
                        {
                            continue;
                        }
                        float rr = sqrtf(r2);
                        sum = vadd(sum, vmul(d, (p->inv_mass[i] / wsum) * (md - rr) / rr));
                        contacts++;
                    }
                }
            }
        }
        if (contacts > 0)
        {
            put(p->dp_x, p->dp_y, p->dp_z, i, vmul(sum, 1.0f / (float)contacts)); // averaged: a Jacobi sum overshoots
        }
    }
    for (int32_t i = 0; i < p->particle_count; ++i)
    {
        if (p->chain[i] >= 0)
        {
            p->pp_x[i] += p->dp_x[i];
            p->pp_y[i] += p->dp_y[i];
            p->pp_z[i] += p->dp_z[i];
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

// Each particle against each shaped body in ascending lane order. The correction is split by inverse mass between
// the particle and the body at the contact point; Coulomb friction removes the tangential slip up to mu times the
// push. The body the particle is attached to is skipped: the attachment already ties them.
static void body_contacts(phase* f)
{
    ymir_particles* p = f->p;
    for (int32_t i = 0; i < p->particle_count; ++i)
    {
        int32_t chain = p->chain[i];
        if (chain < 0 || p->inv_mass[i] == 0.0f)
        {
            continue;
        }
        float mu = p->chain_friction[chain];
        float r = p->radius[i];
        float wi = p->inv_mass[i];
        for (int32_t k = 0; k < p->body_count; ++k)
        {
            ymir_particle_proxy* b = p->proxies + k;
            if (b->shape == YMIR_PROXY_NONE || k == p->skip_body[i])
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
            v3 n, s;
            float d = proxy_distance(b, x, &n, &s);
            if (d >= r)
            {
                continue;
            }
            float depth = r - d;
            float wb = proxy_w(b, s, n);
            float w = wi + wb;
            x = vadd(x, vmul(n, depth * wi / w));
            proxy_push(f, b, s, vmul(n, -depth / w));
            if (mu > 0.0f)
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
                    float move = vmin(sl, mu * depth);
                    float wt = wi + proxy_w(b, s, t);
                    x = vsub(x, vmul(t, move * wi / wt));
                    proxy_push(f, b, s, vmul(t, move / wt));
                }
            }
            put(p->pp_x, p->pp_y, p->pp_z, i, x);
        }
    }
}

// ---- attachments, chain constraints, tethers -----------------------------------------------------------------

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

// One distance constraint between two particles; returns lambda (negative when it pulled them together).
static float distance_constraint(ymir_particles* p, int32_t i, int32_t j, float rest, float alpha)
{
    float wi = p->inv_mass[i], wj = p->inv_mass[j];
    v3 d = vsub(P(p, p->pp_x, p->pp_y, p->pp_z, j), P(p, p->pp_x, p->pp_y, p->pp_z, i));
    float len = vlen(d);
    if (len < 1e-9f || wi + wj + alpha == 0.0f)
    {
        return 0.0f;
    }
    v3 n = vmul(d, 1.0f / len);
    float dl = -(len - rest) / (wi + wj + alpha);
    put(p->pp_x, p->pp_y, p->pp_z, i, vsub(P(p, p->pp_x, p->pp_y, p->pp_z, i), vmul(n, wi * dl)));
    put(p->pp_x, p->pp_y, p->pp_z, j, vadd(P(p, p->pp_x, p->pp_y, p->pp_z, j), vmul(n, wj * dl)));
    add_struct(p, i, vmul(n, -dl));
    add_struct(p, j, vmul(n, dl));
    return dl;
}

// Each chain in lane order, links in index order (Gauss-Seidel), stretch then bend. One iteration per substep:
// the small step does the converging. Stretch lambda is the link's tension (positive when stretched).
static void solve_chains(phase* f)
{
    ymir_particles* p = f->p;
    float hh = f->h * f->h;
    for (int32_t c = 0; c < p->chain_count; ++c)
    {
        if (p->chain_kind[c] != YMIR_CHAIN_LIVE)
        {
            continue;
        }
        int32_t first = p->chain_first[c], last = first + p->chain_size[c];
        for (int32_t i = first; i + 1 < last; ++i)
        {
            distance_constraint(p, i, i + 1, p->rest_link[i], p->chain_stretch[c] / hh);
        }
        for (int32_t i = first; i + 2 < last; ++i)
        {
            distance_constraint(p, i, i + 2, p->rest_bend[i], p->chain_bend[c] / hh);
        }
    }
}

static int tetherable(const ymir_particles* p, int32_t a)
{
    return p->att_kind[a] == YMIR_ATTACH_PARTICLE_BODY || p->att_kind[a] == YMIR_ATTACH_PARTICLE_STATIC;
}

// Long-range attachments: a unilateral distance bound at the chain's rest path length from each attachment point
// to every particle of its chain, and between the attachment points of one chain. They bear load past the links,
// so their structural impulses are booked on the particles they pull and the sensors read the load, not the path.
static void solve_tethers(phase* f)
{
    ymir_particles* p = f->p;
    for (int32_t a = 0; a < p->attachment_count; ++a)
    {
        if (!tetherable(p, a))
        {
            continue;
        }
        int32_t i = p->att_particle[a], c = p->chain[i];
        int32_t first = p->chain_first[c], last = first + p->chain_size[c];
        for (int32_t j = first; j < last; ++j)
        {
            if (j == i)
            {
                continue;
            }
            float bound = fabsf(p->rest_pos[j] - p->rest_pos[i]);
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
            add_force(p, a, vmul(n, dl * f->k)); // the force on the chain, toward the attachment
            add_struct(p, j, vmul(n, dl));        // the chain between i and j pulls j toward i
            add_struct(p, i, vmul(n, -dl));       // and i toward j
        }
    }
    for (int32_t a = 0; a < p->attachment_count; ++a)
    {
        if (!tetherable(p, a))
        {
            continue;
        }
        for (int32_t b = a + 1; b < p->attachment_count; ++b)
        {
            if (!tetherable(p, b) || p->chain[p->att_particle[a]] != p->chain[p->att_particle[b]] ||
                p->att_particle[a] == p->att_particle[b])
            {
                continue;
            }
            float bound = fabsf(p->rest_pos[p->att_particle[a]] - p->rest_pos[p->att_particle[b]]);
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
            add_force(p, a, vmul(n, -dl * f->k)); // each point pulls the chain away from the other
            add_force(p, b, vmul(n, dl * f->k));
            add_struct(p, p->att_particle[a], vmul(n, dl)); // the chain between pulls each end toward the other
            add_struct(p, p->att_particle[b], vmul(n, -dl));
        }
    }
}

// ---- the step ------------------------------------------------------------------------------------------------

int32_t ymir_particles_begin(ymir_particles* p)
{
    if (p == NULL || p->live_chains == 0)
    {
        return 0;
    }
    for (int32_t k = 0; k < p->body_count; ++k)
    {
        ymir_particle_proxy* b = p->proxies + k;
        memset(b->impulse, 0, sizeof(b->impulse));
        memset(b->angular_impulse, 0, sizeof(b->angular_impulse));
    }
    for (int32_t i = 0; i < p->particle_count; ++i)
    {
        p->skip_body[i] = -1;
        p->tension_acc[i] = 0.0f;
    }
    for (int32_t a = 0; a < p->attachment_count; ++a)
    {
        p->att_force_x[a] = p->att_force_y[a] = p->att_force_z[a] = 0.0f;
        if (p->att_kind[a] == YMIR_ATTACH_PARTICLE_BODY)
        {
            p->skip_body[p->att_particle[a]] = p->att_body[a];
        }
    }
    return 1;
}

// One substep of `h` against the proxies' poses at its start and velocities after the rigid solve: proxies are
// predicted like particles, every constraint moves both sides by inverse mass, and each proxy's correction returns
// as sub_impulse (its correction over h) for the caller to apply before the rigid positions integrate. Position,
// velocity and momentum stay one state: XPBD's velocity update, done by the rigid integrator.
void ymir_particles_substep(ymir_particles* p, float h, float gx, float gy, float gz)
{
    if (p == NULL || p->live_chains == 0)
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
        if (p->chain[i] < 0)
        {
            continue;
        }
        v3 x = P(p, p->pos_x, p->pos_y, p->pos_z, i);
        if (p->inv_mass[i] > 0.0f)
        {
            v3 v = vadd(P(p, p->vel_x, p->vel_y, p->vel_z, i), vmul(g, h));
            put(p->vel_x, p->vel_y, p->vel_z, i, v);
            x = vadd(x, vmul(v, h));
        }
        put(p->pp_x, p->pp_y, p->pp_z, i, x);
        p->struct_x[i] = p->struct_y[i] = p->struct_z[i] = 0.0f;
    }
    build_grid(p);
    particle_contacts(p);
    solve_attachments(&f);
    solve_chains(&f);
    solve_tethers(&f);
    body_contacts(&f);
    // Link tension by momentum balance: what the structure delivered to the particles past the link, along it.
    for (int32_t c = 0; c < p->chain_count; ++c)
    {
        if (p->chain_kind[c] != YMIR_CHAIN_LIVE)
        {
            continue;
        }
        int32_t first = p->chain_first[c], last = first + p->chain_size[c];
        v3 past = {0.0f, 0.0f, 0.0f};
        for (int32_t i = last - 2; i >= first; --i)
        {
            past = vadd(past, P(p, p->struct_x, p->struct_y, p->struct_z, i + 1));
            v3 d = vsub(P(p, p->pp_x, p->pp_y, p->pp_z, i + 1), P(p, p->pp_x, p->pp_y, p->pp_z, i));
            float len = vlen(d);
            if (len > 1e-9f)
            {
                p->tension_acc[i] += -vdot(past, d) / len * f.k;
            }
        }
    }
    for (int32_t i = 0; i < p->particle_count; ++i)
    {
        if (p->chain[i] < 0)
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

void ymir_particles_end(ymir_particles* p, int32_t substeps)
{
    if (p == NULL || p->live_chains == 0)
    {
        return;
    }
    float mean = 1.0f / (float)substeps;
    for (int32_t i = 0; i < p->particle_count; ++i)
    {
        p->link_tension[i] = p->chain[i] >= 0 ? p->tension_acc[i] * mean : 0.0f;
    }
    for (int32_t c = 0; c < p->chain_count; ++c) // the last particle of a chain starts no link
    {
        if (p->chain_kind[c] == YMIR_CHAIN_LIVE)
        {
            p->link_tension[p->chain_first[c] + p->chain_size[c] - 1] = 0.0f;
        }
    }
    for (int32_t a = 0; a < p->attachment_count; ++a)
    {
        p->att_force_x[a] *= mean;
        p->att_force_y[a] *= mean;
        p->att_force_z[a] *= mean;
    }
}
