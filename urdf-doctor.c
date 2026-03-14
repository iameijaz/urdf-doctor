/*
 * urdf-doctor — Static analysis tool for URDF robot description files
 *
 * Checks for common errors, inconsistencies, and simulation pitfalls
 * without requiring a ROS2 installation.
 *
 * Author: Ijaz Ahmed [Verbit]
 * License: MIT
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif
#include <ctype.h>
#include <stdarg.h>
#ifdef _WIN32
#  define strcasecmp _stricmp
#else
#  include <strings.h>
#endif

#define VERSION "1.0.0"
#define MAX_LINKS     512
#define MAX_JOINTS    512
#define MAX_MATERIALS 128
#define MAX_NAME      256
#define MAX_PATH      1024
#define MAX_ERRORS    1024

/* ── Severity levels ─────────────────────────────────────────── */
typedef enum {
    SEV_ERROR   = 0,   /* will crash or break simulation */
    SEV_WARNING = 1,   /* likely wrong, may cause problems */
    SEV_INFO    = 2,   /* style/portability issues */
} severity_t;

static const char *SEV_LABEL[] = { "ERROR", "WARN ", "INFO " };
static const char *SEV_COLOR[] = { "\033[0;31m", "\033[0;33m", "\033[0;34m" };
#define RST "\033[0m"
#define GRN "\033[0;32m"
#define DIM "\033[2m"

/* ── Diagnostic record ───────────────────────────────────────── */
typedef struct {
    severity_t  sev;
    char        context[MAX_NAME];  /* link/joint name */
    char        msg[512];
} diag_t;

static diag_t  g_diags[MAX_ERRORS];
static int     g_ndiags = 0;
static int     g_no_color = 0;

static void add_diag(severity_t sev, const char *ctx, const char *fmt, ...) {
    if (g_ndiags >= MAX_ERRORS) return;
    diag_t *d = &g_diags[g_ndiags++];
    d->sev = sev;
    strncpy(d->context, ctx ? ctx : "", MAX_NAME - 1);
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(d->msg, sizeof(d->msg), fmt, ap);
    va_end(ap);
}

#define ERR(ctx, ...)  add_diag(SEV_ERROR,   ctx, __VA_ARGS__)
#define WARN(ctx, ...) add_diag(SEV_WARNING, ctx, __VA_ARGS__)
#define INFO(ctx, ...) add_diag(SEV_INFO,    ctx, __VA_ARGS__)

/* ── Data structures ─────────────────────────────────────────── */

typedef struct {
    double x, y, z;
} vec3_t;

typedef struct {
    double ixx, ixy, ixz, iyy, iyz, izz;
    double mass;
    vec3_t com;
    int    has_inertial;
    int    has_mass;
    int    has_inertia;
} inertial_t;

typedef struct {
    char     name[MAX_NAME];
    int      has_visual;
    int      has_collision;
    int      has_inertial;
    double   visual_size;       /* rough bounding estimate */
    double   collision_size;
    inertial_t inertial;
    int      in_tree;           /* connected to kinematic chain */
    int      is_root;
    char     visual_mesh[MAX_PATH];
    char     collision_mesh[MAX_PATH];
    char     material[MAX_NAME];
} link_t;

typedef enum {
    JOINT_UNKNOWN, JOINT_REVOLUTE, JOINT_CONTINUOUS, JOINT_PRISMATIC,
    JOINT_FIXED, JOINT_FLOATING, JOINT_PLANAR
} joint_type_t;

typedef struct {
    char         name[MAX_NAME];
    joint_type_t type;
    char         parent[MAX_NAME];
    char         child[MAX_NAME];
    vec3_t       axis;
    int          has_axis;
    int          has_origin;
    int          has_limits;
    double       lower, upper;
    double       effort, velocity;
    int          has_safety;
    double       soft_lower, soft_upper;
} joint_t;

typedef struct {
    char name[MAX_NAME];
    int  defined;
} material_t;

static link_t     g_links[MAX_LINKS];
static int        g_nlinks = 0;
static joint_t    g_joints[MAX_JOINTS];
static int        g_njoints = 0;
static material_t g_materials[MAX_MATERIALS];
static int        g_nmaterials = 0;
static char       g_robot_name[MAX_NAME] = "";

/* ── XML parser — minimal hand-rolled ───────────────────────── */

/* Find first occurrence of tag in xml, returns pointer to char after '>' */
static const char *find_tag(const char *xml, const char *tag) {
    char open[MAX_NAME];
    snprintf(open, sizeof(open), "<%s", tag);
    size_t taglen = strlen(open);
    const char *p = xml;
    while ((p = strstr(p, open)) != NULL) {
        /* verify next char is whitespace, '/', or '>' to avoid prefix matches */
        char next = p[taglen];
        if (next == ' ' || next == '\t' || next == '\n' || next == '\r' ||
            next == '/' || next == '>') return p;
        p++;
    }
    return NULL;
}

/* Extract attribute value from a tag string into buf */
static int get_attr(const char *tag_start, const char *attr, char *buf, int bufsz) {
    char search[MAX_NAME];
    snprintf(search, sizeof(search), "%s=\"", attr);
    const char *p = strstr(tag_start, search);
    if (!p) return 0;
    /* find end of the opening tag: scan forward for '>' but skip quoted values */
    const char *scan = tag_start + 1;
    const char *tag_end = NULL;
    while (*scan) {
        if (*scan == '"') { scan++; while (*scan && *scan != '"') scan++; if (*scan) scan++; continue; }
        if (*scan == '>') { tag_end = scan; break; }
        scan++;
    }
    /* p must be before the tag end */
    if (tag_end && p > tag_end) return 0;
    p += strlen(search);
    int i = 0;
    while (*p && *p != '"' && i < bufsz - 1)
        buf[i++] = *p++;
    buf[i] = '\0';
    return i > 0;
}

/* Get text content between <tag ...> and </tag> */
static int get_content(const char *xml, const char *tag, char *buf, int bufsz) __attribute__((unused));
static int get_content(const char *xml, const char *tag, char *buf, int bufsz) {
    char open[MAX_NAME], close[MAX_NAME];
    snprintf(open,  sizeof(open),  "<%s", tag);
    snprintf(close, sizeof(close), "</%s>", tag);
    const char *p = strstr(xml, open);
    if (!p) return 0;
    p = strchr(p, '>');
    if (!p) return 0;
    p++;
    const char *e = strstr(p, close);
    if (!e) return 0;
    int len = (int)(e - p);
    if (len >= bufsz) len = bufsz - 1;
    memcpy(buf, p, len);
    buf[len] = '\0';
    return 1;
}

/* Find the end of a block starting at p (finds matching </tag>) */
static const char *find_block_end(const char *start, const char *tag) __attribute__((unused));
static const char *find_block_end(const char *start, const char *tag) {
    char close[MAX_NAME];
    snprintf(close, sizeof(close), "</%s>", tag);
    /* handle self-closing */
    const char *sc = strstr(start, "/>");
    const char *cl = strstr(start, close);
    if (!cl) return sc;
    if (!sc) return cl + strlen(close);
    return (sc < cl) ? sc : cl + strlen(close);
}

static double parse_double(const char *s) {
    if (!s || !*s) return 0.0;
    return strtod(s, NULL);
}

static int parse_vec3(const char *s, vec3_t *v) {
    if (!s || !*s) return 0;
    return sscanf(s, "%lf %lf %lf", &v->x, &v->y, &v->z) == 3;
}

/* ── Link parser ─────────────────────────────────────────────── */

static void parse_inertial(const char *block, inertial_t *in) {
    in->has_inertial = 1;

    /* mass */
    const char *mass_tag = find_tag(block, "mass");
    if (mass_tag) {
        char val[64] = "";
        if (get_attr(mass_tag, "value", val, sizeof(val))) {
            in->mass = parse_double(val);
            in->has_mass = 1;
        }
    }

    /* origin → com */
    const char *orig = find_tag(block, "origin");
    if (orig) {
        char xyz[128] = "";
        get_attr(orig, "xyz", xyz, sizeof(xyz));
        parse_vec3(xyz, &in->com);
    }

    /* inertia tensor */
    const char *itag = find_tag(block, "inertia");
    if (itag) {
        char v[32];
        #define GETD(f, n) (get_attr(itag, n, v, sizeof(v)) ? (in->f = parse_double(v)) : 0.0)
        GETD(ixx, "ixx"); GETD(ixy, "ixy"); GETD(ixz, "ixz");
        GETD(iyy, "iyy"); GETD(iyz, "iyz"); GETD(izz, "izz");
        #undef GETD
        in->has_inertia = 1;
    }
}

static double geometry_size(const char *block) {
    /* returns a rough size estimate from geometry tags */
    const char *box = find_tag(block, "box");
    if (box) {
        char sz[128] = "";
        get_attr(box, "size", sz, sizeof(sz));
        vec3_t v = {0};
        parse_vec3(sz, &v);
        return fmax(fmax(fabs(v.x), fabs(v.y)), fabs(v.z));
    }
    const char *cyl = find_tag(block, "cylinder");
    if (cyl) {
        char r[32] = "", l[32] = "";
        get_attr(cyl, "radius", r, sizeof(r));
        get_attr(cyl, "length", l, sizeof(l));
        return fmax(parse_double(r) * 2.0, parse_double(l));
    }
    const char *sph = find_tag(block, "sphere");
    if (sph) {
        char r[32] = "";
        get_attr(sph, "radius", r, sizeof(r));
        return parse_double(r) * 2.0;
    }
    const char *mesh = find_tag(block, "mesh");
    if (mesh) {
        char sc[128] = "";
        get_attr(mesh, "scale", sc, sizeof(sc));
        if (*sc) {
            vec3_t s = {1,1,1};
            parse_vec3(sc, &s);
            return fmax(fmax(fabs(s.x), fabs(s.y)), fabs(s.z));
        }
        return 1.0; /* default scale */
    }
    return 0.0;
}

static void parse_mesh_path(const char *block, char *out, int outsz) {
    const char *mesh = find_tag(block, "mesh");
    if (!mesh) return;
    get_attr(mesh, "filename", out, outsz);
}

static void parse_link(const char *block, link_t *lk) {
    /* visual */
    const char *vis = find_tag(block, "visual");
    if (vis) {
        lk->has_visual = 1;
        lk->visual_size = geometry_size(vis);
        parse_mesh_path(vis, lk->visual_mesh, MAX_PATH);

        /* material name */
        const char *mat = find_tag(vis, "material");
        if (mat) get_attr(mat, "name", lk->material, MAX_NAME);
    }

    /* collision */
    const char *col = find_tag(block, "collision");
    if (col) {
        lk->has_collision = 1;
        lk->collision_size = geometry_size(col);
        if (!*lk->collision_mesh)
            parse_mesh_path(col, lk->collision_mesh, MAX_PATH);
    }

    /* inertial */
    const char *inr = find_tag(block, "inertial");
    if (inr) {
        parse_inertial(inr, &lk->inertial);
        lk->has_inertial = 1;
    }
}

/* ── Joint parser ────────────────────────────────────────────── */

static joint_type_t parse_joint_type(const char *s) {
    if (!s) return JOINT_UNKNOWN;
    if (!strcmp(s, "revolute"))   return JOINT_REVOLUTE;
    if (!strcmp(s, "continuous")) return JOINT_CONTINUOUS;
    if (!strcmp(s, "prismatic"))  return JOINT_PRISMATIC;
    if (!strcmp(s, "fixed"))      return JOINT_FIXED;
    if (!strcmp(s, "floating"))   return JOINT_FLOATING;
    if (!strcmp(s, "planar"))     return JOINT_PLANAR;
    return JOINT_UNKNOWN;
}

static void parse_joint(const char *tag_start, const char *block, joint_t *jt) {
    char type_str[64] = "";
    get_attr(tag_start, "type", type_str, sizeof(type_str));
    jt->type = parse_joint_type(type_str);

    const char *parent = find_tag(block, "parent");
    if (parent) get_attr(parent, "link", jt->parent, MAX_NAME);

    const char *child = find_tag(block, "child");
    if (child) get_attr(child, "link", jt->child, MAX_NAME);

    /* axis */
    const char *axis = find_tag(block, "axis");
    if (axis) {
        char xyz[128] = "";
        get_attr(axis, "xyz", xyz, sizeof(xyz));
        if (parse_vec3(xyz, &jt->axis)) jt->has_axis = 1;
    }

    /* origin */
    jt->has_origin = (find_tag(block, "origin") != NULL);

    /* limits */
    const char *lim = find_tag(block, "limit");
    if (lim) {
        char v[64];
        jt->has_limits = 1;
        if (get_attr(lim, "lower",    v, sizeof(v))) jt->lower    = parse_double(v);
        if (get_attr(lim, "upper",    v, sizeof(v))) jt->upper    = parse_double(v);
        if (get_attr(lim, "effort",   v, sizeof(v))) jt->effort   = parse_double(v);
        if (get_attr(lim, "velocity", v, sizeof(v))) jt->velocity = parse_double(v);
    }

    /* safety controller */
    const char *safety = find_tag(block, "safety_controller");
    if (safety) {
        char v[64];
        jt->has_safety = 1;
        if (get_attr(safety, "soft_lower_limit", v, sizeof(v))) jt->soft_lower = parse_double(v);
        if (get_attr(safety, "soft_upper_limit", v, sizeof(v))) jt->soft_upper = parse_double(v);
    }
}

/* ── Main XML scan ───────────────────────────────────────────── */

static void scan_xml(const char *xml) {
    const char *p = xml;

    /* robot name */
    const char *robot = find_tag(xml, "robot");
    if (robot) get_attr(robot, "name", g_robot_name, MAX_NAME);

    /* materials at top level */
    const char *mp = xml;
    while ((mp = find_tag(mp, "material")) != NULL) {
        char n[MAX_NAME] = "";
        get_attr(mp, "name", n, MAX_NAME);
        if (*n && g_nmaterials < MAX_MATERIALS) {
            /* check if already seen */
            int dup = 0;
            for (int i = 0; i < g_nmaterials; i++)
                if (!strcmp(g_materials[i].name, n)) { dup = 1; break; }
            if (!dup) {
                memcpy(g_materials[g_nmaterials].name, n, MAX_NAME);
                g_materials[g_nmaterials].name[MAX_NAME-1] = '\0';
                g_materials[g_nmaterials].defined = 1;
                g_nmaterials++;
            }
        }
        mp++;
    }

    /* links */
    p = xml;
    while ((p = find_tag(p, "link")) != NULL) {
        if (g_nlinks >= MAX_LINKS) break;
        link_t *lk = &g_links[g_nlinks];
        memset(lk, 0, sizeof(*lk));

        get_attr(p, "name", lk->name, MAX_NAME);

        /* find block end */
        const char *block_start = strchr(p, '>');
        if (!block_start) { p++; continue; }

        /* self-closing <link name="x"/> — no content */
        if (*(block_start - 1) == '/') { p++; continue; }
        block_start++;

        char block[65536] = "";
        const char *block_end = strstr(block_start, "</link>");
        if (block_end) {
            int len = (int)(block_end - block_start);
            if (len > (int)sizeof(block) - 1) len = sizeof(block) - 1;
            memcpy(block, block_start, len);
        }

        parse_link(block, lk);
        if (*lk->name) g_nlinks++;
        p++;
    }

    /* joints */
    p = xml;
    while ((p = find_tag(p, "joint")) != NULL) {
        if (g_njoints >= MAX_JOINTS) break;
        joint_t *jt = &g_joints[g_njoints];
        memset(jt, 0, sizeof(*jt));

        get_attr(p, "name", jt->name, MAX_NAME);

        const char *block_start = strchr(p, '>');
        if (!block_start) { p++; continue; }
        if (*(block_start - 1) == '/') { p++; continue; }
        block_start++;

        char block[16384] = "";
        const char *block_end = strstr(block_start, "</joint>");
        if (block_end) {
            int len = (int)(block_end - block_start);
            if (len > (int)sizeof(block) - 1) len = sizeof(block) - 1;
            memcpy(block, block_start, len);
        }

        parse_joint(p, block, jt);
        if (*jt->name) g_njoints++;
        p++;
    }
}

/* ── Helpers ─────────────────────────────────────────────────── */

static link_t *find_link(const char *name) {
    for (int i = 0; i < g_nlinks; i++)
        if (!strcmp(g_links[i].name, name)) return &g_links[i];
    return NULL;
}

static int has_illegal_chars(const char *name) {
    for (const char *p = name; *p; p++)
        if (isspace((unsigned char)*p) || *p == '/' || *p == '\\' ||
            *p == '<' || *p == '>' || *p == '&')
            return 1;
    return 0;
}

static double vec3_len(vec3_t v) {
    return sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
}

static int is_positive_definite(inertial_t *in) {
    /* Sylvester's criterion: all leading principal minors > 0 */
    double ixx = in->ixx, ixy = in->ixy, ixz = in->ixz;
    double iyy = in->iyy, iyz = in->iyz, izz = in->izz;
    if (ixx <= 0) return 0;
    if (ixx * iyy - ixy * ixy <= 0) return 0;
    double det = ixx*(iyy*izz - iyz*iyz)
               - ixy*(ixy*izz - iyz*ixz)
               + ixz*(ixy*iyz - iyy*ixz);
    return det > 0;
}

/* ── Check functions ─────────────────────────────────────────── */

static void check_links(void) {
    for (int i = 0; i < g_nlinks; i++) {
        link_t *lk = &g_links[i];
        const char *n = lk->name;

        /* invalid name */
        if (has_illegal_chars(n))
            ERR(n, "link name contains illegal characters (spaces or /\\<>&)");

        /* duplicate names */
        for (int j = i + 1; j < g_nlinks; j++)
            if (!strcmp(n, g_links[j].name))
                ERR(n, "duplicate link name");

        /* visual without collision */
        if (lk->has_visual && !lk->has_collision)
            WARN(n, "has visual geometry but no collision geometry");

        /* collision without visual */
        if (lk->has_collision && !lk->has_visual)
            INFO(n, "has collision geometry but no visual geometry");

        /* missing inertial (skip fixed-only links — checked per joint) */
        if (!lk->has_inertial && lk->has_visual)
            WARN(n, "missing <inertial> block — required for dynamic simulation");

        if (!lk->has_inertial) continue;
        inertial_t *in = &lk->inertial;

        /* missing mass */
        if (!in->has_mass)
            ERR(n, "inertial block present but missing <mass> element");
        else {
            if (in->mass < 0.0)
                ERR(n, "negative mass value (%.6f kg)", in->mass);
            else if (in->mass == 0.0)
                ERR(n, "zero mass — will crash controllers");
            else if (in->mass < 1e-6)
                WARN(n, "extremely small mass (%.2e kg) — may cause instability", in->mass);
            else if (in->mass > 1e4)
                WARN(n, "extremely large mass (%.1f kg) — check units (mm vs m?)", in->mass);
        }

        /* missing inertia tensor */
        if (!in->has_inertia) {
            ERR(n, "inertial block present but missing <inertia> element");
            continue;
        }

        /* zero/negative principal values */
        if (in->ixx <= 0.0)
            ERR(n, "ixx = %.6f — must be positive", in->ixx);
        if (in->iyy <= 0.0)
            ERR(n, "iyy = %.6f — must be positive", in->iyy);
        if (in->izz <= 0.0)
            ERR(n, "izz = %.6f — must be positive", in->izz);

        /* positive definiteness */
        if (in->ixx > 0 && in->iyy > 0 && in->izz > 0 &&
            !is_positive_definite(in))
            ERR(n, "inertia matrix is not positive-definite — unphysical tensor");

        /* triangle inequality check */
        if (in->ixx > 0 && in->iyy > 0 && in->izz > 0) {
            if (in->ixx > in->iyy + in->izz)
                WARN(n, "ixx (%.4f) > iyy+izz (%.4f) — violates triangle inequality", in->ixx, in->iyy+in->izz);
            if (in->iyy > in->ixx + in->izz)
                WARN(n, "iyy (%.4f) > ixx+izz (%.4f) — violates triangle inequality", in->iyy, in->ixx+in->izz);
            if (in->izz > in->ixx + in->iyy)
                WARN(n, "izz (%.4f) > ixx+iyy (%.4f) — violates triangle inequality", in->izz, in->ixx+in->iyy);
        }

        /* cross terms unrealistic relative to diagonal */
        double diag_max = fmax(fmax(in->ixx, in->iyy), in->izz);
        if (diag_max > 0) {
            if (fabs(in->ixy) > diag_max)
                WARN(n, "ixy (%.4f) exceeds max diagonal value — likely wrong", in->ixy);
            if (fabs(in->ixz) > diag_max)
                WARN(n, "ixz (%.4f) exceeds max diagonal value — likely wrong", in->ixz);
            if (fabs(in->iyz) > diag_max)
                WARN(n, "iyz (%.4f) exceeds max diagonal value — likely wrong", in->iyz);
        }

        /* CoM far from origin — flag if > 10m */
        double com_dist = vec3_len(in->com);
        if (com_dist > 10.0)
            WARN(n, "center of mass at (%.2f %.2f %.2f) — %.2fm from link origin, likely wrong",
                 in->com.x, in->com.y, in->com.z, com_dist);

        /* collision vs visual size mismatch */
        if (lk->has_visual && lk->has_collision &&
            lk->visual_size > 0 && lk->collision_size > 0) {
            double ratio = lk->collision_size / lk->visual_size;
            if (ratio > 5.0)
                WARN(n, "collision geometry (%.3f) is %.1fx larger than visual (%.3f)",
                     lk->collision_size, ratio, lk->visual_size);
        }

        /* mesh path checks */
        if (*lk->visual_mesh) {
            if (strstr(lk->visual_mesh, "../../../"))
                WARN(n, "unsafe relative mesh path: %s", lk->visual_mesh);
            if (strncmp(lk->visual_mesh, "file://", 7) == 0)
                WARN(n, "absolute file:// URI breaks portability: %s", lk->visual_mesh);

            /* unit mismatch heuristic: if mesh scale or link size >> 10 */
            if (lk->visual_size > 100.0)
                WARN(n, "visual size %.1f — possible mm/m unit mismatch in mesh", lk->visual_size);

            /* unsupported formats */
            const char *ext = strrchr(lk->visual_mesh, '.');
            if (ext && (strcasecmp(ext, ".obj") == 0 || strcasecmp(ext, ".3ds") == 0 ||
                        strcasecmp(ext, ".fbx") == 0))
                WARN(n, "mesh format '%s' may not be supported by all simulators", ext);
        }

        /* material reference */
        if (*lk->material) {
            int found = 0;
            for (int m = 0; m < g_nmaterials; m++)
                if (!strcmp(g_materials[m].name, lk->material)) { found = 1; break; }
            if (!found)
                WARN(n, "references undefined material '%s'", lk->material);
        }
    }
}

static void check_joints(void) {
    for (int i = 0; i < g_njoints; i++) {
        joint_t *jt = &g_joints[i];
        const char *n = jt->name;

        /* invalid name */
        if (has_illegal_chars(n))
            ERR(n, "joint name contains illegal characters");

        /* duplicate names */
        for (int j = i + 1; j < g_njoints; j++)
            if (!strcmp(n, g_joints[j].name))
                ERR(n, "duplicate joint name");

        /* parent/child link existence */
        if (*jt->parent && !find_link(jt->parent))
            ERR(n, "parent link '%s' not found in URDF", jt->parent);
        if (*jt->child && !find_link(jt->child))
            ERR(n, "child link '%s' not found in URDF", jt->child);

        /* same parent and child */
        if (*jt->parent && *jt->child && !strcmp(jt->parent, jt->child))
            ERR(n, "parent and child link are the same: '%s'", jt->parent);

        /* missing origin */
        if (!jt->has_origin)
            INFO(n, "missing <origin> tag — defaults to identity transform");

        /* axis checks for revolute/prismatic */
        if (jt->type == JOINT_REVOLUTE || jt->type == JOINT_PRISMATIC ||
            jt->type == JOINT_CONTINUOUS) {

            if (!jt->has_axis) {
                WARN(n, "missing <axis> tag — defaults to (1 0 0)");
            } else {
                double len = vec3_len(jt->axis);
                if (len < 1e-9)
                    ERR(n, "joint axis is zero vector (0 0 0) — invalid");
                else if (fabs(len - 1.0) > 0.01)
                    WARN(n, "joint axis not normalized (length=%.4f) — should be unit vector", len);
            }
        }

        /* limits for revolute/prismatic */
        if (jt->type == JOINT_REVOLUTE || jt->type == JOINT_PRISMATIC) {
            if (!jt->has_limits) {
                ERR(n, "revolute/prismatic joint missing <limit> element");
            } else {
                if (jt->lower > jt->upper)
                    ERR(n, "joint limits inverted: lower (%.4f) > upper (%.4f)",
                        jt->lower, jt->upper);

                if (jt->lower == jt->upper)
                    WARN(n, "joint limits are equal (%.4f) — joint cannot move", jt->lower);

                if (jt->effort <= 0.0)
                    WARN(n, "effort limit is %.4f — actuated joint with zero/negative effort", jt->effort);

                if (jt->velocity <= 0.0)
                    WARN(n, "velocity limit is %.4f — actuated joint with zero/negative velocity", jt->velocity);

                /* unrealistic ranges */
                if (jt->type == JOINT_REVOLUTE) {
                    double range = jt->upper - jt->lower;
                    if (range > 2.0 * M_PI * 10)
                        WARN(n, "revolute joint range %.2f rad (%.1f turns) — unrealistically large",
                             range, range / (2.0 * M_PI));
                }

                /* extremely high limits */
                if (jt->effort > 1e6)
                    WARN(n, "effort limit %.1f Nm — extremely high, may cause unstable controllers", jt->effort);
                if (jt->velocity > 1000.0)
                    WARN(n, "velocity limit %.1f rad/s — extremely high", jt->velocity);

                /* safety controller soft limits */
                if (!jt->has_safety)
                    INFO(n, "no <safety_controller> defined — soft limits not set");
                else {
                    if (jt->soft_lower < jt->lower)
                        WARN(n, "soft_lower_limit (%.4f) < hard lower (%.4f)",
                             jt->soft_lower, jt->lower);
                    if (jt->soft_upper > jt->upper)
                        WARN(n, "soft_upper_limit (%.4f) > hard upper (%.4f)",
                             jt->soft_upper, jt->upper);
                }
            }
        }

        /* floating joint warning */
        if (jt->type == JOINT_FLOATING)
            WARN(n, "floating joint — verify this is intentional (6-DOF unconstrained)");
    }
}

static void check_tree(void) {
    /* mark children */
    int child_count[MAX_LINKS] = {0};
    int parent_count[MAX_LINKS] = {0};

    for (int j = 0; j < g_njoints; j++) {
        joint_t *jt = &g_joints[j];
        link_t *child  = find_link(jt->child);
        link_t *parent = find_link(jt->parent);
        if (child)  { child->in_tree = 1;  }
        if (parent) { parent->in_tree = 1; }

        /* count how many times each link appears as child */
        for (int i = 0; i < g_nlinks; i++) {
            if (!strcmp(g_links[i].name, jt->child))  child_count[i]++;
            if (!strcmp(g_links[i].name, jt->parent)) parent_count[i]++;
        }
    }

    /* find roots (links that never appear as child) */
    int n_roots = 0;
    char root_name[MAX_NAME] = "";
    for (int i = 0; i < g_nlinks; i++) {
        if (child_count[i] == 0) {
            n_roots++;
            memcpy(root_name, g_links[i].name, MAX_NAME);
            root_name[MAX_NAME-1] = '\0';
        }
        /* link appears as child more than once = loop */
        if (child_count[i] > 1)
            ERR(g_links[i].name, "appears as child in %d joints — cyclic chain detected",
                child_count[i]);
    }

    if (g_nlinks == 0) {
        ERR("", "no links found in URDF");
        return;
    }

    if (n_roots == 0)
        ERR("", "no root link detected — possible cyclic kinematic chain");
    else if (n_roots > 1)
        ERR("", "%d root links detected — URDF must have exactly one root link", n_roots);

    /* unused links */
    for (int i = 0; i < g_nlinks; i++) {
        if (!g_links[i].in_tree && n_roots == 1 &&
            strcmp(g_links[i].name, root_name) != 0 && g_njoints > 0)
            WARN(g_links[i].name, "link is not connected to any joint — orphan link");
    }

    /* cycle detection via DFS */
    if (n_roots == 1 && g_njoints > 0) {
        /* simple cycle check: if total joints >= total links, there's a cycle */
        if (g_njoints >= g_nlinks)
            WARN("", "joint count (%d) >= link count (%d) — possible kinematic loop",
                 g_njoints, g_nlinks);
    }
}

/* ── Output ──────────────────────────────────────────────────── */

static void print_results(int show_info) {
    int nerr = 0, nwarn = 0, ninfo = 0;
    for (int i = 0; i < g_ndiags; i++) {
        switch (g_diags[i].sev) {
            case SEV_ERROR:   nerr++;  break;
            case SEV_WARNING: nwarn++; break;
            case SEV_INFO:    ninfo++; break;
        }
    }

    for (int i = 0; i < g_ndiags; i++) {
        diag_t *d = &g_diags[i];
        if (d->sev == SEV_INFO && !show_info) continue;

        const char *col = g_no_color ? "" : SEV_COLOR[d->sev];
        const char *rst = g_no_color ? "" : RST;
        const char *dim = g_no_color ? "" : DIM;

        if (*d->context)
            printf("  %s[%s]%s %s<%s>%s %s\n",
                   col, SEV_LABEL[d->sev], rst,
                   dim, d->context, rst,
                   d->msg);
        else
            printf("  %s[%s]%s %s\n",
                   col, SEV_LABEL[d->sev], rst,
                   d->msg);
    }

    /* summary line */
    const char *grn = g_no_color ? "" : GRN;
    const char *red = g_no_color ? "" : "\033[0;31m";
    const char *ylw = g_no_color ? "" : "\033[0;33m";
    const char *rst = g_no_color ? "" : RST;
    const char *dim = g_no_color ? "" : DIM;

    printf("\n");
    printf("  %s links%s     : %d\n", dim, rst, g_nlinks);
    printf("  %s joints%s    : %d\n", dim, rst, g_njoints);
    printf("  %s materials%s : %d\n", dim, rst, g_nmaterials);
    printf("\n");

    if (nerr == 0 && nwarn == 0) {
        printf("  %s✓ No errors or warnings found.%s\n", grn, rst);
    } else {
        if (nerr > 0)  printf("  %s✗ %d error(s)%s\n",   red, nerr,  rst);
        if (nwarn > 0) printf("  %s⚠ %d warning(s)%s\n", ylw, nwarn, rst);
        if (ninfo > 0 && show_info)
            printf("  %sℹ %d info%s\n", dim, ninfo, rst);
    }
}

/* ── CLI ─────────────────────────────────────────────────────── */

static void usage(const char *prog) {
    printf(
        "urdf-doctor v%s — URDF static analysis tool\n\n"
        "Usage: %s [options] <robot.urdf>\n\n"
        "Options:\n"
        "  -a, --all         Show all diagnostics including INFO\n"
        "  -e, --errors      Show only errors (no warnings)\n"
        "  -q, --quiet       Only print summary line\n"
        "  --no-color        Disable colour output\n"
        "  --version         Print version\n"
        "  --help            Show this help\n\n"
        "Exit codes:\n"
        "  0  No errors\n"
        "  1  Warnings found (no errors)\n"
        "  2  Errors found\n"
        "  3  Could not read file\n\n"
        "Examples:\n"
        "  urdf-doctor robot.urdf\n"
        "  urdf-doctor robot.urdf --all\n"
        "  urdf-doctor robot.urdf --errors\n"
        "  urdf-doctor robot.urdf --no-color > report.txt\n",
        VERSION, prog
    );
}

int main(int argc, char *argv[]) {
    const char *filename = NULL;
    int show_all   = 0;
    int errors_only = 0;
    int quiet      = 0;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) {
            usage(argv[0]); return 0;
        } else if (!strcmp(argv[i], "--version")) {
            printf("urdf-doctor v%s\n", VERSION); return 0;
        } else if (!strcmp(argv[i], "--all")      || !strcmp(argv[i], "-a")) { show_all    = 1; }
        else if (!strcmp(argv[i], "--errors")   || !strcmp(argv[i], "-e")) { errors_only = 1; }
        else if (!strcmp(argv[i], "--quiet")    || !strcmp(argv[i], "-q")) { quiet       = 1; }
        else if (!strcmp(argv[i], "--no-color"))                            { g_no_color  = 1; }
        else if (argv[i][0] != '-') filename = argv[i];
        else { fprintf(stderr, "Unknown option: %s\n", argv[i]); return 3; }
    }

    if (!filename) { usage(argv[0]); return 3; }

    /* read file */
    FILE *f = fopen(filename, "rb");
    if (!f) { fprintf(stderr, "Cannot open: %s\n", filename); return 3; }
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    rewind(f);
    char *xml = malloc(sz + 1);
    if (!xml) { fclose(f); fprintf(stderr, "Out of memory\n"); return 3; }
    if (fread(xml, 1, sz, f) != (size_t)sz) { free(xml); fclose(f); fprintf(stderr, "Read error\n"); return 3; }
    xml[sz] = '\0';
    fclose(f);

    /* run */
    const char *dim = g_no_color ? "" : DIM;
    const char *rst = g_no_color ? "" : RST;
    const char *blu = g_no_color ? "" : "\033[0;34m";

    if (!quiet) {
        printf("%surdf-doctor%s v%s\n", blu, rst, VERSION);
        printf("%s────────────────────────────────────%s\n", dim, rst);
        printf("  %s%s%s\n\n", dim, filename, rst);
    }

    scan_xml(xml);
    free(xml);

    check_links();
    check_joints();
    check_tree();

    /* filter for errors_only */
    if (errors_only) {
        for (int i = 0; i < g_ndiags; i++)
            if (g_diags[i].sev != SEV_ERROR) g_diags[i].sev = (severity_t)99;
    }

    if (!quiet) print_results(show_all);
    else {
        int nerr = 0, nwarn = 0;
        for (int i = 0; i < g_ndiags; i++) {
            if (g_diags[i].sev == SEV_ERROR)   nerr++;
            if (g_diags[i].sev == SEV_WARNING)  nwarn++;
        }
        printf("%d errors, %d warnings\n", nerr, nwarn);
    }

    /* exit code */
    int nerr = 0, nwarn = 0;
    for (int i = 0; i < g_ndiags; i++) {
        if (g_diags[i].sev == SEV_ERROR)   nerr++;
        if (g_diags[i].sev == SEV_WARNING)  nwarn++;
    }
    return nerr > 0 ? 2 : (nwarn > 0 ? 1 : 0);
}
