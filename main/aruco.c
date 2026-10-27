/*
 * aruco.c - Detecteur ArUco minimal en C pur (sans OpenCV). Voir aruco.h.
 */
#include "aruco.h"
#include "aruco_dict_4x4_250.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ------------------------------------------------------------------------- */
/* Allocation : PSRAM de preference sur ESP32, malloc ailleurs               */
/* ------------------------------------------------------------------------- */
#ifdef ESP_PLATFORM
#include "esp_heap_caps.h"
#define ARUCO_MALLOC(sz)       heap_caps_malloc_prefer((sz), 2, MALLOC_CAP_SPIRAM, MALLOC_CAP_DEFAULT)
#define ARUCO_REALLOC(p, sz)   heap_caps_realloc_prefer((p), (sz), 2, MALLOC_CAP_SPIRAM, MALLOC_CAP_DEFAULT)
#define ARUCO_FREE(p)          heap_caps_free(p)
#else
#define ARUCO_MALLOC(sz)       malloc(sz)
#define ARUCO_REALLOC(p, sz)   realloc((p), (sz))
#define ARUCO_FREE(p)          free(p)
#endif

#define MARKER_BITS   4                 /* 4x4 cases de code          */
#define GRID          (MARKER_BITS + 2) /* + 1 case de bordure noire  */
#define N_DIRS        16                /* directions pour les coins  */

/* Valeurs du buffer binaire */
#define PX_LIGHT   0
#define PX_DARK    1
#define PX_VISITED 2

void aruco_default_params(aruco_params_t *p)
{
    p->block_size        = 8;
    p->min_contrast      = 20;
    p->min_side_px       = 16;
    p->max_hamming       = 2;
    p->max_border_errors = 3;
    p->refine_corners    = 1;
    p->allowed_ids       = NULL;
    p->n_allowed_ids     = 0;
}

/* ========================================================================= */
/* 1. Seuillage adaptatif                                                    */
/* ========================================================================= */
/*
 * L'image est decoupee en blocs de block_size x block_size. Pour chaque bloc on
 * prend le min et le max sur ses 3x3 blocs voisins ; le seuil local est leur
 * milieu. Si le contraste local est trop faible (zone uniforme), on utilise un
 * seuil global (Otsu) a la place.
 */
static int otsu_threshold(const uint8_t *img, int n)
{
    uint32_t hist[256] = {0};
    for (int i = 0; i < n; i++) hist[img[i]]++;

    double sum = 0;
    for (int t = 0; t < 256; t++) sum += (double)t * hist[t];

    double sum_b = 0, w_b = 0, best = -1;
    int thr = 128;
    for (int t = 0; t < 256; t++) {
        w_b += hist[t];
        if (w_b == 0) continue;
        double w_f = n - w_b;
        if (w_f == 0) break;
        sum_b += (double)t * hist[t];
        double m_b = sum_b / w_b, m_f = (sum - sum_b) / w_f;
        double between = w_b * w_f * (m_b - m_f) * (m_b - m_f);
        if (between > best) { best = between; thr = t; }
    }
    return thr;
}

static int threshold_image(const uint8_t *gray, int w, int h, const aruco_params_t *p,
                           uint8_t *bin)
{
    const int bs = p->block_size;
    const int bw = (w + bs - 1) / bs, bh = (h + bs - 1) / bs;
    const int nb = bw * bh;

    uint8_t *bmin = ARUCO_MALLOC(4 * (size_t)nb);
    if (!bmin) return -1;
    uint8_t *bmax = bmin + nb, *nmin = bmin + 2 * nb, *nmax = bmin + 3 * nb;

    /* min / max de chaque bloc */
    for (int by = 0; by < bh; by++) {
        for (int bx = 0; bx < bw; bx++) {
            uint8_t lo = 255, hi = 0;
            int y1 = (by + 1) * bs < h ? (by + 1) * bs : h;
            int x1 = (bx + 1) * bs < w ? (bx + 1) * bs : w;
            for (int y = by * bs; y < y1; y++) {
                const uint8_t *row = gray + (size_t)y * w;
                for (int x = bx * bs; x < x1; x++) {
                    if (row[x] < lo) lo = row[x];
                    if (row[x] > hi) hi = row[x];
                }
            }
            bmin[by * bw + bx] = lo;
            bmax[by * bw + bx] = hi;
        }
    }

    /* min / max sur le voisinage 3x3 de blocs */
    for (int by = 0; by < bh; by++) {
        for (int bx = 0; bx < bw; bx++) {
            uint8_t lo = 255, hi = 0;
            for (int dy = -1; dy <= 1; dy++) {
                int yy = by + dy;
                if (yy < 0 || yy >= bh) continue;
                for (int dx = -1; dx <= 1; dx++) {
                    int xx = bx + dx;
                    if (xx < 0 || xx >= bw) continue;
                    if (bmin[yy * bw + xx] < lo) lo = bmin[yy * bw + xx];
                    if (bmax[yy * bw + xx] > hi) hi = bmax[yy * bw + xx];
                }
            }
            nmin[by * bw + bx] = lo;
            nmax[by * bw + bx] = hi;
        }
    }

    const int global_thr = otsu_threshold(gray, w * h);

    for (int y = 0; y < h; y++) {
        const uint8_t *row = gray + (size_t)y * w;
        uint8_t *brow = bin + (size_t)y * w;
        const int by = y / bs;
        for (int x = 0; x < w; x++) {
            const int b = by * bw + x / bs;
            int thr = (nmax[b] - nmin[b] < p->min_contrast)
                          ? global_thr
                          : (nmin[b] + nmax[b]) / 2;
            brow[x] = row[x] < thr ? PX_DARK : PX_LIGHT;
        }
    }

    ARUCO_FREE(bmin);
    return 0;
}

/* ========================================================================= */
/* 2. Composantes connexes sombres et recherche du quadrilatere              */
/* ========================================================================= */
/*
 * Chaque zone sombre (8-connexite) est parcourue une seule fois. Pendant le
 * parcours, on garde le pixel le plus extreme dans 16 directions. Les coins
 * d'un quadrilatere convexe font forcement partie de ces points extremes : on
 * choisit ensuite les 4 qui donnent la plus grande aire.
 */
typedef struct {
    int count;
    int minx, miny, maxx, maxy;
    int ext_val[N_DIRS];
    int ext_x[N_DIRS], ext_y[N_DIRS];
} component_t;

static int dir_x[N_DIRS], dir_y[N_DIRS];
static int dirs_ready = 0;

static void init_dirs(void)
{
    if (dirs_ready) return;
    for (int k = 0; k < N_DIRS; k++) {
        float a = (float)k * 2.0f * (float)M_PI / N_DIRS;
        dir_x[k] = (int)lroundf(1024.0f * cosf(a));
        dir_y[k] = (int)lroundf(1024.0f * sinf(a));
    }
    dirs_ready = 1;
}

typedef struct {
    int32_t *data;
    int size, cap;
} stack_t_;

static int stack_push(stack_t_ *s, int32_t v)
{
    if (s->size == s->cap) {
        int ncap = s->cap * 2;
        int32_t *nd = ARUCO_REALLOC(s->data, (size_t)ncap * sizeof(int32_t));
        if (!nd) return -1;
        s->data = nd;
        s->cap = ncap;
    }
    s->data[s->size++] = v;
    return 0;
}

/* Parcourt la zone sombre contenant le pixel 'seed'. */
static int flood_component(uint8_t *bin, int w, int h, int seed, stack_t_ *st,
                           component_t *c)
{
    c->count = 0;
    c->minx = c->miny = 1 << 30;
    c->maxx = c->maxy = -1;
    for (int k = 0; k < N_DIRS; k++) c->ext_val[k] = -(1 << 30);

    st->size = 0;
    bin[seed] = PX_VISITED;
    if (stack_push(st, seed)) return -1;

    while (st->size > 0) {
        int idx = st->data[--st->size];
        int x = idx % w, y = idx / w;

        c->count++;
        if (x < c->minx) c->minx = x;
        if (x > c->maxx) c->maxx = x;
        if (y < c->miny) c->miny = y;
        if (y > c->maxy) c->maxy = y;
        for (int k = 0; k < N_DIRS; k++) {
            int v = x * dir_x[k] + y * dir_y[k];
            if (v > c->ext_val[k]) {
                c->ext_val[k] = v;
                c->ext_x[k] = x;
                c->ext_y[k] = y;
            }
        }

        for (int dy = -1; dy <= 1; dy++) {
            int yy = y + dy;
            if (yy < 0 || yy >= h) continue;
            for (int dx = -1; dx <= 1; dx++) {
                int xx = x + dx;
                if (xx < 0 || xx >= w) continue;
                int n = yy * w + xx;
                if (bin[n] == PX_DARK) {
                    bin[n] = PX_VISITED;
                    if (stack_push(st, n)) return -1;
                }
            }
        }
    }
    return 0;
}

static float quad_area(const aruco_point_t q[4])
{
    float a = 0;
    for (int i = 0; i < 4; i++) {
        const aruco_point_t *p0 = &q[i], *p1 = &q[(i + 1) & 3];
        a += p0->x * p1->y - p1->x * p0->y;
    }
    return 0.5f * a; /* > 0 : sens horaire a l'ecran (y vers le bas) */
}

static float dist2(aruco_point_t a, aruco_point_t b)
{
    float dx = a.x - b.x, dy = a.y - b.y;
    return dx * dx + dy * dy;
}

/* Choisit 4 coins parmi les points extremes. Retourne 1 si un quad plausible est trouve. */
static int component_to_quad(const component_t *c, const aruco_params_t *p, int w, int h,
                             aruco_point_t quad[4])
{
    /* Filtres rapides : taille, contact avec le bord de l'image */
    int bw = c->maxx - c->minx + 1, bh = c->maxy - c->miny + 1;
    if (bw < p->min_side_px || bh < p->min_side_px) return 0;
    if (c->minx <= 0 || c->miny <= 0 || c->maxx >= w - 1 || c->maxy >= h - 1) return 0;

    /* Points extremes uniques, tries par angle autour du centre de la boite */
    aruco_point_t pts[N_DIRS];
    float ang[N_DIRS];
    int n = 0;
    float cx = 0.5f * (c->minx + c->maxx), cy = 0.5f * (c->miny + c->maxy);
    for (int k = 0; k < N_DIRS; k++) {
        int dup = 0;
        for (int j = 0; j < n; j++)
            if ((int)pts[j].x == c->ext_x[k] && (int)pts[j].y == c->ext_y[k]) { dup = 1; break; }
        if (dup) continue;
        pts[n].x = (float)c->ext_x[k];
        pts[n].y = (float)c->ext_y[k];
        ang[n] = atan2f(pts[n].y - cy, pts[n].x - cx);
        n++;
    }
    if (n < 4) return 0;
    for (int i = 1; i < n; i++) { /* tri par insertion */
        aruco_point_t tp = pts[i];
        float ta = ang[i];
        int j = i - 1;
        while (j >= 0 && ang[j] > ta) { pts[j + 1] = pts[j]; ang[j + 1] = ang[j]; j--; }
        pts[j + 1] = tp;
        ang[j + 1] = ta;
    }

    /* Quadrilatere d'aire maximale (les points tries par angle donnent un quad convexe) */
    float best = 0;
    for (int a = 0; a < n; a++)
        for (int b = a + 1; b < n; b++)
            for (int d = b + 1; d < n; d++)
                for (int e = d + 1; e < n; e++) {
                    aruco_point_t q[4] = {pts[a], pts[b], pts[d], pts[e]};
                    float ar = quad_area(q);
                    if (ar > best) { best = ar; memcpy(quad, q, sizeof(q)); }
                }
    if (best <= 0) return 0;

    /* Forme plausible : cotes assez longs et pas trop desequilibres */
    float min_side2 = 1e30f, max_side2 = 0;
    for (int i = 0; i < 4; i++) {
        float s = dist2(quad[i], quad[(i + 1) & 3]);
        if (s < min_side2) min_side2 = s;
        if (s > max_side2) max_side2 = s;
    }
    float min_side = 0.75f * (float)p->min_side_px;
    if (min_side2 < min_side * min_side) return 0;
    if (max_side2 > 16.0f * min_side2) return 0; /* rapport des cotes <= 4 */

    /* La zone sombre doit "remplir" correctement le quad.
     * Bordure seule d'un marqueur = 44 % de sa surface ; on reste tolerant. */
    float fill = (float)c->count / best;
    if (fill < 0.2f || fill > 1.3f) return 0;

    /* Chaque point extreme doit etre dans le quad ou tres pres de ses bords :
     * sinon la zone n'est pas un quadrilatere (forme en L, cercle...). */
    for (int i = 0; i < n; i++) {
        for (int e = 0; e < 4; e++) {
            aruco_point_t a = quad[e], b = quad[(e + 1) & 3];
            float ex = b.x - a.x, ey = b.y - a.y;
            float len = sqrtf(ex * ex + ey * ey);
            /* distance signee vers l'exterieur */
            float d = (ex * (pts[i].y - a.y) - ey * (pts[i].x - a.x)) / len;
            if (-d > 1.5f + 0.04f * len) return 0;
        }
    }
    return 1;
}

/* ========================================================================= */
/* 3. Affinage sous-pixel des coins                                          */
/* ========================================================================= */
static float sample_bilinear(const uint8_t *g, int w, int h, float x, float y)
{
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x > w - 1.001f) x = w - 1.001f;
    if (y > h - 1.001f) y = h - 1.001f;
    int x0 = (int)x, y0 = (int)y;
    float fx = x - x0, fy = y - y0;
    const uint8_t *r0 = g + (size_t)y0 * w + x0, *r1 = r0 + w;
    float top = r0[0] + fx * (r0[1] - r0[0]);
    float bot = r1[0] + fx * (r1[1] - r1[0]);
    return top + fy * (bot - top);
}

/*
 * Pour chaque cote, on cherche la position exacte du bord (gradient max,
 * interpolation parabolique) en plusieurs points le long d'une normale, puis
 * on ajuste une droite. Les coins sont les intersections des 4 droites.
 */
static void refine_quad(const uint8_t *gray, int w, int h, aruco_point_t q[4])
{
    float lp_x[4], lp_y[4], ld_x[4], ld_y[4]; /* droite : point + direction */

    for (int e = 0; e < 4; e++) {
        aruco_point_t a = q[e], b = q[(e + 1) & 3];
        float tx = b.x - a.x, ty = b.y - a.y;
        float len = sqrtf(tx * tx + ty * ty);
        tx /= len;
        ty /= len;
        float nx = ty, ny = -tx; /* normale vers l'exterieur (quad en sens horaire) */

        float range = len / 16.0f + 1.0f; /* < 1/2 case : on ne touche pas l'interieur */
        if (range > 3.0f) range = 3.0f;

        enum { NS = 12, NP = 15 };
        float ex[NS], ey[NS];
        int ne = 0;
        for (int s = 0; s < NS; s++) {
            float t = 0.15f + 0.7f * (float)s / (NS - 1);
            float px = a.x + (b.x - a.x) * t, py = a.y + (b.y - a.y) * t;

            float prof[NP];
            for (int k = 0; k < NP; k++) {
                float o = -range + 2.0f * range * k / (NP - 1);
                prof[k] = sample_bilinear(gray, w, h, px + nx * o, py + ny * o);
            }
            /* sombre dedans, clair dehors -> derivee positive vers l'exterieur */
            int kb = -1;
            float gb = 8.0f;
            for (int k = 1; k < NP - 1; k++) {
                float gk = prof[k + 1] - prof[k - 1];
                if (gk > gb) { gb = gk; kb = k; }
            }
            if (kb < 2 || kb > NP - 3) continue;
            float gm = prof[kb] - prof[kb - 2], gp = prof[kb + 2] - prof[kb];
            float den = gm - 2.0f * gb + gp;
            float off = (fabsf(den) > 1e-6f) ? 0.5f * (gm - gp) / den : 0.0f;
            if (off < -1 || off > 1) off = 0;
            float o = -range + 2.0f * range * (kb + off) / (NP - 1);
            ex[ne] = px + nx * o;
            ey[ne] = py + ny * o;
            ne++;
        }
        if (ne < 4) return; /* pas assez de points : on garde les coins bruts */

        /* droite des moindres carres orthogonaux (ACP) */
        float mx = 0, my = 0;
        for (int i = 0; i < ne; i++) { mx += ex[i]; my += ey[i]; }
        mx /= ne;
        my /= ne;
        float sxx = 0, sxy = 0, syy = 0;
        for (int i = 0; i < ne; i++) {
            float dx = ex[i] - mx, dy = ey[i] - my;
            sxx += dx * dx; sxy += dx * dy; syy += dy * dy;
        }
        float theta = 0.5f * atan2f(2.0f * sxy, sxx - syy);
        lp_x[e] = mx;
        lp_y[e] = my;
        ld_x[e] = cosf(theta);
        ld_y[e] = sinf(theta);
    }

    aruco_point_t r[4];
    for (int i = 0; i < 4; i++) {
        int e0 = (i + 3) & 3, e1 = i; /* coin i = cote precedent x cote suivant */
        float det = ld_x[e0] * ld_y[e1] - ld_y[e0] * ld_x[e1];
        if (fabsf(det) < 1e-3f) return;
        float dx = lp_x[e1] - lp_x[e0], dy = lp_y[e1] - lp_y[e0];
        float t = (dx * ld_y[e1] - dy * ld_x[e1]) / det;
        r[i].x = lp_x[e0] + t * ld_x[e0];
        r[i].y = lp_y[e0] + t * ld_y[e0];
        if (dist2(r[i], q[i]) > 9.0f) return; /* deplacement > 3 px : suspect */
    }
    memcpy(q, r, sizeof(r));
}

/* ========================================================================= */
/* 4. Homographie carre unite -> quad, lecture des cases                     */
/* ========================================================================= */
typedef struct { float a, b, c, d, e, f, g, h; } homog_t;

/* (0,0)->q0, (1,0)->q1, (1,1)->q2, (0,1)->q3  (Heckbert) */
static int square_to_quad(const aruco_point_t q[4], homog_t *H)
{
    float x0 = q[0].x, y0 = q[0].y, x1 = q[1].x, y1 = q[1].y;
    float x2 = q[2].x, y2 = q[2].y, x3 = q[3].x, y3 = q[3].y;
    float sx = x0 - x1 + x2 - x3, sy = y0 - y1 + y2 - y3;
    if (fabsf(sx) < 1e-6f && fabsf(sy) < 1e-6f) {
        H->a = x1 - x0; H->b = x2 - x1; H->c = x0;
        H->d = y1 - y0; H->e = y2 - y1; H->f = y0;
        H->g = 0; H->h = 0;
        return 0;
    }
    float dx1 = x1 - x2, dx2 = x3 - x2, dy1 = y1 - y2, dy2 = y3 - y2;
    float den = dx1 * dy2 - dx2 * dy1;
    if (fabsf(den) < 1e-9f) return -1;
    H->g = (sx * dy2 - dx2 * sy) / den;
    H->h = (dx1 * sy - sx * dy1) / den;
    H->a = x1 - x0 + H->g * x1;
    H->b = x3 - x0 + H->h * x3;
    H->c = x0;
    H->d = y1 - y0 + H->g * y1;
    H->e = y3 - y0 + H->h * y3;
    H->f = y0;
    return 0;
}

static void homog_apply(const homog_t *H, float u, float v, float *x, float *y)
{
    float z = H->g * u + H->h * v + 1.0f;
    *x = (H->a * u + H->b * v + H->c) / z;
    *y = (H->d * u + H->e * v + H->f) / z;
}

/* Lit la luminosite moyenne de chaque case de la grille 8x8. */
static void read_cells(const uint8_t *gray, int w, int h, const homog_t *H, float cells[GRID][GRID])
{
    static const float sub[3] = {0.3f, 0.5f, 0.7f}; /* echantillons au centre de la case */
    for (int r = 0; r < GRID; r++) {
        for (int c = 0; c < GRID; c++) {
            float acc = 0;
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++) {
                    float x, y;
                    homog_apply(H, (c + sub[j]) / GRID, (r + sub[i]) / GRID, &x, &y);
                    acc += sample_bilinear(gray, w, h, x, y);
                }
            cells[r][c] = acc / 9.0f;
        }
    }
}

/* Seuil d'Otsu sur les 64 valeurs de cases */
static float cells_threshold(float cells[GRID][GRID], float *contrast)
{
    float v[GRID * GRID];
    int n = 0;
    for (int r = 0; r < GRID; r++)
        for (int c = 0; c < GRID; c++) v[n++] = cells[r][c];
    for (int i = 1; i < n; i++) { /* tri */
        float t = v[i];
        int j = i - 1;
        while (j >= 0 && v[j] > t) { v[j + 1] = v[j]; j--; }
        v[j + 1] = t;
    }
    float total = 0;
    for (int i = 0; i < n; i++) total += v[i];
    float best = -1, thr = 0.5f * (v[0] + v[n - 1]), sum_lo = 0;
    *contrast = 0;
    for (int k = 1; k < n; k++) { /* k valeurs sombres, n-k claires */
        sum_lo += v[k - 1];
        float m_lo = sum_lo / k, m_hi = (total - sum_lo) / (n - k);
        float between = (float)k * (n - k) * (m_hi - m_lo) * (m_hi - m_lo);
        if (between > best) {
            best = between;
            thr = 0.5f * (v[k - 1] + v[k]);
            *contrast = m_hi - m_lo;
        }
    }
    return thr;
}

static int popcount64(uint64_t v)
{
    int n = 0;
    while (v) { v &= v - 1; n++; }
    return n;
}

static int id_allowed(const aruco_params_t *p, int id)
{
    if (!p->allowed_ids || p->n_allowed_ids <= 0) return 1;
    for (int i = 0; i < p->n_allowed_ids; i++)
        if (p->allowed_ids[i] == id) return 1;
    return 0;
}

/*
 * Decode le quad. Retourne 1 si c'est un marqueur valide ; remplit id, hamming
 * et le nombre de rotations (sens horaire) a appliquer a la grille lue.
 */
static int decode_quad(const uint8_t *gray, int w, int h, const aruco_point_t q[4],
                       const aruco_params_t *p, int *out_id, int *out_ham, int *out_rot)
{
    homog_t H;
    if (square_to_quad(q, &H)) return 0;

    float cells[GRID][GRID];
    read_cells(gray, w, h, &H, cells);

    float contrast;
    float thr = cells_threshold(cells, &contrast);
    if (contrast < (float)p->min_contrast) return 0;

    /* La bordure doit etre noire */
    int border_err = 0;
    for (int i = 0; i < GRID; i++) {
        border_err += cells[0][i] > thr;
        border_err += cells[GRID - 1][i] > thr;
        if (i > 0 && i < GRID - 1) {
            border_err += cells[i][0] > thr;
            border_err += cells[i][GRID - 1] > thr;
        }
    }
    if (border_err > p->max_border_errors) return 0;

    /* Bits interieurs (1 = blanc) */
    uint8_t bits[MARKER_BITS][MARKER_BITS];
    for (int r = 0; r < MARKER_BITS; r++)
        for (int c = 0; c < MARKER_BITS; c++)
            bits[r][c] = cells[r + 1][c + 1] > thr;

    int best_ham = 99, best_id = -1, best_rot = 0;
    for (int rot = 0; rot < 4; rot++) {
        uint64_t code = 0;
        for (int r = 0; r < MARKER_BITS; r++)
            for (int c = 0; c < MARKER_BITS; c++) {
                /* grille lue tournee de 'rot' quarts de tour (sens horaire) */
                int sr = r, sc = c;
                for (int k = 0; k < rot; k++) {
                    int t = sr;
                    sr = MARKER_BITS - 1 - sc;
                    sc = t;
                }
                code = (code << 1) | bits[sr][sc];
            }
        for (int id = 0; id < ARUCO_DICT_SIZE; id++) {
            if (!id_allowed(p, id)) continue;
            int ham = popcount64(code ^ ARUCO_DICT_4X4_250[id]);
            if (ham < best_ham) { best_ham = ham; best_id = id; best_rot = rot; }
        }
    }
    if (best_id < 0 || best_ham > p->max_hamming) return 0;

    *out_id = best_id;
    *out_ham = best_ham;
    *out_rot = best_rot;
    return 1;
}

/* ========================================================================= */
/* 5. Fonction principale                                                    */
/* ========================================================================= */
int aruco_detect(const uint8_t *gray, int w, int h, const aruco_params_t *params,
                 aruco_marker_t *out, int max_out, aruco_stats_t *stats)
{
    aruco_params_t def;
    if (!params) {
        aruco_default_params(&def);
        params = &def;
    }
    aruco_stats_t st_local;
    aruco_stats_t *st = stats ? stats : &st_local;
    memset(st, 0, sizeof(*st));

    init_dirs();

    uint8_t *bin = ARUCO_MALLOC((size_t)w * h);
    if (!bin) return -1;
    if (threshold_image(gray, w, h, params, bin)) { ARUCO_FREE(bin); return -1; }

    stack_t_ stack = {ARUCO_MALLOC(4096 * sizeof(int32_t)), 0, 4096};
    if (!stack.data) { ARUCO_FREE(bin); return -1; }

    int n_out = 0;
    const int npix = w * h;
    for (int i = 0; i < npix; i++) {
        if (bin[i] != PX_DARK) continue;

        component_t comp;
        if (flood_component(bin, w, h, i, &stack, &comp)) {
            n_out = -1;
            break;
        }
        st->n_components++;

        aruco_point_t q[4];
        if (!component_to_quad(&comp, params, w, h, q)) continue;
        st->n_quads++;

        if (params->refine_corners) refine_quad(gray, w, h, q);

        int id, ham, rot;
        if (!decode_quad(gray, w, h, q, params, &id, &ham, &rot)) {
            st->n_rejected++;
            continue;
        }
        st->n_detected++;
        if (n_out >= max_out) continue;

        /* Coins dans l'ordre d'OpenCV : le coin "haut-gauche" du marqueur
         * depend de la rotation trouvee. */
        aruco_marker_t *m = &out[n_out++];
        m->id = id;
        m->hamming = ham;
        m->center.x = m->center.y = 0;
        for (int k = 0; k < 4; k++) {
            m->corners[k] = q[(k + 4 - rot) & 3];
            m->center.x += 0.25f * q[k].x;
            m->center.y += 0.25f * q[k].y;
        }
    }

    ARUCO_FREE(stack.data);
    ARUCO_FREE(bin);
    return n_out;
}

/* ========================================================================= */
/* 6. Dessin                                                                 */
/* ========================================================================= */
void aruco_gray_to_rgb(const uint8_t *gray, int w, int h, uint8_t *rgb)
{
    const int n = w * h;
    for (int i = 0; i < n; i++) {
        rgb[3 * i] = rgb[3 * i + 1] = rgb[3 * i + 2] = gray[i];
    }
}

/* Remplit un carre de cote 'size' centre sur (cx, cy), en restant dans l'image. */
static void put_square(uint8_t *rgb, int w, int h, int cx, int cy, int size,
                       uint8_t r, uint8_t g, uint8_t b)
{
    int x0 = cx - size / 2, y0 = cy - size / 2;
    for (int y = y0; y < y0 + size; y++) {
        if (y < 0 || y >= h) continue;
        for (int x = x0; x < x0 + size; x++) {
            if (x < 0 || x >= w) continue;
            uint8_t *px = rgb + 3 * ((size_t)y * w + x);
            px[0] = r; px[1] = g; px[2] = b;
        }
    }
}

/* Trace un segment epais (Bresenham + un carre de 'thickness' px a chaque pas). */
static void draw_line(uint8_t *rgb, int w, int h, int x0, int y0, int x1, int y1,
                      uint8_t r, uint8_t g, uint8_t b, int thickness)
{
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    for (;;) {
        put_square(rgb, w, h, x0, y0, thickness, r, g, b);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void aruco_draw_markers(uint8_t *rgb, int w, int h, const aruco_marker_t *markers, int n,
                        uint8_t r, uint8_t g, uint8_t b, int thickness)
{
    if (thickness < 1) thickness = 1;
    for (int i = 0; i < n; i++) {
        const aruco_point_t *c = markers[i].corners;
        for (int k = 0; k < 4; k++) {
            const aruco_point_t *p0 = &c[k], *p1 = &c[(k + 1) & 3];
            draw_line(rgb, w, h, (int)lroundf(p0->x), (int)lroundf(p0->y),
                      (int)lroundf(p1->x), (int)lroundf(p1->y), r, g, b, thickness);
        }
        /* coin 0 en rouge, pour l'orientation */
        put_square(rgb, w, h, (int)lroundf(c[0].x), (int)lroundf(c[0].y),
                   thickness + 4, 255, 0, 0);
    }
}
