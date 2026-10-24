/*
 * aruco.h - Detecteur ArUco minimal en C pur (sans OpenCV)
 *
 * Dictionnaire : DICT_6X6_250 (le meme que cv::aruco::DICT_6X6_250).
 * Entree       : une image en niveaux de gris, 1 octet par pixel (ex. PIXFORMAT_GRAYSCALE).
 * Sortie       : ID + 4 coins de chaque marqueur, dans le meme ordre qu'OpenCV
 *                (haut-gauche, haut-droit, bas-droit, bas-gauche du marqueur, sens horaire).
 *
 * Etapes : seuillage adaptatif -> composantes connexes sombres -> quadrilatere
 *          -> affinage sous-pixel des bords -> homographie -> lecture 8x8 cases
 *          -> verification de la bordure -> comparaison au dictionnaire (4 rotations).
 */
#ifndef ARUCO_H
#define ARUCO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x, y;
} aruco_point_t;

typedef struct {
    int id;                    /* ID dans DICT_6X6_250 (0..249)                       */
    aruco_point_t corners[4];  /* coins, meme ordre qu'OpenCV                         */
    aruco_point_t center;      /* centre du marqueur                                  */
    int hamming;               /* nb de bits corriges (0 = lecture parfaite)          */
} aruco_marker_t;

typedef struct {
    int block_size;        /* taille des blocs du seuillage adaptatif, en pixels (def. 8)     */
    int min_contrast;      /* ecart min. noir/blanc pour faire confiance au seuil local (20)  */
    int min_side_px;       /* cote minimal d'un marqueur dans l'image, en pixels (def. 16)    */
    int max_hamming;       /* nb max de bits faux corriges (def. 2, max conseille 5)          */
    int max_border_errors; /* nb max de cases de la bordure noire lues blanches (def. 3)      */
    int refine_corners;    /* 1 = affinage sous-pixel des coins (def. 1)                      */
    const int *allowed_ids;/* liste des IDs acceptes, ou NULL pour tous                       */
    int n_allowed_ids;
} aruco_params_t;

/* Statistiques de debug, equivalent des "rejectedCandidates" d'OpenCV. */
typedef struct {
    int n_components;  /* zones sombres trouvees                                 */
    int n_quads;       /* zones dont la forme est un quadrilatere plausible      */
    int n_rejected;    /* quadrilateres dont le contenu n'est pas un marqueur    */
    int n_detected;    /* marqueurs valides                                      */
} aruco_stats_t;

/* Remplit les parametres par defaut. */
void aruco_default_params(aruco_params_t *p);

/*
 * Detecte les marqueurs dans l'image.
 *   gray    : image w x h, 1 octet/pixel, stride = w
 *   params  : NULL = parametres par defaut
 *   out     : tableau de sortie, max_out cases
 *   stats   : NULL si inutile
 * Retourne le nombre de marqueurs ecrits dans out, ou -1 en cas d'echec d'allocation.
 *
 * Memoire temporaire allouee : environ w*h octets (77 Ko en QVGA) + quelques Ko.
 * Sur ESP32, elle est prise de preference en PSRAM.
 */
int aruco_detect(const uint8_t *gray, int w, int h, const aruco_params_t *params,
                 aruco_marker_t *out, int max_out, aruco_stats_t *stats);

/* ------------------------------------------------------------------------- */
/* Dessin (pour le debug)                                                    */
/* ------------------------------------------------------------------------- */
/*
 * L'image de la camera est en niveaux de gris : pour dessiner en couleur, on la
 * convertit d'abord en RGB (3 octets/pixel, ordre R, G, B, soit w*h*3 octets :
 * 230 Ko en QVGA, a allouer en PSRAM sur ESP32).
 */
void aruco_gray_to_rgb(const uint8_t *gray, int w, int h, uint8_t *rgb);

/*
 * Encadre chaque marqueur avec la couleur (r, g, b) et une epaisseur en pixels.
 * Un petit carre rouge marque le coin 0 (haut-gauche du marqueur), comme
 * cv::aruco::drawDetectedMarkers, pour voir l'orientation.
 */
void aruco_draw_markers(uint8_t *rgb, int w, int h, const aruco_marker_t *markers, int n,
                        uint8_t r, uint8_t g, uint8_t b, int thickness);

#ifdef __cplusplus
}
#endif

#endif /* ARUCO_H */
