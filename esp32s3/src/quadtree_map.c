/**
 * quadtree_map.c
 * Module: Quadtree occupancy map.
 * Board: ESP32-S3
 */

#include "quadtree_map.h"

#include <stdlib.h>
#include <string.h>

#if defined(ESP_PLATFORM)
#include "esp_log.h"
#include "esp_attr.h"   /* IRAM_ATTR */
static const char *TAG_QT = "quadtree_map";
#else
#define IRAM_ATTR       /* host builds: no-op */
#endif


// clamp an int to the log-odds range

static inline int8_t _clamp(int v)
{
    if (v > QT_VALUE_MAX) return (int8_t)QT_VALUE_MAX;
    if (v < QT_VALUE_MIN) return (int8_t)QT_VALUE_MIN;
    return (int8_t)v;
}

// return which quadrant of [xmn,xmx]×[ymn,ymx] contains (x,y)
//   0 = NW  (x < cx, y >= cy)
//   1 = NE  (x >= cx, y >= cy)
//   2 = SW  (x < cx, y < cy)
//   3 = SE  (x >= cx, y < cy)

static inline int _quadrant(float xmn, float xmx,
                             float ymn, float ymx,
                             float x,   float y)
{
    float cx = 0.5f * (xmn + xmx);
    float cy = 0.5f * (ymn + ymx);
    int east  = (x >= cx) ? 1 : 0;
    int north = (y >= cy) ? 1 : 0;
    return north ? east : (2 + east);
}

// fill the bounds of child quadrant q inside [xmn,xmx]×[ymn,ymx]
static inline void _child_bounds(float xmn, float xmx,
                                  float ymn, float ymx, int q,
                                  float *cxmn, float *cxmx,
                                  float *cymn, float *cymx)
{
    float cx = 0.5f * (xmn + xmx);
    float cy = 0.5f * (ymn + ymx);
    switch (q) {
        case 0: *cxmn=xmn; *cxmx=cx;  *cymn=cy;  *cymx=ymx; break; // NW
        case 1: *cxmn=cx;  *cxmx=xmx; *cymn=cy;  *cymx=ymx; break; // NE
        case 2: *cxmn=xmn; *cxmx=cx;  *cymn=ymn; *cymx=cy;  break; // SW
        case 3: *cxmn=cx;  *cxmx=xmx; *cymn=ymn; *cymx=cy;  break; // SE
    }
}

// Allocate one node from the pool/ returns QT_NULL if pool=full
static uint16_t _alloc(QuadTreeMap *map, uint8_t depth)
{
    if (map->count >= QT_POOL_SIZE) return QT_NULL;
    uint16_t idx = map->count++;
    QTNode *n = &map->pool[idx];
    n->children[0] = n->children[1] =
    n->children[2] = n->children[3] = QT_NULL;
    n->value = 0;
    n->depth = depth;
    return idx;
}


/* Static BSS pool — moves 48 KB off the heap so the quadtree is never a
 * source of heap fragmentation.  One instance only; the device always has
 * exactly one map (s_map in main.c).  Host test builds keep the heap path
 * so tests that instantiate multiple maps (truth + slam) still work. */
#if defined(ESP_PLATFORM)
static QTNode s_qt_pool[QT_POOL_SIZE];
#endif

void qt_init(QuadTreeMap *map,
             float x_min, float x_max,
             float y_min, float y_max)
{
#if defined(ESP_PLATFORM)
    memset(s_qt_pool, 0, sizeof(s_qt_pool));
    map->pool = s_qt_pool;
#else
    map->pool = (QTNode *)calloc(QT_POOL_SIZE, sizeof(QTNode));
    if (!map->pool) return;
#endif

    map->count = 1; /* slot 0 reserved as QT_NULL */
    map->x_min = x_min;
    map->x_max = x_max;
    map->y_min = y_min;
    map->y_max = y_max;
    _alloc(map, 1); /* root at index 1 */
}

void qt_free(QuadTreeMap *map)
{
    if (!map) return;
#if !defined(ESP_PLATFORM)
    free(map->pool);
#endif
    map->pool  = NULL;
    map->count = 0;
}

static IRAM_ATTR void _update(QuadTreeMap *map, uint16_t idx,
                              float xmn, float xmx, float ymn, float ymx,
                              float x, float y, int8_t delta)
{
    QTNode *n = &map->pool[idx];

    // max depth = this is a leaf -> update value + return.
    if (n->depth >= QT_MAX_DEPTH) {
        n->value = _clamp((int)n->value + (int)delta);
        return;
    }

    // internal node : find/create the right child then descend.
    int q = _quadrant(xmn, xmx, ymn, ymx, x, y);

    if (n->children[q] == QT_NULL) {
        uint16_t child = _alloc(map, n->depth + 1);
        if (child == QT_NULL) {
#if defined(ESP_PLATFORM)
            static bool s_pool_full_warned;
            if (!s_pool_full_warned) {
                s_pool_full_warned = true;
                ESP_LOGW(TAG_QT,
                         "node pool full (%u nodes); further qt_update calls are dropped",
                         (unsigned)QT_POOL_SIZE);
            }
#endif
            return;
        }
        // Re-read n: _alloc may have changed pool pointer on realloc.
        // (Here pool is fixed size so pointer is stable, but good
        //  practice.)
        map->pool[idx].children[q] = child;
    }

    float cxmn, cxmx, cymn, cymx;
    _child_bounds(xmn, xmx, ymn, ymx, q, &cxmn, &cxmx, &cymn, &cymx);
    _update(map, map->pool[idx].children[q],
            cxmn, cxmx, cymn, cymx, x, y, delta);
}

IRAM_ATTR void qt_update(QuadTreeMap *map, float x, float y, int8_t delta)
{
    if (!map || !map->pool) return;
    if (x < map->x_min || x >= map->x_max) return; // ignore out-of-bounds positions.
    if (y < map->y_min || y >= map->y_max) return;
    _update(map, 1 /* root */,
            map->x_min, map->x_max, map->y_min, map->y_max,
            x, y, delta);
}

static int8_t _query(const QuadTreeMap *map, uint16_t idx,
                     float xmn, float xmx, float ymn, float ymx,
                     float x, float y)
{
    const QTNode *n = &map->pool[idx];

    // leaf -> return stored value
    if (n->depth >= QT_MAX_DEPTH) return n->value;

    int q = _quadrant(xmn, xmx, ymn, ymx, x, y);
    if (n->children[q] == QT_NULL) return 0; // never observed

    float cxmn, cxmx, cymn, cymx;
    _child_bounds(xmn, xmx, ymn, ymx, q, &cxmn, &cxmx, &cymn, &cymx);
    return _query(map, n->children[q],
                  cxmn, cxmx, cymn, cymx, x, y);
}

int8_t qt_query(QuadTreeMap *map, float x, float y)
{
    if (!map || !map->pool) return 0;
    if (x < map->x_min || x >= map->x_max) return 0;
    if (y < map->y_min || y >= map->y_max) return 0;
    return _query(map, 1,
                  map->x_min, map->x_max, map->y_min, map->y_max,
                  x, y);
}

int8_t qt_query_const(const QuadTreeMap *map, float x, float y)
{
    if (!map || !map->pool) return 0;
    if (x < map->x_min || x >= map->x_max) return 0;
    if (y < map->y_min || y >= map->y_max) return 0;
    return _query(map, 1,
                  map->x_min, map->x_max, map->y_min, map->y_max,
                  x, y);
}

//  qt_iterate_occupied

static void _iterate(const QuadTreeMap *map, uint16_t idx,
                     float xmn, float xmx, float ymn, float ymx,
                     void (*cb)(float, float, int8_t, void *), void *ud)
{
    if (idx == QT_NULL) return;
    const QTNode *n = &map->pool[idx];

    // Leaf : report if occupied.
    if (n->depth >= QT_MAX_DEPTH) {
        if (n->value > 0)
            cb(0.5f*(xmn+xmx), 0.5f*(ymn+ymx), n->value, ud);
        return;
    }

    // internal : recurse into existing children only.
    for (int q = 0; q < 4; q++) {
        if (n->children[q] == QT_NULL) continue;
        float cxmn, cxmx, cymn, cymx;
        _child_bounds(xmn, xmx, ymn, ymx, q, &cxmn, &cxmx, &cymn, &cymx);
        _iterate(map, n->children[q],
                 cxmn, cxmx, cymn, cymx, cb, ud);
    }
}

void qt_iterate_occupied(QuadTreeMap *map,
                         void (*cb)(float cx, float cy,
                                    int8_t value, void *userdata),
                         void *userdata)
{
    if (!map || !map->pool || !cb) return;
    _iterate(map, 1,
             map->x_min, map->x_max, map->y_min, map->y_max,
             cb, userdata);
}


// report actual used nodes × node size.
// Node size is fixed at 10 bytes on every platform because we

size_t qt_memory_bytes(const QuadTreeMap *map)
{
    if (!map) return 0;
    return (size_t)(map->count) * sizeof(QTNode);
}


/* ── qt_compact ─────────────────────────────────────────────────────────────
 * Snapshot all confident wall cells, wipe the pool, re-insert.
 *
 * Up to QT_COMPACT_MAX cells are saved in a static BSS buffer (no stack
 * allocation).  After the in-place pool reset, each saved cell is
 * re-inserted with qt_update(map, cx, cy, saved_value): because the leaf
 * starts at 0 after the reset, adding saved_value as the delta restores
 * the exact log-odds value in one call.
 * ────────────────────────────────────────────────────────────────────────── */
#define QT_COMPACT_MAX 512

typedef struct { float cx, cy; int8_t value; } _compact_cell_t;

static _compact_cell_t _s_compact_buf[QT_COMPACT_MAX];
static int             _s_compact_cnt = 0;
static int8_t          _s_compact_min = 0;

static void _compact_cb(float cx, float cy, int8_t value, void *ud)
{
    (void)ud;
    if (value < _s_compact_min)        return;
    if (_s_compact_cnt >= QT_COMPACT_MAX) return;
    _s_compact_buf[_s_compact_cnt].cx    = cx;
    _s_compact_buf[_s_compact_cnt].cy    = cy;
    _s_compact_buf[_s_compact_cnt].value = value;
    _s_compact_cnt++;
}

void qt_compact(QuadTreeMap *map, int8_t min_value)
{
    if (!map || !map->pool) return;

    /* 1. Collect confident occupied cells */
    _s_compact_cnt = 0;
    _s_compact_min = min_value;
    qt_iterate_occupied(map, _compact_cb, NULL);

    uint16_t saved = (uint16_t)_s_compact_cnt;
    uint16_t before = map->count;

    /* 2. Reset pool in-place — no malloc/free, just wipe and reinitialise */
    memset(map->pool, 0, (size_t)QT_POOL_SIZE * sizeof(QTNode));
    map->count = 1;   /* slot 0 stays reserved as QT_NULL */
    _alloc(map, 1);   /* recreate root at index 1, depth 1 */

    /* 3. Re-insert saved cells — leaf starts at 0, so delta = saved value */
    for (int i = 0; i < _s_compact_cnt; i++) {
        qt_update(map, _s_compact_buf[i].cx,
                       _s_compact_buf[i].cy,
                       _s_compact_buf[i].value);
    }

#if defined(ESP_PLATFORM)
    ESP_LOGI(TAG_QT, "compact: %u→%u nodes  saved=%u cells  freed=%u nodes",
             (unsigned)before, (unsigned)map->count,
             (unsigned)saved,
             (unsigned)(before - map->count));
#else
    (void)before; (void)saved;
#endif
}