#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <assert.h>

#include "quadtree_map.h"

/* 
 * Tests:
 *   1. Single insert + query
 *   2. Log-odds accumulation and clamping
 *   3. 10 000 cells : all queries return correct value
 *   4. Memory stays under 200 KB after 10 000 insertions
 *   5. iterate_occupied only visits cells with value > 0
 *   6. Out-of-bounds updates and queries are safe (no crash)
 * */

#define ROOM_M 10.0f
#define NUM_CELLS 10000
#define RAM_LIMIT (200 * 1024)   // 200 KB 



static int g_run    = 0;
static int g_passed = 0;

#define TEST(name, cond)                                        \
    do {                                                        \
        g_run++;                                                \
        if (cond) {                                             \
            printf("  [PASS] %s\n", name);                     \
            g_passed++;                                         \
        } else {                                                \
            printf("  [FAIL] %s\n", name);                     \
        }                                                       \
    } while (0)

//deterministic pseudo-random (LCG)
static uint32_t _rng(uint32_t *s)
{
    *s = *s * 1664525u + 1013904223u;
    return *s;
}

static float _rand_pos(uint32_t *s)
{                              
    return (float)(_rng(s) % 10000) / 1000.0f; // Returns a float in [0, ROOM_M)  
}

// Test 1:  single insert + query 
static void test_single_cell(void)
{
    printf("\nTest 1 : single insert + query\n");

    QuadTreeMap map;
    qt_init(&map, 0.0f, ROOM_M, 0.0f, ROOM_M);

    qt_update(&map, 3.0f, 4.0f, QT_HIT_INC);

    int8_t hit = qt_query(&map, 3.0f, 4.0f);
    TEST("hit cell has positive value", hit > 0);

    int8_t untouched = qt_query(&map, 9.0f, 9.0f);
    TEST("untouched cell returns 0", untouched == 0);

    qt_free(&map);
}

// Test 2: accumulation and clamping 
static void test_accumulation(void)
{
    printf("\nTest 2 : log-odds accumulation and clamping\n");

    QuadTreeMap map;
    qt_init(&map, 0.0f, ROOM_M, 0.0f, ROOM_M);

    
    for (int i = 0; i < 5; i++) // five hits -> raise the value
        qt_update(&map, 5.0f, 5.0f, QT_HIT_INC);
    int8_t after5 = qt_query(&map, 5.0f, 5.0f);
    // 5 × QT_HIT_INC = 75 > QT_VALUE_MAX = 40 : value clamps at max after 3 hits (3×15=45 → clamped to 40)
    TEST("5 hits : value == QT_VALUE_MAX (clamped)", after5 == QT_VALUE_MAX);
    qt_update(&map, 5.0f, 5.0f, QT_MISS_DEC); // One free-ray update must lower the value
    int8_t after_miss = qt_query(&map, 5.0f, 5.0f);
    TEST("miss decreases value", after_miss < after5);

    
    for (int i = 0; i < 200; i++)// Repeated hits must clamp at QT_VALUE_MAX
        qt_update(&map, 2.0f, 2.0f, QT_HIT_INC);
    int8_t clamped = qt_query(&map, 2.0f, 2.0f);
    TEST("value clamped at QT_VALUE_MAX", clamped == QT_VALUE_MAX);
    for (int i = 0; i < 200; i++)// repeated misses must clamp at QT_VALUE_MIN
        qt_update(&map, 7.0f, 7.0f, QT_MISS_DEC);
    int8_t clamped_free = qt_query(&map, 7.0f, 7.0f);
    TEST("value clamped at QT_VALUE_MIN", clamped_free == QT_VALUE_MIN);

    qt_free(&map);
}

// Test 3 : 10 000 random cells/ all queries correct
static void test_10k_correctness(void)
{
    printf("\nTest 3 : 10 000 cells — all queries correct\n");

    QuadTreeMap map;
    qt_init(&map, 0.0f, ROOM_M, 0.0f, ROOM_M);

    uint32_t seed = 0xDEADBEEF;
    float xs[NUM_CELLS], ys[NUM_CELLS];

    for (int i = 0; i < NUM_CELLS; i++) {
        xs[i] = _rand_pos(&seed);
        ys[i] = _rand_pos(&seed);
        qt_update(&map, xs[i], ys[i], QT_HIT_INC);
    }

    int wrong = 0;
    for (int i = 0; i < NUM_CELLS; i++) {
        if (qt_query(&map, xs[i], ys[i]) <= 0) wrong++;
    }

    TEST("all 10 000 cells query correctly (value > 0)", wrong == 0);
    if (wrong)
        printf("    (failed : %d / %d)\n", wrong, NUM_CELLS);

    qt_free(&map);
}

// Test 4 : memory under 200 KB 
static void test_memory(void)
{
    printf("\nTest 4 : memory under 200 KB\n");

    QuadTreeMap map;
    qt_init(&map, 0.0f, ROOM_M, 0.0f, ROOM_M);

    uint32_t seed = 0xCAFEBABE;
    for (int i = 0; i < NUM_CELLS; i++)
        qt_update(&map, _rand_pos(&seed), _rand_pos(&seed), QT_HIT_INC);

    size_t used = qt_memory_bytes(&map);
    printf("    node_count = %u  |  sizeof(QTNode) = %zu bytes  |  "
           "total = %zu bytes (%.1f KB)\n",
           (unsigned)map.count, sizeof(QTNode), used, used / 1024.0);

    TEST("memory < 200 KB", used < RAM_LIMIT);

    qt_free(&map);
}

// Test 5 : iterate_occupied
static int s_iter_total = 0;
static int s_iter_wrong = 0;

static void _cb(float cx, float cy, int8_t value, void *ud)
{
    (void)cx; (void)cy; (void)ud;
    s_iter_total++;
    if (value <= 0) s_iter_wrong++;
}

static void test_iterate(void)
{
    printf("\nTest 5 : iterate_occupied\n");
    QuadTreeMap map;
    qt_init(&map, 0.0f, ROOM_M, 0.0f, ROOM_M);
    // Insert 100 obstacle cells at known distinct grid positions 
    //(0.5m spacing → each maps to a different depth-6 cell)
    uint32_t seed = 0xABCD1234;
    (void)seed;
    int inserted = 0;
    for (int row = 0; row < 10; row++) {
        for (int col = 0; col < 10; col++) {
            float x = 0.2f + col * 0.3f;   // 0.2, 0.5, 0.8 … 2.9 m 
            float y = 0.2f + row * 0.3f;
            qt_update(&map, x, y, QT_HIT_INC);
            inserted++;
        }
    }

    // insert 100 free-space cells on the right side (no overlap)
    for (int row = 0; row < 10; row++) {
        for (int col = 0; col < 10; col++) {
            float x = 6.0f + col * 0.3f;
            float y = 0.2f + row * 0.3f;
            qt_update(&map, x, y, QT_MISS_DEC);
        }
    }

    s_iter_total = 0;
    s_iter_wrong = 0;
    qt_iterate_occupied(&map, _cb, NULL);

    TEST("iterate visits exactly the 100 obstacle cells",
         s_iter_total == inserted);
    TEST("iterate never reports a non-positive cell",
         s_iter_wrong == 0);

    qt_free(&map);
}

// Test 6 : out-of-bounds safety 
static void test_out_of_bounds(void)
{
    printf("\nTest 6 : out-of-bounds safety\n");

    QuadTreeMap map;
    qt_init(&map, 0.0f, ROOM_M, 0.0f, ROOM_M);

    // These should be silently ignored -> no crash, no corruption 
    qt_update(&map, -1.0f,  5.0f, QT_HIT_INC);
    qt_update(&map, 11.0f,  5.0f, QT_HIT_INC);
    qt_update(&map,  5.0f, -1.0f, QT_HIT_INC);
    qt_update(&map,  5.0f, 11.0f, QT_HIT_INC);

    int8_t v1 = qt_query(&map, -1.0f, 5.0f);
    int8_t v2 = qt_query(&map, 11.0f, 5.0f);

    TEST("out-of-bounds update ignored, query returns 0",
         v1 == 0 && v2 == 0);

    // valid cell must still work after the bad updates
    qt_update(&map, 5.0f, 5.0f, QT_HIT_INC);
    TEST("valid cell still works after bad updates",
         qt_query(&map, 5.0f, 5.0f) > 0);

    qt_free(&map);
}



int main(void)
{
    printf("=== quadtree_map test suite ===\n");
    printf("Room %.0f m × %.0f m | MAX_DEPTH %d | "
           "QT_POOL_SIZE %d | node size %zu bytes\n",
           ROOM_M, ROOM_M, QT_MAX_DEPTH, QT_POOL_SIZE, sizeof(QTNode));

    test_single_cell();
    test_accumulation();
    test_10k_correctness();
    test_memory();
    test_iterate();
    test_out_of_bounds();

    printf("\n==============================\n");
    printf("Result : %d / %d tests passed\n", g_passed, g_run);
    return (g_passed == g_run) ? 0 : 1;
}