

#include <stdint.h>
#include <string.h>

#include <thingset.h>
#include <thingset/sdk.h>

/*
 * Groups / first layer data object IDs
 */
#define ID_ROOT        0x00

/* Measurements */
#define ID_MEAS        0x5
#define ID_MEAS_V1_LOW 0x50
#define ID_MEAS_V2_LOW 0x51
#define ID_MEAS_V_HIGH 0x52
#define ID_MEAS_I1_LOW 0x53
#define ID_MEAS_I2_LOW 0x54
#define ID_MEAS_I_HIGH 0x55
#define ID_MEAS_TEMP1  0x56
#define ID_MEAS_TEMP2  0x57

/*
 * Subset definitions for statements and publish/subscribe
 */

/* UART serial */
#define SUBSET_SER  (1U << 0)
/* CAN bus */
#define SUBSET_CAN  (1U << 1)
/* Control data sent and received via CAN */
#define SUBSET_CTRL (1U << 3)

/* Measure variables */
static float32_t V1_low_value;
static float32_t V2_low_value;
static float32_t I1_low_value;
static float32_t I2_low_value;
static float32_t I_high_value;
static float32_t V_high_value;

static float32_t temp_1_value;
static float32_t temp_2_value;

/* Temporary storage fore measured value (ctrl task) */
static float32_t meas_data;

/* ThingSet object definitions */
THINGSET_ADD_GROUP(ID_ROOT, ID_MEAS, "Measurements", THINGSET_NO_CALLBACK);

THINGSET_ADD_ITEM_FLOAT(ID_MEAS, ID_MEAS_V1_LOW, "rV1Low_V", &V1_low_value, 2,
                        THINGSET_ANY_R, SUBSET_CAN);

THINGSET_ADD_ITEM_FLOAT(ID_MEAS, ID_MEAS_V2_LOW, "rV2Low_V", &V2_low_value, 2,
                        THINGSET_ANY_R, SUBSET_CAN);

THINGSET_ADD_ITEM_FLOAT(ID_MEAS, ID_MEAS_V_HIGH, "rVHigh_V", &V_high_value, 2,
                        THINGSET_ANY_R, SUBSET_CAN);

THINGSET_ADD_ITEM_FLOAT(ID_MEAS, ID_MEAS_I1_LOW, "rI1Low_A", &I1_low_value, 2,
                        THINGSET_ANY_R, SUBSET_CAN);

THINGSET_ADD_ITEM_FLOAT(ID_MEAS, ID_MEAS_I2_LOW, "rI2Low_A", &I2_low_value, 2,
                        THINGSET_ANY_R, SUBSET_CAN);

THINGSET_ADD_ITEM_FLOAT(ID_MEAS, ID_MEAS_I_HIGH, "rIHigh_A", &I_high_value, 2,
                        THINGSET_ANY_R, SUBSET_CAN);

THINGSET_ADD_ITEM_FLOAT(ID_MEAS, ID_MEAS_TEMP1, "rTemp_degC", &temp_1_value, 2,
                        THINGSET_ANY_R, SUBSET_CAN);

THINGSET_ADD_ITEM_FLOAT(ID_MEAS, ID_MEAS_TEMP2, "rTemp2_degC", &temp_2_value, 2,
                        THINGSET_ANY_R, SUBSET_CAN);

