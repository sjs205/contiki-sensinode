#define MOTOR_PORT 1
#define MOTOR_A_PIN P1_2
#define MOTOR_A_MASK 0x04
#define MOTOR_B_PIN P1_3
#define MOTOR_B_MASK 0x08

#define DOOR_SENSOR_PORT 1
#define DOOR_SENSOR_TOP_PIN P0_6
#define DOOR_SENSOR_TOP_MASK 0x40
#define DOOR_SENSOR_BOTTOM_PIN P0_7
#define DOOR_SENSOR_BOTTOM_MASK 0x80

/* Active-low inputs */
typedef enum {
  DOOR_UNKNOWN,
  DOOR_CLOSED,
  DOOR_OPEN,
  DOOR_ERROR,
} door_state;

void door_init();
void door_open();
void door_close();
void door_stop();
door_state get_door_state();
door_state set_door_state(door_state state);
