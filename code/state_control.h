#ifndef CODE_STATE_CONTROL_H_
#define CODE_STATE_CONTROL_H_


typedef enum {
    normal,
    fork_straight,
    fork_turn
}state;

typedef enum {
    angle,
    none
}element_state;

extern state car_state;
extern element_state line_state;

void state_update();
void show_state();


#endif /* CODE_STATE_CONTROL_H_ */
