#ifndef CODE_ELEMENT_LINE_H_
#define CODE_ELEMENT_LINE_H_


extern int element_line_L,element_line_R,element_line_U,element_line_B,element_line_M;

void element_update();
int count_valid_exits();
int check_sharp_turn_state(int lookahead_y);

#endif /* CODE_ELEMENT_LINE_H_ */
