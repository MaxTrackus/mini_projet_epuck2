#ifndef CENTRAL_UNIT_H
#define CENTRAL_UNIT_H

//start the central unit thread
void central_unit_start(void);

void set_mode_with_selector(void);
void set_straight_move_in_mm(uint32_t distance_in_mm);

#endif /* CENTRAL_UNIT_H */
