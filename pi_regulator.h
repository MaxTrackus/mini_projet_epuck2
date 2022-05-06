#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

void set_enablePiRegulator(bool status);

//start the PI regulator thread
void pi_regulator_start(void);

#endif /* PI_REGULATOR_H */
