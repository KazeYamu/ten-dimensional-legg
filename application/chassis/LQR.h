#ifndef _LQR_H
#define _LQR_H

#include "chassis.h"
#include "balance.h"

void LQR_calc(ten_balance_observation *observe,ten_balance_target *balance_targets,four_control_data *control_datas);
void LQR_SetAir(uint8_t air);   /* 手册ch9: 悬空降维开关(轮相关通道清零) */

#endif
