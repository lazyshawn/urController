
#ifndef USER_INTERFACE_H
#define USER_INTERFACE_H

#include "common.h"
#include "data_exchange.h"
#include "thread_pool.h"
// 监听键盘事件
#include <stdio.h>
#include <termio.h>

void interface_thread_function(void);
int scanKeyboard(void);
void display_menu(void);
void display_current_information(SVO svoLocal);
void teleoperate_robot(void);

#endif

