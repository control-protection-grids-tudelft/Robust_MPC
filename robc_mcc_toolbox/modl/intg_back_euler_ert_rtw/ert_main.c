#include <stddef.h>
#include <stdio.h>
#include "intg_back_euler.h"

void rt_OneStep(void);
void rt_OneStep(void)
{
  static boolean_T OverrunFlag = false;
  if (OverrunFlag) {
    rtmSetErrorStatus(intg_back_euler_M, "Overrun");
    return;
  }

  OverrunFlag = true;
  intg_back_euler_step();
  OverrunFlag = false;
}

int_T main(int_T argc, const char *argv[])
{
  (void)(argc);
  (void)(argv);
  intg_back_euler_initialize();
  printf("Warning: The simulation will run forever. "
         "Generated ERT main won't simulate model step behavior. "
         "To change this behavior select the 'MAT-file logging' option.\n");
  fflush((NULL));
  while (rtmGetErrorStatus(intg_back_euler_M) == (NULL)) {
  }

  intg_back_euler_terminate();
  return 0;
}
