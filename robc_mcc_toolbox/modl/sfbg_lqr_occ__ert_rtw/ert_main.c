#include <stddef.h>
#include <stdio.h>
#include "sfbg_lqr_occ_.h"

void rt_OneStep(void);
void rt_OneStep(void)
{
  static boolean_T OverrunFlag = false;
  if (OverrunFlag) {
    rtmSetErrorStatus(sfbg_lqr_occ__M, "Overrun");
    return;
  }

  OverrunFlag = true;
  sfbg_lqr_occ__step();
  OverrunFlag = false;
}

int_T main(int_T argc, const char *argv[])
{
  (void)(argc);
  (void)(argv);
  sfbg_lqr_occ__initialize();
  printf("Warning: The simulation will run forever. "
         "Generated ERT main won't simulate model step behavior. "
         "To change this behavior select the 'MAT-file logging' option.\n");
  fflush((NULL));
  while (rtmGetErrorStatus(sfbg_lqr_occ__M) == (NULL)) {
  }

  sfbg_lqr_occ__terminate();
  return 0;
}
