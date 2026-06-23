#include <stddef.h>
#include <stdio.h>
#include "sfb_ccc_.h"

void rt_OneStep(void);
void rt_OneStep(void)
{
  static boolean_T OverrunFlag = false;
  if (OverrunFlag) {
    rtmSetErrorStatus(sfb_ccc__M, "Overrun");
    return;
  }

  OverrunFlag = true;
  sfb_ccc__step();
  OverrunFlag = false;
}

int_T main(int_T argc, const char *argv[])
{
  (void)(argc);
  (void)(argv);
  sfb_ccc__initialize();
  printf("Warning: The simulation will run forever. "
         "Generated ERT main won't simulate model step behavior. "
         "To change this behavior select the 'MAT-file logging' option.\n");
  fflush((NULL));
  while (rtmGetErrorStatus(sfb_ccc__M) == (NULL)) {
  }

  sfb_ccc__terminate();
  return 0;
}
