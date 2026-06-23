#include "intg_back_euler.h"

D_Work_intg_back_euler intg_back_euler_DWork;
ExternalInputs_intg_back_euler intg_back_euler_U;
ExternalOutputs_intg_back_euler intg_back_euler_Y;
static RT_MODEL_intg_back_euler intg_back_euler_M_;
RT_MODEL_intg_back_euler *const intg_back_euler_M = &intg_back_euler_M_;
void intg_back_euler_step(void)
{
  if (intg_back_euler_DWork.DiscreteTimeIntegrator1_IC_LOAD != 0) {
    intg_back_euler_DWork.DiscreteTimeIntegrator1_DSTATE = intg_back_euler_U.u0;
    if (intg_back_euler_DWork.DiscreteTimeIntegrator1_DSTATE > 1.1) {
      intg_back_euler_DWork.DiscreteTimeIntegrator1_DSTATE = 1.1;
    } else if (intg_back_euler_DWork.DiscreteTimeIntegrator1_DSTATE < -1.1) {
      intg_back_euler_DWork.DiscreteTimeIntegrator1_DSTATE = -1.1;
    }
  }

  intg_back_euler_Y.u = intg_back_euler_DWork.DiscreteTimeIntegrator1_DSTATE +
    intg_back_euler_U.du;
  if (intg_back_euler_Y.u > 1.1) {
    intg_back_euler_Y.u = 1.1;
  } else if (intg_back_euler_Y.u < -1.1) {
    intg_back_euler_Y.u = -1.1;
  }

  intg_back_euler_DWork.DiscreteTimeIntegrator1_IC_LOAD = 0U;
  intg_back_euler_DWork.DiscreteTimeIntegrator1_DSTATE = intg_back_euler_Y.u;
}

void intg_back_euler_initialize(void)
{
  intg_back_euler_DWork.DiscreteTimeIntegrator1_IC_LOAD = 1U;
}

void intg_back_euler_terminate(void)
{
}
