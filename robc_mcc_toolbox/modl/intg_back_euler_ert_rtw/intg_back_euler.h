#ifndef intg_back_euler_h_
#define intg_back_euler_h_
#ifndef intg_back_euler_COMMON_INCLUDES_
#define intg_back_euler_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif

#include "intg_back_euler_types.h"

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

typedef struct {
  real_T DiscreteTimeIntegrator1_DSTATE;
  uint8_T DiscreteTimeIntegrator1_IC_LOAD;
} D_Work_intg_back_euler;

typedef struct {
  real_T du;
  real_T u0;
} ExternalInputs_intg_back_euler;

typedef struct {
  real_T u;
} ExternalOutputs_intg_back_euler;

struct tag_RTM_intg_back_euler {
  const char_T * volatile errorStatus;
};

extern D_Work_intg_back_euler intg_back_euler_DWork;
extern ExternalInputs_intg_back_euler intg_back_euler_U;
extern ExternalOutputs_intg_back_euler intg_back_euler_Y;
extern void intg_back_euler_initialize(void);
extern void intg_back_euler_step(void);
extern void intg_back_euler_terminate(void);
extern RT_MODEL_intg_back_euler *const intg_back_euler_M;

#endif

