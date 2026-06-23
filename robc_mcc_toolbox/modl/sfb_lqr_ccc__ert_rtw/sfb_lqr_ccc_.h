#ifndef sfb_lqr_ccc__h_
#define sfb_lqr_ccc__h_
#ifndef sfb_lqr_ccc__COMMON_INCLUDES_
#define sfb_lqr_ccc__COMMON_INCLUDES_
#include "rtwtypes.h"
#endif

#include "sfb_lqr_ccc__types.h"

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

typedef struct {
  real_T Idk;
  real_T Iqk;
  real_T IdR;
  real_T IqR;
} ExternalInputs_sfb_lqr_ccc_;

typedef struct {
  real_T Vd;
  real_T Vq;
} ExternalOutputs_sfb_lqr_ccc_;

struct tag_RTM_sfb_lqr_ccc_ {
  const char_T * volatile errorStatus;
};

extern ExternalInputs_sfb_lqr_ccc_ sfb_lqr_ccc__U;
extern ExternalOutputs_sfb_lqr_ccc_ sfb_lqr_ccc__Y;
extern void sfb_lqr_ccc__initialize(void);
extern void sfb_lqr_ccc__step(void);
extern void sfb_lqr_ccc__terminate(void);
extern RT_MODEL_sfb_lqr_ccc_ *const sfb_lqr_ccc__M;

#endif

