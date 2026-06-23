#ifndef ff_occ__h_
#define ff_occ__h_
#ifndef ff_occ__COMMON_INCLUDES_
#define ff_occ__COMMON_INCLUDES_
#include "rtwtypes.h"
#endif

#include "ff_occ__types.h"

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

typedef struct {
  real_T id_ref;
  real_T iq_ref;
} ExternalInputs_ff_occ_;

typedef struct {
  real_T Vd;
  real_T Vq;
} ExternalOutputs_ff_occ_;

struct tag_RTM_ff_occ_ {
  const char_T * volatile errorStatus;
};

extern ExternalInputs_ff_occ_ ff_occ__U;
extern ExternalOutputs_ff_occ_ ff_occ__Y;
extern void ff_occ__initialize(void);
extern void ff_occ__step(void);
extern void ff_occ__terminate(void);
extern RT_MODEL_ff_occ_ *const ff_occ__M;

#endif

