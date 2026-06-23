#ifndef sfbg_lqr_occ__h_
#define sfbg_lqr_occ__h_
#ifndef sfbg_lqr_occ__COMMON_INCLUDES_
#define sfbg_lqr_occ__COMMON_INCLUDES_
#include "rtwtypes.h"
#endif

#include "sfbg_lqr_occ__types.h"

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

typedef struct {
  real_T Idk;
  real_T Iqk;
  real_T id_ref;
  real_T iq_ref;
  real_T idpast;
  real_T iqpast;
} ExternalInputs_sfbg_lqr_occ_;

typedef struct {
  real_T Vd;
  real_T Vq;
} ExternalOutputs_sfbg_lqr_occ_;

struct tag_RTM_sfbg_lqr_occ_ {
  const char_T * volatile errorStatus;
};

extern ExternalInputs_sfbg_lqr_occ_ sfbg_lqr_occ__U;
extern ExternalOutputs_sfbg_lqr_occ_ sfbg_lqr_occ__Y;
extern void sfbg_lqr_occ__initialize(void);
extern void sfbg_lqr_occ__step(void);
extern void sfbg_lqr_occ__terminate(void);
extern RT_MODEL_sfbg_lqr_occ_ *const sfbg_lqr_occ__M;

#endif

