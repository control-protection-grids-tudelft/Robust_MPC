#include "sfb_ccc_.h"
#include "rtwtypes.h"

ExternalInputs_sfb_ccc_ sfb_ccc__U;
ExternalOutputs_sfb_ccc_ sfb_ccc__Y;
static RT_MODEL_sfb_ccc_ sfb_ccc__M_;
RT_MODEL_sfb_ccc_ *const sfb_ccc__M = &sfb_ccc__M_;
void sfb_ccc__step(void)
{
  real_T tmp;
  real_T tmp_0;
  tmp = sfb_ccc__U.Idk - sfb_ccc__U.IdR;
  tmp_0 = sfb_ccc__U.Iqk - sfb_ccc__U.IqR;
  sfb_ccc__Y.Vd = 0.097867465691326483 * tmp + 0.0015342812223293461 * tmp_0;
  sfb_ccc__Y.Vq = -0.0015342835501127572 * tmp + 0.097867465756776614 * tmp_0;
}

void sfb_ccc__initialize(void)
{
}

void sfb_ccc__terminate(void)
{
}
