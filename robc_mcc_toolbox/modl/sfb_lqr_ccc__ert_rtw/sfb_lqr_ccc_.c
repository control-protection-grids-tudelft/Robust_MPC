#include "sfb_lqr_ccc_.h"
#include "rtwtypes.h"

ExternalInputs_sfb_lqr_ccc_ sfb_lqr_ccc__U;
ExternalOutputs_sfb_lqr_ccc_ sfb_lqr_ccc__Y;
static RT_MODEL_sfb_lqr_ccc_ sfb_lqr_ccc__M_;
RT_MODEL_sfb_lqr_ccc_ *const sfb_lqr_ccc__M = &sfb_lqr_ccc__M_;
void sfb_lqr_ccc__step(void)
{
  real_T tmp;
  real_T tmp_0;
  tmp = sfb_lqr_ccc__U.Idk - sfb_lqr_ccc__U.IdR;
  tmp_0 = sfb_lqr_ccc__U.Iqk - sfb_lqr_ccc__U.IqR;
  sfb_lqr_ccc__Y.Vd = 0.099303274033402208 * tmp + 0.0015599849697825 * tmp_0;
  sfb_lqr_ccc__Y.Vq = -0.0015599849697824585 * tmp + 0.099303274033404623 *
    tmp_0;
}

void sfb_lqr_ccc__initialize(void)
{
}

void sfb_lqr_ccc__terminate(void)
{
}
