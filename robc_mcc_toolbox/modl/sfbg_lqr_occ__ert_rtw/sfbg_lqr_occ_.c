#include "sfbg_lqr_occ_.h"
#include "rtwtypes.h"

ExternalInputs_sfbg_lqr_occ_ sfbg_lqr_occ__U;
ExternalOutputs_sfbg_lqr_occ_ sfbg_lqr_occ__Y;
static RT_MODEL_sfbg_lqr_occ_ sfbg_lqr_occ__M_;
RT_MODEL_sfbg_lqr_occ_ *const sfbg_lqr_occ__M = &sfbg_lqr_occ__M_;
void sfbg_lqr_occ__step(void)
{
  real_T uk[2];
  real_T tmp;
  real_T tmp_0;
  real_T tmp_1;
  real_T tmp_2;
  int32_T i;
  static const real_T a[8] = { -2.1048657758239382, -0.016532948317620054,
    0.016532948317585734, -2.1048657758239142, -0.14658342193564106,
    0.01657949847896651, -0.016579498478969876, -0.14658342193564119 };

  tmp = sfbg_lqr_occ__U.Idk - sfbg_lqr_occ__U.idpast;
  tmp_0 = sfbg_lqr_occ__U.Iqk - sfbg_lqr_occ__U.iqpast;
  tmp_1 = sfbg_lqr_occ__U.Idk - sfbg_lqr_occ__U.id_ref;
  tmp_2 = sfbg_lqr_occ__U.Iqk - sfbg_lqr_occ__U.iq_ref;
  for (i = 0; i < 2; i++) {
    uk[i] = ((a[i + 2] * tmp_0 + a[i] * tmp) + a[i + 4] * tmp_1) + a[i + 6] *
      tmp_2;
  }

  sfbg_lqr_occ__Y.Vd = uk[0];
  sfbg_lqr_occ__Y.Vq = uk[1];
}

void sfbg_lqr_occ__initialize(void)
{
}

void sfbg_lqr_occ__terminate(void)
{
}
