#include "sfbg_occ_.h"
#include "rtwtypes.h"

ExternalInputs_sfbg_occ_ sfbg_occ__U;
ExternalOutputs_sfbg_occ_ sfbg_occ__Y;
static RT_MODEL_sfbg_occ_ sfbg_occ__M_;
RT_MODEL_sfbg_occ_ *const sfbg_occ__M = &sfbg_occ__M_;
void sfbg_occ__step(void)
{
  real_T uk[2];
  real_T tmp;
  real_T tmp_0;
  real_T tmp_1;
  real_T tmp_2;
  int32_T i;
  static const real_T a[8] = { -0.17069379295078535, -0.0011477179912480068,
    0.0011418297085013722, -0.17069659132330939, -0.00094051479688594811,
    0.00021450617923844342, -0.00021452786867247439, -0.0009405382434120017 };

  tmp = sfbg_occ__U.Idk - sfbg_occ__U.idpast;
  tmp_0 = sfbg_occ__U.Iqk - sfbg_occ__U.iqpast;
  tmp_1 = sfbg_occ__U.Idk - sfbg_occ__U.id_ref;
  tmp_2 = sfbg_occ__U.Iqk - sfbg_occ__U.iq_ref;
  for (i = 0; i < 2; i++) {
    uk[i] = ((a[i + 2] * tmp_0 + a[i] * tmp) + a[i + 4] * tmp_1) + a[i + 6] *
      tmp_2;
  }

  sfbg_occ__Y.Vd = uk[0];
  sfbg_occ__Y.Vq = uk[1];
}

void sfbg_occ__initialize(void)
{
}

void sfbg_occ__terminate(void)
{
}
