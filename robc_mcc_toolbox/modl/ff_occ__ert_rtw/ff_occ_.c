#include "ff_occ_.h"

ExternalInputs_ff_occ_ ff_occ__U;
ExternalOutputs_ff_occ_ ff_occ__Y;
static RT_MODEL_ff_occ_ ff_occ__M_;
RT_MODEL_ff_occ_ *const ff_occ__M = &ff_occ__M_;
void ff_occ__step(void)
{
  ff_occ__Y.Vd = 0.0060826446280996815 * ff_occ__U.id_ref + 0.255294369383557 *
    ff_occ__U.iq_ref;
  ff_occ__Y.Vq = -0.255294369383557 * ff_occ__U.id_ref + 0.0060826446280996807 *
    ff_occ__U.iq_ref;
}

void ff_occ__initialize(void)
{
}

void ff_occ__terminate(void)
{
}
