#ifndef QUADRUPED_DESCRIPTION_H
#define QUADRUPED_DESCRIPTION_H

#include <quadruped_base/quadruped_base.h>

namespace champ
{
    namespace URDF
    {
        void loadFromHeader(champ::QuadrupedBase &base)
        {
      base.lf.hip.setOrigin(0.00723678392728447, 0.0485586045266115, -0.0196700000000003, 0.0, 0.0, 0.0);
base.lf.upper_leg.setOrigin(0.0524464493931691, 0.0209174990447602, -0.0240000000000004, 0.0, 0.0, 0.0);
base.lf.lower_leg.setOrigin(0.0, 0.00500250095523978, -0.0649999643759598, 0.0, 0.0, 0.0);
     base.lf.foot.setOrigin(0.0, 0.00499999999999999, -0.074999999999994, 0.0, 0.0, 0.0);

      base.rf.hip.setOrigin(0.00723678407973241, -0.0485586045266115, -0.0196700000000008, 0.0, 0.0, 0.0);
base.rf.upper_leg.setOrigin(0.0524464492407212, -0.0209174999999999, -0.0239999999999998, 0.0, 0.0, 0.0);
base.rf.lower_leg.setOrigin(0.0, -0.00500249999999999, -0.0649999643759603, 0.0, 0.0, 0.0);
     base.rf.foot.setOrigin(0.0, -0.005, -0.074999999999994, 0.0, 0.0, 0.0);

      base.lh.hip.setOrigin(-0.0692103140818923, 0.0485586045266112, -0.0196700000000008, 0.0, 0.0, 0.0);
base.lh.upper_leg.setOrigin(-0.0524464492407207, 0.0209175, -0.0239999999999998, 0.0, 0.0, 0.0);
base.lh.lower_leg.setOrigin(0.0, 0.00500249999999998, -0.0649999643759606, 0.0, 0.0, 0.0);
     base.lh.foot.setOrigin(0.0, 0.00499999999999999, -0.074999999941437, 0.0, 0.0, 0.0);

      base.rh.hip.setOrigin(-0.069210317591168, -0.0485586045266115, -0.0196700000000008, 0.0, 0.0, 0.0);
base.rh.upper_leg.setOrigin(-0.052446449240721, -0.0209174999999999, -0.0239999999999998, 0.0, 0.0, 0.0);
base.rh.lower_leg.setOrigin(0.0, -0.00500249999999999, -0.0649999643759606, 0.0, 0.0, 0.0);
     base.rh.foot.setOrigin(0.0, -0.005, -0.074999999941437, 0.0, 0.0, 0.0);
        }
    }
}
#endif