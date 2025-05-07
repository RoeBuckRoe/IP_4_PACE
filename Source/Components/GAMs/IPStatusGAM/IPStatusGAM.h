#ifndef SOURCE_COMPONENTS_GAMS_IPStatusGAM_IPStatusGAM_H_
#define SOURCE_COMPONENTS_GAMS_IPStatusGAM_IPStatusGAM_H_

#include "GAM.h"
#include "StatefulI.h"

namespace InvertedPendulum {

class IPStatusGAM : public MARTe::GAM, public MARTe::StatefulI {
public:
    CLASS_REGISTER_DECLARATION()

    IPStatusGAM();

    virtual ~IPStatusGAM();

    virtual bool Setup();

    virtual bool Execute();

    virtual bool PrepareNextState(const MARTe::char8* const currentStateName,
                                  const MARTe::char8* const nextStateName);

private:
    MARTe::uint32* inputEncoderPosition;
    MARTe::uint8* outputSwitchState;
};

} // namespace InvertedPendulum

#endif // SOURCE_COMPONENTS_GAMS_IPStatusGAM_IPStatusGAM_H_
