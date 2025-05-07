#include "IPStatusGAM.h"

#include "AdvancedErrorManagement.h"

//TODO
#include <iostream>

namespace InvertedPendulum {

IPStatusGAM::IPStatusGAM() : GAM(),
                         inputEncoderPosition(NULL_PTR(MARTe::uint32*)),
                         outputSwitchState(NULL_PTR(MARTe::uint8*)) {
}

IPStatusGAM::~IPStatusGAM() {
}

bool IPStatusGAM::Setup() {
    bool ok = GetNumberOfInputSignals() == 1;
    if (!ok) {
        REPORT_ERROR(MARTe::ErrorManagement::ParametersError, "Number of input signals must be 1.");
    }
    if (ok) {
        ok = GetSignalType(MARTe::InputSignals, 0u) == MARTe::UnsignedInteger32Bit;
        if (ok) {
            inputEncoderPosition = static_cast<MARTe::uint32*>(GetInputSignalMemory(0u));
        }
        else {
            REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "First input signal shall be of type "
                    "uint32");
        }
    }
    if (ok) {
        ok = GetNumberOfOutputSignals() == 1;
        if (!ok) {
            REPORT_ERROR(MARTe::ErrorManagement::ParametersError, "Number of output signals must be 1.");
        }
    }
    if (ok) {
        ok = GetSignalType(MARTe::OutputSignals, 0u) == MARTe::UnsignedInteger8Bit;
        if (ok) {
            outputSwitchState = static_cast<MARTe::uint8*>(GetOutputSignalMemory(0u));
        }
        else {
            REPORT_ERROR(MARTe::ErrorManagement::InitialisationError, "First output signal shall be of type "
                    "uint8");
        }
    }
    return ok;
}

bool IPStatusGAM::Execute() {
    const int acceptableRangeMin = 1150; // Define your acceptable range minimum
    const int acceptableRangeMax = 1250; // Define your acceptable range maximum

    // Check if the encoder position is within the acceptable range.
    if (*inputEncoderPosition >= acceptableRangeMin && *inputEncoderPosition <= acceptableRangeMax) {
        *outputSwitchState = 3;
    }
    else {
        *outputSwitchState = 1;
    }

    return true;
}

bool IPStatusGAM::PrepareNextState(const MARTe::char8* const currentStateName,
                                 const MARTe::char8* const nextStateName) {
    return true;
}

CLASS_REGISTER(IPStatusGAM, "1.0");

} // namespace InvertedPendulum
