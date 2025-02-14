#ifndef MFI_GAM_SIGNAL_CHECKER_H_
#define MFI_GAM_SIGNAL_CHECKER_H_

#include "GAM.h"

namespace MFI {

bool GAMCheckSignalProperties(MARTe::GAM& gam, const MARTe::char8* const name, 
                              MARTe::SignalDirection direction,
                              MARTe::TypeDescriptor type, MARTe::uint32 dimensions,
                              MARTe::uint32 elements,
                              MARTe::uint32& signalIdx);

} // namespace MFI

#endif // MFI_GAM_SIGNAL_CHECKER_H_