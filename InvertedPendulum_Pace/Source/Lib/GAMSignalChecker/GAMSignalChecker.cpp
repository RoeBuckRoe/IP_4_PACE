#include "GAMSignalChecker.h"

using namespace MARTe;

namespace MFI {

bool GAMCheckSignalProperties(GAM& gam, const char8* const name, 
                           SignalDirection direction,
                           TypeDescriptor type, uint32 dimensions,
                           uint32 elements,
                           uint32& signalIdx) {
    bool ok = gam.GetSignalIndex(direction, signalIdx, name);
    
    if (ok) {
        TypeDescriptor typeVariableOut = gam.GetSignalType(direction, signalIdx);
        ok = (typeVariableOut == type);
    }
    
    if (ok) {
        uint32 numberOfDimensions;
        ok = gam.GetSignalNumberOfDimensions(direction, signalIdx, numberOfDimensions);
        if (ok) {
            ok = (numberOfDimensions == dimensions);
        }
    }    
    
    if (ok) {
        uint32 numberOfElements;
        ok = gam.GetSignalNumberOfElements(direction, signalIdx, numberOfElements);
        if (ok) {
            ok = (numberOfElements == elements);
        }
    }

    return ok;
}

} // namespace MFI 
