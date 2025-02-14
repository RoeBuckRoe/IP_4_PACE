#include "DataSourceSignalChecker.h"

using namespace MARTe;

namespace MFI {

bool DataSourceCheckSignalProperties(DataSourceI& dataSource, uint32 signalIdx, TypeDescriptor type, 
                                     uint32 dimensions, uint32 elements) {
    bool ok = (dataSource.GetSignalType(signalIdx) == type);

    if (ok) {
        uint8 numberOfDimensions;
        ok = dataSource.GetSignalNumberOfDimensions(signalIdx, numberOfDimensions);
        if (ok) {            
            ok = (numberOfDimensions == dimensions);
        }

        uint32 numberOfElements;
        if (ok) {
            ok = dataSource.GetSignalNumberOfElements(signalIdx, numberOfElements);
        }
    }

    return ok;        
}

} // namespace MFI 
