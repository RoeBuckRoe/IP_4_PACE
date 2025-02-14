#ifndef MFI_DATA_SOURCE_SIGNAL_CHECKER_H_
#define MFI_DATA_SOURCE_SIGNAL_CHECKER_H_

#include "DataSourceI.h"

namespace MFI {

bool DataSourceCheckSignalProperties(MARTe::DataSourceI& dataSource, MARTe::uint32 signalIdx, 
                           MARTe::TypeDescriptor type, MARTe::uint32 dimensions, 
                           MARTe::uint32 elements);

} // namespace MFI

#endif // MFI_DATA_SOURCE_SIGNAL_CHECKER_H_
