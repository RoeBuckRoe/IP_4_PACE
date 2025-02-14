#include "STM32Reader.h"
#include "AdvancedErrorManagement.h"

using namespace MARTe;

namespace MFI {

STM32Reader::STM32Reader() : MemoryMapSynchronisedInputBroker() {
    stm32 = NULL;
}

STM32Reader::~STM32Reader() {

}

bool STM32Reader::Init(const SignalDirection direction,
                      DataSourceI &dataSourceIn,
                      const char8 * const functionName,
                      void * const gamMemoryAddress) {
    bool ret = MemoryMapSynchronisedInputBroker::Init(direction, dataSourceIn, functionName, gamMemoryAddress);
    if (ret) {
        stm32 = dynamic_cast<STM32*>(&dataSourceIn);
        ret = (stm32 != NULL);
        if (!ret) {
            REPORT_ERROR(ErrorManagement::FatalError, "Failed dynamic_cast from DataSourceI* to STM32*");
        }
    }

    return ret;
}

bool STM32Reader::Execute() {   
    stm32->RxSynchronise();
    
    return MemoryMapSynchronisedInputBroker::Execute();
}

CLASS_REGISTER(STM32Reader, "1.0");

} // namespace MFI
