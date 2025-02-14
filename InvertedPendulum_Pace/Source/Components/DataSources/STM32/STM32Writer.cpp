#include "STM32Writer.h"
#include "AdvancedErrorManagement.h"

using namespace MARTe;

namespace MFI {

STM32Writer::STM32Writer() : MemoryMapSynchronisedOutputBroker() {
    stm32 = NULL;
}

STM32Writer::~STM32Writer() {

}

bool STM32Writer::Init(const SignalDirection direction,
                      DataSourceI &dataSourceIn,
                      const char8 * const functionName,
                      void * const gamMemoryAddress) {
    bool ret = MemoryMapSynchronisedOutputBroker::Init(direction, dataSourceIn, functionName, gamMemoryAddress);
    if (ret) {
        stm32 = dynamic_cast<STM32*>(&dataSourceIn);
        ret = (stm32 != NULL);
        if (!ret) {
            REPORT_ERROR(ErrorManagement::FatalError, "Failed dynamic_cast from DataSourceI* to STM32*");
        }
    }

    return ret;
}

bool STM32Writer::Execute() {    
    bool ret = MemoryMapSynchronisedOutputBroker::Execute();

    stm32->TxSynchronise();

    return ret;
}

CLASS_REGISTER(STM32Writer, "1.0");

} // namespace MFI
