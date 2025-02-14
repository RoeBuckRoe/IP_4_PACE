#ifndef MFI_STM32_WRITER_H_
#define MFI_STM32_WRITER_H_

#include "MemoryMapSynchronisedOutputBroker.h"
#include "STM32.h"

namespace MFI {

class STM32Writer : public MARTe::MemoryMapSynchronisedOutputBroker {
 public:

    CLASS_REGISTER_DECLARATION();
    
    STM32Writer();
    virtual ~STM32Writer();

    virtual bool Init(const MARTe::SignalDirection direction,
                      MARTe::DataSourceI &dataSourceIn,
                      const MARTe::char8 * const functionName,
                      void * const gamMemoryAddress);
    
    virtual bool Execute();

 private:

    STM32* stm32;
};

} // namespace MFI
#endif // MFI_STM32_WRITER_H_
