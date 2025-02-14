#ifndef MFI_STM32_READER_H_
#define MFI_STM32_READER_H_

#include "MemoryMapSynchronisedInputBroker.h"
#include "STM32.h"

namespace MFI {

class STM32Reader : public MARTe::MemoryMapSynchronisedInputBroker {
 public:

    CLASS_REGISTER_DECLARATION();
    
    STM32Reader();
    virtual ~STM32Reader();

    virtual bool Init(const MARTe::SignalDirection direction,
                      MARTe::DataSourceI &dataSourceIn,
                      const MARTe::char8 * const functionName,
                      void * const gamMemoryAddress);
    
    virtual bool Execute();

 private:

    STM32* stm32;
};

} // namespace MFI
#endif // MFI_STM32_READER_H_
