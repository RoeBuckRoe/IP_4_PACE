#ifndef MFI_DATASOURCES_STM32_DATAFRAME_H_
#define MFI_DATASOURCES_STM32_DATAFRAME_H_

#include "CompilerTypes.h"
#include "SerialBuffer.h"

namespace MFI {
namespace DataFrame {

extern const MARTe::uint8 SYNC_BYTES[4];
const MARTe::uint32 RX_FRAME_SIZE = 8u + 4u; //12;

/**
 * @brief Remove any leading non-message bytes from the buffer
 * 
 * Removes any data from the buffer which precedes the synchronisation bytes
 */
MARTe::uint32 SanitiseRxBuffer(SerialBuffer& buffer);

/**
 * @brief The contents of an Rx data frame
 */

class RxDataFrame {
 public:    
    RxDataFrame();
    MARTe::int32 positionRotor;
    MARTe::uint32 encoder_counter;
};

/**
 * @brief Parse a raw Rx data frame
 * 
 * Assumes that the buffer is the correct length for an Rx data frame, and that it does not include
 * the synchronisation bytes at the start of the data frame
 */
void ParseRxDataFramePayload(MARTe::uint8* raw, RxDataFrame& dataframe);

/**
 * @brief Get the next available STM32 Rx message from the buffer. The contents of the message are 
 * removed from the buffer as part of the extraction.
 * 
 * Assumes that SanitiseBuffer has been called on the buffer immediately prior, so that there are no
 * leading non-message bytes in the buffer
 * 
 * Returns true if a message was extracted successfully, otherwise false
 */
bool GetNextRxDataFrame(SerialBuffer& buffer, RxDataFrame& dataframe);

} // namespace DataFrame
} // namespace MFI

#endif // MFI_DATASOURCES_STM32_DATAFRAME_H_