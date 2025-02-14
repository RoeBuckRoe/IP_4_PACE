#include "SerialBuffer.h"
#include "CompilerTypes.h"
#include "DataFrame.h"

#include "gtest/gtest.h"

using namespace MFI;
using namespace MARTe;

namespace dataframe_test {

bool buffers_equal(const uint8* buffer1, const uint8* buffer2, uint32 size) {
    for (uint32 i = 0; i < size; i++) {
        if (buffer1[i] != buffer2[i]) {
            return false;
        }
    }

    return true;
}

bool buffer_contains(SerialBuffer& buffer, const uint8* expected, uint32 expected_len) {
    if (buffer.count() != expected_len) {
        return false;
    }
    
    uint8* temp = new uint8[expected_len];
    buffer.dequeue(temp, expected_len);
    bool ret = buffers_equal(temp, expected, expected_len);
    delete [] temp;

    return ret;
}

}

class SanitisingABuffer : public ::testing::Test {
 public:
    SanitisingABuffer() : BUFSIZE(10u), buffer(BUFSIZE) {}

 protected:
    const uint32 BUFSIZE;
    SerialBuffer buffer;
};

TEST_F(SanitisingABuffer, GivesEmptyBufferIfBufferWasEmpty) {   
    DataFrame::SanitiseRxBuffer(buffer);

    ASSERT_EQ(0u, buffer.count());
}

TEST_F(SanitisingABuffer, DiscardsNoBytesIfBufferWasEmpty) {   
    uint32 discarded = DataFrame::SanitiseRxBuffer(buffer);

    ASSERT_EQ(0u, discarded);
}

TEST_F(SanitisingABuffer, DoesNotDiscardIfBufferContainsFirstSyncByte) {   
    buffer.queue(DataFrame::SYNC_BYTES, 1u);
    
    uint32 discarded = DataFrame::SanitiseRxBuffer(buffer);
    
    ASSERT_TRUE(discarded == 0u && 
                dataframe_test::buffer_contains(buffer, DataFrame::SYNC_BYTES, 1u));
}

TEST_F(SanitisingABuffer, DiscardsIfFirstSyncByteFollowedByNonSyncByte) {   
    buffer.queue(DataFrame::SYNC_BYTES, 1u);
    uint8 temp = 0xFF;
    buffer.queue(&temp, 1u);
    
    uint32 discarded = DataFrame::SanitiseRxBuffer(buffer);
    
    ASSERT_TRUE(discarded == 2u && buffer.count() == 0u);
}

class SanitisingABufferWithSyncBytesOnly: public testing::TestWithParam<uint32> {
 public:
    SanitisingABufferWithSyncBytesOnly() : BUFSIZE(10u), buffer(BUFSIZE) {
        uint32 nbytes = GetParam();
        buffer.queue(DataFrame::SYNC_BYTES, nbytes);
    }

 protected:
    const uint32 BUFSIZE;
    SerialBuffer buffer;
};

TEST_P(SanitisingABufferWithSyncBytesOnly, DoesNotDiscardAnyBytes) {
    uint32 discarded = DataFrame::SanitiseRxBuffer(buffer);
    
    ASSERT_TRUE(discarded == 0u &&
                dataframe_test::buffer_contains(buffer, DataFrame::SYNC_BYTES, GetParam())); 
}

INSTANTIATE_TEST_CASE_P(AllNumberOfSyncBytes,
                         SanitisingABufferWithSyncBytesOnly,
                         testing::Values(1u, 2u, 3u, 4u));

class SanitisingABufferWithSyncBytesPrecededByNonSyncByte: public testing::TestWithParam<uint32> {
 public:
    SanitisingABufferWithSyncBytesPrecededByNonSyncByte() : BUFSIZE(10u), buffer(BUFSIZE) {
        uint8 temp = 0xFF;
        buffer.queue(&temp, 1u);
        uint32 nbytes = GetParam();
        buffer.queue(DataFrame::SYNC_BYTES, nbytes);
    }

 protected:
    const uint32 BUFSIZE;
    SerialBuffer buffer;
};

TEST_P(SanitisingABufferWithSyncBytesPrecededByNonSyncByte, DiscardsNonSyncByte) {
    uint32 discarded = DataFrame::SanitiseRxBuffer(buffer);
    
    ASSERT_TRUE(discarded == 1u &&
                dataframe_test::buffer_contains(buffer, DataFrame::SYNC_BYTES, GetParam())); 
}

INSTANTIATE_TEST_CASE_P(AllNumberOfSyncBytes,
                         SanitisingABufferWithSyncBytesPrecededByNonSyncByte,
                         testing::Values(1u, 2u, 3u, 4u));

class SanitisingABufferWithSyncBytesPrecededByTwoNonSyncBytes: public testing::TestWithParam<uint32> {
 public:
    SanitisingABufferWithSyncBytesPrecededByTwoNonSyncBytes() : BUFSIZE(10u), buffer(BUFSIZE) {
        uint8 temp[2] = {0xFF, 0xFF};
        buffer.queue(temp, 2u);
        uint32 nbytes = GetParam();
        buffer.queue(DataFrame::SYNC_BYTES, nbytes);
    }

 protected:
    const uint32 BUFSIZE;
    SerialBuffer buffer;
};

TEST_P(SanitisingABufferWithSyncBytesPrecededByTwoNonSyncBytes, DiscardsNonSyncByte) {
    uint32 discarded = DataFrame::SanitiseRxBuffer(buffer);
    
    ASSERT_TRUE(discarded == 2u &&
                dataframe_test::buffer_contains(buffer, DataFrame::SYNC_BYTES, GetParam())); 
}

INSTANTIATE_TEST_CASE_P(AllNumberOfSyncBytes,
                         SanitisingABufferWithSyncBytesPrecededByTwoNonSyncBytes,
                         testing::Values(1u, 2u, 3u, 4u));

class SanitisingABufferWithSyncBytesPrecededByThreeNonSyncBytes: public testing::TestWithParam<uint32> {
 public:
    SanitisingABufferWithSyncBytesPrecededByThreeNonSyncBytes() : BUFSIZE(10u), buffer(BUFSIZE) {
        uint8 temp[3] = {0xFF, 0xFF, 0x00};
        buffer.queue(temp, 3u);
        uint32 nbytes = GetParam();
        buffer.queue(DataFrame::SYNC_BYTES, nbytes);
    }

 protected:
    const uint32 BUFSIZE;
    SerialBuffer buffer;
};

TEST_P(SanitisingABufferWithSyncBytesPrecededByThreeNonSyncBytes, DiscardsNonSyncByte) {
    uint32 discarded = DataFrame::SanitiseRxBuffer(buffer);
    
    ASSERT_TRUE(discarded == 3u &&
                dataframe_test::buffer_contains(buffer, DataFrame::SYNC_BYTES, GetParam())); 
}

INSTANTIATE_TEST_CASE_P(AllNumberOfSyncBytes,
                         SanitisingABufferWithSyncBytesPrecededByThreeNonSyncBytes,
                         testing::Values(1u, 2u, 3u, 4u));

class SanitisingABufferWithMultipleSyncBytes: public ::testing::Test {
 public:
    SanitisingABufferWithMultipleSyncBytes() : BUFSIZE(10u), buffer(BUFSIZE) {
        uint8 temp[] = {0x00, 0xAC, 0xDD, 0xAC, 0x00, 0x00, 0x00, 0xAC, 0xDD};
        buffer.queue(temp, sizeof(temp));
    }

 protected:
    const uint32 BUFSIZE;
    SerialBuffer buffer;
};

TEST_F(SanitisingABufferWithMultipleSyncBytes, DiscardsNonSyncBytes) {
    uint32 discarded = DataFrame::SanitiseRxBuffer(buffer);
    
    ASSERT_TRUE(discarded == 6u &&
                dataframe_test::buffer_contains(buffer, DataFrame::SYNC_BYTES, 3u)); 
}

class SanitisingABufferWithNoSyncBytes: public testing::TestWithParam<uint32> {
 public:
    SanitisingABufferWithNoSyncBytes() : BUFSIZE(10u), buffer(BUFSIZE) {
        uint32 nbytes = GetParam();
        for (uint32 i = 1; i <= nbytes; i++) {
            uint8 n = i;
            buffer.queue(&n, 1u);
        }
    }

 protected:
    const uint32 BUFSIZE;
    SerialBuffer buffer;
};

TEST_P(SanitisingABufferWithNoSyncBytes, DiscardsAllBytes) {
    uint32 discarded = DataFrame::SanitiseRxBuffer(buffer);
    
    ASSERT_TRUE(discarded == GetParam() && buffer.count() == 0u); 
}

INSTANTIATE_TEST_CASE_P(AVarietyOfNumbersOfSyncBytes,
                         SanitisingABufferWithNoSyncBytes,
                         testing::Values(1u, 3u, 5u, 7u));