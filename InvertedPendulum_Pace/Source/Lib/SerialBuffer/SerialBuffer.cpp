#include "SerialBuffer.h"

#include <iostream>

using namespace MARTe;

namespace MFI {

SerialBuffer::SerialBuffer(uint32 size) {
    start_ = new uint8[size];
    end_ = start_ + size;
    head_ = tail_ = start_;
    size_ = size;
    count_ = 0;
}

SerialBuffer::~SerialBuffer() {
    delete [] start_;
}

uint32 SerialBuffer::size() const {
    return size_;
}

uint32 SerialBuffer::count() const {
    return count_;
}

uint32 SerialBuffer::available() const {
    return size_ - count_;
}

uint32 SerialBuffer::queue(const uint8* buffer, uint32 size) {
    if ((count_ + size) > size_) {
        size = size_ - count_;
    }

    for (uint32 i = 0; i < size; i++) {
        *tail_ = buffer[i];
        tail_ = _advance(tail_, 1);
    }
    count_ += size;

    return size;
}

uint32 SerialBuffer::dequeue(uint8* buffer, uint32 size) {
    if (size > count_) {
        size = count_;
    }

    for (uint32 i = 0; i < size; i++) {
        buffer[i] = *head_;
        head_ = _advance(head_, 1);
    }
    count_ -= size;

    return size;
}

void SerialBuffer::empty() {
    count_ = 0u;
    head_ = tail_ = start_;
}

void SerialBuffer::empty(uint32 n) {
    if (n > count_) {
        n = count_;
    }
    
    for (uint32 i = 0; i < n; i++) {
        head_ = _advance(head_, 1);
        count_--;
    }
}

bool SerialBuffer::find(uint8 value, uint32& index, uint32 start) {
    if (start >= count_) {
        return false;
    }
    
    uint8* ptr = _advance(head_, start);
    for (uint32 i = start; i < count_; i++) {       
        if (*ptr == value) {
            index = i;
            
            return true;
        }

        ptr = _advance(ptr, 1);
    }

    return false;
}

bool SerialBuffer::at(uint32 index, uint8& value) {   
    if (index < count_) {
        uint8* ptr = _advance(head_, index);
        value = *ptr;
            
        return true;
    }

    return false;
}

uint8* SerialBuffer::_advance(uint8* ptr, uint32 n) {
    for (uint32 i = 0; i < n; i++) {
        ptr++;

        if (ptr >= end_) {
            ptr = start_;
        }
    }

    return ptr;
}

void print_buffer(SerialBuffer& buffer) {
    uint8 value;
    for (uint32 i = 0; i < buffer.count(); i++) {
        buffer.at(i, value);
        std::cout << std::hex << static_cast<unsigned short int>(value);
        std::cout << " ";
    }
    std::cout << std::endl;
}

} // namespace MFI