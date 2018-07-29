// NRT: Struct to help with multi-threading without using a mutex.
#pragma once

enum ReadWriteControl : unsigned int
{
    RW_NONE = 0x0,
    RW_WRITING = 0x01,
    RW_READING = 0x02,
};

template<class T>
struct ReadOrWriteObject
{
    std::atomic<ReadWriteControl> lock;
    T object;
};
