#pragma once

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <streambuf>
#include <iostream>

#ifndef _ASSET_H_
#define _ASSET_H_

extern "C" {

// Define the asset struct
typedef struct __attribute__((__packed__)) _asset {
    uint8_t* buf;
    size_t size;
} asset;

// Custom streambuf to read from asset data
class AssetStreambuf : public std::streambuf {
public:
    AssetStreambuf(const asset& a) {
        // Set the buffer to point to the start of the asset data and set the end pointer
        setg(reinterpret_cast<char*>(a.buf), reinterpret_cast<char*>(a.buf), reinterpret_cast<char*>(a.buf) + a.size);
    }

protected:
    // Override underflow to read data from the asset buffer
    int underflow() override {
        if (gptr() < egptr()) {
            return static_cast<unsigned char>(*gptr());
        }
        return EOF;
    }
};

// Custom input stream class that uses the AssetStreambuf
class AssetStream : public std::istream {
public:
    AssetStream(const asset& a) 
        : std::istream(nullptr), // Initialize with nullptr for the buffer
          buf(a) // Create the custom stream buffer
    {
        // Associate the stream with the buffer
        rdbuf(&buf);
    }

    // Delete copy constructor and copy assignment operator
    AssetStream(const AssetStream&) = delete;
    AssetStream& operator=(const AssetStream&) = delete;

private:
    AssetStreambuf buf;
};

// Define the macro for embedding static assets
#define STATIC_FILE(x)                                                                                                 \
    extern "C" {                                                                                                       \
    extern uint8_t _binary_static_##x##_start[], _binary_static_##x##_size[];                                          \
    static asset x##_asset = {_binary_static_##x##_start, (size_t)_binary_static_##x##_size};                          \
    static AssetStream x(x##_asset);                                                                                   \
    }


} // extern "C"

#endif // _ASSET_H_
