#pragma once

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <streambuf>
#include <iostream>

#ifndef _ASSET_H_
#define _ASSET_H_

extern "C" {

typedef struct __attribute__((__packed__)) _asset {
    uint8_t* buf;
    size_t size;
} asset;

#define ASSET(x)                                                                                                       \
    extern "C" {                                                                                                       \
    extern uint8_t _binary_static_##x##_start[], _binary_static_##x##_size[];                                          \
    static asset x = {_binary_static_##x##_start, (size_t)_binary_static_##x##_size};                                  \
    }

class AssetStreambuf : public std::streambuf {
public:
    AssetStreambuf(const asset& a) {
        setg(reinterpret_cast<char*>(a.buf), reinterpret_cast<char*>(a.buf), reinterpret_cast<char*>(a.buf) + a.size);
    }

protected:
    int underflow() override {
        if (gptr() < egptr()) {
            return static_cast<unsigned char>(*gptr());
        }
        return EOF;
    }
};

class AssetStream : public std::istream {
public:
    AssetStream(const asset& a) 
        : std::istream(nullptr),
          buf(a)
    {
        
        rdbuf(&buf);
    }

    AssetStream(const AssetStream&) = delete;
    AssetStream& operator=(const AssetStream&) = delete;

private:
    AssetStreambuf buf;
};

}

#endif