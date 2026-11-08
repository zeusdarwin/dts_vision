#pragma once
#include <memory>
namespace dts_vision {}
#define DTS_IMPL_DEFINITION(CLASS) \
public: \
    ~CLASS() noexcept; \
\
private: \
    struct Impl; \
    std::unique_ptr<Impl> _impl; \
\
public:
