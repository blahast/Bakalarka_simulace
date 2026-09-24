#pragma once

#include <memory>
#include <utility>
using std::move;

template<typename T>
using unique_c_ptr = std::unique_ptr<T, void(*)(T*)>;