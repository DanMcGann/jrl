#pragma once
#include <pybind11/pybind11.h>

#include <boost/optional.hpp>

namespace PYBIND11_NAMESPACE {
namespace detail {
template <typename T>
struct type_caster<std::optional<T>> : optional_caster<std::optional<T>> {};
}  // namespace detail
}  // namespace PYBIND11_NAMESPACE
