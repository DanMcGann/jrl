#include <pybind11/pybind11.h>

#include <boost/optional.hpp>

namespace PYBIND11_NAMESPACE {
namespace detail {
template <typename T>
struct type_caster<boost::optional<T>> : optional_caster<boost::optional<T>> {};
}  // namespace detail
}  // namespace PYBIND11_NAMESPACE
