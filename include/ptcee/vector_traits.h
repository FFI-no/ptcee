// Copyright (c) 2024 Norwegian Defence Research Establishment (FFI)

#pragma once

#include "Eigen/Core"

#include <cstddef>
#include <type_traits>
#include <utility>

namespace ptc
{
/// \brief Helper template for providing the dimension and Real-type of a type.
///
/// All custom types to be used with DMap must specialize this template,
/// which means that a definition of `Real` and `dim` is required.
/// ```cpp
/// template<>
/// ptc::vector_traits<MyClass>
/// {
///   using Real = double;
///   static constexpr size_t dim = 2;
/// }
///
/// The only exceptions to this are classes that are derived from Eigen::Matrix with non-dynamic size, derived from
/// [arithmetic types](https://en.cppreference.com/w/c/language/arithmetic_types), or derived from tuples or pairs of
/// such types. In this case you can omit the specialization of `vector_traits`, unless you want to override the
/// dimension or real type.
/// ```
template<typename T, typename = void>
struct vector_traits;

// ----- Implementation -----
template<typename T>
struct vector_traits<const T&> : public vector_traits<T, void>
{};

template<typename T>
struct vector_traits<const T> : public vector_traits<T, void>
{};

template<typename T>
struct vector_traits<T&> : public vector_traits<T, void>
{};

template<typename U, typename V>
struct vector_traits<std::pair<U, V>, void>
{
  using Real = typename vector_traits<U>::Real;
  static constexpr auto dim = vector_traits<U>::dim + vector_traits<V>::dim;
};

template<typename... Ts>
struct vector_traits<std::tuple<Ts...>, void>
{
  using FirstT = std::tuple_element_t<0, std::tuple<Ts...>>;
  using Real = typename vector_traits<FirstT>::Real;
  static constexpr auto dim = (vector_traits<Ts>::dim + ...);
};

namespace detail
{
template<typename T, typename = void>
struct base_traits;

// ----- Implementation -----
template<typename T>
struct base_traits<T, std::enable_if_t<std::is_arithmetic_v<T>>>
{
  using Real = std::remove_const_t<std::remove_reference_t<T>>;
  static constexpr int dim = 1;
};

template<typename Derived>
struct base_traits<Eigen::MatrixBase<Derived>>

{
  static_assert(Derived::IsVectorAtCompileTime, "Cannot get vector_traits for non-vector Eigen::Matrix");

  using Real = typename Derived::Scalar;
  static constexpr int dim = Derived::RowsAtCompileTime;
};

template<typename T, typename V>
T resolve_traits_impl(const V&);

template<typename Derived>
auto resolve_traits_impl(const Eigen::MatrixBase<Derived>&) -> base_traits<Eigen::MatrixBase<Derived>>;

template<typename T>
using resolve_traits_t = decltype(resolve_traits_impl(std::declval<T>()));

template<typename T, typename = typename base_traits<T>::Real>
std::true_type has_base_traits_impl(const T&);

std::false_type has_base_traits_impl(...);

template<typename T, typename = typename resolve_traits_t<T>::Real>
std::true_type has_resolvable_traits_impl(const T&);

std::false_type has_resolvable_traits_impl(...);

template <class T>
using has_base_traits = decltype(has_base_traits_impl(std::declval<const T&>()));

template <class T>
using has_resolvable_traits = decltype(has_resolvable_traits_impl(std::declval<const T&>()));

template<typename T, bool v>
constexpr bool assert_unspecialized_traits()
{
  static_assert(v, "You forgot to specialize ptc::vector_traits<> for your type");

  return v;
}
}

template<typename T>
struct vector_traits<T, std::enable_if_t<!std::is_const_v<T> && std::is_object_v<T> && detail::has_base_traits<T>::value>> : public detail::base_traits<T>
{};

template<typename T>
struct vector_traits<T, std::enable_if_t<!std::is_const_v<T> && std::is_object_v<T> && !detail::has_base_traits<T>::value && detail::has_resolvable_traits<T>::value>> : public detail::resolve_traits_t<T>
{};

template<typename T, typename>
struct vector_traits
{
  static constexpr auto value = detail::assert_unspecialized_traits<T, false>();
};
}
