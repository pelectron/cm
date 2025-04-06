/**
 *  Copyright 2024-2025 Pelé Constam
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
#ifndef CORTEX_EVENT_HPP
#define CORTEX_EVENT_HPP

#include "cortex/assert.hpp"
#include "cortex/common.hpp"
#include "cortex/function.hpp"
#include "cortex/nvic.hpp"
#include "cortex/type_list.hpp"
#include "type_list/type_list.hpp"
#include <bitset>
#include <concepts>
#include <functional>
#include <iterator>
#include <type_traits>
#include <utility>
namespace cm {

namespace {
  template<typename T, typename = void>
  struct boolean_castable : std::false_type {};
  template<typename T>
  struct boolean_castable<
      T, std::void_t<decltype(static_cast<bool>(std::declval<T>()))>>
      : std::true_type {};

  template<typename T, typename = void>
  struct has_signature_typedef : std::false_type {};

  template<typename T>
  struct has_signature_typedef<T, std::void_t<typename T::signature>>
      : std::true_type {};

} // namespace

template<typename UH>
concept UserHandler = std::default_initializable<UH> and FunctionConcept<UH> and
                      boolean_castable<UH, void>::value;

/**
 * @brief This concept describes an iterable sequence of values with push_back
 * and clear methods.
 *
 * A container C needs to have:
 * - an inner typedef called ``value_type``
 * - ``begin()`` and ``end()`` methods that return stl style iterators
 * to the beginning and end of the container
 * - a ``clear()`` method to reset the sequence
 * - a ``push_back`` method that takes a parameter of type ``value_type``
 *
 * @tparam C the sequence type
 */
template<typename C>
concept Container = requires(C&& container, typename C::value_type t) {
  { container.begin() };
  { container.end() };
  { container.clear() };
  { container.push_back(t) };
} and std::default_initializable<C>;

/**
 * @brief This concept describes a Container with ``value_type`` == ``T``.
 *
 * @tparam C the container
 * @tparam T the value type
 */
template<typename C, typename T>
concept ContainerOf = Container<C> and std::same_as<T, typename C::value_type>;

template<typename C, typename T>
concept PushBackAble = requires(C c, T&& t) {
  { c.push_back(std::forward<T>(t)) };
};

/**
 * @brief An accumulator accumulates values and is used by the
 * BroadcastEventHandler to accumualte return values.
 *
 * An accumulator A needs to have:
 * - an inner typedef called ``value_type``
 * - a ``clear()`` method to reset the accumulator
 * - a ``push_back`` method that takes a parameter of type ``value_type``.
 * ``push_back`` called by the BroadcastEventHandler for every user handler that
 * is envoked.
 * - an optional const method ``result()`` which returns the Accumulators
 * result. If not provided, BroadcastEventHandler::get_return_value will return
 * the accumulator, else BroadcastEventHandler::get_return_value will return
 * accum.result().
 *
 * @tparam A
 */
template<typename A>
concept Accumulator = requires(A& accum, typename A::value_type t) {
  { accum.clear() };
  { accum.push_back(t) };
} and std::default_initializable<A>;

/**
 * @brief This concept describes an Accumulator with ``value_type`` == ``T``.
 *
 * @tparam A the accumulator
 * @tparam T the value type
 */
template<typename A, typename T>
concept AccumulatorOf =
    Container<A> and std::same_as<T, typename A::value_type>;

template<typename A>
concept HasResult = Accumulator<A> and requires(A&& a) {
  { a.result() };
};

/**
 * A Container with the ``value_type`` satisfying the UserHandler concept.
 *
 * @tparam T the user handler list
 */
template<typename T>
concept UserHandlerList = Container<T> and UserHandler<typename T::value_type>;

namespace {
  template<typename H, typename IndexList, typename ArgList, typename = void>
  struct has_set_arg : std::false_type {};

  template<std::size_t...>
  struct value_list {};

  template<typename H, template<std::size_t...> typename L1,
           template<typename...> typename L2, std::size_t... Is,
           typename... Args>
  struct has_set_arg<
      H, L1<Is...>, L2<Args...>,
      std::void_t<decltype(std::declval<H>().template set_arg<Is...>(
          std::declval<Args>()...))>> : std::true_type {};

  template<typename H, template<typename T, T...> typename L1,
           template<typename...> typename L2, typename T, T... Is,
           typename... Args>
  struct has_set_arg<
      H, L1<T, Is...>, L2<Args...>,
      std::void_t<decltype(std::declval<H>().template set_arg<Is...>(
          std::declval<Args>()...))>> : std::true_type {};

  template<typename H, typename IndexList, typename ArgList>
  inline constexpr bool has_set_arg_v =
      has_set_arg<H, IndexList, ArgList, void>::value;

  template<typename H, typename Indices, typename ArgList, typename = void>
  struct can_set_args_impl : std::false_type {};

  template<typename H, template<typename T, T...> typename L1,
           template<typename...> typename L2, typename T, T... Is,
           typename... Args>
  struct can_set_args_impl<
      H, L1<T, Is...>, L2<Args...>,
      std::void_t<decltype(std::declval<H>().template set_arg<Is>(
          std::declval<Args>()))...>> : std::true_type {};

  template<typename H, typename... Args>
  struct can_set_args
      : can_set_args_impl<H,
                          decltype(std::make_index_sequence<sizeof...(Args)>()),
                          TypeList<Args...>> {};

  template<typename T>
  struct par_tuple_return {
    T ret_val;
    void set_return_value(const T& t) { ret_val = t; }
    void set_return_value(T&& t) { ret_val = std::move(t); }
    T return_value() const { return ret_val; }
    void set_return_value(const T& t) volatile { ret_val = t; }
    void set_return_value(T&& t) volatile { ret_val = std::move(t); }
    T return_value() const volatile { return ret_val; }
  };

  template<typename T>
  struct par_tuple_return<T&> {
    T* ret_val;
    void set_return_value(T& t) { ret_val = &t; }
    void set_return_value(volatile T& t) volatile { ret_val = &t; }
    T& return_value() const { return *ret_val; }
    volatile T& return_value() const volatile { return *ret_val; }
  };

  template<>
  struct par_tuple_return<void> {};

  template<std::size_t I, typename T>
  class par_tuple_element {
    T value_{};

  public:
    constexpr par_tuple_element() = default;
    constexpr par_tuple_element(const T& t) : value_(t) {}
    constexpr par_tuple_element(T&& t) : value_(std::move(t)) {}

    constexpr T& get() & noexcept { return value_; }
    constexpr const T& get() const& noexcept { return value_; }
    constexpr T&& get() && { return std::move(value_); }
    constexpr const T&& get() const&& { return std::move(value_); }
    volatile T& get() volatile& noexcept { return value_; }
    const volatile T& get() const volatile& noexcept { return value_; }
    T&& get() volatile&& { return std::move(value_); }
    const T&& get() const volatile&& { return std::move(value_); }
    template<typename U>
    void set(U&& value) noexcept(std::is_nothrow_assignable_v<T, U&&>) {
      value_ = std::forward<U>(value);
    }
    template<typename... Us>
    void
    set(Us&&... values) noexcept(std::is_nothrow_constructible_v<T, Us&&...> and
                                 std::is_nothrow_move_assignable_v<T>) {
      value_ = T(std::forward<Us>(values)...);
    }
    template<typename U>
    void
    set(U&& value) volatile noexcept(std::is_nothrow_assignable_v<T, U&&>) {
      value_ = std::forward<U>(value);
    }
    template<typename... Us>
    void set(Us&&... values) volatile noexcept(
        std::is_nothrow_constructible_v<T, Us&&...> and
        std::is_nothrow_move_assignable_v<T>) {
      value_ = T(std::forward<Us>(values)...);
    }
  };

  template<std::size_t I, typename T>
  class par_tuple_element<I, T&> {
    T* value_{};

  public:
    constexpr par_tuple_element() = default;
    constexpr par_tuple_element(T& t) : value_(&t) {}
    constexpr T& get() & noexcept { return *value_; }
    constexpr const T& get() const& noexcept { return *value_; }
    constexpr T&& get() && { return std::move(*value_); }
    constexpr const T&& get() const&& { return std::move(*value_); }
    constexpr void set(T& value) noexcept { value_ = &value; }
    volatile T& get() volatile& noexcept { return *value_; }
    const volatile T& get() const volatile& noexcept { return *value_; }
    T&& get() volatile&& { return std::move(*value_); }
    const T&& get() const volatile&& { return std::move(*value_); }
    void set(volatile T& value) volatile noexcept { value_ = &value; }
  };

  template<std::size_t I, typename T>
  class par_tuple_element<I, const T&> : public par_tuple_element<I, T> {};

  template<std::size_t I, typename T>
  class par_tuple_element<I, T&&> : public par_tuple_element<I, T> {};

  template<typename IndexSeq, typename TypeList>
  class par_tuple_impl;

  template<template<typename...> typename List, std::size_t... Is,
           typename... Ts>
  class par_tuple_impl<std::index_sequence<Is...>, List<Ts...>>
      : private par_tuple_element<Is, Ts>... {
  public:
    using arguments = List<Ts...>;

    static constexpr std::size_t num_args = sizeof...(Ts);
    constexpr par_tuple_impl()            = default;
    template<typename... T>
    constexpr par_tuple_impl(T&&... ts)
        : par_tuple_element<Is, Ts>(std::forward<T>(ts))... {}

    /// access theI-th arg
    /// @{
    template<std::size_t I>
    constexpr auto& get() & {
      return static_cast<par_tuple_element<I, type_at_t<I, List<Ts...>>>*>(this)
          ->get();
    }

    template<std::size_t I>
    constexpr const auto& get() const& {
      return static_cast<
                 const par_tuple_element<I, type_at_t<I, List<Ts...>>>*>(this)
          ->get();
    }

    template<std::size_t I>
    constexpr auto&& get() && {
      return std::move(
          static_cast<par_tuple_element<I, type_at_t<I, List<Ts...>>&&>>(*this)
              .get());
    }

    template<std::size_t I>
    constexpr const auto&& get() const&& {
      return std::move(
          static_cast<const par_tuple_element<I, type_at_t<I, List<Ts...>>>&&>(
              *this)
              .get());
    }

    template<std::size_t I>
    volatile auto& get() volatile& {
      return static_cast<
                 volatile par_tuple_element<I, type_at_t<I, List<Ts...>>>*>(
                 this)
          ->get();
    }

    template<std::size_t I>
    const volatile auto& get() const volatile& {
      return static_cast<
                 const volatile par_tuple_element<I,
                                                  type_at_t<I, List<Ts...>>>*>(
                 this)
          ->get();
    }

    template<std::size_t I>
    volatile auto&& get() volatile&& {
      return std::move(
          static_cast<
              volatile par_tuple_element<I, type_at_t<I, List<Ts...>>>&&>(*this)
              .get());
    }

    template<std::size_t I>
    const volatile auto&& get() const volatile&& {
      return std::move(
          static_cast<
              const volatile par_tuple_element<I, type_at_t<I, List<Ts...>>>&&>(
              *this)
              .get());
    }
    /// @}

    /**
     * @brief set the I-th argument's value by assignment. Equivalent to
     * ``argument = std::forward<U>(value)``.
     *
     * @tparam I the argument index
     * @tparam U the values type
     * @param value the value to set
     * @{
     */
    template<std::size_t I, typename U>
      requires std::assignable_from<type_at_t<I, List<Ts...>>, U&&>
    constexpr void set(U&& value) noexcept(
        std::is_nothrow_assignable_v<type_at_t<I, List<Ts...>>, U&&>) {
      static_cast<par_tuple_element<I, type_at_t<I, List<Ts...>>*>>(this)->set(
          std::forward<U>(value));
    }

    template<std::size_t I, typename U>
      requires std::assignable_from<type_at_t<I, List<Ts...>>, U&&>
    void set(U&& value) volatile noexcept(
        std::is_nothrow_assignable_v<type_at_t<I, List<Ts...>>, U&&>) {
      static_cast<volatile par_tuple_element<I, type_at_t<I, List<Ts...>>*>>(
          this)
          ->set(std::forward<U>(value));
    }
    /// @}

    /**
     * @brief set the I-th argument's value by constructing a new value with
     * ctor_args and assigning it to the argument. Equivalent to
     * ``argument = T(std::forward<Us>(ctor_args)...)``, where T is the
     * arguments type.
     *
     * @tparam I the argument index
     * @tparam U the values type
     * @param ctor_args the values to construct the argument
     * @{
     */
    template<std::size_t I, typename... Us>
      requires std::is_constructible_v<type_at_t<I, List<Ts...>>, Us&&...>
    constexpr void set(Us&&... ctor_args) noexcept(
        std::is_nothrow_constructible_v<type_at_t<I, List<Ts...>>, Us&&...>) {
      static_cast<par_tuple_element<I, type_at_t<I, List<Ts...>>*>>(this)->set(
          std::forward<Us>(ctor_args)...);
    }

    template<std::size_t I, typename... Us>
      requires std::is_constructible_v<type_at_t<I, List<Ts...>>, Us&&...>
    void set(Us&&... ctor_args) volatile noexcept(
        std::is_nothrow_constructible_v<type_at_t<I, List<Ts...>>, Us&&...>) {
      static_cast<volatile par_tuple_element<I, type_at_t<I, List<Ts...>>*>>(
          this)
          ->set(std::forward<Us>(ctor_args)...);
    }
    /// @}
  };

  template<Accumulator A>
  struct accum_result {
    using type = A;
  };

  template<typename A>
    requires HasResult<A>
  struct accum_result<A> {
    using type = decltype(std::declval<const A>().result());
  };

  template<Accumulator A>
  using accum_result_t = typename accum_result<A>::type;

  /**
   * @brief An Accumulator of type T, which does not actually accumulate.
   * @tparam T the accumulators value type
   * @{
   */
  template<typename T>
  struct NullAccumulator {
    using value_type = T;
    constexpr void push_back(const T&) noexcept {}
    constexpr void push_back(T&) noexcept {}
    constexpr void push_back(T&&) noexcept {}
    constexpr void clear() noexcept {}
    void push_back(const T&) volatile noexcept {}
    void push_back(T&) volatile noexcept {}
    void push_back(T&&) volatile noexcept {}
    void clear() volatile noexcept {}
  };

  template<>
  struct NullAccumulator<void> {
    using value_type = void*;
    constexpr void clear() noexcept {}
    constexpr void push_back(void*) noexcept {}
    void push_back(void*) volatile noexcept {}
    void clear() volatile noexcept {}
  };
  /// @}
} // namespace

/**
 * A raw event handler is either a raw function pointer of type void(*)() or a
 * FixedEventHandler.
 */
template<typename H>
concept RawHandler = std::convertible_to<std::decay_t<H>, void (*)()>;

/**
 * A primary handler H is default constructible and invocable without arguments.
 * A primary handler can be used as an event handler in an EventSpec.
 */
template<typename H>
concept PrimaryHandler =
    std::constructible_from<H> and
    (FunctionOf<H, void()> or FunctionOf<H, void() volatile> or
     FunctionOf<H, void() noexcept> or
     FunctionOf<H, void() volatile noexcept>) and
    (not std::convertible_to<std::decay_t<H>, void (*)()>);

template<typename H, typename... Args>
concept HasSetUserHandler = requires(H&& handler, Args&&... args) {
  { handler.set_user_handler(std::forward<Args>(args)...) };
};

template<typename H, typename... Args>
concept HasEraseUserHandler = requires(H&& handler, Args&&... args) {
  { handler.remove_user_handler(std::forward<Args>(args)...) };
};

template<typename H, typename IndexList, typename... Args>
concept HasSetArg = has_set_arg_v<H, IndexList, TypeList<Args...>>;

template<typename H, typename... Args>
concept CanSetArgs = can_set_args<H, Args...>::value;

template<typename H>
concept HasGetReturnValue = requires(H&& handler) {
  { handler.get_return_value() };
};

/**
 * @brief An advanced handler
 *
 * @tparam H
 */
template<typename H>
concept AdvancedHandlerConcept =
    PrimaryHandler<H> and has_signature_typedef<H>::value /* and
                         HasSetUserHandler<H, Args...> or HasEraseUserHandler<H,
                         Args...> or HasSetArg<H, List<Indices...>,Args...>*/
    ;

/**
 * an event handler is either a function pointer of type 'void (*) ()' for raw
 * events, or a more complex object. An EventHandler generally consists of two
 * things:
 *  - a user handler: this is the actual callback to be invoked when the event
 * fires.
 *  - an argument store: stores the arguments which will be passed to the user
 * handler when the event fires.
 */
template<typename H>
concept EventHandler = RawHandler<H> or PrimaryHandler<H>;

template<typename... Ts>
class Tuple
    : public par_tuple_impl<decltype(std::make_index_sequence<sizeof...(Ts)>()),
                            TypeList<Ts...>> {
  using Base =
      par_tuple_impl<decltype(std::make_index_sequence<sizeof...(Ts)>()),
                     TypeList<Ts...>>;

public:
  constexpr Tuple(Ts&&... ts) : Base(std::forward<Ts>(ts)...) {}
  using Base::Base;
  using Base::operator=;
};

/// this class holds arguments passed and return values
template<typename Signature>
class FunctionParams
    : private par_tuple_return<
          typename signature_traits<Signature>::return_type>,
      private par_tuple_impl<
          decltype(std::make_index_sequence<list_size_v<
                       typename signature_traits<Signature>::arguments>>()),
          typename signature_traits<Signature>::arguments> {
public:
  using traits      = signature_traits<Signature>;
  using arguments   = typename traits::arguments;
  using return_type = typename signature_traits<Signature>::return_type;
  using Base        = par_tuple_impl<
             decltype(std::make_index_sequence<list_size_v<arguments>>()), arguments>;
  static constexpr bool returns_void    = std::same_as<return_type, void>;
  static constexpr std::size_t num_args = list_size_v<arguments>;

  constexpr FunctionParams() = default;

  using Base::get;

  /**
   * @brief set the I-th argument's value by assignment. Equivalent to
   * ``argument = std::forward<U>(value)``.
   *
   * @tparam I the argument index
   * @tparam U the values type
   * @param value the value to set
   * @{
   */
  template<std::size_t I, typename U>
    requires std::assignable_from<type_at_t<I, arguments>, U&&>
  constexpr void set_arg(U&& value) noexcept(
      std::is_nothrow_assignable_v<type_at_t<I, arguments>, U&&>) {
    Base::set(std::forward<U>(value));
  }

  template<std::size_t I, typename U>
    requires std::assignable_from<type_at_t<I, arguments>, U&&>
  void set_arg(U&& value) volatile noexcept(
      std::is_nothrow_assignable_v<type_at_t<I, arguments>, U&&>) {
    Base::set(std::forward<U>(value));
  }
  /// @}

  /**
   * @brief set the I-th argument's value by constructing a new value with
   * ctor_args and assigning it to the argument. Equivalent to
   * ``argument = T(std::forward<Us>(ctor_args)...)``, where T is the arguments
   * type.
   *
   * @tparam I the argument index
   * @tparam U the values type
   * @param ctor_args the values to construct the argument
   * @{
   */
  template<std::size_t I, typename... Us>
    requires std::is_constructible_v<type_at_t<I, arguments>, Us&&...>
  constexpr void set_arg(Us&&... ctor_args) noexcept(
      std::is_nothrow_constructible_v<type_at_t<I, arguments>, Us&&...>) {
    Base::set(std::forward<Us>(ctor_args)...);
  }

  template<std::size_t I, typename... Us>
    requires std::is_constructible_v<type_at_t<I, arguments>, Us&&...>
  void set_arg(Us&&... ctor_args) volatile noexcept(
      std::is_nothrow_constructible_v<type_at_t<I, arguments>, Us&&...>) {
    Base::set(std::forward<Us>(ctor_args)...);
  }
  /// @}

  /**
   * @brief set the return value. Only available if the return type is not void.
   *
   * @tparam U a type assignable to the return value
   * @param value the value to set
   * @{
   */
  template<typename U>
    requires(not std::same_as<return_type, void>) and
            std::assignable_from<return_type, U&&>
  constexpr void set_return_value(U&& value) noexcept(
      std::is_nothrow_assignable_v<return_type, U&&>) {
    static_cast<RStore&>(*this).set_return_value(std::forward<U>(value));
  }

  template<typename U>
    requires(not std::same_as<return_type, void>) and
            std::assignable_from<return_type, U&&>
  constexpr void set_return_value(U&& value) volatile noexcept(
      std::is_nothrow_assignable_v<return_type, U&&>) {
    static_cast<volatile RStore&>(*this).set_return_value(
        std::forward<U>(value));
  }
  /// @}

  /**
   * get the return value.
   * @{
   */
  return_type get_return_value() const
    requires(not std::same_as<return_type, void>)
  {
    return this->return_value();
  }

  return_type get_return_value() volatile const
    requires(not std::same_as<return_type, void>)
  {
    return this->return_value();
  }
  /// @}
private:
  using RStore = par_tuple_return<return_type>;
};

/**
 * @brief An event handler for a single UserHandler.
 *
 * It stores a single user handler of type UserHandler, its arguments and return
 * value, and satisfies the AdvancedHandlerConcept.
 *
 * Either use as is, or inherit from EventHandler to layer more behaviour on top
 * of it.
 *
 * @tparam SingleShot if true, the user handler will be moved onto the stack
 * before calling it, else the user handler is invoked inplace.
 * @tparam UserHandler the type of the user handler
 * @return
 */
template<bool SingleShot, UserHandler UserHandler>
class BasicEventHandler
    : private par_tuple_return<
          typename function_traits<UserHandler>::return_type>,
      private par_tuple_impl<
          decltype(std::make_index_sequence<list_size_v<
                       typename function_traits<UserHandler>::arguments>>()),
          typename function_traits<UserHandler>::arguments> {

  static constexpr bool returns_void =
      std::same_as<typename function_traits<UserHandler>::return_type, void>;
  static constexpr std::size_t num_args =
      list_size_v<typename function_traits<UserHandler>::arguments>;

  // converts logical argument index to its position in the bitset
  static constexpr std::size_t bit_idx(std::size_t i) noexcept {
    return returns_void ? i : i + 1;
  }

  // the return type storage
  using RStore =
      par_tuple_return<typename function_traits<UserHandler>::return_type>;
  // the argument tuple.
  using Tup = par_tuple_impl<
      decltype(std::make_index_sequence<list_size_v<
                   typename function_traits<UserHandler>::arguments>>()),
      typename function_traits<UserHandler>::arguments>;

  template<std::size_t... Is>
  constexpr void
  apply(std::index_sequence<Is...>) noexcept(traits::is_noexcept) {
    if constexpr (returns_void) {
      if constexpr (SingleShot)
        std::invoke(std::move(user_handler), get<Is>()...);
      else
        std::invoke(user_handler, get<Is>()...);
    } else {
      if constexpr (SingleShot)
        set_return_value(std::invoke(std::move(user_handler), get<Is>()...));
      else
        set_return_value(std::invoke(user_handler, get<Is>()...));
    }
  }
  template<std::size_t... Is>
  void
  apply(std::index_sequence<Is...>) volatile noexcept(traits::is_noexcept) {
    if constexpr (returns_void) {
      if constexpr (SingleShot)
        std::invoke(std::move(user_handler), get<Is>()...);
      else
        std::invoke(user_handler, get<Is>()...);
    } else {
      if constexpr (SingleShot)
        set_return_value(std::invoke(std::move(user_handler), get<Is>()...));
      else
        set_return_value(std::invoke(user_handler, get<Is>()...));
    }
  }

  /// access the I-th arg
  /// @{
  template<std::size_t I>
    requires(I < num_args)
  constexpr auto& get() & {
    CM_ASSERT_MESSAGE(param_set_bits_.test(bit_idx(I)),
                      "Attempted to get argument that has not been set yet");
    param_set_bits_.reset(bit_idx(I));
    return static_cast<Tup*>(this)->template get<I>();
  }

  template<std::size_t I>
    requires(I < num_args)
  constexpr const auto& get() const& {
    CM_ASSERT_MESSAGE(param_set_bits_.test(bit_idx(I)),
                      "Attempted to get argument that has not been set yet");
    param_set_bits_.reset(bit_idx(I));
    return static_cast<const Tup*>(this)->template get<I>();
  }

  template<std::size_t I>
    requires(I < num_args)
  constexpr auto&& get() && {
    CM_ASSERT_MESSAGE(param_set_bits_.test(bit_idx(I)),
                      "Attempted to get argument that has not been set yet");
    param_set_bits_.reset(bit_idx(I));
    return std::move(static_cast<Tup&&>(*this)->template get<I>());
  }

  template<std::size_t I>
    requires(I < num_args)
  constexpr const auto&& get() const&& {
    CM_ASSERT_MESSAGE(param_set_bits_.test(bit_idx(I)),
                      "Attempted to get argument that has not been set yet");
    param_set_bits_.reset(bit_idx(I));
    return std::move(static_cast<const Tup&&>(*this)->template get<I>());
  }

  template<std::size_t I>
    requires(I < num_args)
  volatile auto& get() volatile& {
    CM_ASSERT_MESSAGE(param_set_bits_.test(bit_idx(I)),
                      "Attempted to get argument that has not been set yet");
    param_set_bits_.reset(bit_idx(I));
    return static_cast<volatile Tup*>(this)->template get<I>();
  }

  template<std::size_t I>
    requires(I < num_args)
  volatile const auto& get() volatile const& {
    CM_ASSERT_MESSAGE(param_set_bits_.test(bit_idx(I)),
                      "Attempted to get argument that has not been set yet");
    param_set_bits_.reset(bit_idx(I));
    return static_cast<volatile const Tup*>(this)->template get<I>();
  }

  template<std::size_t I>
    requires(I < num_args)
  auto&& get() volatile&& {
    CM_ASSERT_MESSAGE(param_set_bits_.test(bit_idx(I)),
                      "Attempted to get argument that has not been set yet");
    param_set_bits_.reset(bit_idx(I));
    return std::move(static_cast<volatile Tup&&>(*this)->template get<I>());
  }

  template<std::size_t I>
    requires(I < num_args)
  const auto&& get() volatile const&& {
    CM_ASSERT_MESSAGE(param_set_bits_.test(bit_idx(I)),
                      "Attempted to get argument that has not been set yet");
    param_set_bits_.reset(bit_idx(I));
    return std::move(
        static_cast<volatile const Tup&&>(*this)->template get<I>());
  }
  /// @}

public:
  using traits      = function_traits<UserHandler>;
  using signature   = typename traits::signature_type;
  using arguments   = typename traits::arguments;
  using return_type = typename traits::return_type;

  constexpr BasicEventHandler() = default;

  void operator()() noexcept(traits::is_noexcept) {
    apply(std::make_index_sequence<num_args>());
  }

  /**
   * @brief set the I-th argument's value by assignment. Equivalent to
   * ``argument = std::forward<U>(value)``.
   *
   * @tparam I the argument index
   * @tparam U the values type
   * @param value the value to set
   * @{
   */
  template<std::size_t I, typename U>
    requires std::assignable_from<type_at_t<I, arguments>, U&&> and
             (I < num_args)
  constexpr void set_arg(U&& value) noexcept(
      std::is_nothrow_assignable_v<type_at_t<I, arguments>, U&&>) {
    CM_ASSERT_MESSAGE(not param_set_bits_.test(bit_idx(I)),
                      "Attempted to set argument that has been set");
    static_cast<Tup&>(*this).template set_arg<I>(std::forward<U>(value));
    param_set_bits_.set(bit_idx(I));
  }

  template<std::size_t I, typename U>
    requires std::assignable_from<type_at_t<I, arguments>, U&&> and
             (I < num_args)
  constexpr void set_arg(U&& value) volatile noexcept(
      std::is_nothrow_assignable_v<type_at_t<I, arguments>, U&&>) {
    CM_ASSERT_MESSAGE(not param_set_bits_.test(bit_idx(I)),
                      "Attempted to set argument that has been set");
    static_cast<Tup&>(*this).template set_arg<I>(std::forward<U>(value));
    param_set_bits_.set(bit_idx(I));
  }
  /// @}

  /**
   * @brief set the I-th argument's value by constructing a new value with
   * ctor_args and assigning it to the argument. Equivalent to
   * ``argument = T(std::forward<Us>(ctor_args)...)``, where T is the arguments
   * type.
   *
   * @tparam I the argument index
   * @tparam U the values type
   * @param ctor_args the values to construct the argument
   * @{
   */
  template<std::size_t I, typename... Us>
    requires std::is_constructible_v<type_at_t<I, arguments>, Us&&...> and
             (I < num_args)
  constexpr void set_arg(Us&&... ctor_args) noexcept(
      std::is_nothrow_constructible_v<type_at_t<I, arguments>, Us&&...>) {
    CM_ASSERT_MESSAGE(not param_set_bits_.test(bit_idx(I)),
                      "Attempted to set argument that has been set");
    static_cast<Tup*>(this)->template set_arg<I>(
        std::forward<Us>(ctor_args)...);
    param_set_bits_.set(bit_idx(I));
  }

  template<std::size_t I, typename... Us>
    requires std::is_constructible_v<type_at_t<I, arguments>, Us&&...> and
             (I < num_args)
  void set_arg(Us&&... ctor_args) volatile noexcept(
      std::is_nothrow_constructible_v<type_at_t<I, arguments>, Us&&...>) {
    CM_ASSERT_MESSAGE(not param_set_bits_.test(bit_idx(I)),
                      "Attempted to set argument that has been set");
    static_cast<volatile Tup*>(this)->template set_arg<I>(
        std::forward<Us>(ctor_args)...);
    param_set_bits_.set(bit_idx(I));
  }
  /// @}

  /**
   * @brief set the return value. Only available if the return type is not void.
   *
   * @tparam U a type assignable to return_type
   * @param value the value to set
   * @{
   */
  template<typename U>
    requires(not std::same_as<return_type, void>) and
            std::assignable_from<return_type, U&&>
  constexpr void set_return_value(U&& value) noexcept(
      std::is_nothrow_assignable_v<return_type, U&&>) {
    CM_ASSERT_MESSAGE(not param_set_bits_.test(0),
                      "Attempted to set return value that has been set");
    static_cast<RStore&>(*this).set_return_value(std::forward<U>(value));
    param_set_bits_.set(0);
  }

  template<typename U>
    requires(not std::same_as<return_type, void>) and
            std::assignable_from<return_type, U&&>
  void set_return_value(U&& value) volatile noexcept(
      std::is_nothrow_assignable_v<return_type, U&&>) {
    CM_ASSERT_MESSAGE(not param_set_bits_.test(0),
                      "Attempted to set return value that has been set");
    static_cast<RStore&>(*this).set_return_value(std::forward<U>(value));
    param_set_bits_.set(0);
  }
  /// @}

  /**
   * @brief returns the return value
   * @{
   */
  return_type get_return_value() const
    requires(not std::same_as<return_type, void>)
  {
    CM_ASSERT_MESSAGE(param_set_bits_.test(0),
                      "Attempted to retrieve return value that does not exist");
    param_set_bits_.reset(0);
    return this->return_value();
  }

  return_type get_return_value() volatile const
    requires(not std::same_as<return_type, void>)
  {
    CM_ASSERT_MESSAGE(param_set_bits_.test(0),
                      "Attempted to retrieve return value that does not exist");
    param_set_bits_.reset(0);
    return this->return_value();
  }
  /// @}

private:
  UserHandler user_handler{};
  std::bitset<num_args + (returns_void ? 1 : 0)> param_set_bits_{};
};

/**
 * @brief An Accumulator of type T, which does not actually accumulate.
 * @tparam T the accumulators value type
 * @{
 */
template<typename UserHandlerList>
struct DefaultAccumulate
    : NullAccumulator<typename function_traits<
          typename UserHandlerList::value_type>::return_type> {};

/**
 * @brief An event handler for a list of user handlers.
 *
 * The BroadcastEventHandler stores a sequence of user handlers in a
 * UserHandlerList and the argument set for them, and accumulates the
 * user handlers return values in an Accumulator.
 *
 * @tparam SingleShot if true, the user handler list is cleared everytime the
 * BroadcastEventHandler is called.
 * @tparam UserHandlerList a type conforming to the UserHandlerList concept
 * @tparam ResultAccumulator a type conforming to the Accumulator concept
 */
template<bool SingleShot, UserHandlerList UserHandlerList,
         Accumulator ResultAccumulator = DefaultAccumulate<UserHandlerList>>
class BroadcastEventHandler
    : private ResultAccumulator,
      private par_tuple_impl<
          decltype(std::make_index_sequence<
                   list_size_v<typename function_traits<
                       typename UserHandlerList::value_type>::arguments>>()),
          typename function_traits<
              typename UserHandlerList::value_type>::arguments> {

  UserHandlerList user_handlers{};

  // converts logical argument index to its position in the bitset
  static constexpr std::size_t bit_idx(std::size_t i) noexcept {
    return returns_void ? i : i + 1;
  }

  // the argument tuple.
  using Tup = par_tuple_impl<
      decltype(std::make_index_sequence<list_size_v<typename function_traits<
                   typename UserHandlerList::value_type>::arguments>>()),
      typename function_traits<
          typename UserHandlerList::value_type>::arguments>;
  template<std::size_t... Is>
  void apply(std::index_sequence<Is...>) noexcept(traits::is_noexcept) {
    static_cast<volatile ResultAccumulator*>(this)
        ->clear(); // clear the accumulator
    for (auto& uh : user_handlers) {
      if (!static_cast<bool>(uh))
        continue;
      if constexpr (returns_void) {
        if constexpr (SingleShot)
          std::invoke(std::move(uh),
                      static_cast<volatile Tup&>(*this).template get<Is>()...);
        else
          std::invoke(uh,
                      static_cast<volatile Tup&>(*this).template get<Is>()...);
      } else {
        if constexpr (SingleShot)
          static_cast<volatile ResultAccumulator*>(this)->push_back(std::invoke(
              std::move(uh),
              static_cast<volatile Tup&>(*this).template get<Is>()...));
        else
          static_cast<volatile ResultAccumulator*>(this)->push_back(std::invoke(
              uh,
              static_cast<volatile Tup&>(*this).template get<Is>()...));
      }
    }
    param_set_bits_.reset();
    if constexpr (not returns_void)
      param_set_bits_.set(0);
    if constexpr (SingleShot)
      user_handlers.clear();
  }

public:
  using traits      = function_traits<typename UserHandlerList::value_type>;
  using signature   = typename traits::signature_type;
  using arguments   = typename traits::arguments;
  using return_type = typename traits::return_type;
  using result_type = accum_result_t<ResultAccumulator>;
  using accumulator_type                = ResultAccumulator;
  static constexpr bool returns_void    = std::same_as<return_type, void>;
  static constexpr std::size_t num_args = list_size_v<arguments>;

  constexpr BroadcastEventHandler() = default;

  /**
   * @brief set the I-th argument's value by assignment. Equivalent to
   * ``argument = std::forward<U>(value)``.
   *
   * @tparam I the argument index
   * @tparam U the values type
   * @param value the value to set
   */
  template<std::size_t I, typename U>
    requires std::assignable_from<type_at_t<I, arguments>, U&&> and
             (I < num_args)
  constexpr void set_arg(U&& value) noexcept(
      std::is_nothrow_assignable_v<type_at_t<I, arguments>, U&&>) {
    CM_ASSERT_MESSAGE(not param_set_bits_.test(bit_idx(I)),
                      "Attempted to set argument that has been set");
    static_cast<Tup&>(*this).template set_arg<I>(std::forward<U>(value));
    param_set_bits_.set(bit_idx(I));
  }

  /**
   * @brief set the I-th argument's value by constructing a new value with
   * ctor_args and assigning it to the argument. Equivalent to
   * ``argument = T(std::forward<Us>(ctor_args)...)``, where T is the arguments
   * type.
   *
   * @tparam I the argument index
   * @tparam U the values type
   * @param ctor_args the values to construct the argument
   */
  template<std::size_t I, typename... Us>
    requires std::is_constructible_v<type_at_t<I, arguments>, Us&&...> and
             (I < num_args)
  constexpr void set_arg(Us&&... ctor_args) noexcept(
      std::is_nothrow_constructible_v<type_at_t<I, arguments>, Us&&...>) {
    CM_ASSERT_MESSAGE(not param_set_bits_.test(bit_idx(I)),
                      "Attempted to set argument that has been set");
    static_cast<Tup*>(this)->template set_arg<I>(
        std::forward<Us>(ctor_args)...);
    param_set_bits_.set(bit_idx(I));
  }

  /**
   * @brief get the accumulators result, if it implements result(), or the
   * accumulator itself.
   *
   * @return
   * @{
   */
  result_type get_return_value() const
    requires(not std::same_as<return_type, void>)
  {
    CM_ASSERT_MESSAGE(param_set_bits_.test(0),
                      "Attempted to retrieve return value that does not exist");
    param_set_bits_.reset(0);
    return static_cast<const ResultAccumulator*>(this)->get_return_value();
  }

  result_type get_return_value() volatile const
    requires(not std::same_as<return_type, void>)
  {
    CM_ASSERT_MESSAGE(param_set_bits_.test(0),
                      "Attempted to retrieve return value that does not exist");
    param_set_bits_.reset(0);
    return static_cast<volatile const ResultAccumulator*>(this)
        ->get_return_value();
  }
  /// @}

  /**
   * @brief adds a userhandler to the user handler list
   *
   * @tparam UserHandler
   * @param f
   */
  template<UserHandler UH>
    requires PushBackAble<UserHandlerList, UH>
  constexpr void set_user_handler(UH&& f) {
    user_handlers.push_back(std::forward<UH>(f));
  }

  /**
   * @brief the call operator used by the EventDispatcher.
   */
  void operator()() noexcept(traits::is_noexcept) {
    apply(std::make_index_sequence<num_args>());
  }

private:
  static constexpr std::size_t bit_set_size =
      ((num_args == 0) and returns_void) ? 1
                                         : num_args + (returns_void ? 1 : 0);
  std::bitset<bit_set_size> param_set_bits_{};
};

/**
 * A type erased event handler.
 *
 * @tparam SingleShot If true, the user handler will be moved onto the stack
 * before it is invoked. If false, the user handler is invoked in place.
 *
 * @tparam BufferSize The size of the storage used to store user handlers. User
 * handlers with a greater size cannot be stored by this handler type.
 *
 * @tparam BufferAlignment The alignment of the storage used for user handlers.
 * User handlers with greater alignment cannot be stored by this handler type.
 *
 * @tparam Args arguments the user handler is invoked with.
 */
template<bool Singleshot, typename Signature, std::size_t BufferSize,
         std::size_t BufferAlignment, typename... Args>
class AnyEventHandler
    : public BasicEventHandler<
          Singleshot,
          MoveOnlyFunction<Signature, BufferSize, BufferAlignment>> {
public:
  constexpr AnyEventHandler() = default;
};

// TODO
template<std::convertible_to<void (*)()> auto f>
struct FixedEventHandler {
  using Cb = void (*)();
  constexpr operator Cb() const noexcept { return f; }
  constexpr void operator()() const noexcept { f(); }
};

namespace impl {
  template<typename T>
  inline constexpr bool is_fixed_raw_handler_v = false;

  template<typename T>
  struct is_fixed_raw_handler : std::false_type {};

  template<typename T>
    requires std::same_as<T, void (*)()>
  struct is_fixed_raw_handler<T> : std::true_type {};

  template<std::convertible_to<void (*)()> auto f>
  inline constexpr bool is_fixed_raw_handler_v<FixedEventHandler<f>> = true;

} // namespace impl

/**
 * An EventSpec describes an event completely.
 *
 * @tparam Id the events id.
 * @tparam Priority the priority of the event.
 * @tparam Handler This can either be 'void(*)()' (for raw events), or any of
 * the discussed **event handlers** above (not **user handlers**!).
 */
template<IRQn Id, Priority P, EventHandler Handler>
struct EvSpec {
  static constexpr IRQn id           = Id;
  static constexpr Priority priority = P;
  using handler_type                 = Handler;
};

template<IRQn Id, Priority Priority>
using RawEvent = EvSpec<Id, Priority, void (*)()>;

template<IRQn Id, Priority Priority, auto f>
using FixedRawEvent = EvSpec<Id, Priority, FixedEventHandler<f>>;

template<IRQn Id, Priority Priority, typename F>
using Event = EvSpec<Id, Priority, BasicEventHandler<true, F>>;

template<IRQn Id, Priority Priority, typename F>
using RecurringEvent = EvSpec<Id, Priority, BasicEventHandler<false, F>>;

template<IRQn Id, Priority Priority, typename Signature, std::size_t Size,
         std::size_t Align>
using AnyEvent =
    EvSpec<Id, Priority,
           BasicEventHandler<true, MoveOnlyFunction<Signature, Size, Align>>>;

template<IRQn Id, Priority Priority, typename Signature, std::size_t Size,
         std::size_t Align>
using AnyRecurringEvent =
    EvSpec<Id, Priority,
           BasicEventHandler<false, MoveOnlyFunction<Signature, Size, Align>>>;

template<typename Event>
struct event_handler_type {
  using type = std::conditional_t<
      std::is_convertible_v<typename Event::handler_type, void (*)()>,
      void (*)(), typename Event::handler_type>;
};

template<typename Event>
using event_handler_type_t = typename event_handler_type<Event>::type;

template<typename Event>
concept EventSpec = EventHandler<typename Event::handler_type>and
  requires()
{
  { Event::id } -> std::convertible_to<IRQn>;
  { Event::priority } -> std::convertible_to<Priority>;
};

namespace impl {

  template<EventSpec Event>
  struct is_raw_event
      : std::is_convertible<typename Event::handler_type, void (*)()> {};

  template<EventSpec Event>
  inline constexpr bool is_raw_event_v = is_raw_event<Event>::value;

  template<EventSpec Event>
  struct is_fixed_event : std::is_const<Event> {};

  template<EventSpec Event>
    requires is_fixed_raw_handler_v<typename Event::handler_type>
  struct is_fixed_event<Event> : std::true_type {};

  template<EventSpec Event>
  struct is_fixed_raw_event
      : std::conjunction<is_raw_event<Event>, is_fixed_event<Event>> {};

  template<EventSpec Event>
  inline constexpr bool is_fixed_event_v = is_fixed_event<Event>::value;

  template<EventSpec Event>
  struct is_advanced_event : std::false_type {};

  template<EventSpec Event>
    requires PrimaryHandler<typename Event::handler_type>
  struct is_advanced_event<Event> : std::true_type {};

  template<EventSpec Event>
  inline constexpr bool is_advanced_event_v = is_advanced_event<Event>::value;

  template<typename Handler>
  struct handler_reference_type {
    // TODO: maybe add volatile here
    using type = std::add_lvalue_reference_t<Handler>;
  };

  template<typename H>
    requires std::convertible_to<H, void (*)()>
  struct handler_reference_type<H> {
    using type = volatile IrqHandler&;
  };

  template<typename H>
    requires is_fixed_raw_handler_v<H>
  struct handler_reference_type<H> {
    using type = const volatile IrqHandler&;
  };

} // namespace impl

template<typename Event>
concept FixedEvent = EventSpec<Event> and impl::is_fixed_event_v<Event>;

template<typename Event>
concept MutableEvent = not FixedEvent<Event>;

template<EventSpec Event>
using handler_reference_t =
    typename impl::handler_reference_type<typename Event::handler_type>::type;

template<EventSpec Source, EventSpec Dest>
inline constexpr std::size_t connect_index =
    index_of_v<typename signature_traits<typename event_handler_type_t<
                   Source>::signature>::return_type,
               typename signature_traits<
                   typename event_handler_type_t<Dest>::signature>::arguments>;

template<EventSpec Source, EventSpec Dest,
         std::size_t Index = connect_index<Source, Dest>>
struct connect {};

} // namespace cm
#endif
