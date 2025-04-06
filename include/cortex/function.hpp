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
#ifndef CM_FUNCTION_HPP
#define CM_FUNCTION_HPP
#include "cortex/common.hpp"
#include "cortex/type_list.hpp"
#include <concepts>
#include <cstddef>
#include <functional>
#include <iterator>
#include <memory>
#include <new>
#include <type_traits>
#include <utility>

namespace cm {

/**
 * extracts the signature from a callable F, respecting const, volatile and
 * noexcept qualifications. The extracted signature can be accessed by its inner
 * typedef ``type``.
 *
 * F is either
 * - a signature type, i.e. ``Ret(Args...)[volatile][const][noexcept]``,
 * - a function pointer type, i.e. ``Ret(*)(Args...)[noexcept]``,
 * - a member function pointer type, i.e.
 * ``Ret(T::*)(Args...)[volatile][const][noexcept]``,
 * - or types with an unambigous call operator, i.e. a special case of the
 * member function pointer.
 *
 * for signature types, the signature itself is returned.
 * for function pointer types, the signature is formed by removing the pointer
 * for member function pointers and callables
 * @{
 */
template<typename F, typename = void>
struct extract_signature;
template<typename R, typename... A>
struct extract_signature<R(A...)> {
  using type = R(A...);
};
template<typename R, typename... A>
struct extract_signature<R(A...) noexcept> {
  using type = R(A...) noexcept;
};
template<typename R, typename... A>
struct extract_signature<R(A...) const> {
  using type = R(A...) const;
};
template<typename R, typename... A>
struct extract_signature<R(A...) const noexcept> {
  using type = R(A...) const noexcept;
};
template<typename R, typename... A>
struct extract_signature<R (*)(A...)> {
  using type = R(A...);
};
template<typename R, typename... A>
struct extract_signature<R (*)(A...) noexcept> {
  using type = R(A...) noexcept;
};
template<typename T, typename R, typename... A>
struct extract_signature<R (T::*)(A...)> {
  using type = R(A...);
};
template<typename T, typename R, typename... A>
struct extract_signature<R (T::*)(A...) noexcept> {
  using type = R(A...) noexcept;
};
template<typename T, typename R, typename... A>
struct extract_signature<R (T::*)(A...) const> {
  using type = R(A...) const;
};
template<typename T, typename R, typename... A>
struct extract_signature<R (T::*)(A...) const noexcept> {
  using type = R(A...) const noexcept;
};
template<typename R, typename... A>
struct extract_signature<R(A...) volatile> {
  using type = R(A...) volatile;
};
template<typename R, typename... A>
struct extract_signature<R(A...) volatile noexcept> {
  using type = R(A...) volatile noexcept;
};
template<typename R, typename... A>
struct extract_signature<R(A...) volatile const> {
  using type = R(A...) volatile const;
};
template<typename R, typename... A>
struct extract_signature<R(A...) volatile const noexcept> {
  using type = R(A...) volatile const noexcept;
};
template<typename T, typename R, typename... A>
struct extract_signature<R (T::*)(A...) volatile> {
  using type = R(A...) volatile;
};
template<typename T, typename R, typename... A>
struct extract_signature<R (T::*)(A...) volatile noexcept> {
  using type = R(A...) volatile noexcept;
};
template<typename T, typename R, typename... A>
struct extract_signature<R (T::*)(A...) volatile const> {
  using type = R(A...) volatile const;
};
template<typename T, typename R, typename... A>
struct extract_signature<R (T::*)(A...) volatile const noexcept> {
  using type = R(A...) volatile const noexcept;
};
template<typename F>
struct extract_signature<F, std::void_t<decltype(&F::operator())>> {
  using type = typename extract_signature<decltype(&F::operator())>::type;
};

template<typename F>
using extract_signature_t = typename extract_signature<F>::type;
/// @}

template<typename Signature>
struct signature_traits;

template<typename R, class... A>
struct signature_traits<R(A...)> {
  using signature_type              = R(A...);
  using ptr_type                    = R (*)(A...);
  using invoke_ptr_type             = R (*)(void*, A...);
  using invoke_obj_type             = void*;
  using mutable_obj_type            = void*;
  using return_type                 = R;
  using arguments                   = TypeList<A...>;
  static constexpr bool is_noexcept = false;
  static constexpr bool is_const    = false;
  static constexpr bool is_volatile = false;
  template<typename F>
  static constexpr bool matches = std::is_invocable_r_v<R, F, A...>;
};
template<typename R, class... A>
struct signature_traits<R(A...) noexcept> {
  using signature_type              = R(A...) noexcept;
  using ptr_type                    = R (*)(A...) noexcept;
  using invoke_ptr_type             = R (*)(void*, A...) noexcept;
  using invoke_obj_type             = void*;
  using mutable_obj_type            = void*;
  using return_type                 = R;
  using arguments                   = TypeList<A...>;
  static constexpr bool is_noexcept = true;
  static constexpr bool is_const    = false;
  static constexpr bool is_volatile = false;
  template<typename F>
  static constexpr bool matches = std::is_nothrow_invocable_r_v<R, F, A...>;
};
template<typename R, class... A>
struct signature_traits<R(A...) const> {
  using signature_type              = R(A...) const;
  using ptr_type                    = R (*)(A...);
  using invoke_ptr_type             = R (*)(const void*, A...);
  using invoke_obj_type             = const void*;
  using mutable_obj_type            = void*;
  using return_type                 = R;
  using arguments                   = TypeList<A...>;
  static constexpr bool is_noexcept = false;
  static constexpr bool is_const    = true;
  static constexpr bool is_volatile = false;
  template<typename F>
  static constexpr bool matches = std::is_invocable_r_v<R, const F, A...>;
};
template<typename R, class... A>
struct signature_traits<R(A...) const noexcept> {
  using signature_type              = R(A...) const noexcept;
  using ptr_type                    = R (*)(A...) noexcept;
  using invoke_ptr_type             = R (*)(const void*, A...) noexcept;
  using invoke_obj_type             = const void*;
  using mutable_obj_type            = void*;
  using return_type                 = R;
  using arguments                   = TypeList<A...>;
  static constexpr bool is_noexcept = true;
  static constexpr bool is_const    = true;
  static constexpr bool is_volatile = false;
  template<typename F>
  static constexpr bool matches =
      std::is_nothrow_invocable_r_v<R, const F, A...>;
};

template<typename R, class... A>
struct signature_traits<R(A...) volatile> {
  using signature_type              = R(A...) volatile;
  using ptr_type                    = R (*)(A...);
  using invoke_ptr_type             = R (*)(volatile void*, A...);
  using invoke_obj_type             = volatile void*;
  using mutable_obj_type            = volatile void*;
  using return_type                 = R;
  using arguments                   = TypeList<A...>;
  static constexpr bool is_noexcept = false;
  static constexpr bool is_const    = false;
  static constexpr bool is_volatile = true;
  template<typename F>
  static constexpr bool matches = std::is_invocable_r_v<R, volatile F, A...>;
};
template<typename R, class... A>
struct signature_traits<R(A...) volatile noexcept> {
  using signature_type              = R(A...) volatile noexcept;
  using ptr_type                    = R (*)(A...) noexcept;
  using invoke_ptr_type             = R (*)(volatile void*, A...) noexcept;
  using invoke_obj_type             = volatile void*;
  using mutable_obj_type            = volatile void*;
  using return_type                 = R;
  using arguments                   = TypeList<A...>;
  static constexpr bool is_noexcept = true;
  static constexpr bool is_const    = false;
  static constexpr bool is_volatile = true;
  template<typename F>
  static constexpr bool matches =
      std::is_nothrow_invocable_r_v<R, volatile F, A...>;
};
template<typename R, class... A>
struct signature_traits<R(A...) volatile const> {
  using signature_type              = R(A...) const;
  using ptr_type                    = R (*)(A...);
  using invoke_ptr_type             = R (*)(volatile const void*, A...);
  using invoke_obj_type             = volatile const void*;
  using mutable_obj_type            = volatile void*;
  using return_type                 = R;
  using arguments                   = TypeList<A...>;
  static constexpr bool is_noexcept = false;
  static constexpr bool is_const    = true;
  static constexpr bool is_volatile = true;
  template<typename F>
  static constexpr bool matches =
      std::is_invocable_r_v<R, volatile const F, A...>;
};
template<typename R, class... A>
struct signature_traits<R(A...) volatile const noexcept> {
  using signature_type   = R(A...) const noexcept;
  using ptr_type         = R (*)(A...) noexcept;
  using invoke_ptr_type  = R (*)(volatile const void*, A...) noexcept;
  using invoke_obj_type  = volatile const void*;
  using mutable_obj_type = volatile void*;
  using return_type      = R;
  using arguments        = TypeList<A...>;
  static constexpr bool is_noexcept = true;
  static constexpr bool is_const    = true;
  static constexpr bool is_volatile = true;
  template<typename F>
  static constexpr bool matches =
      std::is_nothrow_invocable_r_v<R, volatile const F, A...>;
};

template<typename F>
struct function_traits : signature_traits<extract_signature_t<F>> {};

template<typename F>
concept FunctionConcept = requires() {
  { typename function_traits<F>::ptr_type{} };
};

template<typename F, typename Signature>
concept FunctionOf =
    std::same_as<typename function_traits<F>::signature_type, Signature>;

namespace {

  template<typename Sig>
  struct VTable {
    using traits  = signature_traits<Sig>;
    using ObjPtr  = typename traits::mutable_obj_type;
    using Move    = void (*)(ObjPtr dest, ObjPtr src);
    using Dtor    = void (*)(ObjPtr);
    using Invoke  = typename traits::invoke_ptr_type;
    Move move     = nullptr;
    Dtor dtor     = nullptr;
    Invoke invoke = nullptr;
  };

  template<typename T, typename Sig>
  struct move_for {
    using traits  = signature_traits<Sig>;
    using ObjPtr  = typename traits::mutable_obj_type;
    using PtrType = std::conditional_t<traits::is_volatile, volatile T*, T*>;
    using Move    = void (*)(ObjPtr dest, ObjPtr src);

    constexpr Move get() noexcept {
      return [](ObjPtr dest,
                ObjPtr src) noexcept(std::is_nothrow_move_constructible_v<T>) {
        std::construct_at(*static_cast<PtrType>(dest),
                          *static_cast<PtrType>(src));
      };
    }
  };

  template<typename T, typename Sig>
  struct dtor_for {
    using traits  = signature_traits<Sig>;
    using ObjPtr  = typename traits::mutable_obj_type;
    using PtrType = std::conditional_t<traits::is_volatile, volatile T*, T*>;
    using Dtor    = void (*)(ObjPtr);

    constexpr Dtor get() noexcept {
      if constexpr (std::is_trivially_destructible_v<T>)
        return [](ObjPtr) noexcept {};
      else
        return [](ObjPtr buffer) noexcept(std::is_nothrow_destructible_v<T>) {
          std::destroy_at(static_cast<PtrType>(buffer));
        };
    }
  };

  template<typename T, typename Sig>
  struct invoke_for;

  template<typename T, typename Ret, typename... Args>
  struct invoke_for<T, Ret(Args...)> {
    using Invoke = Ret (*)(void*, Args...);

    static constexpr Invoke get() noexcept {
      return [](void* f, Args... args) {
        std::invoke(*std::launder(static_cast<T*>(f)),
                    std::forward<Args>(args)...);
      };
    }
  };

  template<typename T, typename Ret, typename... Args>
  struct invoke_for<T, Ret(Args...) const> {
    using Invoke = Ret (*)(const void*, Args...);
    static constexpr Invoke get() noexcept {
      return [](const void* f, Args... args) {
        std::invoke(*std::launder(static_cast<const T*>(f)),
                    std::forward<Args>(args)...);
      };
    }
  };

  template<typename T, typename Ret, typename... Args>
  struct invoke_for<T, Ret(Args...) noexcept> {
    using Invoke = Ret (*)(void*, Args...) noexcept;
    static constexpr Invoke get() noexcept {
      return [](void* f, Args... args) noexcept {
        std::invoke(*std::launder(static_cast<T*>(f)),
                    std::forward<Args>(args)...);
      };
    }
  };

  template<typename T, typename Ret, typename... Args>
  struct invoke_for<T, Ret(Args...) const noexcept> {
    using Invoke = Ret (*)(const void*, Args...) noexcept;
    static constexpr Invoke get() noexcept {
      return [](const void* f, Args... args) noexcept {
        std::invoke(*std::launder(static_cast<const T*>(f)),
                    std::forward<Args>(args)...);
      };
    }
  };

  template<typename T, typename Ret, typename... Args>
  struct invoke_for<T, Ret(Args...) volatile> {
    using Invoke = Ret (*)(volatile void*, Args...);

    static constexpr Invoke get() noexcept {
      return [](volatile void* f, Args... args) {
        std::invoke(*std::launder(static_cast<volatile T*>(f)),
                    std::forward<Args>(args)...);
      };
    }
  };

  template<typename T, typename Ret, typename... Args>
  struct invoke_for<T, Ret(Args...) volatile const> {
    using Invoke = Ret (*)(volatile const void*, Args...);
    static constexpr Invoke get() noexcept {
      return [](volatile const void* f, Args... args) {
        std::invoke(*std::launder(static_cast<volatile const T*>(f)),
                    std::forward<Args>(args)...);
      };
    }
  };

  template<typename T, typename Ret, typename... Args>
  struct invoke_for<T, Ret(Args...) volatile noexcept> {
    using Invoke = Ret (*)(volatile void*, Args...) noexcept;
    static constexpr Invoke get() noexcept {
      return [](volatile void* f, Args... args) noexcept {
        std::invoke(*std::launder(static_cast<T*>(f)),
                    std::forward<Args>(args)...);
      };
    }
  };

  template<typename T, typename Ret, typename... Args>
  struct invoke_for<T, Ret(Args...) volatile const noexcept> {
    using Invoke = Ret (*)(volatile const void*, Args...) noexcept;
    static constexpr Invoke get() noexcept {
      return [](volatile const void* f, Args... args) noexcept {
        std::invoke(*std::launder(static_cast<const T*>(f)),
                    std::forward<Args>(args)...);
      };
    }
  };

  template<typename T, typename Sig>
  constexpr inline VTable<Sig> table_v = {move_for<T, Sig>::get(),
                                          dtor_for<T, Sig>::get(),
                                          invoke_for<T, Sig>::get()};
} // namespace

template<typename F, typename Signature>
concept Callable =
    std::same_as<Signature, typename function_traits<F>::signature_type>;

template<typename F, typename Signature>
concept MoveOnlyCallable =
    std::move_constructible<F> and Callable<F, Signature>;
template<typename Signature>
class FunctionRefBase {
public:
  using traits      = signature_traits<Signature>;
  using return_type = typename traits::return_type;

  template<typename F>
  static constexpr bool bindable =
      traits::template matches<F> and
      (not std::same_as<std::decay_t<F>, FunctionRefBase>) and
      (not std::derived_from<std::decay_t<F>, FunctionRefBase>);

  constexpr FunctionRefBase() noexcept                             = default;
  constexpr FunctionRefBase(FunctionRefBase&& other) noexcept      = default;
  constexpr FunctionRefBase(const FunctionRefBase&) noexcept       = default;
  constexpr FunctionRefBase& operator=(FunctionRefBase&&) noexcept = default;
  constexpr FunctionRefBase&
  operator=(const FunctionRefBase&) noexcept = default;

  template<typename F>
  constexpr FunctionRefBase(F&& f)
    requires bindable<F>
  {
    bind(std::forward<F>(f));
  }

  template<typename F>
  constexpr FunctionRefBase& operator=(F&& f)
    requires bindable<F>
  {
    bind(std::forward<F>(f));
    return *this;
  }

  constexpr operator bool() const noexcept { return invoke != nullptr; }
  template<FunctionOf<Signature> F>
  constexpr void bind(F& f)
    requires bindable<F>
  {
    using T = std::decay_t<F>;
    invoke  = invoke_for<T, Signature>::get();
    buffer  = static_cast<ObjPtr>(&f);
  }

  constexpr void reset() {
    if (invoke == nullptr)
      return;
    invoke = nullptr;
    buffer = nullptr;
  }

protected:
  using InvokePtr  = typename traits::invoke_ptr_type;
  using ObjPtr     = typename traits::invoke_obj_type;
  InvokePtr invoke = nullptr;
  ObjPtr buffer    = nullptr;
};

/**
 * An non-owning, type erasing wrapper for callables, similar std::function. Any
 * callable F satisfying the calling signature can be referenced by FunctionRef.
 * @tparam Signature the function signature, e.g. ``void(int,double)``
 */
template<typename Signature>
class FunctionRef;

template<typename Ret, typename... Args>
class FunctionRef<Ret(Args...)> : public FunctionRefBase<Ret(Args...)> {
  using Base = FunctionRefBase<Ret(Args...)>;

public:
  using Base::Base;
  using Base::operator=;
  using Base::operator bool;
  using Base::bind;
  using Base::reset;
  constexpr CM_INLINE Ret operator()(Args... args) {
    return this->invoke(this->buffer, std::forward<Args>(args)...);
  }
};

template<typename Ret, typename... Args>
class FunctionRef<Ret(Args...) noexcept>
    : public FunctionRefBase<Ret(Args...) noexcept> {
  using Base = FunctionRefBase<Ret(Args...) noexcept>;

public:
  using Base::Base;
  using Base::operator=;
  using Base::operator bool;
  using Base::bind;
  using Base::reset;
  constexpr CM_INLINE Ret operator()(Args&&... args) noexcept {
    return this->invoke(this->buffer, std::forward<Args>(args)...);
  }
};

template<typename Ret, typename... Args>
class FunctionRef<Ret(Args...) const>
    : public FunctionRefBase<Ret(Args...) const> {
  using Base = FunctionRefBase<Ret(Args...) const>;

public:
  using Base::Base;
  using Base::operator=;
  using Base::operator bool;
  using Base::bind;
  using Base::reset;
  constexpr CM_INLINE Ret operator()(Args... args) const {
    return this->invoke(this->buffer, std::forward<Args>(args)...);
  }
};

template<typename Ret, typename... Args>
class FunctionRef<Ret(Args...) const noexcept>
    : public FunctionRefBase<Ret(Args...) const noexcept> {
  using Base = FunctionRefBase<Ret(Args...) const noexcept>;

public:
  using Base::Base;
  using Base::operator=;
  using Base::operator bool;
  using Base::bind;
  using Base::reset;
  constexpr CM_INLINE Ret operator()(Args&&... args) const noexcept {
    return this->invoke(this->buffer, std::forward<Args>(args)...);
  }
};

template<typename Ret, typename... Args>
class FunctionRef<Ret(Args...) volatile>
    : public FunctionRefBase<Ret(Args...) volatile> {
  using Base = FunctionRefBase<Ret(Args...) volatile>;

public:
  using Base::Base;
  using Base::operator=;
  using Base::operator bool;
  using Base::bind;
  using Base::reset;
  CM_INLINE Ret operator()(Args... args) volatile {
    return this->invoke(this->buffer, std::forward<Args>(args)...);
  }
};

template<typename Ret, typename... Args>
class FunctionRef<Ret(Args...) volatile noexcept>
    : public FunctionRefBase<Ret(Args...) volatile noexcept> {
  using Base = FunctionRefBase<Ret(Args...) volatile noexcept>;

public:
  using Base::Base;
  using Base::operator=;
  using Base::operator bool;
  using Base::bind;
  using Base::reset;
  CM_INLINE Ret operator()(Args&&... args) volatile noexcept {
    return this->invoke(this->buffer, std::forward<Args>(args)...);
  }
};

template<typename Ret, typename... Args>
class FunctionRef<Ret(Args...) volatile const>
    : public FunctionRefBase<Ret(Args...) volatile const> {
  using Base = FunctionRefBase<Ret(Args...) volatile const>;

public:
  using Base::Base;
  using Base::operator=;
  using Base::operator bool;
  using Base::bind;
  using Base::reset;
  CM_INLINE Ret operator()(Args... args) volatile const {
    return this->invoke(this->buffer, std::forward<Args>(args)...);
  }
};

template<typename Ret, typename... Args>
class FunctionRef<Ret(Args...) volatile const noexcept>
    : public FunctionRefBase<Ret(Args...) volatile const noexcept> {
  using Base = FunctionRefBase<Ret(Args...) volatile const noexcept>;

public:
  using Base::Base;
  using Base::operator=;
  using Base::operator bool;
  using Base::bind;
  using Base::reset;
  CM_INLINE Ret operator()(Args&&... args) volatile const noexcept {
    return this->invoke(this->buffer, std::forward<Args>(args)...);
  }
};

// clang-format off
static_assert(FunctionConcept<FunctionRef<void()>>);
static_assert(FunctionConcept<FunctionRef<void() noexcept>>);
static_assert(FunctionConcept<FunctionRef<void() const>>);
static_assert(FunctionConcept<FunctionRef<void() const noexcept>>);
static_assert(FunctionConcept<FunctionRef<void() volatile>>);
static_assert(FunctionConcept<FunctionRef<void() volatile noexcept>>);
static_assert(FunctionConcept<FunctionRef<void() volatile const>>);
static_assert(FunctionConcept<FunctionRef<void() volatile const noexcept>>);

static_assert(FunctionConcept<FunctionRef<int()>>);
static_assert(FunctionConcept<FunctionRef<int() noexcept>>);
static_assert(FunctionConcept<FunctionRef<int() const>>);
static_assert(FunctionConcept<FunctionRef<int() const noexcept>>);
static_assert(FunctionConcept<FunctionRef<int() volatile>>);
static_assert(FunctionConcept<FunctionRef<int() volatile noexcept>>);
static_assert(FunctionConcept<FunctionRef<int() volatile const>>);
static_assert(FunctionConcept<FunctionRef<int() volatile const noexcept>>);

static_assert(FunctionConcept<FunctionRef<void(int)>>);
static_assert(FunctionConcept<FunctionRef<void(int) noexcept>>);
static_assert(FunctionConcept<FunctionRef<void(int) const>>);
static_assert(FunctionConcept<FunctionRef<void(int) const noexcept>>);
static_assert(FunctionConcept<FunctionRef<void(int) volatile>>);
static_assert(FunctionConcept<FunctionRef<void(int) volatile noexcept>>);
static_assert(FunctionConcept<FunctionRef<void(int) volatile const>>);
static_assert(FunctionConcept<FunctionRef<void(int) volatile const noexcept>>);

static_assert(FunctionConcept<FunctionRef<int(int)>>);
static_assert(FunctionConcept<FunctionRef<int(int) noexcept>>);
static_assert(FunctionConcept<FunctionRef<int(int) const>>);
static_assert(FunctionConcept<FunctionRef<int(int) const noexcept>>);
static_assert(FunctionConcept<FunctionRef<int(int) volatile>>);
static_assert(FunctionConcept<FunctionRef<int(int) volatile noexcept>>);
static_assert(FunctionConcept<FunctionRef<int(int) volatile const>>);
static_assert(FunctionConcept<FunctionRef<int(int) volatile const noexcept>>);
// clang-format on

/**
 * An owning, type erasing wrapper for callables, like std::function, that does
 * not heap allocate by using an internal buffer. Any callable F satisfying the
 * calling signature can be stored if sizeof(F) <= Size, alignof(F) <= Align,
 * and F is move constructible.
 * @tparam Signature the function signature, e.g. ``void(int,double)``
 * @tparam Size the buffer size
 * @tparam Align the buffer aignment
 */
template<typename Signature, std::size_t Size, std::size_t Align>
class MoveOnlyFunctionBase {
public:
  template<typename F>
  static constexpr bool bindable =

      (not std::same_as<std::decay_t<F>, MoveOnlyFunctionBase>) and
      (not std::derived_from<std::decay_t<F>, MoveOnlyFunctionBase>) and
      alignof(std::decay_t<F>) <= Align and sizeof(std::decay_t<F>) <= Size;

  using traits      = signature_traits<Signature>;
  using return_type = typename traits::return_type;

  constexpr MoveOnlyFunctionBase() noexcept = default;

  constexpr MoveOnlyFunctionBase(MoveOnlyFunctionBase&& other) {
    other.vtable->move(buffer, other.buffer);
    vtable = other.vtable;
    other.reset();
  }

  MoveOnlyFunctionBase(const MoveOnlyFunctionBase&)            = delete;
  MoveOnlyFunctionBase& operator=(const MoveOnlyFunctionBase&) = delete;

  template<typename F>
  constexpr MoveOnlyFunctionBase(F&& f)
    requires bindable<F> and MoveOnlyCallable<std::decay_t<F>, Signature>
  {
    bind(std::forward<F>(f));
  }

  MoveOnlyFunctionBase& operator=(MoveOnlyFunctionBase&& other) {
    reset();
    other.vtable->move(buffer, other.buffer);
    vtable = other.vtable;
    other.reset();
    return *this;
  }

  template<typename F>
  constexpr MoveOnlyFunctionBase& operator=(F&& f)
    requires bindable<F> and MoveOnlyCallable<std::decay_t<F>, Signature>
  {
    bind(std::forward<F>(f));
    return *this;
  }

  constexpr ~MoveOnlyFunctionBase() {
    if (vtable == nullptr)
      return;
    vtable->dtor(buffer);
    vtable = nullptr;
  }

  constexpr operator bool() const noexcept { return vtable != nullptr; }

  template<typename F>
  constexpr void bind(F&& f)
    requires bindable<F> and MoveOnlyCallable<std::decay_t<F>, Signature>
  {
    reset();
    using T = std::decay_t<F>;
    vtable  = &table_v<T, Signature>;
    construct_at(reinterpret_cast<T*>(buffer), std::forward<F>(f));
  }

  constexpr void reset() {
    if (vtable == nullptr)
      return;
    vtable->dtor(buffer);
    vtable = nullptr;
  }

protected:
  const VTable<Signature>* vtable = nullptr;
  alignas(Align) unsigned char buffer[Size]{};
};
/**
 * An owning, type erasing wrapper for callables, like std::function, that does
 * not heap allocate by using an internal buffer. Any callable F satisfying the
 * calling signature can be stored if sizeof(F) <= Size, alignof(F) <= Align,
 * and F is move constructible.
 * @tparam Signature the function signature, e.g. ``void(int,double)``
 * @tparam Size the buffer size
 * @tparam Align the buffer aignment
 */
template<typename Signature, std::size_t Size, std::size_t Align>
class MoveOnlyFunction;

template<typename Ret, typename... Args, std::size_t Size, std::size_t Align>
class MoveOnlyFunction<Ret(Args...), Size, Align>
    : MoveOnlyFunctionBase<Ret(Args...), Size, Align> {
  using Base = MoveOnlyFunctionBase<Ret(Args...), Size, Align>;

public:
  using Base::Base;
  using Base::operator=;
  using Base::operator bool;
  using Base::bind;
  using Base::reset;
  constexpr CM_INLINE Ret operator()(Args... args) {
    return this->vtable->invoke(this->buffer, std::forward<Args>(args)...);
  }
};

template<typename Ret, typename... Args, std::size_t Size, std::size_t Align>
class MoveOnlyFunction<Ret(Args...) noexcept, Size, Align>
    : MoveOnlyFunctionBase<Ret(Args...) noexcept, Size, Align> {
  using Base = MoveOnlyFunctionBase<Ret(Args...) noexcept, Size, Align>;

public:
  using Base::Base;
  using Base::operator=;
  using Base::operator bool;
  using Base::bind;
  using Base::reset;
  constexpr CM_INLINE Ret operator()(Args... args) noexcept {
    return this->vtable->invoke(this->buffer, std::forward<Args>(args)...);
  }
};

template<typename Ret, typename... Args, std::size_t Size, std::size_t Align>
class MoveOnlyFunction<Ret(Args...) const, Size, Align>
    : MoveOnlyFunctionBase<Ret(Args...) const, Size, Align> {
  using Base = MoveOnlyFunctionBase<Ret(Args...) const, Size, Align>;

public:
  using Base::Base;
  using Base::operator=;
  using Base::operator bool;
  using Base::bind;
  using Base::reset;
  constexpr CM_INLINE Ret operator()(Args... args) const {
    return this->vtable->invoke(this->buffer, std::forward<Args>(args)...);
  }
};

template<typename Ret, typename... Args, std::size_t Size, std::size_t Align>
class MoveOnlyFunction<Ret(Args...) const noexcept, Size, Align>
    : MoveOnlyFunctionBase<Ret(Args...) const noexcept, Size, Align> {
  using Base = MoveOnlyFunctionBase<Ret(Args...) const noexcept, Size, Align>;

public:
  using Base::Base;
  using Base::operator=;
  using Base::operator bool;
  using Base::bind;
  using Base::reset;
  constexpr CM_INLINE Ret operator()(Args... args) const noexcept {
    return this->vtable->invoke(this->buffer, std::forward<Args>(args)...);
  }
};

template<typename Ret, typename... Args, std::size_t Size, std::size_t Align>
class MoveOnlyFunction<Ret(Args...) volatile, Size, Align>
    : MoveOnlyFunctionBase<Ret(Args...) volatile, Size, Align> {
  using Base = MoveOnlyFunctionBase<Ret(Args...) volatile, Size, Align>;

public:
  using Base::Base;
  using Base::operator=;
  using Base::operator bool;
  using Base::bind;
  using Base::reset;
  CM_INLINE Ret operator()(Args... args) volatile {
    return this->vtable->invoke(this->buffer, std::forward<Args>(args)...);
  }
};

template<typename Ret, typename... Args, std::size_t Size, std::size_t Align>
class MoveOnlyFunction<Ret(Args...) volatile noexcept, Size, Align>
    : MoveOnlyFunctionBase<Ret(Args...) volatile noexcept, Size, Align> {
  using Base =
      MoveOnlyFunctionBase<Ret(Args...) volatile noexcept, Size, Align>;

public:
  using Base::Base;
  using Base::operator=;
  using Base::operator bool;
  using Base::bind;
  using Base::reset;
  CM_INLINE Ret operator()(Args... args) volatile noexcept {
    return this->vtable->invoke(this->buffer, std::forward<Args>(args)...);
  }
};

template<typename Ret, typename... Args, std::size_t Size, std::size_t Align>
class MoveOnlyFunction<Ret(Args...) volatile const, Size, Align>
    : MoveOnlyFunctionBase<Ret(Args...) volatile const, Size, Align> {
  using Base = MoveOnlyFunctionBase<Ret(Args...) volatile const, Size, Align>;

public:
  using Base::Base;
  using Base::operator=;
  using Base::operator bool;
  using Base::bind;
  using Base::reset;
  CM_INLINE Ret operator()(Args... args) volatile const {
    return this->vtable->invoke(this->buffer, std::forward<Args>(args)...);
  }
};

template<typename Ret, typename... Args, std::size_t Size, std::size_t Align>
class MoveOnlyFunction<Ret(Args...) volatile const noexcept, Size, Align>
    : MoveOnlyFunctionBase<Ret(Args...) volatile const noexcept, Size, Align> {
  using Base =
      MoveOnlyFunctionBase<Ret(Args...) volatile const noexcept, Size, Align>;

public:
  using Base::Base;
  using Base::operator=;
  using Base::operator bool;
  using Base::bind;
  using Base::reset;

  CM_INLINE Ret operator()(Args... args) volatile const noexcept {
    return this->vtable->invoke(this->buffer, std::forward<Args>(args)...);
  }
};

// clang-format off
static_assert(FunctionConcept<MoveOnlyFunction<void(), 4, 4>>);
static_assert(FunctionConcept<MoveOnlyFunction<void() noexcept, 4, 4>>);
static_assert(FunctionConcept<MoveOnlyFunction<void() const, 4, 4>>);
static_assert(FunctionConcept<MoveOnlyFunction<void() const noexcept, 4, 4>>);
static_assert(FunctionConcept<MoveOnlyFunction<void() volatile , 4, 4>>);
static_assert(FunctionConcept<MoveOnlyFunction<void() volatile noexcept, 4, 4>>);
static_assert(FunctionConcept<MoveOnlyFunction<void() volatile const, 4, 4>>);
static_assert(FunctionConcept<MoveOnlyFunction<void() volatile const noexcept, 4, 4>>);

static_assert(FunctionConcept<MoveOnlyFunction<int(), 4, 4>>);
static_assert(FunctionConcept<MoveOnlyFunction<int() noexcept, 4, 4>>);
static_assert(FunctionConcept<MoveOnlyFunction<int() const, 4, 4>>);
static_assert(FunctionConcept<MoveOnlyFunction<int() const noexcept, 4, 4>>);
static_assert(FunctionConcept<MoveOnlyFunction<int() volatile , 4, 4>>);
static_assert(FunctionConcept<MoveOnlyFunction<int() volatile noexcept, 4, 4>>);
static_assert(FunctionConcept<MoveOnlyFunction<int() volatile const, 4, 4>>);
static_assert(FunctionConcept<MoveOnlyFunction<int() volatile const noexcept, 4, 4>>);

static_assert(FunctionConcept<MoveOnlyFunction<void(int), 4, 4>>);
static_assert(FunctionConcept<MoveOnlyFunction<void(int) noexcept, 4, 4>>);
static_assert(FunctionConcept<MoveOnlyFunction<void(int) const, 4, 4>>);
static_assert(FunctionConcept<MoveOnlyFunction<void(int) const noexcept, 4, 4>>);
static_assert(FunctionConcept<MoveOnlyFunction<void(int) volatile , 4, 4>>);
static_assert(FunctionConcept<MoveOnlyFunction<void(int) volatile noexcept, 4, 4>>);
static_assert(FunctionConcept<MoveOnlyFunction<void(int) volatile const, 4, 4>>);
static_assert(FunctionConcept<MoveOnlyFunction<void(int) volatile const noexcept, 4, 4>>);

static_assert(FunctionConcept<MoveOnlyFunction<int(int), 4, 4>>);
static_assert(FunctionConcept<MoveOnlyFunction<int(int) noexcept, 4, 4>>);
static_assert(FunctionConcept<MoveOnlyFunction<int(int) const, 4, 4>>);
static_assert(FunctionConcept<MoveOnlyFunction<int(int) const noexcept, 4, 4>>);
static_assert(FunctionConcept<MoveOnlyFunction<int(int) volatile , 4, 4>>);
static_assert(FunctionConcept<MoveOnlyFunction<int(int) volatile noexcept, 4, 4>>);
static_assert(FunctionConcept<MoveOnlyFunction<int(int) volatile const, 4, 4>>);
static_assert(FunctionConcept<MoveOnlyFunction<int(int) volatile const noexcept, 4, 4>>);
// clang-format on
template<typename Signature, typename Function>
class IntrusiveFunctionBase {
  using traits   = signature_traits<Signature>;
  Function* next = nullptr;
  template<typename Sig, typename Func>
  friend class IntrusiveFunctionList;
};

template<typename Signature, typename Function>
class IntrusiveFunctionList {
  static_assert(std::derived_from<
                    Function, IntrusiveFunctionBase<Signature, Function>> and
                FunctionOf<Function, Signature>);

  using traits = signature_traits<Signature>;

  Function* head = nullptr;
  Function* tail = nullptr;

public:
  using value_type = Function;

  class iterator : public std::forward_iterator_tag {
    Function* f;

  public:
    using value_type      = Function;
    using difference_type = std::ptrdiff_t;
    constexpr iterator(Function* f) : f(f) {}
    constexpr iterator(const iterator&)            = default;
    constexpr iterator(iterator&&)                 = default;
    constexpr iterator& operator=(const iterator&) = default;
    constexpr iterator& operator=(iterator&&)      = default;
    constexpr value_type& operator*() { return *f; }
    constexpr value_type& operator->() { return *f; }
    constexpr iterator& operator++() noexcept {
      if (f != nullptr)
        f = f->next;
      return *this;
    }
    constexpr iterator operator++(int) noexcept {
      if (f == nullptr)
        return nullptr;
      auto it = f;
      f       = f->next;
      return it;
    }
    constexpr bool operator==(const iterator& other) const noexcept {
      return f == other.f;
    }
    constexpr bool operator!=(const iterator& other) const noexcept {
      return f != other.f;
    }
  };
  constexpr IntrusiveFunctionList() = default;
  void push_back(Function& function) noexcept {
    if (head == nullptr) {
      head          = &function;
      tail          = &function;
      function.next = nullptr;
      return;
    }
    tail->next = &function;
    tail       = tail->next;
  }

  void clear() {
    Function* it = head;
    head         = nullptr;
    tail         = nullptr;
    while (it != nullptr) {
      Function* next = it->next;
      it->next       = nullptr;
      it             = next;
    }
  }
  constexpr iterator begin() noexcept { return head; }
  constexpr iterator end() noexcept { return nullptr; }
};

template<typename T>
struct FalseCompar {
  constexpr bool operator()(const T&, const T&) const noexcept { return false; }
};

template<typename T>
struct TrueCompar {
  constexpr bool operator()(const T&, const T&) const noexcept { return true; }
};

template<typename Signature, typename Function,
         typename Compar = std::less<Function>>
class SortedIntrusiveFunctionList : Compar {
  static_assert(std::derived_from<
                    Function, IntrusiveFunctionBase<Signature, Function>> and
                FunctionOf<Function, Signature>);
  static_assert(
      std::is_invocable_r_v<bool, Compar, const Function&, const Function&>);
  using traits = signature_traits<Signature>;

  Function* head = nullptr;
  Function* tail = nullptr;

  constexpr bool less_than(const Function& a, const Function& b) {
    return static_cast<Compar&>(*this)(a, b);
  }

public:
  using value_type = Function;

  class iterator : public std::forward_iterator_tag {
    Function* f;

  public:
    using value_type      = Function;
    using difference_type = std::ptrdiff_t;
    constexpr iterator(Function* f) : f(f) {}
    constexpr iterator(const iterator&)            = default;
    constexpr iterator(iterator&&)                 = default;
    constexpr iterator& operator=(const iterator&) = default;
    constexpr iterator& operator=(iterator&&)      = default;
    constexpr value_type& operator*() { return *f; }
    constexpr value_type& operator->() { return *f; }
    constexpr iterator& operator++() noexcept {
      if (f != nullptr)
        f = f->next;
      return *this;
    }
    constexpr iterator operator++(int) noexcept {
      if (f == nullptr)
        return nullptr;
      auto it = f;
      f       = f->next;
      return it;
    }
    constexpr bool operator==(const iterator& other) const noexcept {
      return f == other.f;
    }
    constexpr bool operator!=(const iterator& other) const noexcept {
      return f != other.f;
    }
  };

  constexpr SortedIntrusiveFunctionList() = default;

  constexpr void push_back(Function& function) noexcept {
    if (head == nullptr) {
      // empty list
      head          = &function;
      tail          = &function;
      function.next = nullptr;
      return;
    }

    if (head == tail) {
      // list with one element
      if (less_than(function, *head)) {
        head       = &function;
        head->next = tail;
      } else {
        tail       = &function;
        tail->next = nullptr;
        head->next = tail;
      }
      return;
    }

    // list with at least two elements
    Function* it   = head->next;
    Function* prev = head;
    while (it != nullptr) {
      if (less_than(function, *it)) {
        prev->next    = &function;
        function.next = it;
        return;
      }
      prev = it;
      it   = it->next;
    }

    tail->next = &function;
    tail       = tail->next;
  }

  constexpr bool erase(Function& function) noexcept {
    if (head == nullptr)
      return false;

    if (head == tail) {
      if (head != &function)
        return false;
      head          = nullptr;
      tail          = nullptr;
      function.next = nullptr;
      return true;
    }

    if (head == &function) {
      head          = function.next;
      function.next = nullptr;
      return true;
    }

    Function* it   = head->next;
    Function* prev = head;
    while (it != nullptr) {
      if (it == &function) {
        prev->next    = function.next;
        function.next = nullptr;
        if (it == tail) {
          tail = prev;
        }
        return true;
      }
      prev = it;
      it   = it->next;
    }
    return false;
  }

  constexpr bool erase(iterator pos) noexcept {
    if (pos == end())
      return false;
    return erase(*pos);
  }

  void clear() {
    Function* it = head;
    head         = nullptr;
    tail         = nullptr;
    while (it != nullptr) {
      Function* next = it->next;
      it->next       = nullptr;
      it             = next;
    }
  }
  constexpr iterator begin() noexcept { return head; }
  constexpr iterator end() noexcept { return nullptr; }
};

template<typename Signature>
class IntrusiveFunctionRef
    : public FunctionRef<Signature>,
      public IntrusiveFunctionBase<Signature, IntrusiveFunctionRef<Signature>> {
public:
  using FunctionRef<Signature>::operator();
  using FunctionRef<Signature>::operator bool;
};

template<typename Signature>
class IntrusiveFunctionRefList
    : public IntrusiveFunctionList<Signature, IntrusiveFunctionRef<Signature>> {
public:
  using Base =
      IntrusiveFunctionList<Signature, IntrusiveFunctionRef<Signature>>;
  using value_type = typename Base::value_type;
  using iterator   = typename Base::iterator;
  using Base::begin;
  using Base::clear;
  using Base::end;
  using Base::push_back;
};

template<typename Signature, std::size_t Size, std::size_t Align>
class IntrusiveMoveOnlyFunction
    : public MoveOnlyFunction<Signature, Size, Align>,
      public IntrusiveFunctionBase<
          Signature, IntrusiveMoveOnlyFunction<Signature, Size, Align>> {
public:
  using MoveOnlyFunction<Signature, Size, Align>::operator();
  using MoveOnlyFunction<Signature, Size, Align>::operator bool;
};

template<typename Signature, std::size_t Size, std::size_t Align>
class IntrusiveMoveOnlyFunctionList
    : public IntrusiveFunctionList<
          Signature, IntrusiveMoveOnlyFunction<Signature, Size, Align>> {
public:
  using Base =
      IntrusiveFunctionList<Signature,
                            IntrusiveMoveOnlyFunction<Signature, Size, Align>>;
  using value_type = typename Base::value_type;
  using iterator   = typename Base::iterator;
  using Base::begin;
  using Base::clear;
  using Base::end;
  using Base::push_back;
};

} // namespace cm
#endif
