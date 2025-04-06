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

#ifndef CM_EVENT_DISPATCHER_HPP
#define CM_EVENT_DISPATCHER_HPP
#include "cortex/assert.hpp"
#include "cortex/event.hpp"
#include "cortex/fpu.hpp"
#include "cortex/nvic.hpp"
#include "cortex/systick.hpp"
// #include "scheduler_implementation.hpp"
#include "cortex/type_list.hpp"
#include "type_list/type_list.hpp"

#include <concepts>
#include <cstddef>
#include <type_traits>
#include <utility>

namespace cm {

template<EventSpec E, typename... Args>
  requires std::constructible_from<event_handler_type_t<E>, Args...>
struct EvInit : Tuple<Args...> {
  using event = E;
  constexpr std::decay_t<typename E::handler_type> init() const& {
    return init_impl(std::make_index_sequence<sizeof...(Args)>());
  }
  constexpr std::decay_t<typename E::handler_type> init() && {
    return init_impl(std::make_index_sequence<sizeof...(Args)>());
  }

  constexpr EvInit(const E&, Args&&... args)
      : Tuple<Args...>(std::forward<Args>(args)...) {}

private:
  template<std::size_t... Is>
  constexpr std::decay_t<typename E::handler_type>
  init_impl(std::index_sequence<Is...>) const& {
    return {this->template get<Is>()...};
  }
  template<std::size_t... Is>
  constexpr std::decay_t<typename E::handler_type>
  init_impl(std::index_sequence<Is...>) && {
    return {std::move(this->template get<Is>())...};
  }
};

template<EventSpec E, typename... Args>
EvInit(const E&, Args&&...) -> EvInit<E, std::decay_t<Args>...>;

template<typename T>
concept EventInit = requires(T&& t) {
  { typename T::event{} } -> EventSpec;
  { t.init() } -> std::convertible_to<typename T::event::handler_type>;
};

/**
 * The event dispatcher is responsible for event handling. The cortex version
 * makes extensive use of the nvic.
 *
 * How it works:
 *
 * The EventDispatcher consists of two main components:
 * - a vector table
 * - a handler tuple
 *
 * The handler tuple stores the advanced event handlers
 * The vector table contains the raw event handlers and functions linking the
 * interrupts to their advanced handlers.
 *
 * @tparam NvicTraits the nvic to use. The nvics vector table must be
 * relocatable.
 * @tparam Events event to register with the dispatcher. Must be of type
 * EventSpec<...> or equivalent.
 */
template<typename Core, EventSpec... Events>
class EventDispatcher {

  using nvic_type        = typename Core::nvic;
  using events           = TypeList<Events...>;
  // type list of events which can be put directly in the vector table, as the
  // handler type is a function pointer taking no arguments.
  using raw_events       = filter_t<impl::is_raw_event, events>;
  using fixed_raw_events = filter_t<impl::is_fixed_raw_event, events>;

  // type list of events which need the irq_jump link.
  using advanced_events = filter_t<impl::is_advanced_event, events>;

  // the tuple used to stored advanced event handlers
  using EventHandlerTuple =
      apply_t<Tuple, transform_t<event_handler_type, advanced_events>>;

  template<IRQn Id>
  static inline constexpr bool contains_event_v = ((Events::id == Id) or ...);

public:
  using vector_table_type = typename nvic_type::vector_table_type;

  constexpr EventDispatcher() = default;

  template<EventSpec Event>
  static void enable() {
    static_assert(contains_v<Event, events>,
                  "Event is not listed in the template parameters of this "
                  "event dispatcher");
    nvic_type::irq_enable(Event::id);
  }

  template<EventSpec Event>
  static void disable() {
    static_assert(contains_v<Event, events>,
                  "Event is not listed in the template parameters of this "
                  "event dispatcher");
    nvic_type::irq_disable(Event::id);
  }

  /**
   * @brief access the events handler.
   *
   * If Event is a raw event, a reference to corresponding location in the
   * vector table is returned, i.e. a volatile IrqHandler&. Else a reference to
   * the handler is returned. If the event is fixed, the returned reference is
   * const.
   *
   * @tparam Event the event
   */
  template<EventSpec Event>
  static handler_reference_t<Event> get_handler() {
    if constexpr (RawHandler<Event>) {
      return vec_table.IRQ[Event::id];
    } else {
      event_handlers.template get<index_of_v<Event, advanced_events>>();
    }
  }

  /**
   * @brief set the events user handler by forwarding args to the event handlers
   * set_user_handler method and returning the result. Equivalent to
   * ``get_handler<Event>().set_user_handler(args...)``. The exact requirements
   * for args are dependent on the event handler.
   *
   * Only available if the event handler has a ``set_user_handler`` method.
   *
   * @tparam Event the event
   * @tparam Args the arguments' types
   * @param args the arguments
   * @return whatever event_handler_type_t<Event>::set_user_handler(args...)
   * returns.
   */
  template<EventSpec Event, typename... Args>
    requires HasSetUserHandler<event_handler_type_t<Event>, Args&&...>
  static CM_INLINE decltype(auto) set_user_handler(Args&&... args) {
    return get_handler<Event>().set_user_handler(std::forward<Args>(args)...);
  }

  /**
   * @brief erase the events user handler by forwarding args to the event
   * handlers erase_user_handler method and returning the result. Equivalent to
   * ``get_handler<Event>().erase_user_handler(args...)``. The exact
   * requirements for args are dependent on the event handler. Usually an
   * iterator or reference to the user handler to be removed is passed in.
   *
   * Only available if the event handler has an ``erase_user_handler`` method.
   *
   * @tparam Event the event
   * @tparam Args the arguments' types
   * @param args the arguments
   * @return whatever event_handler_type_t<Event>::erase_user_handler(args...)
   * returns.
   */
  template<EventSpec Event, typename... Args>
    requires HasEraseUserHandler<event_handler_type_t<Event>, Args&&...>
  static CM_INLINE decltype(auto) erase_user_handler(Args&&... args) {
    return get_handler<Event>().erase_user_handler(std::forward<Args>(args)...);
  }

  /**
   * @brief set a single argument of an event by index.
   *
   * Advanced events can have parameters passed to them. These are set with this
   * method.
   *
   *
   * ``Is`` is the index pack. Usually, ``Is`` is just a single index.
   *
   * Example:
   *
   * ```
   * struct Param{
   *  char c;
   *  int i;
   *  const char* s;
   * };
   *
   * // the event
   * using E = AnyEvent<IRQ0, 0, void(int, double, const Param&), 16, 4>;
   *
   * // the event dispatcher
   * using D = EventDispatcher<Core, ..., E, ...>;
   * // set the first argument
   * D::set_arg<E, 0>(15);
   * // set the second argument
   * D::set_arg<E, 1>(2.0);
   * // set the third argument
   * D::set_arg<E, 2>('x', 42, "hello world!");
   * // user handler will be called in IRQ0 with the arguments set to the values
   * // above.
   * D::notify<E>();
   * ```
   *
   * ``Is`` is a pack to allow for event handlers which contain a map/table-like
   * container of user handlers.
   *
   * Example:
   *
   * To set the ``K``-th argument of the ``I``-th user handler of the event
   * ``E`` of the dispatcher ``D``.
   *
   * ```
   * constexpr size_t K = 2;
   * constexpr size_t I = 5;
   * D::set_arg<E, I, K>(...);
   * ```
   *
   * @tparam Event the event
   * @tparam Is the indices. Usually a single index.
   * @tparam Args deduced
   * @param args the values used to construct the argument
   * @return get_handler<Event>.set_arg<Is...>(args...)
   */
  template<EventSpec Event, std::size_t... Is, typename... Args>
    requires HasSetArg<event_handler_type_t<Event>, std::index_sequence<Is...>,
                       Args&&...>
  static CM_INLINE decltype(auto) set_arg(Args&&... args) {
    return event_handlers.template get<index_of_v<Event, advanced_events>>()
        .template set_arg<Is...>(std::forward<Args>(args)...);
  }

  /**
   * @brief set an events arguments.
   *
   * Pseudo code equivalent: ``set_arg<Event,0>(args[0]),
   * set_arg<Event,1>(args[1]), ..., set_arg<Event,
   * sizeof...(Args)-1>(args[sizeof...(Args)-1])``
   *
   * @tparam Event the event
   * @tparam Args the arguments' types
   * @param args the arguments
   */
  template<EventSpec Event, typename... Args>
    requires CanSetArgs<event_handler_type_t<Event>, Args&&...>
  static CM_INLINE void set_args(Args&&... args) {
    set_args<Event>(std::make_index_sequence<sizeof...(Args)>(),
                    std::forward<Args>(args)...);
  }

  /**
   * @brief trigger an event by activating its IRQ
   *
   * @tparam Event the event to trigger
   */
  template<EventSpec Event>
  static CM_INLINE void notify() {
    nvic_type::irq_trigger(Event::id);
  }

  /**
   * @brief sets the arguments of an event an then triggers it. Equivalent to
   * ``set_args<Event>(args...); notify<Event>();``.
   *
   * @tparam Event the event to trigger
   * @tparam Args the arguments' types for Event
   * @param args the arguments
   */
  template<EventSpec Event, typename... Args>
    requires CanSetArgs<event_handler_type_t<Event>, Args&&...>
  static CM_INLINE void notify(Args&&... args) {
    set_args<Event>(std::forward<Args>(args)...);
    nvic_type::irq_trigger(Event::id);
  }

  /**
   * Initialize the event dispatcher. This will relocate the vector table and
   * fill in the event related entries. Entries in the original vector table are
   * preserved.
   *
   * TODO: find a method to do this on M0, where there is no VTOR.
   */
  template<EventInit... EventInit>
  static CM_INLINE void init(EventInit&&... init) {
    CM_ASSERT_MESSAGE(nvic_type::can_relocate_vector_table(),
                      "The EventDispatcher requires a relocatable vector "
                      "table, i.e. the VTOR register must be implemented.")
    switch (nvic_type::relocate_vector_table(vec_table, true)) {
    case RelocateStatus::Success:
      break;
    case RelocateStatus::InvalidAddress:
      [[fallthrough]];
    case RelocateStatus::InvalidAlignment:
      [[fallthrough]];
    case RelocateStatus::VtorNotPresent:
      // TODO: real status handling
      return;
    }
    // CM_ASSERT_MESSAGE(
    //     reinterpret_cast<const char*>(this) ==
    //         reinterpret_cast<const char*>(&vec_table),
    //     "The event dispatcher cannot be initialized properly because the "
    //     "offset between the vec_table member and the this pointer is not
    //     0.");
    init_fixed_raw_events(fixed_raw_events{});
    init_advanced_events(advanced_events{});
    set_priorities(events{});
    (init_ev(std::forward<EventInit>(init)), ...);
    (nvic_type::irq_enable(Events::id), ...);
  }
  //
  // /**
  //  * grants cortex scheduler implementations access to the vector table.
  //  * The implementation instance acts as the key, which nobody but the
  //  * implementation and the Scheduler have access to.
  //  */
  // template<Priority P>
  // CM_INLINE vector_table_type& get_vec_table(
  //     SchedulerImplementation<Core, P, Events...>&) {
  //   return vec_table;
  // }

private:
  template<EventInit EventInit>
  static void init_ev(const EventInit& args) {
    if constexpr (RawHandler<typename EventInit::event::handler_type>) {
      vec_table.IRQ[EventInit::event::id] = args.init();
    } else {
      event_handlers.template get<
          index_of_v<advanced_events, typename EventInit::event>>() =
          args.init();
    }
  }
  template<EventSpec... Evs>
  static CM_INLINE void init_fixed_raw_events(TypeList<Evs...>) {
    (static_cast<void>(vec_table.IRQ[Evs::id] = typename Evs::handler_type{}),
     ...);
  }

  // sets the irq jump functions
  template<EventSpec... Evs>
  static CM_INLINE void init_advanced_events(TypeList<Evs...>) {
    (static_cast<void>(vec_table.IRQ[Evs::id] = &irq_jump<Evs>), ...);
  }

  template<EventSpec... Evs>
  static CM_INLINE void set_priorities(TypeList<Evs...>) {
    (nvic_type::irq_set_priority(Evs::id, Evs::priority), ...);
  }

  template<typename Event>
  static void irq_jump() noexcept {
    CM_ASSERT(Event::id == nvic_type::irq_current());
    std::invoke(
        event_handlers.template get<index_of_v<Event, advanced_events>>());
  }

  template<EventSpec Event, std::size_t... Is, typename... Args>
    requires CanSetArgs<event_handler_type_t<Event>, Args&&...>
  static CM_INLINE void set_args(std::index_sequence<Is...>, Args&&... args) {
    // TODO static assertion for compatibility of Args and the event handler
    (set_arg<Is>(std::forward<Args>(args)), ...);
  }
  static constinit inline vector_table_type vec_table{};
  static constinit inline EventHandlerTuple event_handlers{};
};
} // namespace cm
#endif
