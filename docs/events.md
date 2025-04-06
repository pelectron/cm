# Events

<!--toc:start-->
- [Events](#events)
  - [Components](#components)
  - [Event(handlers) vs Tasks](#eventhandlers-vs-tasks)
  - [Event Attributes](#event-attributes)
  - [Raw Events](#raw-events)
    - [Example](#example)
  - [Advanced Events](#advanced-events)
    - [User Handlers](#user-handlers)
      - [Requirements](#requirements)
    - [Parameters](#parameters)
    - [Primary Handler](#primary-handler)
    - [Static vs type erased (TE) Handlers](#static-vs-type-erased-te-handlers)
    - [Single Shot vs Recurring](#single-shot-vs-recurring)
  - [Provided event handlers](#provided-event-handlers)
<!--toc:end-->

Events are a lightweight signaling mechanism, tied closely to interrupts.
They aim to be equal (or close) to interrupts in terms of speed, but
optionally adding parameter passing and the ability to bind
complex function objects/functors into them, not just free functions.

## Components

The main components are:

- ``Event dispatcher``: The event dispatcher is the interface the end
application uses. It is used to set an events user handler(s) and arguments,
access the events handler, and trigger/fire events. It requires EventSpecs
and an Nvic with relocatable vector table.
- ``EventSpecs``: Compile time specifications that describe an event and how
it is to be handled. They describe which IRQ this event belongs to,
what priority this event has, and the **event handler**.
- ``Event Handler``: The **(event) handler** is interface used by the event
dispatcher to store the **user handlers** and the arguments. The
dispatcher calls the **handler** when the event is fired.
- ``User Handler``: The **user handler** is the function/function
object/callable that the main application wants to be executed when an event
is fired, i.e. the actual callback.

To achieve this, the event handling is split into two parts, namely the
**handler** / **event handler** and the **user handler**.

The **handler** is interface used by the event dispatcher and is used to
store the **user handler** and the arguments. The dispatcher calls the
**handler** when the event is fired. The
**handler** will then delegate that call to the **user handler**. As such,
the **handler** is the thing that forms the general behaviour, while the
**user handler** is merely the "subscriber" to that behaviour.

The **user handler** is the function/function object/callable that the main
application wants to be executed, i.e. the actual callback.

## Event(handlers) vs Tasks

Events and their handlers are "short lived" operations. Events are signaled,
and then processed as soon as possible by their handlers (while respecting
priorities). Event handlers are called with the arguments given when the
event was signaled. They must be short lived and run to completion,
just like ordinary/traditional interrupt handlers. Events and interrupts use
the same channels, share the same stack, and make direct use of the
given hardware interrupt controller. This means that the event ids used
must not overlap with interrupts used outside the os. But defining interrupts
outside the os is only necessary when an interrupt must be configured with a
priority higher than the maximum os priority, as raw events can be used
instead (see below).

Tasks are long running operations, own a private stack, and need an expensive
context switch to enter and exit the execution context. They are costly to
setup and tear down, and are best used for long, complex operations.

## Event Attributes

Events have three main attributes:

- ``id``: the IRQ the event belongs to. Firing/signaling/notifying an event means
activating its IRQ.
- ``priority``: the priority the event has. 0 means highest, lowest is
2^num_prio_bits - 1
- ``handler_type``: the handler type.

The first two attributes directly map onto the cortex-M NVIC. See the
exception and interrupt handling sections of your MCUs datasheet and TRM for
available ids and priorities.

The handler type controls the interrupt handling behaviour. The minimal
interface of a Handler is quite basic:

- it must be a callable which takes no arguments and returns nothing,
i.e. has the calling signature ``void()``
- it must be constructible without arguments.

There are two kinds of events/handlers: [raw](#raw-events) and [advanced](#advanced-events).

These kinds are split further into fixed and non-fixed variants. Fixed events
do not allow mutable access to their handler during runtime. Non-fixed events
allow mutable access.

## Raw Events

Raw events are plain old interrupts. The ``handler_type`` of a raw event is
convertible to ``void(*)()``, i.e. a pointer to a plain function taking no
arguments and returning nothing. This is also the only type of event where a
raw function pointer as the **handler type** is allowed. No overhead is
incurred compared to traditional interrupt handling, as the **handler** will
be put directly into the vector table. ``FixedRawEvent<Id, Priority, f>`` and
``RawEvent<Id, Priority>`` can be used as shorthand ``EventSpec`` definitions.

### Example

```{cpp}
 void my_func(){...}

 using Ev1 = RawEvent<cm::IRQ0, 0>;
 using Ev2 = FixedRawEvent<cm::IRQ2, 1, [](){ ... }>;
 using Ev3 = FixedRawEvent<cm::IRQ3, 1, my_func>;
```

## Advanced Events

Advanced event handlers can be stateful and can have parameters passed to
them. Because of this, they are not as fast as interrupts, but come quite
close. An advanced events **handler type** is a bit more complicated than a
plain function pointer. The minimal interface of an advanced events handler
type is the [primary handler](#primary-handler).

In addition to the minimal interface, providing any of the following member
functions on your handler will enable the corresponding function in the
``EventDispatcher``:

- void init();
- ``template<size_t I> void set_arg<I>(Arg&& arg)``
- ``void set_user_handler(UserHandler&&h)``

Primary handlers that feature any part of this opt-in interface are called
``Advanced Handlers``. The purpose of this interface is to enable parameter
passing, flexibility, and behaviour control.

This library provides two main handler class templates:

- ``EventHandler``: Stores a single user handler, its arguments, and invokes
it. Can be used as is, or inherited from to overlay a more advanced behaviour.
- ``BroadcastEventHandler``: Stores a list of user handlers, one set of
arguments, and  invokes all of them when called. Can be used as is, or
inherited from to overlay a more advanced behaviour. Do not use r value
reference parameters in the user handler.

### Primary Handler

Primary handlers are callables which take no arguments, return void, are are
default constructible, convertible to bool, and not convertible to
``void(*)()``,. No other semantics are required.

Example:

```cpp
 struct PrimaryHandler{
   // returns true if the PrimaryHandler can be called, like a nullptr check
   // of raw function pointers.
   operator bool()const;
   // the call operator
   void operator ()();
   // any kind of data members
   data member{};
 };

 static_assert(cm::PrimaryHandlerConcept<PrimaryHandler>);
```

The primary handler is responsible for handling the hardware level event and
propagating the necessary data to its user handler(s) and firing off other
events in response.

A concrete example would be a UART interrupt handler. These are the
assumptions made for the UART peripheral in this example:

- There is only one IRQ for the whole peripheral, IRQ15. On hardware
activity, i.e. TX/RX/errors, only one interrupt is generated, which the
UartHandler maps onto. The IRQ source, i.e. TX/RX/errors, must be queried by
flags.
- The peripheral is capable of writing a whole buffer/reading into a whole
buffer, and generating an interrupt when finished, just to keep everything
simple.
- The event priority is 2.

```cpp
enum UartError{
  ...
};

// the mmio hardware driver
struct UART{
 // mmio stuff + some utilities as member functions
 // volatile int ....
};

struct UartHandler{
  // the UART state enum
  enum State{
    uninitialized=0,
    idle = 1,
    tx   = 1<<1,
    rx   = 1<<2,
    err  = 1<<3
  };

  // user handler type for rx events
  using OnReceive = cm::FunctionRef<void(UartError , void*, int ) volatile>;
  // user handler type for tx events
  using OnTransmit = cm::FunctionRef<void(UartError , int ) volatile>;

  // the hardware driver
  volatile UART* uart;
  // rx and tx buffers
  // {
  volatile void* rx_buf;
  volatile int rx_size;
  volatile const void*tx_buf;
  volatile int tx_size;
  // }
  // user handlers
  // {
  OnReceive on_rx;
  OnTransmit on_rx;
  // }
  volatile State state = uninitialized;

  // general api init, not required by the event dispatcher, only here to
  // display functionality
  void init(...){
    ...
    state |=idle;
  }

  // general api write, not required by the event dispatcher, only here to
  // display functionality
  void async_write(const void*buf, int size, OnTransmit tx_callback){
    if(state&State::err==State::err)
      return;
    // set up transmit through driver
    uart->set_tx_buf(buf);
    uart->set_tx_size(size);
    // update state
    this->state |= tx;
    tx_buf = buf;
    tx_size = size;
    this->on_tx = tx_callback;
    // enable transmit
    uart->tx_isr_enable();
    uart->error_isr_enable();
    uart->start_transmit();
  }

  // general api read, not required by the event dispatcher, only here to
  // display functionality
  void async_read(void*buf, int size, OnReceive rx_callback){
    if(state&State::err==State::err)
      return;
    // set up transmit through driver
    uart->set_rx_buf(buf);
    uart->set_rx_size(size);
    // update state
    this->state |= rx;
    rx_buf = buf;
    rx_size = size;
    this->on_rx = rx_callback;
    // enable transmit
    uart->rx_isr_enable();
    uart->error_isr_enable();
    uart->start_receive();
  }

  // the actual function that gets called upon every isr.
  void operator()(){
    if(uart->tx_isr_flag_is_active()) {
      state &= ~State::tx;
      uart->tx_isr_disable();
      auto error = uart->tx_error();
      if (is_error(error))
        state|=error;
      // reset buffer
      tx_buf = nullptr;
      tx_size = 0;
      // calling the user handler
      if(not on_tx) // no user handler installed
        return;
      auto user_handler = std::move(on_tx); // moving the user handler onto
                                            // the stack. This allows the
                                            // current user handler to
                                            // install a new tx user handler
                                            // without overwriting its own
                                            // guts.
      user_handler(error, uart->size_written());
    }else if(uart->rx_isr_flag_is_active()) {
      state &= ~State::rx;
      uart->rx_isr_disable();
      auto error = uart->rx_error();
      if (is_error(error))
        state|=error;
      // reset buffer
      rx_buf = nullptr;
      rx_size = 0;
      if(not on_rx) // no user handler installed
        return;
      // calling the user handler
      auto user_handler = std::move(on_rx);
      user_handler(error, rx_buf, uart->size_received());
    }else{
      // error flag active
      state |= State::err;
      // disable irqs
      uart->rx_isr_disable();
      uart->tx_isr_disable();
      uart->error_isr_disable();

      if(state&tx==tx) { // was transmitting
        if(on_tx)
          std::move(on_tx)(uart->tx_err(), uart->size_written());
      }
      if(state&rx==rx) { // was transmitting
        if(on_rx)
          std::move(on_rx)(uart->rx_err(), uart->size_received());
      }
    }
  }
};
```

Once you have your handler, you an define an ``EventSpec``:

```cpp
// the event spec on IRQ15 with priority 2 and UartHandler as the event
// handler.
using UartEvent = cm::EventSpec<cm::IRQ15, 2, UartHandler>
```

The ``UartEvent`` can then be added to the ``EventDispatcher``

```cpp
// the core you are using
using MyCore = cm::Core<...>;
// the event dispatcher
using Dispatcher = cm::EventDispatcher<MyCore, UartEvent, other events...>;
```

TODO: finish primary handler section

### User Handlers

Advanced handlers (usually, but don't have to) have one or more user handlers
associated with them. User handlers are callbacks that the main application
logic wants executed. The advanced handler provides a certain behaviour to the
event, e.g. single shot/broadcast/parameters, and the user handler consumes the
behaviour.

Advanced handlers store their user handler and are responsible for calling them.
A user handlers required call signature is given by the handler.

By defining the member function ``void set_user_handler(...)``, the
user handler can be set through the ``EventDispatcher``s
``set_user_handler<EventSpec>(...)`` function.

Example: <!--TODO: user handler example-->

#### Requirements

A user handlers exact requirements depend on the handler. There are however a
few common requirements a user handler must meet, at least for the event
handlers provided by this library. For a user handler ``h`` of type ``UH``, the
following must hold:

- ``std::default_initializable<UH>``: ``UH`` is default constructible.
- ``std::convertible_to<UH, bool>``: ``h`` is convertible to ``bool`` to test
if ``h`` can be invoked.
- ``h`` must be callable without ambiguity, i.e. no templated call operators
and no ambiguous overloads.

### Parameters

Advanced handlers can have parameters associated with them. By defining the
member function  ``set_arg<I>(...)``, the arguments of a handler can be set
through the ``EventDispatcher``s ``set_arg<Event, I>(...)`` and
``set_args<Event>(...)``. This can be cumbersome, and as such the library
provides default implementations for this behaviour, namely ``EventHandler``
and ``BroadcastEventHandler``.

### Static vs type erased (TE) Handlers

Statically typed **handlers** have a fixed type as the **user handler** and
offer greater type safety, greater speed (possible inlining), and occupy less
space than the type erased alternative. What they lack is flexibility
compared to type erased **handlers**.

Type erased **handlers** can bind any **user handler**, given that the **user
handler** is callable with the arguments specified by the TE **handler** and
the **user handler** fits into the **handlers** internal buffer. This incurs
an extra indirect call and takes up a bit more space, but is much more
flexible.

### Single Shot vs Recurring

Single shot event handlers can be used when the user handler is changed out
frequently, i.e. every time the event is fired. These handlers move the user
handler onto the stack with std::move(), and then invoke it. This way, the
user handler can install a new user handler for the same event during its
execution without overwriting its own guts. After a single shot event was
handled, it is generally (read always) not safe to fire the event again
without setting a new user handler. The only possibility of it being safe-ish
is iff the event handler is statically typed and its user handler can be
called in a moved from state without causing undefined behaviour. Type erased
single shot handlers always relocate their user handler onto stack (std::move
and invoke destructor on moved from storage) and need to be reinstalled
every time the event has been handled.

Recurring event handlers can be used for complex user handlers which are
invoked multiple times without needing to be changed, for example a state
machine processing callback. These event handlers do not move the user
handler onto the stack to call them. This also means that it is not safe to
install a new handler from the currently installed handler.

## Provided event handlers

- EventHandler<bool, F,Args...>: a statically typed event handler
for the user handler F. Args... are the arguments provided to the user
handler. The boolean indicates if this is a single shot (true) or recurring
(false) handler.
- AnyEventHandler<bool, Size, Align, Args...>: a type erasing event
handler with an internal buffer of size Size and alignment Align. Accepts any
user handler it can store (the user handler must be move constructible) and
that can be called with Args....
