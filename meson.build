project('cm', 'cpp', default_options: ['cpp_std=c++20', 'warning_level=3'])

inc_dir = include_directories('include')

tl_dep = dependency(
  'type_list',
  fallback: ['type_list', 'type_list_dep'],
  default_options: ['tests=disabled'],
)

cpu = host_machine.cpu()
if cpu not in ['cortex-m7', 'cortex-m4', 'cortex-m3', 'cortex-m0', 'cortex-m0plus']
  error(
    'Unsupported cpu, expected one of: cortex-m7, cortex-m4, cortex-m3, cortex-m0, cortex-m0plus',
  )
endif

core = cpu.split('-')[1].to_upper()
sources = [
  'include/cortex/assert.hpp',
  'include/cortex/common.hpp',
  'include/cortex/core.hpp',
  'include/cortex/event.hpp',
  'include/cortex/event_dispatcher.hpp',
  'include/cortex/fpu.hpp',
  'include/cortex/function.hpp',
  'include/cortex/mpu.hpp',
  'include/cortex/nvic.hpp',
  'include/cortex/scheduler.hpp',
  'include/cortex/scs.hpp',
  'include/cortex/systick.hpp',
  'include/cortex/type_list.hpp',
]

install_headers(sources, subdir : 'cortex')

if (get_option('generate_header'))
  f= configure_file(
    input: 'include/cortex/cortex.hpp.in',
    output: 'cortex.hpp',
    configuration: {'CORE': 'cm::@0@<@1@>'.format(core, get_option('max_irq_number'))},
  )
install_headers(f, subdir : 'cortex')
  sources += f
endif

cm_dep = declare_dependency(include_directories: inc_dir, dependencies: tl_dep, sources:sources)

if(get_option('dev'))
  executable('main', 'main.cpp', include_directories: inc_dir, dependencies: tl_dep, sources:sources)
endif
