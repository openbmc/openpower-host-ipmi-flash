project(
    'openpower-host-ipmi-flash',
    'cpp',
    version: '0.1',
    meson_version: '>=1.1.1',
    default_options: [
        'werror=true',
        'warning_level=3',
        'cpp_std=c++23',
        'tests=' + (meson.is_subproject() ? 'disabled' : 'auto'),
    ],
)

# Dependencies
phosphor_logging_dep = dependency('phosphor-logging')
libsystemd = dependency('libsystemd')
sdbusplus_dep = dependency('sdbusplus', required: false, include_type: 'system')
ipmid_dep = dependency('libipmid')

# Project Arguments
cpp = meson.get_compiler('cpp')
add_project_arguments(
    cpp.get_supported_arguments(['-DBOOST_COROUTINES_NO_DEPRECATION_WARNING']),
    language: 'cpp',
)

root_inc = include_directories('.')

hiomapdir_pre = declare_dependency(
    include_directories: root_inc,
    dependencies: [phosphor_logging_dep, libsystemd, sdbusplus_dep, ipmid_dep],
)

library(
    'hiomap',
    'hiomap.cpp',
    implicit_include_directories: false,
    dependencies: hiomapdir_pre,
    version: meson.project_version(),
    override_options: ['b_lundef=false'],
    install: true,
    install_dir: get_option('libdir') / 'ipmid-providers',
)
