
from __future__ import print_function

import argparse
import logging
import os
import re
import shutil
import sys

import pyhhi.build.common.system as system
import pyhhi.build.common.util as util
import pyhhi.build.common.ver as ver
import pyhhi.build.common.bldtools as bldtools
import pyhhi.build.cmksupp as cmksupp
from pyhhi.build.common.bldtools import BuildScriptInstaller
from pyhhi.build.common.error import InvalidCommandLineArgumentError


class CMakeLauncherApp(object):

    def __init__(self):
        self._logger = logging.getLogger(__name__)
        self._sys_info = system.SystemInfo()
        self._cmake_launcher = None
        self._dict_generator_choice = {'linux': ['umake', 'ninja'],
                                       'macosx': ['xcode', 'umake', 'ninja'],
                                       'windows': ['vs16', 'vs15', 'vs14', 'vs12', 'vs11', 'vs10', 'umake', 'mgwmake', 'ninja']}
        self._top_dir = None
        self._cmake_mod_list = ['pyhhi.build.app.cmk',
                                'pyhhi.build.cmkfnd',
                                'pyhhi.build.cmksupp',
                                'pyhhi.build.common.bldtools',
                                'pyhhi.build.common.cmbldver',
                                'pyhhi.build.common.error',
                                'pyhhi.build.common.system',
                                'pyhhi.build.common.util',
                                'pyhhi.build.common.ver']

    def __call__(self):
        self.main(sys.argv[1:])

    def main(self, argv):
        # self._print_env()
        (params, cmake_argv) = self._parse_command_line(argv)
        if params.install_dir:
            top_dir = util.get_top_dir()
            os.chdir(top_dir)
            script_dir = util.get_script_dir()
            # print("sys.path: ", sys.path)
            script_installer = BuildScriptInstaller(verbose=True)
            script_installer.install_script(params.install_dir, os.path.join(script_dir, 'cmake.py'), self._cmake_mod_list)
        elif params.py_cache_dirs:
            for dname in params.py_cache_dirs:
                if not os.path.exists(dname):
                    continue
                if not os.path.isdir(dname):
                    continue
                self._remove_pycache(dname)
        else:
            # Apply additional checks on params.
            self._check_params(params)
            if self._sys_info.is_linux() or self._sys_info.is_macosx():
                self._remove_make_env_vars()
            # Delayed construction to get the current log level at construction time of CMakeLauncher.
            self._cmake_launcher = cmksupp.CMakeLauncher()
            self._cmake_launcher.launch(params, cmake_argv)

    def _check_params(self, params):
        pass

    def _print_env(self):
        env_var_list = list(os.environ.keys())
        env_var_list.sort()
        for env_var in env_var_list:
            print(env_var, os.environ[env_var])
            # print(env_var)
            # if env_var.startswith('MAKE') or (env_var == 'MFLAGS'):
            #    print(env_var)
        # sys.exit(0)

    def _remove_make_env_vars(self):
        make_env_vars = []
        for env_var in os.environ.keys():
            if env_var.startswith('MAKE') or (env_var == 'MFLAGS'):
                make_env_vars.append(env_var)
        for env_var in make_env_vars:
            self._logger.debug("deleting environment variable: %s", env_var)
            del os.environ[env_var]

    def _parse_command_line(self, argv):

        _usage = """
%(prog)s [options] [variant=debug,release,relwithdebinfo,minsizerel] [link=static,shared] [toolset=<toolset_spec>] [address-model=32]

%(prog)s is a script front end to cmake to simplify its usage on Linux,
Windows, MacOSX using cmake's generators "Unix Makefiles", "Ninja", "Xcode" and
"Visual Studio 16 - Visual Studio 10" and its compilers.

arguments:
  variant:          debug if not specified
  link:             static if not specified
  toolset:          default c++ compiler if not specified
                    examples/windows: msvc-19.1x, msvc-19.0, msvc-18.0, msvc-17.0, msvc-16.0, intel, gcc
                    examples/linux:   gcc-4.9, gcc-5, gcc-6, clang, intel
  address-model=32: windows: builds 32 bit binaries instead of 64 bit binaries

"""
        _epilog = """
usage examples:

  # debug build, create the build tree if it does not exist.
  %(prog)s -b

  # release build, create the build tree if it does not exist.
  %(prog)s -b variant=release

  # release build using shared libraries, create the build tree if it does not exist.
  %(prog)s -b variant=release link=shared

  # create a build tree without building anything
  %(prog)s

  # create a build tree specifying a cmake cache entry without building anything
  %(prog)s -DENABLE_SOMETHING=1

"""
        parser = argparse.ArgumentParser(usage=_usage, epilog=_epilog, formatter_class=argparse.RawDescriptionHelpFormatter)

        parser.add_argument("--cmake-bin-dir", action="store", dest="cmk_bin_dir",
                            help="specify a directory to search for CMake overriding the default CMake search path.")

        parser.add_argument("-g", "-G", action="store", dest="generator", choices=self._dict_generator_choice[self._sys_info.get_platform()],
                            help="""specify a cmake generator the script has special support for.
                                    Supported generators: ninja, umake, mgwmake, vs16, vs15, vs14, vs12, vs11, vs10, xcode.
                                    The choices accepted are platform and installation dependent. The environment variable
                                    DEFAULT_CMAKE_GENERATOR may be used to override the default value.""")

        parser.add_argument("-D", action="append", dest="cache_entries",
                            help="specify a cmake cache entry. The option will be ignored if a build tree already exists.")

        parser.add_argument("-W", action="append", dest="warning_flags", help="specify a cmake warning flag.")

        parser.add_argument("-b", action="store_true", dest="build", default=False,
                            help="""invokes cmake --build to build the default debug configuration unless overridden by additional arguments.
                                    If a build tree does not exist, it will be created.""")

        parser.add_argument("-j", action="store", dest="build_jobs", type=int, nargs='?', default=1,
                            const=str(self._get_optimal_number_cmake_jobs()),
                            help="""specify the number of parallel build jobs. If you omit the numeric argument,
                                    cmake --build ... will be invoked with -j%(const)s.""")

        parser.add_argument("--target", action="store", dest="build_target",
                            help="specify a build target overriding the default target.")

        parser.add_argument("--clean-first", action="store_true", dest="clean_first", default=False,
                            help="build target clean first, then build the active target.")

        parser.add_argument("--verbosity", action="store", dest="build_verbosity", choices=['cmake', 'quiet', 'minimal', 'normal', 'detailed', 'diagnostic'], default='minimal',
                            help="""specify (ms)build verbosity level [default: %(default)s]. 
                                 The choice 'cmake' requires cmake 3.14.x or higher to increase build verbosity for Visual Studio and other generators.""")

        util.app_args_add_log_level(parser)

        g = parser.add_argument_group("advanced options")
        g.add_argument("-i", action="store", dest="install_dir", nargs='?', const=os.path.join(self._sys_info.get_home_dir(native=True), 'bin'),
                       help="install this script and exit. The default destination directory is %(const)s.")

        g.add_argument("--py-cache-clean", action="store", dest="py_cache_dirs", nargs='+',
                       help="search for Python cache files in one or more directory trees, remove them and exit. This special option is intended "
                            "for cross-platform Makefiles to clean up the source tree as python cache files are stored next to the source and "
                            "not in the build tree.")

        # -j may be followed by a non-numeric argument which the parser is not able to handle.
        if '-j' in argv:
            i = argv.index('-j')
            # If the next argument is all numeric, attach it directly. In all other cases use the optimal number of parallel jobs.
            if i < (len(argv) - 1):
                if re.match(r'\d+$', argv[i + 1]):
                    argv_parsed = argv
                else:
                    # make a copy -> don't want to modify sys.argv.
                    argv_parsed = argv[:]
                    argv_parsed[i] = '-j' + str(self._get_optimal_number_cmake_jobs())
            else:
                # make a copy -> don't want to modify sys.argv.
                argv_parsed = argv[:]
                argv_parsed[i] = '-j' + str(self._get_optimal_number_cmake_jobs())
        else:
            argv_parsed = argv

        (cmake_py_options, args_left) = parser.parse_known_args(argv_parsed)

        # configure the python logger asap
        util.app_configure_logging(cmake_py_options.log_level)

        launcher_params = cmksupp.CMakeLauncherParams()
        cmake_args = []
        if cmake_py_options.install_dir:
            # print("-i found, install dir", cmake_py_options.install_dir)
            if os.path.isabs(cmake_py_options.install_dir):
                launcher_params.install_dir = cmake_py_options.install_dir
            else:
                launcher_params.install_dir = os.path.abspath(cmake_py_options.install_dir)
        elif cmake_py_options.py_cache_dirs:
            launcher_params.py_cache_dirs = []
            for dname in cmake_py_options.py_cache_dirs:
                launcher_params.py_cache_dirs.append(os.path.normpath(os.path.abspath(dname)))

        self._top_dir = os.getcwd()
        if cmake_py_options.install_dir or launcher_params.py_cache_dirs:
            return launcher_params, cmake_args

        # if args_left:
        #    print("cmake args", args_left)

        # Assign cmake.py options to attributes of CMakeLauncherParams.
        launcher_params.cmk_bin_dir = cmake_py_options.cmk_bin_dir
        launcher_params.cmk_build = cmake_py_options.build
        launcher_params.cmk_build_jobs = cmake_py_options.build_jobs
        launcher_params.clean_first = cmake_py_options.clean_first
        launcher_params.cmk_build_target = cmake_py_options.build_target
        launcher_params.cmk_build_verbosity = cmake_py_options.build_verbosity
        launcher_params.cmk_generator_alias = cmake_py_options.generator
        if cmake_py_options.cache_entries:
            launcher_params.cmk_cache_entries = ['-D' + x for x in cmake_py_options.cache_entries]
        if cmake_py_options.warning_flags:
            launcher_params.cmk_warning_flags = ['-W' + x for x in cmake_py_options.warning_flags]

        re_cmake_py_arg = re.compile(r'(toolset|variant|link|address-model)=(\S+)')
        for arg in args_left:
            # print("processing argument " + arg)
            re_match = re_cmake_py_arg.match(arg)
            if re_match:
                arg_key = re_match.group(1)
                arg_value = re_match.group(2)
                if arg_key == 'toolset':
                    if arg_value.endswith('.cmake'):
                        # A toolset=<something>.cmake expression is supposed to be a toolchain file to enable
                        # some kind of cross compilation.
                        if not os.path.exists(arg_value):
                            raise InvalidCommandLineArgumentError("toolchain file={} does not exist.".format(arg_value))
                        launcher_params.toolset_str = os.path.abspath(arg_value)
                    else:
                        launcher_params.toolset_str = self._normalize_toolset_spec(arg_value)
                elif arg_key == 'variant':
                    build_configs = arg_value.split(',')
                    for cfg in build_configs:
                        if cfg not in ['release', 'debug', 'relwithdebinfo', 'minsizerel']:
                            raise InvalidCommandLineArgumentError("argument {} is not understood.".format(arg))
                    launcher_params.build_configs = build_configs
                elif arg_key == 'link':
                    link_variants = arg_value.split(',')
                    for lnk in link_variants:
                        if lnk not in ['static', 'shared']:
                            raise InvalidCommandLineArgumentError("argument {} is not understood.".format(arg))
                    launcher_params.link_variants = link_variants
                elif arg_key == 'address-model':
                    if arg_value == '32':
                        launcher_params.target_arch = 'x86'
                    elif arg_value == '64':
                        launcher_params.target_arch = 'x86_64'
                    else:
                        raise InvalidCommandLineArgumentError("argument {} is not understood.".format(arg))
                else:
                    raise Exception("argument {} is not understood.".format(arg))
                continue
            if arg == '--':
                # Semantics seems to be tricky and I haven't seen a convincing use case yet.
                # In configuration mode all unknown arguments are passed verbatim to cmake and in build mode
                # all unknown arguments are really build tool arguments and are passed verbatim to the build tool.
                raise InvalidCommandLineArgumentError("argument '--' encountered, this is not yet supported, please contact the maintainer.")
            # all other arguments are passed on to cmake verbatim.
            cmake_args.append(arg)
        return launcher_params, cmake_args

    def _normalize_toolset_spec(self, toolset_spec):
        toolset_spec_norm = toolset_spec
        if self._sys_info.is_linux():
            if toolset_spec.find('-g++') >= 0:
                # x86_64-w64-mingw32-g++-posix -> normalized to x86_64-w64-mingw32-gcc-posix
                toolset_spec_norm = toolset_spec.replace('-g++', '-gcc')
            if toolset_spec_norm.find('-gcc') >= 0:
                # looks like a cross compiler specification which requires a toolchain file matching the toolset spec and the linux system.
                toolset_spec_norm = self._find_toolchain_file(toolset_spec_norm)
        elif self._sys_info.is_windows():
            if toolset_spec.startswith('msvc-'):
                msvc_registry = bldtools.MsvcRegistry()
                if toolset_spec == 'msvc-19.2x':
                    if msvc_registry.is_version_installed((14, 2)):
                        cl_version = msvc_registry.get_compiler_version((14, 2))
                        toolset_spec_norm = "msvc-{0:d}.{1:d}".format(cl_version[0], cl_version[1])
                    else:
                        raise InvalidCommandLineArgumentError("toolset={} not available.".format(toolset_spec))
                elif toolset_spec == 'msvc-19.1x':
                    if msvc_registry.is_version_installed((14, 1)):
                        cl_version = msvc_registry.get_compiler_version((14, 1))
                        toolset_spec_norm = "msvc-{0:d}.{1:d}".format(cl_version[0], cl_version[1])
                    else:
                        raise InvalidCommandLineArgumentError("toolset={} not available.".format(toolset_spec))
                else:
                    # msvc-19.00 -> normalized to 19.0
                    re_match = re.match(r'msvc-(\d+)\.(\d+)', toolset_spec)
                    if re_match:
                        minor_version = int(re_match.group(2))
                        toolset_spec_norm = "msvc-{0}.{1:d}".format(re_match.group(1), minor_version)
        elif self._sys_info.is_macosx():
            pass
        else:
            # unsupported platform -> contact maintainer
            assert False
        return toolset_spec_norm

    def _find_toolchain_file(self, toolset_spec):
        toolchain_file = None
        toolchains_dir = self._find_toolchains_dir()
        if toolchains_dir is not None:
            if self._sys_info.is_linux():
                toolchain_file_suffix = '-' + self._sys_info.get_os_distro_short()
                if self._sys_info.get_os_distro_short() == 'ubuntu':
                    os_version_str = ver.ubuntu_version_tuple_to_str(self._sys_info.get_os_version())
                else:
                    os_version_str = self._sys_info.get_os_version()
                toolchain_file_suffix += os_version_str.replace('.', '') + '.cmake'
                if os.path.exists(os.path.join(toolchains_dir, toolset_spec + toolchain_file_suffix)):
                    toolchain_file = os.path.join(toolchains_dir, toolset_spec + toolchain_file_suffix)
        if toolchain_file is None:
            msg = "toolset=" + toolset_spec + " cannot be mapped to a default toolchain file automatically. Please use a toolchain file or "
            msg += "contact technical support."
            raise Exception(msg)
        return toolchain_file

    def _find_toolchains_dir(self):
        toolchains_dir = os.path.join(util.get_script_dir(), '..', 'cmake', 'toolchains')
        if not os.path.exists(toolchains_dir):
            self._logger.warning("toolchains dir {} does not exist.".format(toolchains_dir))
            return None
        toolchains_dir = os.path.normpath(toolchains_dir)
        return toolchains_dir

    def _get_workspace_folder(self):
        assert self._top_dir is not None
        return self._top_dir

    def _get_optimal_number_cmake_jobs(self):
        """Returns the optimal number of cmake jobs."""
        cmake_jobs = self._sys_info.get_number_processors()
        if 'CMAKE_MAX_JOBS' in os.environ:
            cmake_max_jobs = int(os.environ['CMAKE_MAX_JOBS'], 10)
            if cmake_jobs > cmake_max_jobs:
                cmake_jobs = cmake_max_jobs
        assert cmake_jobs >= 1
        return cmake_jobs

    def _remove_pycache(self, py_dir_root):
        os.chdir(py_dir_root)
        re_pyc_file = re.compile(r'.+\.pyc$', re.IGNORECASE)
        for root, dirs, files in os.walk(py_dir_root):
            if '__pycache__' in dirs:
                self._logger.info("rm -rf {0}".format(os.path.join(root,'__pycache__')))
                shutil.rmtree(os.path.join(root, '__pycache__'))
                dirs.remove('__pycache__')
            for fname in files:
                if re_pyc_file.match(fname):
                    self._logger.info("rm {0}".format(os.path.join(root, fname)))
                    os.remove(os.path.join(root, fname))
