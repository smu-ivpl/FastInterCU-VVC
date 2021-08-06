from __future__ import print_function

import glob
import logging
import os
import platform
import re
import shutil
import subprocess
import sys
import tempfile
import textwrap

import pyhhi.build.common.system as system
import pyhhi.build.common.util as util
import pyhhi.build.common.ver as ver


class MsvcRegistry(object):

    class __MsvcRegistry(object):

        def __init__(self):
            self._logger = logging.getLogger(__name__)
            self._sys_info = system.SystemInfo()
            self._supported_msvc_versions = ['14.2', '14.1', '14.0', '12.0', '11.0', '10.0']
            program_dir = self._sys_info.get_program_dir('x86')
            # VS2019, VS2017 come with a locator tool vswhere to search for the installation directory.
            # The dictionary _msvc_install_dir_dict will be augmented with keys 14.2 and 14.1 by method _do_inventory_vc14x().
            self._msvc_install_dir_dict = {'14.0': [os.path.join(program_dir, "Microsoft Visual Studio 14.0", 'VC')],
                                           '12.0': [os.path.join(program_dir, "Microsoft Visual Studio 12.0", 'VC')],
                                           '11.0': [os.path.join(program_dir, "Microsoft Visual Studio 11.0", 'VC')],
                                           '10.0': [os.path.join(program_dir, "Microsoft Visual Studio 10.0", 'VC')]}
            # a list of sorted version tuples identifying the installed MSVC products
            self._installed_msvc_versions = []
            # key = msvc_version, value = full path of vcvarsall.bat
            self._compiler_command_dict = {}
            # key = msvc_version, value = options to be passed to the setup command; e.g. -vcvars_ver=14.0, -vcvars_ver=14.1x
            self._compiler_command_option_dict = {}
            # key = msvc_version, value = vc version
            self._compiler_version_dict = {}
            # key = msvc_version, value = True/False
            self._is_vs2017_toolset_dict = {}
            # key = msvc_version, value = True/False; e.g. '14.1' -> True indicates 14.1 is an alternative toolset installed with vs2019.
            self._is_vs2019_toolset_dict = {}
            # clear information on alternative toolset upfront
            for version in self._supported_msvc_versions:
                self._is_vs2017_toolset_dict[version] = False
                self._is_vs2019_toolset_dict[version] = False

            if self._logger.isEnabledFor(logging.DEBUG):
                self._logger.debug("performing in-depth VS inventory for debugging.")
                self._do_inventory_winreg()
            vswhere = self._find_vswhere()
            if vswhere:
                # Update VS2019 installation paths via vswhere.exe
                self._do_inventory_vc14x('14.2', vswhere)
                # Update VS2017 installation paths via vswhere.exe
                self._do_inventory_vc14x('14.1', vswhere)
            else:
                pass
            self._do_inventory()
            self._dump_inventory()

        def _dump_inventory(self):
            if self._logger.isEnabledFor(logging.DEBUG):
                for version in self._installed_msvc_versions:
                    version_str = ver.version_tuple_to_str(version)
                    cl_version_str = ver.version_tuple_to_str(self._compiler_version_dict[version_str])
                    self._logger.debug("found MSVC version {}, CL version {}, setup={}".format(version_str, cl_version_str, self._compiler_command_dict[version_str]))

        def get_compiler_command(self, version=None):
            if version is None:
                version = self.get_latest_version()
            return self._compiler_command_dict[ver.version_tuple_to_str(version)]

        def get_compiler_version(self, version=None):
            if version is None:
                version = self.get_latest_version()
            return self._compiler_version_dict[ver.version_tuple_to_str(version)]

        def get_latest_version(self, max_version=None):
            if not self._installed_msvc_versions:
                raise Exception("No supported Microsoft Visual C++ version found, please check your installation or contact technical support.")
            if max_version is None:
                max_version = ver.version_tuple_from_str(self._supported_msvc_versions[0])
            for version in self._installed_msvc_versions:
                if ver.version_compare(max_version, version) >= 0:
                    return version
            raise Exception("No suitable Microsoft Visual C++ version found, please check your installation or contact technical support.")

        def is_version_installed(self, version):
            if (ver.version_compare(version, ver.version_tuple_from_str(self._supported_msvc_versions[0])) > 0) or (ver.version_compare(version, ver.version_tuple_from_str(self._supported_msvc_versions[-1])) < 0):
                raise Exception("Microsoft Visual C++ toolset msvc-" + ver.version_tuple_to_str(version) + " is not supported.")
            for v in self._installed_msvc_versions:
                if ver.version_compare(v, version) == 0:
                    return True
            return False

        def is_vs2017_toolset(self, version):
            if not self.is_version_installed(version):
                return False
            version_str = ver.version_tuple_to_str(version)
            if version_str in self._is_vs2017_toolset_dict:
                return self._is_vs2017_toolset_dict[version_str]
            return False

        def is_vs2019_toolset(self, version):
            if not self.is_version_installed(version):
                return False
            version_str = ver.version_tuple_to_str(version)
            if version_str in self._is_vs2019_toolset_dict:
                return self._is_vs2019_toolset_dict[version_str]
            return False

        def _do_inventory(self):
            for version in self._supported_msvc_versions:
                if version not in self._msvc_install_dir_dict:
                    continue
                setup_cmd = None
                for vc_dir in self._msvc_install_dir_dict[version]:
                    self._logger.debug("trying VC install dir %s", vc_dir)
                    # multiple vc install directories per version are possible to support community and professional/enterprise editions.
                    cl_cmd = self._find_cl_cmd(vc_dir, version)
                    if cl_cmd:
                        self._logger.debug("found VC compiler %s", cl_cmd)
                        if version in ['14.2', '14.1']:
                            setup_cmd = os.path.normpath(os.path.join(os.path.dirname(cl_cmd), '..', '..', '..', '..', '..', '..', 'Auxiliary', 'Build', 'vcvarsall.bat'))
                        elif version in ['14.0']:
                            if os.path.exists(os.path.join(vc_dir, '..', 'Common7', 'IDE', 'devenv.exe')):
                                self._logger.debug("found VS 2015 IDE installed.")
                                setup_cmd = os.path.join(vc_dir, 'vcvarsall.bat')
                            elif '14.2' in self._compiler_command_dict:
                                # We've got 14.0 as an alternative VS 2019 toolset.
                                self._logger.debug("found msvc-14.0 installed as an alternative VS 2019 toolset.")
                                setup_cmd = self._compiler_command_dict['14.2']
                                self._is_vs2019_toolset_dict[version] = True
                                self._compiler_command_option_dict[version] = '-vcvars_ver=14.0'
                            elif '14.1' in self._compiler_command_dict:
                                # We've got 14.0 as an alternative VS 2017 toolset.
                                self._logger.debug("found msvc-14.0 installed as an alternative VS 2017 toolset.")
                                setup_cmd = self._compiler_command_dict['14.1']
                                self._is_vs2017_toolset_dict[version] = True
                                self._compiler_command_option_dict[version] = '-vcvars_ver=14.0'
                        else:
                            setup_cmd = os.path.join(vc_dir, 'vcvarsall.bat')
                        break
                if (setup_cmd is not None) and os.path.exists(setup_cmd):
                    assert cl_cmd is not None
                    cl_version = self._query_msvc_compiler_version(cl_cmd)
                    self._compiler_command_dict[version] = setup_cmd
                    self._compiler_version_dict[version] = cl_version
                    if (version == '14.2') and ('14.1' not in self._msvc_install_dir_dict):
                        # Search for alternative toolset vc141 installed with vs2019
                        self._logger.debug("searching for alternative VS2019 toolset vc141.")
                        vc_dir in self._msvc_install_dir_dict[version][0]
                        setup_cmd = self._compiler_command_dict['14.2']
                        cl_cmd = self._find_cl_cmd(vc_dir, '14.1')
                        if cl_cmd:
                            self._logger.debug("found alternative VC compiler {}".format(cl_cmd))
                            cl_version = self._query_msvc_compiler_version(cl_cmd)
                            self._compiler_command_dict['14.1'] = setup_cmd
                            self._compiler_version_dict['14.1'] = cl_version
                            self._compiler_command_option_dict['14.1'] = '-vcvars_ver=14.1x'
                            self._is_vs2019_toolset_dict['14.1'] = True

            msvc_version_list = []
            for version in self._compiler_version_dict:
                msvc_version_list.append(ver.version_tuple_from_str(version))
            if msvc_version_list:
                self._installed_msvc_versions = ver.version_list_sort(msvc_version_list)
                self._installed_msvc_versions.reverse()
                # print("sorted msvc versions: ", self._installed_msvc_versions)

        def _find_cl_cmd(self, vc_inst_dir, version_str):
            cl_cmd = None
            if version_str in ['14.2', '14.1']:
                msvc_dir = os.path.join(vc_inst_dir, 'Tools', 'MSVC')
                if os.path.exists(msvc_dir):
                    version_dir_list = [ver.version_tuple_from_str(x) for x in os.listdir(msvc_dir) if re.match(r'[0-9.]+$', x)]
                    if version_dir_list:
                        version_dir_list = ver.version_list_sort(version_dir_list)
                        version_dir_list.reverse()
                        # VS2019 installs toolset v141 side-by-side in a folder named '14.16.27023', toolset v142 is
                        # installed in a folder named '14.20.27508'.
                        for version in version_dir_list:
                            if (version_str == '14.2') and (version[1] >= 30):
                                self._logger.debug("ignoring cl installation folder: {}".format(os.path.join(msvc_dir, ver.version_tuple_to_str(version))))
                                continue
                            if (version_str == '14.1') and (version[1] >= 20):
                                self._logger.debug("ignoring cl installation folder: {}".format(os.path.join(msvc_dir, ver.version_tuple_to_str(version))))
                                continue
                            cl_cmd = os.path.join(msvc_dir, ver.version_tuple_to_str(version), 'bin', 'HostX64', 'x64', 'cl.exe')
                            if os.path.exists(cl_cmd):
                                break
                            else:
                                cl_cmd = None
            else:
                cl_cmd = os.path.join(vc_inst_dir, 'bin', 'amd64', 'cl.exe')
                if not os.path.exists(cl_cmd):
                    cl_cmd = None
            if cl_cmd:
                self._logger.debug("found cl: {}".format(cl_cmd))
            return cl_cmd

        def _query_msvc_compiler_version(self, cl_cmd):
            version_file_dir = tempfile.mkdtemp()
            version_file = os.path.join(version_file_dir, 'vc_version_peek.h')
            with open(version_file, "w") as versionf:
                versionf.write(textwrap.dedent("""\
                    #pragma message(_MSC_FULL_VER _MSC_BUILD)
                    """.format()))            
            retv = subprocess.check_output([cl_cmd, '/EP', version_file], stderr=self._sys_info.get_subprocess_devnull(), universal_newlines=True).lstrip()
            # print("_query_msvc_compiler_versionf(): retv=" + retv)
            re_match = re.match(r'#pragma\s+message\(([0-9][0-9])([0-9][0-9])([0-9]+)\s+([0-9]+)\)', retv)
            if re_match:
                cl_version_str = "%s.%s.%s.%s" % (re_match.group(1), re_match.group(2), re_match.group(3), re_match.group(4))
                # print("_query_msvc_compiler_cpp(): cl version: " + cl_version_str)
                cl_version = ver.version_tuple_from_str(cl_version_str)
            else:
                raise Exception("Failed to parse compiler version from " + version_file + ". Please contact technical support.")
            if os.path.exists(version_file):
                # print("temp. version file=" + version_file)
                shutil.rmtree(version_file_dir)
            return cl_version

        def _do_inventory_winreg(self):
            if int(platform.python_version_tuple()[0]) >= 3:
                import winreg
                win_registry = winreg
            else:
                import _winreg
                win_registry = _winreg

            msvc_registry_sections = ['Microsoft', 'Wow6432Node\\Microsoft']
            reg_key_joiner = "\\"
            vc_install_dir_dict = {}

            for version in self._supported_msvc_versions:
                for section in msvc_registry_sections:
                    try:
                        sub_key = reg_key_joiner.join(['Software', section, 'VisualStudio', version, 'Setup', 'VC' ])
                        #print("_find_latest_msvc(): trying registry key:" + sub_key)
                        rkey = win_registry.OpenKey(win_registry.HKEY_LOCAL_MACHINE, sub_key)
                        #print("_find_latest_msvc(): found registry key:" + sub_key)
                        vc_product_dir = win_registry.QueryValueEx(rkey, 'ProductDir')[0]
                        vc_product_dir = util.normalize_path(vc_product_dir)
                        if os.path.exists(os.path.join(vc_product_dir, 'vcvarsall.bat')):
                            # check for vcvarsall.bat won't work as its existence won't indicate a complete installation of the development product.
                            cl_cmd = self._find_cl_cmd(vc_product_dir, version)
                            if (cl_cmd is not None) and os.path.exists(cl_cmd):
                                vc_install_dir_dict[version] = util.normalize_path(vc_product_dir)
                                self._logger.debug("found VC install dir %s", vc_install_dir_dict[version])
                        win_registry.CloseKey(rkey)
                    except WindowsError:
                        continue
            return vc_install_dir_dict

        def _do_inventory_vc14x(self, msvc_version_str, vswhere=None):
            if msvc_version_str == '14.2':
                vswhere_version_expr = '[16.0,17.0)'
                vs_alias_str = 'VS2019'
            elif msvc_version_str == '14.1':
                vswhere_version_expr = '[15.0,16.0)'
                vs_alias_str = 'VS2017'
            else:
                assert False
            if vswhere is None:
                vswhere = self._find_vswhere()
            if vswhere is None:
                self._logger.debug("{0} locator vswhere.exe not found, {0} detection disabled.".format(vs_alias_str))
                return
            else:
                self._logger.debug("found {} locator: {}".format(vs_alias_str, vswhere))
            vc_dir_fnd = False
            try:
                vswhere_argv = [vswhere, '-latest']
                # vswhere_argv.extend(['-products', 'Enterprise'])
                # vswhere_argv.extend(['-products', 'Professional'])
                # vswhere_argv.extend(['-products', 'Community'])
                vswhere_argv.extend(['-products', '*'])
                vswhere_argv.extend(['-requires', 'Microsoft.VisualStudio.Component.VC.Tools.x86.x64'])
                vswhere_argv.extend(['-property', 'installationPath'])
                vswhere_argv.extend(['-version', vswhere_version_expr])
                retv = subprocess.check_output(vswhere_argv, universal_newlines=True).rstrip()
                if retv != '':
                    self._logger.debug("{} install path: {}".format(vs_alias_str, retv))
                    vc_dir = os.path.join(retv, 'VC')
                    if os.path.exists(vc_dir):
                        self._logger.debug("{} VC install path: {}".format(vs_alias_str, vc_dir))
                        self._msvc_install_dir_dict[msvc_version_str] = [vc_dir]
                        vc_dir_fnd = True
                else:
                    self._logger.debug("{} install path: <none>".format(vs_alias_str))
            except subprocess.CalledProcessError:
                self._logger.debug("{} vswhere locator call failed for some reason.".format(vs_alias_str))
            if not vc_dir_fnd:
                self._logger.debug("{0} VC not found, {0} detection disabled.".format(vs_alias_str))

        def _find_vswhere(self):
            vswhere_prog = None
            prog_dirs = [self._sys_info.get_program_dir('x86_64'), self._sys_info.get_program_dir('x86')]
            for d in prog_dirs:
                vswhere_dir = os.path.join(d, 'Microsoft Visual Studio', 'Installer')
                if os.path.exists(os.path.join(vswhere_dir, 'vswhere.exe')):
                    vswhere_prog = os.path.join(vswhere_dir, 'vswhere.exe')
                    break
            return vswhere_prog

    # the singleton as a class attribute
    instance = None

    def __init__(self):
        self._logger = logging.getLogger(__name__)
        if MsvcRegistry.instance is None:
            MsvcRegistry.instance = MsvcRegistry.__MsvcRegistry()

    def __getattr__(self, item):
        return getattr(MsvcRegistry.instance, item)


class Toolset(object):

    class PlatformInfo(object):
        def __init__(self, target_os, api_level=None):
            # e.g. windows, linux, macosx, android, iphone, iphonesimulator
            self._target_os = target_os
            self._target_os_version = None
            self._api_level = api_level
            self._isysroot = None
            # The sdk_version attribute is only available on MacOSX.
            self._sdk_version = None
            self._target_archs = []
            # Some compilers require extra flags to emit object code complying with a specific ABI or platform
            # specific SDK.
            self._target_cflags_dict = {}
            self._target_runtime_lib_dict = {}

        def get_target_os(self):
            return self._target_os

        def get_target_os_version(self):
            return self._target_os_version

        def set_target_os_version(self, version):
            self._target_os_version = version
            self.set_sdk_version(version)

        def get_sdk_version(self):
            return self._sdk_version

        def set_sdk_version(self, version):
            self._sdk_version = version

        def get_api_level(self):
            return self._api_level

        def set_api_level(self, api_level):
            self._api_level = api_level

        def get_isysroot(self):
            return self._isysroot

        def set_isysroot(self, isysroot):
            self._isysroot = isysroot

        def get_target_arch(self, index=None):
            if index is None:
                return self._target_archs
            else:
                return self._target_archs[index]

        def set_target_archs(self, target_archs):
            self._target_archs = target_archs

        def get_target_cflags(self, target_arch):
            if target_arch in self._target_cflags_dict:
                return self._target_cflags_dict[target_arch]
            else:
                return tuple()

        def set_target_cflags(self, target_arch, target_cflags):
            self._target_cflags_dict[target_arch] = target_cflags

        def get_target_runtime_libs(self, target_arch):
            if target_arch in self._target_runtime_lib_dict:
                return self._target_runtime_lib_dict[target_arch]
            else:
                return tuple()

        def set_target_runtime_libs(self, target_arch, target_runtime_libs):
            self._target_runtime_lib_dict[target_arch] = target_runtime_libs

    def __init__(self, sys_info, toolset=None, stl='default'):
        self._logger = logging.getLogger(__name__)
        self._sys_info = sys_info
        if stl == 'default':
            # only relevant for Android toolchains.
            stl = 'gnustl'
        if sys_info.is_windows():
            self._msvc_registry = MsvcRegistry()
        self._toolset = None
        self._toolset_versioned = None
        self._toolset_info_short = None
        self._is_mingw = False
        # The ndk_finder attribute helps to qualify an Android toolset and will be created on first use.
        self._ndk_finder = None
        self._version = None
        self._internal_version = None   # msvc only: version number reported by cl /?
        # A list of Platform objects
        self._platform_info = []
        self._compiler_cmd = None
        self._compiler_prefix = None
        self._compiler_tag = None
        self._boost_compiler_tag = None
        self._lib_debug_tag = 'd'
        self._lib_prefix_shared = 'lib'
        self._lib_prefix_static = 'lib'
        self._intel_search_path = []
        if sys_info.is_linux():
            self._lib_ext_shared = ('.so', '.so')
            self._lib_ext_static = '.a'
            # Add Intel compiler search path
            if os.path.exists(os.path.join('/opt', 'intel', 'bin')):
                self._intel_search_path.append(os.path.join('/opt', 'intel', 'bin'))
        elif sys_info.is_macosx():
            self._lib_ext_shared = ('.dylib', '.dylib')
            self._lib_ext_static = '.a'
            # Add Intel compiler search path
            self._intel_search_path.append(os.path.join('/usr', 'local', 'bin'))
            if os.path.exists(os.path.join('/opt', 'intel', 'bin')):
                self._intel_search_path.append(os.path.join('/opt', 'intel', 'bin'))
        elif sys_info.is_windows():
            # Assume msvc naming convention as the default on windows
            self._lib_prefix_shared = ''
            self._lib_prefix_static = 'lib'
            self._lib_ext_shared = ('.dll', '.lib')
            self._lib_ext_static = '.lib'
            # Add Intel compiler search path
            intel_inst_root = os.path.join(self._sys_info.get_program_dir('x86'), 'IntelSWTools', 'compilers_and_libraries', 'windows', 'bin')
            if os.path.exists(os.path.join(intel_inst_root, 'intel64')):
                self._intel_search_path.append(os.path.join(intel_inst_root, 'intel64'))
        else:
            assert False
        # and set the toolset attributes with real values
        self._qualify_toolset(sys_info, toolset, stl)
        self._toolset_info_short = self._create_toolset_info_short()

    def __str__(self):
        s = "toolset: %s\n" % self._toolset
        s += "toolset versioned: %s\n" % self._toolset_versioned
        if self._internal_version:
            s += "version: %s [%s]\n" % (ver.version_tuple_to_str(self._version),ver.version_tuple_to_str(self._internal_version))
        else:
            s += "version: %s\n" % ver.version_tuple_to_str(self._version)
        if self._toolset.startswith('msvc'):
            if self._msvc_registry.is_vs2017_toolset(self._version):
                s += "VS 2017 toolset!\n"
            if self._msvc_registry.is_vs2019_toolset(self._version):
                s += "VS 2019 toolset!\n"

        s += "platform(s):\n"
        for platform_info in self._platform_info:
            s += "  target os: %s\n" % platform_info.get_target_os()
            target_os_version = platform_info.get_target_os_version()
            if target_os_version:
                s += "  target os version: %s\n" % ver.version_tuple_to_str(target_os_version)
            sdk_version = platform_info.get_sdk_version()
            if sdk_version:
                s += "  sdk version: %s\n" % ver.version_tuple_to_str(sdk_version)
            s += "  target arch: %s\n" % platform_info.get_target_arch()
            for target in platform_info.get_target_arch():
                cflags_tuple = platform_info.get_target_cflags(target)
                if cflags_tuple:
                    cflags = ' '.join(cflags_tuple)
                    s += "  " + target + " => cflags: " + cflags + "\n"
                runtime_libs = platform_info.get_target_runtime_libs(target)
                for lib in runtime_libs:
                    s += "  rtl(" + target + "): " + lib + "\n"
            isysroot = platform_info.get_isysroot()
            if isysroot:
                s += "  isysroot: %s\n" % isysroot
        s += "mingw?: %s\n" % self._is_mingw
        s += "compiler prefix: %s\n" % self._compiler_prefix
        s += "compiler command: %s\n" % self._compiler_cmd
        s += "compiler tag: %s\n" % self._compiler_tag
        s += "boost compiler tag: %s\n" % self._boost_compiler_tag
        s += "lib debug tag: %s\n" % self._lib_debug_tag
        s += "lib prefix shared: %s\n" % self._lib_prefix_shared
        s += "lib prefix static: %s\n" % self._lib_prefix_static
        s += "lib extension shared: %s %s\n" % self._lib_ext_shared
        s += "lib extension static: %s\n" % self._lib_ext_static
        return s

    def get_toolset_info_short(self, target_arch=None):
        """Returns a short description of the toolset."""
        str = self._toolset_info_short
        # get the default platform
        platform_info = self.get_platform_info(0)
        if target_arch is None:
            target_arch = platform_info.get_target_arch(0)
        cflags_tuple = platform_info.get_target_cflags(target_arch)
        if cflags_tuple:
            str += '; '
            str += 'toolset flags: ' + ' '.join(cflags_tuple)
        return str

    def get_toolset(self):
        """Returns the toolset string as understood by BoostBuild without a version suffix.
        The following toolsets are supported: msvc, gcc, clang."""
        return self._toolset

    def get_version(self):
        return self._version

    def get_internal_version(self):
        return self._internal_version

    def get_toolset_versioned(self):
        """Returns the toolset string as understood by b2 with a version suffix.
        Example: msvc-11.0 or gcc-4.8"""
        return self._toolset_versioned

    def get_platform_info(self, index=None):
        """Returns a list of PlatformInfo objects or a single PlatformInfo object consisting of
        platform specific attributes.
        Example: clang may support platforms macosx, iphone and iphonesimulator."""
        if index is None:
            return self._platform_info
        else:
            return self._platform_info[index]

    def is_mingw(self):
        return self._is_mingw

    def get_compiler_prefix(self):
        return self._compiler_prefix

    def get_compiler_command(self):
        """Returns the absolute path of the c++ compiler."""
        return self._compiler_cmd

    def get_compiler_tag(self):
        return self._compiler_tag

    def get_boost_compiler_tag(self):
        return self._boost_compiler_tag

    def get_lib_debug_tag(self):
        return self._lib_debug_tag

    def get_lib_prefix_shared(self):
        return self._lib_prefix_shared

    def get_lib_prefix_static(self):
        return self._lib_prefix_static

    def get_lib_ext_shared(self):
        return self._lib_ext_shared

    def get_lib_ext_static(self):
        return self._lib_ext_static

    def _qualify_toolset(self, sys_info, toolset, stl='gnustl'):
        # toolset is either
        #  1) a toolset specification supported by Boost.Build: gcc-4.6, gcc, clang, msvc, msvc-x.y, darwin
        #  2) a relative compiler command: clang++, g++, g++-4.9, g++-5
        #  3) an absolute compiler command

        if toolset is None:
            toolset = self._get_default_toolset(sys_info)

        # The relative compiler commands clang++, clang++-3.6, g++, g++-4.9 will be translated into the corresponding
        # Boost.Build toolset specifications.
        toolset = self._normalize_toolset_spec(toolset)

        if self._qualify_android_toolset(sys_info, toolset, stl):
            pass
        else:
            # Initialize the platform_info attributes for a native toolset.
            platform_info = self.PlatformInfo(sys_info.get_platform())
            platform_info.set_target_archs([sys_info.get_os_arch()])
            # Handle msvc upfront to simplify the logic. The msvc toolset implies desktop windows for the time
            # being. Portable platforms supported by msvc are not covered yet.
            if toolset.find('msvc') >= 0:
                self._toolset = 'msvc'
                if toolset == 'msvc':
                    self._toolset_versioned = 'msvc-' + ver.version_tuple_to_str(self._msvc_registry.get_latest_version())
                else:
                    self._toolset_versioned = toolset
                re_match = re.match(r'msvc-([0-9.]+)$', self._toolset_versioned)
                if re_match:
                    # make sure this version is installed
                    if not self._msvc_registry.is_version_installed(ver.version_tuple_from_str(re_match.group(1))):
                        raise Exception("Microsoft Visual C++ toolset msvc-" + re_match.group(1) + " is not installed.")
                    # found a full product version, assume 32 and 64 bit compilers are available
                    self._version = ver.version_tuple_from_str(re_match.group(1))
                    self._compiler_cmd = self._msvc_registry.get_compiler_command(self._version)
                    if sys_info.get_os_arch() == 'x86_64':
                        platform_info.set_target_archs(['x86', 'x86_64'])
                    else:
                        platform_info.set_target_archs(['x86'])
                    self._platform_info.append(platform_info)
                    # extract msvc/cl's internal version; e.g. 18.0.40629
                    self._internal_version = self._msvc_registry.get_compiler_version(self._version)
                else:
                    raise Exception("The toolset " + toolset + " is not supported, please contact technical support.")
            else:
                if os.path.isabs(toolset):
                    self._compiler_cmd = toolset
                else:
                    if sys_info.is_macosx():
                        if toolset == 'clang':
                            # Assume an Xcode toolchain, /usr/bin/clang++ will be ignored to support side-by-side installations of different Xcode toolchains.
                            self._compiler_cmd = self._find_xcode_clang()
                        elif toolset == 'darwin':
                            self._compiler_cmd = util.find_tool_on_path('g++', True)
                        elif toolset == 'intel':
                            self._compiler_cmd = self._find_intel()
                        else:
                            raise Exception("The toolset " + toolset + " is not supported on MacOSX, please contact technical support.")
                    else:
                        # check for gcc-x.y and clang-x.y
                        re_match = re.match(r'(gcc|clang)-([0-9.]+)$', toolset)
                        if re_match:
                            # self._toolset_versioned = toolset
                            if re_match.group(1) == 'gcc':
                                self._compiler_cmd = self._find_versioned_compiler('g++', ver.version_tuple_from_str(re_match.group(2)))
                            elif re_match.group(1) == 'clang':
                                self._compiler_cmd = self._find_versioned_compiler('clang++', ver.version_tuple_from_str(re_match.group(2)))
                            else:
                                assert False
                        elif toolset == 'gcc':
                            self._compiler_cmd = util.find_tool_on_path('g++', True)
                        elif toolset == 'clang':
                            self._compiler_cmd = self._find_clang()
                        elif toolset == 'intel':
                            self._compiler_cmd = self._find_intel()
                        else:
                            # toolset is supposed to be a c++ compiler command like g++-4.7, clang++-4.5 or some prefixed cross compiler.
                            self._compiler_cmd = util.find_tool_on_path(toolset, True)
                # now guess the toolset category given the compiler command
                self._toolset = self._get_toolset_category(self._compiler_cmd)
                if self._toolset == 'clang':
                    self._version = self._get_clang_version(self._compiler_cmd)
                elif self._toolset in ['gcc', 'darwin']:
                    self._version = self._get_gcc_version(self._compiler_cmd)
                elif self._toolset == 'intel':
                    self._version = self._get_intel_version(self._compiler_cmd)
                else:
                    assert False

                # This does not work if /usr/bin/clang++ is linked to clang somewhere in the 
                # filesystem.
                #if sys_info.is_linux() or sys_info.is_macosx():
                #    self._compiler_cmd = os.path.realpath(self._compiler_cmd)
                if self._toolset == 'darwin':
                    self._platform_info.append(platform_info)
                elif self._toolset == 'clang':
                    if sys_info.is_macosx():
                        (sdk_path, sdk_version) = self._discover_macosx_sdk(platform_info.get_target_os())
                        platform_info.set_isysroot(sdk_path)
                        platform_info.set_target_os_version(sdk_version)
                        self._platform_info.append(platform_info)
                        # Add iphone platform information
                        platform_info = self.PlatformInfo('iphone')
                        platform_info.set_target_archs(['combined', 'armv7', 'arm64'])
                        (sdk_path, sdk_version) = self._discover_macosx_sdk(platform_info.get_target_os())
                        platform_info.set_isysroot(sdk_path)
                        platform_info.set_target_os_version(sdk_version)
                        self._platform_info.append(platform_info)
                        # Add iphonesimulator platform information
                        platform_info = self.PlatformInfo('iphonesimulator')
                        platform_info.set_target_archs(['combined', 'x86_64', 'x86'])
                        (sdk_path, sdk_version) = self._discover_macosx_sdk(platform_info.get_target_os())
                        platform_info.set_isysroot(sdk_path)
                        platform_info.set_target_os_version(sdk_version)
                        self._platform_info.append(platform_info)
                    elif sys_info.is_linux():
                        self._platform_info.append(platform_info)
                    else:
                        assert False
                elif self._toolset == 'gcc':
                    # find out the compiler prefix if any.
                    self._compiler_prefix = self._get_compiler_prefix(self._compiler_cmd)
                    gcc_machine = self._get_machine(self._compiler_cmd)
                    # analyze the compiler's machine string to understand what kind of cross compiler we've got.
                    # e.g. ubuntu: x86_64-w64-mingw32 or i686-w64-mingw32
                    #      windows: x86_64-w64-mingw32
                    #               mingw32: mingw32
                    if re.search('mingw', gcc_machine):
                        self._is_mingw = True
                        platform_info = self.PlatformInfo('windows')
                        if re.match('x86_64-', gcc_machine):
                            if sys_info.is_windows():
                                # assume the 64 bit compiler can do 32 bit as well.
                                platform_info.set_target_archs(['x86_64', 'x86'])
                            else:
                                platform_info.set_target_archs(['x86_64'])
                        else:
                            platform_info.set_target_archs(['x86'])
                    elif re.match('arm-', gcc_machine):
                        platform_info.set_target_archs(['arm'])
                    elif re.match('aarch64-', gcc_machine):
                        platform_info.set_target_archs(['aarch64'])
                    self._platform_info.append(platform_info)
                elif self._toolset == 'intel':
                    if sys_info.is_linux():
                        self._platform_info.append(platform_info)
                    elif sys_info.is_macosx():
                        (sdk_path, sdk_version) = self._discover_macosx_sdk(platform_info.get_target_os())
                        platform_info.set_isysroot(sdk_path)
                        platform_info.set_target_os_version(sdk_version)
                        self._platform_info.append(platform_info)
                    elif sys_info.is_windows():
                        #platform_info.set_target_archs(['x86_64', 'x86'])
                        self._platform_info.append(platform_info)
                    else:
                        raise Exception("The toolset " + self._toolset + " is not supported on this platform yet, please contact technical support.")
                else:
                    assert False

            # fill in the compiler taqs
            if self._toolset == 'msvc':
                self._compiler_tag = self._toolset_versioned
                self._boost_compiler_tag = 'vc' + str(self._version[0]) + str(self._version[1])
                self._lib_debug_tag = 'gd'
            elif self._toolset == 'darwin':
                self._compiler_tag = 'gcc-' + ver.version_tuple_to_str(self._version[:2])
                self._boost_compiler_tag = 'xgcc' + str(self._version[0]) + str(self._version[1])
            elif self._toolset == 'clang':
                self._compiler_tag = 'clang-' + ver.version_tuple_to_str(self._version[:2])
                if sys_info.is_macosx():
                    # The boost libraries are tagged with the version of the gcc backend and not the
                    # clang version.
                    gcc_version = self._get_gcc_version(self._compiler_cmd)
                    self._boost_compiler_tag = 'clang-darwin' + str(gcc_version[0]) + str(gcc_version[1])
                elif sys_info.is_linux():
                    self._boost_compiler_tag = 'clang' + str(self._version[0]) + str(self._version[1])
                else:
                    assert False
            elif self._toolset == 'gcc':
                if self._is_mingw:
                    self._compiler_tag = 'gcc-mingw-' + ver.version_tuple_to_str(self._version[:2])
                    self._boost_compiler_tag = 'mgw' + str(self._version[0]) + str(self._version[1])
                    self._lib_prefix_shared = 'lib'
                    self._lib_prefix_static = 'lib'
                    self._lib_ext_shared = ('.dll', '.dll.a')
                    self._lib_ext_static = '.a'
                    self._add_mingw_runtime_lib_info(sys_info)
                else:
                    self._compiler_tag = 'gcc-' + ver.version_tuple_to_str(self._version[:2])
                    self._boost_compiler_tag = 'gcc' + str(self._version[0]) + str(self._version[1])
            elif self._toolset == 'intel':
                self._compiler_tag = 'intel-' + ver.version_tuple_to_str(self._version[:2])
                if sys_info.is_linux() or sys_info.is_macosx():
                    self._boost_compiler_tag = 'il' + str(self._version[0]) + str(self._version[1])
                elif sys_info.is_windows():
                    self._lib_debug_tag = 'gd'
                    self._boost_compiler_tag = 'iw'
                else:
                    assert False
            else:
                assert False

    def _normalize_toolset_spec(self, toolset):
        self._logger.debug("entering toolset=%s", toolset)
        # normalize a toolset specification like g++-4.9 to gcc-4.9 as understood by Boost.Build.
        if self._sys_info.is_macosx():
            # deals with cmake which seems to detect c++ rather than clang++ on macosx. For some reason /usr/bin/c++ is linked to clang++
            # and the same is true for the Xcode path /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++.
            re_match = re.match(r'(clang|[cg])\+\+(-[0-9.]+)?', toolset, re.IGNORECASE)
        else:
            re_match = re.match(r'(clang|g)\+\+(-[0-9.]+)?', toolset, re.IGNORECASE)
        if re_match:
            toolset_norm = re_match.group(1)
            if toolset_norm == 'g':
                toolset_norm = 'gcc'
            elif toolset_norm == 'c':
                toolset_norm = 'clang'
            if re_match.group(2):
                toolset_norm += re_match.group(2)
        else:
            toolset_norm = toolset
        self._logger.debug("returning toolset=%s", toolset_norm)
        return toolset_norm

    def _get_toolset_category(self, compiler_cmd):
        assert os.path.isabs(compiler_cmd)
        basename = os.path.basename(compiler_cmd)
        if (basename.find('clang') >= 0) or (self._sys_info.is_macosx() and (basename == 'c++')):
            # deals with cmake which seems to detect c++ rather than clang++ on macosx.
            toolset_category = 'clang'
        elif (basename == 'icpc') or (basename == 'icl.exe'):
            toolset_category = 'intel'
        else:
            if self._sys_info.is_macosx():
                if basename in ['g++', 'gcc']:
                    toolset_category = 'darwin'
                else:
                    toolset_category = 'gcc'
            else:
                toolset_category = 'gcc'
        return toolset_category

    def _add_mingw_runtime_lib_info(self, sys_info):
        if not sys_info.is_linux():
            return
        # The LINUX mingw compilers are configured to link to the shared runtime by default, which has to be deployed to
        # the target system.
        if os.path.dirname(self.get_compiler_command()) != '/usr/bin':
            # Ignore this mingw compiler, it might be user private.
            return
        platform_info = self.get_platform_info(0)
        target_arch = platform_info.get_target_arch(0)
        compiler_prefix = self.get_compiler_prefix()
        compiler_version = ver.version_tuple_to_str(self.get_version()[:2])
        re_dll_names = re.compile(r'((libstdc.*)|(libgcc_s_.*)|(libwinpthread.*))\.dll$', re.IGNORECASE)
        dir_list = []
        if sys_info.get_os_distro() == 'ubuntu':
            # Ubuntu 15.04
            # /usr/lib/gcc/x86_64-w64-mingw32/4.9-posix/libstdc++-6.dll
            # /usr/lib/gcc/x86_64-w64-mingw32/4.9-posix/libgcc_s_seh-1.dll
            # /usr/x86_64-w64-mingw32/lib/libwinpthread-1.dll
            #
            # Ubuntu 14.04
            # /usr/lib/gcc/x86_64-w64-mingw32/4.8/libstdc++-6.dll
            # /usr/lib/gcc/x86_64-w64-mingw32/4.8/libgcc_s_sjlj-1.dll
            # /usr/x86_64-w64-mingw32/lib/libwinpthread-1.dll
            #
            thread_model = self._get_gcc_thread_model(self.get_compiler_command())
            if thread_model == 'win32':
                # Assume thread model win32 without a dependency on libwinpthread.
                dir_list = [os.path.join('/usr/lib/gcc', compiler_prefix, compiler_version + '-win32')]
            else:
                # Assume thread model posix.
                dir_list = [os.path.join('/usr/lib/gcc', compiler_prefix, compiler_version),
                            os.path.join('/usr/lib/gcc', compiler_prefix, compiler_version + '-posix'),
                            os.path.join('/usr', compiler_prefix, 'lib')]
        elif sys_info.is_redhat():
            retv = subprocess.check_output([self.get_compiler_command(), '-print-sysroot'], universal_newlines=True)
            sys_root = retv.lstrip().rstrip()
            if (len(sys_root) > 0) and os.path.exists(sys_root):
                dir_list = [os.path.join(sys_root, 'mingw', 'bin')]
        dll_list = []
        for dir in dir_list:
            if os.path.exists(dir):
                file_list = [os.path.join(dir, f) for f in os.listdir(dir) if re_dll_names.match(f)]
                if file_list:
                    dll_list.extend(file_list)
        if dll_list:
            #print('_add_mingw_runtime_lib_info', dll_list)
            platform_info.set_target_runtime_libs(target_arch, tuple(dll_list))

    def _get_default_toolset(self, sys_info):
        toolset = None

        if 'BJAM_TOOLSET' in os.environ:
            # A power users can override the platform dependent default toolset via his environment.
            #
            # e.g. macosx:  BJAM_TOOLSET=clang
            #      windows: BJAM_TOOLSET=msvc-10.0
            #
            toolset = os.environ['BJAM_TOOLSET']
        else:
            if sys_info.is_linux():
                toolset = 'gcc'
            elif sys_info.is_macosx():
                #toolset = 'darwin'
                toolset = 'clang'
            elif sys_info.is_windows():
                # the default toolset is the latest msvc.
                toolset = 'msvc-' + ver.version_tuple_to_str(self._msvc_registry.get_latest_version())
            else:
                assert False
        return toolset

    def _get_gcc_version(self, gcc_cmd):
        # gcc -dumpversion may just emit the major version (g++-7/ubuntu 17.10)
        retv = subprocess.check_output([gcc_cmd, '-E', '-dM', '-x', 'c++', os.devnull], universal_newlines=True)
        lines = retv.splitlines()
        gcc_version_list = [0, 0, 0]
        re_gcc_version = re.compile(r'#define\s+(__GNUC__|__GNUC_MINOR__|__GNUC_PATCHLEVEL__)\s+(\d+)')
        for l in lines:
            re_match = re_gcc_version.match(l)
            if re_match:
                if re_match.group(1) == '__GNUC__':
                    gcc_version_list[0] = int(re_match.group(2), 10)
                    self._logger.debug("found gcc major %d", gcc_version_list[0])
                elif re_match.group(1) == '__GNUC_MINOR__':
                    gcc_version_list[1] = int(re_match.group(2), 10)
                    self._logger.debug("found gcc minor %d", gcc_version_list[1])
                elif re_match.group(1) == '__GNUC_PATCHLEVEL__':
                    gcc_version_list[2] = int(re_match.group(2), 10)
                    self._logger.debug("found gcc patch level %d", gcc_version_list[2])
        version = tuple(gcc_version_list)
        return version

    def _get_gcc_thread_model(self, gcc_cmd):
        thread_model = 'posix'
        retv = subprocess.check_output([gcc_cmd, '-v'], stderr=subprocess.STDOUT, universal_newlines=True)
        lines = retv.splitlines()
        re_thread_model = re.compile(r'Thread model:\s*(\S+)', re.IGNORECASE)
        for l in lines:
            re_match = re_thread_model.match(l)
            if re_match:
                thread_model = re_match.group(1)
        return thread_model

    def _get_clang_version(self, clang_cmd):
        # ubuntu: Ubuntu clang version 3.0-6ubuntu3 (tags/RELEASE_30/final) (based on LLVM 3.0)
        # macosx:
        retv = subprocess.check_output([clang_cmd, '--version'], universal_newlines=True)
        version_response = retv.rstrip()
        lines = version_response.splitlines()
        re_version_match = re.search(r'\s+version\s+([0-9]+\.[0-9]+)', lines[0], re.IGNORECASE)
        if not re_version_match:
            raise Exception("The clang compiler has returned an unsupported version string. Please contact technical support.")
        return ver.version_tuple_from_str(re_version_match.group(1))

    def _get_intel_version(self, icpc_cmd):
        if self._sys_info.is_windows():
            re_version_match = None
            # stderr=subprocess.STDOUT:
            retv = subprocess.check_output([icpc_cmd, '/?'], stderr=subprocess.STDOUT, universal_newlines=True)
            version_response = retv.rstrip()
            lines = version_response.splitlines()
            re_version_match = re.search(r'\s+Version\s+([0-9.]+)', lines[0], re.IGNORECASE)
        else:
            retv = subprocess.check_output([icpc_cmd, '--version'], universal_newlines=True)
            version_response = retv.rstrip()
            lines = version_response.splitlines()
            re_version_match = re.match(r'^icpc\s+[^0-9.]+\s+([0-9.]+)', lines[0], re.IGNORECASE)
        if not re_version_match:
            raise Exception("The intel compiler has returned an unsupported version string. Please contact technical support.")
        return ver.version_tuple_from_str(re_version_match.group(1))

    def _get_machine(self, gcc_cmd):
        retv = subprocess.check_output([gcc_cmd, '-dumpmachine'], universal_newlines=True)
        return retv.lower()

    def _get_compiler_prefix(self, compiler_cmd):
        # same for gcc and clang
        if os.path.isabs(compiler_cmd):
            compiler_cmd = os.path.basename(compiler_cmd)
        # arm-linux-gnueabihf-g++-4.8
        # arm-linux-gnueabihf-g++
        # arm-linux-androideabi-g++
        # arm-linux-androideabi-g++-4.9
        # i686-w64-mingw32-gcc-4.8
        # i686-w64-mingw32-gcc
        # arm-linux-androideabi-clang
        compiler_cmd_parts = compiler_cmd.split('-')
        if len(compiler_cmd_parts) < 4:
            return None
        else:
            return '-'.join(compiler_cmd_parts[:3])

    def _create_toolset_info_short(self):
        toolset_info = None
        if self._toolset == 'msvc':
            toolset_info = self._toolset + '-' + ver.version_tuple_to_str(self._version)
            if self._internal_version:
                toolset_info += ' [' + ver.version_tuple_to_str(self._internal_version) + ']'
        elif self._toolset in ['clang', 'darwin']:
            toolset_info = self._toolset + ' ' + ver.version_tuple_to_str(self._version)
        elif self._toolset == 'gcc':
            toolset_info = 'gcc ' + ver.version_tuple_to_str(self._version)
            if self._compiler_cmd != 'g++':
                toolset_info += ' using ' + self._compiler_cmd
        elif self._toolset == 'intel':
            toolset_info = 'intel ' + ver.version_tuple_to_str(self._version)
        else:
            assert False
        return toolset_info

    def _find_xcode_clang(self):
        retv = subprocess.check_output(['xcrun', '--sdk', 'macosx', '--find', 'clang++'], universal_newlines=True)
        clang_cmd = retv.lstrip().rstrip()
        if not os.path.exists(clang_cmd):
            raise Exception("Discovery of Xcode clang failed, please contact technical support.")
        return clang_cmd

    def _find_clang(self):
        if self._sys_info.is_linux():
            clang_cmd = util.find_tool_on_path('clang++')
            if clang_cmd is None:
                # Try /usr/bin/clang++-x.y as a fallback, ubuntu clang packages may not add a link to the default version.
                clang_cmds = glob.glob('/usr/bin/clang++-[0-9]*')
                if clang_cmds:
                    # select the highest version.
                    re_clang_version = re.compile(r'.*\+\+-([0-9.]+)$')
                    version_list = []
                    for clang in clang_cmds:
                        re_match = re_clang_version.match(clang)
                        if re_match:
                            version_list.append(ver.version_tuple_from_str(re_match.group(1)))
                    version_list = ver.version_list_sort(version_list)
                    clang_cmd = '/usr/bin/clang++-' + ver.version_tuple_to_str(version_list[-1])
        else:
            clang_cmd = util.find_tool_on_path('clang++')
        if clang_cmd is None:
            raise Exception("Discovery of clang++ failed, please check your installation or contact technical support.")
        return clang_cmd

    def _find_intel(self):
        icpc_cmd = None
        if self._intel_search_path:
            if self._sys_info.is_linux() or self._sys_info.is_macosx():
                icpc_cmd = util.find_tool_on_path('icpc', search_path=self._intel_search_path)
            elif self._sys_info.is_windows():
                icpc_cmd = util.find_tool_on_path('icl.exe', search_path=self._intel_search_path)
            else:
                assert False
        if icpc_cmd is None:
            raise Exception("Discovery of Intel compiler failed, please check your installation or contact technical support.")
        return icpc_cmd

    def _find_versioned_compiler(self, compiler_prog, toolset_version):
        """Find versioned compiler on the path."""
        self._logger.debug("entering: compiler=%s %s", compiler_prog, ver.version_tuple_to_str(toolset_version))
        compiler_cmd = util.find_tool_on_path(compiler_prog + '-' + ver.version_tuple_to_str(toolset_version))
        if compiler_cmd:
            self._logger.debug("returning: %s", compiler_cmd)
            return compiler_cmd
        # Starting with g++ 5.x ubuntu systems use g++-5 instead of g++-<major>.<minor>.
        compiler_cmd = util.find_tool_on_path(compiler_prog + '-' + str(toolset_version[0]))
        if compiler_cmd is None:
            # Search for the compiler without a version suffix but make sure its version matches the toolset version
            compiler_cmd = util.find_tool_on_path(compiler_prog, True)
        self._logger.debug("con't validating version of %s, expected: %s", compiler_cmd, ver.version_tuple_to_str(toolset_version))
        version = None
        if compiler_prog.startswith('g++'):
            version = self._get_gcc_version(compiler_cmd)
        elif compiler_prog.startswith('clang++'):
            version = self._get_clang_version(compiler_cmd)
        else:
            assert False
        if version:
            assert len(version) >= 2
            if ver.version_compare(version[:2], toolset_version) != 0:
                raise Exception("Search for compiler command " + compiler_prog + " version " + ver.version_tuple_to_str(toolset_version) + " failed, please contact technical support.")
        self._logger.debug("returning: %s", compiler_cmd)
        return compiler_cmd

    def _discover_macosx_sdk(self, target_os):
        sdk_version = None
        if target_os == 'macosx':
            sdk = target_os
        elif target_os == 'iphone':
            sdk = 'iphoneos'
        elif target_os == 'iphonesimulator':
            sdk = target_os
        else:
            assert False
        retv = subprocess.check_output(['xcrun', '--sdk', sdk, '--show-sdk-path'], universal_newlines=True)
        sdk_path = retv.lstrip().rstrip()
        if not os.path.exists(sdk_path):
            raise Exception("The location of the platform SDK cannot be discovered, please contact technical support.")
        sdk_basename = os.path.basename(sdk_path)
        re_match = re.match(r'[^0-9]+(\d+\.\d+)\.sdk$', sdk_basename, re.IGNORECASE)
        if re_match:
            sdk_version = ver.version_tuple_from_str(re_match.group(1))
        return (sdk_path, sdk_version)

    def _qualify_android_toolset(self, sys_info, toolset, stl='gnustl'):
        if sys_info.is_windows():
            # No Android support on windows yet.
            return False
        # All android cross compilers are supposed to be named '[path]*android*'.
        if os.path.isabs(toolset):
            toolset_cmd = os.path.basename(toolset)
        else:
            toolset_cmd = toolset
        if toolset_cmd.find('android') < 0:
            return False

        if os.path.isabs(toolset):
            toolset_cmd = toolset
        else:
            # toolset is supposed to be a prefixed g++ or possibly clang++ command.
            toolset_cmd = util.find_tool_on_path(toolset)
            if toolset_cmd is None:
                # analyze toolset and search for -android or -androideabi
                toolset_prefix_list = toolset.split('-')
                if len(toolset_prefix_list) < 4:
                    return False
                if re.match('android', toolset_prefix_list[2]):
                    # android or androideabi
                    toolchain_prefix = '-'.join(toolset_prefix_list[:3])
                    ndk_finder = self._get_ndk_finder()
                    toolset_cmd = os.path.join(ndk_finder.get_ndksa_root(stl), toolchain_prefix, 'bin', toolset)
                    assert os.path.exists(toolset_cmd)
                else:
                    return False

        toolset_machine = self._get_machine(toolset_cmd)
        if re.search('-android', toolset_machine):
            toolset_machine_parts = toolset_machine.split('-')
            if len(toolset_machine_parts) < 3:
                return False
            # Looks like this toolset is really a supported Android toolset, initialize all attributes.
            self._compiler_cmd = toolset_cmd

            self._compiler_prefix = self._get_compiler_prefix(toolset_cmd)
            if re.match(r'.*((clang)|(clang\+\+))$', toolset_cmd):
                self._toolset = 'clang'
            else:
                self._toolset = 'gcc'

            platform_info = self.PlatformInfo('android', self._get_android_api_level(toolset_cmd))
            if re.match('(armv5te)|(armv7a)|(arm)$', toolset_machine_parts[0]):
                #armeabi armeabi-v7a armeabi-v7a-hard
                platform_info.set_target_archs(['armeabi-v7a', 'armeabi'])
                platform_info.set_target_cflags('armeabi-v7a', tuple(['-march=armv7-a', '-mfloat-abi=softfp', '-mfpu=vfpv3-d16']))
            elif toolset_machine_parts[0] == 'aarch64':
                platform_info.set_target_archs(['arm64-v8a'])
            elif toolset_machine_parts[0] == 'i686':
                platform_info.set_target_archs(['x86'])
            elif toolset_machine_parts[0] == 'x86_64':
                platform_info.set_target_archs(['x86_64'])
            else:
                assert False

            self._platform_info.append(platform_info)

            if self._toolset == 'gcc':
                self._version = self._get_gcc_version(toolset_cmd)
                self._compiler_tag = 'gcc-' + ver.version_tuple_to_str(self._version[:2])
                self._boost_compiler_tag = 'gcc' + str(self._version[0]) + str(self._version[1])
            elif self._toolset == 'clang':
                self._version = self._get_clang_version(toolset_cmd)
                self._compiler_tag = 'clang-' + ver.version_tuple_to_str(self._version[:2])
                if sys_info.is_macosx():
                    # The boost libraries are tagged with the version of the gcc backend and not the
                    # clang version.
                    gcc_version = self._get_gcc_version(self._compiler_cmd)
                    self._boost_compiler_tag = 'clang-darwin' + str(gcc_version[0]) + str(gcc_version[1])
                else:
                    self._boost_compiler_tag = 'clang' + str(self._version[0]) + str(self._version[1])
            else:
                assert False
            return True
        else:
            assert False

    def _get_ndk_finder(self):
        if self._ndk_finder is None:
            import pyhhi.build.common.android
            self._ndk_finder = pyhhi.build.common.android.NdkFinder()
        return self._ndk_finder

    def _get_android_api_level(self, toolset_cmd):
        # toolset_cmd is supposed to be an absolute path to be able to support different API levels and
        # architectures. If the toolset command refers to a standalone toolchain, there is a single api-level header and
        # the API level can be extracted unless unified headers are used.
        # If the toolset command refers to the NDK compiler (->QtCreator), the latest API level will be
        # taken as default.
        api_level_header = os.path.join(os.path.dirname(toolset_cmd), '..', 'sysroot', 'usr', 'include', 'android', 'api-level.h')
        if os.path.exists(api_level_header):
            # This looks like a standalone toolchain installation.
            # #define __ANDROID_API__ 21
            re_api_define = re.compile(r'^#define\s+__ANDROID_API__\s+(\d+)')
            with open(api_level_header) as fin:
                for line in fin:
                    re_match = re_api_define.match(line)
                    if re_match:
                        api_level = int(re_match.group(1), 10)
                        return api_level
            # Most likely NDK r15 with unified headers, the API level is now defined in arm-linux-androideabi-clang++.
            ndk_finder = self._get_ndk_finder()
            ndk_version = ndk_finder.get_ndk_version()
            if ver.version_compare(ndk_version, (15, 0)) >= 0:
                return self._get_android_api_level_unified(toolset_cmd)
            raise Exception("The Android API level cannot be extracted from  " + api_level_header + ". Please contact technical support.")
        else:
            # Assume the original NDK toolchain with multiple platform/API level support
            ndk_finder = self._get_ndk_finder()
            ndk_platforms = ndk_finder.get_ndk_platforms()
            re_match = re.match(r'android-(\d+)', ndk_platforms[-1])
            if re_match:
                api_level = int(re_match.group(1), 10)
            else:
                raise Exception("The Android API level cannot be determined, please contact technical support.")
        return api_level

    def _get_android_api_level_unified(self, toolset_cmd):
        if toolset_cmd.endswith('clang++') or toolset_cmd.endswith('clang'):
            # -D__ANDROID_API__=26
            re_api_level_def = re.compile(r'\s+-D__ANDROID_API__=(\d+)')
            with open(toolset_cmd) as fin:
                for line in fin:
                    re_match = re_api_level_def.search(line)
                    if re_match:
                        api_level = int(re_match.group(1), 10)
                        return api_level
        raise Exception("The Android API level cannot be determined. Please contact technical support.")


class FatBinaryTool(object):

    def __init__(self):
        # search for lipo and raise an Exception if the command cannot be found.
        self._lipo_cmd = util.find_tool_on_path('lipo', True)

    def createLibs(self, src_lib_dirs, dst_lib_dir, incremental=True):
        if len(src_lib_dirs) < 2:
            raise Exception("A universal library requires at least two different input formats.")
        for d in src_lib_dirs:
            if not os.path.exists(d):
                raise Exception("The directory " + d + " does not exist.")
        if not os.path.exists(dst_lib_dir):
            os.makedirs(dst_lib_dir)
        lib_dir = src_lib_dirs[0]
        lib_names = [d for d in os.listdir(lib_dir) if (d.endswith('.a') or d.endswith('.dylib'))]
        for lib in lib_names:
            src_mtime = 0
            src_libs = []
            for d in src_lib_dirs:
                src_lib = os.path.join(d, lib)
                if os.path.exists(src_lib):
                    mtime = os.path.getmtime(src_lib)
                    if mtime > src_mtime:
                        src_mtime = mtime
                    src_libs.append(src_lib)
            if len(src_libs) >= 2:
                lipo_argv = [self._lipo_cmd, '-create']
                lipo_argv.extend(src_libs)
                dst_lib = os.path.join(dst_lib_dir, lib)
                lipo_argv.extend(['-output', dst_lib])
                build_universal = True
                if os.path.exists(dst_lib):
                    mtime = os.path.getmtime(dst_lib)
                    if incremental and (mtime > src_mtime):
                        build_universal = False
                if build_universal:
                    # and launch lipo to create the universal binary file.
                    cmd_line = ' '.join(lipo_argv)
                    print("Launching: " + cmd_line)
                    retv = util.subproc_call_flushed(lipo_argv)
                    if retv != 0:
                        raise Exception("Creating a universal file failed -> " + cmd_line)
            else:
                raise Exception("error: creating a univeral file requires at least two input files, check " + lib)


class DyLibInstallNameInfo(object):

    def __init__(self, dylib):
        assert os.path.exists(dylib)
        self.basename = os.path.basename(dylib)
        self.inst_dir = os.path.dirname(dylib)
        self.inst_name = None
        self.depends_list = []
        self.rpath_list = []


class DyLibInstallNameInfoInspector(object):

    def __init__(self, ignore_system_libs=True):
        self._ignore_system_libs = ignore_system_libs
        self._inst_name_info_cache = {}

    def create_install_name_info(self, dylib):
        assert os.path.exists(dylib)
        if not os.path.isabs(dylib):
            dylib = os.path.abspath(dylib)
        if dylib not in self._inst_name_info_cache:
            inst_name_info = DyLibInstallNameInfo(dylib)
            inst_name_info.inst_name = self.get_install_name(dylib)
            (depends_list, rpath_list) = self.get_depends_list(dylib)
            inst_name_info.depends_list = depends_list
            inst_name_info.rpath_list = rpath_list
            self._inst_name_info_cache[dylib] = inst_name_info
        else:
            inst_name_info = self._inst_name_info_cache[dylib]
        return inst_name_info

    def get_install_name(self, dylib):
        assert os.path.exists(dylib)
        lines = subprocess.check_output(['otool', '-D', dylib], universal_newlines=True).rstrip().splitlines()
        return lines[1]

    def get_depends_list(self, dylib):
        assert os.path.exists(dylib)
        depends_list = []
        rpath_list = []
        load_cmd_lines = subprocess.check_output(['otool', '-l', dylib], universal_newlines=True).rstrip().splitlines()

        re_depends_path = re.compile(r'\s*name\s+([^ ]+)')
        re_rpath = re.compile(r'\s*path\s+([^ ]+)')

        # otool -L dylib ->
        # lib/clang-4.2/x86_64/plugins/debug/codec/libmpg4nulldec_plugin.dylib:
        #   libmpg4nulldec_plugin.dylib (compatibility version 0.0.0, current version 0.0.0)
        #   @loader_path/lib/libvlccore.5.dylib (compatibility version 7.0.0, current version 7.0.0)
        #   /usr/lib/libstdc++.6.dylib (compatibility version 7.0.0, current version 56.0.0)
        #   /usr/lib/libSystem.B.dylib (compatibility version 1.0.0, current version 169.3.0)
        #
        # --
        # otool -l dylib ->
        #           cmd LC_RPATH
        #           cmdsize 80
        #           path /Users/rauthenberg/projects/VlcPluginSampleLib/lib/clang-6.1/x86_64 (offset 12)
        #
        #           cmd LC_LOAD_DYLIB
        #           cmdsize 48
        #           name /usr/lib/libc++.1.dylib (offset 24)
        #
        re_lc_rpath = re.compile(r'\s*cmd\s+LC_RPATH')
        re_lc_load_dylib = re.compile(r'\s*cmd\s+LC_LOAD_DYLIB')
        line_cnt = 0
        while line_cnt < len(load_cmd_lines):
            line = load_cmd_lines[line_cnt].lstrip()
            if re_lc_rpath.match(line):
                re_match = re_rpath.match(load_cmd_lines[line_cnt + 2])
                if re_match:
                    rpath_list.append(re_match.group(1))
                line_cnt += 3
            elif re_lc_load_dylib.match(line):
                re_match = re_depends_path.match(load_cmd_lines[line_cnt + 2])
                if re_match:
                    depends_path = re_match.group(1)
                    if self._ignore_system_libs:
                        if not depends_path.startswith(('/System/', '/Library/', '/usr/lib/')):
                            depends_list.append(depends_path)
                    else:
                        depends_list.append(depends_path)
                line_cnt += 3
            else:
                line_cnt += 1

        return (depends_list, rpath_list)

    def modify_depends(self, dylib, depends_dict):
        argv = ['install_name_tool']
        for key in depends_dict:
            argv.append('-change')
            argv.append(key)
            argv.append(depends_dict[key])
        argv.append(dylib)
        #print("modify_depends(): ", argv)
        util.subproc_check_call_flushed(argv)
    
    def change_rpaths(self, dylib, rpaths_dict):
        argv = ['install_name_tool']
        for key in rpaths_dict:
            argv.append('-rpath')
            argv.append(key)
            argv.append(rpaths_dict[key])
        argv.append(dylib)
        #print("change_rpaths(): ", argv)
        util.subproc_check_call_flushed(argv)

    def delete_rpaths(self, dylib, rpath_list):
        argv = ['install_name_tool']
        for rpath in rpath_list:
            argv.append('-delete_rpath')
            argv.append(rpath)
        argv.append(dylib)
        #print("change_rpaths(): ", argv)
        util.subproc_check_call_flushed(argv)

    def modify_install_name(self, dylib, install_name):
        argv = ['install_name_tool', '-id', install_name, dylib]
        util.subproc_check_call_flushed(argv)


class BuildScriptInstaller(object):

    def __init__(self, verbose=False):
        self._logger = logging.getLogger(__name__)
        # print("BuildScriptInstaller.__init__(): __name__=" + __name__)
        self._verbose = verbose

    def set_verbose(self, verbose):
        self._verbose = verbose

    def install_script(self, inst_dir, script, modules):
        assert inst_dir is not None
        script = os.path.abspath(script)
        module_flist = []
        package_dir_set = set()
        # python modules are specified in import syntax like "<package>.<name>".
        for pymod in modules:
            pymod_elem = pymod.split('.')
            rel_path = os.path.sep.join(pymod_elem)
            module_flist.append(rel_path + '.py')
            if len(pymod_elem) > 1:
                # module is part of a python package
                pkg_list = list(pymod_elem[:-1])
                #print("processing python package dir list", pkg_list)
                while len(pkg_list) > 0:
                    pkg_dir = os.path.sep.join(pkg_list)
                    #print("processing pkg dir", pkg_dir)
                    if pkg_dir not in package_dir_set:
                        package_dir_set.add(pkg_dir)
                    pkg_list.pop()
        for pkg_dir in package_dir_set:
            #print("processing pkg dir", pkg_dir)
            module_flist.append(os.path.join(pkg_dir, '__init__.py'))
        #print("install_script: modules:", module_flist)

        # make sure all files exist before trying to copy anything.
        if not os.path.exists(script):
            raise Exception("file " + script + " does not exist.")
        # key = module_file_path, value = file system path
        # e.g. pyhhi/build/common/ver.py must be mapped to a directory listed by sys.path.
        module_flist_src_dict = {}
        for f in module_flist:
            for pth in sys.path:
                fpath = os.path.join(pth, f)
                if os.path.exists(fpath):
                    module_flist_src_dict[f] = fpath
                    break
                else:
                    fpath = None
            if fpath is None:
                raise Exception("module file {0} not found.".format(fpath))

        # create destination directory
        if not os.path.exists(inst_dir):
            os.makedirs(inst_dir)

        for f in module_flist:
            assert f in module_flist_src_dict
            fpath_src = module_flist_src_dict[f]
            dname = os.path.dirname(f)
            dst_dir = os.path.join(inst_dir, dname)
            #print("cp " + fpath_src + " -> " + os.path.join(dst_dir, dname))
            if not os.path.exists(dst_dir):
                os.makedirs(dst_dir)
                #print("creating directory " + dst_dir)
            if self._verbose:
                print("copying %-15s %s" % (os.path.basename(fpath_src), dst_dir))
            shutil.copy(fpath_src, dst_dir)

        # copy the script to <inst_dir>
        if self._verbose:
            print("copying %-15s %s" % (os.path.basename(script), inst_dir))
        shutil.copy(script, inst_dir)

