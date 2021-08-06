
from __future__ import print_function

import logging
import os
import re
import subprocess

import pyhhi.build.common.util as util
import pyhhi.build.common.ver as ver
from pyhhi.build.common.system import SystemInfo


class CMakeFinder(object):

    def __init__(self, sys_info=None):
        self._logger = logging.getLogger(__name__)
        if sys_info is None:
            self._sys_info = SystemInfo()
        else:
            self._sys_info = sys_info
        self._cmake_prog = None
        self._cmake_version = None
        self._cmake_search_path = self._sys_info.get_path()
        if self._sys_info.is_windows():
            cmake_dir_list = []
            cmake_inst_dir = self._query_winreg_cmake_inst_dir()
            if cmake_inst_dir is None:
                # cmake 3.6.1 is 64 bit but earlier cmake versions are 32 bit only.
                if self._sys_info.get_os_arch() == 'x86_64':
                    cmake_dir_list.append(os.path.join(self._sys_info.get_program_dir('x86_64'), 'CMake', 'bin'))
                cmake_dir_list.append(os.path.join(self._sys_info.get_program_dir('x86'), 'CMake', 'bin'))
                for cmake_dir in cmake_dir_list:
                    if os.path.exists(cmake_dir):
                        if cmake_dir not in self._cmake_search_path:
                            self._cmake_search_path.append(cmake_dir)
            else:
                # Append cmake install directory picked up from the registry (3.8.0 or higher).
                self._cmake_search_path.append(cmake_inst_dir)
        elif self._sys_info.is_macosx():
            # The default installation path is /Applications/CMake.app/Contents/bin on MacOSX.
            cmake_dir = os.path.join('/Applications', 'CMake.app', 'Contents', 'bin')
            if os.path.exists(cmake_dir):
                if cmake_dir not in self._cmake_search_path:
                    self._cmake_search_path.append(cmake_dir)
        elif self._sys_info.is_linux():
            pass
        else:
            assert False

    def set_cmake_search_path(self, search_path):
        if search_path:
            self._cmake_search_path = search_path[:]
            self._logger.debug("cmake search path changed to: %s", ';'.join(self._cmake_search_path))

    def find_cmake(self):
        """Returns the absolute path of a cmake executable."""
        if self._cmake_prog is None:
            self._logger.debug("cmake search path: %s", ';'.join(self._cmake_search_path))
            self._cmake_prog = util.find_tool_on_path('cmake', must_succeed=True, search_path=self._cmake_search_path)
            self._cmake_version = self._query_cmake_version(self._cmake_prog)
        return self._cmake_prog

    def is_cmake_installed(self):
        return self.find_cmake() is not None

    def get_cmake_version(self):
        assert self._cmake_version is not None
        return self._cmake_version

    def _query_cmake_version(self, cmake_cmd):
        retv = subprocess.check_output([cmake_cmd, '--version'], universal_newlines=True)
        return self._parse_cmake_version_retv(retv)

    def _parse_cmake_version_retv(self, retv):
        version_response = retv.rstrip()
        lines = version_response.splitlines()
        # Possible version information:
        #   cmake version 3.7.2
        #   cmake version 3.8.0-rc2
        #   cmake3 version 3.6.3           # redhat 7.x cmake3
        re_version_match = re.match(r'cmake\d*\s+version\s+([0-9]+\.[0-9]+\.[0-9]+)', lines[0], re.IGNORECASE)
        if not re_version_match:
            raise Exception("cmake has returned an unsupported version string. Please contact technical support.")
        self._logger.debug("cmake version: %s", re_version_match.group(1))
        return ver.version_tuple_from_str(re_version_match.group(1))

    def _query_winreg_cmake_inst_dir(self):
        cmake_install_dir = None
        if self._sys_info.is_python3():
            import winreg
            win_registry = winreg
        else:
            import _winreg
            win_registry = _winreg

        reg_section = "SOFTWARE\\Kitware\\CMake"

        try:
            # rkey must be initialized in case OpenKey() does not return but raises an exception.
            rkey = None
            rkey = win_registry.OpenKey(win_registry.HKEY_LOCAL_MACHINE, reg_section)
            install_dir = win_registry.QueryValueEx(rkey, 'InstallDir')[0]
            if os.path.exists(os.path.join(install_dir, 'bin', 'cmake.exe')):
                cmake_install_dir = os.path.join(install_dir, 'bin')
        except WindowsError:
            if rkey is not None:
                win_registry.CloseKey(rkey)
        if cmake_install_dir:
            self._logger.debug("cmake install dir (winreg): %s", cmake_install_dir)
        else:
            self._logger.debug("cmake install dir (winreg): %s", "not found")
        return cmake_install_dir
