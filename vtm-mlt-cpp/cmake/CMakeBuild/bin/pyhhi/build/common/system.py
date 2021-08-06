
from __future__ import print_function

import multiprocessing
import logging
import platform
import os
import sys
import re
import shutil
import tempfile
import textwrap
import subprocess

import pyhhi.build.common.ver as ver
import pyhhi.build.common.util as util


class SystemInfo(object):

    class __SystemInfo(object):

        def __init__(self):
            self._logger = logging.getLogger(__name__)
            # print("Constructing a new SystemInfo object")
            # check python version: it has to be 2.7 or higher
            self._python_version = ver.get_python_version(True)

            # create all attributes with default values which will be overwritten in platform specific sections later on
            self._python_launcher = None
            self._python_arch = 'x86_64'
            self._python_implementation = platform.python_implementation()
            self._os_distro = platform.platform()
            # self._os_distro_short = platform.platform()
            self._win32api_installed = False
            self._windows_msys = False
            self._redhat_system = False
            self._debian_system = False
            self._suse_system = False
            self._pkg_fmt = None
            self._pkg_arch = None

            self._os_codename = 'unknown'
            self._os_arch = 'x86_64'
            self._os_version = (0, 0, 0)
            self._num_processors = multiprocessing.cpu_count()

            platform_system = platform.system().lower()
            self._platform_system = platform_system

            self._desktop_dir = None
            self._default_proj_home_dir = None

            if platform_system == 'linux':
                # e.g. x86_64 or i686
                platform_machine = platform.machine().lower()
                if platform_machine != 'x86_64':
                    assert re.match(r'i[6543]86', platform_machine, re.IGNORECASE)
                    platform_machine = 'x86'

                self._os_arch = platform_machine
                self._python_arch = platform_machine

                # there's no portable way in python to obtain the linux version.
                self._query_linux_distro_info()

                if self.is_debian():
                    self._pkg_fmt = 'deb'
                    self._pkg_arch = subprocess.check_output(['dpkg', '--print-architecture'], universal_newlines=True).rstrip()
                elif self.is_redhat() or self.is_suse():
                    self._pkg_fmt = 'rpm'
                    self._pkg_arch = subprocess.check_output(['rpm', '--eval', '%_arch'], universal_newlines=True).rstrip()
                else:
                    # unknown linux system, no logic available to figure out the package format or package architecture yet.
                    self._pkg_fmt = 'unknown'
                    self._pkg_arch = subprocess.check_output(['uname', '-m'], universal_newlines=True).rstrip()
                    if self._pkg_arch == 'x86_64':
                        self._pkg_arch = 'amd64'
            elif platform_system == 'windows':
                self._programx86_dir = None
                self._program_dir = None
                self._program_data_dir = None

                if 'MSYSTEM' in os.environ:
                    self._windows_msys = True

                # obtain additional version information
                # windows 7: ('7', '6.1.7601', 'SP1', 'Multiprocessor Free')
                self._os_version = ver.version_tuple_from_str(platform.win32_ver()[1])

                # Hm, win32api not installed/available -> system detection may not by accurate and includes some guessing.
                if platform.architecture()[0] != '64bit':
                    self._python_arch = 'x86'
                    self._os_arch = 'x86'
                    if ('PROCESSOR_ARCHITEW6432' in os.environ) and (os.getenv('PROCESSOR_ARCHITEW6432') == 'AMD64'):
                        # 32 bit python interpreter and 64 bit windows
                        self._os_arch = 'x86_64'
                    
                if self._os_arch == 'x86_64':
                    if self._python_arch == 'x86':
                        self._program_dir = os.path.normpath(os.getenv('PROGRAMW6432'))
                        self._programx86_dir = os.path.normpath(os.getenv('PROGRAMFILES'))
                    else:
                        self._program_dir = os.path.normpath(os.getenv('PROGRAMFILES'))
                        self._programx86_dir = os.path.normpath(os.getenv('PROGRAMFILES(X86)'))
                    assert self._programx86_dir is not None
                elif self._os_arch == 'x86':
                    self._program_dir = os.path.normpath(os.getenv('PROGRAMFILES'))
                else:
                    assert False
                assert self._program_dir is not None
                self._program_data_dir = os.path.normpath(os.getenv('PROGRAMDATA'))

                if self._windows_msys:
                    pass
                elif os.path.exists(os.path.join(r'C:\Windows', 'py.exe')):
                    self._python_launcher = os.path.join(r'C:\Windows', 'py.exe')

                # probe the registry to ensure the shell will pass additional arguments to the
                # registered python interpreter.
                # if pywin_check:
                #    self.check_pywin_registry()
            elif platform_system == 'darwin':
                # create a dictionary to simplify the mapping between minor version ID and codenames
                codename_dict = {'10.4': 'tiger',
                                 '10.5': 'leopard',
                                 '10.6': 'snow leopard',
                                 '10.7': 'lion',
                                 '10.8': 'mountain lion',
                                 '10.9': 'mavericks',
                                 '10.10': 'yosemite',
                                 '10.11': 'el capitan',
                                 '10.12': 'sierra',
                                 '10.13': 'high sierra',
                                 '10.14': 'mojave'}
                # replace darwin by macosx
                self._platform_system = 'macosx'
                # e.g. ('10.7.4', ('', '', ''), 'x86_64')
                mac_ver = platform.mac_ver()
                # save the macosx version as a tuple of integers
                self._os_version = ver.version_tuple_from_str(mac_ver[0])
                # analyze the version tuple to derive the codename: lion, mountain lion, etc
                major_minor_ver = str(self._os_version[0]) + '.' + str(self._os_version[1])
                if major_minor_ver in codename_dict:
                    self._os_codename = codename_dict[major_minor_ver]
                #e.g. x86_64
                self._os_arch = mac_ver[2]
                # python architecture is the same as the macosx architecture
                self._python_arch = self._os_arch
            else:
                raise Exception('unsupported platform detected: ' + platform_system)
            self._query_home_dir()
            self._query_default_proj_home_dir()
            self._query_desktop_dir()
            self._query_search_path()

        def get_python_version(self):
            return self._python_version

        def get_python_executable(self):
            return sys.executable

        def get_python_launcher(self):
            return self._python_launcher

        def is_python3(self):
            return ver.version_compare(self._python_version, (3, 0)) >= 0

        def get_script_ext(self):
            if self.is_python3():
                script_ext = '3.py'
            else:
                script_ext = '.py'
            return script_ext

        def get_python_implementation(self):
            return self._python_implementation

        def get_python_arch(self):
            return self._python_arch

        def get_platform(self):
            return self._platform_system

        def get_platform_long(self):
            return platform.platform()

        def is_linux(self):
            return self._platform_system == 'linux'

        def get_pkg_fmt(self):
            assert self._pkg_fmt is not None
            return self._pkg_fmt

        def get_pkg_arch(self):
            assert self._pkg_arch is not None
            return self._pkg_arch

        def is_redhat(self):
            return self._redhat_system

        def is_debian(self):
            return self._debian_system

        def is_suse(self):
            return self._suse_system

        def is_cray(self):
            if self._os_distro is None:
                return False
            return self._os_distro == 'cray'

        def is_windows(self):
            return self._platform_system == 'windows'

        def is_windows8(self):
            return self.is_windows() and (self._os_version[0] == 6) and (self._os_version[1] == 2)

        def is_windows_msys(self):
            return self._windows_msys

        def check_pywin_registry(self):
            assert self.is_windows()
            # Create a temporary script and invoke it to see whether argument passing works or not.
            # The test must go through the windows shell to be meaningful.
            tmp_dir = tempfile.mkdtemp()
            probe_script_name = os.path.join(tmp_dir, 'pywin_test.py')
            tmp_file = open(probe_script_name, "w")
            # create a simple script to echo the command arguments separated by a single space
            if self.is_python3():
                tmp_file.write("#!/usr/bin/env python3\n")
            else:
                tmp_file.write("#!/usr/bin/env python\n")
                tmp_file.write("from __future__ import print_function\n")
            tmp_file.write("import sys\n")
            tmp_file.write("if len(sys.argv) > 1:\n")
            tmp_file.write("    joiner = ' '\n")
            tmp_file.write("    print(joiner.join(sys.argv[1:]))\n")
            tmp_file.close()

            # invoke the script through the windows shell and check the output
            retv = subprocess.check_output([probe_script_name, "probe"], shell=True, universal_newlines=True).rstrip()
            shutil.rmtree(tmp_dir)
            # print("check_pywin_registry: '" + retv + "'")
            if retv != "probe":
                msg = "\nThe python installation is broken, the shell does not pass on any command line arguments to the python script.\n"
                msg += "The following steps are most likely to fix the problem, for further assistance contact technical support.\n"
                msg += " - Deinstall all versions of python and reboot the system.\n"
                msg += " - Download the latest 64 bit version of python from www.python.org and install it, the build system will work with python 2.7.x or python 3.x.\n"
                raise Exception(msg)
            return True

        def check_os_detection(self, todo_list):
            if (self._os_version[0] == 0) and self.is_linux():
                if not os.path.exists('/usr/bin/lsb_release'):
                    todo_list.append("The system identification depends on lsb_release which does not seem to be available.")
                    if self.is_redhat():
                        todo_list.append("On redhat 6.x/7.x and compatible systems, you may need to install redhat-lsb-core.")
                        todo_list.append("")

        def is_macosx(self):
            return self._platform_system == 'macosx'

        def get_os_distro(self):
            return self._os_distro

        def get_os_distro_short(self):
            return self._os_distro

        def get_os_codename(self):
            return self._os_codename

        def get_os_arch(self):
            return self._os_arch

        def get_os_version(self):
            return self._os_version

        def get_number_processors(self):
            return self._num_processors

        def get_system_info_full_str(self):
            """Return a string consisting of colon separated fields intended for the Boost.Build script interface."""
            str_list = []

            str_list.append(self.get_platform())
            str_list.append(self.get_os_arch())
            str_list.append(str(self.get_number_processors()))

            # should be a single lowercase word as it may be used to make up a package filename
            str_list.append(self.get_os_distro_short())

            str_list.append(self.get_os_codename())

            # create a version string given the version tuple
            joiner = '.'
            str_list.append(joiner.join([str(x) for x in self.get_os_version()]))

            # no sure it's really needed somewhere in Boost.Build
            str_list.append(self._python_arch)
            if self.is_linux():
                if self.is_debian():
                    str_list.append('debian')
                elif self.is_redhat():
                    str_list.append('redhat')
                elif self.is_suse():
                    str_list.append('suse')
                else:
                    str_list.append('unknown_linux_flavor')
            else:
                str_list.append('none')
            str_list.append(self.get_home_dir())
            return ';'.join(str_list)

        def get_path(self):
            return self._search_path

        def get_home_dir(self, native=False):
            if self.is_windows_msys() and native:
                home_dir = os.path.normpath(os.path.expandvars('$USERPROFILE'))
            else:
                home_dir = self._home_dir
            return home_dir

        def get_default_proj_home_dir(self):
            return self._default_proj_home_dir

        def get_desktop_dir(self):
            return self._desktop_dir

        def get_program_dir(self, target_arch):
            if not self.is_windows():
                raise Exception("The method get_program_dir is only supported on the windows platform.")
            if (self.get_os_arch() == 'x86_64') and (target_arch == 'x86'):                
                program_dir = self._programx86_dir
            else:
                program_dir = self._program_dir
            assert os.path.exists(program_dir)            
            return program_dir

        def get_program_data_dir(self):
            return self._program_data_dir

        def get_short_path(self, fpath):
            if self.is_windows():
                fpath = os.path.normpath(self.get_short_path_win(fpath))
            return fpath

        def get_short_path_win(self, fpath):
            # need to go through the shell to get the short path name
            tempdir = tempfile.gettempdir()
            get_short_path_script = os.path.join(tempdir, 'pyhhi_get_short_path.cmd')
            if not os.path.exists(get_short_path_script):
                # create a shell command script to do the path conversion
                with open(get_short_path_script, "w") as script_file:
                    script_file.write(textwrap.dedent("""\
                        @ECHO OFF
                        echo %~s1
                    """))
            # invoke the script through the windows shell.
            short_path = subprocess.check_output([get_short_path_script, fpath], shell=True, universal_newlines=True).rstrip()
            return short_path

        def check_comspec(self):
            comspec = os.getenv('COMSPEC')
            if (comspec is None) or (not os.path.exists(comspec)):
                raise Exception("The environment variable COMSPEC must be fixed, please contact technical support.")

        def get_subprocess_devnull(self):
            if ver.version_compare(self._python_version, (3,3)) >= 0:
                devnull =  subprocess.DEVNULL
            else:
                self._logger.debug("attribute subprocess.DEVNULL not available (python < 3.3), using os.devnull instead")
                devnull = self._get_devnull()
            return devnull

        def _get_devnull(self):
            if not hasattr(self, '_devnull'):
                self._devnull = os.open(os.devnull, os.O_RDWR)
            return self._devnull

        def _query_linux_distro_info(self):
            if 'CRAYOS_VERSION' in os.environ:
                self._os_distro = 'cray'
                self._os_version = ver.version_tuple_from_str(os.environ['CRAYOS_VERSION'])
            else:
                lsb_release = '/usr/bin/lsb_release'
                if os.path.exists(lsb_release):
                    # use lsb_release if available and assume all options are supported and return
                    # sensible values.

                    # obtain a human readable description of the distribution. This should be a single word as it
                    # may be used to generate package filenames.
                    retv = subprocess.check_output([lsb_release, '-is'], universal_newlines=True)
                    self._os_distro = retv.rstrip().lower()

                    retv = subprocess.check_output([lsb_release, '-rs'], universal_newlines=True)
                    version_str = retv.rstrip()
                    # version_str = "4"
                    # version_str = "4.0-rolling"
                    # version_str = "rolling"
                    re_match = re.match(r'([0-9.,_-]+\d+)|(\d+)', version_str)
                    if re_match:
                        self._os_version = ver.version_tuple_from_str(re_match.group(0))

                    retv = subprocess.check_output([lsb_release, '-cs'], universal_newlines=True)
                    self._os_codename = retv.rstrip().lower()
                    if self._os_codename == 'n/a':
                        self._os_codename = 'none'
                else:
                    # lsb_release not found -> try to guess the distro but don't try to parse the
                    # proprietary files to figure out the remaining system info bits.
                    if os.path.exists('/etc/fedora-release'):
                        self._os_distro = 'fedora'
                    elif os.path.exists('/etc/redhat-release'):
                        self._os_distro = 'redhat'
                    elif os.path.exists('/etc/SuSE-release'):
                        self._os_distro = 'suse'
                    elif os.path.exists('/etc/debian_version'):
                        self._os_distro = 'debian'
                    else:
                        self._os_distro = 'unknown'

            # make sure os_distro and os_codename do not contain any spaces as they may become part of a
            # package filename.
            if self._os_distro is not None:
                self._os_distro = self._os_distro.replace(' ', '-')

            if self._os_codename is not None:
                self._os_codename = self._os_codename.replace(' ', '-')

            # determine the general flavor of the linux system: redhat, debian or suse
            if os.path.exists('/etc/redhat-release'):
                self._redhat_system = True
            elif os.path.exists('/etc/debian_version'):
                self._debian_system = True
            else:
                # not sure how to do this for suse, revert back to regex
                if re.match(r'(suse)|(opensuse)', self._os_distro):
                    self._suse_system = True

        def _query_home_dir(self):
            home_dir = os.path.expanduser('~')
            # make sure the user's home directory exists
            if not os.path.exists(home_dir):
                raise Exception('home directory "' + home_dir + '" does not exist.')
            self._home_dir = os.path.normpath(home_dir)

        def _query_default_proj_home_dir(self):
            if 'PROJ_HOME' in os.environ:
                proj_home_dir = os.path.normpath(os.path.expandvars('$PROJ_HOME'))
            else:
                proj_home_dir = os.path.join(self.get_home_dir(native=True), 'projects')
            if os.path.exists(proj_home_dir):
                self._default_proj_home_dir = proj_home_dir
            else:
                self._default_proj_home_dir = None

        def _query_search_path(self):
            self._search_path = []
            env_path = os.getenv('PATH')
            for dir in env_path.split(os.path.pathsep):
                self._search_path.append(util.normalize_path(dir))

        def _query_desktop_dir(self):
            # MSYS has its own environment but Desktop comes from the native windows home.
            home_dir = self.get_home_dir(native=True)
            desktop_dir = os.path.join(home_dir, 'Desktop')
            if os.path.exists(desktop_dir):
                self._desktop_dir = desktop_dir
            else:
                self._desktop_dir = None

    # the singleton as a class attribute
    instance = None

    def __init__(self, pywin_check=False):
        self._logger = logging.getLogger(__name__)
        if SystemInfo.instance is None:
            SystemInfo.instance = SystemInfo.__SystemInfo()
        if SystemInfo.instance.is_windows() and pywin_check:
            # The caller requested the additional windows registry check.
            # SystemInfo.instance.check_pywin_registry()

            # One more addtional check to catch a corrupted COMSPEC setting which yields to subsequent failures of
            # subprocess calls if shell=True is used.
            SystemInfo.instance.check_comspec()

            if SystemInfo.instance.get_os_arch() == 'x86':
                msg = "\nThe build system requires windows 64 bit but the platform seems to be windows 32 bit.\n"
                msg += "Please contact technical support for further assistance.\n"
                raise Exception(msg)
            elif (SystemInfo.instance.get_os_arch() == 'x86_64') and (SystemInfo.instance.get_python_arch() == 'x86'):
                msg = "\nPython 32 bit is not supported on windows 64 bit, please use python 64 bit."
                raise Exception(msg)

    def __getattr__(self, item):
        return getattr(SystemInfo.instance, item)
