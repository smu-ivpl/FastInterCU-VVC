from __future__ import print_function

import os
import stat
import re
import shutil
import subprocess
import logging

import pyhhi.build.common.system as system
import pyhhi.build.common.util as util
import pyhhi.build.common.ver as ver


class NdkFinder(object):
    
    def __init__(self, use_env=True):
        self._logger = logging.getLogger(__name__)
        self._sys_info = system.SystemInfo()
        self._use_env = use_env  # check for ANDROID_NDK_ROOT when searching for NDK.
        self._re_ndk_root = re.compile(r'((android-ndk-r\d+([a-z]?))|(ndk-bundle))$')
        self._ndksa_root = os.path.join(self._sys_info.get_home_dir(), 'bin', 'ndk-sa')
        # set up a platform specific NDK search path
        if self._sys_info.is_macosx():
            ndk_search_path = [os.path.join(self._sys_info.get_home_dir(), 'Library', 'Android', 'Sdk'),
                               os.path.join(self._sys_info.get_home_dir(), 'Library'),
                               self._sys_info.get_home_dir()]
        else:
            ndk_search_path = [os.path.join(self._sys_info.get_home_dir(), 'Android', 'Sdk'),
                               self._sys_info.get_home_dir()]
        self._ndk_search_path = [d for d in ndk_search_path if os.path.exists(d)]
        self._logger.debug("NDK search path: " + ';'.join(self._ndk_search_path))
        #
        (self._ndk_root, self._ndk_version) = self._find_ndk_root()

        self._ndk_platforms = self._query_platforms(self._ndk_root)

        self._ndksa_toolchains = {'gnustl': self._query_ndksa_toolchains(self.get_ndksa_root('gnustl')),
                                  'libc++': self._query_ndksa_toolchains(self.get_ndksa_root('libc++'))}

        self._ndk_toolchain_dict = {'arm-linux-androideabi':'arm-linux-androideabi',
                                    'aarch64-linux-android':'aarch64-linux-android',
                                    'x86':'i686-linux-android',
                                    'x86_64':'x86_64-linux-android'}

    def get_ndk_root(self):
        return self._ndk_root

    def get_ndk_version(self):
        return self._ndk_version

    def get_ndksa_root(self, stl='gnustl'):
        if stl:
            return os.path.join(self._ndksa_root, stl)
        else:
            return self._ndksa_root

    def get_ndksa_toolchains(self, stl='gnustl'):
        return self._ndksa_toolchains[stl]
    
    def get_ndk_platforms(self):
        return self._ndk_platforms

    def get_api_level_from_platform(self, platform):
        # platform := android-21, etc
        re_match = re.match(r'android-(\d+)$', platform)
        if re_match:
            return int(re_match.group(1))
        else:
            raise Exception("Unsupported android platform specification encountered: " + platform)

    def create_ndksa_toolchain(self, ndk_toolchain, api_level=None, unified_headers=False, ndk_stl='gnustl', ndk_llvm_version=None, inst_dir=None):
        # whether bash scripts have proper hash bang lines or not.
        hash_bang_missing = False
        if ver.version_compare(self._ndk_version, (11,0)) < 0:
            hash_bang_missing = True
        # ndk_toolchain := arm-linux-androideabi-4.9, arm-linux-androideabi-clang3.6, arm-linux-androideabi
        #               := arm-linux-androideabi-4.9 aarch64-linux-android-4.9 x86-4.9 x86_64-4.9
        ndk_toolchain_parts = ndk_toolchain.split('-')
        if re.match('(clang)?[0-9.]+$', ndk_toolchain_parts[-1]):
            joiner = '-'
            ndk_toolchain_prefix = joiner.join(ndk_toolchain_parts[:-1])
        else:
            ndk_toolchain_prefix = ndk_toolchain
        if ndk_toolchain_prefix in self._ndk_toolchain_dict:
            if inst_dir is None:
                ndksa_inst_dir = os.path.join(self.get_ndksa_root(ndk_stl), self._ndk_toolchain_dict[ndk_toolchain_prefix])
            else:
                ndksa_inst_dir = os.path.join(inst_dir, ndk_stl, self._ndk_toolchain_dict[ndk_toolchain_prefix])
        else:
            raise Exception("The NDK toolchain " + ndk_toolchain + " is not supported.")
        # Create a possibly patched version of the original script which can be invoked from python.
        mk_toolchain_script = self._create_mk_toolchain_script(self._ndk_root, hash_bang_missing)
        # build the argument vector
        mk_toolchain_args = [mk_toolchain_script]
        if api_level is None:
            api_level = self.get_api_level_from_platform(self._ndk_platforms[-1])

        if (ver.version_compare(self._ndk_version, (14, 0)) >= 0) and (ver.version_compare(self._ndk_version, (16, 0)) < 0):
            if unified_headers:
                mk_toolchain_args.append('--unified-headers')
        if ver.version_compare(self._ndk_version, (12,0)) >= 0:
            # NDK 12 comes with a new python script expecting different command line options
            if ndk_toolchain_parts[0] == 'aarch64':
                ndk_arch = 'arm64'
            else:
                ndk_arch = ndk_toolchain_parts[0]
            mk_toolchain_args.append('--arch=' + ndk_arch)
            mk_toolchain_args.append('--api=' + str(api_level))
            # --force -> remove destination directory if it exists.
            mk_toolchain_args.append('--force')
        else:
            ndk_platform = 'android-%d' % api_level
            mk_toolchain_args.append('--platform=' + ndk_platform)
            mk_toolchain_args.append('--toolchain=' + ndk_toolchain)
            if ver.version_compare(self._ndk_version, (11,0)) < 0:
                if ndk_llvm_version is not None:
                    mk_toolchain_args.append('--llvm-version=' + ndk_llvm_version)
            else:
                # enable clang by default
                mk_toolchain_args.append('--use-llvm')

        if ndk_stl is not None:
            mk_toolchain_args.append('--stl=' + ndk_stl)
        mk_toolchain_args.append('--install-dir=' + ndksa_inst_dir)
        #print("create_ndksa_toolchain:", mk_toolchain_args)
        #return
        
        # prepare the destination directory
        if os.path.exists(ndksa_inst_dir):
            shutil.rmtree(ndksa_inst_dir)
        if not os.path.exists(self._ndksa_root):
            os.makedirs(self._ndksa_root)
        
        print("Launching: " + ' '.join(mk_toolchain_args))
        # and launch the script to create the new toolchain
        util.subproc_check_call_flushed(mk_toolchain_args)

        if hash_bang_missing:
            if os.path.exists(mk_toolchain_script):
                os.remove(mk_toolchain_script)
            # patch the clang++ shell wrapper to have a hash bang
            self._patch_clang_shell_script(os.path.join(ndksa_inst_dir, 'bin', self._ndk_toolchain_dict[ndk_toolchain_prefix] + '-clang++'))
            self._patch_clang_shell_script(os.path.join(ndksa_inst_dir, 'bin', self._ndk_toolchain_dict[ndk_toolchain_prefix] + '-clang'))

        # update the list of available toolchains.
        self._ndksa_toolchains[ndk_stl] = self._query_ndksa_toolchains(os.path.join(ndksa_inst_dir, '..'))

    def _find_ndk_root(self):
        self._logger.debug("entering: use_env=" + str(self._use_env))
        if self._use_env and ('ANDROID_NDK_ROOT' in os.environ):
            self._logger.debug("using environment variable ANDROID_NDK_ROOT")
            ndk_root = util.normalize_path(os.path.expandvars('$ANDROID_NDK_ROOT'))
            if not os.path.exists(ndk_root):
                raise Exception("environment variable ANDROID_NDK_ROOT points to a non-existing directory, please fix it or remove it.")
            ndk_version = self._query_ndk_version(ndk_root)
        else:
            # self._logger.warn("The environment variable NDK is not defined.")
            (ndk_root, ndk_version) = self._find_ndk_root_default(self._ndk_search_path)
        return ndk_root, ndk_version

    def _find_ndk_root_default(self, ndk_search_path):
        self._logger.debug("entering: ndk_search_path=" + ";".join(ndk_search_path))
        # Search for android-ndk-r10d, android-ndk-r10, or similar and select the latest release. If the regular expressions matches, 
        # there are always two groups but the second group is empty if no patch level is given.
        ndk_version_list = []
        ndk_dir_map = {}

        for ndk_super_root in ndk_search_path:
            for d in os.listdir(ndk_super_root):
                re_match = self._re_ndk_root.match(d)
                if re_match:
                    # the map associates a numeric version tuple with the NDK directory because the version sorter works on lists of numeric version tuples.
                    ndk_root = os.path.join(ndk_super_root, d)
                    ndk_version = self._query_ndk_version(ndk_root)
                    if ndk_version not in ndk_dir_map:
                        ndk_dir_map[ndk_version] = ndk_root
                        ndk_version_list.append(ndk_version)
                        self._logger.debug("find NDK " + ndk_root)
                    else:
                        self._logger.debug("duplicate NDK version encountered: " + ndk_root + " -> ignoring")
        if not ndk_version_list:
            raise Exception("The Android NDK installation directory cannot be found automatically, please contact technical support.")
        ndk_version_list = ver.version_list_sort(ndk_version_list)
        ndk_version = ndk_version_list[-1]
        ndk_root = ndk_dir_map[ndk_version]
        self._logger.debug("leaving, ndk_root=" + ndk_root)
        return ndk_root, ndk_version

    def _query_ndk_version(self, ndk_root):
        version_file = os.path.join(ndk_root, 'source.properties')
        if not os.path.exists(version_file):
            raise Exception("Android NDK version file " + version_file + " is missing, please contact technical support.")
        # Pkg.Revision = 11.2.2725575
        re_ndk_version = re.compile(r'Pkg\.Revision\s*=\s*([0-9.]+)', re.IGNORECASE)
        with open(version_file) as vf:
            for line in vf:
                re_match = re_ndk_version.match(line)
                if re_match:
                    ndk_version = ver.version_tuple_from_str(re_match.group(1))
                    return ndk_version
        raise Exception("Android NDK " + ndk_root + ": version information is missing. Please contact technical support.")

    def _query_platforms(self, ndk_root):
        platforms_dir = os.path.join(ndk_root, 'platforms')
        assert os.path.exists(platforms_dir)
        re_platform = re.compile('android-(\d+)$')
        # enumerate installed platforms in ascending order.
        version_list = [ver.version_tuple_from_str(re_platform.match(d).group(1)) for d in os.listdir(platforms_dir) if re_platform.match(d)]
        if version_list:
            version_list = ver.version_list_sort(version_list)
            ndk_platforms = ['android-' + ver.version_tuple_to_str(version) for version in version_list]
            return ndk_platforms
        else:
            assert False

    def _query_ndksa_toolchains(self, ndksa_root):
        ndksa_toolchains = []
        if os.path.exists(ndksa_root):
            ndksa_toolchains = [d for d in os.listdir(ndksa_root) if os.path.isdir(os.path.join(ndksa_root, d))]
        return ndksa_toolchains

    def _create_mk_toolchain_script(self, ndk_root, hash_bang_missing):
        if ver.version_compare(self._ndk_version, (12, 0)) >= 0:
            mk_toolchain_script_basename = 'make_standalone_toolchain.py'
        else:
            mk_toolchain_script_basename = 'make-standalone-toolchain.sh'
        mk_toolchain_script = os.path.join(ndk_root, 'build', 'tools', mk_toolchain_script_basename)
        # print("mk_toolchain_script: " + mk_toolchain_script)
        assert os.path.exists(mk_toolchain_script)
        if hash_bang_missing:
            mk_toolchain_script_tmp = os.path.join(ndk_root, 'build', 'tools', 'make-standalone-toolchain.bash')
            self._add_bash_hash_bang(mk_toolchain_script, mk_toolchain_script_tmp)
        else:
            mk_toolchain_script_tmp = mk_toolchain_script
        return mk_toolchain_script_tmp

    def _patch_clang_shell_script(self, clang_script):
        if not os.path.exists(clang_script):
            return
        script_tmp = clang_script + '.tmp'
        self._add_bash_hash_bang(clang_script, script_tmp)
        assert os.path.exists(script_tmp)
        os.remove(clang_script)
        os.rename(script_tmp, clang_script)

    def _add_bash_hash_bang(self, script_old, script_new):
        if not os.path.exists(script_old):
            raise Exception("The script " + script_old + " does not exist.")
        # read the content of the original script
        input_lines = []
        with open(script_old) as fin:
            for line in fin:
                input_lines.append(line.rstrip())
        # create a new script with a hash bang 
        if os.path.exists(script_new):
            os.remove(script_new)
        with open(script_new, "w") as fout:
            if not re.match(r'#!\s*/[a-zA-Z]+', input_lines[0]):
                fout.write("#!/bin/bash\n")
            for line in input_lines:
                fout.write(line + "\n")
        # make the script executable
        st = os.stat(script_new)
        os.chmod(script_new, st.st_mode | stat.S_IXUSR)
