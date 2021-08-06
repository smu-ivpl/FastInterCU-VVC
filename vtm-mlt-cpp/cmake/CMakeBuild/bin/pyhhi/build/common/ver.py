
from __future__ import print_function

import platform
import re
import os.path
import plistlib
import subprocess
import functools


import pyhhi.build.common.util as util
import pyhhi.build.common.cmbldver as cmbldver


def get_cmake_build_version():
    return version_tuple_from_str(cmbldver.CMAKE_BUILD_VERSION_STR)


def _get_python_version_numeric():
    re_number = re.compile(r'(\d+).*')
    version_list = []
    # extra logic to deal with python on ubuntu 11.10: 2.7.2+
    for x in platform.python_version_tuple():
        re_match = re_number.match(x)
        if re_match:
            version_list.append(int(re_match.group(1), 10))
        else:
            version_list.append(0)
    return tuple(version_list)


def check_python_version(version_tuple=None):
    """Checks the python version and throws an exception if the version is not supported."""
    if version_tuple is None:
        version_tuple = _get_python_version_numeric()
    if version_compare(version_tuple, (2, 7)) < 0:
        raise Exception('python ' + platform.python_version() + ' is not supported. Please update to 2.7 or higher.')


def get_python_version(check_version=False):
    version_tuple = _get_python_version_numeric()
    if check_version:
        check_python_version(version_tuple)
    return version_tuple


def version_tuple_from_str(version, nelem=None):
    """Split a version string using '.' and '-' as separators and return a tuple of integers."""
    re_match = re.match(r'([0-9.,_-]+\d+)|(\d+)', version)
    if re_match:
        version_list = re.split('[.,_-]', re_match.group(0))
        if nelem is not None:
            # adjust the list to contain exactly the specified number of elements.
            while len(version_list) < nelem:
                version_list.append('0')
            return tuple([int(x, 10) for x in version_list[:nelem]])
        else:
            return tuple([int(x, 10) for x in version_list])
    else:
        raise Exception("The version string '" + version + "' is not supported, no leading numeric digits found.")


def get_boost_version_str(version_str):
    re_match = re.match(r'([0-9.]+\d+)', version_str)
    if not re_match:
        raise Exception("The version string '" + version_str + "' cannot be converted into a boost compliant version string.")
    boost_version_str = re.sub(r'\.', '_', re_match.group(1))
    return boost_version_str


def version_tuple_to_str(version, sep='.'):
    """Join the version components using '.' and return the string."""
    return sep.join([str(x) for x in version])


def version_list_to_str(version_list):
    return ' '.join([version_tuple_to_str(x) for x in version_list])


def ubuntu_version_tuple_to_str(version):
    """Join the first two version components of an ubuntu release number and return the string."""
    assert len(version) >= 2
    version_str = '%d.%02d' % (version[0], version[1])
    return version_str


def version_compare(version1, version2):
    """Compare two version iterables consisting of arbitrary number of numeric elements."""
    len_version1 = len(version1)
    len_version2 = len(version2)
    if len_version1 == len_version2:
        # Both version objects have the same number of components, compare them left to right
        for i in range(len_version1):
            if version1[i] > version2[i]: return 1
            elif version1[i] < version2[i]: return -1
        # version objects compare equal
        return 0
    elif len_version1 > len_version2:
        version2_tmp = list(version2)
        while len(version2_tmp) < len_version1: version2_tmp.append(0)
        return version_compare(version1, version2_tmp)
    else:
        version1_tmp = list(version1)
        while len(version1_tmp) < len_version2: version1_tmp.append(0)
        return version_compare(version1_tmp, version2)


def version_list_sort(version_list):

    # Written for Python 2.7 and 3.x: functools.cmp_to_key() requires 2.7 or higher.

    # Notes: python 2.x supports a second argument cmp to specify the comparision function but
    #        python 3.x does not.
    return sorted(version_list, key=functools.cmp_to_key(version_compare))


def version_str_to_rpm_version_tuple(version):
    re_match = re.match(r'([^-]+)-(\S+)', version)
    if re_match:
        return re_match.group(1), re_match.group(2)
    else:
        re_match = re.match(r'([\d.-]+)[-.](\d+)', version)
        if re_match:
            return re_match.group(1), re_match.group(2)
    raise Exception("The version string '" + version + "' is not a valid RPM version string.")


def get_default_version_filename(src_filename):
    """Returns the default version filename given a source filename."""
    repo_name = util.find_repo_name_from_src_path(src_filename)
    repo_path = util.find_repo_path_from_src_path(src_filename)
    return os.path.join(repo_path, 'include', repo_name, 'version.h')


def parse_version_file(version_file, verbatim=False):
    """Parse standard version file and return the version ID as a numeric tuple or verbatim."""
    if not os.path.exists(version_file):
        raise Exception("version file '" + version_file + "' does not exist.")

    # look at the extension to determine the file type: header file or plist file.
    (root, ext) = os.path.splitext(version_file)
    if ext == '.plist':
        return _parse_version_plist_file(version_file, verbatim)
    else:
        return _parse_version_h_file(version_file, verbatim)


def _parse_version_h_file(version_file, verbatim=False):
    """Parse standard version file and return the version ID as a numeric tuple or verbatim."""
    if not os.path.exists(version_file):
        raise Exception("version file '" + version_file + "' does not exist.")

    re_version_expr = re.compile(r'(^#if\s+![ ]*defined\(.*)|(^#\s*define\s+\S+_VERSION\s+)')
    re_version_tag_expr = re.compile(r'^#if\s+!\s*defined\(\s*([a-zA-Z0-9_]+)\s*\)')

    version_tag = None
    with open(version_file) as f:
        for line in f:
            re_match = re_version_expr.match(line)
            if not re_match:
                continue
            # print("version.h: found cpp line: {}".format(line))
            if version_tag is None:
                re_match = re_version_tag_expr.match(line)
                if re_match:
                    version_tag = re_match.group(1)
                    # print("found version tag: {}".format(version_tag))
                    re_version_str = re.compile(r'^#define\s+{}\s+"([^"]+)'.format(version_tag))
                    re_version_str2 = re.compile(r'^#define\s+{}\s+(\d+)'.format(version_tag))
                continue
            else:
                # print("checking cpp line: {}".format(line))
                # version tag found
                re_match = re_version_str.match(line)
                if not re_match:
                    re_match = re_version_str2.match(line)
                if re_match:
                    if verbatim:
                        return re_match.group(1)
                    else:
                        return version_tuple_from_str(re_match.group(1))

    raise Exception("No version ID found in file '" + version_file + "'.")


def _parse_version_plist_file(plist_file, verbatim=False):
    """Parse Info.plist file and return the version ID as a numeric tuple or verbatim."""
    if not os.path.exists(plist_file):
        raise Exception("version file '" + plist_file + "' does not exist.")

    version_str = None

    if (platform.system().lower() == 'darwin') and _is_binary_plist_file(plist_file):
        # On MacOSX there are two different plist formats supported: the older xml format and on more recent releases
        # the binary format. The latter is not supported by Python's plistlib directly.
        retv = subprocess.check_output(['/usr/bin/plutil', '-convert', 'xml1', '-o', '-', plist_file], universal_newlines=True)
        pl_dict = plistlib.readPlistFromString(retv)
    else:
        pl_dict = plistlib.readPlist(plist_file)

    # An Info.plist file is expected to contain at least one of the following version keys: CFBundleVersion and CFBundleShortVersionString.
    if 'CFBundleShortVersionString' in pl_dict:
        version_str = pl_dict['CFBundleShortVersionString']
    elif 'CFBundleVersion' in pl_dict:
        version_str = pl_dict['CFBundleVersion']
    else:
        raise Exception("No version ID found in file '" + plist_file + "'.")
    if verbatim:
        return version_str
    else:
        return version_tuple_from_str(version_str)


def _is_binary_plist_file(plist_file):
    if not os.path.exists(plist_file):
        raise Exception("file '" + plist_file + "' does not exist.")
    retv = subprocess.check_output(['/usr/bin/file', plist_file], universal_newlines=True)
    if re.search('binary property list', retv):
        return True
    else:
        return False

