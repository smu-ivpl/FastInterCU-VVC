
from __future__ import print_function

import logging
import os
import posixpath
import re
import shutil
import subprocess
import sys
import tempfile

import pyhhi.build.common.ver as ver
import pyhhi.build.common.util as util

from pyhhi.build.common.system import SystemInfo


class VcsCommandTracer(object):

    def __init__(self, enable_command_history=True):
        self._logger = logging.getLogger(__name__)
        if self._logger.isEnabledFor(logging.DEBUG):
            self._verbosity_level = 1
        else:
            self._verbosity_level = 0
        self._prog_name = os.path.basename(sys.argv[0])
        self._command_history = []
        self._enable_command_history = enable_command_history

    def add_to_command_history(self, cmd_argv):
        if self._enable_command_history:
            self._command_history.append(' '.join(cmd_argv))

    def get_command_history(self):
        return self._command_history

    def clear_command_history(self):
        self._command_history = []

    def trace_command(self, cmd_argv):
        if self._verbosity_level > 0:
            print("Launching: %s" % ' '.join(cmd_argv))
        self.add_to_command_history(cmd_argv)


class VcsUrl(object):

    def __init__(self, url_str=None):
        self._sys_info = SystemInfo()
        self._proto = None
        self._username = None
        self._host = None
        self._resource_path = None
        self._re_scp_url = re.compile(r'^([^@/\\:]+)@([^:]+):([^/].+)$')

        if url_str:
            (self._proto, self._username, self._host, self._resource_path) = self.parse_from_string(url_str)

    def get_protocol(self):
        return self._proto

    def get_hostname(self):
        return self._host

    def get_username(self):
        return self._username

    def get_resource_path(self, native=False):
        if self._resource_path:
            if native and (self._proto is not None) and (self._proto == 'file') and self._sys_info.is_windows():
                # Convert the resource path to a native file system path.
                if self._resource_path.startswith('/'):
                    assert len(self._resource_path) > 1
                    return os.path.normpath(self._resource_path[1:])
                elif re.match(r'^[a-zA-Z]:', self._resource_path):
                    assert len(self._resource_path) > 2
                    return os.path.normpath(self._resource_path)
        return self._resource_path

    def set_resource_path(self, res_path):
        self._resource_path = res_path

    def parse_from_string(self, url_str):
        proto = None
        username = None
        host = None
        resource_path = None

        re_match = self._re_scp_url.match(url_str)
        if re_match:
            # Split scp URL
            proto = 'ssh'
            username = re_match.group(1)
            host = re_match.group(2)
            resource_path = re_match.group(3)
        else:
            # Check for proto://
            re_match = re.match(r'^([a-z][^:/]+)://(.+)$', url_str)
            if re_match:
                proto = re_match.group(1)
                if proto == 'file':
                    # A relative path will be converted into an absolute path.
                    resource_path = re_match.group(2)
                    if self._sys_info.is_windows():
                        fs_path_native = os.path.abspath(os.path.normpath(resource_path))
                        resource_path = util.to_posix_path(fs_path_native)
                    else:
                        resource_path = os.path.abspath(resource_path)
                else:
                    host_res_path = re_match.group(2)
                    re_match = re.match(r'^([^/]+)/(.+)$', host_res_path)
                    if re_match:
                        host_part = re_match.group(1)
                        host = host_part
                        resource_path = re_match.group(2)
                        if proto == 'ssh':
                            # Analyze the host_part
                            re_match = re.match(r'([^@]+)@([^@]+)$', host_part)
                            if re_match:
                                username = re_match.group(1)
                                host = re_match.group(2)
                    else:
                        # Unsupported URL
                        raise Exception("Unsupported URL: {0}".format(url_str))
            else:
                # Assume a native file system path.
                proto = 'file'
                fpath_abs = os.path.abspath(url_str)
                # On windows backslashes are converted to slashes.
                resource_path = util.to_posix_path(fpath_abs)

        self._proto = proto
        self._username = username
        self._host = host
        if resource_path is None:
            self._resource_path = resource_path
        else:
            if proto != 'file':
                # Windows file URLs start with a driver letter and are not supported by posixpath.
                # Example file://C:/Users/user -> resource path C:/Users/user.
                resource_path = posixpath.normpath(resource_path)
            self._resource_path = resource_path

        return proto, username, host, resource_path

    def __str__(self):
        if self._proto == 'file':
            s = "{0}://{1}".format(self._proto, self._resource_path)
        elif (self._proto == 'ssh') and (self._username is not None):
            s = "{0}://{1}@{2}/{3}".format(self._proto, self._username, self._host, self._resource_path)
        else:
            s = "{0}://{1}/{2}".format(self._proto, self._host, self._resource_path)
        return s


class VcsUtil(object):

    def __init__(self, cmd_tracer=None):
        self._logger = logging.getLogger(__name__)
        self._sys_info = SystemInfo()
        self._remove_tmp_files = True
        if self._logger.isEnabledFor(logging.DEBUG):
            self._remove_tmp_files = False
        self._re_empty_line = re.compile(r'^\s*$')
        self._prog_name = os.path.basename(sys.argv[0])
        if cmd_tracer is None:
            self._cmd_tracer = VcsCommandTracer()
        else:
            self._cmd_tracer = cmd_tracer

    def get_subprocess_devnull(self):
        return self._sys_info.get_subprocess_devnull()

    def strip_empty_comment_lines(self, comment_lines):
        """Strip empty lines at the beginning and from the end of comment lines passed and return them."""

        if not comment_lines:
            return comment_lines
        while len(comment_lines) > 0:
            if self._re_empty_line.match(comment_lines[0]):
                comment_lines.pop(0)
            else:
                break
        while len(comment_lines) > 0:
            if self._re_empty_line.match(comment_lines[-1]):
                comment_lines.pop()
            else:
                break
        return comment_lines

    def save_comment_lines_tmpf(self, comment_lines, add_final_newline=True):
        assert len(comment_lines) > 0
        (tmp_fh, tmp_fname) = tempfile.mkstemp(text=True)
        tmp_file = os.fdopen(tmp_fh, 'w')
        tmp_file.write('\n'.join(comment_lines))
        if add_final_newline:
            # and one newline to terminate the last line.
            tmp_file.write('\n')
        tmp_file.close()
        return tmp_fname

    def remove_tmp_file(self, fname):
        if not os.path.exists(fname):
            return
        if self._remove_tmp_files:
            os.remove(fname)
        else:
            print("Keeping temporary file: ", fname)


class GitHelper(VcsUtil):

    def __init__(self, cmd_tracer=None):
        VcsUtil.__init__(self, cmd_tracer)
        self._logger = logging.getLogger(__name__)
        # git-svn-id: https://visvn.fe.hhi.de/svn/svn_CMakeBuild/tags/3.11.2-1@791 a315372b-2729-4e68-a986-494f0c48443b
        self._re_svn_import_id = re.compile(r'^(git-svn-id|svntogit-id):\s*([^@]+)@([0-9]+)\s+(\S+)')
        # Attribute _git_executable reserved for future extensions.
        self._git_executable = util.find_tool_on_path('git', must_succeed=True)
        self._git_version = self._discover_git_version()

    def _discover_git_version(self):
        git_argv = [self._git_executable, '--version']
        retv = subprocess.check_output(git_argv, universal_newlines=True)
        lines = retv.splitlines()
        re_match = re.search(r'[0-9.]+\.\d+', lines[0])
        if re_match:
            git_version = ver.version_tuple_from_str(re_match.group(0))
            return git_version
        else:
            raise Exception("git --version returned: '{0}' - version expression not understood.".format(retv))

    def get_git_executable(self):
        return self._git_executable

    def get_git_version(self):
        return self._git_version

    def get_command_history(self):
        return self._cmd_tracer.get_command_history()

    def clear_command_history(self):
        self._cmd_tracer.clear_command_history()

    def is_empty_repo(self, repo_dir=None):
        git_argv = [self._git_executable, 'show-ref', '-q']
        # Save initial current working directory
        cur_dir = os.getcwd()
        if repo_dir is None:
            assert os.path.exists(os.path.join(cur_dir, '.git'))
            self.trace_git_command(git_argv)
            retv = util.subproc_call_flushed(git_argv)
        else:
            assert os.path.exists(os.path.join(repo_dir, '.git'))
            os.chdir(repo_dir)
            self.trace_git_command(git_argv)
            retv = util.subproc_call_flushed(git_argv)
            # Recover initial current working directory
            os.chdir(cur_dir)
        return retv != 0

    def get_local_branches(self):
        local_branches = []
        git_argv = [self._git_executable, 'branch']
        self.trace_git_command(git_argv)
        retv = subprocess.check_output(git_argv, universal_newlines=True)
        re_expr = re.compile(r'^\*?\s+(\S+)')
        for line in retv.splitlines():
            l = line.rstrip()
            re_match = re_expr.match(l)
            if re_match:
                local_branches.append(re_match.group(1))
        return local_branches

    def get_remote_branches(self, repo='origin'):
        remote_branches = []
        git_argv = [self._git_executable, 'ls-remote', '--heads', repo]
        self.trace_git_command(git_argv)
        retv = subprocess.check_output(git_argv, universal_newlines=True)
        re_expr = re.compile(r'\S+\s+refs/heads/(\S+)')
        for line in retv.splitlines():
            l = line.rstrip().lstrip()
            re_match = re_expr.match(l)
            if re_match:
                remote_branches.append(re_match.group(1))
        return remote_branches

    def get_local_tags(self):
        local_tags = []
        git_argv = [self._git_executable, 'tag']
        self.trace_git_command(git_argv)
        retv = subprocess.check_output(git_argv, universal_newlines=True)
        for line in retv.splitlines():
            l = line.rstrip().lstrip()
            if re.match(r'^\S+', l):
                local_tags.append(l)
        return local_tags

    def get_remote_tags(self, repo='origin'):
        remote_tags = []
        git_argv = [self._git_executable, 'ls-remote', '--tags', '--refs', repo]
        self.trace_git_command(git_argv)
        retv = subprocess.check_output(git_argv, universal_newlines=True)
        # 5acba4bba0159c921d3e9de82a450d0e48092d1f	refs/tags/3.10.1-1
        re_expr = re.compile(r'\S+\s+refs/tags/(\S+)')
        for line in retv.splitlines():
            l = line.rstrip().lstrip()
            re_match = re_expr.match(l)
            if re_match:
                remote_tags.append(re_match.group(1))
        return remote_tags

    def get_submodule_paths(self):
        submd_paths = []
        if not os.path.exists('.gitmodules'):
            # If .gitmodules does not exist, we don't have any submodules.
            # If it does exist, we may have some provided the file is not empty.
            return submd_paths
        elif os.stat('.gitmodules').st_size == 0:
            # Check for empty .gitmodules
            return submd_paths

        if ver.version_compare(self._git_version, (2, 7)) < 0:
            raise Exception("git version {0} is too old, please update to 2.7 or higher".format(ver.version_tuple_to_str(self._git_version)))
        git_argv = [self._git_executable, 'submodule--helper', 'list']
        self.trace_git_command(git_argv)
        retv = subprocess.check_output(git_argv, universal_newlines=True)
        for line in retv.splitlines():
            if self._re_empty_line.match(line):
                continue
            l = line.rstrip().lstrip()
            fields = l.split()
            if fields:
                self._logger.debug("found submodule path {0}".format(fields[-1]))
                submd_paths.append(fields[-1])
        return submd_paths

    def get_tag_log(self, tag, strip_git_svn_id=False, strip_empty_lines=True):
        tag_log = []
        git_argv = [self._git_executable, 'tag', '-l', '--format=%(contents)', tag]
        self.trace_git_command(git_argv)
        retv = subprocess.check_output(git_argv, universal_newlines=True)
        log_lines = retv.splitlines()
        for line in log_lines:
            if strip_git_svn_id and (line.startswith('git-svn-id:') or line.startswith('svntogit-id:')):
                continue
            tag_log.append(line)
        if strip_empty_lines:
            tag_log = self.strip_empty_comment_lines(tag_log)
        return tag_log

    def get_commit_hash(self, tag=None):
        git_argv = [self._git_executable, 'rev-list', '-n', '1']
        if tag is None:
            git_argv.append('HEAD')
        else:
            git_argv.append(tag)
        self.trace_git_command(git_argv)
        retv = subprocess.check_output(git_argv, universal_newlines=True)
        return retv.lstrip().rstrip()

    def parse_svn_import_id(self, comment_lines):
        for line in comment_lines:
            re_match = self._re_svn_import_id.match(line)
            if re_match:
                svn_url = re_match.group(2)
                svn_rev = int(re_match.group(3))
                svn_repo_uuid = re_match.group(4)
                return svn_url, svn_rev, svn_repo_uuid
        return None

    def get_wc_commit_log(self, strip_git_svn_id=True, strip_empty_lines=True):
        commit_log = []
        # List commit message preserving newlines.
        git_argv = [self._git_executable, 'log', '--pretty=tformat:%B', '-n', '1']
        self.trace_git_command(git_argv)
        retv = subprocess.check_output(git_argv, universal_newlines=True)
        log_lines = retv.splitlines()
        for line in log_lines:
            if strip_git_svn_id and line.startswith('git-svn-id:'):
                continue
            commit_log.append(line)
        if strip_empty_lines:
            # Strip leading and trailing empty lines
            commit_log = self.strip_empty_comment_lines(commit_log)
        return commit_log

    def trace_git_command(self, git_argv):
        self._cmd_tracer.trace_command(git_argv)

    def get_latest_cmakebuild_tag(self, cmakebuild_repo):
        cmakebuild_tags = self.get_remote_tags(cmakebuild_repo)
        cmakebuild_version_list = []
        cmakebuild_release_tag_map = {}
        re_expr_release_tag = re.compile(r'^[0-9.-]+$')
        for tag in cmakebuild_tags:
            if re_expr_release_tag.match(tag):
                # Release tag found
                cmakebuild_version = ver.version_tuple_from_str(tag)
                cmakebuild_version_list.append(cmakebuild_version)
                cmakebuild_version_str = ver.version_tuple_to_str(cmakebuild_version)
                # Adds a mapping from a normalized version string to the tag: e.g. 3.12.3.4 -> 3.12.3-4
                cmakebuild_release_tag_map[cmakebuild_version_str] = tag
            else:
                self._logger.info("ignoring CMakeBuild tag {} - it does not match the release pattern.".format(tag))
        if not cmakebuild_version_list:
            raise Exception("No CMakeBuild release tag found.")
        cmakebuild_version_list = ver.version_list_sort(cmakebuild_version_list)
        cmakebuild_version = cmakebuild_version_list[-1]
        cmakebuild_tag = cmakebuild_release_tag_map[ver.version_tuple_to_str(cmakebuild_version)]
        self._logger.info("selecting CMakeBuild tag {}".format(cmakebuild_tag))
        return cmakebuild_tag


class SvnProperties(object):

    def __init__(self):
        self._svn_prop_list = []
        # key = svn property, value = list of property values
        self._svn_prop_value_dict = {}

    def is_empty(self):
        return len(self._svn_prop_list) == 0

    def clear(self):
        self._svn_prop_list = []
        self._svn_prop_value_dict = {}

    def add_property(self, prop, prop_values):
        # Each property must have at least one associated value.
        assert len(prop_values) >= 1
        self._svn_prop_list.append(prop)
        self._svn_prop_value_dict[prop] = prop_values

    def update_property(self, prop, prop_values):
        # Each property must have at least one associated value.
        assert len(prop_values) >= 1
        assert self.has_property(prop)
        self._svn_prop_value_dict[prop] = prop_values

    def get_property_list(self):
        return self._svn_prop_list

    def has_property(self, prop):
        if prop in self._svn_prop_value_dict:
            return True
        else:
            return False

    def get_property(self, prop):
        return self._svn_prop_value_dict[prop]

    def remove_property(self, prop):
        if not self.has_property(prop):
            return
        self._svn_prop_list.remove(prop)
        self._svn_prop_value_dict.pop(prop)

    def __str__(self):
        if self.is_empty():
            return ''
        lines = []
        for prop in self._svn_prop_list:
            # lines.append("{0:>2}{1}".format(' ', prop))
            lines.append(prop)
            for prop_value in self._svn_prop_value_dict[prop]:
                # lines.append("{0:>4}{1}".format(' ', prop_value))
                lines.append("{0:>2}{1}".format(' ', prop_value))
            # SVN seems to be in favor of a newline terminating multivalue properties
            if len(self._svn_prop_value_dict[prop]) > 1:
                lines.append('\n')
        # Serialize svn properties into a single string and avoid a duplicate newline after
        # a multivalue property; e.g.
        props_str = ''
        for ln in lines:
            if ln != '\n':
                props_str += "{}\n".format(ln)
            else:
                # a newline value must not be duplicated.
                props_str += ln
        return props_str


class SvnPropFileParser(object):

    def __init__(self):
        self._logger = logging.getLogger(__name__)

    def parse_property_file(self, filenm):
        if not os.path.exists(filenm):
            raise Exception("SVN property file {0} does not exist.".format(filenm))
        svn_prop_lines = []
        with open(filenm) as svnpropf:
            for line in svnpropf:
                l = line.rstrip()
                svn_prop_lines.append(l)
        svn_prop = self.parse_property_file_content(svn_prop_lines)
        return svn_prop

    def parse_gitignore_file(self, filenm):
        if not os.path.exists(filenm):
            raise Exception("gitignore file {0} does not exist.".format(filenm))
        gitignore_lines = []
        with open(filenm) as svnpropf:
            for line in svnpropf:
                l = line.rstrip()
                gitignore_lines.append(l)
        svn_prop = self.parse_gitignore_file_content(gitignore_lines)
        return svn_prop

    def parse_property_file_content(self, svn_prop_lines):
        svn_prop = SvnProperties()
        prop_name = None
        prop_value_list = []
        re_prop_name = re.compile(r'^(\s*)(\S+)')
        re_discard_line = re.compile(r'(^\s*$)|(^#.*$)|(^Properties on.*$)')
        line_cnt = 1
        while len(svn_prop_lines) > 0:
            line = svn_prop_lines[0]
            if not re_discard_line.match(line):
                # print(line)
                if prop_name is None:
                    # This line is supposed to contain a property like svn:ignore, leading ws is supported.
                    re_match = re_prop_name.match(line)
                    if re_match:
                        leading_ws_prop_name = re_match.group(1)
                        prop_name = re_match.group(2)
                        prop_value_list = []
                    else:
                        raise Exception("SVN property parsing error at line {0:d}: {1}".format(line_cnt, line))
                else:
                    # Next property value or next property name?
                    if line.startswith(' ' + leading_ws_prop_name):
                        prop_value = line.lstrip()
                        prop_value_list.append(prop_value)
                    else:
                        if prop_value_list:
                            svn_prop.add_property(prop_name, prop_value_list)
                        else:
                            self._logger.debug("ignoring empty SVN property: {0}".format(prop_name))
                        # print("new property: {0} -> {1}".format(prop_name, ' '.join(prop_value_list)))
                        prop_name = None
                        prop_value_list = []
                        # This line must be reparsed, it's the next property.
                        continue
            else:
                self._logger.debug("ignoring line: {0}".format(line))
            svn_prop_lines.pop(0)
            line_cnt = line_cnt + 1

        if prop_name is not None:
            if prop_value_list:
                svn_prop.add_property(prop_name, prop_value_list)
                # print("new property: {0} -> {1}".format(prop_name, ' '.join(prop_value_list)))
        #if not svn_prop.is_empty():
        #    print(svn_prop)
        return svn_prop

    def parse_gitignore_file_content(self, gitignore_lines, svn_prop=None):
        svn_ignores = []
        re_discard_line = re.compile(r'(^\s*$)|(^#.*$)')
        re_top_folder = re.compile(r'^/([^/]+)/$')
        re_folder = re.compile(r'^([^/]+)/$')

        for line in gitignore_lines:
            if re_discard_line.match(line):
                continue
            ln_trimmed = line.lstrip().rstrip()
            if '/' in ln_trimmed:
                re_match = re_top_folder.match(ln_trimmed)
                if re_match:
                    svn_ignores.append(re_match.group(1))
                    continue
                re_match = re_folder.match(ln_trimmed)
                if re_match:
                    svn_ignores.append(re_match.group(1))
                    continue
            else:
                svn_ignores.append(ln_trimmed)

        if svn_ignores:
            if svn_prop is None:
                svn_prop = SvnProperties()
            svn_prop.add_property('svn:ignore', svn_ignores)
        return svn_prop


class SvnHelper(VcsUtil):

    def __init__(self, cmd_tracer=None):
        VcsUtil.__init__(self, cmd_tracer)
        self._logger = logging.getLogger(__name__)
        self._dry_run = False

        self._svn_repo_std_layout = True
        # Not really useful right now but added for future extensions to deal with platforms
        # which don't have svn on the search path (e.g. windows or macOS).
        self._svn_executable = util.find_tool_on_path('svn', must_succeed=True)
        self._svnadmin_executable = self._find_svnadmin()
        self._svnmucc_executable = self._find_svnmucc()

    def _find_svnadmin(self):
        if os.path.isabs(self._svn_executable):
            svn_prog = self._svn_executable
        else:
            svn_prog = util.find_tool_on_path('svn', must_succeed=True)
        svn_dir = os.path.dirname(svn_prog)
        if self._sys_info.is_windows():
            svnadmin = os.path.join(svn_dir, 'svnadmin.{0}'.format('exe'))
        else:
            svnadmin = os.path.join(svn_dir, 'svnadmin')
        return svnadmin

    def _find_svnmucc(self):
        if os.path.isabs(self._svn_executable):
            svn_prog = self._svn_executable
        else:
            svn_prog = util.find_tool_on_path('svn', must_succeed=True)
        svn_dir = os.path.dirname(svn_prog)
        if self._sys_info.is_windows():
            svnmucc = os.path.join(svn_dir, 'svnmucc.{0}'.format('exe'))
        else:
            svnmucc = os.path.join(svn_dir, 'svnmucc')
        if not os.path.exists(svnmucc):
            svnmucc = None
        return svnmucc

    def set_dry_run(self, dry_run=True):
        self._dry_run = dry_run

    def get_svn_executable(self):
        return self._svn_executable

    def get_svnadmin_executable(self):
        return self._svnadmin_executable

    def create_empty_repository(self, repo_path, std_layout=True):
        repo_path = os.path.normpath(os.path.abspath(repo_path))
        if os.path.exists(repo_path):
            raise Exception("SVN repository path {} already exists.".format(repo_path))
        repo_dir = os.path.dirname(repo_path)
        if not os.path.exists(repo_dir):
            os.makedirs(repo_dir)
        svn_argv = [self._svnadmin_executable, 'create', repo_path]
        self.trace_svn_command(svn_argv)
        if self._dry_run:
            self._logger.warning("dry run: {0}".format(' '.join(svn_argv)))
        else:
            util.subproc_check_call_flushed(svn_argv)
        if std_layout:
            svn_repo_path_posix = util.to_posix_path(repo_path)
            for pth in ['trunk', 'branches', 'tags']:
                svn_url = "file://{0}/{1}".format(svn_repo_path_posix, pth)
                svn_argv = [self._svn_executable, 'mkdir', '-m', 'creating folder {}'.format(pth), svn_url]
                self.trace_svn_command(svn_argv)
                if self._dry_run:
                    self._logger.warning("dry run: {0}".format(' '.join(svn_argv)))
                else:
                    util.subproc_check_call_flushed(svn_argv)

    def has_svnmucc(self):
        return self._svnmucc_executable is not None

    def get_svnmucc_executable(self):
        return self._svn_executable

    def get_command_history(self):
        return self._cmd_tracer.get_command_history()

    def clear_command_history(self):
        self._cmd_tracer.clear_command_history()

    def get_tag_url(self, svn_repo, tag):
        assert self._svn_repo_std_layout
        return "{0}/tags/{1}".format(svn_repo, tag)

    def get_branch_url(self, svn_repo, branch):
        assert self._svn_repo_std_layout
        if branch == 'trunk':
            return "{0}/{1}".format(svn_repo, branch)
        else:
            return "{0}/branches/{1}".format(svn_repo, branch)

    def get_tags(self, svn_repo, with_rev=False):
        tags = self.get_svn_folders(svn_repo + '/tags', with_rev)
        return tags

    def get_latest_tag(self, svn_repo):
        tags = self.get_tags(svn_repo)
        if not tags:
            return None
        tag_rev_list = []
        for tag in tags:
            svn_tag_url = self.get_tag_url(svn_repo, tag)
            tag_rev = self.get_svn_rev(svn_tag_url)
            tag_rev_list.append(tuple([tag, tag_rev]))
        # Sort by revision
        tag_rev_list = sorted(tag_rev_list, key=lambda tag_rev: tag_rev[1])
        (latest_tag, tag_rev) = tag_rev_list[-1]
        return latest_tag

    def get_branches(self, svn_repo, with_rev=False):
        branches = self.get_svn_folders(svn_repo + '/branches', with_rev)
        return branches

    def get_svn_folders(self, svn_url, with_rev=False):
        svn_folders = []
        svn_argv = [self._svn_executable, 'list', svn_url]
        self.trace_svn_command(svn_argv)
        retv = subprocess.check_output(svn_argv, universal_newlines=True)
        re_trailing_slash = re.compile(r'^(.+)/$')
        for line in retv.splitlines():
            l = line.rstrip().lstrip()
            re_match = re_trailing_slash.match(l)
            if re_match:
                svn_folder = re_match.group(1)
                if with_rev:
                    rev = self.get_svn_rev("{0}/{1}".format(svn_url, svn_folder))
                    svn_folder += "@{0:d}".format(rev)
                svn_folders.append(svn_folder)
        return svn_folders

    def get_svn_rev(self, svn_url):
        svn_argv = [self._svn_executable, 'info', '--show-item', 'last-changed-revision', svn_url]
        self.trace_svn_command(svn_argv)
        retv = subprocess.check_output(svn_argv, universal_newlines=True)
        retv = retv.rstrip()
        rev = int(retv)
        return rev

    def get_repo_uuid(self, svn_repo):
        svn_argv = [self._svn_executable, 'info', '--show-item', 'repos-uuid', svn_repo]
        self.trace_svn_command(svn_argv)
        retv = subprocess.check_output(svn_argv, universal_newlines=True)
        retv = retv.lstrip().rstrip()
        return retv

    def get_svn_log_msg(self, svn_url):
        # svn log -l 1 https://visvn.fe.hhi.de/svn/svn_CMakeBuild/tags/3.11.0-1
        svn_argv = [self._svn_executable, 'log', '-l', '1', svn_url]
        self.trace_svn_command(svn_argv)
        retv = subprocess.check_output(svn_argv, universal_newlines=True)
        comment_lines = retv.splitlines()
        assert len(comment_lines) >= 3
        assert comment_lines[0].startswith('---')
        comment_lines.pop(0)
        # Drop rXXXX ...
        comment_lines.pop(0)
        assert comment_lines[-1].startswith('---')
        comment_lines.pop()
        comment_lines = self.strip_empty_comment_lines(comment_lines)
        return comment_lines

    def svn_save_property_values_tmpf(self, prop_values):
        tmp_fname = self.save_comment_lines_tmpf(prop_values)
        return tmp_fname

    def svn_save_properties(self, svn_props, fname, header_lines=None):
        svn_props_serialized = str(svn_props)
        with open(fname, "w") as propf:
            if header_lines:
                header = '\n'.join(header_lines)
                propf.write(header)
                propf.write('\n')
            propf.write(svn_props_serialized)

    def svn_read_properties(self, svn_res_path):
        svn_argv = [self._svn_executable, 'proplist', '--verbose', svn_res_path]
        self.trace_svn_command(svn_argv)
        retv = subprocess.check_output(svn_argv, universal_newlines=True)
        prop_lines = retv.splitlines()
        prop_file_parser = SvnPropFileParser()
        return prop_file_parser.parse_property_file_content(prop_lines)

    def svn_commit_wc(self, comment_lines, wc_dir):
        tmp_fname = self.save_comment_lines_tmpf(comment_lines)
        svn_argv = [self._svn_executable, 'ci', '-F', tmp_fname, wc_dir]
        self.trace_svn_command(svn_argv)
        if self._dry_run:
            self._logger.warning("dry run: {0}".format(' '.join(svn_argv)))
        else:
            util.subproc_check_call_flushed(svn_argv)

    def svn_import_wc(self, comment_lines, wc_dir, svn_url, global_ignores=None, quiet=False):
        # svn import -F <file> --auto-props --config-option config:miscellany:global-ignores=.git wc_dir svn_url
        # e.g. svn_url = https://visvn.fe.hhi.de/svn/svn_CMakeBuildClone/branches/imports/3.10.1-1
        tmp_fname = self.save_comment_lines_tmpf(comment_lines)
        svn_argv = [self._svn_executable, 'import', '-F', tmp_fname, '--auto-props']
        # Add any global ignores.
        if global_ignores:
            svn_argv.extend(['--config-option', 'config:miscellany:global-ignores=' + ' '.join(global_ignores)])
        if quiet:
            svn_argv.append('-q')
        # finalize the command line
        svn_argv.extend([wc_dir, svn_url])
        self.trace_svn_command(svn_argv)
        if self._dry_run:
            self._logger.warning("dry run: {0}".format(' '.join(svn_argv)))
        else:
            util.subproc_check_call_flushed(svn_argv)

    def svn_set_properties_url(self, svn_url, svn_props, work_dir=None):
        if svn_props.is_empty():
            return
        if self.has_svnmucc():
            self._svn_set_properties_url_svnmucc(svn_url, svn_props)
        else:
            raise Exception("svn_set_properties_url() requires svnmucc which is not available.")

    def _svn_set_properties_url_svnmucc(self, svn_url, svn_props):
        if svn_props.is_empty():
            return
        tmp_files = []
        try:
            svn_argv = [self._svnmucc_executable]
            (svn_url_prefix, svn_path) = self.svn_split_url(svn_url)
            for prop in svn_props.get_property_list():
                prop_values = svn_props.get_property(prop)
                if len(prop_values) > 1:
                    tmp_fname = self.svn_save_property_values_tmpf(prop_values)
                    tmp_files.append(tmp_fname)
                    svn_argv.extend(['propsetf', prop, tmp_fname, svn_path])
                else:
                    svn_argv.extend(['propset', prop, prop_values[0], svn_path])
            svn_argv.append('-m')
            svn_argv.append("Properties inserted by {0}".format(self._prog_name))
            svn_argv.extend(['-U', svn_url_prefix])
            self.trace_svn_command(svn_argv)
            if self._dry_run:
                self._logger.warning("dry run: {0}".format(' '.join(svn_argv)))
            else:
                util.subproc_check_call_flushed(svn_argv)
        finally:
            for pth in tmp_files:
                if os.path.exists(pth):
                    os.remove(pth)

    def svn_set_properties_wc(self, svn_props, work_dir):
        if svn_props.is_empty():
            return
        tmp_files = []
        try:
            for prop in svn_props.get_property_list():
                prop_values = svn_props.get_property(prop)
                if len(prop_values) > 1:
                    tmp_fname = self.svn_save_property_values_tmpf(prop_values)
                    tmp_files.append(tmp_fname)
                    svn_argv = [self._svn_executable, 'propset', prop, '-F', tmp_fname, work_dir]
                else:
                    svn_argv = [self._svn_executable, 'propset', prop, prop_values[0], work_dir]
                self.trace_svn_command(svn_argv)
                if self._dry_run:
                    self._logger.warning("dry run: {0}".format(' '.join(svn_argv)))
                else:
                    util.subproc_check_call_flushed(svn_argv)
        finally:
            for pth in tmp_files:
                if os.path.exists(pth):
                    os.remove(pth)

    def svn_remote_copy(self, comment_lines, svn_url_src, svn_url_dst, create_intermediate_folders=True):
        tmp_fname = self.save_comment_lines_tmpf(comment_lines)
        svn_argv = [self._svn_executable, 'copy', '-F', tmp_fname]
        if create_intermediate_folders:
            svn_argv.append('--parents')
        svn_argv.extend([svn_url_src, svn_url_dst])
        self.trace_svn_command(svn_argv)
        if self._dry_run:
            self._logger.warning("dry run: {0}".format(' '.join(svn_argv)))
        else:
            util.subproc_check_call_flushed(svn_argv)
        self.remove_tmp_file(tmp_fname)

    def svn_remote_rm(self, svn_url, comment_lines):
        tmp_fname = self.save_comment_lines_tmpf(comment_lines)
        svn_argv = [self._svn_executable, 'rm', '-F', tmp_fname, svn_url]
        self.trace_svn_command(svn_argv)
        if self._dry_run:
            self._logger.warning("dry run: {0}".format(' '.join(svn_argv)))
        else:
            util.subproc_check_call_flushed(svn_argv)
        self.remove_tmp_file(tmp_fname)

    def svn_split_url(self, svn_url):
        svn_url_norm = svn_url.rstrip('/')
        re_match = re.match(r'^(.+)/([^/]+)$', svn_url_norm)
        if re_match:
            svn_root_url = re_match.group(1)
            svn_url_path = "/{}".format(re_match.group(2))
        else:
            raise Exception("SVN URL {0} not supported.".format(svn_url))
        return svn_root_url, svn_url_path

    def trace_svn_command(self, svn_argv):
        self._cmd_tracer.trace_command(svn_argv)
