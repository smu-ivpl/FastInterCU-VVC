
from __future__ import print_function

import argparse
import logging
import os
import re
import subprocess
import sys

import pyhhi.build.common.util as util
import pyhhi.build.common.ver as ver
import pyhhi.build.vcsutil as vcsutil

from pyhhi.build.common.system import SystemInfo
from pyhhi.build.common.error import InvalidInputParameterError


class CMakeBuildUpdateAppParams(object):

    def __init__(self):
        self.top_dir = None
        self.update_prepare = True
        self.cmakebuild_repo = 'git@vigitlab.fe.hhi.de:git/CMakeBuildCore.git'
        # A CMakeBuild tag of None means latest tag.
        self.cmakebuild_tag = None


class CMakeBuildUpdateApp(object):

    def __init__(self):
        self._logger = logging.getLogger(__name__)
        self._sys_info = SystemInfo()
        self._prog_name = os.path.basename(sys.argv[0])
        self._prog_name_full = os.path.normpath(os.path.abspath(sys.argv[0]))
        self._list_summary = True
        self._summary_lines = []
        self._git_helper = vcsutil.GitHelper()
        self._git_executable = self._git_helper.get_git_executable()

        # List of python modules this script depends on.
        self._py_mod_list = ['pyhhi.build.app.cmbldup',
                             'pyhhi.build.vcsutil',
                             'pyhhi.build.common.cmbldver',
                             'pyhhi.build.common.error',
                             'pyhhi.build.common.system',
                             'pyhhi.build.common.util',
                             'pyhhi.build.common.ver']

        self._usage = """
%(prog)s [options] [top_dir]

%(prog)s updates a CMakeBuild Git subtree embedded inside a Git standard workspace.  
The update procedure requires a clean Git working directory. Unfinished work must be 
saved somehow; e.g. use git stash .. or anything else.

The optional positional argument allows you to update an existing Git workspace out-of-tree.
"""

        self._epilog = """
"""

    def __call__(self, argv=None):
        if argv is None:
            self.main(sys.argv[1:])
        else:
            self.main(argv)

    def main(self, argv):
        params = self._parse_command_line(argv)
        self.cmakebuild_update(params)

    def cmakebuild_update(self, params):

        initial_work_dir = os.getcwd()
        # (Re)create GitHelper in case the log level has been changed.
        self._git_helper = vcsutil.GitHelper()
        params = self._check_params(params)
        if params.top_dir is None:
            params.top_dir = util.get_top_dir()
        self._summary_lines = []
        if params.update_prepare:
            print("{}: preparing CMakeBuild update ...".format(self._prog_name))

            cmakebuild_version_current = self._get_current_cmakebuild_version(params.top_dir)
            if cmakebuild_version_current is None:
                print("{}: no existing CMakeBuild version found.".format(params.top_dir))

            if not self._is_workspace_clean(params.top_dir):
                return

            if params.cmakebuild_tag is None:
                cmakebuild_tag = self._git_helper.get_latest_cmakebuild_tag(params.cmakebuild_repo)

                if cmakebuild_version_current and (ver.version_compare(cmakebuild_version_current, ver.version_tuple_from_str(cmakebuild_tag)) == 0):
                    print("{}: CMakeBuild is up to date, nothing to be done.".format(params.top_dir))
                    return
            else:
                # Validate CMakeBuild tag
                cmakebuild_tags = self._git_helper.get_remote_tags(params.cmakebuild_repo)
                if params.cmakebuild_tag not in cmakebuild_tags:
                    raise InvalidInputParameterError("requested tag {0} does not exists in {1}.".format(params.cmakebuild_tag, params.cmakebuild_repo))
                cmakebuild_tag = params.cmakebuild_tag
                if cmakebuild_version_current and (ver.version_compare(ver.version_tuple_from_str(cmakebuild_tag), cmakebuild_version_current) <= 0):
                    print("{}: CMakeBuild is up to date, nothing to be done.".format(params.top_dir))
                    return

            install_dir = os.path.join(params.top_dir, 'build', 'cmakebuild_update')
            self._install_self(install_dir)

            update_script = os.path.join(install_dir, self._prog_name)
            # Prepare execv argument vector to call self with different arguments using the current python executable.
            if self._sys_info.is_windows():
                child_args = [self._sys_info.get_short_path(self._sys_info.get_python_executable())]
            else:
                child_args = [self._sys_info.get_python_executable()]
            child_args.extend([update_script, '--update'])
            if cmakebuild_tag:
                child_args.extend(['-t', cmakebuild_tag])
            if params.cmakebuild_repo:
                child_args.extend(['--cmakebuild-repo', params.cmakebuild_repo])

            # Add currrent log option to child_args[] to propagate the current log option to the child process.
            log_level = self._logger.getEffectiveLevel()
            log_level_str = None
            if log_level == logging.DEBUG:
                log_level_str = 'debug'
            elif log_level == logging.INFO:
                log_level_str = 'info'
            elif log_level == logging.WARNING:
                log_level_str = 'warning'
            elif log_level == logging.ERROR:
                log_level_str = 'error'
            elif log_level == logging.CRITICAL:
                log_level_str = 'critical'
            if log_level_str is not None:
                child_args.append('--log={}'.format(log_level_str))

            child_args.append(params.top_dir)
            os.chdir(params.top_dir)
            if self._sys_info.is_windows():
                # Unfortunately, there are issues with os.execv() on Windows and therefore a subprocess call is used instead.
                util.subproc_check_call_flushed(child_args)
            else:
                # execv() is preferred as the child is likely to remove python files just being executed
                # by the current python process.
                os.execv(child_args[0], child_args)
        else:
            try:
                print("{}: updating CMakeBuild in {} to {}".format(self._prog_name, params.top_dir, params.cmakebuild_tag))
                os.chdir(params.top_dir)
                if not self._is_workspace_clean():
                    return

                self._append_item_to_summary("Git workspace:", params.top_dir)
                if self._list_summary:
                    cmakebuild_version_current = self._get_current_cmakebuild_version(params.top_dir)
                    if cmakebuild_version_current is not None:
                        self._append_item_to_summary("Current CMakeBuild version:", "{}-{:d}".format(ver.version_tuple_to_str(cmakebuild_version_current[:3]), cmakebuild_version_current[-1]))
                    self._append_item_to_summary("New CMakeBuild version:", params.cmakebuild_tag)

                # Inventory of CMakeBuild to remember existing directories to be removed later on.
                (git_cmakebuild_dirs, cmakebuild_dirs) = self._get_git_cmakebuild_dirs(params.top_dir)
                if params.cmakebuild_tag in git_cmakebuild_dirs:
                    git_cmakebuild_dirs.remove(params.cmakebuild_tag)
                if params.cmakebuild_tag in cmakebuild_dirs:
                    cmakebuild_dirs.remove(params.cmakebuild_tag)
                # print("found existing Git CMakeBuild subdirs:", git_cmakebuild_dirs)
                # print("found existing CMakeBuild subdirs:", cmakebuild_dirs)

                # Add new CMakeBuild subtree
                self._add_cmakebuild_to_git_repo(params.cmakebuild_tag, params.cmakebuild_repo)

                # Update .svnimportprops in case is does exist.
                svnimportprops_file = os.path.join(params.top_dir, '.svnimportprops')
                svnimportprops_modified = self._update_svnimportprops(svnimportprops_file, params.cmakebuild_tag)

                for dname in git_cmakebuild_dirs:
                    git_argv = [self._git_executable, 'rm', '-r', os.path.join('CMakeBuild', dname)]
                    util.subproc_check_call_flushed(git_argv)
                if svnimportprops_modified or git_cmakebuild_dirs:
                    # Need a git commit if '.svnimportprops' has been updated or git rm -r has been launched.
                    git_comment = []
                    if git_cmakebuild_dirs:
                        git_comment.append("rm -r CMakeBuild/{}".format(','.join(git_cmakebuild_dirs)))
                    if svnimportprops_modified:
                        git_comment.append("modifying {}".format('.svnimportprops'))
                    git_argv = [self._git_executable, 'commit', '-am', '{}: {}'.format(self._prog_name, ', '.join(git_comment))]
                    util.subproc_check_call_flushed(git_argv)

                # Check again to get rid of any python cache files
                cmakebuild_dirs.extend(git_cmakebuild_dirs)
                for dname in cmakebuild_dirs:
                    dname_path = os.path.join(params.top_dir, 'CMakeBuild', dname)
                    if os.path.exists(dname_path):
                        util.rmtree(dname_path)

                print("{}: finished updating CMakeBuild in {} to {}".format(self._prog_name, params.top_dir, params.cmakebuild_tag))

                if self._list_summary:
                    print("\n{0:^80}".format("Summary"))
                    print("{0:^80}".format("======="))
                    for line in self._summary_lines:
                        print(line)
            finally:
                pass
        os.chdir(initial_work_dir)

    def _parse_command_line(self, argv):

        params = CMakeBuildUpdateAppParams()

        parser = argparse.ArgumentParser(usage=self._usage, epilog=self._epilog, formatter_class=argparse.RawDescriptionHelpFormatter)

        parser.add_argument("top_dir", action="store", nargs='?', default=None,
                            help="specifies a Git workspace to work on. By default the scripts installation path will be used to deduce the workspace.")

        util.app_args_add_log_level(parser)

        parser.add_argument("-t", "--tag", action="store", dest="cmakebuild_tag", metavar="TAG",
                            help="specifies a CMakeBuild tag to be used. If omitted the latest release tag will be used.")

        parser.add_argument("--cmakebuild-repo", action="store", dest="cmakebuild_repo", metavar="REPO", default=params.cmakebuild_repo,
                            help="specifies an alternative CMakeBuild repository overriding %(default)s." )

        parser.add_argument("--update", action="store_true", dest="update", default=False,
                            help="execute cmakebuild update, internal option reserved for the os.execv() machinery.")

        args = parser.parse_args(argv)

        # configure the python logger
        util.app_configure_logging(args.log_level)

        if args.top_dir:
            params.top_dir = os.path.normpath(os.path.abspath(args.top_dir))
        if args.cmakebuild_tag:
            params.cmakebuild_tag = args.cmakebuild_tag
        if args.cmakebuild_repo:
            params.cmakebuild_repo = args.cmakebuild_repo
        if args.update:
            params.update_prepare = False

        return params

    def _check_params(self, params):
        params.cmakebuild_repo = self._check_git_url(params.cmakebuild_repo)
        if params.top_dir:
            if not os.path.exists(params.top_dir):
                raise InvalidInputParameterError("Git workspace {} does not exist.".format(params.top_dir))
            if not os.path.exists(os.path.join(params.top_dir, '.git')):
                raise InvalidInputParameterError("Workspace {} does not seem to be a Git workspace.".format(params.top_dir))
        return params

    def _check_git_url(self, git_url):
        vcs_url = vcsutil.VcsUrl(git_url)
        proto = vcs_url.get_protocol()
        if proto in ['ssh', 'https']:
            # Normalize a remote Git URL to end with .git even if specified without the extension.
            res_path = vcs_url.get_resource_path()
            if not res_path.endswith('.git'):
                vcs_url.set_resource_path("{}.git".format(res_path))
        else:
            raise InvalidInputParameterError("Git URL {0} is not supported yet.".format(git_url))
        # Normalize all URLs.
        git_url = str(vcs_url)
        self._logger.debug("returning url={0}".format(git_url))
        return git_url

    def _install_self(self, install_dir):
        script_installer = self._create_script_installer()
        # print("py_mod_list:", self._py_mod_list)
        script_installer.install_script(install_dir, self._prog_name_full, self._py_mod_list)

    def _create_script_installer(self):
        import pyhhi.build.common.bldtools
        script_installer = pyhhi.build.common.bldtools.BuildScriptInstaller(verbose=True)
        return script_installer

    def _get_current_cmakebuild_version(self, top_dir):
        assert os.path.exists(top_dir)
        cmakebuild_top_dir = os.path.join(top_dir, 'CMakeBuild')
        if not os.path.exists(cmakebuild_top_dir):
            return None
        re_version_dir = re.compile(r'^\d+[0-9.]+-\d+$')
        version_list = []
        for fname in os.listdir(cmakebuild_top_dir):
            if os.path.isdir(os.path.join(cmakebuild_top_dir, fname)):
                if re_version_dir.match(fname) and os.path.exists(os.path.join(cmakebuild_top_dir, fname, 'CMakeBuild', 'bin', 'cmake.py')):
                    version_list.append(ver.version_tuple_from_str(fname))
        if not version_list:
            return None
        if len(version_list) > 1:
            raise InvalidInputParameterError("Workspace {} contains multiple CMakeBuild versions, please update manually.".format(top_dir))
        self._logger.debug("found CMakeBuild {} in {}".format(ver.version_tuple_to_str(version_list[0]), top_dir))
        return version_list[0]

    def _get_git_cmakebuild_dirs(self, top_dir):
        assert os.path.exists(top_dir)
        cmakebuild_top_dir = os.path.join(top_dir, 'CMakeBuild')
        if not os.path.exists(cmakebuild_top_dir):
            return []

        re_version_dir = re.compile(r'^\d+[0-9.]+-\d+$')
        git_cmakebuild_version_dirs = []
        cmakebuild_version_dirs = []
        for fname in os.listdir(cmakebuild_top_dir):
            if os.path.isdir(os.path.join(cmakebuild_top_dir, fname)):
                if re_version_dir.match(fname):
                    git_argv = [self._git_executable, 'ls-files', os.path.join('CMakeBuild', fname)]
                    retv = subprocess.check_output(git_argv, universal_newlines=True)
                    retv = retv.lstrip().rstrip()
                    if retv != '':
                        git_cmakebuild_version_dirs.append(fname)
                    else:
                        cmakebuild_version_dirs.append(fname)
        return git_cmakebuild_version_dirs, cmakebuild_version_dirs

    def _is_workspace_clean(self, top_dir=None):
        if top_dir:
            initial_work_dir = os.getcwd()
            os.chdir(top_dir)
        else:
            initial_work_dir = None
            top_dir = os.getcwd()
        git_argv = [self._git_executable, 'status', '-b', '--porcelain']
        retv = subprocess.check_output(git_argv, universal_newlines=True)
        retv = retv.lstrip().rstrip()
        status_lines = retv.splitlines()
        if status_lines[0].startswith('## HEAD (no branch)'):
            raise InvalidInputParameterError("Git workspace {} has detached HEAD, not upgradable.".format(top_dir))
        if len(status_lines) > 1:
            raise InvalidInputParameterError("Git workspace {} is not clean, please check.".format(top_dir))
        if initial_work_dir:
            os.chdir(initial_work_dir)
        return True

    def _add_cmakebuild_to_git_repo(self, cmakebuild_tag, cmakebuild_repo):

        self._logger.info("using CMakeBuild tag {} as subtree.".format(cmakebuild_tag))
        #
        # Extra annoying step to convince git subtree the working dir is clean
        #
        git_argv = [self._git_executable, 'checkout']
        self._git_helper.trace_git_command(git_argv)
        util.subproc_check_call_flushed(git_argv)

        subtree_prefix = "CMakeBuild/{0}".format(cmakebuild_tag)
        git_argv = [self._git_executable, 'subtree', 'add', '-P', subtree_prefix, '--squash', '-m', "{0}: creates subtree -P {1} tag={2}".format(self._prog_name, subtree_prefix, cmakebuild_tag), cmakebuild_repo, cmakebuild_tag]
        util.subproc_check_call_flushed(git_argv)

    def _update_svnimportprops(self, svnimportprops_file, cmakebuild_tag, svnimportprops_file_new=None):
        svnimportprops_modified = False
        if not os.path.exists(svnimportprops_file):
            return svnimportprops_modified
        svnprop_file_parser = vcsutil.SvnPropFileParser()
        svnprops = svnprop_file_parser.parse_property_file(svnimportprops_file)
        if not svnprops.has_property('svn:externals'):
            svnimportprops_modified = True
            svnprops.add_property('svn:externals', ["/svn/svn_CMakeBuild/tags/{}/CMakeBuild CMakeBuild".format(cmakebuild_tag)])
        else:
            svnprop_externals = svnprops.get_property('svn:externals')
            re_external_cmakebuild = re.compile(r'^/svn/svn_CMakeBuild/tags/([^/]+)/CMakeBuild\s+')
            svnprop_externals_new = []
            for line in svnprop_externals:
                if (line == '') or line.isspace():
                    continue
                self._logger.debug("processing line {}".format(line))
                re_match = re_external_cmakebuild.match(line)
                if re_match:
                    cmakebuild_tag_existing = re_match.group(1)
                    self._logger.debug("{}: found CMakeBuild tag {}".format(svnimportprops_file, cmakebuild_tag_existing))
                    if cmakebuild_tag_existing != cmakebuild_tag:
                        svnimportprops_modified = True
                        svnprop_externals_new.append("/svn/svn_CMakeBuild/tags/{}/CMakeBuild CMakeBuild".format(cmakebuild_tag))
                    else:
                        svnprop_externals_new.append(line)
                else:
                    svnprop_externals_new.append(line)
            if svnimportprops_modified:
                svnprops.update_property('svn:externals', svnprop_externals_new)
        if svnimportprops_modified:
            self._logger.debug("{}: updating changed svn:externals".format(svnimportprops_file))
            if svnimportprops_file_new is None:
                svnimportprops_file_new = svnimportprops_file
            with open(svnimportprops_file_new, 'w') as f:
                svnprops_str = str(svnprops)
                f.write(svnprops_str)
        return svnimportprops_modified

    def _append_item_to_summary(self, item, value):
        self._summary_lines.append("{0:<30} {1}".format(item, value))
