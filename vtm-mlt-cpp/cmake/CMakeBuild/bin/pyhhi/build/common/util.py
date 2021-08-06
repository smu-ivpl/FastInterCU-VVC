from __future__ import print_function

import inspect
import logging
import os
import platform
import re
import shutil
import stat
import subprocess
import sys
import traceback


# imports a base exception with an attribute to enable or disable traceback information.
from pyhhi.build.common.error import BaseError


def exec_main_default_try(main_fnc, sys_exit_err=1, finally_action=None):
    """Execute main_fnc inside a try block and dump the callstack in case of exceptions."""
    exit_error = False
    prog_name = os.path.basename(sys.argv[0])
    try:
        main_fnc()
    # except (InvalidInputParameterError, InvalidCommandLineArgumentError) BaseError as e:
    except BaseError as e:
        exit_error = True
        if e.list_traceback:
            exc_type, exc_value, exc_traceback = sys.exc_info()
            lines = traceback.format_exception(exc_type, exc_value, exc_traceback)
            for line in lines[:-1]:
                print(line.rstrip())
            print('-----')
        print("{0}: error: {1}".format(prog_name, e.msg))

    except KeyboardInterrupt:
        exit_error = True
        #print("Keyboard interrupt signaled")
    except Exception:
        exit_error = True
        exc_type, exc_value, exc_traceback = sys.exc_info()
        lines = traceback.format_exception(exc_type, exc_value, exc_traceback)
        for line in lines[:-1]:
            print(line.rstrip())
        print('-----')
        print(lines[-1])
    finally:
        if finally_action:
            finally_action()
    if exit_error:
        # some exception thrown, exit with error code to inform the shell something went wrong.
        sys.exit(sys_exit_err)
    return 0


def app_args_add_log_level(parser):
    parser.add_argument("--log-level", action="store", dest="log_level", choices=['warning', 'info', 'debug'], default="warning",
                        help="configure the log level [default: %(default)s]")


def app_configure_logging(log_level):
    # assuming loglevel is bound to the string value obtained from the
    # command line argument. Convert to upper case to allow the user to
    # specify --log-level=DEBUG or --log-level=debug
    numeric_level = getattr(logging, log_level.upper(), None)
    if not isinstance(numeric_level, int):
        raise ValueError('Invalid log level: %s' % log_level)

    # FORMAT = "py-trace %(module)s.%(funcName)s: %(lineno)d: %(message)s"
    FORMAT = "%(module)s.%(funcName)s: %(lineno)d: %(message)s"
    logging.basicConfig(format=FORMAT, level=numeric_level)


def normalize_path(fpath):
    # remove leading and trailing spaces
    fpath = fpath.lstrip().rstrip()
    if platform.system().lower() == 'windows':
        # ensure drive letters are in uppercase which os.path.normpath() does not seem to enforce.
        re_drive_letter = re.compile(r'([a-z]):', re.IGNORECASE)
        re_match = re_drive_letter.match(fpath)
        if re_match:
            fpath = re_match.group(1).upper() + fpath[1:]
    return os.path.normpath(fpath)


def to_posix_path(fpath):
    fpath = normalize_path(fpath)
    if platform.system().lower() == 'windows':
        fpath = fpath.replace('\\', '/')
    return fpath


def find_tool_on_path(tool, must_succeed=False, search_path=None):
    """Find a tool on the search path and return the full path."""
    if os.path.isabs(tool):
        if platform.system().lower() == 'windows':
            tool_basename = os.path.basename(tool)
            if not re.search(r'\.\S+$', tool_basename):
                tool = os.path.join(os.path.dirname(tool), tool_basename + '.exe')
        if os.path.exists(tool):                
            return tool
    else:
        if platform.system().lower() == 'windows':
            # special fix for windows to qualify tool with .exe if tool does not have an extension.
            if not re.search(r'\.\S+$', tool):
                tool += '.exe'
        if search_path is None:
            search_path = os.path.expandvars('$PATH')
            search_path = search_path.split(os.path.pathsep)
        for d in search_path:
            if d == '.':
                continue
            prog = os.path.join(d, tool)            
            if os.path.exists(prog) and (not os.path.isdir(prog)):
                # Always return an absolute path in case the current working directory will be changed later on.
                prog = os.path.abspath(prog)
                return prog
    if must_succeed:
        if os.path.isabs(tool):
            raise Exception("The command '" + tool + "' does not exist.")
        else:
            if search_path:
                raise Exception("The command " + tool + " cannot be found on PATH=" + ';'.join(search_path))
            else:
                raise Exception("The command " + tool + " cannot be found on PATH.")
    return None


def get_tool_dir(tool):
    if platform.system().lower() == 'windows':
        # some folks use slashes on windows or combination of slashes and backslashes
        re_pathsep = re.compile(r'[\\/:]+')
    else:
        # assume a linux path
        re_pathsep = re.compile(r'[/]+')
    if re_pathsep.search(tool):
        tool_path = normalize_path(os.path.abspath(tool))
    else:
        tool_path = find_tool_on_path(tool)
    # Resolve any symbolic links to get the real location; e.g.
    # bjam.py is a symbolic link and points to $HOME/bin/bjam.py. The real location of bjam.py is writeable but
    # the location of the symbolic link may not.
    tool_path = os.path.realpath(tool_path)
    tool_dir = os.path.dirname(tool_path)
    return tool_dir


def get_top_dir():
    # 1st check: try current working directory
    top_dir = os.getcwd()
    if is_top_dir(top_dir):
        return top_dir
    # 2nd check: try to deduce top from the script directory; e.g.  <top>/CMakeBuild/bin or <top>/BoostBuild/bin
    top_dir = os.path.normpath(os.path.join(get_script_dir(), '..', '..'))
    if not is_top_dir(top_dir):
        raise Exception(os.path.basename(sys.argv[0]) + " failed to find its parent workspace, please contact technical support.")
    return top_dir


def is_top_dir(top_dir):
    if not os.path.exists(top_dir):
        return False
    # Path check returns true if CMakeBuild is a top-level SVN external or Git submodule.
    if not os.path.exists(os.path.join(top_dir, 'CMakeBuild')):
        return False
    if os.path.exists(os.path.join(top_dir, 'CMakeBuild', 'bin', 'cmake.py')) or os.path.exists(os.path.join(top_dir, 'CMakeBuild', 'CMakeBuild', 'bin', 'cmake.py')):
        return True

    # Check for CMakeBuild versioned subtree
    cmakebuild_dir = os.path.join(top_dir, 'CMakeBuild')
    for fname in os.listdir(cmakebuild_dir):
        if os.path.isdir(os.path.join(cmakebuild_dir, fname)):
            if os.path.exists(os.path.join(cmakebuild_dir, fname, 'CMakeBuild', 'bin', 'cmake.py')):
                return True
    return False


def get_script_dir():
    py_util_fname = inspect.getfile(get_script_dir)
    # <top>/CMakeBuild/bin/pyhhi/build/common/util.py
    script_dir = os.path.normpath(os.path.join(os.path.dirname(py_util_fname), '..', '..', '..'))
    return script_dir


def get_boost_build_dir(top_dir):
    # Assume the standard SVN layout without a submodules folder holding the externals.
    boost_build_dir = os.path.join(top_dir, 'BoostBuild')
    if not os.path.exists(boost_build_dir):
        raise Exception("The directory '" + top_dir + "' does not seem to be a workspace directory with a BoostBuild folder, please contact technical support.")
    return boost_build_dir


def find_repo_path_from_src_path(src_path):
    src_path = os.path.abspath(src_path.rstrip(os.path.sep))
    is_windows = platform.system().lower() == 'windows'
    # split the path
    if is_windows:
        drive_path_comps = os.path.splitdrive(src_path)
        src_path = drive_path_comps[1]

    dir_list = src_path.lstrip(os.path.sep).split(os.path.sep)
    joiner = os.path.sep

    while dir_list:
        # walk the tree until 'src' is found. The parent of src is supposed to be the name of the repository.
        dir = dir_list.pop()
        if len(dir_list) < 2:
            break

        if dir.lower() == 'src':
            if is_windows:
                jamroot_dir = os.path.join(drive_path_comps[0], os.path.sep, joiner.join(dir_list[:-1]))
                repo_path = os.path.join(drive_path_comps[0], os.path.sep, joiner.join(dir_list))
            else:
                # repo_path: /home/rauthenberg/projects/BoostBuild/BoostBuild
                jamroot_dir = os.path.join(os.path.sep, joiner.join(dir_list[:-1]))
                repo_path = os.path.join(os.path.sep, joiner.join(dir_list))
            if os.path.isfile(os.path.join(jamroot_dir, 'Jamroot')):
                return repo_path
    raise Exception("The repository path cannot be deduced from '" + src_path + "'")


def find_repo_name_from_src_path(src_path):
    repo_path = find_repo_path_from_src_path(src_path)
    if platform.system().lower() == 'windows':
        drive_path_comps = os.path.splitdrive(repo_path)
        repo_path = drive_path_comps[1]
    return repo_path.lstrip(os.path.sep).split(os.path.sep)[-1]


def subproc_call_flushed(*popenargs, **kwargs):
    """Run command with arguments.  Wait for command to complete or
    timeout, then return the returncode attribute.

    The arguments are the same as for the Popen constructor.  Example:

    retcode = call(["ls", "-l"])
    """
    sys.stdout.flush()
    return subprocess.call(*popenargs, **kwargs)


def subproc_check_call_flushed(*popenargs, **kwargs):
    """Run command with arguments.  Wait for command to complete.  If
    the exit code was zero then return, otherwise raise
    CalledProcessError.  The CalledProcessError object will have the
    return code in the returncode attribute.

    The arguments are the same as for the call function.  Example:

    check_call(["ls", "-l"])
    """
    sys.stdout.flush()
    return subprocess.check_call(*popenargs, **kwargs)


def rmtree(directory):
    """On Windows invokes rmtree_readonly() and on other platforms shutil.rmtree().
    This convenience function may be used as an replacement of shutil.rmtree(directory)
    to be able to remove directory trees on Windows containing some readonly files.
    """
    if sys.platform.startswith('win'):
        rmtree_readonly(directory)
    else:
        shutil.rmtree(directory)


def rmtree_readonly(directory):
    """Remove a directory tree on Windows where some files are readonly."""

    def remove_readonly(func, path, _excinfo):
        "Clear the readonly bit and reattempt the removal"
        os.chmod(path, stat.S_IWRITE)
        func(path)

    shutil.rmtree(directory, onerror=remove_readonly)
