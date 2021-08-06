
from __future__ import print_function

import argparse
import logging
import os.path
import sys

import pyhhi.build.common.util as util
import pyhhi.build.cmkdoc as cmkdoc

from pyhhi.build.common.error import InvalidCommandLineArgumentError, InvalidInputParameterError


class CMakeDocUtilApp(object):

    def __init__(self):
        self._logger = logging.getLogger(__name__)

    def __call__(self):
        self.main(sys.argv[1:])

    def main(self, argv):
        params = self._parse_command_line(argv)
        self.update_cmake_manual_docs(params)

    def update_cmake_manual_docs(self, params):
        self._check_params(params)
        cmkdocutil = cmkdoc.CMakeManualRstUtil(dry_run=params.dry_run)
        if params.update_action == 'add':
            cmkdocutil.add_extension_modules(params.rst_module_filenm, params.extension_module_names,
                                             output_rst_filenm=params.output_rst_filenm, section_title=params.extension_section_title)
        elif params.update_action == 'remove':
            cmkdocutil.remove_extension_module_section(params.rst_module_filenm, output_rst_filenm=params.output_rst_filenm, section_title=params.extension_section_title)
        elif params.update_action == 'remove_all':
            cmkdocutil.remove_all_extension_modules(params.rst_module_filenm, output_rst_filenm=params.output_rst_filenm)
        else:
            assert False

    def _check_params(self, params):
        assert params.rst_module_filenm is not None
        if not os.path.exists(params.rst_module_filenm):
            raise InvalidInputParameterError("CMake doc module file {} does not exist.".format(params.rst_module_filenm))

    def _parse_command_line(self, argv):

        params = cmkdoc.CMakeRstUtilParams()

        parser = argparse.ArgumentParser()

        parser.add_argument("input", action="store", help="specifies the CMake RST module file to be updated.")

        parser.add_argument("-m", action="append", dest="module_names",
                            help="specifies an extension module name, the option must be repeated for each extension module.")
        parser.add_argument("-R", action="store_true", dest="remove_all", default=False,
                            help="remove all non-standard module sections.")
        parser.add_argument("-r", action="store_true", dest="remove", default=False,
                            help="remove a single non-standard module section.")
        parser.add_argument("-o", action="store", dest="output_rst_filenm")
        parser.add_argument("-s", action="store", dest="section_title", default=params.extension_section_title,
                            help="specifies the section to be updated [default: %(default)s]")
        parser.add_argument("--dry-run", action="store_true", dest="dry_run", default=False)

        util.app_args_add_log_level(parser)

        args = parser.parse_args(argv)

        # configure the python logger asap
        util.app_configure_logging(args.log_level)

        params.dry_run = args.dry_run
        params.rst_module_filenm = os.path.abspath(args.input)

        if args.module_names:
            params.update_action = 'add'
            params.extension_module_names = args.module_names
        elif args.remove_all:
            params.update_action = 'remove_all'
        elif args.remove:
            params.update_action = 'remove'
        else:
            raise InvalidCommandLineArgumentError("no update action specified, use -m, -R or -r.")
        if args.section_title:
            params.extension_section_title = args.section_title
        if args.output_rst_filenm:
            params.output_rst_filenm = os.path.abspath(args.output_rst_filenm)

        return params
