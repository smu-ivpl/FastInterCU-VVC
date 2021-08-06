
from __future__ import print_function

import logging
import re
import os

from pyhhi.build.common.error import InvalidInputParameterError


class CMakeRstUtilParams(object):

    def __init__(self):
        self.dry_run = False
        self.rst_module_filenm = None
        self.update_action = 'add'
        self.extension_module_names = []
        self.extension_section_title = "Extension Modules"
        self.output_rst_filenm = None


class RstModuleSection(object):

    def __init__(self):
        self.section_title = None
        self.section_title_marker_line = None
        self.section_header = []
        self.section_content = []


class CMakeManualRstUtil(object):

    def __init__(self, dry_run=False):
        self._logger = logging.getLogger(__name__)
        self._dry_run = dry_run
        self._section_title_cmake_modules_orig = "All Modules"
        self._section_title_cmake_modules = "Standard Modules"
        self._reserved_section_titles = [self._section_title_cmake_modules, self._section_title_cmake_modules_orig, "Legacy CPack Modules"]
        self._section_title_marker = '='
        self._is_cmake_source_tree = False
        # Location of <module>.rst
        self._cmake_help_module_dir = None
        # Automatic removal of cmake module wrapper files.
        self._autoremove = True
        self._re_section_content = re.compile(r'\s+/module/([a-zA-Z0-9_-]+)\s*$')

    def is_cmake_source_tree(self):
        return self._is_cmake_source_tree

    def add_extension_modules(self, rst_module_filenm, extension_module_names, section_title="Extension Modules", output_rst_filenm=None):

        if not os.path.exists(rst_module_filenm):
            raise InvalidInputParameterError("CMake RST module file {} does not exist.".format(rst_module_filenm))
        if output_rst_filenm is None:
            # in-place replacement
            output_rst_filenm = rst_module_filenm
        # Empty sections are not permitted, use one of the remove_xxx() methods to get rid of a section.
        assert extension_module_names
        self._detect_cmake_source_tree(rst_module_filenm)
        self._check_section_title(section_title)
        (module_file_header, section_list) = self._parse_rst_module_file(rst_module_filenm)

        if len(section_list) > 0:
            # CMake's original module file has one or more sections. All extension sections are inserted in front of
            # section titled "All Modules" or "Standard Modules". The latter title is used after the first extension section has been inserted.

            # Get cmake module names
            cmake_module_names_set = self._get_cmake_module_names(section_list)
            # Extension module names must not replace existing cmake module names
            extension_module_names_set = set(extension_module_names)
            if not cmake_module_names_set.isdisjoint(extension_module_names_set):
                conflicting_names = cmake_module_names_set.intersection(extension_module_names_set)
                msg = "CMake builtin module names cannot be part of an extension section.\n"
                msg += "Please check the extension module names: " + " ".join(conflicting_names)
                raise Exception(msg)
        else:
            # Looks like a corrupted cmake-modules.7.rst
            assert False
        # Get a list of RST extension module wrapper files belonging to section "section_title".
        existing_extension_module_names = self._get_extension_module_names(section_list, section_title)
        if existing_extension_module_names:
            self._logger.debug('existing extensions: %s', ' '.join(existing_extension_module_names))
        modified = self._update_section(section_list, section_title, extension_module_names)
        if modified:
            # print("new content lines:")
            # self._dump_section_list(section_list[0:-1])
            # Change title of the CMake module section from "All Modules" to "Standard Modules".
            for section in section_list:
                if section.section_title == self._section_title_cmake_modules_orig:
                    section.section_title = self._section_title_cmake_modules
                    section.section_title_marker_line = '{0:=>{width}}'.format(self._section_title_marker, width=len(section.section_title))
                    break

            section_list = self._sort_section_list(section_list)
            # Save top-level RST module file
            self._save_rst_module_file(output_rst_filenm, module_file_header, section_list)

            # Remove RST extension module wrapper files not belonging to this section anymore.
            outdated_extension_module_names = []
            for mod_nm in existing_extension_module_names:
                if mod_nm not in extension_module_names:
                    outdated_extension_module_names.append(mod_nm)
            if outdated_extension_module_names:
                self._logger.debug('outdated extensions: %s', ' '.join(outdated_extension_module_names))
                # Remove outdated RST extension module wrapper files.
                self._remove_rst_extension_module_files(outdated_extension_module_names)

    def remove_extension_module_section(self, rst_module_filenm, section_title="Extension Modules", output_rst_filenm=None):
        if not os.path.exists(rst_module_filenm):
            raise Exception("CMake RST module file " + rst_module_filenm + " does not exist.")
        if output_rst_filenm is None:
            output_rst_filenm = rst_module_filenm
        self._detect_cmake_source_tree(rst_module_filenm)
        self._check_section_title(section_title)
        (module_file_header, section_list) = self._parse_rst_module_file(rst_module_filenm)
        assert len(section_list) > 0

        # Get a list of RST extension module wrapper files belonging to section "section_title".
        existing_extension_module_names = self._get_extension_module_names(section_list, section_title)
        if existing_extension_module_names:
            self._logger.debug('existing extensions: %s', ' '.join(existing_extension_module_names))

        modified = False
        new_section_list = []
        for sect in section_list:
            if section_title == sect.section_title:
                modified = True
            else:
                new_section_list.append(sect)
        assert len(new_section_list) > 0
        sect = new_section_list[0]
        if sect.section_title == self._section_title_cmake_modules:
            modified = True
            # Recover the original section title
            sect.section_title = self._section_title_cmake_modules_orig
            sect.section_title_marker_line = '{0:=>{width}}'.format(self._section_title_marker, width=len(sect.section_title))

        if modified:
            # Save top-level RST module file
            self._save_rst_module_file(output_rst_filenm, module_file_header, new_section_list)
            # Remove RST extension module wrapper files belonging to the section just removed.
            self._remove_rst_extension_module_files(existing_extension_module_names)

    def remove_all_extension_modules(self, rst_module_filenm, output_rst_filenm=None):
        if not os.path.exists(rst_module_filenm):
            raise Exception("CMake RST module file " + rst_module_filenm + " does not exist.")
        if output_rst_filenm is None:
            output_rst_filenm = rst_module_filenm
        self._detect_cmake_source_tree(rst_module_filenm)
        (module_file_header, section_list) = self._parse_rst_module_file(rst_module_filenm)
        existing_extension_module_names = self._get_extension_module_names(section_list)
        if existing_extension_module_names:
            self._logger.debug('existing extensions: %s', ' '.join(existing_extension_module_names))
        modified = False

        while section_list:
            sect = section_list[0]
            if sect.section_title == self._section_title_cmake_modules:
                # Found first original section
                modified = True
                sect.section_title = self._section_title_cmake_modules_orig
                sect.section_title_marker_line = '{0:=>{width}}'.format(self._section_title_marker, width=len(sect.section_title))
                break
            elif sect.section_title == self._section_title_cmake_modules_orig:
                # Found first original section
                break
            else:
                modified = True
                section_list.pop(0)
        assert len(section_list) > 0
        if modified:
            # Save top-level RST module file
            self._save_rst_module_file(output_rst_filenm, module_file_header, section_list)
            # Remove all RST extension module wrapper files
            self._remove_rst_extension_module_files(existing_extension_module_names)

    def _detect_cmake_source_tree(self, rst_module_filenm):
        rst_manual_dir = os.path.dirname(rst_module_filenm)
        rst_module_dir = os.path.join(rst_manual_dir, '..', 'module')
        if (os.path.basename(rst_manual_dir) == 'manual') and (os.path.exists(rst_module_dir) and os.path.isdir(rst_module_dir)):
            self._is_cmake_source_tree = True
            self._cmake_help_module_dir = self._get_rst_module_dir(rst_module_filenm)
            assert os.path.exists(self._cmake_help_module_dir) and os.path.isdir(self._cmake_help_module_dir)
        else:
            self._is_cmake_source_tree = False
            self._cmake_help_module_dir = None

    def _get_rst_module_dir(self, rst_module_filenm):
        rst_manual_dir = os.path.dirname(rst_module_filenm)
        rst_module_dir = os.path.join(rst_manual_dir, '..', 'module')
        return os.path.normpath(rst_module_dir)

    def _check_section_title(self, section_title):
        if section_title in self._reserved_section_titles:
            raise InvalidInputParameterError("section {} is protected and cannot be modified, please contact technical support.".format(section_title))

    def _check_extension_module_names(self, extension_module_names):
        if not self.is_cmake_source_tree():
            return
        for mod_nm in extension_module_names:
            rst_module_file = os.path.join(self._cmake_help_module_dir, mod_nm + '.rst')
            if not os.path.exists(rst_module_file):
                raise Exception("file {} does not exist.".format(rst_module_file))

    def _update_section(self, section_list, section_title, extension_module_names):
        section = None
        modified = False
        assert extension_module_names
        extension_module_names.sort()
        for sec in section_list:
            if sec.section_title == section_title:
                section = sec
                break
        if not section:
            # add a new section
            modified = True
            section = RstModuleSection()
            section.section_title = section_title
            section.section_title_marker_line = '{0:=>{width}}'.format(self._section_title_marker, width=len(section.section_title))
            section.section_header.append('')
            section.section_header.append('.. toctree::')
            section.section_header.append('   :maxdepth: 1')
            section.section_header.append('')
            section_list.insert(0, section)
        else:
            # Section already present, update only if section content is different.
            existing_extension_module_names = []
            # re_section_content = re.compile(r'\s+/module/([a-zA-Z0-9_-]+)\s*$')
            for line in section.section_content:
                re_match = self._re_section_content.match(line)
                if re_match:
                    existing_extension_module_names.append(re_match.group(1))
            existing_extension_module_names.sort()
            if len(existing_extension_module_names) != len(extension_module_names):
                modified = True
            else:
                for module_nm in existing_extension_module_names:
                    if module_nm not in extension_module_names:
                        modified = True
                        break
            if modified:
                # replace the section
                section.section_content = []
        if modified:
            # replace the content of an existing section
            for module_nm in extension_module_names:
                section.section_content.append('   /module/' + module_nm)
        return modified

    def _sort_section_list(self, section_list):
        """Sort section list by title and keep CMake's original sections at the end."""

        section_list_ext = []
        while section_list:
            sect = section_list[0]
            if sect.section_title in [self._section_title_cmake_modules_orig, self._section_title_cmake_modules]:
                break
            else:
                section_list_ext.append(sect)
                section_list.pop(0)

        section_title_dict = {}
        section_list_sorted = []
        for sect in section_list_ext:
            section_title_dict[sect.section_title] = sect
        title_list = list(section_title_dict.keys())
        title_list.sort()
        for title in title_list:
            section_list_sorted.append(section_title_dict[title])

        # and append CMake's original sections.
        section_list_sorted.extend(section_list)
        return section_list_sorted

    def _parse_rst_module_file(self, rst_module_filenm):

        module_file_header = []
        section_list = []
        section = None
        previous_line = None
        re_empty_line = re.compile(r'^\s*$')

        with open(rst_module_filenm) as f:
            for line in f:
                line = line.rstrip()
                if not section_list:
                    # Processing lines in front of the first section header
                    if line.startswith(self._section_title_marker):
                        section = RstModuleSection()
                        section.section_title = previous_line
                        section.section_title_marker_line = line
                        self._logger.debug("found section: {}".format(previous_line))
                        section_list.append(section)
                        # Remove the title line from the module header.
                        module_file_header.pop()
                        if module_file_header and re_empty_line.match(module_file_header[-1]):
                            # Remove the last empty line from the module header to simplify saving sections later on.
                            module_file_header.pop()
                    else:
                        module_file_header.append(line)
                else:
                    if section.section_content:
                        if self._re_section_content.match(line):
                            section.section_content.append(line)
                        else:
                            section = RstModuleSection()
                    else:
                        # No section content found yet.
                        if not section.section_title:
                            if line.startswith(self._section_title_marker):
                                section.section_title = previous_line
                                section.section_title_marker_line = line
                                section_list.append(section)
                                self._logger.debug("found section: {}".format(previous_line))
                            else:
                                pass
                        else:
                            if self._re_section_content.match(line):
                                section.section_content.append(line)
                            else:
                                section.section_header.append(line)
                previous_line = line

        # self._dump_module_file_header(module_file_header)
        # self._dump_section_list(section_list)
        return module_file_header, section_list

    def _get_extension_module_names(self, section_list, section_title=None):
        extension_module_names = []
        for sect in section_list:
            if sect.section_title in [self._section_title_cmake_modules_orig, self._section_title_cmake_modules]:
                continue
            if (section_title is None) or (section_title == sect.section_title):
                for line in sect.section_content:
                    re_match = self._re_section_content.match(line)
                    if re_match:
                        extension_module_names.append(re_match.group(1))
        return extension_module_names

    def _get_cmake_module_names(self, section_list):

        cmake_module_names = set()
        section_all_modules_fnd = False

        for sect in section_list:
            if not section_all_modules_fnd:
                if sect.section_title in [self._section_title_cmake_modules_orig, self._section_title_cmake_modules]:
                    section_all_modules_fnd = True
                else:
                    # One of the extension sections in front of "All Modules" or "Standard Modules"
                    continue
            for line in sect.section_content:
                re_match = self._re_section_content.match(line)
                if re_match:
                    cmake_module_names.add(re_match.group(1))
        return cmake_module_names

    def _save_rst_module_file(self, output_rst_filenm, module_file_header, section_list):
        if self._dry_run:
            self._logger.debug("leaving without saving anything to %s", output_rst_filenm)
            return
        self._logger.debug("saving updated content to %s", output_rst_filenm)
        with open(output_rst_filenm, "w") as f:
            for line in module_file_header:
                f.write(line + '\n')
            for sect in section_list:
                f.write('\n')
                f.write(sect.section_title + '\n')
                f.write(sect.section_title_marker_line + '\n')
                for line in sect.section_header:
                    f.write(line + '\n')
                for line in sect.section_content:
                    f.write(line + '\n')

    def _remove_rst_extension_module_files(self, extension_module_names):
        if not self._autoremove:
            return
        if not self.is_cmake_source_tree():
            return
        for mod_nm in extension_module_names:
            rst_module_file = os.path.join(self._cmake_help_module_dir, mod_nm + '.rst')
            if os.path.exists(rst_module_file):
                self._logger.debug("removing %s", rst_module_file)
                if not self._dry_run:
                    os.remove(rst_module_file)

    def _dump_module_file_header(self, module_file_header):
        for line in module_file_header:
            print("file header: " + line)

    def _dump_section_list(self, section_list):
        print("number of sections: %d" %(len(section_list)))
        for section in section_list:
            print("section title:        " + section.section_title)
            print("section title marker: " + section.section_title_marker_line)
            for line in section.section_header:
                print("section header: " + line)
            for line in section.section_content:
                print("section content: " + line)