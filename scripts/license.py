#!/usr/bin/env python
# encoding: utf-8
#
#    This file is part of beautiful-bullet.
#
#    Copyright (c) 2021, 2022 Bernardo Fichera <bernardo.fichera@gmail.com>
#
#    Permission is hereby granted, free of charge, to any person obtaining a copy
#    of this software and associated documentation files (the "Software"), to deal
#    in the Software without restriction, including without limitation the rights
#    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#    copies of the Software, and to permit persons to whom the Software is
#    furnished to do so, subject to the following conditions:
#
#    The above copyright notice and this permission notice shall be included in all
#    copies or substantial portions of the Software.
#
#    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#    SOFTWARE.

import fnmatch
import os
import shutil


license = '''This file is part of beautiful-bullet.

Copyright (c) 2021, 2022 Bernardo Fichera <bernardo.fichera@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.'''


def make_dirlist(folder, extensions):
    matches = []
    for root, _, filenames in os.walk(folder):
        for ext in extensions:
            for filename in fnmatch.filter(filenames, '*' + ext):
                matches.append(os.path.join(root, filename))
    return matches


def insert_header(fname, prefix, postfix, license, kept_header=[]):
    input = open(fname, 'r')
    ofname = '/tmp/' + fname.split('/')[-1]
    output = open(ofname, 'w')

    for line in kept_header:
        output.write(line + '\n')

    output.write(prefix + '\n')

    has_postfix = len(postfix) > 0

    my_prefix = prefix

    if has_postfix:
        my_prefix = ''
    for line in license.split('\n'):
        if len(line) > 0:
            output.write(my_prefix + '    ' + line + '\n')
        else:
            output.write(my_prefix + '\n')

    if has_postfix:
        output.write(postfix + '\n')

    in_header = False

    for count, line in enumerate(input):
        # Skip kept header
        header = len(
            list(filter(lambda x: x == line[0:len(x)], kept_header))) != 0

        # Check prefix for beginning header
        check_prefix = (line[0:len(prefix)] == prefix and count == 0)

        # Check postfix end header
        check_postfix = (has_postfix and (line[0:len(postfix)] == postfix))

        # Check if in header
        if check_prefix:
            in_header = True

        # Check if off header (without postfix)
        if not has_postfix and line[0:len(prefix)] != prefix:
            in_header = False

        # Write line
        if (not in_header) and (not check_prefix) and (not header):
            output.write(line)

        # Check if off header (with postfix)
        if check_postfix:
            in_header = False

    output.close()
    shutil.move(ofname, fname)


def insert(directory):
    # C/C++
    cpp = make_dirlist(directory, ['.hpp', '.cpp', '.h', '.c', '.cc'])
    for i in cpp:
        insert_header(i, '/*', '*/', license)

    # Python
    py = make_dirlist(directory, ['.py'])
    header = ['#!/usr/bin/env python', '# encoding: utf-8']
    for i in py:
        insert_header(
            i, '#', '', license, header)

    # CMake (metapackages should not have any comments)
    cmake = make_dirlist(directory, ['CMakeLists.txt'])
    for i in cmake:
        insert_header(i, '#', '', license)

    # XML/URDF
    xml_urdf = make_dirlist(
        directory, ['.xml', '.urdf', '.xacro', '.launch'])
    header = ['<?xml version="1.0"?>']

    for i in xml_urdf:
        insert_header(
            i, '<!--', '-->', license, header)


if __name__ == '__main__':
    # Insert license in the sources
    insert('src')

    # Insert license in the scripts
    insert('scripts')

    # Insert license in the wscript main
    wscripts = [
        'wscript',
        'src/examples/wscript'
    ]

    for wscript in wscripts:
        insert_header(wscript, '#', '', license, [
                      '#!/usr/bin/env python', '# encoding: utf-8'])
