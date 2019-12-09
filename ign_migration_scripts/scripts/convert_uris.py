#!/usr/bin/env python
#
# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import print_function
from lxml import etree
import argparse
import sys
import os
import re

def convert_uris(model_sdf):
    """Convert uris of type `model://` to the absolute paths
    parent model

    Parameters
    ----------
    model_sdf : Model sdf

    Returns
    -------
    TODO

    """
    inp = etree.parse(model_sdf)
    sdf = inp.getroot()
    # Assume there's just one <model>
    uris = sdf.xpath('//uri')
    for uri in uris:
        print("Checking URI: ", uri.text, file=sys.stderr)
        match = re.match("^\s*model://\s*", uri.text)
        if match is not None:
            print("Found match", match.group(), file=sys.stderr)
            model_abs_dir = os.path.dirname(
                os.path.dirname(os.path.abspath(model_sdf.name)))
            uri.text = os.path.join(model_abs_dir, uri.text[match.end():])
        # Check if using relative paths
        rel_uri_match = re.match("^\s*://\s*", uri.text)
        if rel_uri_match is None:
            print("Found relative URI:", uri.text, file=sys.stderr)
            model_abs_dir = os.path.dirname(os.path.abspath(model_sdf.name))
            uri.text = os.path.join(model_abs_dir, uri.text)
            print("New URI:", uri.text, file=sys.stderr)
    return sdf

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('infile', type=argparse.FileType('r'),
                        help='Input SDF file')

    args = parser.parse_args()
    sdf = convert_uris(args.infile)
    print(etree.tostring(sdf, pretty_print=True, encoding="unicode"))
