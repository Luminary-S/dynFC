#!/usr/bin/python
# -*- coding: utf-8 -*-

import yaml

# print(YAML_Path)

yaml_fname = YAML_Path + "demo.yaml"
with open(yaml_fname) as f:
    x = yaml.load(f)

print(x)