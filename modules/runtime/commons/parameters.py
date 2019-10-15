# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import json
import os
from bark.commons import Params


class ParameterServer(Params):
    def __init__(self, **kwargs):
        Params.__init__(self)
        self.param_filename = None
        if "filename" in kwargs:
            self.load(kwargs["filename"])
            self.param_filename = kwargs["filename"]
        elif "json" in kwargs:
            self.convert_to_param(kwargs["json"])
        else:
            self.store = dict()
        self.param_descriptions = dict()
        

    def __getitem__(self, key):
        if isinstance(key, tuple):
            new_key = key[0]
            self.param_descriptions[new_key] = key[1]
            if len(key) == 3:
                default_val = key[2]
            else:
                default_val = "MISSING"
        else:
            new_key = key
        if new_key in self.store:
            return self.store[new_key]
        else:
            if isinstance(key, tuple):  # if key for parameter
                self.store[new_key] = default_val
                return self.store[new_key]
            else:  # else it is a Params instance
                self.store[new_key] = ParameterServer()
                return self.store[new_key]

    def __setitem__(self, key, value):
        if isinstance(key, tuple):
            new_key = key[0]
            self.param_descriptions[new_key] = key[1]
        else:
            new_key = key
        self.store[new_key] = value

    def __delitem__(self, key):
        del self.store[key]

    def __iter__(self):
        return iter(self.store)

    def __len__(self):
        return len(self.store)

    def load(self, fn):
        with open(fn) as file:
            dict = json.load(file)
        self.convert_to_param(dict)

    def convert_to_param(self, new_dict):
        self.store = dict()
        for key, value in new_dict.items():
            if isinstance(value, dict):
                param = ParameterServer()
                self.store[key] = param.convert_to_param(value)
            else:
                self.store[key] = value

        return self

    def convert_to_dict(self, print_description=False):
        dict = {}
        for key, value in self.store.items():
            if isinstance(value, ParameterServer):
                v = value.convert_to_dict(print_description)
                if len(v) == 0:
                    if print_description:
                        if key in self.param_descriptions:
                            dict[key] = self.param_descriptions[key]
                        else:
                            dict[key] = "--"
                    else:
                        dict[key] = "MISSING"
                else:
                    dict[key] = v
            else:
                dict[key] = value

                if print_description:
                    if key in self.param_descriptions:
                        dict[key] = self.param_descriptions[key]
                    else:
                        dict[key] = "--"

        return dict

    def save(self, filename, print_description=False):
        #if not os.path.exists(os.path.dirname(filename)):
            #try:
           #     os.makedirs(os.path.dirname(filename))
          #  except:
         #       print("Could not dump parameters, no rights to create file directory.")
        #        return
        with open(filename, 'w') as outfile:
            print("Writing parameters to {}".format(os.path.abspath(filename)))
            outfile.write(
                json.dumps(
                    self.convert_to_dict(print_description=print_description),
                    indent=4))

    def get_val_iter(self, hierarchy, description, default_value):
        if not hierarchy:
            raise ValueError("PARAM HIERARCHY IS EMPTY")
        else:
            key = hierarchy[0]
            hierarchy.pop(0)
            if key in self.store:
                #if isinstance(self.store[key], ParameterServer):
                if type(self.store[key]) == ParameterServer:
                    #raise ValueError("get here")
                    return self.store[key].get_val_iter(
                        hierarchy, description, default_value)
                else:
                    return self.store[key]
            else:
                if not hierarchy:
                    self.store[key] = default_value
                    return default_value
                else:
                    self.store[key] = ParameterServer()
                    return self.store[key].get_val_iter(
                        hierarchy, description, default_value)
        return

    def get_val_from_string(self, hierarchy, description, default_value):
        hierarchy = [x.strip() for x in hierarchy.split("::")]
        return self.get_val_iter(hierarchy, description, default_value)
        #helper = self
        #for key in hierarchy:
        #	helper = helper[key, description, default_value]
        #return helper
        
    # get values
    def get_bool(self, param_name, description, default_value):
        #return self[param_name, description, default_value]
        return self.get_val_from_string(param_name, description, default_value)

    def get_real(self, param_name, description, default_value):
        #return self[param_name, description, default_value]
        return self.get_val_from_string(param_name, description, default_value)

    def get_int(self, param_name, description, default_value):
        #return self[param_name, description, default_value]
        return self.get_val_from_string(param_name, description, default_value)

    def get_listlist_float(self, param_name, description, default_value):
        return self.get_val_from_string(param_name, description, default_value)

    def access(self, param_name):
        return self[param_name]

    def set_bool(self, param_name, value):
        self[param_name] = value
        return

    def set_real(self, param_name, value):
        self[param_name] = value
        return

    def set_int(self, param_name, value):
        self[param_name] = value
        return

    def AddChild(self, name):
        if not name in self.store:
            self.store[name] = ParameterServer()
        return self.store[name]
