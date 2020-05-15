# Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import json
import os
from bark.core.commons import Params
import logging
logging.getLogger().setLevel(logging.DEBUG)


class ParameterServer(Params):
    def __init__(self, **kwargs):
        Params.__init__(self)
        self.param_filename = None
        self.log_if_default = kwargs.pop("log_if_default", False)
        if "filename" in kwargs:
            self.load(kwargs["filename"])
            self.param_filename = kwargs["filename"]
        elif "json" in kwargs:
            self.ConvertToParam(kwargs["json"])
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
                if self.log_if_default:
                  logging.warning("Using default {} for {}".format(
                  default_val, new_key))
                return self.store[new_key]
            else:  # else it is a Params instance
                self.store[new_key] = ParameterServer(log_if_default = self.log_if_default)
                return self.store[new_key]

    def FindKey(self, param_key):
      delimiter = "::"
      for key, value in self.store.items():
                  if isinstance(value, ParameterServer):
                      found_key_tmp =  value.FindKey(param_key)
                      if found_key_tmp:
                        return "{}{}{}".format(key, delimiter,
                              found_key_tmp)
                  else:
                      if param_key == key:
                        return param_key
      return None


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
        self.ConvertToParam(dict)

    def ConvertToParam(self, new_dict):
        self.store = dict()
        for key, value in new_dict.items():
            if isinstance(value, dict):
                param = ParameterServer(log_if_default = self.log_if_default)
                self.store[key] = param.ConvertToParam(value)
            else:
                self.GetValFromString(key, "", value, log_if_default=False)

        return self

    def ConvertToDict(self, print_description=False):
        dict = {}
        for key, value in self.store.items():
            if isinstance(value, ParameterServer):
                v = value.ConvertToDict(print_description)
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

    def Save(self, filename, print_description=False):
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
                    self.ConvertToDict(print_description=print_description),
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
                    return self.store[key], False
            else:
                if not hierarchy:
                    self.store[key] = default_value
                    return default_value, True
                else:
                    self.store[key] = ParameterServer(log_if_default = self.log_if_default)
                    return self.store[key].get_val_iter(
                        hierarchy, description, default_value)
        return

    def GetCondensedParamList(self):
        def check_append(value):
          if isinstance(value, float) or isinstance(value, int) \
              or isinstance(value,bool):
              return True
          # list float
          elif isinstance(value, list) and all(isinstance(el, float) for el in value):
            return True
          # list list float
          elif isinstance(value, list):
              for el in value:
                if not isinstance(el, list):
                  return False
                for e in el:
                  if not isinstance(e, float):
                    return False
              return True
          return False
        hierarchy_delimiter = "::"
        condensed_param_list = []
        for key, value in self.store.items():
            if isinstance(value, ParameterServer):
                child_list = value.GetCondensedParamList()
                for param_tuple in child_list:
                    param_name = "{}{}{}".format(key,
                                    hierarchy_delimiter, param_tuple[0])
                    param_value = param_tuple[1]
                    if check_append(param_value):
                        condensed_param_list.append((param_name, param_value))
            else:
                if check_append(value):
                    condensed_param_list.append((key, value))
        test = condensed_param_list
        return condensed_param_list

    def GetValFromString(self, hierarchy, description, default_value, log_if_default):
        hierarchy = [x.strip() for x in hierarchy.split("::")]
        if log_if_default:
          # first search for key, otherwise default already integrated into store
          found_key = self.FindKey(hierarchy[-1])
        value, used_default = self.get_val_iter(hierarchy.copy(), description, default_value)
        if log_if_default and used_default:
          logging.warning("Using default {} for {}".format(
                                default_value, hierarchy))
          if found_key:
            logging.warning("Did you mean param {}".format(found_key))
          else:
            logging.warning("Key was nowhere found.")
        return value
        
    # get values
    def GetBool(self, param_name, description, default_value):
        #return self[param_name, description, default_value]
        return self.GetValFromString(param_name, description, default_value, self.log_if_default)

    def GetReal(self, param_name, description, default_value):
        #return self[param_name, description, default_value]
        return self.GetValFromString(param_name, description, default_value, self.log_if_default)

    def GetInt(self, param_name, description, default_value):
        #return self[param_name, description, default_value]
        return self.GetValFromString(param_name, description, default_value, self.log_if_default)

    def GetListListFloat(self, param_name, description, default_value):
        return self.GetValFromString(param_name, description, default_value, self.log_if_default)
    
    def GetListFloat(self, param_name, description, default_value):
        return self.GetValFromString(param_name, description, default_value, self.log_if_default)

    def access(self, param_name):
        return self[param_name]

    def SetBool(self, param_name, value):
        self[param_name] = value
        return

    def SetReal(self, param_name, value):
        self[param_name] = value
        return

    def SetInt(self, param_name, value):
        self[param_name] = value
        return

    def AddChild(self, name):
        return self.__getitem__(name)
