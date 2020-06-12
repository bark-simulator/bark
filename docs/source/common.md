Common
==========================================================

Commonly used classes and functions in BARK.


## Geometry

BARK provides an easy-to-use geometry library that supports points, lines, and polygons for performant geometric calculations.
By wrapping the `boost::geometry` state-of-the-art algorithms as well as high usability is provided.
It implements all geometric functions, such as collision checks and distance calculations.


## BaseObject

All objects in BARK share a common base class, the `BaseType`.
It provides functionalities and members that are shared and used in all classes.
For example, it contains the global `ParameterServer` instance that holds all parameters.

```cpp
class BaseType {
 public:
  explicit BaseType(ParamPtr params) : params_(params) {}
  ~BaseType() {}

  ParamPtr GetParams() const { return params_;}
  ...
 private:
  ParamPtr params_;
};
```


## ParameterServer

The `ParameterServer` is shared with all objects in BARK.
Its abstract implementation is reimplemented in Python.
Child nodes can be added by using the `AddChild`-function.

```cpp
class Params {
 public:
  Params() {}

  virtual ~Params() {}

  // get and set parameters as in python
  virtual bool GetBool(const std::string &param_name,
                        const std::string &description,
                        const bool &default_value) = 0;

  virtual float GetReal(const std::string &param_name,
                         const std::string &description,
                         const float &default_value) = 0;

  virtual int GetInt(const std::string &param_name,
                      const std::string &description,
                      const int &default_value) = 0;

  // not used atm
  virtual void SetBool(const std::string &param_name, const bool &value) = 0;
  virtual void SetReal(const std::string &param_name, const float &value) = 0;
  virtual void SetInt(const std::string &param_name, const int &value) = 0;

  virtual int operator[](const std::string &param_name) = 0;

  virtual ParamPtr AddChild(const std::string &name) = 0;
};
```
