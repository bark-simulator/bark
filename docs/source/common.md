Common
==========================================================

Commonly used functions and classes in BARK.

## Geometry
BARK comes with a geometry library allowing manipulations and calculations with 2D-Point-, Line- and Polygon-objects. It wraps the `boost::geometry` implementations in order to provide higher level geometric manipulations, such as searching for the nearest point on a linestring and returning the corresponding `s`-value.


## BaseObject
A common base class for all BARK classes provides common functionality. Currently, it contains the global `ParameterServer` instance. In the future, this class can be extended with loggers and more modules. 


```cpp
class BaseType {
 public:
  explicit BaseType(ParamPtr params) : params_(params) {}
  ~BaseType() {}

  ParamPtr GetParams() const { return params_;}

 private:
  ParamPtr params_;  // do not own
};
```


### ParameterServer
The ParameterServer is shared within the whole runtime. Its abstract implementation is reimplemented in Python.
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
