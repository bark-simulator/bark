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
  explicit BaseType(Params* params) : params_(params) {}
  ~BaseType() {}

  Params* get_params() const { return params_;}

 private:
  Params* params_;  // do not own
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
  virtual bool get_bool(const std::string &param_name,
                        const std::string &description,
                        const bool &default_value) = 0;

  virtual float get_real(const std::string &param_name,
                         const std::string &description,
                         const float &default_value) = 0;

  virtual int get_int(const std::string &param_name,
                      const std::string &description,
                      const int &default_value) = 0;

  // not used atm
  virtual void set_bool(const std::string &param_name, const bool &value) = 0;
  virtual void set_real(const std::string &param_name, const float &value) = 0;
  virtual void set_int(const std::string &param_name, const int &value) = 0;

  virtual int operator[](const std::string &param_name) = 0;

  virtual Params* AddChild(const std::string &name) = 0;
};
```
