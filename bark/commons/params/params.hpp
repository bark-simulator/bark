// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_COMMONS_PARAMS_PARAMS_HPP_
#define BARK_COMMONS_PARAMS_PARAMS_HPP_

#include <string>
#include <vector>
#include "bark/commons/distribution/distribution.hpp"
#include "boost/variant.hpp"

namespace bark {
namespace commons {

typedef std::vector<std::vector<float>> ListListFloat;

typedef std::vector<float> ListFloat;
typedef boost::variant<bool, float, int, std::string, ListListFloat, ListFloat>
    Parameter;
typedef std::pair<std::string, Parameter> ParamPair;

inline std::ostream& operator<<(std::ostream& os, const Parameter& p) {
  return boost::apply_visitor(
      [&os](const auto& p) -> std::ostream& { return os << p; }, p);
}

typedef std::vector<ParamPair> CondensedParamList;

class Params {
 public:
  Params() {}

  virtual ~Params() {}

  // get and set parameters as in python
  virtual bool GetBool(const std::string& param_name,
                       const std::string& description,
                       const bool& default_value) = 0;

  virtual float GetReal(const std::string& param_name,
                        const std::string& description,
                        const float& default_value) = 0;

  virtual int GetInt(const std::string& param_name,
                     const std::string& description,
                     const int& default_value) = 0;

  virtual std::string GetString(const std::string& param_name,
                                const std::string& description,
                                const std::string& default_value) = 0;

  virtual std::vector<std::vector<float>> GetListListFloat(
      const std::string& param_name, const std::string& description,
      const ListListFloat& default_value) = 0;

  virtual std::vector<float> GetListFloat(const std::string& param_name,
                                          const std::string& description,
                                          const ListFloat& default_value) = 0;

  DistributionPtr GetDistributionFromType(
      const std::string& distribution_type,
      const std::shared_ptr<Params>& distr_params) const;

  // getting is implemented in C++ to avoid wrapping all distribution types to
  // python
  DistributionPtr GetDistribution(const std::string& param_name,
                                  const std::string& description,
                                  const std::string& default_distribution_type);

  virtual CondensedParamList GetCondensedParamList() const = 0;

  std::string Print() const;

  virtual void SetBool(const std::string& param_name, const bool& value) = 0;
  virtual void SetReal(const std::string& param_name, const float& value) = 0;
  virtual void SetInt(const std::string& param_name, const int& value) = 0;
  virtual void SetListListFloat(const std::string& param_name,
                                const ListListFloat& value) = 0;
  virtual void SetListFloat(const std::string& param_name,
                            const ListFloat& value) = 0;
  virtual void SetDistribution(const std::string& param_name,
                               const std::string& distribution_type) = 0;

  virtual void SetString(const std::string& param_name,
                         const std::string& default_value) = 0;

  virtual int operator[](const std::string& param_name) = 0;
  virtual std::shared_ptr<Params> AddChild(const std::string& name) = 0;
};

typedef std::shared_ptr<Params> ParamsPtr;

}  // namespace commons
}  // namespace bark

#endif  // BARK_COMMONS_PARAMS_PARAMS_HPP_
