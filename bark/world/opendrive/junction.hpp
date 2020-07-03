// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_OPENDRIVE_JUNCTION_HPP_
#define BARK_WORLD_OPENDRIVE_JUNCTION_HPP_

#include "bark/world/opendrive/commons.hpp"

#include <map>
#include <string>
#include <vector>

namespace bark {
namespace world {
namespace opendrive {

class Junction {
 public:
  Junction(const std::string& name, uint32_t id) : id_(id), name_(name) {}
  Junction() {}
  ~Junction() {}

  using Connections = std::map<uint32_t, Connection>;

  //! setter functions
  void AddConnection(Connection con) { connections_[con.id_] = con; }
  void SetId(uint32_t id) { id_ = id; }

  //! getter functions
  Connections GetConnections() const { return connections_; }
  Connection GetConnection(uint32_t id) const { return connections_.at(id); }
  uint32_t GetId() const { return id_; }

 private:
  uint32_t id_;
  std::string name_;
  Connections connections_;
};

using JunctionPtr = std::shared_ptr<Junction>;

}  // namespace opendrive
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_OPENDRIVE_JUNCTION_HPP_
