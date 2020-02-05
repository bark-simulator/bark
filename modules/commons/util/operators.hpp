// Copyright (c) 2020 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <ostream>
#include <vector>

template < class T >
inline std::ostream& operator << (std::ostream& os, const std::vector<T>& v) 
{
    os << "[";
    for (typename std::vector<T>::const_iterator ii = v.begin(); ii != v.end(); ++ii)
    {
        os << " ";
        os.operator<<(*ii);
    }
    os << " ]";
    return os;
}


template <>
inline std::ostream& operator << <std::vector<float>>(std::ostream& os, const std::vector<std::vector<float>>& v) 
{
    os << "[";
    for (const auto sb : v)
    {
      os << "[";
      for (const auto e : sb) {
        os << " " << e;
      }
      os << " ]";
    }
    os << " ]";
    return os;
}