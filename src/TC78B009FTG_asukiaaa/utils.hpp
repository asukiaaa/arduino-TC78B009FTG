#pragma once

#include "./ValueTypes.hpp"

namespace TC78B009FTG_asukiaaa {
namespace utils {

String getStrOfKX(ValueTypes::KX kx) {
  return kx == ValueTypes::KX::x1 ? "x1" : "x8";
}

}  // namespace utils
}  // namespace TC78B009FTG_asukiaaa
