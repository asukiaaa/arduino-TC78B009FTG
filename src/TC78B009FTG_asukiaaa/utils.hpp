#pragma once

#include "./ValueTypes.hpp"

namespace TC78B009FTG_asukiaaa {
namespace utils {

String getStrOfKX(ValueTypes::KX kx) {
  return kx == ValueTypes::KX::x1 ? F("x1") : F("x8");
}

}  // namespace utils
}  // namespace TC78B009FTG_asukiaaa
