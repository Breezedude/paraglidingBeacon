#pragma once
// ===========================================================================
//  Battery state of charge from the resting cell voltage.
//
//  A lithium cell does not discharge in a straight line: it drops quickly from
//  full, spends most of its capacity between 3.9 and 3.7 V and then falls away.
//  A linear scale reads far too low in the middle and hides the last third
//  entirely, so the percentage comes from the discharge curve instead.
// ===========================================================================

#include <cstdint>

namespace battery {

// 0..100, interpolated along the open circuit voltage curve of the cell.
uint8_t percentFromMillivolts(uint16_t millivolts);

}  // namespace battery
