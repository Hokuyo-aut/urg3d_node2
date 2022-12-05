// Copyright 2022 HOKUYO AUTOMATIC CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file auxiliary_define.hpp
 * @brief 補助データ数値定義
 */

#ifndef AUXILIARY_DEFINE_HPP
#define AUXILIARY_DEFINE_HPP

#define MAXIMUM_RECORD_TIMES 10

#define GYRO_FACTOR (500.0 / 0x7FFF)
#define ACCEL_FACTOR (4.0 / 0x7FFF)
#define COMPASS_FACTOR 0.15f
#define TEMPERATURE_FACTOR 333.87
#define TEMPERATURE_OFFSET 21.0

#endif // AUXILIARY_DEFINE_HPP
