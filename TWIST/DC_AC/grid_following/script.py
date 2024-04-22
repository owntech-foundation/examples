"""
Copyright (c) 2021-2024 LAAS-CNRS

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation, either version 2.1 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.

SPDX-License-Identifier: LGPL-2.1
"""

"""
@brief  This is a script to get data from the memory of the SPIN board

@author Regis Ruelland <regis.ruelland@laas.fr>
"""

import time
import struct
import numpy as np
import pandas as pd
from matplotlib import pyplot as plt

RECORD_SIZE = 2048
NB_CURVES = 11
curves = [
        {'name': 'I1_low', 'format': 'h',},
        {'name': 'I2_low', 'format': 'h',},
        {'name': 'V1_low', 'format': 'h',},
        {'name': 'V2_low', 'format': 'h',},
        {'name': 'Ihigh_value', 'format': 'h',},
        {'name': 'Iref', 'format': 'h',},
        {'name': 'duty_cycle', 'format': 'h',},
        {'name': 'Vref', 'format': 'h',},
        {'name': 'pll_angle', 'format': 'h',},
        {'name': 'pll_error', 'format': 'h',},
        {'name': 'pll_w', 'format': 'h',},
        ]

# we assume the datas are in a file named "records.dat" in the same directory than the
# script
df = pd.read_csv('records.dat', delimiter=' ', names=['k', 'data'])
datas = [int(d, 16) for d in df['data']]
print(f"time : {toc-tic}")
datas = np.reshape(datas, (-1, len(curves)))
results = {}
for k, curve in enumerate(curves):
    results[curve['name']] = [1. / 256. * struct.unpack('h', struct.pack('H', d))[0] for d in datas[:, k]] 

results = pd.DataFrame(results)
results.to_csv('latest_result.csv')

