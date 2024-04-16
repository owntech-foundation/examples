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

SPDX-License-Identifier: LGLPV2.1
"""

"""
@brief  This is a class for the factory test of Twitst 1.4.1

@author Regis Ruelland <regis.ruelland@laas.fr>
"""

import argparse 
import matplotlib.pyplot as plt
import numpy as np

if __name__ == '__main__':
    parser = argparse.ArgumentParser("plot_records", "plot_records <filename>")
    parser.add_argument("filename")
    args = parser.parse_args()
    with open(args.filename, 'r') as f:
        line1 = f.readline()
        datas = np.fromiter(f, dtype=float)
    names = line1.split(',')
    datas = datas.reshape(-1, len(names)-1)
    plt.plot(datas)
    plt.legend(names)
    plt.grid()
plt.show()
