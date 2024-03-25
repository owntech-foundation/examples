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
