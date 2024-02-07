import time
import struct
import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
from pyocd.core.helpers import ConnectHelper
from pyocd.flash.file_programmer import FileProgrammer
from pyocd.debug.elf.symbols import ELFSymbolProvider


# Replace ST_LINK_ID with the ID of your st-link, on the command promt
# enter "pyocd list" to get the ID.
ST_LINK_ID='002900364741500520383733'

options = {'connect_mode': 'attach', 'target_override': 'stm32g474retx'}
session = ConnectHelper.session_with_chosen_probe(ST_LINK_ID, options=options)
session.open()
session.target.elf = '.pio/build/bootloader_stlink/firmware.elf'
provider = ELFSymbolProvider(session.target.elf)
addr = provider.get_symbol_value("record_array")
if addr is None :
    print("problem address")
RECORD_SIZE = 2048
curves = [
        {'name': 'I_low', 'format': 'f',},
        {'name': 'V_low', 'format': 'f',},
        {'name': 'Vhigh_value', 'format': 'f',},
        {'name': 'Iref', 'format': 'f',},
        {'name': 'duty_cycle', 'format': 'f',},
        {'name': 'Vref', 'format': 'f',},
        {'name': 'angle', 'format': 'f',}
        ]

print(f"record addr = {addr:x}")


tic = time.time()
datas = session.target.read_memory_block32(addr, 7*RECORD_SIZE)
toc = time.time()
print(f"time : {toc-tic}")
datas = np.reshape(datas, (-1, len(curves)))
results = {}
for k, curve in enumerate(curves):
    results[curve['name']] = [struct.unpack(curve['format'], struct.pack('I', d))[0] for d in datas[:, k]]


fig, axs = plt.subplots(4,sharex=True)

axs[0].plot(results['V_low'],label = 'V_low1')
axs[0].plot(results['Vhigh_value'],label = 'Vhigh_value')
axs[0].plot(results['Vref'],label = 'Vref')
axs[0].set(ylabel='voltage (V)')
axs[0].legend(loc='upper right')

axs[1].plot(results['I_low'],label = 'I_low1')
axs[1].plot(results['Iref'],label = 'Iref')
axs[1].set(ylabel='current (A)')
axs[1].legend(loc='upper right')

axs[2].plot(results['duty_cycle'],label = 'duty_cycle')
axs[2].set(ylabel='duty_cycle')
axs[2].set_ylim([0,1])

axs[3].plot(results['angle'],label = 'angle')
axs[3].set(ylabel='angle (rad)')


plt.show()

session.close()
