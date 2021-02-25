import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

if __name__ == '__main__':




    data_logs = [ '../qd-1.047199_K20.000000_B2.000000.csv',
                  '../qd-1.047199_K20.000000_B8.000000.csv',
                  '../qd-1.047199_K20.000000_B10.000000.csv',
                  '../qd-1.047199_K80.000000_B2.000000.csv',
                  '../qd-1.047199_K80.000000_B8.000000.csv',
                  '../qd-1.047199_K100.000000_B2.000000.csv',
                  '../qd-1.047199_K100.000000_B10.000000.csv',]

    first_index = 20
    last_index = 120

    for data_log in data_logs:

        data = pd.read_csv(data_log, skipinitialspace=True)

        time = np.add.accumulate(data['dt'][first_index:last_index]) - data['dt'][0] # é data['dt'][[0] mesmo pra subtrair o primeiro número gigante (segundos desde 1970)
        q = data['q'][first_index:last_index]
        tau = data['tau'][first_index:last_index]

        fig,ax = plt.subplots()
        ax.plot(q)
        ax.set_ylabel('q [rad]')

        ax2 = ax.twinx()
        ax2.plot(tau, color='red')
        ax2.set_ylabel('Tau [Nm]', color='red')

        ax.set_xlabel('Time [ms]')
        plt.show()