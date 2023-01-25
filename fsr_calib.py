import time
import pandas as pd
from openhand_node.src.openhand_node.hardware import FSR, FTsensor
import matplotlib.pyplot as plt



def main():

    no_cycles = 3
    readings_per_cycle = 5

    # Initialise the serial monitor
    fsr = FSR('COM9', '9600')

    # Initialise the F/T sensor
    #ft_sensor = FTsensor('/dev/ttyUSB1')
    #ft_sensor.set_zeros()

    # Initialise the dataframe
    df = pd.DataFrame(columns=['FSR_Reading', 'Force'])

    for i in range(no_cycles):

        print('Starting Cycle ',i)
        print('Place Finger on F/T Sensor...')
        time.sleep(3)
        print('Collecting Data...')

        for j in range(readings_per_cycle):

            # get fsr reading and print
            fsr_reading = fsr.read_sensor()
            print('FSR Reading: ', fsr_reading)

            # get f/t sensor reading and print
            #ft_readings = ft_sensor.read_forces()
            ft_readings = [0,0,1]
            print('Force: ', ft_readings[2])

            # add to dataframe
            df2 = pd.DataFrame([[fsr_reading, ft_readings[2]]],
            columns=['FSR_Reading', 'Force'])
            df = df.append(df2)

            time.sleep(0.001)

        print('Cycle Complete, Remove Finger...')
        time.sleep(5)
        print('Resetting F/T Sensor...')
        #ft_sensor.set_zeros()

    print('Data Collection Complete...')

    # plot the collected data
    plt.plot(df['FSR_Reading'], df['Force'])
    plt.show()

    # save the df as a csv
    df.to_csv('FSR_calib_data.csv')


if __name__ == '__main__':
    main()