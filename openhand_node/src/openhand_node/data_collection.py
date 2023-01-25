
from gc import collect
from hands import Model_O
import time
import numpy as np
from hardware import TacTip, FSR, FTsensor
import pandas as pd



def move_to(hand, ft_sensor, finger):
    
    hand.reset()
    ft_sensor.set_zeros()
    ft_readings = ft_sensor.read_forces()
    print('Initial forces: ',ft_readings)
    finger_pos = 0

    detection_offset = 0.13

    # While no change in force reading -> keep moving
    while ft_readings[2] < 0.3 and ft_readings[2]>=-0.2: 
        
        finger_pos += 0.001
        hand.moveMotor(finger, finger_pos)

        ft_readings = ft_sensor.read_forces()
        ft_sensor.set_zeros()
        print(round(ft_readings[2],1))

        min_point = finger_pos
    
    print('Contact Detected!')

    return min_point



def main():

    # Initialise the dataframe to store the data being collected
    df = pd.DataFrame(columns=['Image_Name','Finger_Pos','FT_sensor','FSR_reading'])


    finger_name = 'Right'
    
    # Initialise the tacip
    print('Initialising TacTip...')
    thumb_tactip = TacTip(320,240,40, finger_name, 13, -25, [110,0,225,240], 0, process=True, display=True)
    thumb_tactip.start_cap()
    time.sleep(1)
    thumb_tactip.start_processing_display()

    # Initialise the FSR
    #fsr = FSR('/dev/ttyACM0', '9600')

    # Initialise the F/T sensor
    ft_sensor = FTsensor('/dev/ttyUSB1')
    ft_sensor.set_zeros()

    # initialise the Model-O
    T = Model_O('/dev/ttyUSB0', 1,4,3,2,'MX', 0.4, 0.21, -0.1, 0.05)
    finger_dict ={'Thumb':3,'Right':1,'Left':2}
    collection_finger = finger_dict[finger_name] # 1 is right, 2 is left, 3 is thumb
    T.reset() # reset the hand


    min_finger_pos = move_to(T, ft_sensor, collection_finger)

    #min_finger_pos = 0.3 # find this by monitoring ft sensor or ssim?
    max_finger_pos = min_finger_pos + 0.1
    finger_positions = np.linspace(min_finger_pos,max_finger_pos,100) 

    # for hand_pos in hand_pose_list:
    # Could wrap this in another loop to move the hand up/down 
    for pos in finger_positions:
        T.moveMotor(collection_finger, pos)

        # Collect some data
        image_name = finger_name+'_'+str(pos)+'.jpg'
        thumb_tactip.save_image('images/'+finger_name+'/'+image_name)

        # tactip.save_frame
        
        # Read the F/T sensor
        ft_readings = ft_sensor.read_forces()
        
        # Read the fsr
        #fsr_reading = fsr.read_sensor()
        fsr_reading =10

        ssim = thumb_tactip.get_ssim()

        df2 = pd.DataFrame([[image_name, pos, ssim, ft_readings[0], ft_readings[1], ft_readings[2], ft_readings[3], ft_readings[4], ft_readings[5], fsr_reading]],
            columns=['Image_Name','Finger_Pos', 'SSIM', 'fx', 'fy', 'fz', 'tx', 'ty', 'tz','FSR_reading'])
        df = df.append(df2)

        time.sleep(0.05)

    # Reset the hand
    T.reset()
    # Rezero the F/T sensor
    ft_sensor.set_zeros()




    T.release() # relax the hand
    thumb_tactip.stop() # stop the tactip

    df.to_csv('data/'+finger_name+'.csv')

    pass






if __name__ == '__main__':
    main()