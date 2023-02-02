
from gc import collect
from hands import Model_O
import time
import numpy as np
from hardware import TacTip, FSR, FTsensor, Dobot_Controller
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


    finger_name = 'Thumb'
    
    # Initialise the tacip
    print('Initialising TacTip...')
    thumb_tactip = TacTip(320,240,40, finger_name, 13, -25, [110,0,225,240], 0, process=True, display=True)
    thumb_tactip.start_cap()
    time.sleep(1)
    thumb_tactip.start_processing_display()

    # Initialise the Dobot
    dobot = Dobot_Controller('192.168.1.6')
    dobot.client_dash.EnableRobot()
    dobot.client_dash.RobotMode()
    dobot.client_dash.PayLoad(0.5, 0.005)
    time.sleep(3)
    home = {'x':200, 'y':0, 'z':0, 'r':235}
    collection_start = {'x':410, 'y':-17, 'z':35, 'r':235}
    dobot.client_move.MovJ(home['x'], home['y'], home['z'], home['r']) # ensure the robot is at home
    time.sleep(10)
    dobot_positions = np.linspace(collection_start['x'], 430, 50)

    # Initialise the F/T sensor
    ft_sensor = FTsensor('COM11')
    ft_sensor.set_zeros()

    # initialise the Model-O
    T = Model_O('COM12', 1,4,3,2,'MX', 0.4, 0.21, -0.1, 0.05)
    finger_dict ={'Thumb':3,'Right':1,'Left':2}
    collection_finger = finger_dict[finger_name] # 1 is right, 2 is left, 3 is thumb
    T.reset() # reset the hand

    # Move to start collection
    dobot.client_move.MovJ(collection_start['x'], collection_start['y'], collection_start['z'], collection_start['r'])
    time.sleep(10)

    for j in dobot_positions:

        dobot.client_move.MovJ(j, collection_start['y'], collection_start['z'], collection_start['r'])
        time.sleep(3)

        min_finger_pos = move_to(T, ft_sensor, collection_finger)

        #min_finger_pos = 0.3 # find this by monitoring ft sensor or ssim?
        max_finger_pos = min_finger_pos + 0.1
        finger_positions = np.linspace(min_finger_pos,max_finger_pos,100) 

        # for hand_pos in hand_pose_list:
        # Could wrap this in another loop to move the hand up/down 
        for pos in finger_positions:
            T.moveMotor(collection_finger, pos)
            print('Finger Pos: ', pos)

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