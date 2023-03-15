
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
    count = 0 

    detection_offset = 0.13
    ft_sensor.set_zeros()
    # While no change in force reading -> keep moving
    #while ft_readings[2]>=-0.35:
    while count < 2:
        
        finger_pos += 0.005
        hand.moveMotor(finger, finger_pos)

        ft_readings = ft_sensor.read_forces()
        #ft_sensor.set_zeros()
        print(ft_readings[2])

        min_point = finger_pos
        # Should now only detect contact if 2 concurrent loops register a great enough force
        if ft_readings[2]<=-0.2:
            count+=1
        else:
            count = 0
    
    print('Contact Detected!')

    return min_point



def main():

    # Initialise the dataframe to store the data being collected
    df = pd.DataFrame(columns=['Image_Name','Finger_Pos','FT_sensor'])

    finger_name = 'Thumb'
    
    # Initialise the tacip
    print('Initialising TacTip...')
    thumb_tactip = TacTip(320,240,40, finger_name, 13, -25, [110,0,225,240], 0, process=True, display=True)
    thumb_tactip.start_cap()
    time.sleep(1)
    thumb_tactip.start_processing_display()
    thumb_tactip.save_image('images/'+finger_name+'/'+'default.jpg')

    # Initialise the Dobot
    dobot = Dobot_Controller('192.168.1.6')
    dobot.client_dash.EnableRobot()
    dobot.client_dash.RobotMode()
    dobot.client_dash.PayLoad(0.5, 0.005)
    time.sleep(3)
    home = {'x':190, 'y':0, 'z':0, 'r':235}
    collection_start = {'x':355, 'y':-17, 'z':50, 'r':235}
    dobot.client_move.MovJ(home['x'], home['y'], home['z'], home['r']) # ensure the robot is at home
    time.sleep(10)
    dobot_positions = np.linspace(collection_start['x'], 330, 50)

    dobot_rots = np.linspace(220, 250, 10)

    # Initialise the F/T sensor
    ft_sensor = FTsensor('COM16')
    ft_sensor.set_zeros()

    # initialise the Model-O
    T = Model_O('COM12', 1,4,3,2,'MX', 0.4, 0.21, -0.1, 0.05)
    finger_dict ={'Thumb':3,'Middle':2,'Index':1}
    collection_finger = finger_dict[finger_name] # 1 is right, 2 is left, 3 is thumb
    T.reset() # reset the hand

    # Move to start collection
    dobot.client_move.MovJ(collection_start['x'], collection_start['y'], collection_start['z'], collection_start['r'])
    time.sleep(10)

    for n, j in enumerate(dobot_positions):

        print('Dobot Position Number: ',n)
        dobot.client_move.MovJ(j, collection_start['y'], collection_start['z'], collection_start['r'])
        time.sleep(0.5)

        for no, rot in enumerate(dobot_rots):

            print('Dobot rot Number: ',no)
            dobot.client_move.MovJ(j, collection_start['y'], collection_start['z'], rot)
            time.sleep(0.5)

            T.reset()
            min_finger_pos = move_to(T, ft_sensor, collection_finger)

            #min_finger_pos = 0.3 # find this by monitoring ft sensor or ssim?
            max_finger_pos = min_finger_pos + 0.1
            finger_positions = np.linspace(min_finger_pos,max_finger_pos,10)

            for i, pos in enumerate(finger_positions):
                T.moveMotor(collection_finger, pos)
                time.sleep(0.4)
                # Collect some data
                image_name = finger_name+'_'+str(no)+'_'+str(n)+'_'+str(i)+'.jpg'
                thumb_tactip.save_image('images/'+finger_name+'/'+image_name)
                
                # Read the F/T sensor
                ft_readings = ft_sensor.read_forces()
                print('Finger Pos: ', pos, 'Force: ', np.linalg.norm(ft_readings[:3]))

                df2 = pd.DataFrame([[image_name, pos, ft_readings[0], ft_readings[1], ft_readings[2], ft_readings[3], ft_readings[4], ft_readings[5]]],
                    columns=['Image_Name','Finger_Pos', 'fx', 'fy', 'fz', 'tx', 'ty', 'tz'])
                df = df.append(df2)

                time.sleep(0.2)
                T.moveMotor(collection_finger, min_finger_pos-0.1) # remove finger
                time.sleep(0.5)
                ft_sensor.set_zeros()
                time.sleep(1)

        print('Resetting Hand...')
        time.sleep(1)
        # Reset the hand
        #T.reset() # this is the line that breaks it - change to moveMotor()?
        T.moveMotor(collection_finger, 0)
        time.sleep(1)
        # Rezero the F/T sensor
        ft_sensor.set_zeros()


    T.release() # relax the hand
    thumb_tactip.stop() # stop the tactip
    dobot.client_move.MovJ(home['x'], home['y'], home['z'], home['r']) # return dobot to home pos
    time.sleep(3)

    df.to_csv('data/'+finger_name+'.csv')

    pass






if __name__ == '__main__':
    main()