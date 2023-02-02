from hardware import Dobot_Controller, TacTip
import time
import cv2



def main1():

    home = {'x':200, 'y':0, 'z':0, 'r':235}
    collection_start = {'x':390, 'y':-17, 'z':35, 'r':235}

    cont = Dobot_Controller('192.168.1.6')
    cont.client_dash.EnableRobot()
    cont.client_dash.RobotMode()
    cont.client_dash.PayLoad(0.5, 0.005)
    time.sleep(3)
    cont.client_dash.GetAngle()

    cont.client_move.MovJ(home['x'], home['y'], home['z'], home['r']) # ensure the robot is at home

    time.sleep(5)

    cont.client_move.MovJ(collection_start['x'], collection_start['y'], collection_start['z'], collection_start['r']) # data collection start point

    time.sleep(10)

    cont.client_move.MovJ(home['x'], home['y'], home['z'], home['r']) # ensure the robot is at home

    time.sleep(5)

    cont.client_dash.DisableRobot()


def main():


    name = 'thumb'
    fps=40
    width=320
    height=240

    vid = cv2.VideoCapture(0)
    if not vid.isOpened():
        print("Cannot open camera " + name)
        exit()
    vid.set(cv2.CAP_PROP_FPS, fps)
    vid.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    vid.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    
    # Let camera warm up
    time.sleep(1)

    while True:
        ret, frame = vid.read()
        if ret == True:
            cv2.imshow('test', frame)
        else:
            print('No Frame...')

        cv2.waitKey()
        #time.sleep(0.1)




if __name__ =='__main__':
    main()