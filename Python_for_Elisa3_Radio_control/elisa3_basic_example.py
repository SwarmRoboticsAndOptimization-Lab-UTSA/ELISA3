import elisa3
import time

robotAddr = [4050, 4096, 4469, 4021, 4060, 4104, 4101, 4083]
elisa = elisa3.Elisa3(robotAddr)
elisa.start()

counter = 0

while 1:

    if(counter == 5):
        for addr in robotAddr:
            elisa.setRed(addr, 10)
            elisa.setGreen(addr, 0)
            elisa.setBlue(addr, 0)
        #elisa.setRed(robotAddr[0], 10)
        #elisa.setGreen(robotAddr[0], 0)
        #elisa.setBlue(robotAddr[0], 0)
        elisa.setSmallLed(robotAddr[0], 0, 1)
        elisa.setSmallLed(robotAddr[0], 1, 0)
        elisa.setSmallLed(robotAddr[0], 2, 0)
        elisa.setSmallLed(robotAddr[0], 3, 0)
        elisa.setSmallLed(robotAddr[0], 4, 1)
        elisa.setSmallLed(robotAddr[0], 5, 0)
        elisa.setSmallLed(robotAddr[0], 6, 0)
        elisa.setSmallLed(robotAddr[0], 7, 0)
    elif(counter == 10):
        for addr in robotAddr:
            elisa.setRed(addr, 0)
            elisa.setGreen(addr, 10)
            elisa.setBlue(addr, 0)      
        #elisa.setRed(robotAddr[0], 0)
        #elisa.setGreen(robotAddr[0], 10)
        #elisa.setBlue(robotAddr[0], 0)              
        elisa.setLeftSpeed(robotAddr[0], 10)
        elisa.setRightSpeed(robotAddr[0], 10)
        elisa.turnOnFrontIRs(robotAddr[0])
        elisa.turnOffBackIR(robotAddr[0])
        elisa.setSmallLed(robotAddr[0], 0, 0)
        elisa.setSmallLed(robotAddr[0], 1, 1)
        elisa.setSmallLed(robotAddr[0], 2, 0)
        elisa.setSmallLed(robotAddr[0], 3, 0)
        elisa.setSmallLed(robotAddr[0], 4, 0)
        elisa.setSmallLed(robotAddr[0], 5, 1)
        elisa.setSmallLed(robotAddr[0], 6, 0)
        elisa.setSmallLed(robotAddr[0], 7, 0)        
    elif(counter == 15):
        for addr in robotAddr:
            elisa.setRed(addr, 0)
            elisa.setGreen(addr, 0)
            elisa.setBlue(addr, 10)        
        #elisa.setRed(robotAddr[0], 0)
        #elisa.setGreen(robotAddr[0], 0)
        #elisa.setBlue(robotAddr[0], 10)
        elisa.setSmallLed(robotAddr[0], 0, 0)
        elisa.setSmallLed(robotAddr[0], 1, 0)
        elisa.setSmallLed(robotAddr[0], 2, 1)
        elisa.setSmallLed(robotAddr[0], 3, 0)
        elisa.setSmallLed(robotAddr[0], 4, 0)
        elisa.setSmallLed(robotAddr[0], 5, 0)
        elisa.setSmallLed(robotAddr[0], 6, 1)
        elisa.setSmallLed(robotAddr[0], 7, 0)
    elif(counter == 20):
        counter = 0
        for addr in robotAddr:
            elisa.setRed(addr, 0)
            elisa.setGreen(addr, 0)
            elisa.setBlue(addr, 0)        
        elisa.setLeftSpeed(robotAddr[0], -10)
        elisa.setRightSpeed(robotAddr[0], -10)
        elisa.turnOffFrontIRs(robotAddr[0])
        elisa.turnOnBackIR(robotAddr[0])        
        elisa.setSmallLed(robotAddr[0], 0, 0)
        elisa.setSmallLed(robotAddr[0], 1, 0)
        elisa.setSmallLed(robotAddr[0], 2, 0)
        elisa.setSmallLed(robotAddr[0], 3, 1)
        elisa.setSmallLed(robotAddr[0], 4, 0)
        elisa.setSmallLed(robotAddr[0], 5, 0)
        elisa.setSmallLed(robotAddr[0], 6, 0)
        elisa.setSmallLed(robotAddr[0], 7, 1)          
        print(str(robotAddr[0]) + " battery = " + str(elisa.getBatteryPercent(robotAddr[0])))
        print(str(robotAddr[0]) + " selector = " + str(elisa.getSelector(robotAddr[0])))        

    for addr in robotAddr:
        print(str(addr) + " prox: " + str(elisa.getAllProximity(addr)))
    print()
    
    #prox = elisa.getAllProximity(robotAddr[0])
    #print(str(robotAddr[0]) + " prox: " + str(prox))
    #prox = elisa.getAllProximity(robotAddr[4])
    #print(str(robotAddr[4]) + " prox: " + str(prox))

    #ground = elisa.getAllGround(robotAddr[0])
    #print(str(robotAddr[0]) + " ground: " + str(ground))

    #accx = elisa.getAccX(robotAddr[0])
    #accy = elisa.getAccY(robotAddr[0])
    #accz = elisa.getAccZ(robotAddr[0])
    #print(str(robotAddr[0]) + " acc: " + str(accx) + ", " + str(accy) + ", " + str(accz))

    #print(str(robotAddr[0]) + " theta: " + str(elisa.getOdomTheta(robotAddr[0])))
    #print(str(robotAddr[0]) + " xpos: " + str(elisa.getOdomXpos(robotAddr[0])))
    #print(str(robotAddr[0]) + " ypos: " + str(elisa.getOdomYpos(robotAddr[0])))
    #print(str(robotAddr[0]) + " left steps: " + str(elisa.getLeftMotSteps(robotAddr[0])))
    #print(str(robotAddr[0]) + " right steps: " + str(elisa.getRightMotSteps(robotAddr[0])))


    counter = counter + 1
    time.sleep(0.1)
