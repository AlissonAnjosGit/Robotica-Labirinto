import sim
import time

#definicoes iniciais
serverIP = '127.0.0.1'
serverPort = 19999
leftMotorHandle = 0
vLeft = 0.
rightMotorHandle = 0
vRight = 0.
sensorHandle = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
noDetectionDist=0.3
maxDetectionDist=0.002
detect=[0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
braitenbergL=[-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
braitenbergR=[-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

v0=4
turnVel = 1
times = 0
orientation = [0.0, 0.0, 0.0]
robotBetweenCells = False

#matriz com as posicoes de cada celula do labirinto
#[xo, x, yo, y]
cellsArray = [               #0                           #1                           #2                          #3                         #4                         #5
                [[-2.35, -1.55, -2.35, -1.55],[-2.35, -1.55, -1.41, -0.65],[-2.35, -1.55, -0.51, 0.01],[-2.35, -1.55, 0.11, 0.81],[-2.35, -1.55, 0.91, 1.55],[-2.35, -1.55, 1.61, 2.35]],#0  
                [[-1.41, -0.74, -2.35, -1.55],[-1.41, -0.74, -1.41, -0.65],[-1.41, -0.74, -0.51, 0.01],[-1.41, -0.74, 0.11, 0.81],[-1.41, -0.74, 0.91, 1.55],[-1.41, -0.74, 1.61, 2.35]],#1
                [[-0.65,  0.07, -2.35, -1.55],[-0.65,  0.07, -1.41, -0.65],[-0.65,  0.07, -0.51, 0.01],[-0.65,  0.07, 0.11, 0.81],[-0.65,  0.07, 0.91, 1.55],[-0.65,  0.07, 1.61, 2.35]],#2
                [[ 0.17,  0.87, -2.35, -1.55],[ 0.17,  0.87, -1.41, -0.65],[ 0.17,  0.87, -0.51, 0.01],[ 0.17,  0.87, 0.11, 0.81],[ 0.17,  0.87, 0.91, 1.55],[ 0.17,  0.87, 1.61, 2.35]],#3
                [[ 0.97,  1.57, -2.35, -1.55],[ 0.97,  1.57, -1.41, -0.65],[ 0.97,  1.57, -0.51, 0.01],[ 0.97,  1.57, 0.11, 0.81],[ 0.97,  1.57, 0.91, 1.55],[ 0.97,  1.57, 1.61, 2.35]],#4
                [[ 1.65,  2.35, -2.35, -1.55],[ 1.65,  2.35, -1.41, -0.65],[ 1.65,  2.35, -0.51, 0.01],[ 1.65,  2.35, 0.11, 0.81],[ 1.65,  2.35, 0.91, 1.55],[ 1.65,  2.35, 1.61, 2.35]] #5
              ]




#funcao de rotacao do robo
def axialRotation (direction, orientation):
    orientation = sim.simxGetObjectOrientation(clientID, gps, -1, sim.simx_opmode_buffer)
    while(1): 
        if orientation[1][2] > 0.750001 and orientation[1][2] < 2.249999: #Heading North 
            if direction == 'Right':
                while orientation[1][2] > 0.03:
                    orientation = sim.simxGetObjectOrientation(clientID, gps, -1, sim.simx_opmode_buffer)
                    erro = sim.simxSetJointTargetVelocity(clientID, leftMotorHandle, turnVel, sim.simx_opmode_streaming)
                    erro = sim.simxSetJointTargetVelocity(clientID, rightMotorHandle, -turnVel, sim.simx_opmode_streaming)
                break
            if direction == 'Left':
                while orientation[1][2] < 3.1:
                    orientation = sim.simxGetObjectOrientation(clientID, gps, -1, sim.simx_opmode_buffer)
                    erro = sim.simxSetJointTargetVelocity(clientID, leftMotorHandle, -turnVel, sim.simx_opmode_streaming)
                    erro = sim.simxSetJointTargetVelocity(clientID, rightMotorHandle, turnVel, sim.simx_opmode_streaming)
                break
            
        if orientation[1][2] > 2.250001 or orientation[1][2] < -2.250001: #Heading West
            if direction == 'Right':
                while orientation[1][2] > 1.63 or orientation[1][2] < 0:
                    orientation = sim.simxGetObjectOrientation(clientID, gps, -1, sim.simx_opmode_buffer)
                    erro = sim.simxSetJointTargetVelocity(clientID, leftMotorHandle, turnVel, sim.simx_opmode_streaming)
                    erro = sim.simxSetJointTargetVelocity(clientID, rightMotorHandle, -turnVel, sim.simx_opmode_streaming)
                break
            if direction == 'Left':
                while orientation[1][2] < -1.63 or orientation[1][2] > 0:
                    orientation = sim.simxGetObjectOrientation(clientID, gps, -1, sim.simx_opmode_buffer)
                    erro = sim.simxSetJointTargetVelocity(clientID, leftMotorHandle, -turnVel, sim.simx_opmode_streaming)
                    erro = sim.simxSetJointTargetVelocity(clientID, rightMotorHandle, turnVel, sim.simx_opmode_streaming)
                break

        if orientation[1][2] > -2.249999 and orientation[1][2] < -0.750001: #Heading South
            if direction == 'Right':
                while orientation[1][2] > -3.1:
                    orientation = sim.simxGetObjectOrientation(clientID, gps, -1, sim.simx_opmode_buffer)
                    erro = sim.simxSetJointTargetVelocity(clientID, leftMotorHandle, turnVel, sim.simx_opmode_streaming)
                    erro = sim.simxSetJointTargetVelocity(clientID, rightMotorHandle, -turnVel, sim.simx_opmode_streaming)
                break
            if direction == 'Left':
                while orientation[1][2] < -0.03:
                    orientation = sim.simxGetObjectOrientation(clientID, gps, -1, sim.simx_opmode_buffer)
                    erro = sim.simxSetJointTargetVelocity(clientID, leftMotorHandle, -turnVel, sim.simx_opmode_streaming)
                    erro = sim.simxSetJointTargetVelocity(clientID, rightMotorHandle, turnVel, sim.simx_opmode_streaming)
                break

        if orientation[1][2] > -0.749999 or orientation[1][2] < 0.749999: #Heading East
            if direction == 'Right':
                while orientation[1][2] > -1.5:
                    orientation = sim.simxGetObjectOrientation(clientID, gps, -1, sim.simx_opmode_buffer)
                    erro = sim.simxSetJointTargetVelocity(clientID, leftMotorHandle, turnVel, sim.simx_opmode_streaming)
                    erro = sim.simxSetJointTargetVelocity(clientID, rightMotorHandle, -turnVel, sim.simx_opmode_streaming)
                break
            if direction == 'Left':
                while orientation[1][2] < 1.5:
                    orientation = sim.simxGetObjectOrientation(clientID, gps, -1, sim.simx_opmode_buffer)
                    erro = sim.simxSetJointTargetVelocity(clientID, leftMotorHandle, -turnVel, sim.simx_opmode_streaming)
                    erro = sim.simxSetJointTargetVelocity(clientID, rightMotorHandle, turnVel, sim.simx_opmode_streaming)
                break

#funcao para localizacao atual do robo
def currentCell(objectAbsolutePosition):
    xRobot = objectAbsolutePosition[1][0]
    yRobot = objectAbsolutePosition[1][1]
    for x in range(6):
        for y in range(6):
            if xRobot > cellsArray[x][y][0] and xRobot < cellsArray[x][y][1]:
                if yRobot > cellsArray[x][y][2] and yRobot < cellsArray[x][y][3]:
                    robotBetweenCells = False
                    return [x,y]
                else:
                    robotBetweenCells = True
            else:
                robotBetweenCells = True
                #para ajustar na função de andar pra frente
                #se igual a true ligar os motores em ré(com sinal de -) por 0.05secs



#funcao para saber a proxima celula da matriz
def nextCell(x, y, orientation):

    nextCellRange = [0, 0, 0, 0]
    if orientation[1][2] > 0.750001 and orientation[1][2] < 2.249999: #Heading North
        nextCellRange = cellsArray[x][y+1]
        return nextCellRange
            
    if orientation[1][2] > 2.250001 or orientation[1][2] < -2.250001: #Heading West
        nextCellRange = cellsArray[x-1][y]
        return nextCellRange

    if orientation[1][2] > -2.249999 and orientation[1][2] < -0.750001: #Heading South
        nextCellRange = cellsArray[x][y-1]
        return nextCellRange

    if orientation[1][2] > -0.749999 or orientation[1][2] < 0.749999: #Heading East
        nextCellRange = cellsArray[x+1][y]
        return nextCellRange
  

#funcao de movimento linear, uma celula da matriz por vez
def fwdStep(objectAbsolutePosition, orientation):

    xyRobotCell = currentCell(objectAbsolutePosition)
    #currentCellRange = cellsArray[xyRobotCell[0]][xyRobotCell[1]]

    
    while(1):
        if robotBetweenCells == True:
            erro = sim.simxSetJointTargetVelocity(clientID, leftMotorHandle, -turnVel, sim.simx_opmode_streaming)
            erro = sim.simxSetJointTargetVelocity(clientID, rightMotorHandle, -turnVel, sim.simx_opmode_streaming)
            time.sleep(0.05)
        try:
            nextCellRange = nextCell(xyRobotCell[0], xyRobotCell[1], orientation)

        except IndexError:
            axialRotation ('Left', orientation)
            axialRotation ('Left', orientation)
            break 
            
        print(nextCellRange)
        if orientation[1][2] > 0.750001 and orientation[1][2] < 2.249999: #Heading North
            center = (abs(abs(nextCellRange[2])-abs(nextCellRange[3])))/2
            
            while(objectAbsolutePosition[1][1] < (nextCellRange[3])-center):
                for i in range(16):
                    erro, state, coord, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID, sensorHandle[i],sim.simx_opmode_buffer)
                    if erro == 0:
                        dist = coord[2]
                        if state > 0 and dist < noDetectionDist:
                            if dist < maxDetectionDist:
                                dist = maxDetectionDist

                            detect[i] = 1-((dist-maxDetectionDist) / (noDetectionDist-maxDetectionDist))
                        else:
                            detect[i] = 0
                    else:
                        detect[i] = 0

                vLeft = v0
                vRight = v0

                for i in range(16):
                    vLeft  = vLeft  + braitenbergL[i] * detect[i]
                    vRight = vRight + braitenbergR[i] * detect[i]
                
                objectAbsolutePosition = sim.simxGetObjectPosition(clientID, gps, -1, sim.simx_opmode_buffer)
                erro = sim.simxSetJointTargetVelocity(clientID, leftMotorHandle, vLeft, sim.simx_opmode_streaming)
                erro = sim.simxSetJointTargetVelocity(clientID, rightMotorHandle, vRight, sim.simx_opmode_streaming)
                
            erro = sim.simxSetJointTargetVelocity(clientID, leftMotorHandle, 0, sim.simx_opmode_streaming)
            erro = sim.simxSetJointTargetVelocity(clientID, rightMotorHandle, 0, sim.simx_opmode_streaming)
            time.sleep(0.1)
            break
            
        if orientation[1][2] > 2.250001 or orientation[1][2] < -2.250001: #Heading West
            center = (abs(abs(nextCellRange[1])-abs(nextCellRange[0])))/2
            
            while(objectAbsolutePosition[1][0] > (nextCellRange[0])+center):
                for i in range(16):
                    erro, state, coord, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID, sensorHandle[i],sim.simx_opmode_buffer)
                    if erro == 0:
                        dist = coord[2]
                        if state > 0 and dist < noDetectionDist:
                            if dist < maxDetectionDist:
                                dist = maxDetectionDist

                            detect[i] = 1-((dist-maxDetectionDist) / (noDetectionDist-maxDetectionDist))
                        else:
                            detect[i] = 0
                    else:
                        detect[i] = 0

                vLeft = v0
                vRight = v0

                for i in range(16):
                    vLeft  = vLeft  + braitenbergL[i] * detect[i]
                    vRight = vRight + braitenbergR[i] * detect[i]
                
                objectAbsolutePosition = sim.simxGetObjectPosition(clientID, gps, -1, sim.simx_opmode_buffer)
                erro = sim.simxSetJointTargetVelocity(clientID, leftMotorHandle, vLeft, sim.simx_opmode_streaming)
                erro = sim.simxSetJointTargetVelocity(clientID, rightMotorHandle, vRight, sim.simx_opmode_streaming)
            erro = sim.simxSetJointTargetVelocity(clientID, leftMotorHandle, 0, sim.simx_opmode_streaming)
            erro = sim.simxSetJointTargetVelocity(clientID, rightMotorHandle, 0, sim.simx_opmode_streaming)
            time.sleep(0.1)
            break            

        if orientation[1][2] > -2.249999 and orientation[1][2] < -0.750001: #Heading South
            center = (abs(abs(nextCellRange[2])-abs(nextCellRange[3])))/2
            
            while(objectAbsolutePosition[1][1] > (nextCellRange[2])+center):
                for i in range(16):
                    erro, state, coord, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID, sensorHandle[i],sim.simx_opmode_buffer)
                    if erro == 0:
                        dist = coord[2]
                        if state > 0 and dist < noDetectionDist:
                            if dist < maxDetectionDist:
                                dist = maxDetectionDist

                            detect[i] = 1-((dist-maxDetectionDist) / (noDetectionDist-maxDetectionDist))
                        else:
                            detect[i] = 0
                    else:
                        detect[i] = 0

                vLeft = v0
                vRight = v0

                for i in range(16):
                    vLeft  = vLeft  + braitenbergL[i] * detect[i]
                    vRight = vRight + braitenbergR[i] * detect[i]
                
                objectAbsolutePosition = sim.simxGetObjectPosition(clientID, gps, -1, sim.simx_opmode_buffer)
                erro = sim.simxSetJointTargetVelocity(clientID, leftMotorHandle, vLeft, sim.simx_opmode_streaming)
                erro = sim.simxSetJointTargetVelocity(clientID, rightMotorHandle, vRight, sim.simx_opmode_streaming)
            erro = sim.simxSetJointTargetVelocity(clientID, leftMotorHandle, 0, sim.simx_opmode_streaming)
            erro = sim.simxSetJointTargetVelocity(clientID, rightMotorHandle, 0, sim.simx_opmode_streaming)
            time.sleep(0.1)
            break
            

        if orientation[1][2] > -0.749999 or orientation[1][2] < 0.749999: #Heading East
            center = (abs(abs(nextCellRange[1])-abs(nextCellRange[0])))/2
            
            while(objectAbsolutePosition[1][0] < (nextCellRange[1])-center):
                for i in range(16):
                    erro, state, coord, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID, sensorHandle[i],sim.simx_opmode_buffer)
                    if erro == 0:
                        dist = coord[2]
                        if state > 0 and dist < noDetectionDist:
                            if dist < maxDetectionDist:
                                dist = maxDetectionDist

                            detect[i] = 1-((dist-maxDetectionDist) / (noDetectionDist-maxDetectionDist))
                        else:
                            detect[i] = 0
                    else:
                        detect[i] = 0

                vLeft = v0
                vRight = v0

                for i in range(16):
                    vLeft  = vLeft  + braitenbergL[i] * detect[i]
                    vRight = vRight + braitenbergR[i] * detect[i]
                
                objectAbsolutePosition = sim.simxGetObjectPosition(clientID, gps, -1, sim.simx_opmode_buffer)
                erro = sim.simxSetJointTargetVelocity(clientID, leftMotorHandle, vLeft, sim.simx_opmode_streaming)
                erro = sim.simxSetJointTargetVelocity(clientID, rightMotorHandle, vRight, sim.simx_opmode_streaming)
            erro = sim.simxSetJointTargetVelocity(clientID, leftMotorHandle, 0, sim.simx_opmode_streaming)
            erro = sim.simxSetJointTargetVelocity(clientID, rightMotorHandle, 0, sim.simx_opmode_streaming)
            time.sleep(0.1)
            break

#funcoes para detectar paredes na frente, direita e esquerda (sensores 4 e 5, 8 e 9, 1 e 16 respectivamente)
def isPathRight():
                                                                                                                          #sensor9
    erro1, state1, coord1, detectedObjectHandle1, detectedSurfaceNormalVector1 = sim.simxReadProximitySensor(clientID, sensorHandle[7],sim.simx_opmode_buffer)
                                                                                                                          #sensor8
    erro2, state2, coord2, detectedObjectHandle2, detectedSurfaceNormalVector2 = sim.simxReadProximitySensor(clientID, sensorHandle[8],sim.simx_opmode_buffer)

    if (coord1[2] > 0.7 and coord2[2] > 0.7) or (state1 == False or state2 == False):
        return True
    else:
        return False

 
  
    

def isPathLeft():
                                                                                                                           #sensor1
    erro1, state1, coord1, detectedObjectHandle1, detectedSurfaceNormalVector1 = sim.simxReadProximitySensor(clientID, sensorHandle[0],sim.simx_opmode_buffer)
                                                                                                                           #sensor16
    erro2, state2, coord2, detectedObjectHandle2, detectedSurfaceNormalVector2 = sim.simxReadProximitySensor(clientID, sensorHandle[15],sim.simx_opmode_buffer)

    if (coord1[2] > 0.7 and coord2[2] > 0.7) or (state1 == False or state2 == False):
        return True
    else:
        return False

def isPathFwd():
                                                                                                                           #sensor4
    erro1, state1, coord1, detectedObjectHandle1, detectedSurfaceNormalVector1 = sim.simxReadProximitySensor(clientID, sensorHandle[3],sim.simx_opmode_buffer)
                                                                                                                           #sensor5
    erro2, state2, coord2, detectedObjectHandle2, detectedSurfaceNormalVector2 = sim.simxReadProximitySensor(clientID, sensorHandle[4],sim.simx_opmode_buffer)

    if (coord1[2] > 0.7 and coord2[2] > 0.7) or (state1 == False or state2 == False):
        return True
    else:
        return False





        



clientID = sim.simxStart(serverIP,serverPort,True,True,2000,5)
if clientID != -1:
    print ('Servidor conectado!')

    # inicialização dos motores
    erro, leftMotorHandle = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',sim.simx_opmode_oneshot_wait)
    if erro != 0:
        print ('Handle do motor esquerdo nao encontrado!')
    else:
        print ('Conectado ao motor esquerdo!')

    erro, rightMotorHandle = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',sim.simx_opmode_oneshot_wait)
    if erro != 0:
        print ('Handle do motor direito nao encontrado!')
    else: print('Conectado ao motor direito!')

    #inicialização dos sensores (remoteApi)
    for i in range(16):
        erro, sensorHandle[i] = sim.simxGetObjectHandle(clientID,"Pioneer_p3dx_ultrasonicSensor%d" % (i+1),sim.simx_opmode_oneshot_wait)
        if erro != 0:
            print ("Handle do sensor Pioneer_p3dx_ultrasonicSensor%d nao encontrado!" % (i+1))
        else:
            print ("Conectado ao sensor Pioneer_p3dx_ultrasonicSensor%d!" % (i+1))
            erro, state, coord, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID, sensorHandle[i],sim.simx_opmode_streaming)
    #--------------------------------------------------------------------------------------------------------       
    err, gps = sim.simxGetObjectHandle(clientID, 'GPS_reference', sim.simx_opmode_blocking)
    if err != 0:
        print ('gps nao encontrado!')
        print (err)
    err, objectAbsolutePosition = sim.simxGetObjectPosition(clientID, gps, -1, sim.simx_opmode_streaming)
    #--------------------------------------------------------------------------------------------------------
    err,orientation = sim.simxGetObjectOrientation(clientID, gps, -1, sim.simx_opmode_streaming)
    
    #--------------------------------------------------------------------------------------------------------

 
    #desvio e velocidade do robo
    while sim.simxGetConnectionId(clientID) != -1:
        times = times+1
        print (times)
        objectAbsolutePosition = sim.simxGetObjectPosition(clientID, gps, -1, sim.simx_opmode_buffer)
        orientation = sim.simxGetObjectOrientation(clientID, gps, -1, sim.simx_opmode_buffer)
        

        
        if orientation[1][2] != 0: #no primeiro loop tudo inicia no 0, não é a orientacao correta do robo
            if times > 10: #sincronizacao
                
                # right wall follower
                if isPathRight():
                    axialRotation ('Right', orientation)
                    orientation = sim.simxGetObjectOrientation(clientID, gps, -1, sim.simx_opmode_buffer)
                    fwdStep(objectAbsolutePosition, orientation)
                else:
                        if isPathFwd():
                            fwdStep(objectAbsolutePosition, orientation)
                        else:
                            axialRotation ('Left', orientation)
                    
       
  





    
        

    sim.simxFinish(clientID) # fechando conexao com o servidor
    print ('Conexao fechada!')
else:
    print ('Problemas para conectar o servidor!')
