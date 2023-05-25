import rospy
import time
from std_msgs.msg import String
from dynamixel_workbench_msgs.srv import DynamixelCommand
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from numpy import interp
from numpy import pi
import click


# ---------- Constants ----------
posHome=np.array([0.2,0,0.242,0])
gripClose=np.radians(-100)
gripOpen=0
AngHomeGrad=np.array([0,0,-90,0,-100])
AngHomeRad=np.multiply(AngHomeGrad,np.pi/180)

Zpiso=0.140 # default 138
ZpisoInt=0.10
ZpisoEx=0.170
Zportaherramientas=0.145
Zseguridad=0.2
ZseguridadEx=0.24
ZseguridadPH=0.25
beta=0*np.pi/180
betaEx=10*np.pi/180
rmin=0.17
rmax=0.29
lim=70*np.pi/180

PosArcMinIzq=np.array([[rmin*np.cos(lim),rmin*np.sin(lim),Zseguridad,0],[rmin*np.cos(lim),rmin*np.sin(lim),ZpisoInt,0]])
PosArcMinDer=np.array([[rmin*np.cos(-lim),rmin*np.sin(-lim),ZpisoInt,0],[rmin*np.cos(-lim),rmin*np.sin(-lim),Zseguridad,0]])
PosArcMaxDer=np.array([[rmax*np.cos(-lim),rmax*np.sin(-lim),Zseguridad,betaEx],[rmax*np.cos(-lim),rmax*np.sin(-lim),ZpisoEx-0.02,betaEx]])
PosArcMaxIzq=np.array([[rmax*np.cos(lim),rmax*np.sin(lim),ZpisoEx-0.02,betaEx],[rmax*np.cos(lim),rmax*np.sin(lim),Zseguridad+0.015,betaEx]])

PuntosD=np.array([
    [0.200-0.025,0.150,Zseguridad,betaEx],
    [0.200-0.025,0.150,Zpiso-0.020,betaEx],
    [0.250-0.025,0.150,Zpiso-0.020,betaEx],
    [0.250-0.025,0.125,Zpiso-0.020,betaEx],
    [0.225-0.025,0.100,Zpiso-0.020,betaEx],
    [0.200-0.025,0.125,Zpiso-0.020,betaEx],
    [0.200-0.025,0.150,Zpiso-0.020,betaEx],
    [0.200-0.025,0.150,Zseguridad,betaEx]])
PuntosJ=np.array([
    [0.250,0.100,Zseguridad+0.01,betaEx],
    [0.250,0.100,Zpiso+0.01,betaEx],
    [0.250,0.050,Zpiso,betaEx],
    [0.250,0.075,Zpiso,betaEx],
    [0.200,0.075,Zpiso-0.012,betaEx],
    [0.200,0.100,Zpiso-0.012,betaEx],
    [0.200,0.100,Zseguridad,betaEx]])
PuntosL=np.array([
    [0.250,0.050,Zseguridad,betaEx],
    [0.250,0.050,Zpiso-0.020,betaEx],
    [0.200,0.050,Zpiso-0.020,betaEx],
    [0.200,0.025,Zpiso-0.020,betaEx],
    [0.225,0.025,Zseguridad,betaEx]])
PuntosTri=np.array([[0.200,0,Zseguridad,betaEx],[0.200,0,Zpiso-0.02,betaEx],[0.200,0.050,Zpiso-0.017,betaEx],[0.2433,0.025,Zpiso-0.005,betaEx],[0.200,0,Zpiso-0.015,betaEx],[0.200,0,Zseguridad,betaEx]])
PuntosRectasParalelas=np.array([[0.200,-0.050,Zseguridad,betaEx],[0.200,-0.050,Zpiso-0.015,betaEx],[0.200,-0.100,Zpiso-0.015,betaEx],[0.200,-0.100,Zseguridad,betaEx],[0.225,-0.100,Zseguridad,betaEx],[0.225,-0.100,Zpiso,betaEx],[0.225,-0.050,Zpiso-0.005,betaEx],[0.225,-0.050,Zseguridad,betaEx],[0.250,-0.050,Zseguridad,betaEx],[0.250,-0.050,Zpiso,betaEx],[0.250,-0.100,Zpiso,betaEx],[0.250,-0.100,Zseguridad,betaEx]])
PuntosEQTablero=np.array([[0.200,-0.125,Zseguridad,betaEx],[0.200,-0.125,Zpiso+0.005,betaEx],[0.200,-0.125,Zseguridad-0.005,betaEx],[0.210,-0.125,Zseguridad-0.005,betaEx],[0.210,-0.125,Zpiso+0.0085,betaEx],[0.210,-0.125,Zseguridad-0.005,betaEx],[0.220,-0.125,Zseguridad-0.005,betaEx],[0.220,-0.125,Zpiso+0.01,betaEx],[0.220,-0.125,Zseguridad-0.005,betaEx],[0.230,-0.125,Zseguridad-0.005,betaEx],[0.230,-0.125,Zpiso+0.015,betaEx],[0.230,-0.125,Zseguridad-0.005,betaEx],[0.240,-0.125,Zseguridad-0.005,betaEx],[0.240,-0.125,Zpiso+0.02,betaEx],[0.240,-0.125,Zseguridad+0.03,betaEx]])



# ---------- ROS ----------
def jointCommand(command, id_num, addr_name, value, time):
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:        
        dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,id_num,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))

def ActualizarRegistros(P_o):
    for i in range(len(P_o)):
        jointCommand('', (i+1), 'Goal_Position', int(P_o[i]), 0)
        s=i
    print(P_o)

pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=0)
rospy.init_node('joint_publisher', anonymous=False)
state = JointTrajectory() 
state.header.stamp = rospy.Time.now()
state.joint_names = ["joint_1","joint_2","joint_3","joint_4","joint_4"]

def joint_publisher(q,t):
    state = JointTrajectory()
    state.header.stamp = rospy.Time.now()
    state.joint_names = ["joint_1","joint_2","joint_3","joint_4","joint_5"]
    point = JointTrajectoryPoint()
    point.positions = q 
    point.time_from_start = rospy.Duration(t)
    state.points.append(point)
    pub.publish(state)
    # print('Cambio de punto q:')  

    fkine(q[0],q[1],q[2],q[3],q[4])
    time.sleep(3*t)


#---------- Inverse Kinematics ----------
def theta_1(P_x,P_y):
    return np.arctan2(P_y,P_x)

def theta_2(P_x,P_y,P_z,beta,L_1,L_2,L_3,L_4):
    R=np.sqrt(P_x**2+P_y**2)
    D_r=R-L_4*np.cos(beta)
    D_z=P_z-L_4*np.sin(beta)-L_1
    return -2*np.arctan((2*D_r*L_2 - np.sqrt(- D_r**4 - 2*D_r**2*D_z**2 + 2*D_r**2*L_2**2 + 2*D_r**2*L_3**2 - D_z**4 + 2*D_z**2*L_2**2 + 2*D_z**2*L_3**2 - L_2**4 + 2*L_2**2*L_3**2 - L_3**4))/(D_r**2 + D_z**2 + 2*D_z*L_2 + L_2**2 - L_3**2))

def theta_3(P_x,P_y,P_z,beta,L_1,L_2,L_3,L_4):
    R=np.sqrt(P_x**2+P_y**2)
    D_r=R-L_4*np.cos(beta)
    D_z=P_z-L_4*np.sin(beta)-L_1
    return -2*np.arctan((np.sqrt((- D_r**2 - D_z**2 + L_2**2 + 2*L_2*L_3 + L_3**2)*(D_r**2 + D_z**2 - L_2**2 + 2*L_2*L_3 - L_3**2)) - 2*L_2*L_3)/(D_r**2 + D_z**2 - L_2**2 - L_3**2))

def theta_4(beta,theta_2,theta_3):
    return beta-theta_2-theta_3

def ikine4r(P_x,P_y,P_z,beta):
    L_1=0.137
    L_2=0.105
    L_3=0.105
    L_4=0.095
    T1=theta_1(P_x,P_y)
    T2=theta_2(P_x,P_y,P_z,beta,L_1,L_2,L_3,L_4)
    T3=theta_3(P_x,P_y,P_z,beta,L_1,L_2,L_3,L_4)
    T4=theta_4(beta,T2,T3)
    return np.array([round(T1,3),round(T2,3),round(T3,3),round(T4,3)])


#---------- Forward Kinematics ----------
def fkine(theta_1,theta_2,theta_3,theta_4,theta_5):
    L_1=0.137
    L_2=0.105
    L_3=0.105
    L_4=0.095
    beta=np.arccos(np.cos(theta_2 + theta_3 + theta_4)*np.cos(theta_1))
    P_x=np.cos(theta_1)*(L_3*np.cos(theta_2 + theta_3) - L_2*np.sin(theta_2) + L_4*np.cos(theta_2 + theta_3 + theta_4))
    P_y=np.sin(theta_1)*(L_3*np.cos(theta_2 + theta_3) - L_2*np.sin(theta_2) + L_4*np.cos(theta_2 + theta_3 + theta_4))
    P_z=L_1 + L_3*np.sin(theta_2 + theta_3) + L_2*np.cos(theta_2) + L_4*np.sin(theta_2 + theta_3 + theta_4)
    if theta_5<-40*np.pi/180:
        Grip=1
    else:
        Grip=0

    # print("Coordenadas:")
    # print('Pos X: '+"%.2f" % P_x+'°\tPos Y:'+"%.2f" % P_y+'°\tPos Z:'+"%.2f" % P_z+'°\tBeta:'+"%.2f" % beta)

    # if Grip==1:
    #     print("\nEl gripper está cerrado\n")
    # else:
    #     print("\nEl gripper está abierto\n")

    #return np.array([P_x,P_y,P_z,beta,Grip])


# ----------     ----------
#Función que permite realizar la trayectoria a un punto Pos en un tiempo t. Los sleep se realizan para no saturar el bus de comunicaciones
def Rut2p(Pos,t,tool):
    q=np.concatenate(((ikine4r(Pos[0],Pos[1],Pos[2],Pos[3])+AngHomeRad[0:4]),np.array([tool])), axis=0)
    time.sleep(2*t)
    joint_publisher(q,t)
    time.sleep(2*t)

def ConvPosArr2QArr(PosArr):
    [r,c]=PosArr.shape
    QArr=np.zeros([r,c])
    for i in range(r):
        QArr[i,:]=ikine4r(PosArr[i,0],PosArr[i,1],PosArr[i,2],PosArr[i,3])    
    return QArr

def mapAn(theta):
    return int(interp(theta,[-150, 150],[0, 1023]))

# ---------- CLI ----------
@click.group()
def cli():
    pass

@cli.command()
def gripper_open():
    jointCommand('', 5, 'Goal_Position', 260, 0)

@cli.command()
def gripper_close():
    jointCommand('', 5, 'Goal_Position', 160, 0)

@cli.command()
def home():
    joint_publisher(AngHomeRad, 1)

@cli.command()
def draw_workspace():
    start_time = time.time()

    #Se genera un arreglo de puntos para toda la rutina de espacio de trabajo
    #También un arreglo de tiempos para cada movimiento en corcondancia con la distancia a recorrer.
    ArrPos=[PosArcMinIzq[0,:],PosArcMinIzq[1,:],PosArcMinDer[0,:],PosArcMinDer[1,:],PosArcMaxDer[0,:],PosArcMaxDer[1,:],PosArcMaxIzq[0,:],PosArcMaxIzq[1,:],PosArcMinIzq[0,:],posHome]
    ArrTiempo=[1.5, 0.5, 0.5, 0.5, 1,2, 1.5, 2, 0.5, 1]
    for i in range(len(ArrPos)):
        Rut2p(ArrPos[i],ArrTiempo[i],gripClose)
    #Finaliza volviendo al home
    Rut2p(posHome,1.5,gripClose)

    end_time = time.time()
    Tiempo=end_time-start_time
    print("\ntiempo de ejecucion: %.2f s" % Tiempo)

@cli.command()
def draw_figures():
    start_time = time.time()

    #CIRCULO
    #Se realizan 50 puntos para el círculo, aunque este parámetro se puede variar
    N=50
    Rads=np.linspace(0,2*np.pi,N)
    ZPuntos=np.ones(N)*(Zpiso)
    BetaPuntos=np.ones(N)*betaEx
    t=0.2
    ArrTiempo=np.ones(N)*t
    PuntosCirc=np.zeros([N,4])
    PuntosCirc[:,0]=0.225+0.025*np.cos(Rads)
    PuntosCirc[:,1]=-0.025+0.025*np.sin(Rads)
    PuntosCirc[:,2]=ZPuntos-0.01+0.005*np.cos(Rads)
    PuntosCirc[:,3]=BetaPuntos

    #Se establece una posición de inicio antes de empezar el círculo
    PosIni=[0.25,0,ZseguridadEx,betaEx]

    #Se dirige a la posición de inicio
    Rut2p(PosIni,1.5,gripClose)
    
    #Rutína de movimiento del círculo
    for i in range(N):
        Rut2p(PuntosCirc[i,:],ArrTiempo[i],gripClose)
    
    #Vuelve a Home
    Rut2p(posHome,2,gripClose)


    #RECTAS PARALELAS
    ArrTiempo=[1,1,1,1,1,1,1,1,1,1,1,1]
    for i in range(len(ArrTiempo)):
        Rut2p(PuntosRectasParalelas[i,:],ArrTiempo[i],gripClose)
    Rut2p(posHome,1,gripClose)

    end_time = time.time()  
    Tiempo=end_time-start_time
    print("\ntiempo de ejecucion: %.2f s" % Tiempo)

@cli.command()
def draw_letters():
    start_time = time.time()
    ArrTiempo=[1.5,1.5,1.5,0.5,0.5,0.5,1,1,1.5,0.5,0.5,0.5,0.5,1,1]
    for i in range(8):
        Rut2p(PuntosD[i,:],ArrTiempo[i],gripClose)
    time.sleep(1)
    for i in range(7):
        Rut2p(PuntosJ[i,:],ArrTiempo[i],gripClose)
    Rut2p(posHome,1.5,gripClose)
    time.sleep(1)
    for i in range(5):
        Rut2p(PuntosL[i,:],ArrTiempo[i],gripClose)

    end_time = time.time()
    Tiempo=end_time-start_time
    print("\ntiempo de ejecucion: %.2f s" % Tiempo)


# ---------- main ----------
if __name__ == '__main__':
    #Definir los límites de torques (se dejan en el máximo)
    Torques=[1023,1023,1023,1023,1023]
    for i in range(5):    
        jointCommand('', (i+1), 'Torque_Limit', Torques[i], 0)

    cli()
