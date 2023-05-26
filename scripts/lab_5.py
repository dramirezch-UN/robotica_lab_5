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
HOME=np.array([0.2,0,0.242,0])
CLOSED_GRIPPER_ANG=np.radians(-100)
OPEN_GRIPPER_ANG=0
HOME_DEGREES=np.array([0,0,-90,0,-100])
HOME_RADIANS=np.multiply(HOME_DEGREES,np.pi/180)

z_floor=0.140
z_floor_in=0.10
z_floor_out=0.170
z_tool=0.215
z_safety=0.2
z_safety_out=0.24
z_safety_ph=0.4
beta=0*np.pi/180
betaEx=10*np.pi/180
rmin=0.17
rmax=0.29
lim=70*np.pi/180

PosArcMinIzq=np.array([[rmin*np.cos(lim),rmin*np.sin(lim),z_safety,0],[rmin*np.cos(lim),rmin*np.sin(lim),z_floor_in,0]])
PosArcMinDer=np.array([[rmin*np.cos(-lim),rmin*np.sin(-lim),z_floor_in,0],[rmin*np.cos(-lim),rmin*np.sin(-lim),z_safety,0]])
PosArcMaxDer=np.array([[rmax*np.cos(-lim),rmax*np.sin(-lim),z_safety,betaEx],[rmax*np.cos(-lim),rmax*np.sin(-lim),z_floor_out-0.02,betaEx]])
PosArcMaxIzq=np.array([[rmax*np.cos(lim),rmax*np.sin(lim),z_floor_out-0.02,betaEx],[rmax*np.cos(lim),rmax*np.sin(lim),z_safety+0.015,betaEx]])

PuntosD=np.array([
    [0.200-0.025,0.150,z_safety,betaEx],
    [0.200-0.025,0.150,z_floor-0.020,betaEx],
    [0.250-0.025,0.150,z_floor-0.020,betaEx],
    [0.250-0.025,0.125,z_floor-0.020,betaEx],
    [0.225-0.025,0.100,z_floor-0.020,betaEx],
    [0.200-0.025,0.125,z_floor-0.020,betaEx],
    [0.200-0.025,0.150,z_floor-0.020,betaEx],
    [0.200-0.025,0.150,z_safety,betaEx]])
PuntosJ=np.array([
    [0.250,0.100,z_safety+0.01,betaEx],
    [0.250,0.100,z_floor+0.01,betaEx],
    [0.250,0.050,z_floor,betaEx],
    [0.250,0.075,z_floor,betaEx],
    [0.200,0.075,z_floor-0.012,betaEx],
    [0.200,0.100,z_floor-0.012,betaEx],
    [0.200,0.100,z_safety,betaEx]])
PuntosL=np.array([
    [0.250,0.050,z_safety,betaEx],
    [0.250,0.050,z_floor-0.020,betaEx],
    [0.200,0.050,z_floor-0.020,betaEx],
    [0.200,0.025,z_floor-0.020,betaEx],
    [0.225,0.025,z_safety,betaEx]])

PuntosTri=np.array([[0.200,0,z_safety,betaEx],[0.200,0,z_floor-0.02,betaEx],[0.200,0.050,z_floor-0.017,betaEx],[0.2433,0.025,z_floor-0.005,betaEx],[0.200,0,z_floor-0.015,betaEx],[0.200,0,z_safety,betaEx]])
PuntosRectasParalelas=np.array([[0.200,-0.050,z_safety,betaEx],[0.200,-0.050,z_floor-0.015,betaEx],[0.200,-0.100,z_floor-0.015,betaEx],[0.200,-0.100,z_safety,betaEx],[0.225,-0.100,z_safety,betaEx],[0.225,-0.100,z_floor,betaEx],[0.225,-0.050,z_floor-0.005,betaEx],[0.225,-0.050,z_safety,betaEx],[0.250,-0.050,z_safety,betaEx],[0.250,-0.050,z_floor,betaEx],[0.250,-0.100,z_floor,betaEx],[0.250,-0.100,z_safety,betaEx]])
PuntosEQTablero=np.array([[0.200,-0.125,z_safety,betaEx],[0.200,-0.125,z_floor+0.005,betaEx],[0.200,-0.125,z_safety-0.005,betaEx],[0.210,-0.125,z_safety-0.005,betaEx],[0.210,-0.125,z_floor+0.0085,betaEx],[0.210,-0.125,z_safety-0.005,betaEx],[0.220,-0.125,z_safety-0.005,betaEx],[0.220,-0.125,z_floor+0.01,betaEx],[0.220,-0.125,z_safety-0.005,betaEx],[0.230,-0.125,z_safety-0.005,betaEx],[0.230,-0.125,z_floor+0.015,betaEx],[0.230,-0.125,z_safety-0.005,betaEx],[0.240,-0.125,z_safety-0.005,betaEx],[0.240,-0.125,z_floor+0.02,betaEx],[0.240,-0.125,z_safety+0.03,betaEx]])

coorPortaHerramientasArriba=np.array([0.1,-0.2,z_safety_ph,betaEx])
coorPortaHerramientasAbajo=np.array([0.12,-0.21,z_tool,0])
coorPortaHerramientasAbajoDes=np.array([0.1,-0.185,z_tool,betaEx])


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

# ---------- Helpers    ----------
def Rut2p(Pos,t,tool):
    q=np.concatenate(((ikine4r(Pos[0],Pos[1],Pos[2],Pos[3])+HOME_RADIANS[0:4]),np.array([tool])), axis=0)
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
def HOME():
    joint_publisher(HOME_RADIANS, 1)

@cli.command()
def draw_workspace():
    start_time = time.time()

    ArrPos=[PosArcMinIzq[0,:],PosArcMinIzq[1,:],PosArcMinDer[0,:],PosArcMinDer[1,:],PosArcMaxDer[0,:],PosArcMaxDer[1,:],PosArcMaxIzq[0,:],PosArcMaxIzq[1,:],PosArcMinIzq[0,:],HOME]
    ArrTiempo=[1.5, 0.5, 0.5, 0.5, 1,2, 1.5, 2, 0.5, 1]
    for i in range(len(ArrPos)):
        Rut2p(ArrPos[i],ArrTiempo[i],CLOSED_GRIPPER_ANG)
    Rut2p(HOME,1.5,CLOSED_GRIPPER_ANG)

    end_time = time.time()
    Tiempo=end_time-start_time
    print("\ntiempo de ejecucion: %.2f s" % Tiempo)

@cli.command()
def draw_figures():
    start_time = time.time()

    N=50
    Rads=np.linspace(0,2*np.pi,N)
    ZPuntos=np.ones(N)*z_floor
    BetaPuntos=np.ones(N)*betaEx
    t=0.2
    ArrTiempo=np.ones(N)*t
    PuntosCirc=np.zeros([N,4])
    PuntosCirc[:,0]=0.225+0.025*np.cos(Rads)
    PuntosCirc[:,1]=-0.025+0.025*np.sin(Rads)
    PuntosCirc[:,2]=ZPuntos-0.01+0.005*np.cos(Rads)
    PuntosCirc[:,3]=BetaPuntos
    PosIni=[0.25,0,z_safety_out,betaEx]
    Rut2p(PosIni,1.5,CLOSED_GRIPPER_ANG)
    for i in range(N):
        Rut2p(PuntosCirc[i,:],ArrTiempo[i],CLOSED_GRIPPER_ANG)
        
    Rut2p(HOME,2,CLOSED_GRIPPER_ANG)

    ArrTiempo=[1,1,1,1,1,1,1,1,1,1,1,1]
    for i in range(len(ArrTiempo)):
        Rut2p(PuntosRectasParalelas[i,:],ArrTiempo[i],CLOSED_GRIPPER_ANG)
    Rut2p(HOME,1,CLOSED_GRIPPER_ANG)

    end_time = time.time()  
    Tiempo=end_time-start_time
    print("\ntiempo de ejecucion: %.2f s" % Tiempo)

@cli.command()
def draw_letters():
    start_time = time.time()
    ArrTiempo=[1.5,1.5,1.5,0.5,0.5,0.5,1,1,1.5,0.5,0.5,0.5,0.5,1,1]
    for i in range(8):
        Rut2p(PuntosD[i,:],ArrTiempo[i],CLOSED_GRIPPER_ANG)
    time.sleep(1)
    for i in range(7):
        Rut2p(PuntosJ[i,:],ArrTiempo[i],CLOSED_GRIPPER_ANG)
    Rut2p(HOME,1.5,CLOSED_GRIPPER_ANG)
    time.sleep(1)
    for i in range(5):
        Rut2p(PuntosL[i,:],ArrTiempo[i],CLOSED_GRIPPER_ANG)

    end_time = time.time()
    Tiempo=end_time-start_time
    print("\ntiempo de ejecucion: %.2f s" % Tiempo)

@cli.command()
def tool_load():
    start_time = time.time()
    Rut2p(coorPortaHerramientasArriba,0.3,OPEN_GRIPPER_ANG)
    Rut2p(coorPortaHerramientasAbajo,0.5,OPEN_GRIPPER_ANG)
    input()
    print("Apretar gripper")
    Rut2p(coorPortaHerramientasAbajo,0.5,CLOSED_GRIPPER_ANG)
    print("Volviendo a home")
    Rut2p(coorPortaHerramientasArriba,1,CLOSED_GRIPPER_ANG)
    Rut2p(HOME,1,CLOSED_GRIPPER_ANG)
    end_time = time.time()
    Tiempo=end_time-start_time
    print("\ntiempo de ejecucion: %.2f s" % Tiempo)

@cli.command()
def tool_unload():
    start_time = time.time()
    Rut2p(coorPortaHerramientasArriba,0.3,CLOSED_GRIPPER_ANG)
    Rut2p(coorPortaHerramientasAbajoDes,0.5,CLOSED_GRIPPER_ANG)
    input()
    print("Apretar gripper")
    Rut2p(coorPortaHerramientasAbajoDes,0.5,OPEN_GRIPPER_ANG)
    print("Volviendo a home")
    Rut2p(coorPortaHerramientasArriba,1,OPEN_GRIPPER_ANG)
    Rut2p(HOME,1,OPEN_GRIPPER_ANG)
    end_time = time.time()
    Tiempo=end_time-start_time
    print("\ntiempo de ejecucion: %.2f s" % Tiempo)


# ---------- main ----------
if __name__ == '__main__':
    Torques=[1023,1023,1023,1023,1023]
    for i in range(5):    
        jointCommand('', (i+1), 'Torque_Limit', Torques[i], 0)

    cli()
