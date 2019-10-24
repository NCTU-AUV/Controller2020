import numpy as np
from numpy import pi,sin,cos
class AUV():
	def __init__(self):
		self.mass = 27#kg
		self.buoyancy =157#N
		self.buoyancy_center = np.array([[0.],[0.],[1.]])
		# motor 1. [2x3] face and position
		self.motor = ["","","","","","","",""]
		#						Vector 					Position
		self.motor[0]=np.array([[0,0,1.],				[0.3,0,0]])
		self.motor[1]=np.array([[0,0,1.],				[-0.3,0,0]])
		self.motor[2]=np.array([[0,0,1.],				[0,0.3,0]])
		self.motor[3]=np.array([[0,0,1.],				[0,-0.3,0]])
		self.motor[4]=np.array([[2**0.5,-2**0.5, 0.],	[0.2,0.1,0]])
		self.motor[5]=np.array([[-2**0.5,-2**0.5, 0.],	[-0.2,0.1,0]])
		self.motor[6]=np.array([[-2**0.5,2**0.5, 0.],	[-0.2,-0.1,0]])
		self.motor[7]=np.array([[2**0.5,2**0.5, 0.],		[0.2,-0.1,0]])
		self.roll=0.
		self.pitch=0.
		self.yaw=0.
		#Thrust Mapping
		self.Trust = np.empty(shape=(0,6))
		for i in self.motor:
			self.Trust = np.vstack((self.Trust,np.append(i[0],np.cross(i[0],i[1]))))
		self.buoyancy_center=np.array([0,0,-0.2])
	def Eular_update(roll,pitch,yaw):
		self.roll=roll
		self.pitch=pitch
		self.yaw=yaw
	def gravity(self):
		return 9.790
	def buoyancy_effect(self):
		g=self.gravity()
		#                   
		return(np.array([	[(self.buoyancy-self.mass*g)*sin(self.pitch*pi/180.)],
							[-(self.buoyancy-self.mass*g)*cos(self.pitch*pi/180.)*sin(self.roll*pi/180.)],
							[-(self.buoyancy-self.mass*g)*cos(self.pitch*pi/180.)*cos(self.roll*pi/180.)],
							[self.buoyancy*cos(self.roll)*(self.buoyancy_center[2]*sin(self.roll*pi/180.)-self.buoyancy_center[1]*cos(self.pitch*pi/180.))],
							[self.buoyancy*(self.buoyancy_center[0]*cos(self.roll*pi/180)*cos(self.pitch*pi/180.)+self.buoyancy_center[2]*sin(self.pitch*pi/180.))],
							[-self.buoyancy*(self.buoyancy_center[0]*sin(self.roll*pi/180)*cos(self.pitch*pi/180.)+self.buoyancy_center[1]*sin(self.pitch*pi/180.))],
				]))

		'''
 		return (np.array([(self.buoyancy-self.mass*g)*sin(self.pitch*pi/180.),
 								-(self.buoyancy-self.mass*g)*sin(self.roll*pi/180.)*cos(self.pitch*pi/180)
 							])
 				)
 		'''
Po=AUV()
#print(Po.Trust)
print(Po.buoyancy_effect())