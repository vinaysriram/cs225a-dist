import redis
import numpy as np
import matplotlib.pyplot as plt
term_key = 'cs225a::robot::RRPbot::comm::redis_term_flag'
ctrl_key = 'cs225a::robot::RRPbot::comm::redis_ctrl_flag'
mass_key = 'cs225a::robot::RRPbot::actuator::m11_m22_m33'
r = redis.StrictRedis(host='localhost', port=6379, db=0)
A = np.zeros((0, 7))
while (r.get(term_key) == 'false'):
    r.set(ctrl_key, 'true')
    v = 'true'
    while (v == 'true'):
        v = r.get(ctrl_key)
    A = np.concatenate((A, [np.fromstring(r.get(mass_key), sep=' ')]), axis=0)

print A

f1 = plt.figure(1)
plt.plot(A[:,0], A[:,1])
plt.xlabel('Generalized Joint Coordinate');
plt.ylabel('Mass Matrix M_11 value (kg-m^2)');
plt.title('Mass Matrix Response to Joint Coordinate Sweep');
plt.show()

f2 = plt.figure(2)
plt.plot(A[:,0], A[:,2])
plt.xlabel('Generalized Joint Coordinate');
plt.ylabel('Mass Matrix M_22 value (kg-m^2)');
plt.title('Mass Matrix Response to Joint Coordinate Sweep');
plt.show()

f3 = plt.figure(3)
plt.plot(A[:,0], A[:,3])
plt.xlabel('Generalized Joint Coordinate');
plt.ylabel('Mass Matrix M_33 value (kg)');
plt.title('Mass Matrix Response to Joint Coordinate Sweep');
plt.show()

g1 = plt.figure(4)
plt.plot(A[:,0], A[:,4])
plt.xlabel('Generalized Joint Coordinate');
plt.ylabel('Gravity Vector G_1 value (N-m)');
plt.title('Gravity Vector Response to Joint Coordinate Sweep');
plt.show()

g2 = plt.figure(5)
plt.plot(A[:,0], A[:,5])
plt.xlabel('Generalized Joint Coordinate');
plt.ylabel('Gravity Vector G_2 value (N-m)');
plt.title('Gravity Vector Response to Joint Coordinate Sweep');
plt.show()

g3 = plt.figure(6)
plt.plot(A[:,0], A[:,6])
plt.xlabel('Generalized Joint Coordinate');
plt.ylabel('Gravity Vector G_3 value (N)');
plt.title('Gravity Vector Response to Joint Coordinate Sweep');
plt.show()
