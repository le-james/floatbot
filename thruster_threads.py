import threading
import time
import numpy as np



# fire time for each thruster = p*(x,y,z) times
# thruster_matrix = np.arrary([[-1, 1, 0, 0, 1, -1, 0, 0], 
#                              [0, 0, 1, -1, 0, 0, -1, 1], 
#                              [-1, 1, -1, 1, -1, 1, -1, 1]])

# p = np.linalg.pinv(thruster_matrix)



def fire_thruster(name, dt):
    print("going to sleep for", dt, "seconds")
    time.sleep(dt)
    print("hello, i am", name)


dt = [2, 5]

thruster_threads_list = []

# start = time.time()
for i in range(2):
    t = threading.Thread(target=fire_thruster, name="thruster{}".format(i), args=("thruster{}".format(i),dt[i]))
    thruster_threads_list.append(t)
    t.start()
    print(t.name, "has started")

for t in thruster_threads_list:
    # locking call/blocking method - prevents the interpreter from going back to the main loop until the current thread finishes
    t.join()
# end = time.time()
# print("time to finish threads", end - start)

print("all threads finished")