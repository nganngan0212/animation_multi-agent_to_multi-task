
class Task:
    """
    Construct the task with parameters:
        id
        position (x, y, theta = 0)
        req number of robots need to perform task
        is_here: the list of robots id is currently at the task
        is_done: the task is assigned or not
    """
    def __init__(self, id, position, req):

        self.id = id 
        self.position = position
        self.req = req  
        self.is_here = []
        self.is_done = False

    def set_task_position(self, pos):
        self.position.x = pos[0]
        self.position.y = pos[1]

    def new_robot_comes(self, robot_id):
        self.is_here.append(robot_id)

        if len(self.is_here) == self.req:
            self.is_done = True
            print(self.is_done)
            print("Task %.0f is completed. \n" % (self.id))

    def robot_leave(self, robot_id):

        self.is_here.remove(robot_id)
