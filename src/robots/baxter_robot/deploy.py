class Deploy(object):
    def __init__(self, log_file):
        self.traj = load(log_file)