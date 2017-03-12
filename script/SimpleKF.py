import numpy as np
import scipy as sp


class KFSimple():
    def __init__(self, State):
        self.X = State
        self.P = np.zeros([State.shape[0], State.shape[0]])
        self.P /= 3.0

        self.K = np.ones_like(self.P)
        self.I = np.zeros_like(self.P)
        for i in range(self.I.shape[0]):
            self.I[i,i] = 1.0


    def Process(self, State):
        # x =
        # P = P
        K = self.P.dot(np.linalg.inv(self.P+self.I/10.0))
        self.X = self.X+\
                 (self.K.dot(State-self.X))
        self.P = (self.I-self.K).dot(self.P)

