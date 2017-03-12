import numpy as np
import scipy as sp

class PFSimple():
    def __init__(self,State,p_num = 1000):
        # self.particle = State.reshape(-1) + np.random.normal(0.0,0.5,[State.shape[0],p_num])
        self.particle = np.zeros([p_num,State.shape[0]])
        tmp_rnd = np.random.normal(0.0,0.05,[p_num,State.shape[0]])
        for i in range(self.particle.shape[0]):
            self.particle[i,:] = State+tmp_rnd[i,:]
        self.score = np.ones(p_num)


    def Process(self,State):

        # state transition
        self.particle+=np.random.normal(0.0,0.05,self.particle.shape)

        #evaluation
        for i in range(self.particle.shape[0]):
            self.score *= (0.0000001 +
                           np.linalg.norm(State-self.particle[i,:]))

        #Get Result
        self.score = self.score / np.sum(self.score)

        # res = np.mean(self.score.reshape(-1)*self.particle,0)
        res = np.zeros_like(State)
        for i in range(self.particle.shape[0]):
            res += self.particle[i,:] * self.score[i]

        # resample

        tmp_particl = self.particle
        tmp_score = self.score

        for i in range(tmp_particl.shape[0]):
            seed = np.random.uniform(0,1.0)

            index = 0

            while seed > 0:
                seed -= tmp_score[index]
                index+=1

            if index >= self.particle.shape[0]:
                index = self.particle.shape[0]-1

            self.particle[i,:] = tmp_particl[index]
            self.score[i] = tmp_score[index]


            self.score /= np.sum(self.score)

        return res
