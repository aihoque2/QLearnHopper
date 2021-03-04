import random

class QLearn:
    def __init__(self, actions, epsilon, alpha, gamma):
        self.q = {}
        self.actions = actions

        self.alpha = alpha      # learning rate
        self.epsilon = epsilon  # exploration vs exploitation
        self.gamma = gamma      # discount factor
        

    def getQ(self, state, action):
        """Q-val given a state and action
        This is also known as Q(s,a)"""

        return self.q.get((state, action), 0.0) #get the value, else return 0 if it doesn't exist

    def learnQ(self, state, action, reward, Q_local):
        """Update the Q value for (state, action) with 
        the following equation: Q(s,a) = Q(s,a) + alpha*(Q_local - Q(s,a)) 
        where Q_local = (R(s) + gamma * max(Q(s',a))),
        max(Q(s', a)) maxes Q for the action"""

        Q_old = self.q.get((state, action), None)
        if Q_old is None:
            self.q[(state, action)] = reward
        else:
            self.q[(state, action)] = Q_old + self.alpha * (Q_local - Q_old)

    def chooseAction(self, state, return_q=False):
        q = [self.getQ(state, a) for a in self.actions]
        maxQ = max(q)

        if random.random() < self.epsilon:
            minQ = min(q); mag = max(abs(minQ), abs(maxQ))
            # add random values to all the actions, recalculate maxQ
            q = [q[i] + random.random() * mag - .5 * mag for i in range(len(self.actions))] 
            maxQ = max(q)

        count = q.count(maxQ)
        # In case there're several state-action max values 
        # we select a random one among them
        if count > 1:
            best = [i for i in range(len(self.actions)) if q[i] == maxQ]
            i = random.choice(best)
        else:
            i = q.index(maxQ)

        action = self.actions[i]        
        if return_q: # if they want it, give it!
            return action, q
        return action

    def learn(self, state1, action1, reward, state2):
        maxqnew = max([self.getQ(state2, a) for a in self.actions])
        Q_local = reward + (self.gamma*maxqnew)
        self.learnQ(state1, action1, reward, Q_local)