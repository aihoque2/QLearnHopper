import random

class QLearn:
    def __init__(self, actions, alpha, epsilon, gamma):
        self.q = {} #dictionary that stores (state, action) and retunrs Q-value

        self.actions = actions #the set of actions our agent can do

        #hyperparams
        self.alpha = alpha #learning rate
        self.epsilon = epsilon #exploration vs exploitation
        self.gamma = gamma #discount factor


    def getQ(self, state, action): 
        """Q-val given a state and action
        This is also known as Q(s,a)"""
        return self.q.get((state, action), default=0.0)
    
    def learnQ(self, state, action, reward, Q_local):
        """Update the Q value for (state, action) with 
        the following equation: Q(s,a) = Q(s,a) + alpha*(Q_local - Q(s,a)) 
        where Q_local = (R(s) + gamma * max(Q(s',a))),
        max(Q(s', a)) maxes Q for the action"""

        Q_old = self.q.get((state, action), 0.0) #TODO: try to return 0.0 instead of None and see what happens
        if (Q_old == None):
            self.q[(state, action)] = reward
        else:
            self.q[(state, action)] = oldv + self.aplha * (Q_local-oldv)

    def chooseAction(self, state, return_q = False):
        q = [self.getQ(state, a) for a in self.actions]
        maxQ = max(q)

        if (random.random < self.epsilon): #this is to see if we wish to exploit another action 
            minQ = min(q)
            mag = max(abs(minQ), abs(maxQ))
            q = [(q[i] + random.random() * mag - 0.5*mag) for i in range (0, len(actions))]
            maxQ = max(q)
        
        count = q.count(maxQ) #see how many max Q values we ahve

        if (count > 1): #when several actions have the same max Q-val
            best = [i for i in range(len(self.actions)) if (q[i] == maxQ)]
            i = random.choice(best) #index of best action
        
        else:
            i = q.index(maxQ)

        best_action = self.actions[i] 
        if (return_q): #use this to show the best action and q-val
            return best_action, q, state

        return best_action

    def learn(self, state1, action1, reward, state2):
        max_next_q = max([self.getQ(state2, a) for a in self.actions]) #this is max(Q(s',a')) where we max Q with respect to action

        Q_local = reward + (self.gamma * max_next_q)
        self.learnQ(state1, action1, reward, Q_local) #WHEN CALLING MEMBER FUNCTIONS, ALWAYS USE self.funct()



         
