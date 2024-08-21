# this contains the names of each state and controls how we pass between each state. 
# made by Bec 20/08/2024

class StateMachine():
    BallDetect = 1
    RotateState = 2
    PControlState = 3
    ReturnHome = 4

    currentState = BallDetect

    def __init__(self, initialState):
        self.currentState = initialState

    def getState(self):
        return self.currentState

    def getStateString(self):
        if(self.currentState == StateMachine.BallDetect):
            return "BallDetect"
        elif(self.currentState == StateMachine.RotateState):
            return "RotateState"
        elif(self.currentState == StateMachine.PControlState):
            return "PControlState"
        elif(self.currentState == StateMachine.ReturnHome):
            return "ReturnHome"
        

    def setState(self, state):
        self.currentState = state

        
# run this only when this file is run
if __name__ == "__main__":
    print("Error: This is a class file")

